mod base_station;
mod commands;
mod firasim;
mod grsim;
mod transport;

pub use base_station::{BaseStationTransport, TeamColor};
pub use firasim::FIRASimClient;
pub use grsim::GrSimClient;
pub use transport::{FiraSimTransport, GrSimTransport, RobotTransport, TransportError};

use crate::motion::RobotCommand;
use std::collections::HashMap;

/// Identifica el destino de los comandos para logs / configuración.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RadioTarget {
    FiraSim,
    GrSim,
    BaseStation,
}

impl RadioTarget {
    /// Lee `VSSL_RADIO_TARGET`. Default: `firasim`.
    pub fn from_env() -> Self {
        match std::env::var("VSSL_RADIO_TARGET")
            .unwrap_or_default()
            .to_ascii_lowercase()
            .as_str()
        {
            "" | "firasim" => RadioTarget::FiraSim,
            "grsim" => RadioTarget::GrSim,
            "basestation" | "real" | "robots" => RadioTarget::BaseStation,
            other => {
                eprintln!(
                    "[Radio] VSSL_RADIO_TARGET='{other}' inválido, usando 'firasim' por defecto"
                );
                RadioTarget::FiraSim
            }
        }
    }
}

pub struct Radio {
    transport: Box<dyn RobotTransport>,
    command_map: HashMap<(i32, i32), RobotCommand>,
}

impl Radio {
    /// Crea un Radio sobre un transport ya construido.
    pub fn new(transport: Box<dyn RobotTransport>) -> Self {
        Self {
            transport,
            command_map: HashMap::new(),
        }
    }

    /// Construye el transport correspondiente a `VSSL_RADIO_TARGET` y lo envuelve en `Radio`.
    /// Para `BaseStation` lee también `VSSL_TEAM_COLOR`, `VSSL_BASESTATION_DEVICE`, `VSSL_BASESTATION_BAUD`.
    pub async fn from_env() -> Result<Self, TransportError> {
        let target = RadioTarget::from_env();
        eprintln!("[Radio] target = {target:?}");
        let transport: Box<dyn RobotTransport> = match target {
            RadioTarget::FiraSim => {
                Box::new(FiraSimTransport::new("127.0.0.1", 20011).await?)
            }
            RadioTarget::GrSim => Box::new(GrSimTransport::new("127.0.0.1", 20011).await?),
            RadioTarget::BaseStation => {
                let team = TeamColor::from_env();
                Box::new(BaseStationTransport::from_env(team)?)
            }
        };
        Ok(Self::new(transport))
    }

    pub fn add_motion_command(&mut self, cmd: crate::motion::MotionCommand) {
        let key = (cmd.team, cmd.id);
        let entry = self.command_map.entry(key).or_insert_with(|| RobotCommand {
            id: cmd.id,
            team: cmd.team,
            motion: cmd.clone(),
            kicker: crate::motion::KickerCommand {
                id: cmd.id,
                team: cmd.team,
                kick_x: false,
                kick_z: false,
                dribbler: 0.0,
            },
        });
        entry.motion = cmd;
    }

    pub fn add_kicker_command(&mut self, cmd: crate::motion::KickerCommand) {
        let key = (cmd.team, cmd.id);
        let entry = self.command_map.entry(key).or_insert_with(|| RobotCommand {
            id: cmd.id,
            team: cmd.team,
            motion: crate::motion::MotionCommand {
                id: cmd.id,
                team: cmd.team,
                vx: 0.0,
                vy: 0.0,
                omega: 0.0,
                orientation: 0.0,
            },
            kicker: cmd.clone(),
        });
        entry.kicker = cmd;
    }

    pub async fn create_robots(&mut self, robot_ids: &[(u32, u32)]) -> Result<(), TransportError> {
        self.transport.create_robots(robot_ids).await
    }

    pub async fn send_commands(&mut self) -> Result<(), TransportError> {
        if self.command_map.is_empty() {
            return Ok(());
        }

        let all_commands: Vec<RobotCommand> = self.command_map.values().cloned().collect();
        let result = self.transport.send_commands(&all_commands).await;
        self.command_map.clear();
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::{KickerCommand, MotionCommand};
    use async_trait::async_trait;
    use std::sync::{Arc, Mutex};

    struct MockTransport {
        sent: Arc<Mutex<Vec<Vec<RobotCommand>>>>,
    }

    #[async_trait]
    impl RobotTransport for MockTransport {
        async fn send_commands(
            &mut self,
            commands: &[RobotCommand],
        ) -> Result<(), TransportError> {
            self.sent.lock().unwrap().push(commands.to_vec());
            Ok(())
        }
    }

    fn test_radio() -> (Radio, Arc<Mutex<Vec<Vec<RobotCommand>>>>) {
        let sent = Arc::new(Mutex::new(Vec::new()));
        let transport = Box::new(MockTransport { sent: sent.clone() });
        (Radio::new(transport), sent)
    }

    #[test]
    fn radio_add_motion_command() {
        let (mut radio, _) = test_radio();
        radio.add_motion_command(MotionCommand {
            id: 0,
            team: 0,
            vx: 1.0,
            vy: 0.0,
            omega: 0.0,
            orientation: 0.0,
        });
        assert_eq!(radio.command_map.len(), 1);
    }

    #[test]
    fn radio_add_kicker_command() {
        let (mut radio, _) = test_radio();
        radio.add_kicker_command(KickerCommand {
            id: 0,
            team: 0,
            kick_x: true,
            kick_z: false,
            dribbler: 5.0,
        });
        assert_eq!(radio.command_map.len(), 1);
    }

    #[test]
    fn radio_keeps_same_id_for_both_teams() {
        let (mut radio, _) = test_radio();
        radio.add_motion_command(MotionCommand {
            id: 0,
            team: 0,
            vx: 1.0,
            vy: 0.0,
            omega: 0.0,
            orientation: 0.0,
        });
        radio.add_motion_command(MotionCommand {
            id: 0,
            team: 1,
            vx: -1.0,
            vy: 0.0,
            omega: 0.0,
            orientation: 0.0,
        });
        assert_eq!(radio.command_map.len(), 2);
    }

    #[tokio::test]
    async fn radio_dispatches_and_clears() {
        let (mut radio, sent) = test_radio();
        radio.add_motion_command(MotionCommand {
            id: 0,
            team: 0,
            vx: 1.0,
            vy: 0.0,
            omega: 0.0,
            orientation: 0.0,
        });
        radio.send_commands().await.unwrap();
        assert_eq!(radio.command_map.len(), 0);
        let snapshots = sent.lock().unwrap();
        assert_eq!(snapshots.len(), 1);
        assert_eq!(snapshots[0].len(), 1);
    }

    #[test]
    fn radio_target_from_env_defaults_to_firasim() {
        // Sin env var: comportamiento default. (No tocamos env aquí para no afectar otros tests).
        // La validación funcional ocurre en build/CI; este test sólo cubre la rama default.
        // SAFETY: variable de entorno aislada por test (single-threaded asserts).
        let prev = std::env::var("VSSL_RADIO_TARGET").ok();
        unsafe {
            std::env::remove_var("VSSL_RADIO_TARGET");
        }
        assert_eq!(RadioTarget::from_env(), RadioTarget::FiraSim);
        if let Some(v) = prev {
            unsafe {
                std::env::set_var("VSSL_RADIO_TARGET", v);
            }
        }
    }
}
