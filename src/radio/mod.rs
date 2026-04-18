mod commands;
mod firasim;
mod grsim;

pub use firasim::FIRASimClient;
pub use grsim::GrSimClient;

use crate::motion::RobotCommand;
use std::collections::HashMap;

/// Tipo de simulador soportado
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SimulatorType {
    /// grSim - Simulador SSL tradicional
    GrSim,
    /// FIRASim - Simulador VSS/SSL-Simulation
    FIRASim,
}

/// Módulo principal de comunicación con robots
pub struct Radio {
    simulator_type: SimulatorType,
    grsim_client: Option<GrSimClient>,
    firasim_client: Option<FIRASimClient>,
    command_map: HashMap<(i32, i32), RobotCommand>, // ((team, id), command)
}

impl Radio {
    /// Crea un nuevo Radio
    ///
    /// # Argumentos
    /// * `use_radio` - Si es true, usa comunicación serial (no implementado aún)
    /// * `simulator_type` - Tipo de simulador a usar (GrSim o FIRASim)
    /// * `address` - Dirección IP del simulador
    /// * `port` - Puerto del simulador
    pub async fn new(
        use_radio: bool,
        simulator_type: SimulatorType,
        address: &str,
        port: u16,
    ) -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        let grsim_client = if !use_radio && simulator_type == SimulatorType::GrSim {
            Some(GrSimClient::new(address, port).await?)
        } else {
            None
        };

        let firasim_client = if !use_radio && simulator_type == SimulatorType::FIRASim {
            Some(FIRASimClient::new(address, port).await?)
        } else {
            None
        };

        Ok(Self {
            simulator_type,
            grsim_client,
            firasim_client,
            command_map: HashMap::new(),
        })
    }

    /// Agrega un comando de movimiento
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

    /// Agrega un comando de kicker
    pub fn add_kicker_command(&mut self, cmd: crate::motion::KickerCommand) {
        let key = (cmd.team, cmd.id);
        let entry = self.command_map.entry(key).or_insert_with(|| {
            RobotCommand {
                id: cmd.id,
                team: cmd.team,
                motion: crate::motion::MotionCommand {
                    id: cmd.id,
                    team: cmd.team,
                    vx: 0.0,
                    vy: 0.0,
                    omega: 0.0,
                    orientation: 0.0, // Valor por defecto, se actualizará cuando se agregue el comando de movimiento
                },
                kicker: cmd.clone(),
            }
        });
        entry.kicker = cmd;
    }

    /// Crea robots en FIRASim antes de enviar comandos
    ///
    /// # Argumentos
    /// * `robot_ids` - Vector de tuplas (id, team) donde team=0 es azul, team=1 es amarillo
    pub async fn create_robots(
        &self,
        robot_ids: &[(u32, u32)],
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        match self.simulator_type {
            SimulatorType::FIRASim => {
                if let Some(ref client) = self.firasim_client {
                    client.create_robots(robot_ids).await
                } else {
                    Err("FIRASim client no está inicializado".into())
                }
            }
            SimulatorType::GrSim => {
                // grSim no necesita crear robots explícitamente
                eprintln!("[Radio] GrSim no requiere creación explícita de robots");
                Ok(())
            }
        }
    }

    /// Envía todos los comandos acumulados
    ///
    /// Para grSim: agrupa comandos por equipo (requiere un mensaje por equipo)
    /// Para FIRASim: envía todos los comandos en un solo mensaje RobotControl
    pub async fn send_commands(
        &mut self,
    ) -> Result<
        Option<crate::protos::ssl_simulation_synchronous::SimulationSyncResponse>,
        Box<dyn std::error::Error + Send + Sync>,
    > {
        if self.command_map.is_empty() {
            eprintln!("[Radio] send_commands() llamado pero command_map está vacío");
            return Ok(None);
        }

        eprintln!(
            "[Radio] Enviando {} comandos (simulator_type: {:?})",
            self.command_map.len(),
            self.simulator_type
        );

        match self.simulator_type {
            SimulatorType::GrSim => {
                // grSim requiere un mensaje por equipo
                let mut blue_commands: Vec<RobotCommand> = Vec::new();
                let mut yellow_commands: Vec<RobotCommand> = Vec::new();

                for cmd in self.command_map.values() {
                    if cmd.team == 0 {
                        blue_commands.push(cmd.clone());
                    } else {
                        yellow_commands.push(cmd.clone());
                    }
                }

                if let Some(ref client) = self.grsim_client {
                    if !blue_commands.is_empty() {
                        client.send_commands(&blue_commands).await?;
                    }

                    if !yellow_commands.is_empty() {
                        client.send_commands(&yellow_commands).await?;
                    }
                } else {
                    return Err("GrSim client no está inicializado".into());
                }
                self.command_map.clear();
                Ok(None)
            }
            SimulatorType::FIRASim => {
                // FIRASim puerto 20011: solo paquete FIRA (Packet con cmd.robot_commands), no SSL-Simulation
                let all_commands: Vec<RobotCommand> = self.command_map.values().cloned().collect();

                if let Some(ref client) = self.firasim_client {
                    match client.send_commands(&all_commands).await {
                        Ok(()) => {
                            self.command_map.clear();
                            Ok(None)
                        }
                        Err(e) => {
                            eprintln!("[Radio] ✗ Error enviando comandos a FIRASim: {}", e);
                            Err(e)
                        }
                    }
                } else {
                    eprintln!("[Radio] ✗ FIRASim client no está inicializado!");
                    Err("FIRASim client no está inicializado".into())
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::{KickerCommand, MotionCommand};

    fn test_radio(simulator_type: SimulatorType) -> Radio {
        Radio {
            simulator_type,
            grsim_client: None,
            firasim_client: None,
            command_map: HashMap::new(),
        }
    }

    #[test]
    fn test_radio_add_commands() {
        let mut radio = test_radio(SimulatorType::GrSim);

        let motion_cmd = MotionCommand {
            id: 0,
            team: 0,
            vx: 1.0,
            vy: 0.0,
            omega: 0.0,
            orientation: 0.0,
        };

        radio.add_motion_command(motion_cmd);
        assert_eq!(radio.command_map.len(), 1);
    }

    #[test]
    fn test_radio_add_kicker_command() {
        let mut radio = test_radio(SimulatorType::GrSim);

        let kicker_cmd = KickerCommand {
            id: 0,
            team: 0,
            kick_x: true,
            kick_z: false,
            dribbler: 5.0,
        };

        radio.add_kicker_command(kicker_cmd);
        assert_eq!(radio.command_map.len(), 1);
    }

    #[test]
    fn test_radio_keeps_same_robot_id_for_both_teams() {
        let mut radio = test_radio(SimulatorType::FIRASim);

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
}
