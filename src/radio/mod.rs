mod commands;
mod grsim;

pub use grsim::GrSimClient;

use crate::motion::RobotCommand;
use std::collections::HashMap;

/// Módulo principal de comunicación con robots
pub struct Radio {
    use_radio: bool,  // true = serial, false = grSim
    grsim_client: Option<GrSimClient>,
    command_map: HashMap<i32, RobotCommand>, // (id, command)
}

impl Radio {
    /// Crea un nuevo Radio
    pub async fn new(use_radio: bool, grsim_address: &str, grsim_port: u16) -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        let grsim_client = if !use_radio {
            Some(GrSimClient::new(grsim_address, grsim_port).await?)
        } else {
            None // Serial no implementado aún
        };
        
        Ok(Self {
            use_radio,
            grsim_client,
            command_map: HashMap::new(),
        })
    }
    
    /// Agrega un comando de movimiento
    pub fn add_motion_command(&mut self, cmd: crate::motion::MotionCommand) {
        let id = cmd.id;
        let entry = self.command_map.entry(id).or_insert_with(|| {
            RobotCommand {
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
            }
        });
        entry.motion = cmd;
    }
    
    /// Agrega un comando de kicker
    pub fn add_kicker_command(&mut self, cmd: crate::motion::KickerCommand) {
        let id = cmd.id;
        let entry = self.command_map.entry(id).or_insert_with(|| {
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
    
    /// Envía todos los comandos acumulados
    /// Agrupa comandos por equipo ya que grSim requiere un mensaje por equipo
    pub async fn send_commands(&mut self) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        if self.command_map.is_empty() {
            return Ok(());
        }
        
        // Agrupar comandos por equipo
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
                let _ = client.send_commands(&blue_commands).await;
            }
            
            if !yellow_commands.is_empty() {
                let _ = client.send_commands(&yellow_commands).await;
            }
        }
        
        self.command_map.clear();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::{MotionCommand, KickerCommand};
    
    #[tokio::test]
    async fn test_radio_add_commands() {
        let mut radio = Radio::new(false, "127.0.0.1", 20011).await.unwrap();
        
        let motion_cmd = MotionCommand {
            id: 0,
            team: 0,
            vx: 1.0,
            vy: 0.0,
            omega: 0.0,
        };
        
        radio.add_motion_command(motion_cmd);
        assert_eq!(radio.command_map.len(), 1);
    }
    
    #[tokio::test]
    async fn test_radio_add_kicker_command() {
        let mut radio = Radio::new(false, "127.0.0.1", 20011).await.unwrap();
        
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
}
