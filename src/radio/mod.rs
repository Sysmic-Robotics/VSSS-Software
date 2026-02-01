mod commands;
mod grsim;
mod firasim;

pub use grsim::GrSimClient;
pub use firasim::FIRASimClient;

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
    use_radio: bool,  // true = serial, false = simulador
    simulator_type: SimulatorType,
    grsim_client: Option<GrSimClient>,
    firasim_client: Option<FIRASimClient>,
    command_map: HashMap<i32, RobotCommand>, // (id, command)
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
        port: u16
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
            use_radio,
            simulator_type,
            grsim_client,
            firasim_client,
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
    
    /// Crea robots en FIRASim antes de enviar comandos
    /// 
    /// # Argumentos
    /// * `robot_ids` - Vector de tuplas (id, team) donde team=0 es azul, team=1 es amarillo
    pub async fn create_robots(&self, robot_ids: &[(u32, u32)]) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
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
    pub async fn send_commands(&mut self) -> Result<Option<crate::protos::ssl_simulation_synchronous::SimulationSyncResponse>, Box<dyn std::error::Error + Send + Sync>> {
        if self.command_map.is_empty() {
            eprintln!("[Radio] send_commands() llamado pero command_map está vacío");
            return Ok(None);
        }
        
        eprintln!("[Radio] Enviando {} comandos (simulator_type: {:?})", self.command_map.len(), self.simulator_type);
        
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
                        let _ = client.send_commands(&blue_commands).await;
                    }
                    
                    if !yellow_commands.is_empty() {
                        let _ = client.send_commands(&yellow_commands).await;
                    }
                } else {
                    eprintln!("[Radio] ✗ GrSim client no está inicializado");
                }
                self.command_map.clear();
                return Ok(None);
            }
            SimulatorType::FIRASim => {
                // FIRASim acepta todos los comandos en un solo mensaje
                // Intentar modo síncrono para obtener detección y mantener GUI actualizada
                let all_commands: Vec<RobotCommand> = self.command_map.values().cloned().collect();
                
                if let Some(ref client) = self.firasim_client {
                    match client.send_commands_sync(&all_commands).await {
                        Ok(Some(response)) => {
                            eprintln!("[Radio] ✓ Comandos enviados y respuesta recibida de FIRASim");
                            if !response.detection.is_empty() {
                                eprintln!("[Radio] ✓✓✓ Respuesta contiene {} frames de detección!", response.detection.len());
                            }
                            self.command_map.clear();
                            return Ok(Some(response));
                        }
                        Ok(None) => {
                            eprintln!("[Radio] ✓ Comandos enviados a FIRASim (sin respuesta de detección)");
                        }
                        Err(e) => {
                            eprintln!("[Radio] ✗ Error enviando comandos a FIRASim: {}", e);
                            return Err(e);
                        }
                    }
                } else {
                    eprintln!("[Radio] ✗ FIRASim client no está inicializado!");
                    return Err("FIRASim client no está inicializado".into());
                }
            }
        }
        
        self.command_map.clear();
        Ok(None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::{MotionCommand, KickerCommand};
    
    #[tokio::test]
    async fn test_radio_add_commands() {
        let mut radio = Radio::new(false, SimulatorType::GrSim, "127.0.0.1", 20011).await.unwrap();
        
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
        let mut radio = Radio::new(false, SimulatorType::GrSim, "127.0.0.1", 20011).await.unwrap();
        
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
