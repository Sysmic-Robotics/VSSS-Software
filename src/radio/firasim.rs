use tokio::net::UdpSocket;
use std::net::SocketAddr;
use std::fs::OpenOptions;
use std::io::Write;
use crate::radio::commands::{serialize_to_firasim, serialize_to_fira_actuator};
use crate::motion::RobotCommand;
use crate::protos::ssl_simulation_control::{SimulatorCommand, SimulatorControl, TeleportRobot};
use crate::protos::ssl_simulation_synchronous::{SimulationSyncRequest, SimulationSyncResponse};
use crate::protos::ssl_gc_common::{RobotId, Team};
use protobuf::Message;

/// Cliente UDP para comunicación con FIRASim usando protocolo SSL-Simulation
pub struct FIRASimClient {
    socket: UdpSocket,
    address: SocketAddr,
    control_address: SocketAddr, // Puerto para SimulatorCommand (típicamente el mismo que RobotControl)
}

impl FIRASimClient {
    /// Crea un nuevo cliente FIRASim
    /// 
    /// # Argumentos
    /// * `address` - Dirección IP del simulador (típicamente "127.0.0.1")
    /// * `port` - Puerto del simulador para RobotControl (típicamente 20011 o 10300 según configuración de FIRASim)
    pub async fn new(address: &str, port: u16) -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        let socket = UdpSocket::bind("0.0.0.0:0").await?;
        let addr: SocketAddr = format!("{}:{}", address, port).parse()?;
        
        // SimulatorCommand generalmente usa el mismo puerto que RobotControl
        let control_addr: SocketAddr = format!("{}:{}", address, port).parse()?;
        
        Ok(Self {
            socket,
            address: addr,
            control_address: control_addr,
        })
    }
    
    /// Crea robots en FIRASim usando TeleportRobot con present=true
    /// 
    /// # Argumentos
    /// * `robot_ids` - Vector de tuplas (id, team) donde team=0 es azul, team=1 es amarillo
    /// * `positions` - Posiciones iniciales opcionales (x, y, orientation) en metros. Si None, usa (0, 0, 0)
    pub async fn create_robots(&self, robot_ids: &[(u32, u32)]) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let mut sim_control = SimulatorControl::new();
        
        for (id, team) in robot_ids {
            let mut teleport = TeleportRobot::new();
            
            // Crear RobotId con equipo e ID
            let mut robot_id = RobotId::new();
            robot_id.set_id(*id);
            robot_id.set_team(match *team {
                0 => Team::BLUE,
                1 => Team::YELLOW,
                _ => Team::UNKNOWN,
            });
            
            teleport.id = protobuf::MessageField::some(robot_id);
            
            // Establecer posición inicial (distribuir robots en el campo VSS)
            // Campo VSS: ~1.5m x 1.3m, origen en el centro
            // Distribuir robots bien separados para que sean claramente visibles
            // VSS tiene 3 robots por equipo (IDs 0-2)
            let positions = [
                (-0.4, 0.0),   // Robot 0: izquierda del centro  
                (0.0, 0.2),    // Robot 1: centro-arriba
                (0.4, 0.0),    // Robot 2: derecha del centro
            ];
            let (x, y) = positions.get(*id as usize).copied().unwrap_or((0.0, 0.0));
            
            teleport.set_x(x);
            teleport.set_y(y);
            teleport.set_orientation(0.0);
            teleport.set_v_x(0.0);
            teleport.set_v_y(0.0);
            teleport.set_v_angular(0.0);
            
            eprintln!("[FIRASim]   Robot ID {}: posición ({:.2}, {:.2}) m, equipo {}", 
                     id, x, y, if *team == 0 { "AZUL" } else { "AMARILLO" });
            
            // IMPORTANTE: present=true crea el robot si no existe
            teleport.set_present(true);
            
            sim_control.teleport_robot.push(teleport);
        }
        
        // Crear SimulatorCommand
        let mut sim_cmd = SimulatorCommand::new();
        sim_cmd.control = protobuf::MessageField::some(sim_control);
        
        // Serializar y enviar
        let mut buffer = Vec::new();
        sim_cmd.write_to_vec(&mut buffer)?;
        
        eprintln!("[FIRASim] Creando {} robots en FIRASim...", robot_ids.len());
        for (id, team) in robot_ids {
            eprintln!("[FIRASim]   - Robot ID {} del equipo {} ({} bytes)", 
                     id, 
                     if *team == 0 { "AZUL" } else { "AMARILLO" },
                     buffer.len());
        }
        eprintln!("[FIRASim] Enviando SimulatorCommand a {} ({} bytes)", self.control_address, buffer.len());
        match self.socket.send_to(&buffer, &self.control_address).await {
            Ok(bytes_sent) => {
                eprintln!("[FIRASim] ✓ Comando de creación de robots enviado ({} bytes) a {}", bytes_sent, self.control_address);
                eprintln!("[FIRASim] Primeros 32 bytes del mensaje (hex): {:02x?}", &buffer[..buffer.len().min(32)]);
                // Esperar un poco para que FIRASim procese el comando
                tokio::time::sleep(tokio::time::Duration::from_millis(200)).await;
                Ok(())
            }
            Err(e) => {
                eprintln!("[FIRASim] ✗ Error enviando comando de creación de robots a {}: {}", self.control_address, e);
                Err(e.into())
            }
        }
    }
    
    /// Envía un comando individual
    pub async fn send_command(&self, cmd: &RobotCommand) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let buffer = serialize_to_firasim(&[cmd.clone()])?;
        self.socket.send_to(&buffer, &self.address).await?;
        Ok(())
    }
    
    /// Envía múltiples comandos usando SimulationSyncRequest y recibe SimulationSyncResponse con datos de detección
    /// 
    /// FIRASim usa protocolo síncrono: enviamos SimulationSyncRequest con comandos y recibimos
    /// SimulationSyncResponse que contiene los datos de detección (SSL_DetectionFrame).
    pub async fn send_commands_sync(&self, commands: &[RobotCommand]) -> Result<Option<SimulationSyncResponse>, Box<dyn std::error::Error + Send + Sync>> {
        if commands.is_empty() {
            return Ok(None);
        }

        // Enviar comandos asincrónicos como fallback para asegurar movimiento
        // Si FIRASim no responde a modo síncrono, al menos se aplican velocidades
        if let Err(e) = self.send_commands(commands).await {
            eprintln!("[FIRASim] ⚠ Error enviando comandos async (fallback): {}", e);
        }
        
        // Crear SimulationSyncRequest con los comandos
        let mut sync_request = SimulationSyncRequest::new();
        sync_request.set_sim_step(0.016); // ~60 Hz
        
        // Agregar comandos de robots al request
        let robot_control_buffer = serialize_to_firasim(commands)?;
        if !robot_control_buffer.is_empty() {
            use crate::protos::ssl_simulation_robot_control::RobotControl;
            match RobotControl::parse_from_bytes(&robot_control_buffer) {
                Ok(robot_control) => {
                    sync_request.robot_control = protobuf::MessageField::some(robot_control);
                },
                Err(e) => {
                    eprintln!("[FIRASim] ⚠ Error parseando RobotControl para sync request: {}", e);
                }
            }
        }
        
        // Serializar SimulationSyncRequest
        let mut request_buffer = Vec::new();
        sync_request.write_to_vec(&mut request_buffer)?;
        
        // Enviar request
        match self.socket.send_to(&request_buffer, &self.address).await {
            Ok(bytes_sent) => {
                if commands.len() <= 3 {
                    eprintln!("[FIRASim] ✓ SimulationSyncRequest enviado ({} bytes) con {} comandos", bytes_sent, commands.len());
                }
            }
            Err(e) => {
                eprintln!("[FIRASim] ✗ Error enviando SimulationSyncRequest: {}", e);
                return Err(e.into());
            }
        }
        
        // Recibir respuesta (con timeout más largo)
        let mut response_buffer = vec![0u8; 65536];
        match tokio::time::timeout(
            tokio::time::Duration::from_millis(500), // Aumentado a 500ms
            self.socket.recv_from(&mut response_buffer)
        ).await {
            Ok(Ok((size, src_addr))) => {
                response_buffer.truncate(size);
                eprintln!("[FIRASim] ✓ Respuesta recibida: {} bytes desde {}", size, src_addr);
                eprintln!("[FIRASim] Primeros 32 bytes (hex): {:02x?}", &response_buffer[..response_buffer.len().min(32)]);
                
                match SimulationSyncResponse::parse_from_bytes(&response_buffer) {
                    Ok(response) => {
                        if !response.detection.is_empty() {
                            eprintln!("[FIRASim] ✓✓✓ SimulationSyncResponse recibido: {} frames de detección!", response.detection.len());
                            // #region agent log
                            if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open("/home/seba/sysmic/vssl/rustengine/.cursor/debug.log") {
                                let _ = writeln!(file, r#"{{"id":"log_{}_{}","timestamp":{},"location":"firasim.rs:{}","message":"SimulationSyncResponse recibido con detección","data":{{"frames":{}}},"sessionId":"debug-session","runId":"run1","hypothesisId":"hypothesis_14"}}"#, 
                                    std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_millis(),
                                    1,
                                    std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_millis(),
                                    180, response.detection.len());
                            }
                            // #endregion
                        } else {
                            eprintln!("[FIRASim] ⚠ SimulationSyncResponse recibido pero SIN frames de detección");
                        }
                        Ok(Some(response))
                    },
                    Err(e) => {
                        eprintln!("[FIRASim] ✗ Error parseando SimulationSyncResponse: {}", e);
                        eprintln!("[FIRASim] Buffer recibido (primeros 64 bytes): {:02x?}", &response_buffer[..response_buffer.len().min(64)]);
                        // #region agent log
                        if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open("/home/seba/sysmic/vssl/rustengine/.cursor/debug.log") {
                            let _ = writeln!(file, r#"{{"id":"log_{}_{}","timestamp":{},"location":"firasim.rs:{}","message":"Error parseando SimulationSyncResponse","data":{{"error":"{}","size":{}}},"sessionId":"debug-session","runId":"run1","hypothesisId":"hypothesis_14"}}"#, 
                                std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_millis(),
                                2,
                                std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_millis(),
                                185, e, size);
                        }
                        // #endregion
                        Ok(None)
                    }
                }
            },
            Ok(Err(e)) => {
                eprintln!("[FIRASim] ✗ Error recibiendo respuesta: {}", e);
                // #region agent log
                if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open("/home/seba/sysmic/vssl/rustengine/.cursor/debug.log") {
                    let _ = writeln!(file, r#"{{"id":"log_{}_{}","timestamp":{},"location":"firasim.rs:{}","message":"Error recibiendo respuesta","data":{{"error":"{}"}},"sessionId":"debug-session","runId":"run1","hypothesisId":"hypothesis_14"}}"#, 
                        std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_millis(),
                        3,
                        std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_millis(),
                        195, e);
                }
                // #endregion
                Ok(None)
            },
            Err(_) => {
                // Timeout - no es crítico, seguimos sin datos de detección
                eprintln!("[FIRASim] ⚠ Timeout esperando SimulationSyncResponse (500ms)");
                // #region agent log
                if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open("/home/seba/sysmic/vssl/rustengine/.cursor/debug.log") {
                    let _ = writeln!(file, r#"{{"id":"log_{}_{}","timestamp":{},"location":"firasim.rs:{}","message":"Timeout esperando SimulationSyncResponse","data":{{"timeout_ms":500}},"sessionId":"debug-session","runId":"run1","hypothesisId":"hypothesis_14"}}"#, 
                        std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_millis(),
                        4,
                        std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_millis(),
                        200);
                }
                // #endregion
                Ok(None)
            }
        }
    }
    
    /// Envía múltiples comandos a FIRASim (VSS).
    /// Usa el protocolo FIRA (Packet con Commands: id, yellowteam, wheel_left, wheel_right)
    /// que FIRASim espera en el puerto 20011, no SSL-Simulation.
    pub async fn send_commands(&self, commands: &[RobotCommand]) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        if commands.is_empty() {
            return Ok(());
        }

        let buffer = serialize_to_fira_actuator(commands)?;

        if buffer.is_empty() {
            return Err("Buffer FIRA serializado está vacío".into());
        }

        match self.socket.send_to(&buffer, &self.address).await {
            Ok(_) => {
                // Log reducido: solo primeros 3 comandos y cada 60 paquetes
                use std::sync::atomic::{AtomicU64, Ordering};
                static PACKET_COUNT: AtomicU64 = AtomicU64::new(0);
                let n = PACKET_COUNT.fetch_add(1, Ordering::Relaxed) + 1;
                if commands.len() <= 3 || n % 60 == 0 {
                    eprintln!("[FIRASim] ✓ {} comandos enviados a {} (paquete #{})",
                             commands.len(), self.address, n);
                }
                Ok(())
            }
            Err(e) => {
                eprintln!("[FIRASim] ✗ Error enviando a {}: {}", self.address, e);
                Err(e.into())
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::{MotionCommand, KickerCommand};
    
    #[tokio::test]
    async fn test_firasim_client_creation() {
        // Este test solo verifica que se puede crear el cliente
        // No envía datos reales ya que requiere FIRASim corriendo
        let result = FIRASimClient::new("127.0.0.1", 20011).await;
        assert!(result.is_ok());
    }
}
