use tokio::net::UdpSocket;
use std::net::SocketAddr;
use crate::radio::commands::serialize_to_firasim;
use crate::motion::RobotCommand;

/// Cliente UDP para comunicación con FIRASim usando protocolo SSL-Simulation
pub struct FIRASimClient {
    socket: UdpSocket,
    address: SocketAddr,
}

impl FIRASimClient {
    /// Crea un nuevo cliente FIRASim
    /// 
    /// # Argumentos
    /// * `address` - Dirección IP del simulador (típicamente "127.0.0.1")
    /// * `port` - Puerto del simulador (típicamente 10300 para FIRASim)
    pub async fn new(address: &str, port: u16) -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        let socket = UdpSocket::bind("0.0.0.0:0").await?;
        let addr: SocketAddr = format!("{}:{}", address, port).parse()?;
        
        Ok(Self {
            socket,
            address: addr,
        })
    }
    
    /// Envía un comando individual
    pub async fn send_command(&self, cmd: &RobotCommand) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let buffer = serialize_to_firasim(&[cmd.clone()])?;
        self.socket.send_to(&buffer, &self.address).await?;
        Ok(())
    }
    
    /// Envía múltiples comandos en un solo paquete RobotControl
    /// 
    /// FIRASim acepta comandos de múltiples robots en un solo mensaje RobotControl,
    /// a diferencia de grSim que requiere un mensaje por equipo.
    pub async fn send_commands(&self, commands: &[RobotCommand]) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        if commands.is_empty() {
            return Ok(());
        }
        
        let buffer = serialize_to_firasim(commands)?;
        
        if buffer.is_empty() {
            return Err("Buffer serializado está vacío".into());
        }
        
        // Log detallado del primer comando para debugging
        if let Some(first_cmd) = commands.first() {
            eprintln!("[FIRASim] Enviando comando: robot_id={}, team={}, vx={:.3}, vy={:.3}, omega={:.3}, buffer_size={} bytes, destino={}", 
                     first_cmd.id, first_cmd.team, first_cmd.motion.vx, first_cmd.motion.vy, 
                     first_cmd.motion.omega, buffer.len(), self.address);
        }
        
        match self.socket.send_to(&buffer, &self.address).await {
            Ok(bytes_sent) => {
                eprintln!("[FIRASim] ✓ Enviados {} bytes a {}", bytes_sent, self.address);
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
        let result = FIRASimClient::new("127.0.0.1", 10300).await;
        assert!(result.is_ok());
    }
}
