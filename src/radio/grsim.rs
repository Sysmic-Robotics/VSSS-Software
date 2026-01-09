use tokio::net::UdpSocket;
use std::net::SocketAddr;
use crate::radio::commands::serialize_commands;
use crate::motion::RobotCommand;

/// Cliente UDP para comunicación con grSim
pub struct GrSimClient {
    socket: UdpSocket,
    address: SocketAddr,
}

impl GrSimClient {
    /// Crea un nuevo cliente grSim
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
        let buffer = serialize_commands(&[cmd.clone()])?;
        self.socket.send_to(&buffer, &self.address).await?;
        Ok(())
    }
    
    /// Envía múltiples comandos en un solo paquete
    pub async fn send_commands(&self, commands: &[RobotCommand]) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let buffer = serialize_commands(commands)?;
        
        if buffer.is_empty() {
            return Err("Buffer serializado está vacío".into());
        }
        
        self.socket.send_to(&buffer, &self.address).await?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::{MotionCommand, KickerCommand};
    
    #[tokio::test]
    async fn test_grsim_client_creation() {
        // Este test solo verifica que se puede crear el cliente
        // No envía datos reales ya que requiere grSim corriendo
        let result = GrSimClient::new("127.0.0.1", 20011).await;
        assert!(result.is_ok());
    }
}
