use crate::motion::RobotCommand;
use crate::radio::commands::{serialize_commands, serialize_robot_command};
use std::net::SocketAddr;
use tokio::net::UdpSocket;

/// Cliente UDP para comunicación con grSim
pub struct GrSimClient {
    socket: UdpSocket,
    address: SocketAddr,
}

impl GrSimClient {
    fn parse_destination(
        address: &str,
        port: u16,
    ) -> Result<SocketAddr, Box<dyn std::error::Error + Send + Sync>> {
        Ok(format!("{address}:{port}").parse()?)
    }

    /// Crea un nuevo cliente grSim
    pub async fn new(
        address: &str,
        port: u16,
    ) -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        let socket = UdpSocket::bind("0.0.0.0:0").await?;
        let addr = Self::parse_destination(address, port)?;

        Ok(Self {
            socket,
            address: addr,
        })
    }

    /// Envía un comando individual
    pub async fn send_command(
        &self,
        cmd: &RobotCommand,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let buffer = serialize_robot_command(cmd)?;
        self.socket.send_to(&buffer, &self.address).await?;
        Ok(())
    }

    /// Envía múltiples comandos en un solo paquete
    pub async fn send_commands(
        &self,
        commands: &[RobotCommand],
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
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

    #[test]
    fn test_grsim_client_destination_parsing() {
        let result = GrSimClient::parse_destination("127.0.0.1", 20011);
        assert_eq!(
            result.unwrap(),
            "127.0.0.1:20011".parse::<SocketAddr>().unwrap()
        );
    }
}
