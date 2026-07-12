use crate::motion::RobotCommand;
use crate::radio::commands::{serialize_commands, serialize_robot_command};
use crate::radio::transport::TeleportItem;
use protobuf::Message;
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

    /// Reposiciona robots/pelota en grSim vía `GrSim_Packet.replacement`.
    pub async fn teleport(
        &self,
        items: &[TeleportItem],
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        use crate::protos::grSim_Packet::GrSim_Packet;
        use crate::protos::grSim_Replacement::{
            GrSim_BallReplacement, GrSim_Replacement, GrSim_RobotReplacement,
        };
        let mut repl = GrSim_Replacement::new();
        for item in items {
            match *item {
                TeleportItem::Robot {
                    team,
                    id,
                    x,
                    y,
                    theta,
                } => {
                    let mut r = GrSim_RobotReplacement::new();
                    r.set_x(x);
                    r.set_y(y);
                    r.set_dir(theta.to_degrees());
                    r.set_id(id);
                    r.set_yellowteam(team == 1);
                    r.set_turnon(true);
                    repl.robots.push(r);
                }
                TeleportItem::Ball { x, y } => {
                    let mut b = GrSim_BallReplacement::new();
                    b.set_x(x);
                    b.set_y(y);
                    repl.ball = protobuf::MessageField::some(b);
                }
            }
        }
        let mut pkt = GrSim_Packet::new();
        pkt.replacement = protobuf::MessageField::some(repl);
        let mut buffer = Vec::new();
        pkt.write_to_vec(&mut buffer)?;
        self.socket.send_to(&buffer, &self.address).await?;
        Ok(())
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
