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
                    r.set_dir(theta); // grados, como `TeleportItem`
                    r.set_id(id);
                    r.set_yellowteam(team == 1);
                    r.set_turnon(true);
                    repl.robots.push(r);
                }
                TeleportItem::Ball { x, y, vx, vy } => {
                    let mut b = GrSim_BallReplacement::new();
                    b.set_x(x);
                    b.set_y(y);
                    b.set_vx(vx);
                    b.set_vy(vy);
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

    #[tokio::test]
    async fn teleport_orientation_goes_in_degrees() {
        // `TeleportItem.theta` ya viene en grados (la unidad de `dir`): sin convertir.
        use crate::protos::grSim_Packet::GrSim_Packet;
        let sim = UdpSocket::bind("127.0.0.1:0").await.unwrap();
        let port = sim.local_addr().unwrap().port();
        let client = GrSimClient::new("127.0.0.1", port).await.unwrap();
        let item = TeleportItem::Robot { team: 1, id: 2, x: 0.1, y: 0.2, theta: 90.0 };
        client.teleport(&[item]).await.unwrap();
        let mut buf = [0u8; 1024];
        let n = sim.recv(&mut buf).await.unwrap();
        let pkt = GrSim_Packet::parse_from_bytes(&buf[..n]).unwrap();
        assert_eq!(pkt.replacement.robots[0].dir(), 90.0);
    }

    #[tokio::test]
    async fn ball_teleport_carries_its_velocity() {
        use crate::protos::grSim_Packet::GrSim_Packet;
        let sim = UdpSocket::bind("127.0.0.1:0").await.unwrap();
        let port = sim.local_addr().unwrap().port();
        let client = GrSimClient::new("127.0.0.1", port).await.unwrap();
        client.teleport(&[TeleportItem::Ball { x: 0.3, y: 0.4, vx: 0.0, vy: -0.7 }]).await.unwrap();
        let mut buf = [0u8; 1024];
        let n = sim.recv(&mut buf).await.unwrap();
        let pkt = GrSim_Packet::parse_from_bytes(&buf[..n]).unwrap();
        let b = pkt.replacement.ball.clone().unwrap();
        assert_eq!((b.x(), b.y(), b.vx(), b.vy()), (0.3, 0.4, 0.0, -0.7));
    }
}
