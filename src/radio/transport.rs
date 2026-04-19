use crate::motion::RobotCommand;
use async_trait::async_trait;

pub type TransportError = Box<dyn std::error::Error + Send + Sync>;

#[async_trait]
pub trait RobotTransport: Send + Sync {
    async fn send_commands(&mut self, commands: &[RobotCommand]) -> Result<(), TransportError>;

    async fn create_robots(&mut self, _robot_ids: &[(u32, u32)]) -> Result<(), TransportError> {
        Ok(())
    }
}

pub struct FiraSimTransport {
    client: super::FIRASimClient,
}

impl FiraSimTransport {
    pub async fn new(address: &str, port: u16) -> Result<Self, TransportError> {
        Ok(Self {
            client: super::FIRASimClient::new(address, port).await?,
        })
    }
}

#[async_trait]
impl RobotTransport for FiraSimTransport {
    async fn send_commands(&mut self, commands: &[RobotCommand]) -> Result<(), TransportError> {
        self.client.send_commands(commands).await
    }

    async fn create_robots(&mut self, robot_ids: &[(u32, u32)]) -> Result<(), TransportError> {
        self.client.create_robots(robot_ids).await
    }
}

pub struct GrSimTransport {
    client: super::GrSimClient,
}

impl GrSimTransport {
    pub async fn new(address: &str, port: u16) -> Result<Self, TransportError> {
        Ok(Self {
            client: super::GrSimClient::new(address, port).await?,
        })
    }
}

#[async_trait]
impl RobotTransport for GrSimTransport {
    async fn send_commands(&mut self, commands: &[RobotCommand]) -> Result<(), TransportError> {
        let (blue, yellow): (Vec<_>, Vec<_>) = commands.iter().cloned().partition(|c| c.team == 0);
        if !blue.is_empty() {
            self.client.send_commands(&blue).await?;
        }
        if !yellow.is_empty() {
            self.client.send_commands(&yellow).await?;
        }
        Ok(())
    }
}
