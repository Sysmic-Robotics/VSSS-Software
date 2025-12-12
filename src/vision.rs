use tokio::net::UdpSocket;
use tokio::sync::mpsc;
use std::net::{Ipv4Addr, SocketAddr};
use std::error::Error;
use glam::Vec2;
use std::time::{Duration, Instant};

// Import the generated Protobuf structs
use crate::protos::ssl_vision_wrapper::SSL_WrapperPacket; 
use crate::protos::ssl_vision_detection::{SSL_DetectionBall, SSL_DetectionRobot};
use protobuf::Message;
use crate::ui::StatusUpdate;

// --- Data Structures ---

#[derive(Debug, Clone)]
pub struct BallData {
    pub position: Vec2,
    pub velocity: Vec2,
}

#[derive(Debug, Clone)]
pub struct RobotData {
    pub id: u32,
    pub team: u32,
    pub position: Vec2,
    pub orientation: f32,
    pub velocity: Vec2,
    pub angular_velocity: f32,
}

#[derive(Debug, Clone)]
pub enum VisionEvent {
    Ball(BallData),
    Robot(RobotData),
}

// Placeholder tracker - no actual tracking logic
struct Tracker;

impl Tracker {
    fn new() -> Self {
        Self
    }

    // Placeholder: just returns the input values with zero velocities
    fn track(&mut self, _team: i32, _id: i32, x: f32, y: f32, theta: f32, _dt: f32) -> (f32, f32, f32, f32, f32, f32) {
        // Returns: (x, y, theta, vx, vy, omega)
        // TODO: Implement actual tracking logic here
        (x, y, theta, 0.0, 0.0, 0.0)
    }
}

// --- The Vision Module ---

pub struct Vision {
    multicast_ip: String,
    port: u16,
    tracker: Tracker,
    last_error_print: Instant,
    error_count: u64,
}

impl Vision {
    pub fn new(ip: String, port: u16) -> Self {
        Self {
            multicast_ip: ip,
            port,
            tracker: Tracker::new(),
            last_error_print: Instant::now() - Duration::from_secs(10),
            error_count: 0,
        }
    }

    pub async fn run(&mut self, sender: mpsc::Sender<VisionEvent>, status_tx: mpsc::Sender<StatusUpdate>) -> Result<(), Box<dyn Error>> {
        let socket = UdpSocket::bind(SocketAddr::new("0.0.0.0".parse()?, self.port)).await?;
        
        let multi_addr: Ipv4Addr = self.multicast_ip.parse()?;
        let interface = Ipv4Addr::new(0, 0, 0, 0);
        socket.join_multicast_v4(multi_addr, interface)?;

        // Send connection status
        let _ = status_tx.send(StatusUpdate::Connected(self.multicast_ip.clone(), self.port)).await;

        let mut buf = [0u8; 4096];

        loop {
            let (len, _) = socket.recv_from(&mut buf).await?;
            let data = &buf[..len];

            // Send packet received status
            let _ = status_tx.send(StatusUpdate::PacketReceived).await;

            // --- REAL PROTOBUF PARSING ---
            
            // 1. Parse the packet
            match SSL_WrapperPacket::parse_from_bytes(data) {
                Ok(wrapper_packet) => {
                    // 2. Check if it has detection data
                    if let Some(detection) = wrapper_packet.detection.as_ref() {
                        // Note: detection.t_capture is available if you need the timestamp
                        
                        let dt = 0.016; // Fixed timestep (could be calculated from t_capture)

                        // Send detection counts
                        let _ = status_tx.send(StatusUpdate::BallDetected(detection.balls.len())).await;
                        let _ = status_tx.send(StatusUpdate::RobotsDetected(
                            detection.robots_yellow.len() + detection.robots_blue.len()
                        )).await;

                        // 3. Process Balls
                        for ball in detection.balls.iter() {
                            self.process_ball(&sender, &status_tx, ball, dt).await?;
                        }

                        // 4. Process Yellow Robots (Team 1)
                        for robot in detection.robots_yellow.iter() {
                            self.process_robot(&sender, &status_tx, robot, 1, dt).await?;
                        }

                        // 5. Process Blue Robots (Team 0)
                        for robot in detection.robots_blue.iter() {
                            self.process_robot(&sender, &status_tx, robot, 0, dt).await?;
                        }
                    }
                },
                Err(e) => {
                    self.error_count += 1;
                    if self.last_error_print.elapsed() >= Duration::from_secs(2) {
                        eprintln!("Failed to parse packet: {} ({} errors in last 2s)", e, self.error_count);
                        self.last_error_print = Instant::now();
                        self.error_count = 0;
                    }
                }
            }
        }
    }

    // Helper for Balls
    async fn process_ball(&mut self, sender: &mpsc::Sender<VisionEvent>, status_tx: &mpsc::Sender<StatusUpdate>, ball: &SSL_DetectionBall, dt: f32) -> Result<(), Box<dyn Error>> {
        // Access protobuf fields directly (e.g. ball.x())
        // Note: rust-protobuf 3.x often uses Option<f32> for optional fields, 
        // but simple fields might be direct. Check generated code if .x() or .x is needed.
        let raw_x = ball.x(); 
        let raw_y = ball.y(); 
        
        let x = raw_x / 1000.0;
        let y = raw_y / 1000.0;

        let (xf, yf, _thetaf, vx, vy, _omega) = self.tracker.track(-1, -1, x, y, 0.0, dt);

        // Send position to UI
        let _ = status_tx.send(StatusUpdate::BallPosition(Vec2::new(xf, yf))).await;

        let event = VisionEvent::Ball(BallData {
            position: Vec2::new(xf, yf),
            velocity: Vec2::new(vx, vy),
        });

        if let Err(_) = sender.send(event).await {
            return Err("Channel closed".into());
        }
        Ok(())
    }

    // Helper for Robots
    async fn process_robot(&mut self, sender: &mpsc::Sender<VisionEvent>, 
                           status_tx: &mpsc::Sender<StatusUpdate>,
                           robot: &SSL_DetectionRobot, 
                           team: i32, dt: f32) -> Result<(), Box<dyn Error>> 
    {
        let id = robot.robot_id(); // In newer protobufs, might be robot_id() or just robot_id
        let raw_x = robot.x();
        let raw_y = robot.y();
        let raw_theta = robot.orientation();

        let x = raw_x / 1000.0;
        let y = raw_y / 1000.0;
        let theta = raw_theta;

        let (xf, yf, thetaf, vx, vy, omega) = self.tracker.track(team, id as i32, x, y, theta, dt);

        // Send position to UI
        let _ = status_tx.send(StatusUpdate::RobotPosition(id, team as u32, Vec2::new(xf, yf), thetaf)).await;

        let event = VisionEvent::Robot(RobotData {
            id: id,
            team: team as u32,
            position: Vec2::new(xf, yf),
            orientation: thetaf,
            velocity: Vec2::new(vx, vy),
            angular_velocity: omega,
        });

        sender.send(event).await?;
        Ok(())
    }
}