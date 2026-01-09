use tokio::net::UdpSocket;
use tokio::sync::mpsc;
use std::net::{Ipv4Addr, SocketAddr};
use std::error::Error;
use std::sync::{Arc, atomic::{AtomicBool, Ordering}};
use glam::Vec2;
use std::time::{Duration, Instant};

// Import the generated Protobuf structs
use crate::protos::ssl_vision_wrapper::SSL_WrapperPacket; 
use crate::protos::ssl_vision_detection::{SSL_DetectionBall, SSL_DetectionRobot};
use protobuf::Message;
use crate::gui::StatusUpdate;
use crate::tracker::Tracker;

// --- Data Structures ---

#[derive(Debug, Clone)]
#[allow(dead_code)] // These fields will be used when implementing tracking/control logic
pub struct BallData {
    pub position: Vec2,
    pub velocity: Vec2,
}

#[derive(Debug, Clone)]
#[allow(dead_code)] // These fields will be used when implementing tracking/control logic
pub struct RobotData {
    pub id: u32,
    pub team: u32,
    pub position: Vec2,
    pub orientation: f32,
    pub velocity: Vec2,
    pub angular_velocity: f32,
}

#[derive(Debug, Clone)]
#[allow(dead_code)] // These events will be consumed by control modules
pub enum VisionEvent {
    Ball(BallData),
    Robot(RobotData),
}

// Tracker is now imported from crate::tracker module

// --- The Vision Module ---

pub struct Vision {
    multicast_ip: String,
    port: u16,
    tracker: Tracker,
    tracker_enabled: Arc<AtomicBool>, // Flag compartido para habilitar/deshabilitar el tracker
    last_error_print: Instant,
    error_count: u64,
    debug_print_count: u64,
}

impl Vision {
    pub fn new(ip: String, port: u16, tracker_enabled: Arc<AtomicBool>) -> Self {
        Self {
            multicast_ip: ip,
            port,
            tracker: Tracker::new(),
            tracker_enabled,
            last_error_print: Instant::now() - Duration::from_secs(10),
            error_count: 0,
            debug_print_count: 0,
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
        // SSL Vision coordinates are in millimeters
        let raw_x = ball.x(); 
        let raw_y = ball.y(); 
        
        // Debug: Print first few ball positions to verify coordinates
        if self.debug_print_count < 5 {
            eprintln!("Ball: raw_x={} mm, raw_y={} mm, converted: x={} m, y={} m", 
                     raw_x, raw_y, raw_x / 1000.0, raw_y / 1000.0);
            self.debug_print_count += 1;
        }
        
        // Convert to meters for internal processing (tracker works in meters)
        let x_m = raw_x as f64 / 1000.0;
        let y_m = raw_y as f64 / 1000.0;
        let dt_f64 = dt as f64;

        // Usar tracker solo si está habilitado
        let (xf_m, yf_m, vx, vy) = if self.tracker_enabled.load(Ordering::Relaxed) {
            let (xf, yf, _thetaf, vx, vy, _omega) = self.tracker.track(-1, -1, x_m, y_m, 0.0, dt_f64);
            (xf, yf, vx, vy)
        } else {
            // Sin filtro: usar posiciones medidas directamente, velocidades en cero
            (x_m, y_m, 0.0, 0.0)
        };

        // Convert back to millimeters for display (field.rs expects millimeters)
        let xf_mm = (xf_m * 1000.0) as f32;
        let yf_mm = (yf_m * 1000.0) as f32;

        // Send position to UI (in millimeters)
        let _ = status_tx.send(StatusUpdate::BallPosition(Vec2::new(xf_mm, yf_mm))).await;

        // VisionEvent uses meters for internal processing
        let event = VisionEvent::Ball(BallData {
            position: Vec2::new(xf_m as f32, yf_m as f32),
            velocity: Vec2::new(vx as f32, vy as f32),
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
        // Check if required fields are present
        if !robot.has_x() || !robot.has_y() {
            // Skip robots without valid coordinates
            return Ok(());
        }

        let id = robot.robot_id();
        // SSL Vision coordinates are in millimeters, with origin at center of field
        let raw_x = robot.x();
        let raw_y = robot.y();
        let raw_theta = robot.orientation();

        // Debug: Print first few robot positions to verify coordinates
        // Print all robots initially to debug the issue
        if self.debug_print_count < 30 {
            eprintln!("Robot id={} (team {}): raw_x={} mm, raw_y={} mm, theta={} rad, has_x={}, has_y={}, has_id={}", 
                     id, team, raw_x, raw_y, raw_theta, robot.has_x(), robot.has_y(), robot.has_robot_id());
            self.debug_print_count += 1;
        }

        // Skip robots with zero coordinates (likely invalid data)
        // But allow robots at the center if they're actually there
        // The issue might be that all robots are being reported at (0,0)
        if raw_x == 0.0 && raw_y == 0.0 && self.debug_print_count < 5 {
            eprintln!("WARNING: Robot {} (team {}) has zero coordinates - this might indicate missing data", id, team);
        }

        // Convert to meters for internal processing (tracker works in meters)
        let x_m = raw_x as f64 / 1000.0;
        let y_m = raw_y as f64 / 1000.0;
        let theta = raw_theta as f64;
        let dt_f64 = dt as f64;

        // Usar tracker solo si está habilitado
        let (xf_m, yf_m, thetaf, vx, vy, omega) = if self.tracker_enabled.load(Ordering::Relaxed) {
            self.tracker.track(team, id as i32, x_m, y_m, theta, dt_f64)
        } else {
            // Sin filtro: usar posiciones medidas directamente, velocidades en cero
            (x_m, y_m, theta, 0.0, 0.0, 0.0)
        };

        // Convert back to millimeters for display (field.rs expects millimeters)
        let xf_mm = (xf_m * 1000.0) as f32;
        let yf_mm = (yf_m * 1000.0) as f32;

        // Send position to UI (in millimeters, with origin at center)
        let _ = status_tx.send(StatusUpdate::RobotPosition(id, team as u32, Vec2::new(xf_mm, yf_mm), thetaf as f32)).await;

        // VisionEvent uses meters for internal processing
        let event = VisionEvent::Robot(RobotData {
            id: id,
            team: team as u32,
            position: Vec2::new(xf_m as f32, yf_m as f32),
            orientation: thetaf as f32,
            velocity: Vec2::new(vx as f32, vy as f32),
            angular_velocity: omega as f32,
        });

        sender.send(event).await?;
        Ok(())
    }
}