use tokio::net::UdpSocket;
use tokio::sync::mpsc;
use std::net::Ipv4Addr;
use std::error::Error;
use std::sync::{Arc, atomic::{AtomicBool, Ordering}};
use glam::Vec2;
use std::time::{Duration, Instant};

// Import the generated Protobuf structs
// FIRA/VSSS protocol (FIRASim): Environment with Frame (ball, robots) — VSSSLeague/FIRAClient, VSSSProto
use crate::protos::fira_packet::Environment as FiraEnvironment;
use crate::protos::fira_common::{Ball as FiraBall, Robot as FiraRobot};
use crate::protos::ssl_vision_wrapper::SSL_WrapperPacket;
use crate::protos::ssl_vision_detection::{SSL_DetectionBall, SSL_DetectionRobot};
use protobuf::Message;
use crate::gui::StatusUpdate;
use crate::tracker::Tracker;

/// Log por cada robot azul enviado a la GUI (SSL). Desactivado por defecto; usar `[FieldAudit]` en main.
const VISION_LOG_EVERY_BLUE_ROBOT_TO_GUI: bool = false;
/// Log de cada N detecciones FIRA/SSL (≈1 s a ~60 Hz si N=60).
const VISION_LOG_DETECTION_EVERY_N: u64 = 120;
/// Log de estadísticas de paquetes cada N paquetes (~5 s a ~60 Hz si N=300).
const VISION_LOG_PACKET_STATS_EVERY_N: u64 = 3000;

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
        }
    }

    pub async fn run(&mut self, sender: mpsc::Sender<VisionEvent>, status_tx: mpsc::Sender<StatusUpdate>) -> Result<(), Box<dyn Error>> {
        // Crear socket UDP y unirse al grupo multicast
        let bind_addr = format!("0.0.0.0:{}", self.port);
        eprintln!("[Vision] Configurando socket UDP en {} para multicast {}:{}", bind_addr, self.multicast_ip, self.port);
        
        let socket = UdpSocket::bind(&bind_addr).await
            .map_err(|e| format!("Error haciendo bind a {}: {}", bind_addr, e))?;
        
        eprintln!("[Vision] Socket creado exitosamente");
        
        let multi_addr: Ipv4Addr = self.multicast_ip.parse()
            .map_err(|e| format!("Error parseando IP multicast {}: {}", self.multicast_ip, e))?;
        
        // En Windows, obtener la IP local de la interfaz de red
        // Intentar obtener la IP local conectando a una dirección externa y luego obtener local_addr
        let interface = match std::net::UdpSocket::bind("0.0.0.0:0") {
            Ok(test_socket) => {
                // Intentar conectar a una dirección externa para obtener la IP local
                match test_socket.connect("8.8.8.8:80") {
                    Ok(_) => {
                        match test_socket.local_addr() {
                            Ok(addr) => {
                                if let std::net::IpAddr::V4(ip) = addr.ip() {
                                    eprintln!("[Vision] Usando IP local de interfaz: {}", ip);
                                    ip
                                } else {
                                    eprintln!("[Vision] No se pudo obtener IPv4 local, usando 127.0.0.1");
                                    Ipv4Addr::new(127, 0, 0, 1)
                                }
                            }
                            Err(_) => {
                                eprintln!("[Vision] No se pudo obtener dirección local, usando 127.0.0.1");
                                Ipv4Addr::new(127, 0, 0, 1)
                            }
                        }
                    }
                    Err(_) => {
                        eprintln!("[Vision] No se pudo conectar para obtener IP local, usando 127.0.0.1");
                        Ipv4Addr::new(127, 0, 0, 1)
                    }
                }
            }
            Err(_) => {
                eprintln!("[Vision] No se pudo crear socket de prueba, usando 127.0.0.1");
                Ipv4Addr::new(127, 0, 0, 1)
            }
        };
        
        eprintln!("[Vision] Uniéndose al grupo multicast {}:{} con interfaz {}", self.multicast_ip, self.port, interface);
        
        // join_multicast_v4 es síncrono pero funciona con sockets async de tokio
        eprintln!("[Vision] Intentando unirse al multicast {} en interfaz {}...", multi_addr, interface);
        match socket.join_multicast_v4(multi_addr, interface) {
            Ok(_) => {
                eprintln!("[Vision] ✓ Unido exitosamente al grupo multicast {}:{} en interfaz {}", self.multicast_ip, self.port, interface);
            }
            Err(e) => {
                // Si falla con la IP local, intentar con 0.0.0.0 como último recurso
                eprintln!("[Vision] ⚠ Falló con interfaz {} (error: {}), intentando con 0.0.0.0...", interface, e);
                match socket.join_multicast_v4(multi_addr, Ipv4Addr::new(0, 0, 0, 0)) {
                    Ok(_) => {
                        eprintln!("[Vision] ✓ Unido exitosamente al grupo multicast {}:{} usando 0.0.0.0", self.multicast_ip, self.port);
                    }
                    Err(e2) => {
                        let error_msg = format!("Error uniéndose al multicast {}:{}: {}. Verifica que el firewall no esté bloqueando y que FIRASim esté enviando datos.", 
                                               self.multicast_ip, self.port, e2);
                        eprintln!("[Vision] ✗ {}", error_msg);
                        eprintln!("[Vision] Posibles soluciones:");
                        eprintln!("  1. Verifica que FIRASim esté corriendo");
                        eprintln!("  2. Verifica la configuración de FIRASim: Vision multicast = {}, port = {}", self.multicast_ip, self.port);
                        eprintln!("  3. Ejecuta como administrador");
                        eprintln!("  4. Verifica el firewall de Windows");
                        eprintln!("  5. Prueba con: netsh interface ip show joins (para ver grupos multicast activos)");
                        return Err(error_msg.into());
                    }
                }
            }
        }
        
        // Verificar que el socket está listo para recibir
        eprintln!("[Vision] Socket configurado. Dirección local: {:?}", socket.local_addr());

        // Verificar que el socket está listo para recibir
        match socket.local_addr() {
            Ok(addr) => eprintln!("[Vision] Socket listo. Dirección local: {}", addr),
            Err(e) => eprintln!("[Vision] ⚠ No se pudo obtener dirección local: {}", e),
        }
        
        // Send connection status
        let _ = status_tx.send(StatusUpdate::Connected(self.multicast_ip.clone(), self.port)).await;
        eprintln!("[Vision] ========================================");
        eprintln!("[Vision] ✓ Sistema de visión inicializado");
        eprintln!("[Vision] Esperando paquetes de FIRASim en {}:{}", self.multicast_ip, self.port);
        eprintln!("[Vision] ========================================");
        eprintln!("[Vision] INSTRUCCIONES PARA VERIFICAR FIRASim:");
        eprintln!("[Vision] 1. Abre FIRASim");
        eprintln!("[Vision] 2. Ve a Configuración → Network");
        eprintln!("[Vision] 3. Verifica que 'Vision multicast address' = {}", self.multicast_ip);
        eprintln!("[Vision] 4. Verifica que 'Vision port' = {}", self.port);
        eprintln!("[Vision] 5. Asegúrate de que hay robots habilitados en el campo");
        eprintln!("[Vision] 6. Verifica que el simulador está corriendo (no pausado)");
        eprintln!("[Vision] ========================================");

        let mut buf = [0u8; 65536];
        let mut packet_count = 0u64;
        let mut last_packet_time = Instant::now();
        let mut last_status_print = Instant::now();
        let mut detection_count = 0u64;
        let mut geometry_only_count = 0u64;

        eprintln!("[Vision] === INICIANDO RECEPCIÓN DE PAQUETES ===");
        eprintln!("[Vision] Esperando paquetes multicast de {}:{}", self.multicast_ip, self.port);

        loop {
            tokio::select! {
                result = socket.recv_from(&mut buf) => {
                    match result {
                        Ok((len, addr)) => {
                            packet_count += 1;
                            last_packet_time = Instant::now();
                            let data = &buf[..len];

                            // Log primer paquete y cada 120 paquetes
                            if packet_count == 1 {
                                eprintln!("[Vision] ✓ PRIMER PAQUETE de {} ({} bytes)", addr, len);
                                eprintln!("[Vision] Hex: {:02x?}", &data[..len.min(48)]);
                            }

                            let _ = status_tx.send(StatusUpdate::PacketReceived).await;

                            // FIRASim/VSSS envía protocolo FIRA: fira_message.sim_to_ref.Environment
                            // (VSSSLeague/FIRAClient, VSSSProto). Probar primero ese formato.
                            let mut handled = false;
                            if let Ok(env) = FiraEnvironment::parse_from_bytes(data) {
                                if let Some(frame) = env.frame.as_ref() {
                                    detection_count += 1;
                                    let robot_count = frame.robots_yellow.len() + frame.robots_blue.len();
                                    let ball_count = if frame.ball.is_some() { 1 } else { 0 };

                                    if detection_count <= 3 || detection_count % VISION_LOG_DETECTION_EVERY_N == 0 {
                                        eprintln!("[Vision] ✓ DETECTION (FIRA) #{}: {} robots, {} balls",
                                                 detection_count, robot_count, ball_count);
                                    }

                                    let _ = status_tx.send(StatusUpdate::BallDetected(ball_count)).await;
                                    let _ = status_tx.send(StatusUpdate::RobotsDetected(robot_count)).await;

                                    let dt = 0.016f32;
                                    if let Some(ball) = frame.ball.as_ref() {
                                        let _ = self.process_ball_fira(&sender, &status_tx, ball, dt).await;
                                    }
                                    for robot in frame.robots_yellow.iter() {
                                        let _ = self.process_robot_fira(&sender, &status_tx, robot, 1, dt).await;
                                    }
                                    for robot in frame.robots_blue.iter() {
                                        let _ = self.process_robot_fira(&sender, &status_tx, robot, 0, dt).await;
                                    }
                                    handled = true;
                                }
                            }

                            // Fallback: SSL Vision (grSim u otros)
                            if !handled {
                                match SSL_WrapperPacket::parse_from_bytes(data) {
                                    Ok(packet) => {
                                        if let Some(detection) = packet.detection.as_ref() {
                                            detection_count += 1;
                                            let robot_count = detection.robots_yellow.len() + detection.robots_blue.len();
                                            let ball_count = detection.balls.len();

                                            if detection_count <= 3 || detection_count % VISION_LOG_DETECTION_EVERY_N == 0 {
                                                eprintln!("[Vision] ✓ DETECTION (SSL) #{}: {} robots, {} balls",
                                                         detection_count, robot_count, ball_count);
                                            }

                                            let _ = status_tx.send(StatusUpdate::BallDetected(ball_count)).await;
                                            let _ = status_tx.send(StatusUpdate::RobotsDetected(robot_count)).await;

                                            let dt = 0.016f32;
                                            for ball in detection.balls.iter() {
                                                let _ = self.process_ball(&sender, &status_tx, ball, dt).await;
                                            }
                                            for robot in detection.robots_yellow.iter() {
                                                let _ = self.process_robot(&sender, &status_tx, robot, 1, dt).await;
                                            }
                                            for robot in detection.robots_blue.iter() {
                                                let _ = self.process_robot(&sender, &status_tx, robot, 0, dt).await;
                                            }
                                        } else {
                                            geometry_only_count += 1;
                                        }
                                    }
                                    Err(_) => {
                                        geometry_only_count += 1;
                                    }
                                }
                            }
                            
                            if packet_count % VISION_LOG_PACKET_STATS_EVERY_N == 0 && packet_count > 0 {
                                eprintln!("[Vision] Stats: {} packets ({} detection, {} geometry-only)", 
                                         packet_count, detection_count, geometry_only_count);
                            }
                        },
                        Err(e) => {
                            eprintln!("[Vision] Error UDP: {}", e);
                            return Err(e.into());
                        }
                    }
                }
                _ = tokio::time::sleep(Duration::from_secs(5)) => {
                    if packet_count == 0 {
                        if last_status_print.elapsed() >= Duration::from_secs(5) {
                            eprintln!("[Vision] ⚠ Sin paquetes. Verifica FIRASim está enviando a {}:{}", 
                                     self.multicast_ip, self.port);
                            last_status_print = Instant::now();
                        }
                    } else if last_packet_time.elapsed() > Duration::from_secs(5) {
                        eprintln!("[Vision] ⚠ Sin paquetes en 5s (total: {}, detection: {})", 
                                 packet_count, detection_count);
                    }
                }
            }
        }
    }
    
    /// Intenta extraer SSL_DetectionFrame manualmente de un paquete que falló el parsing normal
    fn try_extract_detection(&self, data: &[u8]) -> Option<crate::protos::ssl_vision_detection::SSL_DetectionFrame> {
        use protobuf::CodedInputStream;
        use crate::protos::ssl_vision_detection::SSL_DetectionFrame;
        
        let mut input = CodedInputStream::from_bytes(data);
        
        while let Ok(Some(tag)) = input.read_raw_tag_or_eof() {
            let field_num = tag >> 3;
            let wire_type = tag & 0x07;
            
            if wire_type == 2 { // Length-delimited
                if let Ok(len) = input.read_raw_varint32() {
                    if let Ok(bytes) = input.read_raw_bytes(len) {
                        // Tag 10 = field 1 (detection)
                        if tag == 10 || field_num == 1 {
                            if let Ok(detection) = SSL_DetectionFrame::parse_from_bytes(&bytes) {
                                return Some(detection);
                            }
                        }
                    }
                }
            } else if wire_type == 0 {
                let _ = input.read_raw_varint64(); // Skip varint
            } else if wire_type == 1 {
                let _ = input.read_fixed64(); // Skip fixed64
            } else if wire_type == 5 {
                let _ = input.read_fixed32(); // Skip fixed32
            }
        }
        None
    }

    /// Procesa pelota del protocolo FIRA (coordenadas ya en metros).
    async fn process_ball_fira(&mut self, sender: &mpsc::Sender<VisionEvent>, status_tx: &mpsc::Sender<StatusUpdate>, ball: &FiraBall, dt: f32) -> Result<(), Box<dyn Error>> {
        let x_m = ball.x;
        let y_m = ball.y;
        let dt_f64 = dt as f64;

        let (xf_m, yf_m, vx, vy) = if self.tracker_enabled.load(Ordering::Relaxed) {
            let (xf, yf, _thetaf, vx, vy, _omega) = self.tracker.track(-1, -1, x_m, y_m, 0.0, dt_f64);
            (xf, yf, vx, vy)
        } else {
            (x_m, y_m, 0.0, 0.0)
        };

        let xf_mm = (xf_m * 1000.0) as f32;
        let yf_mm = (yf_m * 1000.0) as f32;

        let _ = status_tx.send(StatusUpdate::BallPosition(Vec2::new(xf_mm, yf_mm))).await;

        let event = VisionEvent::Ball(BallData {
            position: Vec2::new(xf_m as f32, yf_m as f32),
            velocity: Vec2::new(vx as f32, vy as f32),
        });
        if sender.send(event).await.is_err() {
            return Err("Channel closed".into());
        }
        Ok(())
    }

    /// Procesa robot del protocolo FIRA (coordenadas ya en metros).
    async fn process_robot_fira(&mut self, sender: &mpsc::Sender<VisionEvent>, status_tx: &mpsc::Sender<StatusUpdate>, robot: &FiraRobot, team: i32, dt: f32) -> Result<(), Box<dyn Error>> {
        let id = robot.robot_id;
        let x_m = robot.x;
        let y_m = robot.y;
        let theta = robot.orientation;
        let dt_f64 = dt as f64;

        let (xf_m, yf_m, thetaf, vx, vy, omega) = if self.tracker_enabled.load(Ordering::Relaxed) {
            self.tracker.track(team, id as i32, x_m, y_m, theta, dt_f64)
        } else {
            (x_m, y_m, theta, 0.0, 0.0, 0.0)
        };

        let xf_mm = (xf_m * 1000.0) as f32;
        let yf_mm = (yf_m * 1000.0) as f32;

        let _ = status_tx.send(StatusUpdate::RobotPosition(id, team as u32, Vec2::new(xf_mm, yf_mm), thetaf as f32)).await;

        let event = VisionEvent::Robot(RobotData {
            id,
            team: team as u32,
            position: Vec2::new(xf_m as f32, yf_m as f32),
            orientation: thetaf as f32,
            velocity: Vec2::new(vx as f32, vy as f32),
            angular_velocity: omega as f32,
        });
        sender.send(event).await?;
        Ok(())
    }

    // Helper for Balls (SSL Vision, coordenadas en mm)
    async fn process_ball(&mut self, sender: &mpsc::Sender<VisionEvent>, status_tx: &mpsc::Sender<StatusUpdate>, ball: &SSL_DetectionBall, dt: f32) -> Result<(), Box<dyn Error>> {
        // SSL Vision coordinates are in millimeters
        let raw_x = ball.x(); 
        let raw_y = ball.y(); 
        
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
        match status_tx.send(StatusUpdate::RobotPosition(id, team as u32, Vec2::new(xf_mm, yf_mm), thetaf as f32)).await {
            Ok(_) => {
                if VISION_LOG_EVERY_BLUE_ROBOT_TO_GUI && id < 3 && team == 0 {
                    eprintln!("[Vision] ✓ Enviada posición robot azul ID={} a GUI: pos=({:.1}, {:.1}) mm, orientación={:.2} rad", 
                             id, xf_mm, yf_mm, thetaf);
                }
            }
            Err(e) => {
                eprintln!("[Vision] ✗ Error enviando posición de robot ID={} a GUI: {}", id, e);
            }
        }

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