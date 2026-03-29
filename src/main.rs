mod protos;
mod vision;
mod tracker;
mod world;
mod motion;
mod radio;
#[path = "GUI/mod.rs"]
mod gui;

use tokio::sync::{mpsc, Mutex as TokioMutex, RwLock as TokioRwLock};
use vision::{Vision, VisionEvent, BallData, RobotData};
use gui::ConfigUpdate;
use world::World;
use std::collections::HashMap;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use std::sync::{Arc, atomic::{AtomicBool, Ordering}};

// ====== Instrumentación temporal de movimiento (fácil de borrar) ======
const DEBUG_MOTION_LINE_STEP_TARGET: bool = false;
const DEBUG_MOTION_LINE_STEP_X: f32 = 0.20;
const DEBUG_MOTION_LINE_STEP_Y: f32 = -0.10;
const DEBUG_MOTION_ROTATE_IN_PLACE: bool = false;
const DEBUG_MOTION_ROTATE_OMEGA: f64 = 4.8;
const DEBUG_MOTION_CHASE_FACE_KP: f64 = 1.8;
const DEBUG_MOTION_CHASE_FACE_KI: f64 = 0.0;
const DEBUG_MOTION_CHASE_FACE_KD: f64 = 0.05;
const DEBUG_MOTION_CAPTURE_GOAL_X: f32 = 0.75;
const DEBUG_MOTION_CAPTURE_GOAL_Y: f32 = 0.0;
const DEBUG_MOTION_CAPTURE_STAGING_OFFSET_M: f32 = 0.16;
const DEBUG_MOTION_CAPTURE_STAGING_TOL_M: f32 = 0.08;
const DEBUG_MOTION_CAPTURE_HEADING_TOL_RAD: f64 = 0.28;
const DEBUG_MOTION_LOG_POSITION_ERROR: bool = false;
const DEBUG_MOTION_LOG_CMD_VS_REAL: bool = false;
const DEBUG_MOTION_TRACK_LATENCY: bool = false;
const DEBUG_MOTION_TRACK_COUNTS: bool = true;  // un log/seg, útil para verificar que se envían comandos
const DEBUG_MOTION_LOG_KPI: bool = true;        // un log/seg, útil para tuning de velocidad
// ======================================================================

fn main() {
    // Create channels
    let (vision_tx, mut vision_rx) = mpsc::channel(100); 
    let (status_tx, status_rx) = mpsc::channel(100);
    let (config_tx, mut config_rx) = mpsc::channel(10);

    // Configuración para FIRASim (según configuración del simulador)
    let vision_ip = "224.0.0.1".to_string();  // FIRASim usa 224.0.0.1 para visión multicast
    let vision_port = 10002;                  // FIRASim usa puerto 10002 para visión
    
    let gui_ip = vision_ip.clone();
    let gui_port = vision_port;
    let gui_config_tx = config_tx.clone();

    // Create World instance (3 robots per team for VSS)
    // Usar TokioRwLock para acceso async-friendly
    let world = Arc::new(TokioRwLock::new(World::new(3, 3)));
    let world_clone = world.clone();
    let command_timestamps: Arc<TokioMutex<HashMap<(i32, i32), Instant>>> =
        Arc::new(TokioMutex::new(HashMap::new()));
    let commands_sent_counter = Arc::new(std::sync::atomic::AtomicU64::new(0));
    let updates_received_counter = Arc::new(std::sync::atomic::AtomicU64::new(0));
    
    // Create shared flag for tracker state
    let tracker_enabled = Arc::new(AtomicBool::new(true)); // Habilitado por defecto
    let tracker_enabled_clone = tracker_enabled.clone();

    // Spawn a background thread to run the vision system
    std::thread::spawn(move || {
        let rt = tokio::runtime::Runtime::new().unwrap();
        rt.block_on(async {
            // Clonar canales para la tarea de visión (el primer spawn los consume)
            let vision_tx_for_vision = vision_tx.clone();
            let status_tx_for_vision = status_tx.clone();

            // Spawn a task to handle configuration updates and restart vision
            tokio::spawn(async move {
                let mut current_ip = vision_ip;
                let mut current_port = vision_port;
                let tracker_enabled = tracker_enabled_clone;
                let mut vision_tx = vision_tx_for_vision;
                let mut status_tx = status_tx_for_vision;

                let mut vision_handle: Option<tokio::task::JoinHandle<()>> = None;

                loop {
                    // Solo crear un nuevo sistema de visión si no existe uno corriendo
                    if vision_handle.is_none() || vision_handle.as_ref().unwrap().is_finished() {
                        eprintln!("[Main] Iniciando sistema de visión en {}:{}", current_ip, current_port);
                        let mut vision_system = Vision::new(current_ip.clone(), current_port, tracker_enabled.clone());
                        let vision_tx_clone = vision_tx.clone();
                        let status_tx_clone = status_tx.clone();
                        
                        // Spawn vision task
                        vision_handle = Some(tokio::spawn(async move {
                            match vision_system.run(vision_tx_clone, status_tx_clone).await {
                                Ok(_) => {
                                    eprintln!("[Main] Sistema de visión terminó normalmente");
                                }
                                Err(e) => {
                                    eprintln!("[Main] Error en sistema de visión: {}", e);
                                }
                            }
                        }));
                    }

                    // Wait for config update or vision task to complete
                    if let Some(ref mut handle) = vision_handle {
                        // Guardar el estado antes del select para evitar problemas de borrow
                        let handle_finished = handle.is_finished();
                        
                        tokio::select! {
                            Some(config) = config_rx.recv() => {
                                // Update configuration
                                match config {
                                    ConfigUpdate::ChangeIpPort(new_ip, new_port) => {
                                        eprintln!("[Main] Cambiando configuración a {}:{}", new_ip, new_port);
                                        current_ip = new_ip;
                                        current_port = new_port;
                                        // Abort and loop will restart with new config
                                        // Usar vision_handle directamente después del select
                                        if let Some(ref mut h) = vision_handle {
                                            if !h.is_finished() {
                                                h.abort();
                                            }
                                        }
                                        vision_handle = None; // Forzar recreación en la siguiente iteración
                                    }
                                    ConfigUpdate::ToggleTracker(enabled) => {
                                        tracker_enabled.store(enabled, Ordering::Relaxed);
                                        eprintln!("[Main] Tracker {}", if enabled { "habilitado" } else { "deshabilitado" });
                                        // No reiniciar el task, el flag se lee en tiempo real
                                    }
                                }
                            }
                            _ = handle => {
                                // Vision task completed unexpectedly
                                eprintln!("[Main] Sistema de visión terminó inesperadamente, reiniciando en 1 segundo...");
                                vision_handle = None;
                                tokio::time::sleep(Duration::from_secs(1)).await;
                            }
                        }
                    } else {
                        // Si no hay handle, solo esperar por cambios de configuración
                        if let Some(config) = config_rx.recv().await {
                            match config {
                                ConfigUpdate::ChangeIpPort(new_ip, new_port) => {
                                    eprintln!("[Main] Cambiando configuración a {}:{}", new_ip, new_port);
                                    current_ip = new_ip;
                                    current_port = new_port;
                                }
                                ConfigUpdate::ToggleTracker(enabled) => {
                                    tracker_enabled.store(enabled, Ordering::Relaxed);
                                    eprintln!("[Main] Tracker {}", if enabled { "habilitado" } else { "deshabilitado" });
                                }
                            }
                        }
                    }
                }
            });

            // Spawn a background task to consume vision events and update World
            let world_for_vision = world_clone.clone();
            let command_timestamps_for_vision = command_timestamps.clone();
            let updates_received_for_vision = updates_received_counter.clone();
            tokio::spawn(async move {
                while let Some(event) = vision_rx.recv().await {
                    let mut world = world_for_vision.write().await;
                    
                    match event {
                        VisionEvent::Robot(robot_data) => {
                            world.update_robot(
                                robot_data.id as i32,
                                robot_data.team as i32,
                                robot_data.position,
                                robot_data.orientation as f64,
                                robot_data.velocity,
                                robot_data.angular_velocity as f64,
                            );
                            if DEBUG_MOTION_TRACK_LATENCY {
                                let key = (robot_data.team as i32, robot_data.id as i32);
                                let mut map = command_timestamps_for_vision.lock().await;
                                if let Some(sent_at) = map.remove(&key) {
                                    let latency_ms = sent_at.elapsed().as_secs_f64() * 1000.0;
                                    if latency_ms > 1.0 {
                                        eprintln!(
                                            "[MotionDebug] latencia cmd->vision robot={} team={} {:.2} ms",
                                            robot_data.id, robot_data.team, latency_ms
                                        );
                                    }
                                }
                            }
                            if DEBUG_MOTION_TRACK_COUNTS {
                                updates_received_for_vision.fetch_add(1, Ordering::Relaxed);
                            }
                        }
                        VisionEvent::Ball(ball_data) => {
                            world.update_ball(ball_data.position, ball_data.velocity);
                        }
                    }
                }
            });

            // Spawn a task to periodically update World (mark inactive robots)
            let world_for_update = world_clone.clone();
            tokio::spawn(async move {
                let mut interval = tokio::time::interval(Duration::from_millis(100));
                loop {
                    interval.tick().await;
                    let mut world = world_for_update.write().await;
                    world.update();
                }
            });

            // Initialize Motion and Radio systems
            // FIRASim escucha comandos en puerto 20011 (según configuración del simulador)
            // NOTA: Si los robots no aparecen, verifica el puerto en la configuración de FIRASim
            // Puede ser 20011 o 10300 dependiendo de la versión/configuración
            let motion = motion::Motion::new();
            let radio = match radio::Radio::new(false, radio::SimulatorType::FIRASim, "127.0.0.1", 20011).await {
                Ok(r) => {
                    eprintln!("[Main] Radio inicializado para FIRASim en 127.0.0.1:20011");
                    eprintln!("[Main] Asegúrate de que FIRASim esté en ejecución y escuchando en el puerto 20011 (actuador) para que los robots se muevan.");
                    Arc::new(TokioMutex::new(r))
                },
                Err(e) => {
                    eprintln!("[Main] Error inicializando radio: {}", e);
                    // Skip motion/radio integration if radio creation fails
                    // Keep the runtime alive
                    loop {
                        tokio::time::sleep(Duration::from_secs(1)).await;
                    }
                }
            };
            
            // Crear robots en FIRASim antes de enviar comandos
            // VSS tiene 3 robots por equipo (IDs 0-2)
            // Esto asegura que los robots estén presentes en el simulador
            eprintln!("[Main] Creando robots en FIRASim (VSS: 3 robots por equipo)...");
            let robot_ids: Vec<(u32, u32)> = vec![
                (0, 0), // Robot azul ID 0
                (1, 0), // Robot azul ID 1
                (2, 0), // Robot azul ID 2
            ];
            {
                let radio_guard = radio.lock().await;
                match radio_guard.create_robots(&robot_ids).await {
                    Ok(_) => {
                        eprintln!("[Main] ✓ Robots creados exitosamente en FIRASim");
                    }
                    Err(e) => {
                        eprintln!("[Main] ⚠ Error creando robots (puede que ya existan): {}", e);
                        eprintln!("[Main] Continuando de todas formas...");
                    }
                }
            }

            // Limpieza inicial: neutraliza cualquier comando previo latente en FIRASim.
            {
                let mut radio_guard = radio.lock().await;
                for robot_id in 0..3 {
                    radio_guard.add_motion_command(motion::MotionCommand {
                        id: robot_id,
                        team: 0,
                        vx: 0.0,
                        vy: 0.0,
                        omega: 0.0,
                        orientation: 0.0,
                    });
                }
                if let Err(err) = radio_guard.send_commands().await {
                    eprintln!("[Main] Error enviando comandos neutros iniciales: {}", err);
                }
            }
            
            // ========== PRUEBA TEMPORAL: Robot 0 persigue la pelota ==========
            // Poner a `true` para que solo el robot 0 (azul) persiga la pelota con move_direct.
            // Poner a `false` para tarea de prueba circular (robots 0-2) o comportamiento normal.
            const ENABLE_ROBOT_0_CHASE_BALL_TEST: bool = true;
            // =================================================================

            // Tarea de prueba solo cuando NO está activo el modo "robot 0 persigue pelota"
            if !ENABLE_ROBOT_0_CHASE_BALL_TEST {
                let radio_test = radio.clone();
                tokio::spawn(async move {
                    eprintln!("[Main] Tarea de prueba de comandos activa (robots 0-2, movimiento circular)");
                    let mut counter = 0u64;
                    let mut interval = tokio::time::interval(Duration::from_millis(100));
                    loop {
                        interval.tick().await;
                        counter += 1;
                        if counter % 10 == 0 {
                            let mut radio_guard = radio_test.lock().await;
                            use motion::MotionCommand;
                            let t = (counter as f32 * 0.1) * std::f32::consts::PI / 2.0;
                            for robot_id in 0..3 {
                                let (vx, vy, omega) = match robot_id {
                                    0 => (t.cos() * 0.3, t.sin() * 0.3, 0.0),
                                    1 => (0.3, 0.0, 0.5),
                                    2 => (0.0, 0.3, -0.5),
                                    _ => (0.0, 0.0, 0.0),
                                };
                                radio_guard.add_motion_command(MotionCommand {
                                    id: robot_id,
                                    team: 0,
                                    vx: vx as f64,
                                    vy: vy as f64,
                                    omega,
                                    orientation: 0.0,
                                });
                            }
                            let _ = radio_guard.send_commands().await;
                        }
                    }
                });
            }

            // Spawn a task to periodically compute motion commands and send them
            let world_for_motion = world_clone.clone();
            let radio_for_motion = radio.clone();
            let command_timestamps_for_motion = command_timestamps.clone();
            let commands_sent_for_motion = commands_sent_counter.clone();
            let updates_received_for_motion = updates_received_counter.clone();
            tokio::spawn(async move {
                let mut interval = tokio::time::interval(Duration::from_millis(16)); // ~60 FPS
                static LAST_COUNT_LOG_SEC: std::sync::atomic::AtomicU64 =
                    std::sync::atomic::AtomicU64::new(0);
                static LAST_KPI_LOG_SEC: std::sync::atomic::AtomicU64 =
                    std::sync::atomic::AtomicU64::new(0);
                // Histéresis de fase: evita oscilar entre APPROACH y CAPTURA en el borde del umbral.
                let mut in_captura = false;
                loop {
                    interval.tick().await;

                    let (tick_target, robot_snapshot, computed_commands): (
                        glam::Vec2,
                        HashMap<i32, (glam::Vec2, glam::Vec2)>,
                        Vec<motion::MotionCommand>,
                    ) = {
                        let world = world_for_motion.read().await;
                        let ball_pos = if DEBUG_MOTION_LINE_STEP_TARGET {
                            glam::Vec2::new(DEBUG_MOTION_LINE_STEP_X, DEBUG_MOTION_LINE_STEP_Y)
                        } else {
                            world.get_ball_state().position
                        };
                        let robots_data: Vec<_> = world.get_blue_team_active()
                            .into_iter()
                            .cloned()
                            .collect();
                        let robot_snapshot = robots_data
                            .iter()
                            .map(|robot| (robot.id, (robot.position, robot.velocity)))
                            .collect::<HashMap<_, _>>();

                        if robots_data.is_empty() {
                            (ball_pos, robot_snapshot, Vec::new())
                        } else if DEBUG_MOTION_ROTATE_IN_PLACE {
                            if let Some(robot_0) = robots_data.iter().find(|r| r.id == 0) {
                                (
                                    ball_pos,
                                    robot_snapshot,
                                    vec![motion::MotionCommand {
                                        id: robot_0.id,
                                        team: robot_0.team,
                                        vx: 0.0,
                                        vy: 0.0,
                                        omega: DEBUG_MOTION_ROTATE_OMEGA,
                                        orientation: robot_0.orientation,
                                    }],
                                )
                            } else {
                                (ball_pos, robot_snapshot, Vec::new())
                            }
                        } else if ENABLE_ROBOT_0_CHASE_BALL_TEST {
                            // --- Modo prueba: solo robot 0 (azul) persigue la pelota ---
                            if let Some(robot_0) = robots_data.iter().find(|r| r.id == 0) {
                                let goal_pos = glam::Vec2::new(
                                    DEBUG_MOTION_CAPTURE_GOAL_X,
                                    DEBUG_MOTION_CAPTURE_GOAL_Y
                                );
                                let ball_to_goal = (goal_pos - ball_pos).normalize_or_zero();
                                let staging_point = ball_pos - (ball_to_goal * DEBUG_MOTION_CAPTURE_STAGING_OFFSET_M);
                                let to_goal = goal_pos - robot_0.position;

                                let dist_staging   = (staging_point - robot_0.position).length();
                                let dot_robot_ball = (robot_0.position - ball_pos).dot(goal_pos - ball_pos);
                                let behind_ball    = dot_robot_ball < 0.02;
                                let in_front_of_ball = dot_robot_ball > 0.05;
                                let heading_goal   = to_goal.y.atan2(to_goal.x) as f64;

                                // Histéresis: entrar cuando detrás + cerca de staging.
                                // Salir SOLO cuando claramente por delante de la pelota (5cm).
                                // No salir por dist_staging: el robot debe empujar libremente.
                                if behind_ball && dist_staging <= DEBUG_MOTION_CAPTURE_STAGING_TOL_M {
                                    in_captura = true;
                                } else if in_front_of_ball {
                                    in_captura = false;
                                }

                                // Approach: move_to con path planning (pelota es obstáculo → rodea).
                                // Captura: move_direct con target 12cm PASADO la pelota hacia el goal.
                                // Apuntar más allá evita que move_direct se detenga antes de empujar.
                                let push_target = ball_pos + ball_to_goal * 0.12;
                                let mut cmd = if in_captura {
                                    motion.move_direct(robot_0, push_target)
                                } else {
                                    motion.move_to(robot_0, staging_point, &world)
                                };

                                // Approach: mirar hacia staging_point (heading alineado con movimiento).
                                // Captura: mirar hacia la pelota — staging está sobre la línea
                                // ball-goal, entonces face_to(ball) ≈ face_to_angle(goal) desde
                                // staging, y además sigue al ball cuando se mueve (pivote real).
                                let face_cmd = if in_captura {
                                    motion.face_to(robot_0, ball_pos,
                                        DEBUG_MOTION_CHASE_FACE_KP,
                                        DEBUG_MOTION_CHASE_FACE_KI,
                                        DEBUG_MOTION_CHASE_FACE_KD)
                                } else {
                                    motion.face_to(robot_0, staging_point,
                                        DEBUG_MOTION_CHASE_FACE_KP,
                                        DEBUG_MOTION_CHASE_FACE_KI,
                                        DEBUG_MOTION_CHASE_FACE_KD)
                                };
                                cmd.omega = face_cmd.omega;
                                (ball_pos, robot_snapshot, vec![cmd])
                            } else {
                                in_captura = false;
                                (ball_pos, robot_snapshot, Vec::new())
                            }
                        } else {
                            // --- Comportamiento normal: todos los robots azules van a la pelota ---
                            let commands = robots_data
                                .iter()
                                .filter(|robot_state| robot_state.active)
                                .map(|robot_state| motion.move_to(robot_state, ball_pos, &world))
                                .collect();
                            (ball_pos, robot_snapshot, commands)
                        }
                    };

                    if computed_commands.is_empty() {
                        continue;
                    }
                    let computed_count = computed_commands.len() as u64;
                    if DEBUG_MOTION_LOG_KPI {
                        let now_sec = SystemTime::now()
                            .duration_since(UNIX_EPOCH)
                            .unwrap_or_default()
                            .as_secs();
                        let prev_sec = LAST_KPI_LOG_SEC.swap(now_sec, Ordering::Relaxed);
                        if prev_sec != now_sec {
                            let kpi = motion::summarize_commands(&computed_commands);
                            eprintln!(
                                "[MotionDebug] KPI mean_speed={:.3} peak_speed={:.3} mean_abs_omega={:.3}",
                                kpi.mean_speed,
                                kpi.peak_speed,
                                kpi.mean_abs_omega
                            );
                        }
                    }

                    if DEBUG_MOTION_LOG_POSITION_ERROR || DEBUG_MOTION_LOG_CMD_VS_REAL {
                        for cmd in &computed_commands {
                            if let Some((pos, vel)) = robot_snapshot.get(&cmd.id) {
                                let pos_error = (tick_target - *pos).length();
                                if DEBUG_MOTION_LOG_POSITION_ERROR {
                                    eprintln!(
                                        "[MotionDebug] robot={} err={:.3}m target=({:.2},{:.2}) pos=({:.2},{:.2})",
                                        cmd.id, pos_error, tick_target.x, tick_target.y, pos.x, pos.y
                                    );
                                }
                                if DEBUG_MOTION_LOG_CMD_VS_REAL {
                                    let cmd_speed = (cmd.vx * cmd.vx + cmd.vy * cmd.vy).sqrt();
                                    let real_speed = vel.length() as f64;
                                    eprintln!(
                                        "[MotionDebug] robot={} cmd_speed={:.3} real_speed={:.3} omega={:.3}",
                                        cmd.id, cmd_speed, real_speed, cmd.omega
                                    );
                                }
                            }
                        }
                    }

                    let sent_at = Instant::now();
                    if DEBUG_MOTION_TRACK_LATENCY {
                        let mut map = command_timestamps_for_motion.lock().await;
                        for motion_cmd in &computed_commands {
                            map.insert((motion_cmd.team, motion_cmd.id), sent_at);
                        }
                    }
                    let mut radio_guard = radio_for_motion.lock().await;
                    for motion_cmd in computed_commands {
                        radio_guard.add_motion_command(motion_cmd);
                    }

                    if let Err(err) = radio_guard.send_commands().await {
                        eprintln!("[Main] Error enviando comandos de movimiento: {}", err);
                    } else if DEBUG_MOTION_TRACK_COUNTS {
                        commands_sent_for_motion.fetch_add(computed_count, Ordering::Relaxed);
                        let now_sec = SystemTime::now()
                            .duration_since(UNIX_EPOCH)
                            .unwrap_or_default()
                            .as_secs();
                        let prev_sec = LAST_COUNT_LOG_SEC.swap(now_sec, Ordering::Relaxed);
                        if prev_sec != now_sec {
                            let sent = commands_sent_for_motion.load(Ordering::Relaxed);
                            let recv = updates_received_for_motion.load(Ordering::Relaxed);
                            eprintln!(
                                "[MotionDebug] contador sent_ticks={} recv_updates={} ratio={:.2}",
                                sent,
                                recv,
                                if sent > 0 { recv as f64 / sent as f64 } else { 0.0 }
                            );
                        }
                    }
                }
            });

            // Keep the runtime alive
            loop {
                tokio::time::sleep(Duration::from_secs(1)).await;
            }
        });
    });

    // Run GUI (blocks until window is closed)
    // The GUI will consume status_rx directly through its subscription
    let _ = gui::run_gui(gui_ip, gui_port, gui_config_tx, status_rx);
}