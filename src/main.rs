mod protos;
mod vision;
mod tracker;
mod world;
mod motion;
mod radio;
mod skills;
mod tactics;
mod plays;
mod coach;
#[path = "GUI/mod.rs"]
mod gui;

use tokio::sync::{mpsc, Mutex as TokioMutex, RwLock as TokioRwLock};
use vision::{Vision, VisionEvent, BallData, RobotData};
use gui::ConfigUpdate;
use world::World;
use std::collections::HashMap;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use std::sync::{Arc, atomic::{AtomicBool, AtomicU64, Ordering}};

// ====== Flags de diagnóstico y constantes de campo ======
const DEBUG_MOTION_LINE_STEP_TARGET: bool = false;
const DEBUG_MOTION_LINE_STEP_X: f32 = 0.20;
const DEBUG_MOTION_LINE_STEP_Y: f32 = -0.10;
const DEBUG_MOTION_ROTATE_IN_PLACE: bool = false;
const DEBUG_MOTION_ROTATE_OMEGA: f64 = 4.8;
// Goals del campo VSS/FIRA (metros). Azul ataca al lado positivo X.
const DEBUG_MOTION_CAPTURE_GOAL_X: f32 = 0.75;
const DEBUG_MOTION_CAPTURE_GOAL_Y: f32 = 0.0;
/// true  → CoachPlay(RuleBasedCoach): pipeline RL-ready, comportamiento equivalente a StandardPlay
/// false → StandardPlay: lógica directa de tácticas, sin pasar por Coach/Observation
const USE_COACH_PLAY: bool = false;
// Para afinar kp/ki/kd del ChaseSkill, editar los campos en skills/mod.rs: ChaseSkill::new().
const DEBUG_MOTION_LOG_POSITION_ERROR: bool = false;
const DEBUG_MOTION_LOG_CMD_VS_REAL: bool = false;
const DEBUG_MOTION_TRACK_LATENCY: bool = false;
const DEBUG_MOTION_TRACK_COUNTS: bool = false;
const DEBUG_MOTION_LOG_KPI: bool = false;
/// Una línea por segundo: balón, robots azules (posición/θ/velocidad + comando), rivales amarillos.
/// Actívalo para auditar el campo sin mirar la GUI (logs legibles).
const DEBUG_FIELD_AUDIT: bool = true;
// ========================================================

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
            
            // Spawn a task to periodically compute motion commands and send them
            let world_for_motion = world_clone.clone();
            let radio_for_motion = radio.clone();
            let command_timestamps_for_motion = command_timestamps.clone();
            let commands_sent_for_motion = commands_sent_counter.clone();
            let updates_received_for_motion = updates_received_counter.clone();
            // Play activo: controla los 3 robots del equipo azul.
            // USE_COACH_PLAY=false → StandardPlay (tácticas directas, comportamiento actual)
            // USE_COACH_PLAY=true  → CoachPlay(RuleBasedCoach) (pipeline RL-ready)
            let attack_goal = glam::Vec2::new(DEBUG_MOTION_CAPTURE_GOAL_X, DEBUG_MOTION_CAPTURE_GOAL_Y);
            let own_goal = glam::Vec2::new(-DEBUG_MOTION_CAPTURE_GOAL_X, DEBUG_MOTION_CAPTURE_GOAL_Y);
            let mut active_play: Box<dyn plays::Play> = if USE_COACH_PLAY {
                Box::new(plays::CoachPlay::new(
                    Box::new(coach::RuleBasedCoach::new(attack_goal, own_goal)),
                    0, // equipo azul
                ))
            } else {
                Box::new(plays::StandardPlay::new(attack_goal, own_goal))
            };

            tokio::spawn(async move {
                let mut interval = tokio::time::interval(Duration::from_millis(16)); // ~60 FPS
                static LAST_COUNT_LOG_SEC: std::sync::atomic::AtomicU64 =
                    std::sync::atomic::AtomicU64::new(0);
                static LAST_KPI_LOG_SEC: std::sync::atomic::AtomicU64 =
                    std::sync::atomic::AtomicU64::new(0);
                static LAST_FIELD_AUDIT_SEC: AtomicU64 = AtomicU64::new(0);
                let mut field_audit_t0: Option<Instant> = None;
                loop {
                    interval.tick().await;

                    let (tick_target, robot_snapshot, computed_commands): (
                        glam::Vec2,
                        HashMap<i32, (glam::Vec2, glam::Vec2, f64)>,
                        Vec<motion::MotionCommand>,
                    ) = {
                        let world = world_for_motion.read().await;
                        let ball_pos = if DEBUG_MOTION_LINE_STEP_TARGET {
                            glam::Vec2::new(DEBUG_MOTION_LINE_STEP_X, DEBUG_MOTION_LINE_STEP_Y)
                        } else {
                            world.get_ball_state().position
                        };
                        let robot_snapshot = world.get_blue_team_active()
                            .iter()
                            .map(|robot| {
                                (
                                    robot.id,
                                    (robot.position, robot.velocity, robot.orientation),
                                )
                            })
                            .collect::<HashMap<_, _>>();

                        let commands = if DEBUG_MOTION_ROTATE_IN_PLACE {
                            world.get_blue_team_active()
                                .iter()
                                .filter(|r| r.id == 0)
                                .map(|r| motion::MotionCommand {
                                    id: r.id,
                                    team: r.team,
                                    vx: 0.0,
                                    vy: 0.0,
                                    omega: DEBUG_MOTION_ROTATE_OMEGA,
                                    orientation: r.orientation,
                                })
                                .collect()
                        } else {
                            active_play.tick(&world, &motion)
                        };

                        if DEBUG_FIELD_AUDIT {
                            let now_sec = SystemTime::now()
                                .duration_since(UNIX_EPOCH)
                                .unwrap_or_default()
                                .as_secs();
                            let prev_sec = LAST_FIELD_AUDIT_SEC.swap(now_sec, Ordering::Relaxed);
                            if prev_sec != now_sec {
                                let t0 = field_audit_t0.get_or_insert_with(Instant::now);
                                let t_elapsed = t0.elapsed().as_secs_f64();
                                let ball_st = world.get_ball_state();
                                let bp = if DEBUG_MOTION_LINE_STEP_TARGET {
                                    glam::Vec2::new(DEBUG_MOTION_LINE_STEP_X, DEBUG_MOTION_LINE_STEP_Y)
                                } else {
                                    ball_st.position
                                };
                                let bv = ball_st.velocity;
                                let mut y_parts: Vec<String> = world
                                    .get_yellow_team_active()
                                    .iter()
                                    .map(|r| {
                                        format!(
                                            "Y{}({:.2},{:.2})",
                                            r.id, r.position.x, r.position.y
                                        )
                                    })
                                    .collect();
                                y_parts.sort();
                                let yellow_s = if y_parts.is_empty() {
                                    String::new()
                                } else {
                                    format!(" | {}", y_parts.join(" "))
                                };

                                let mut ids: Vec<i32> = commands.iter().map(|c| c.id).collect();
                                ids.sort_unstable();
                                let mut blue_s = String::new();
                                for id in ids {
                                    if let Some(cmd) = commands.iter().find(|c| c.id == id) {
                                        let piece = if let Some((p, v, th)) =
                                            robot_snapshot.get(&id)
                                        {
                                            format!(
                                                " B{}:pos({:.2},{:.2}) θ={:.0}° v={:.2} cmd({:.2},{:.2}) ω={:.2}",
                                                id,
                                                p.x,
                                                p.y,
                                                th.to_degrees(),
                                                v.length(),
                                                cmd.vx,
                                                cmd.vy,
                                                cmd.omega
                                            )
                                        } else {
                                            format!(
                                                " B{}:pos(?) cmd({:.2},{:.2}) ω={:.2}",
                                                id, cmd.vx, cmd.vy, cmd.omega
                                            )
                                        };
                                        blue_s.push_str(&piece);
                                    }
                                }

                                eprintln!(
                                    "[FieldAudit] t={:.1}s ball=({:.3},{:.3})m v_ball=({:.3},{:.3}){}{}",
                                    t_elapsed,
                                    bp.x,
                                    bp.y,
                                    bv.x,
                                    bv.y,
                                    blue_s,
                                    yellow_s
                                );
                            }
                        }

                        (ball_pos, robot_snapshot, commands)
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
                            if let Some((pos, vel, _)) = robot_snapshot.get(&cmd.id) {
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