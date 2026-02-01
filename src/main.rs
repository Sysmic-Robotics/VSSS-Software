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
use world::{World, RobotState};
use std::time::Duration;
use std::sync::{Arc, atomic::{AtomicBool, Ordering}};

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
                    eprintln!("[Main] Radio inicializado exitosamente para FIRASim en 127.0.0.1:20011");
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
            
            // Tarea de prueba: enviar comandos periódicos a un robot para verificar comunicación
            let radio_test = radio.clone();
            let status_tx_for_test = status_tx.clone();
            let vision_tx_for_test = vision_tx.clone();
            tokio::spawn(async move {
                eprintln!("[Main] ========================================");
                eprintln!("[Main] Iniciando tarea de prueba de comandos...");
                eprintln!("[Main] Enviando comandos a robots azules IDs 0-2 cada segundo");
                eprintln!("[Main] Verifica en FIRASim si los robots azules se mueven");
                eprintln!("[Main] ========================================");
                let mut counter = 0u64;
                let mut interval = tokio::time::interval(Duration::from_millis(100)); // 10 Hz
                
                loop {
                    interval.tick().await;
                    counter += 1;
                    
                        // Enviar comandos de movimiento circular para hacer los robots visibles
                        // Esto ayuda a detectar si FIRASim está procesando comandos
                        if counter % 10 == 0 {
                            let mut radio_guard = radio_test.lock().await;
                            
                            // Crear comandos para todos los robots habilitados (0-2 para VSS)
                            // Usar movimiento circular para que sea más obvio
                            use motion::MotionCommand;
                            let t = (counter as f32 * 0.1) * std::f32::consts::PI / 2.0; // Varía cada segundo
                            
                            for robot_id in 0..3 {
                                // Cada robot se mueve en un patrón diferente
                                let (vx, vy, omega) = match robot_id {
                                    0 => (t.cos() * 0.3, t.sin() * 0.3, 0.0),  // Círculo
                                    1 => (0.3, 0.0, 0.5),                        // Línea recta + rotación
                                    2 => (0.0, 0.3, -0.5),                       // Movimiento lateral + rotación
                                    _ => (0.0, 0.0, 0.0),
                                };
                                
                                let test_cmd = MotionCommand {
                                    id: robot_id,
                                    team: 0, // Azul
                                    vx: vx as f64,
                                    vy: vy as f64,
                                    omega,
                                    orientation: 0.0,
                                };
                                
                                radio_guard.add_motion_command(test_cmd);
                            }
                            
                            match radio_guard.send_commands().await {
                                Ok(_) => {
                                    if counter % 60 == 0 { // Log cada 6 segundos
                                        eprintln!("[Main] ✓ Comandos enviados: robots azules IDs 0-2 con movimientos variados (contador: {})", counter);
                                        eprintln!("[Main] ℹ Verifica en FIRASim que los robots se están moviendo");
                                    }
                                }
                                Err(e) => {
                                    eprintln!("[Main] ✗ Error enviando comandos de prueba: {}", e);
                                }
                            }
                        }
                }
            });
            
            // ========== PRUEBA TEMPORAL: Robot 0 persigue la pelota ==========
            // Poner a `true` para que solo el robot 0 (azul) persiga la pelota con move_direct.
            // Poner a `false` (o eliminar este bloque) para volver al comportamiento normal
            // (todos los robots azules van hacia la pelota con path planner).
            const ENABLE_ROBOT_0_CHASE_BALL_TEST: bool = true;
            // =================================================================

            // Spawn a task to periodically compute motion commands and send them
            let world_for_motion = world_clone.clone();
            let radio_for_motion = radio.clone();
            let status_tx_for_motion = status_tx.clone();
            let vision_tx_for_motion = vision_tx.clone();
            tokio::spawn(async move {
                let mut interval = tokio::time::interval(Duration::from_millis(16)); // ~60 FPS
                loop {
                    interval.tick().await;

                    let (ball_pos, robots_data): (glam::Vec2, Vec<RobotState>) = {
                        let world = world_for_motion.read().await;
                        let ball_pos = world.get_ball_state().position;
                        let robots_data: Vec<_> = world.get_blue_team_active()
                            .into_iter()
                            .cloned()
                            .collect();
                        (ball_pos, robots_data)
                    };

                    if robots_data.is_empty() {
                        continue;
                    }

                    let mut radio_guard = radio_for_motion.lock().await;
                    let mut commands_added = 0u32;

                    if ENABLE_ROBOT_0_CHASE_BALL_TEST {
                        // --- Modo prueba: solo robot 0 (azul) persigue la pelota ---
                        if let Some(robot_0) = robots_data.iter().find(|r| r.id == 0) {
                            let motion_cmd = motion.move_direct(robot_0, ball_pos);
                            radio_guard.add_motion_command(motion_cmd);
                            commands_added += 1;
                        }
                    } else {
                        // --- Comportamiento normal: todos los robots azules van a la pelota ---
                        for robot_state in robots_data {
                            if robot_state.active {
                                let motion_cmd = {
                                    let world = world_for_motion.read().await;
                                    motion.move_to(&robot_state, ball_pos, &world)
                                };
                                radio_guard.add_motion_command(motion_cmd);
                                commands_added += 1;
                            }
                        }
                    }

                    if commands_added > 0 {
                        let _ = radio_guard.send_commands().await;
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