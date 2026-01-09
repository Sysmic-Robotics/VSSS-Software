mod protos;
mod vision;
mod tracker;
mod world;
#[path = "GUI/mod.rs"]
mod gui;

use tokio::sync::mpsc;
use vision::{Vision, VisionEvent};
use gui::ConfigUpdate;
use world::World;
use std::time::Duration;
use std::sync::{Arc, RwLock, atomic::{AtomicBool, Ordering}};

fn main() {
    // Create channels
    let (vision_tx, mut vision_rx) = mpsc::channel(100); 
    let (status_tx, status_rx) = mpsc::channel(100);
    let (config_tx, mut config_rx) = mpsc::channel(10);

    let vision_ip = "224.5.23.2".to_string();
    let vision_port = 10020;
    
    let gui_ip = vision_ip.clone();
    let gui_port = vision_port;
    let gui_config_tx = config_tx.clone();

    // Create World instance (11 robots per team for SSL)
    let world = Arc::new(RwLock::new(World::new(11, 11)));
    let world_clone = world.clone();
    
    // Create shared flag for tracker state
    let tracker_enabled = Arc::new(AtomicBool::new(true)); // Habilitado por defecto
    let tracker_enabled_clone = tracker_enabled.clone();

    // Spawn a background thread to run the vision system
    std::thread::spawn(move || {
        let rt = tokio::runtime::Runtime::new().unwrap();
        rt.block_on(async {
            // Spawn a task to handle configuration updates and restart vision
            tokio::spawn(async move {
                let mut current_ip = vision_ip;
                let mut current_port = vision_port;
                let tracker_enabled = tracker_enabled_clone;
                
                loop {
                    let mut vision_system = Vision::new(current_ip.clone(), current_port, tracker_enabled.clone());
                    let vision_tx_clone = vision_tx.clone();
                    let status_tx_clone = status_tx.clone();
                    
                    // Spawn vision task
                    let mut vision_handle = tokio::spawn(async move {
                        let _ = vision_system.run(vision_tx_clone, status_tx_clone).await;
                    });

                    // Wait for config update or vision task to complete
                    tokio::select! {
                        Some(config) = config_rx.recv() => {
                            // Update configuration
                            match config {
                                ConfigUpdate::ChangeIpPort(new_ip, new_port) => {
                                    current_ip = new_ip;
                                    current_port = new_port;
                                    // Abort and loop will restart with new config
                                    if !vision_handle.is_finished() {
                                        vision_handle.abort();
                                    }
                                }
                                ConfigUpdate::ToggleTracker(enabled) => {
                                    tracker_enabled.store(enabled, Ordering::Relaxed);
                                    // No reiniciar el task, el flag se lee en tiempo real
                                    continue; // Continuar el loop sin reiniciar vision
                                }
                            }
                        }
                        _result = &mut vision_handle => {
                            // Vision task completed unexpectedly
                            // Add a small delay before restarting to prevent tight loop
                            tokio::time::sleep(Duration::from_millis(100)).await;
                        }
                    }
                    
                    // If we're here from config update, abort the vision task
                    if !vision_handle.is_finished() {
                        vision_handle.abort();
                    }
                }
            });

            // Spawn a background task to consume vision events and update World
            let world_for_vision = world_clone.clone();
            tokio::spawn(async move {
                while let Some(event) = vision_rx.recv().await {
                    let mut world = world_for_vision.write().unwrap();
                    
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
                    let mut world = world_for_update.write().unwrap();
                    world.update();
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