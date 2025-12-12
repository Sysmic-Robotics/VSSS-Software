mod protos;
mod vision;
mod ui;
mod field;

use tokio::sync::mpsc;
use vision::Vision;
use ui::ConfigUpdate;
use std::time::Duration;

#[tokio::main]
async fn main() {
    // Create channels
    let (vision_tx, mut vision_rx) = mpsc::channel(100); 
    let (status_tx, status_rx) = mpsc::channel(100);
    let (config_tx, mut config_rx) = mpsc::channel(10);

    let vision_ip = "224.5.23.2".to_string();
    let vision_port = 10020;
    
    let ui_ip = vision_ip.clone();
    let ui_port = vision_port;

    // Spawn a task to handle configuration updates and restart vision
    tokio::spawn(async move {
        let mut current_ip = vision_ip;
        let mut current_port = vision_port;
        
        loop {
            let mut vision_system = Vision::new(current_ip.clone(), current_port);
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
                        }
                    }
                    // Abort and loop will restart with new config
                }
                result = &mut vision_handle => {
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

    // Spawn a background task to consume vision events
    tokio::spawn(async move {
        while let Some(_event) = vision_rx.recv().await {
            // Process vision events if needed
            // For now, we just consume them
        }
    });

    // Run UI (blocks until user quits)
    let _ = ui::run_ui(status_rx, config_tx, ui_ip, ui_port).await;
}