// Modo base: headless por defecto.
// Con VSSL_DEBUG_GUI=1 lanza la interfaz gráfica con vectores de velocidad y targets.

mod GUI;

// ====== Parámetros ======
const OWN_TEAM: i32 = 0; // 0=azul, 1=amarillo
const ROBOT_ID: i32 = 0;
// ========================

use rustengine::motion::{Motion, MotionCommand};
use rustengine::skills::{ChaseBallSkill, Skill};
use rustengine::world::World;

fn scenario_tick(
    world: &World,
    motion: &Motion,
    chase_ball: &mut ChaseBallSkill,
) -> Vec<MotionCommand> {
    let team_robots = if OWN_TEAM == 0 {
        world.get_blue_team_active()
    } else {
        world.get_yellow_team_active()
    };
    team_robots
        .iter()
        .find(|robot| robot.id == ROBOT_ID)
        .map(|robot| vec![chase_ball.tick(robot, world, motion)])
        .unwrap_or_default()
}

use rustengine::{
    radio,
    vision::{Vision, VisionEvent, VisionSource},
};
use std::sync::{Arc, atomic::AtomicBool};
use std::time::Duration;
use tokio::sync::{Mutex as TokioMutex, RwLock as TokioRwLock, mpsc};

fn main() {
    if std::env::var("VSSL_DEBUG_GUI").unwrap_or_default() == "1" {
        run_with_gui();
    } else {
        let rt = tokio::runtime::Runtime::new().unwrap();
        rt.block_on(async_main(None));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Modo debug: GUI en el hilo principal, tokio en background
// ─────────────────────────────────────────────────────────────────────────────
fn run_with_gui() {
    let (status_tx, status_rx) = mpsc::channel(100);
    let (motion_tx, motion_rx) = mpsc::channel::<Vec<GUI::RobotMotionDebug>>(32);
    let (config_tx, _config_rx) = mpsc::channel::<GUI::ConfigUpdate>(8);

    let source = VisionSource::from_env();
    let ip = source.multicast_ip().to_string();
    let port = source.port();

    std::thread::spawn(move || {
        let rt = tokio::runtime::Runtime::new().unwrap();
        rt.block_on(async_main(Some((status_tx, motion_tx))));
    });

    GUI::run_gui(ip, port, config_tx, status_rx, motion_rx)
        .expect("GUI terminó con error");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Pipeline principal (headless o con canales para GUI)
// ─────────────────────────────────────────────────────────────────────────────
async fn async_main(
    gui_channels: Option<(
        mpsc::Sender<GUI::StatusUpdate>,
        mpsc::Sender<Vec<GUI::RobotMotionDebug>>,
    )>,
) {
    let (vision_tx, mut vision_rx) = mpsc::channel(100);
    let world = Arc::new(TokioRwLock::new(World::new(3, 3)));
    let tracker_enabled = Arc::new(AtomicBool::new(true));

    let (status_tx, motion_tx) = match gui_channels {
        Some((s, m)) => (Some(s), Some(m)),
        None => (None, None),
    };

    // Vision
    {
        let tracker_enabled = tracker_enabled.clone();
        let source = VisionSource::from_env();
        eprintln!(
            "[main] visión: {:?} ({}:{})",
            source,
            source.multicast_ip(),
            source.port()
        );
        let status_tx_vis = status_tx.clone();
        tokio::spawn(async move {
            let mut vis = Vision::new(source, tracker_enabled);
            let (dummy_tx, _) = mpsc::channel(1);
            let tx = status_tx_vis.unwrap_or(dummy_tx);
            if let Err(err) = vis.run(vision_tx, tx).await {
                eprintln!("[main] vision error: {err}");
            }
        });
    }

    // World updater desde visión
    {
        let world = world.clone();
        tokio::spawn(async move {
            while let Some(event) = vision_rx.recv().await {
                let mut w = world.write().await;
                match event {
                    VisionEvent::Robot(r) => {
                        w.update_robot(
                            r.id as i32,
                            r.team as i32,
                            r.position,
                            r.orientation as f64,
                            r.velocity,
                            r.angular_velocity as f64,
                        );
                    }
                    VisionEvent::Ball(b) => {
                        w.update_ball(b.position, b.velocity);
                    }
                }
            }
        });
    }

    // Marcar robots inactivos
    {
        let world = world.clone();
        tokio::spawn(async move {
            let mut interval = tokio::time::interval(Duration::from_millis(100));
            loop {
                interval.tick().await;
                world.write().await.update();
            }
        });
    }

    let radio = match radio::Radio::from_env().await {
        Ok(r) => Arc::new(TokioMutex::new(r)),
        Err(err) => {
            eprintln!("[main] radio error: {err}");
            return;
        }
    };

    eprintln!("[main] listo. 60 Hz. Ctrl+C para detener.");

    let motion = Motion::new();
    let mut chase_ball = ChaseBallSkill::new();
    let mut interval = tokio::time::interval(Duration::from_millis(16));

    loop {
        interval.tick().await;

        let commands = {
            let world = world.read().await;
            scenario_tick(&world, &motion, &mut chase_ball)
        };

        if commands.is_empty() {
            continue;
        }

        // Enviar debug al GUI si está activo
        if let Some(ref tx) = motion_tx {
            let updates: Vec<GUI::RobotMotionDebug> = commands
                .iter()
                .map(|cmd| GUI::RobotMotionDebug {
                    team: cmd.team as u32,
                    id: cmd.id as u32,
                    vx: cmd.vx as f32,
                    vy: cmd.vy as f32,
                    target: None, // ChaseBallSkill no expone target estático
                })
                .collect();
            let _ = tx.try_send(updates);
        }

        let mut radio = radio.lock().await;
        for cmd in commands {
            radio.add_motion_command(cmd);
        }
        if let Err(err) = radio.send_commands().await {
            eprintln!("[main] error enviando: {err}");
        }
    }
}
