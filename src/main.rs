// Modo base: `main` usa exactamente el mismo pipeline simple que `scenario`.
// Sin GUI, sin plays y sin lógica extra; desde aquí podemos crecer de nuevo.

// ====== Parámetros del escenario base ======
/// Equipo que controla este ejecutable (0=azul, 1=amarillo)
const OWN_TEAM: i32 = 0;
/// Robot que usamos en esta base mínima
const ROBOT_ID: i32 = 0;
// ==========================================

use rustengine::motion::{Motion, MotionCommand};
use rustengine::skills::{ChaseBallSkill, Skill};
use rustengine::world::World;

/// Base de movimiento actual: igual a `scenario`.
/// Se llama a ~60 Hz con el estado actual del mundo.
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
    vision::{Vision, VisionEvent},
};
use std::sync::{Arc, atomic::AtomicBool};
use std::time::Duration;
use tokio::sync::{Mutex as TokioMutex, RwLock as TokioRwLock, mpsc};

fn main() {
    let rt = tokio::runtime::Runtime::new().unwrap();
    rt.block_on(async_main());
}

async fn async_main() {
    let (vision_tx, mut vision_rx) = mpsc::channel(100);
    let world = Arc::new(TokioRwLock::new(World::new(3, 3)));
    let tracker_enabled = Arc::new(AtomicBool::new(true));

    // Vision
    {
        let tracker_enabled = tracker_enabled.clone();
        tokio::spawn(async move {
            let mut vis = Vision::new("224.0.0.1".to_string(), 10002, tracker_enabled);
            let (dummy_status_tx, _) = mpsc::channel(1);
            if let Err(err) = vis.run(vision_tx, dummy_status_tx).await {
                eprintln!("[main] vision error: {err}");
            }
        });
    }

    // Update world with vision events
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

    // Mark inactive robots every 100 ms
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

    let radio =
        match radio::Radio::new(false, radio::SimulatorType::FIRASim, "127.0.0.1", 20011).await {
            Ok(r) => Arc::new(TokioMutex::new(r)),
            Err(err) => {
                eprintln!("[main] radio error: {err}");
                return;
            }
        };

    eprintln!(
        "[main] conectado a FIRASim. Ejecutando la misma base que scenario_tick() a 60 Hz..."
    );
    eprintln!("[main] sin GUI, sin plays. Ctrl+C para detener.");

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

        let mut radio = radio.lock().await;
        for cmd in commands {
            radio.add_motion_command(cmd);
        }

        if let Err(err) = radio.send_commands().await {
            eprintln!("[main] error enviando: {err}");
        }
    }
}
