/// Entorno de prueba de movimiento para FIRASim.
///
/// Corre `cargo run --bin scenario --release` para ejecutar.
/// No tiene GUI — solo logs por stderr y movimiento en FIRASim.
///
/// ────────────────────────────────────────────────────────────
///  EDITA AQUÍ TU ESCENARIO
/// ────────────────────────────────────────────────────────────
/// Modifica `scenario_tick()` con lo que quieras probar.
/// Algunos ejemplos están listos más abajo — descomenta el que quieras.
/// ────────────────────────────────────────────────────────────

// ====== Parámetros del escenario ======
/// Equipo que controla este escenario (0=azul, 1=amarillo)
const OWN_TEAM: i32 = 0;
/// Robot que usas en el escenario (cuando el escenario controla uno solo)
const ROBOT_ID: i32 = 0;
// ======================================

use rustengine::motion::{Motion, MotionCommand};
use rustengine::world::World;
use glam::Vec2;

/// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
/// FUNCIÓN PRINCIPAL DEL ESCENARIO — edita esto libremente.
/// Se llama a ~60 Hz con el estado actual del mundo.
/// Devuelve los comandos de movimiento que quieras aplicar.
/// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
fn scenario_tick(world: &World, motion: &Motion) -> Vec<MotionCommand> {
    // ── Ejemplo 1: Robot 0 va a una posición fija ──────────────
    //go_to_point(world, motion, ROBOT_ID, Vec2::new(0.3, 0.2))
    go_to_point(world, motion, ROBOT_ID, Vec2::new(0.0, 0.0))
    // ── Ejemplo 2: Robot 0 persigue la pelota ──────────────────
    //chase_ball(world, motion, ROBOT_ID)

    // ── Ejemplo 3: Robot 0 hace un circuito de 4 puntos ────────
    // circuit(world, motion, ROBOT_ID, &[
    //     Vec2::new( 0.4,  0.3),
    //     Vec2::new( 0.4, -0.3),
    //     Vec2::new(-0.4, -0.3),
    //     Vec2::new(-0.4,  0.3),
    // ])

    // ── Ejemplo 4: Todos los robots al centro ──────────────────
    // all_to_center(world, motion)
}

// ─────────────────────────────────────────────────────────────
// Funciones de ayuda — puedes usarlas o escribir las tuyas.
//
// NOTA: los robots VSS son diferenciales — sin omega (rotación)
// el robot solo puede avanzar/retroceder en la dirección que ya
// mira. Por eso todos los helpers usan move_and_face en lugar de
// move_to, así el robot rota Y avanza al mismo tiempo.
// Ganancias PID de heading: kp=1.2, ki=0.0, kd=0.10
// ─────────────────────────────────────────────────────────────

const KP: f64 = 1.2;
const KI: f64 = 0.0;
const KD: f64 = 0.10;

/// Mueve un robot a una posición fija mirando hacia donde va.
fn go_to_point(world: &World, motion: &Motion, robot_id: i32, target: Vec2) -> Vec<MotionCommand> {
    let team_robots = if OWN_TEAM == 0 {
        world.get_blue_team_active()
    } else {
        world.get_yellow_team_active()
    };
    team_robots
        .iter()
        .filter(|r| r.id == robot_id)
        .map(|r| motion.move_and_face(r, target, target, world, KP, KI, KD))
        .collect()
}

/// Mueve un robot hacia donde está la pelota.
#[allow(dead_code)]
fn chase_ball(world: &World, motion: &Motion, robot_id: i32) -> Vec<MotionCommand> {
    let ball_pos = world.get_ball_state().position;
    go_to_point(world, motion, robot_id, ball_pos)
}

/// Hace un circuito de puntos (pasa al siguiente al llegar).
#[allow(dead_code)]
fn circuit(world: &World, motion: &Motion, robot_id: i32, waypoints: &[Vec2]) -> Vec<MotionCommand> {
    use std::cell::Cell;
    thread_local! {
        static WAYPOINT_IDX: Cell<usize> = Cell::new(0);
    }

    let team_robots = if OWN_TEAM == 0 {
        world.get_blue_team_active()
    } else {
        world.get_yellow_team_active()
    };

    let Some(robot) = team_robots.iter().find(|r| r.id == robot_id) else {
        return vec![];
    };

    WAYPOINT_IDX.with(|idx_cell| {
        let idx = idx_cell.get();
        let target = waypoints[idx % waypoints.len()];
        if (target - robot.position).length() < 0.08 {
            idx_cell.set((idx + 1) % waypoints.len());
        }
        vec![motion.move_and_face(robot, target, target, world, KP, KI, KD)]
    })
}

/// Mueve todos los robots del equipo al centro del campo.
#[allow(dead_code)]
fn all_to_center(world: &World, motion: &Motion) -> Vec<MotionCommand> {
    let team_robots = if OWN_TEAM == 0 {
        world.get_blue_team_active()
    } else {
        world.get_yellow_team_active()
    };
    team_robots
        .iter()
        .map(|r| motion.move_and_face(r, Vec2::ZERO, Vec2::ZERO, world, KP, KI, KD))
        .collect()
}

// ─────────────────────────────────────────────────────────────
// Infraestructura — no necesitas editar lo que está debajo.
// ─────────────────────────────────────────────────────────────

use rustengine::{radio, vision::{Vision, VisionEvent}};
use tokio::sync::{mpsc, Mutex as TokioMutex, RwLock as TokioRwLock};
use std::sync::{Arc, atomic::{AtomicBool, Ordering}};
use std::time::Duration;

fn main() {
    let rt = tokio::runtime::Runtime::new().unwrap();
    rt.block_on(async_main());
}

async fn async_main() {
    let (vision_tx, mut vision_rx) = mpsc::channel(100);
    let world = Arc::new(TokioRwLock::new(World::new(3, 3)));
    let tracker_enabled = Arc::new(AtomicBool::new(true));

    // Visión
    {
        let world = world.clone();
        let tracker_enabled = tracker_enabled.clone();
        tokio::spawn(async move {
            let mut vis = Vision::new("224.0.0.1".to_string(), 10002, tracker_enabled);
            let (dummy_status_tx, _) = mpsc::channel(1);
            if let Err(e) = vis.run(vision_tx, dummy_status_tx).await {
                eprintln!("[scenario] visión error: {e}");
            }
        });
    }

    // Actualizar mundo con eventos de visión
    {
        let world = world.clone();
        tokio::spawn(async move {
            while let Some(event) = vision_rx.recv().await {
                let mut w = world.write().await;
                match event {
                    VisionEvent::Robot(r) => {
                        w.update_robot(r.id as i32, r.team as i32, r.position,
                            r.orientation as f64, r.velocity, r.angular_velocity as f64);
                    }
                    VisionEvent::Ball(b) => {
                        w.update_ball(b.position, b.velocity);
                    }
                }
            }
        });
    }

    // Marcar robots inactivos cada 100ms
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

    // Radio
    let radio = match radio::Radio::new(false, radio::SimulatorType::FIRASim, "127.0.0.1", 20011).await {
        Ok(r) => Arc::new(TokioMutex::new(r)),
        Err(e) => {
            eprintln!("[scenario] error radio: {e}");
            return;
        }
    };

    eprintln!("[scenario] conectado a FIRASim. Ejecutando scenario_tick() a 60 Hz...");
    eprintln!("[scenario] Ctrl+C para detener.");

    let motion = rustengine::motion::Motion::new();
    let mut interval = tokio::time::interval(Duration::from_millis(16));

    loop {
        interval.tick().await;
        let commands = {
            let world = world.read().await;
            scenario_tick(&world, &motion)
        };
        if commands.is_empty() {
            continue;
        }
        let mut radio = radio.lock().await;
        for cmd in commands {
            radio.add_motion_command(cmd);
        }
        if let Err(e) = radio.send_commands().await {
            eprintln!("[scenario] error enviando: {e}");
        }
    }
}
