// Modo base: headless por defecto.
// Con VSSL_DEBUG_GUI=1 lanza la interfaz gráfica con vectores de velocidad y targets.

use rustengine::GUI;

// ====== Parámetros ======
const OWN_TEAM: i32 = 0; // 0=azul, 1=amarillo
const NUM_ROBOTS: usize = 3;

// Frame-skip del coach (Fase 3 — opción E del horizonte de decisión).
// El coach se consulta una vez cada COACH_DECISION_PERIOD ticks; entre medio
// el dispatcher reusa la última decisión y solo refresca las skills (que sí
// corren a 60 Hz para suavidad de control). Con período=6 a 60 Hz, la policy
// decide a 10 Hz (≈ 100 ms por decisión), valor estándar en VSSS-RL.
const COACH_DECISION_PERIOD: u32 = 6;
// ========================

use rustengine::coach::{Coach, Observation, RuleBasedCoach, SkillChoice};
use rustengine::motion::{Motion, MotionCommand};
use rustengine::skills::{SkillCatalog, SkillId};
use rustengine::world::World;
use glam::Vec2;

/// Direcciones de ataque convencionales según el equipo propio.
/// Azul ataca a +X, amarillo ataca a -X.
fn goals_for_team(own_team: i32) -> (Vec2, Vec2) {
    if own_team == 0 {
        (Vec2::new(0.75, 0.0), Vec2::new(-0.75, 0.0))
    } else {
        (Vec2::new(-0.75, 0.0), Vec2::new(0.75, 0.0))
    }
}

/// Construye el coach inicial. Soporta selección por env var:
/// - `VSSL_COACH=rule_based` (default): baseline clásico.
/// - `VSSL_COACH=none`: no emite decisiones (útil para test de visión/radio).
///
/// El día que llegue Fase 7, este factory va a aceptar también `VSSL_COACH=rl`
/// para cargar un modelo ONNX.
fn make_coach(own_team: i32) -> Option<Box<dyn Coach>> {
    let kind = std::env::var("VSSL_COACH").unwrap_or_else(|_| "rule_based".to_string());
    let (attack_goal, own_goal) = goals_for_team(own_team);
    match kind.as_str() {
        "rule_based" => Some(Box::new(RuleBasedCoach::new(attack_goal, own_goal))),
        "none" => None,
        other => {
            eprintln!("[main] VSSL_COACH='{other}' inválido, usando 'rule_based'");
            Some(Box::new(RuleBasedCoach::new(attack_goal, own_goal)))
        }
    }
}

/// Punto visible (en coordenadas de campo, m) que la skill activa está usando
/// como destino para el overlay de debug en la GUI. Devuelve `None` para skills
/// que no tienen target espacial (Spin).
fn debug_target_for(choice: &SkillChoice, world: &World) -> Option<Vec2> {
    match choice.skill_id {
        SkillId::GoTo | SkillId::FacePoint => Some(choice.target),
        SkillId::ChaseBall => Some(world.get_ball_state().position),
        SkillId::Spin => None,
    }
}

/// Para cada `SkillChoice`, busca el robot correspondiente en el equipo
/// activo y dispatcha la skill via el catálogo. Robots no encontrados
/// (porque están inactivos) se ignoran silenciosamente.
///
/// Devuelve el comando junto con el target visible para overlay (cyan dot en la GUI).
fn dispatch_choices(
    choices: &[SkillChoice],
    catalog: &mut SkillCatalog,
    world: &World,
    motion: &Motion,
    own_team: i32,
) -> Vec<(MotionCommand, Option<Vec2>)> {
    let team_robots: Vec<_> = if own_team == 0 {
        world.get_blue_team_active().into_iter().cloned().collect()
    } else {
        world
            .get_yellow_team_active()
            .into_iter()
            .cloned()
            .collect()
    };

    let mut commands = Vec::with_capacity(choices.len());
    for choice in choices {
        let Some(robot) = team_robots.iter().find(|r| r.id == choice.robot_id) else {
            continue; // robot inactivo o id fuera de rango
        };
        let robot_idx = choice.robot_id as usize;
        if robot_idx >= catalog.num_robots() {
            continue; // catálogo no tiene slot para este id
        }
        let cmd = catalog.tick(robot_idx, choice.skill_id, choice.target, robot, world, motion);
        let target = debug_target_for(choice, world);
        commands.push((cmd, target));
    }
    commands
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
//
//  Estructura del control loop a 60 Hz (Fase 3 del plan RL):
//
//    cada tick (16 ms):
//      ├─ si tick % COACH_DECISION_PERIOD == 0:
//      │     obs ← Observation::from_world(world, OWN_TEAM)
//      │     last_choices ← coach.decide(&obs)
//      └─ commands ← dispatch_choices(last_choices, catalog, ...)
//         radio.send(commands)
//
//  El coach decide a 10 Hz; las skills (UVF + PID) corren a 60 Hz dentro del
//  catálogo. Esto matchea la opción E del horizonte (frame-skip transparente
//  a la policy RL futura) y replica el patrón de rSoccer / Bassani 2020.
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

    let mut coach = make_coach(OWN_TEAM);
    eprintln!(
        "[main] coach: {}",
        if coach.is_some() {
            std::env::var("VSSL_COACH").unwrap_or_else(|_| "rule_based".to_string())
        } else {
            "none".to_string()
        }
    );
    eprintln!(
        "[main] decisión cada {} ticks ({:.0} Hz)",
        COACH_DECISION_PERIOD,
        60.0 / COACH_DECISION_PERIOD as f64
    );
    eprintln!("[main] listo. 60 Hz control loop. Ctrl+C para detener.");

    let motion = Motion::new();
    let mut catalog = SkillCatalog::new(NUM_ROBOTS);
    let mut last_choices: Vec<SkillChoice> = Vec::new();
    let mut tick_counter: u32 = 0;
    let mut interval = tokio::time::interval(Duration::from_millis(16));

    loop {
        interval.tick().await;

        let commands = {
            let world = world.read().await;

            // Cada COACH_DECISION_PERIOD ticks, refrescamos la decisión.
            if tick_counter.is_multiple_of(COACH_DECISION_PERIOD) {
                if let Some(coach) = coach.as_mut() {
                    let obs = Observation::from_world(&world, OWN_TEAM);
                    last_choices = coach.decide(&obs);
                }
            }

            // Cada tick, dispatch sobre las choices vigentes.
            dispatch_choices(&last_choices, &mut catalog, &world, &motion, OWN_TEAM)
        };
        tick_counter = tick_counter.wrapping_add(1);

        if commands.is_empty() {
            continue;
        }

        // Enviar debug al GUI si está activo
        if let Some(ref tx) = motion_tx {
            let updates: Vec<GUI::RobotMotionDebug> = commands
                .iter()
                .map(|(cmd, target)| GUI::RobotMotionDebug {
                    team: cmd.team as u32,
                    id: cmd.id as u32,
                    vx: cmd.vx as f32,
                    vy: cmd.vy as f32,
                    target: *target,
                })
                .collect();
            let _ = tx.try_send(updates);
        }

        let mut radio = radio.lock().await;
        for (cmd, _target) in commands {
            radio.add_motion_command(cmd);
        }
        if let Err(err) = radio.send_commands().await {
            eprintln!("[main] error enviando: {err}");
        }
    }
}
