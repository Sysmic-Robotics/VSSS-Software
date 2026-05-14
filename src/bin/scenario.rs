// Harness de pruebas — usa variables de entorno igual que main.rs.
// Ejecutar:
//   cargo run --bin scenario --release                                  # headless contra FIRASim
//   VSSL_DEBUG_GUI=1 cargo run --bin scenario --release                 # con GUI de debug
//   VSSL_VISION_SOURCE=sslvision VSSL_RADIO_TARGET=basestation cargo run --bin scenario --release
//
// ─────────────────────────────────────────────────────────────────────────────
//  PASO 1: elige el robot y equipo
// ─────────────────────────────────────────────────────────────────────────────
const OWN_TEAM: i32 = 0; // 0 = azul, 1 = amarillo
const ROBOT_ID: i32 = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  PASO 2: elige el escenario (descomenta UNA línea en scenario_tick())
// ─────────────────────────────────────────────────────────────────────────────
//  run_skill_go_to_point  → ir a un punto fijo
//  run_skill_face_point   → girar hacia un punto sin moverse
//  run_skill_chase_ball   → perseguir la pelota
//  run_skill_spin         → rotar en el lugar (signo de x → CCW/CW)
//  run_skill_stop         → detenerse
//  run_skill_hold_position→ mantener posición mirando a un punto
//  run_skill_defend_goal_line → goalkeeper simple
//  run_skill_support_position, run_skill_approach_to_goal,
//  run_skill_push_to_goal, run_skill_align_to_goal → skills out-of-catalog
//  circuit                → recorrer waypoints en orden
//  all_to_center          → mandar a todo el equipo al centro

// ─────────────────────────────────────────────────────────────────────────────
//  PASO 3: tuning de velocidad
// ─────────────────────────────────────────────────────────────────────────────
const MAX_LINEAR_SPEED: f64 = 0.5; // m/s — bajar para primeras pruebas físicas
const MIN_LINEAR_SPEED: f64 = 0.06; // m/s — velocidad mínima antes de frenar
const MAX_ANGULAR_SPEED: f64 = 2.0; // rad/s
const BRAKE_DISTANCE: f32 = 0.40; // m — distancia donde empieza a frenar
const ARRIVAL_THRESHOLD: f32 = 0.06; // m — margen para considerar "llegó"
/// 0.0 = para si no mira hacia donde va  |  1.0 = ignora heading, siempre a tope
const COUPLING_FLOOR: f32 = 0.22;

// ─────────────────────────────────────────────────────────────────────────────
//  PASO 4: tuning de evasión de obstáculos (UVF)
// ─────────────────────────────────────────────────────────────────────────────
const UVF_INFLUENCE_RADIUS: f32 = 0.20; // m — radio de deflexión por obstáculo
const UVF_K_REP: f32 = 1.5; // ganancia repulsiva — más alto = más brusco

// ─────────────────────────────────────────────────────────────────────────────
//  PASO 5: tuning PID de orientación
//  Aplica a: GoToSkill, ChaseBallSkill, FacePointSkill, HoldPositionSkill, etc.
// ─────────────────────────────────────────────────────────────────────────────
const PID_KP: f64 = 3.0;
const PID_KI: f64 = 0.08;
const PID_KD: f64 = 0.20;

// ─────────────────────────────────────────────────────────────────────────────
//  ESCENARIO
// ─────────────────────────────────────────────────────────────────────────────
use glam::Vec2;
use rustengine::motion::{Motion, MotionCommand, MotionConfig};
use rustengine::skills::{
    AlignBallToTargetSkill, ApproachBallBehindSkill, ChaseBallSkill, DefendGoalLineSkill,
    FacePointSkill, GoToSkill, HoldPositionSkill, PushBallSkill, Skill, SpinSkill, StopSkill,
    SupportPositionSkill,
};
use rustengine::world::{RobotState, World};

/// Resultado de cada `run_skill_*`: comando + target visible para overlay GUI.
type SkillTickOutput = (Vec<MotionCommand>, Vec<Option<Vec2>>);

fn scenario_tick(world: &World, motion: &Motion, state: &mut ScenarioState) -> SkillTickOutput {
    run_skill_go_to_point(state, world, motion, ROBOT_ID, Vec2::new(0.0, 0.0))
    //run_skill_face_point(state, world, motion, ROBOT_ID, world.get_ball_state().position)
    //run_skill_chase_ball(state, world, motion, ROBOT_ID)
    //run_skill_spin(state, world, motion, ROBOT_ID, 1.0)  // +1.0 = CCW, -1.0 = CW
    //run_skill_stop(state, world, motion, ROBOT_ID)
    //run_skill_hold_position(state, world, motion, ROBOT_ID, Vec2::new(0.0, 0.0), None)
    //run_skill_defend_goal_line(state, world, motion, ROBOT_ID, Vec2::new(-0.75, 0.0))
    //run_skill_support_position(state, world, motion, ROBOT_ID, Vec2::new(-0.75, 0.0))
    //run_skill_approach_to_goal(state, world, motion, ROBOT_ID, Vec2::new(0.75, 0.0))
    //run_skill_push_to_goal(state, world, motion, ROBOT_ID, Vec2::new(0.75, 0.0))
    //run_skill_align_to_goal(state, world, motion, ROBOT_ID, Vec2::new(0.75, 0.0))
    //circuit(state, world, motion, ROBOT_ID, &[
    //    Vec2::new( 0.5,  0.4),
    //    Vec2::new( 0.5, -0.4),
    //    Vec2::new(-0.5, -0.4),
    //    Vec2::new(-0.5,  0.4),
    //])
    //all_to_center(state, world, motion)
}

fn build_motion() -> Motion {
    Motion::with_config(MotionConfig {
        max_linear_speed: MAX_LINEAR_SPEED,
        min_linear_speed: MIN_LINEAR_SPEED,
        max_angular_speed: MAX_ANGULAR_SPEED,
        brake_distance: BRAKE_DISTANCE,
        arrival_threshold: ARRIVAL_THRESHOLD,
        coupling_floor: COUPLING_FLOOR,
        uvf_influence_radius: UVF_INFLUENCE_RADIUS,
        uvf_k_rep: UVF_K_REP,
    })
}

fn apply_pid(state: &mut ScenarioState) {
    state.go_to_point.kp = PID_KP;
    state.go_to_point.ki = PID_KI;
    state.go_to_point.kd = PID_KD;
    state.face_point.kp = PID_KP;
    state.face_point.ki = PID_KI;
    state.face_point.kd = PID_KD;
    state.hold_position.kp = PID_KP;
    state.hold_position.ki = PID_KI;
    state.hold_position.kd = PID_KD;
    state.chase_ball.kp = PID_KP;
    state.chase_ball.ki = PID_KI;
    state.chase_ball.kd = PID_KD;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Estado del escenario (una instancia de cada skill)
// ─────────────────────────────────────────────────────────────────────────────
struct ScenarioState {
    stop: StopSkill,
    go_to_point: GoToSkill,
    face_point: FacePointSkill,
    hold_position: HoldPositionSkill,
    chase_ball: ChaseBallSkill,
    spin: SpinSkill,
    defend_goal_line: DefendGoalLineSkill,
    support_position: SupportPositionSkill,
    approach_to_goal: ApproachBallBehindSkill,
    push_to_goal: PushBallSkill,
    align_to_goal: AlignBallToTargetSkill,
    circuit_index: usize,
}

impl Default for ScenarioState {
    fn default() -> Self {
        Self {
            stop: StopSkill::new(),
            go_to_point: GoToSkill::new(Vec2::ZERO),
            face_point: FacePointSkill::new(Vec2::ZERO),
            hold_position: HoldPositionSkill::new(Vec2::ZERO),
            chase_ball: ChaseBallSkill::new(),
            spin: SpinSkill::new(),
            defend_goal_line: DefendGoalLineSkill::new(Vec2::new(-0.75, 0.0)),
            support_position: SupportPositionSkill::new(Vec2::new(-0.75, 0.0)),
            approach_to_goal: ApproachBallBehindSkill::new(Vec2::ZERO),
            push_to_goal: PushBallSkill::new(Vec2::ZERO),
            align_to_goal: AlignBallToTargetSkill::new(Vec2::ZERO),
            circuit_index: 0,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Helpers de skills
// ─────────────────────────────────────────────────────────────────────────────
fn find_robot(world: &World, robot_id: i32) -> Option<RobotState> {
    let team = if OWN_TEAM == 0 {
        world.get_blue_team_active()
    } else {
        world.get_yellow_team_active()
    };
    team.into_iter().find(|robot| robot.id == robot_id).cloned()
}

/// Empaqueta el resultado de una skill llamada en un robot: comando + target visible.
fn one(cmd: MotionCommand, target: Option<Vec2>) -> SkillTickOutput {
    (vec![cmd], vec![target])
}

#[allow(dead_code)]
fn run_skill_stop(
    state: &mut ScenarioState,
    world: &World,
    motion: &Motion,
    robot_id: i32,
) -> SkillTickOutput {
    let Some(robot) = find_robot(world, robot_id) else { return (vec![], vec![]) };
    let cmd = state.stop.tick(&robot, world, motion);
    let target = state.stop.current_target(world);
    one(cmd, target)
}

#[allow(dead_code)]
fn run_skill_go_to_point(
    state: &mut ScenarioState,
    world: &World,
    motion: &Motion,
    robot_id: i32,
    target: Vec2,
) -> SkillTickOutput {
    let Some(robot) = find_robot(world, robot_id) else { return (vec![], vec![]) };
    state.go_to_point.set_target(target);
    let cmd = state.go_to_point.tick(&robot, world, motion);
    let target = state.go_to_point.current_target(world);
    one(cmd, target)
}

#[allow(dead_code)]
fn run_skill_face_point(
    state: &mut ScenarioState,
    world: &World,
    motion: &Motion,
    robot_id: i32,
    target: Vec2,
) -> SkillTickOutput {
    let Some(robot) = find_robot(world, robot_id) else { return (vec![], vec![]) };
    state.face_point.set_target(target);
    let cmd = state.face_point.tick(&robot, world, motion);
    let target = state.face_point.current_target(world);
    one(cmd, target)
}

#[allow(dead_code)]
fn run_skill_hold_position(
    state: &mut ScenarioState,
    world: &World,
    motion: &Motion,
    robot_id: i32,
    position: Vec2,
    face: Option<Vec2>,
) -> SkillTickOutput {
    let Some(robot) = find_robot(world, robot_id) else { return (vec![], vec![]) };
    state.hold_position.position = position;
    state.hold_position.face = face;
    let cmd = state.hold_position.tick(&robot, world, motion);
    let target = state.hold_position.current_target(world);
    one(cmd, target)
}

#[allow(dead_code)]
fn run_skill_chase_ball(
    state: &mut ScenarioState,
    world: &World,
    motion: &Motion,
    robot_id: i32,
) -> SkillTickOutput {
    let Some(robot) = find_robot(world, robot_id) else { return (vec![], vec![]) };
    let cmd = state.chase_ball.tick(&robot, world, motion);
    let target = state.chase_ball.current_target(world);
    one(cmd, target)
}

/// Hace girar al robot en su sitio. `direction = +1.0` = CCW, `-1.0` = CW, `0.0` = no spin.
/// Internamente la skill usa el signo de `target.x`, así que mapeamos el float a Vec2.
#[allow(dead_code)]
fn run_skill_spin(
    state: &mut ScenarioState,
    world: &World,
    motion: &Motion,
    robot_id: i32,
    direction: f32,
) -> SkillTickOutput {
    let Some(robot) = find_robot(world, robot_id) else { return (vec![], vec![]) };
    state.spin.set_direction_from(Vec2::new(direction, 0.0));
    let cmd = state.spin.tick(&robot, world, motion);
    // Spin no tiene target espacial — el overlay queda en blanco a propósito.
    one(cmd, state.spin.current_target(world))
}

#[allow(dead_code)]
fn run_skill_defend_goal_line(
    state: &mut ScenarioState,
    world: &World,
    motion: &Motion,
    robot_id: i32,
    own_goal: Vec2,
) -> SkillTickOutput {
    let Some(robot) = find_robot(world, robot_id) else { return (vec![], vec![]) };
    state.defend_goal_line = DefendGoalLineSkill::new(own_goal);
    let cmd = state.defend_goal_line.tick(&robot, world, motion);
    let target = state.defend_goal_line.current_target(world);
    one(cmd, target)
}

#[allow(dead_code)]
fn run_skill_support_position(
    state: &mut ScenarioState,
    world: &World,
    motion: &Motion,
    robot_id: i32,
    own_goal: Vec2,
) -> SkillTickOutput {
    let Some(robot) = find_robot(world, robot_id) else { return (vec![], vec![]) };
    state.support_position = SupportPositionSkill::new(own_goal);
    let cmd = state.support_position.tick(&robot, world, motion);
    let target = state.support_position.current_target(world);
    one(cmd, target)
}

#[allow(dead_code)]
fn run_skill_approach_to_goal(
    state: &mut ScenarioState,
    world: &World,
    motion: &Motion,
    robot_id: i32,
    target_point: Vec2,
) -> SkillTickOutput {
    let Some(robot) = find_robot(world, robot_id) else { return (vec![], vec![]) };
    state.approach_to_goal.set_aim_point(target_point);
    let cmd = state.approach_to_goal.tick(&robot, world, motion);
    let target = state.approach_to_goal.current_target(world);
    one(cmd, target)
}

#[allow(dead_code)]
fn run_skill_push_to_goal(
    state: &mut ScenarioState,
    world: &World,
    motion: &Motion,
    robot_id: i32,
    target_point: Vec2,
) -> SkillTickOutput {
    let Some(robot) = find_robot(world, robot_id) else { return (vec![], vec![]) };
    state.push_to_goal.set_push_target(target_point);
    let cmd = state.push_to_goal.tick(&robot, world, motion);
    let target = state.push_to_goal.current_target(world);
    one(cmd, target)
}

#[allow(dead_code)]
fn run_skill_align_to_goal(
    state: &mut ScenarioState,
    world: &World,
    motion: &Motion,
    robot_id: i32,
    target_point: Vec2,
) -> SkillTickOutput {
    let Some(robot) = find_robot(world, robot_id) else { return (vec![], vec![]) };
    state.align_to_goal.set_target_point(target_point);
    let cmd = state.align_to_goal.tick(&robot, world, motion);
    let target = state.align_to_goal.current_target(world);
    one(cmd, target)
}

#[allow(dead_code)]
fn circuit(
    state: &mut ScenarioState,
    world: &World,
    motion: &Motion,
    robot_id: i32,
    waypoints: &[Vec2],
) -> SkillTickOutput {
    let Some(robot) = find_robot(world, robot_id) else { return (vec![], vec![]) };
    let target = waypoints[state.circuit_index % waypoints.len()];
    if (target - robot.position).length() < 0.08 {
        state.circuit_index = (state.circuit_index + 1) % waypoints.len();
    }
    state.go_to_point.set_target(target);
    let cmd = state.go_to_point.tick(&robot, world, motion);
    let target = state.go_to_point.current_target(world);
    one(cmd, target)
}

#[allow(dead_code)]
fn all_to_center(state: &mut ScenarioState, world: &World, motion: &Motion) -> SkillTickOutput {
    let team_robots = if OWN_TEAM == 0 {
        world.get_blue_team_active()
    } else {
        world.get_yellow_team_active()
    };
    state.go_to_point.set_target(Vec2::ZERO);
    let cmds: Vec<MotionCommand> = team_robots
        .iter()
        .map(|robot| state.go_to_point.tick(robot, world, motion))
        .collect();
    let target = state.go_to_point.current_target(world);
    let targets = vec![target; cmds.len()];
    (cmds, targets)
}

// ─────────────────────────────────────────────────────────────────────────────
//  Runtime
// ─────────────────────────────────────────────────────────────────────────────
use rustengine::GUI;
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

/// Modo debug: GUI en el hilo principal, tokio en background.
/// Replica el patrón de `src/main.rs::run_with_gui` para mantener el harness
/// alineado con el binario principal.
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

async fn async_main(
    gui_channels: Option<(
        mpsc::Sender<GUI::StatusUpdate>,
        mpsc::Sender<Vec<GUI::RobotMotionDebug>>,
    )>,
) {
    let (vision_tx, mut vision_rx) = mpsc::channel(100);
    let world = Arc::new(TokioRwLock::new(World::new(3, 3)));
    let tracker_enabled = Arc::new(AtomicBool::new(true));
    let mut scenario = ScenarioState::default();
    apply_pid(&mut scenario);

    let (status_tx, motion_tx) = match gui_channels {
        Some((s, m)) => (Some(s), Some(m)),
        None => (None, None),
    };

    {
        let tracker_enabled = tracker_enabled.clone();
        let source = VisionSource::from_env();
        eprintln!(
            "[scenario] visión: {:?} ({}:{})",
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
                eprintln!("[scenario] vision error: {err}");
            }
        });
    }

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
            eprintln!("[scenario] radio error: {err}");
            return;
        }
    };

    eprintln!("[scenario] listo. Ejecutando scenario_tick() a 60 Hz... (Ctrl+C para detener)");

    let motion = build_motion();
    let mut interval = tokio::time::interval(Duration::from_millis(16));

    loop {
        interval.tick().await;
        let (commands, targets) = {
            let world = world.read().await;
            scenario_tick(&world, &motion, &mut scenario)
        };
        if commands.is_empty() {
            continue;
        }

        if let Some(ref tx) = motion_tx {
            let updates: Vec<GUI::RobotMotionDebug> = commands
                .iter()
                .zip(targets.iter())
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
        for cmd in commands {
            radio.add_motion_command(cmd);
        }
        if let Err(err) = radio.send_commands().await {
            eprintln!("[scenario] error enviando: {err}");
        }
    }
}
