// Harness de pruebas — usa variables de entorno igual que main.rs.
// Ejecutar: VSSL_VISION_SOURCE=sslvision VSSL_RADIO_TARGET=basestation cargo run --bin scenario --release
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
//  run_skill_chase_ball   → perseguir la pelota
//  run_skill_stop         → detenerse
//  run_skill_face_point   → girar hacia un punto sin moverse
//  run_skill_hold_position→ mantener posición mirando a un punto
//  run_skill_defend_goal_line → goalkeeper simple
//  circuit                → recorrer waypoints en orden

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
    FacePointSkill, GoToSkill, HoldPositionSkill, PushBallSkill, Skill, StopSkill,
    SupportPositionSkill,
};
use rustengine::world::{RobotState, World};

fn scenario_tick(world: &World, motion: &Motion, state: &mut ScenarioState) -> Vec<MotionCommand> {
    run_skill_go_to_point(state, world, motion, ROBOT_ID, Vec2::new(0.0, 0.0))
    //run_skill_chase_ball(state, world, motion, ROBOT_ID)
    //run_skill_stop(state, world, motion, ROBOT_ID)
    //run_skill_face_point(state, world, motion, ROBOT_ID, world.get_ball_state().position)
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

#[allow(dead_code)]
fn run_skill_stop(
    state: &mut ScenarioState,
    world: &World,
    motion: &Motion,
    robot_id: i32,
) -> Vec<MotionCommand> {
    let Some(robot) = find_robot(world, robot_id) else { return vec![] };
    vec![state.stop.tick(&robot, world, motion)]
}

#[allow(dead_code)]
fn run_skill_go_to_point(
    state: &mut ScenarioState,
    world: &World,
    motion: &Motion,
    robot_id: i32,
    target: Vec2,
) -> Vec<MotionCommand> {
    let Some(robot) = find_robot(world, robot_id) else { return vec![] };
    state.go_to_point.set_target(target);
    vec![state.go_to_point.tick(&robot, world, motion)]
}

#[allow(dead_code)]
fn run_skill_face_point(
    state: &mut ScenarioState,
    world: &World,
    motion: &Motion,
    robot_id: i32,
    target: Vec2,
) -> Vec<MotionCommand> {
    let Some(robot) = find_robot(world, robot_id) else { return vec![] };
    state.face_point.set_target(target);
    vec![state.face_point.tick(&robot, world, motion)]
}

#[allow(dead_code)]
fn run_skill_hold_position(
    state: &mut ScenarioState,
    world: &World,
    motion: &Motion,
    robot_id: i32,
    position: Vec2,
    face: Option<Vec2>,
) -> Vec<MotionCommand> {
    let Some(robot) = find_robot(world, robot_id) else { return vec![] };
    state.hold_position.position = position;
    state.hold_position.face = face;
    vec![state.hold_position.tick(&robot, world, motion)]
}

#[allow(dead_code)]
fn run_skill_chase_ball(
    state: &mut ScenarioState,
    world: &World,
    motion: &Motion,
    robot_id: i32,
) -> Vec<MotionCommand> {
    let Some(robot) = find_robot(world, robot_id) else { return vec![] };
    vec![state.chase_ball.tick(&robot, world, motion)]
}

#[allow(dead_code)]
fn run_skill_defend_goal_line(
    state: &mut ScenarioState,
    world: &World,
    motion: &Motion,
    robot_id: i32,
    own_goal: Vec2,
) -> Vec<MotionCommand> {
    let Some(robot) = find_robot(world, robot_id) else { return vec![] };
    state.defend_goal_line = DefendGoalLineSkill::new(own_goal);
    vec![state.defend_goal_line.tick(&robot, world, motion)]
}

#[allow(dead_code)]
fn run_skill_support_position(
    state: &mut ScenarioState,
    world: &World,
    motion: &Motion,
    robot_id: i32,
    own_goal: Vec2,
) -> Vec<MotionCommand> {
    let Some(robot) = find_robot(world, robot_id) else { return vec![] };
    state.support_position = SupportPositionSkill::new(own_goal);
    vec![state.support_position.tick(&robot, world, motion)]
}

#[allow(dead_code)]
fn run_skill_approach_to_goal(
    state: &mut ScenarioState,
    world: &World,
    motion: &Motion,
    robot_id: i32,
    target_point: Vec2,
) -> Vec<MotionCommand> {
    let Some(robot) = find_robot(world, robot_id) else { return vec![] };
    state.approach_to_goal.set_aim_point(target_point);
    vec![state.approach_to_goal.tick(&robot, world, motion)]
}

#[allow(dead_code)]
fn run_skill_push_to_goal(
    state: &mut ScenarioState,
    world: &World,
    motion: &Motion,
    robot_id: i32,
    target_point: Vec2,
) -> Vec<MotionCommand> {
    let Some(robot) = find_robot(world, robot_id) else { return vec![] };
    state.push_to_goal.set_push_target(target_point);
    vec![state.push_to_goal.tick(&robot, world, motion)]
}

#[allow(dead_code)]
fn run_skill_align_to_goal(
    state: &mut ScenarioState,
    world: &World,
    motion: &Motion,
    robot_id: i32,
    target_point: Vec2,
) -> Vec<MotionCommand> {
    let Some(robot) = find_robot(world, robot_id) else { return vec![] };
    state.align_to_goal.set_target_point(target_point);
    vec![state.align_to_goal.tick(&robot, world, motion)]
}

#[allow(dead_code)]
fn circuit(
    state: &mut ScenarioState,
    world: &World,
    motion: &Motion,
    robot_id: i32,
    waypoints: &[Vec2],
) -> Vec<MotionCommand> {
    let Some(robot) = find_robot(world, robot_id) else { return vec![] };
    let target = waypoints[state.circuit_index % waypoints.len()];
    if (target - robot.position).length() < 0.08 {
        state.circuit_index = (state.circuit_index + 1) % waypoints.len();
    }
    state.go_to_point.set_target(target);
    vec![state.go_to_point.tick(&robot, world, motion)]
}

#[allow(dead_code)]
fn all_to_center(state: &mut ScenarioState, world: &World, motion: &Motion) -> Vec<MotionCommand> {
    let team_robots = if OWN_TEAM == 0 {
        world.get_blue_team_active()
    } else {
        world.get_yellow_team_active()
    };
    state.go_to_point.set_target(Vec2::ZERO);
    team_robots
        .iter()
        .map(|robot| state.go_to_point.tick(robot, world, motion))
        .collect()
}

// ─────────────────────────────────────────────────────────────────────────────
//  Runtime
// ─────────────────────────────────────────────────────────────────────────────
use rustengine::{
    radio,
    vision::{Vision, VisionEvent, VisionSource},
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
    let mut scenario = ScenarioState::default();
    apply_pid(&mut scenario);

    {
        let tracker_enabled = tracker_enabled.clone();
        let source = VisionSource::from_env();
        eprintln!(
            "[scenario] visión: {:?} ({}:{})",
            source,
            source.multicast_ip(),
            source.port()
        );
        tokio::spawn(async move {
            let mut vis = Vision::new(source, tracker_enabled);
            let (dummy_status_tx, _) = mpsc::channel(1);
            if let Err(err) = vis.run(vision_tx, dummy_status_tx).await {
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
        let commands = {
            let world = world.read().await;
            scenario_tick(&world, &motion, &mut scenario)
        };
        if commands.is_empty() {
            continue;
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
