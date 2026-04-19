// Entorno de prueba de movimiento para FIRASim.
//
// Corre `cargo run --bin scenario --release` para ejecutar.
// No tiene GUI; solo logs por stderr y movimiento en FIRASim.
//
// ---------------------------------------------------------------------------
//  EDITA AQUI TU ESCENARIO
// ---------------------------------------------------------------------------
// Modifica `scenario_tick()` con lo que quieras probar.
// Todas las primitives de abajo son skills reactivas simples.
// ---------------------------------------------------------------------------

const OWN_TEAM: i32 = 0;
const ROBOT_ID: i32 = 0;

use glam::Vec2;
use rustengine::motion::{Motion, MotionCommand};
use rustengine::skills::{
    AlignBallToTargetSkill, ApproachBallBehindSkill, ChaseBallSkill, DefendGoalLineSkill,
    FacePointSkill, GoToSkill, HoldPositionSkill, PushBallSkill, Skill, StopSkill,
    SupportPositionSkill,
};
use rustengine::world::{RobotState, World};

fn scenario_tick(world: &World, motion: &Motion, state: &mut ScenarioState) -> Vec<MotionCommand> {
    //run_skill_chase_ball(state, world, motion, ROBOT_ID)
    //run_skill_stop(state, world, motion, ROBOT_ID)
    //run_skill_go_to_point(state, world, motion, ROBOT_ID, Vec2::new(0.3, 0.2))
    //run_skill_face_point(state, world, motion, ROBOT_ID, world.get_ball_state().position)
    //run_skill_hold_position(state, world, motion, ROBOT_ID, Vec2::new(0.0, 0.3), None)
    run_skill_defend_goal_line(state, world, motion, ROBOT_ID, Vec2::new(-0.75, 0.0))
    //run_skill_support_position(state, world, motion, ROBOT_ID, Vec2::new(-0.75, 0.0))
    //run_skill_approach_to_goal(state, world, motion, ROBOT_ID, Vec2::new(0.75, 0.0))
    //run_skill_push_to_goal(state, world, motion, ROBOT_ID, Vec2::new(0.75, 0.0))
    //run_skill_align_to_goal(state, world, motion, ROBOT_ID, Vec2::new(0.75, 0.0))
}

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
    let Some(robot) = find_robot(world, robot_id) else {
        return vec![];
    };
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
    let Some(robot) = find_robot(world, robot_id) else {
        return vec![];
    };
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
    let Some(robot) = find_robot(world, robot_id) else {
        return vec![];
    };
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
    let Some(robot) = find_robot(world, robot_id) else {
        return vec![];
    };
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
    let Some(robot) = find_robot(world, robot_id) else {
        return vec![];
    };
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
    let Some(robot) = find_robot(world, robot_id) else {
        return vec![];
    };
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
    let Some(robot) = find_robot(world, robot_id) else {
        return vec![];
    };
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
    let Some(robot) = find_robot(world, robot_id) else {
        return vec![];
    };
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
    let Some(robot) = find_robot(world, robot_id) else {
        return vec![];
    };
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
    let Some(robot) = find_robot(world, robot_id) else {
        return vec![];
    };
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
    let Some(robot) = find_robot(world, robot_id) else {
        return vec![];
    };

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

    {
        // scenario es sim-only (resetea poses en FIRASim), por eso ignora
        // VSSL_VISION_SOURCE y fija FiraSim explícitamente.
        let tracker_enabled = tracker_enabled.clone();
        tokio::spawn(async move {
            let mut vis = Vision::new(VisionSource::FiraSim, tracker_enabled);
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
                let mut current_world = world.write().await;
                match event {
                    VisionEvent::Robot(robot) => {
                        current_world.update_robot(
                            robot.id as i32,
                            robot.team as i32,
                            robot.position,
                            robot.orientation as f64,
                            robot.velocity,
                            robot.angular_velocity as f64,
                        );
                    }
                    VisionEvent::Ball(ball) => {
                        current_world.update_ball(ball.position, ball.velocity);
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

    // scenario es sim-only: hardcodea FiraSim e ignora VSSL_RADIO_TARGET.
    let radio = match radio::FiraSimTransport::new("127.0.0.1", 20011).await {
        Ok(transport) => Arc::new(TokioMutex::new(radio::Radio::new(Box::new(transport)))),
        Err(err) => {
            eprintln!("[scenario] radio error: {err}");
            return;
        }
    };

    eprintln!("[scenario] conectado a FIRASim. Ejecutando scenario_tick() a 60 Hz...");
    eprintln!("[scenario] Ctrl+C para detener.");

    let motion = Motion::new();
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
        for command in commands {
            radio.add_motion_command(command);
        }
        if let Err(err) = radio.send_commands().await {
            eprintln!("[scenario] error enviando: {err}");
        }
    }
}
