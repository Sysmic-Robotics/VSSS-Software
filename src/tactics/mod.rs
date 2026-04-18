use crate::motion::{Motion, MotionCommand};
use crate::skills::{
    ApproachBallBehindSkill, DefendGoalLineSkill, HoldPositionSkill, Skill, SupportPositionSkill,
};
use crate::world::{RobotState, World};
use glam::Vec2;

pub trait Tactic: Send + Sync {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand;
}

struct StuckDetector {
    stuck_ticks: u32,
    yield_ticks: u32,
    last_pos: Vec2,
    yield_dir: f32,
}

impl StuckDetector {
    fn new() -> Self {
        Self {
            stuck_ticks: 0,
            yield_ticks: 0,
            last_pos: Vec2::ZERO,
            yield_dir: 1.0,
        }
    }

    fn update(&mut self, robot_pos: Vec2, target: Vec2) -> Option<Vec2> {
        let dist_to_target = (target - robot_pos).length();
        let moved = (robot_pos - self.last_pos).length();
        self.last_pos = robot_pos;

        if dist_to_target > 0.08 {
            if moved < 0.006 {
                self.stuck_ticks += 1;
            } else {
                self.stuck_ticks = 0;
            }
        } else {
            self.stuck_ticks = 0;
        }

        if self.stuck_ticks >= 30 && self.yield_ticks == 0 {
            self.yield_dir = if (robot_pos.x + robot_pos.y) >= 0.0 {
                1.0
            } else {
                -1.0
            };
            self.yield_ticks = 35;
            self.stuck_ticks = 0;
        }

        if self.yield_ticks > 0 {
            self.yield_ticks -= 1;
            let to_target = (target - robot_pos).normalize_or_zero();
            let perp = Vec2::new(-to_target.y, to_target.x) * self.yield_dir;
            let escape = robot_pos + perp * 0.18;
            Some(Vec2::new(
                escape.x.clamp(-0.68, 0.68),
                escape.y.clamp(-0.58, 0.58),
            ))
        } else {
            None
        }
    }
}

pub struct AttackerTactic {
    goal: Vec2,
    skill: ApproachBallBehindSkill,
    stuck: StuckDetector,
}

impl AttackerTactic {
    pub fn new(attack_goal: Vec2) -> Self {
        Self {
            goal: attack_goal,
            skill: ApproachBallBehindSkill::new(attack_goal),
            stuck: StuckDetector::new(),
        }
    }
}

impl Tactic for AttackerTactic {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        self.skill.set_aim_point(self.goal);

        let ball = world.get_ball_state().position;
        let ball_to_goal = (self.goal - ball).normalize_or_zero();
        let staging = ball - ball_to_goal * self.skill.staging_offset;

        if let Some(escape_pos) = self.stuck.update(robot.position, staging) {
            let mut hold = HoldPositionSkill::new(escape_pos).facing(staging);
            return hold.tick(robot, world, motion);
        }

        self.skill.tick(robot, world, motion)
    }
}

pub struct GoalkeeperTactic {
    skill: DefendGoalLineSkill,
    stuck: StuckDetector,
}

impl GoalkeeperTactic {
    pub fn new(own_goal: Vec2) -> Self {
        Self {
            skill: DefendGoalLineSkill::new(own_goal),
            stuck: StuckDetector::new(),
        }
    }
}

impl Tactic for GoalkeeperTactic {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        let ball = world.get_ball_state().position;
        let defend_pos = self.skill.defend_pos(ball);

        if let Some(escape_pos) = self.stuck.update(robot.position, defend_pos) {
            let mut hold = HoldPositionSkill::new(escape_pos).facing(ball);
            return hold.tick(robot, world, motion);
        }

        self.skill.tick(robot, world, motion)
    }
}

pub struct SupportTactic {
    skill: SupportPositionSkill,
    stuck: StuckDetector,
}

impl SupportTactic {
    pub fn new(own_goal: Vec2) -> Self {
        Self {
            skill: SupportPositionSkill::new(own_goal),
            stuck: StuckDetector::new(),
        }
    }
}

impl Tactic for SupportTactic {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        let ball = world.get_ball_state().position;
        let support_pos = self.skill.support_pos(ball);

        if let Some(escape_pos) = self.stuck.update(robot.position, support_pos) {
            let mut hold = HoldPositionSkill::new(escape_pos).facing(ball);
            return hold.tick(robot, world, motion);
        }

        self.skill.tick(robot, world, motion)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_robot(id: i32, x: f32, y: f32, orient_deg: f32) -> RobotState {
        let mut robot = RobotState::new(id, 0);
        robot.position = Vec2::new(x, y);
        robot.orientation = orient_deg.to_radians() as f64;
        robot
    }

    #[test]
    fn attacker_generates_activity_toward_ball_staging() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.0, 0.0), Vec2::ZERO);
        let robot = make_robot(0, -0.4, 0.0, 0.0);
        let mut tactic = AttackerTactic::new(Vec2::new(0.75, 0.0));

        let cmd = tactic.tick(&robot, &world, &motion);

        assert!(cmd.vx.abs() + cmd.vy.abs() + cmd.omega.abs() > 0.0);
    }

    #[test]
    fn goalkeeper_generates_a_useful_command() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.0, 0.15), Vec2::ZERO);
        let robot = make_robot(2, -0.40, 0.0, 0.0);
        let mut tactic = GoalkeeperTactic::new(Vec2::new(-0.75, 0.0));

        let cmd = tactic.tick(&robot, &world, &motion);

        assert!(cmd.vx.abs() + cmd.vy.abs() + cmd.omega.abs() > 0.0);
    }

    #[test]
    fn support_generates_a_useful_command() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.2, 0.0), Vec2::ZERO);
        let robot = make_robot(1, -0.4, 0.1, 0.0);
        let mut tactic = SupportTactic::new(Vec2::new(-0.75, 0.0));

        let cmd = tactic.tick(&robot, &world, &motion);

        assert!(cmd.vx.abs() + cmd.vy.abs() + cmd.omega.abs() > 0.0);
    }
}
