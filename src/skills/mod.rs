use crate::motion::{Motion, MotionCommand};
use crate::world::{World, RobotState};
use glam::Vec2;

/// Skill: acción primitiva de un solo robot con estado propio.
/// Cada tick devuelve un MotionCommand listo para enviar.
pub trait Skill: Send + Sync {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand;
    fn is_done(&self) -> bool;
    fn reset(&mut self);
}

// ──────────────────────────────────────────────────────────────────────────────
// GoToSkill: navegar hacia una posición fija, mirando hacia donde se mueve.
// ──────────────────────────────────────────────────────────────────────────────
pub struct GoToSkill {
    pub target: Vec2,
    pub arrival_threshold: f32,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    done: bool,
}

impl GoToSkill {
    pub fn new(target: Vec2) -> Self {
        Self {
            target,
            arrival_threshold: 0.08,
            kp: 1.2,
            ki: 0.0,
            kd: 0.10,
            done: false,
        }
    }

    pub fn with_arrival(mut self, threshold: f32) -> Self {
        self.arrival_threshold = threshold;
        self
    }

    fn stop(robot: &RobotState) -> MotionCommand {
        MotionCommand {
            id: robot.id,
            team: robot.team,
            vx: 0.0,
            vy: 0.0,
            omega: 0.0,
            orientation: robot.orientation,
        }
    }
}

impl Skill for GoToSkill {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        let dist = (self.target - robot.position).length();
        if dist < self.arrival_threshold {
            self.done = true;
            return Self::stop(robot);
        }
        let mut cmd = motion.move_to(robot, self.target, world);
        let face = motion.face_to(robot, self.target, self.kp, self.ki, self.kd);
        cmd.omega = face.omega;
        cmd
    }

    fn is_done(&self) -> bool {
        self.done
    }

    fn reset(&mut self) {
        self.done = false;
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// ChaseSkill: posicionarse detrás de la pelota (staging) y empujar hacia el goal.
//
// Fases:
//   APPROACH: move_to(staging_point) + face_to(staging_point)
//             staging_point = ball - ball_to_goal * staging_offset
//   CAPTURA:  move_direct(push_target) + face_to(ball)
//             push_target = ball + ball_to_goal * push_overshoot
//
// Histéresis: entra a CAPTURA cuando behind_ball && dist_staging <= staging_tol.
//             sale de CAPTURA SOLO cuando robot está claramente por delante de la pelota.
// ──────────────────────────────────────────────────────────────────────────────
pub struct ChaseSkill {
    pub goal: Vec2,
    pub staging_offset: f32,
    pub staging_tol: f32,
    pub push_overshoot: f32,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub in_captura: bool,
}

impl ChaseSkill {
    pub fn new(goal: Vec2) -> Self {
        Self {
            goal,
            staging_offset: 0.16,
            staging_tol: 0.08,
            push_overshoot: 0.12,
            kp: 1.2,
            ki: 0.0,
            kd: 0.10,
            in_captura: false,
        }
    }
}

impl Skill for ChaseSkill {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        let ball_pos = world.get_ball_state().position;
        let ball_to_goal = (self.goal - ball_pos).normalize_or_zero();
        let staging_point = ball_pos - ball_to_goal * self.staging_offset;
        let push_target = ball_pos + ball_to_goal * self.push_overshoot;

        let dist_staging = (staging_point - robot.position).length();
        let dot = (robot.position - ball_pos).dot(self.goal - ball_pos);
        let behind_ball = dot < 0.02;
        let in_front_of_ball = dot > 0.05;

        if behind_ball && dist_staging <= self.staging_tol {
            self.in_captura = true;
        } else if in_front_of_ball {
            self.in_captura = false;
        }

        let face_target = if self.in_captura { ball_pos } else { staging_point };
        let mut cmd = if self.in_captura {
            motion.move_direct(robot, push_target)
        } else {
            motion.move_to(robot, staging_point, world)
        };
        let face = motion.face_to(robot, face_target, self.kp, self.ki, self.kd);
        cmd.omega = face.omega;
        cmd
    }

    fn is_done(&self) -> bool {
        false // nunca termina; la pelota puede moverse lejos del goal en cualquier momento
    }

    fn reset(&mut self) {
        self.in_captura = false;
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// DefendSkill: pararse frente al propio arco, siguiendo la Y de la pelota.
// Mantiene X fija (en frente del arco), mueve Y para bloquear tiros.
// ──────────────────────────────────────────────────────────────────────────────
pub struct DefendSkill {
    /// X de la línea de defensa (unos cm adelante del arco)
    pub defend_x: f32,
    /// Rango Y máximo donde el portero se desplaza (típicamente ±ancho_arco/2)
    pub goal_half_y: f32,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

impl DefendSkill {
    /// `own_goal` es el centro del arco propio, e.g. Vec2::new(-0.75, 0.0)
    pub fn new(own_goal: Vec2) -> Self {
        let sign = if own_goal.x < 0.0 { 1.0_f32 } else { -1.0 };
        Self {
            defend_x: own_goal.x + sign * 0.12,
            goal_half_y: 0.20,
            kp: 1.2,
            ki: 0.0,
            kd: 0.10,
        }
    }
}

impl Skill for DefendSkill {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        let ball_pos = world.get_ball_state().position;
        let target_y = ball_pos.y.clamp(-self.goal_half_y, self.goal_half_y);
        let defend_pos = Vec2::new(self.defend_x, target_y);

        let mut cmd = motion.move_to(robot, defend_pos, world);
        let face = motion.face_to(robot, ball_pos, self.kp, self.ki, self.kd);
        cmd.omega = face.omega;
        cmd
    }

    fn is_done(&self) -> bool {
        false
    }

    fn reset(&mut self) {}
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::World;

    fn make_robot(id: i32, x: f32, y: f32, orient_deg: f32) -> RobotState {
        let mut r = RobotState::new(id, 0);
        r.position = Vec2::new(x, y);
        r.orientation = orient_deg.to_radians() as f64;
        r
    }

    #[test]
    fn test_goto_skill_moves_toward_target() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.0, 0.5), Vec2::ZERO); // pelota fuera del camino
        let mut skill = GoToSkill::new(Vec2::new(0.5, 0.0));
        let robot = make_robot(0, 0.0, 0.0, 0.0);

        let cmd = skill.tick(&robot, &world, &motion);
        assert!(cmd.vx > 0.0, "debe moverse en X positivo");
        assert!(!skill.is_done());
    }

    #[test]
    fn test_goto_skill_done_at_target() {
        let motion = Motion::new();
        let world = World::new(3, 3);
        let mut skill = GoToSkill::new(Vec2::new(0.0, 0.0));
        let robot = make_robot(0, 0.03, 0.0, 0.0); // dentro del threshold 0.08m

        skill.tick(&robot, &world, &motion);
        assert!(skill.is_done());
    }

    #[test]
    fn test_chase_skill_approach_phase() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.1, 0.0), Vec2::ZERO);

        let mut skill = ChaseSkill::new(Vec2::new(0.75, 0.0));
        let robot = make_robot(0, -0.3, 0.0, 0.0);

        let _cmd = skill.tick(&robot, &world, &motion);
        // En approach, in_captura debe seguir false (robot lejos del staging)
        assert!(!skill.in_captura);
    }

    #[test]
    fn test_defend_skill_tracks_ball_y() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.0, 0.15), Vec2::ZERO);

        let mut skill = DefendSkill::new(Vec2::new(-0.75, 0.0));
        let robot = make_robot(2, -0.63, 0.0, 0.0);

        let cmd = skill.tick(&robot, &world, &motion);
        // Debe tener componente Y positiva (seguir pelota en y=0.15)
        assert!(cmd.vy > 0.0 || cmd.omega.abs() > 0.0, "debe moverse/rotar hacia pelota");
    }
}
