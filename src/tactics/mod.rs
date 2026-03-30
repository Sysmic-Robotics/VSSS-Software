use crate::motion::{Motion, MotionCommand};
use crate::skills::{ChaseSkill, DefendSkill, Skill};
use crate::world::{RobotState, World};
use glam::Vec2;

/// Tactic: rol continuo de un robot. Mantiene estado entre ticks,
/// decide qué skill usar según el estado del juego.
pub trait Tactic: Send + Sync {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand;
}

// ──────────────────────────────────────────────────────────────────────────────
// StuckDetector: detecta cuando un robot no progresa y activa un escape lateral.
//
// Lógica:
//   - Si el robot está lejos del target pero se mueve < 3mm/tick por 20 ticks
//     consecutivos (~0.33s), se considera atascado.
//   - Al atascarse, el robot ejecuta un "yield": se mueve 18cm en perpendicular
//     al vector robot→target durante 35 ticks (~0.58s), luego reintenta.
//   - La dirección del yield alterna según posición del robot para romper
//     la simetría en choques frontales (robots opuestos eligen lados distintos).
// ──────────────────────────────────────────────────────────────────────────────
struct StuckDetector {
    stuck_ticks: u32,
    yield_ticks: u32,
    last_pos: Vec2,
    yield_dir: f32, // +1.0 o -1.0
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

    /// Llama cada tick. Si detecta stuck, devuelve un target de escape lateral.
    /// Si todo va bien, devuelve None (el tactic usa su lógica normal).
    fn update(&mut self, robot_pos: Vec2, target: Vec2) -> Option<Vec2> {
        let dist_to_target = (target - robot_pos).length();
        let moved = (robot_pos - self.last_pos).length();
        self.last_pos = robot_pos;

        // Solo contar stuck cuando estamos lejos del target (no cuando llegamos y paramos)
        if dist_to_target > 0.08 {
            if moved < 0.003 {
                self.stuck_ticks += 1;
            } else {
                self.stuck_ticks = 0;
            }
        } else {
            self.stuck_ticks = 0;
        }

        // Activar yield tras ~0.33s sin moverse
        if self.stuck_ticks >= 20 && self.yield_ticks == 0 {
            // Dirección de escape: alterna según cuadrante del robot para que
            // dos robots que se bloquean frontalmente elijan lados opuestos.
            self.yield_dir = if (robot_pos.x + robot_pos.y) >= 0.0 { 1.0 } else { -1.0 };
            self.yield_ticks = 35;
            self.stuck_ticks = 0;
        }

        if self.yield_ticks > 0 {
            self.yield_ticks -= 1;
            let to_target = (target - robot_pos).normalize_or_zero();
            // Perpendicular al vector hacia el target
            let perp = Vec2::new(-to_target.y, to_target.x) * self.yield_dir;
            // Moverse 18cm lateral, clampeado al campo
            let escape = robot_pos + perp * 0.18;
            let clamped = Vec2::new(
                escape.x.clamp(-0.68, 0.68),
                escape.y.clamp(-0.58, 0.58),
            );
            Some(clamped)
        } else {
            None
        }
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// AttackerTactic: persigue la pelota y la empuja al arco rival.
// El atacante tiene prioridad: no cede ante otros robots.
// ──────────────────────────────────────────────────────────────────────────────
pub struct AttackerTactic {
    skill: ChaseSkill,
    stuck: StuckDetector,
}

impl AttackerTactic {
    pub fn new(attack_goal: Vec2) -> Self {
        Self {
            skill: ChaseSkill::new(attack_goal),
            stuck: StuckDetector::new(),
        }
    }
}

impl Tactic for AttackerTactic {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        // Solo activar stuck recovery durante APPROACH (no durante CAPTURA,
        // donde el robot DEBE permanecer empujando aunque se mueva despacio).
        if !self.skill.in_captura {
            let ball_pos = world.get_ball_state().position;
            let ball_to_goal = (self.skill.goal - ball_pos).normalize_or_zero();
            let staging = ball_pos - ball_to_goal * self.skill.staging_offset;

            if let Some(escape_pos) = self.stuck.update(robot.position, staging) {
                return motion.move_and_face(robot, escape_pos, staging, world,
                    self.skill.kp, self.skill.ki, self.skill.kd);
            }
        }
        self.skill.tick(robot, world, motion)
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// GoalkeeperTactic: defiende el arco propio, rastrea la Y de la pelota.
// Cede lateralmente cuando está atascado (generalmente contra el robot de soporte).
// ──────────────────────────────────────────────────────────────────────────────
pub struct GoalkeeperTactic {
    skill: DefendSkill,
    stuck: StuckDetector,
}

impl GoalkeeperTactic {
    pub fn new(own_goal: Vec2) -> Self {
        Self {
            skill: DefendSkill::new(own_goal),
            stuck: StuckDetector::new(),
        }
    }
}

impl Tactic for GoalkeeperTactic {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        let ball_pos = world.get_ball_state().position;
        let target_y = ball_pos.y.clamp(-self.skill.goal_half_y, self.skill.goal_half_y);
        let defend_pos = Vec2::new(self.skill.defend_x, target_y);

        if let Some(escape_pos) = self.stuck.update(robot.position, defend_pos) {
            return motion.move_and_face(robot, escape_pos, ball_pos, world,
                self.skill.kp, self.skill.ki, self.skill.kd);
        }

        self.skill.tick(robot, world, motion)
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// SupportTactic: posicionarse como respaldo del atacante, lateral-atrás de la pelota.
// Cede lateralmente cuando está atascado (choque frontal con otro robot).
// ──────────────────────────────────────────────────────────────────────────────
pub struct SupportTactic {
    own_goal: Vec2,
    stuck: StuckDetector,
}

impl SupportTactic {
    pub fn new(own_goal: Vec2) -> Self {
        Self {
            own_goal,
            stuck: StuckDetector::new(),
        }
    }

    fn compute_support_pos(&self, ball: Vec2) -> Vec2 {
        let to_own = (self.own_goal - ball).normalize_or_zero();
        let lateral = Vec2::new(-to_own.y, to_own.x) * 0.20;
        let raw = ball + to_own * 0.25 + lateral;
        Vec2::new(
            raw.x.clamp(-0.60, 0.60),
            raw.y.clamp(-0.55, 0.55),
        )
    }
}

impl Tactic for SupportTactic {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        let ball_pos = world.get_ball_state().position;
        let support_pos = self.compute_support_pos(ball_pos);

        if let Some(escape_pos) = self.stuck.update(robot.position, support_pos) {
            return motion.move_and_face(robot, escape_pos, ball_pos, world, 1.2, 0.0, 0.10);
        }

        // Mirar al balón (como RuleBasedCoach): alinear con UVF hacia support sin forzar
        // heading = hacia un punto vacío que choca con la deflexión del campo.
        motion.move_and_face(robot, support_pos, ball_pos, world, 1.2, 0.0, 0.10)
    }
}
