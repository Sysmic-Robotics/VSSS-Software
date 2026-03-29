use crate::motion::{Motion, MotionCommand};
use crate::skills::{ChaseSkill, DefendSkill, GoToSkill, Skill};
use crate::world::{RobotState, World};
use glam::Vec2;

/// Tactic: rol continuo de un robot. Mantiene estado entre ticks,
/// decide qué skill usar según el estado del juego.
pub trait Tactic: Send + Sync {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand;
}

// ──────────────────────────────────────────────────────────────────────────────
// AttackerTactic: usa ChaseSkill para posicionarse detrás de la pelota y empujar.
// ──────────────────────────────────────────────────────────────────────────────
pub struct AttackerTactic {
    skill: ChaseSkill,
}

impl AttackerTactic {
    pub fn new(attack_goal: Vec2) -> Self {
        Self {
            skill: ChaseSkill::new(attack_goal),
        }
    }
}

impl Tactic for AttackerTactic {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        self.skill.tick(robot, world, motion)
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// GoalkeeperTactic: pararse frente al arco propio, rastrear la Y de la pelota.
// ──────────────────────────────────────────────────────────────────────────────
pub struct GoalkeeperTactic {
    skill: DefendSkill,
}

impl GoalkeeperTactic {
    pub fn new(own_goal: Vec2) -> Self {
        Self {
            skill: DefendSkill::new(own_goal),
        }
    }
}

impl Tactic for GoalkeeperTactic {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        self.skill.tick(robot, world, motion)
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// SupportTactic: posicionarse como respaldo del atacante.
// Se ubica 20cm detrás de la pelota (respecto al arco propio), listo para heredar.
// ──────────────────────────────────────────────────────────────────────────────
pub struct SupportTactic {
    own_goal: Vec2,
    skill: GoToSkill,
}

impl SupportTactic {
    pub fn new(own_goal: Vec2) -> Self {
        Self {
            own_goal,
            skill: GoToSkill::new(Vec2::ZERO),
        }
    }
}

impl Tactic for SupportTactic {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        let ball = world.get_ball_state().position;
        // Dirección desde pelota hacia arco propio (opuesto al ataque)
        let to_own = (self.own_goal - ball).normalize_or_zero();
        // Posición de soporte: 25cm detrás de la pelota en dirección al arco propio,
        // desplazado lateralmente 20cm para no bloquear al atacante.
        let lateral = Vec2::new(-to_own.y, to_own.x) * 0.20;
        let support_pos = ball + to_own * 0.25 + lateral;
        let clamped = Vec2::new(
            support_pos.x.clamp(-0.60, 0.60),
            support_pos.y.clamp(-0.55, 0.55),
        );
        self.skill.target = clamped;
        self.skill.reset(); // objetivo dinámico: nunca marcar como terminado
        self.skill.tick(robot, world, motion)
    }
}
