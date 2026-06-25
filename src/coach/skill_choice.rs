//! Decisión del Coach para un robot en un tick — versión Fase 3.
//!
//! Reemplaza al viejo `RobotTarget` (target-espacial) por una selección
//! discreta de skill del catálogo + un parámetro continuo. Este es el
//! contrato directo con la policy RL:
//!
//! ```text
//!   policy output (per robot)  →  SkillChoice
//!   ─────────────────────────     ───────────────────────────────────
//!   discrete: skill_id ∈ 0..4  →  skill_id (SkillId)
//!   continuous: (x, y) ∈ ℝ²    →  target (Vec2, en coords de campo)
//! ```
//!
//! Mantener el shape simple (un discreto + un Vec2) facilita la paridad
//! Rust ↔ Python: la policy emite un tensor `(N, 3)` para N robots, donde
//! cada fila es `[skill_id, target_x, target_y]`. Skills que ignoran el
//! target (`ChaseBall`) o solo usan su signo (`Spin`) reciben la coord
//! completa y la interpretan según corresponda.

use crate::skills::SkillId;
use glam::Vec2;

/// Decisión atómica del Coach para un robot en un tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkillChoice {
    /// ID del robot al que aplica esta decisión (0, 1 o 2 en VSS 3v3).
    pub robot_id: i32,
    /// Skill del catálogo seleccionada por la policy/regla.
    pub skill_id: SkillId,
    /// Parámetro continuo en coordenadas de campo (m). Interpretación:
    /// - `GoTo` / `FacePoint` → punto destino completo.
    /// - `ChaseBall`          → ignorado.
    /// - `Spin`               → solo se usa el signo de `target.x`.
    pub target: Vec2,
}

impl SkillChoice {
    /// Constructor genérico — la policy RL deserializa así.
    pub fn new(robot_id: i32, skill_id: SkillId, target: Vec2) -> Self {
        Self {
            robot_id,
            skill_id,
            target,
        }
    }

    /// Helper: navegar a un punto.
    pub fn goto(robot_id: i32, target: Vec2) -> Self {
        Self {
            robot_id,
            skill_id: SkillId::GoTo,
            target,
        }
    }

    /// Helper: rotar para mirar a un punto.
    pub fn face_point(robot_id: i32, target: Vec2) -> Self {
        Self {
            robot_id,
            skill_id: SkillId::FacePoint,
            target,
        }
    }

    /// Helper: perseguir la pelota (target ignorado, se setea a cero).
    pub fn chase_ball(robot_id: i32) -> Self {
        Self {
            robot_id,
            skill_id: SkillId::ChaseBall,
            target: Vec2::ZERO,
        }
    }

    /// Helper: spin con dirección dada por el signo del primer argumento.
    /// `direction > 0 → CCW`, `direction < 0 → CW`, `0 → no spin`.
    pub fn spin(robot_id: i32, direction: f32) -> Self {
        Self {
            robot_id,
            skill_id: SkillId::Spin,
            target: Vec2::new(direction, 0.0),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn helpers_set_correct_skill_id() {
        assert_eq!(
            SkillChoice::goto(0, Vec2::new(0.5, 0.0)).skill_id,
            SkillId::GoTo
        );
        assert_eq!(
            SkillChoice::face_point(1, Vec2::new(0.5, 0.0)).skill_id,
            SkillId::FacePoint
        );
        assert_eq!(SkillChoice::chase_ball(2).skill_id, SkillId::ChaseBall);
        assert_eq!(SkillChoice::spin(0, 1.0).skill_id, SkillId::Spin);
    }

    #[test]
    fn chase_ball_zeros_target() {
        let c = SkillChoice::chase_ball(0);
        assert_eq!(c.target, Vec2::ZERO);
    }

    #[test]
    fn spin_encodes_direction_in_target_x() {
        assert!(SkillChoice::spin(0, 1.0).target.x > 0.0);
        assert!(SkillChoice::spin(0, -1.0).target.x < 0.0);
        assert_eq!(SkillChoice::spin(0, 0.0).target.x, 0.0);
    }
}
