//! Receptor (`SkillId::Receive`): el apoyo espera la pelota en un punto, como `Mark`, pero
//! ya orientado para empujarla al arco rival apenas le llegue.
//!
//! En 48 partidos, cuando la pelota le llegaba al apoyo (rebotes, despejes) le faltaba girar
//! una mediana de 43° para apuntar al arco con alguna de sus caras, y solo el 22 % llegaba a
//! ≤ 20°: un diferencial gira antes de empujar, y en ese tiempo llega el rival
//! (`docs/investigacion/02_apoyo.md`, D4). La idea es la redirección de TIGERs (SSL): el
//! receptor se para para que la pelota salga hacia el arco sin alinearse.
//!
//! Se mueve igual que `Mark` (`move_and_face`, la navegación calibrada de motion) y solo
//! cambia hacia dónde se orienta al llegar: una cara en la dirección intermedia, módulo
//! 180°, entre la recta a la pelota y la recta al arco rival, con peso
//! `receive_goal_weight` del arco (0 = mira la pelota, como `Mark`). Sin la dirección de
//! ataque de motion (`Motion::with_areas`), mira la pelota.

use super::{Face, MarkSkill, Skill, SkillStatus, clamp_to_logical_field};
use crate::motion::{Motion, MotionCommand};
use crate::params::params;
use crate::world::{RobotState, World};
use glam::Vec2;

/// Distancia del centro del arco rival a la línea de gol (m).
const GOAL_X: f32 = 0.75;

pub struct ReceiveSkill {
    pub point: Vec2,
    pub goal_weight: f32,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

impl ReceiveSkill {
    pub fn new(point: Vec2) -> Self {
        // Las ganancias de orientación de `Mark`: la misma respuesta ya calibrada.
        let mark = MarkSkill::new(point);
        Self {
            point,
            goal_weight: params().skills.receive_goal_weight,
            kp: mark.kp,
            ki: mark.ki,
            kd: mark.kd,
        }
    }

    pub fn set_point(&mut self, p: Vec2) {
        self.point = p;
    }

    /// Orientación de espera en `at`, entre la recta a la pelota y la recta al arco con peso
    /// `goal_weight` del arco. Con dos caras la pelota está tanto en `b` como en `−b`, así que
    /// hay dos mezclas, `w·g + (1−w)·b` y `w·g − (1−w)·b`. Se queda con la más cercana
    /// (módulo 180°) a la orientación actual `heading`: con la pelota a 90° del arco las dos
    /// valen lo mismo, y elegir por cercanía evita que salte de una a otra. Devuelve la cara
    /// que da hacia el arco.
    pub fn facing(&self, at: Vec2, ball: Vec2, goal: Vec2, heading: f32) -> Vec2 {
        let to_ball = (ball - at).normalize_or_zero();
        let to_goal = (goal - at).normalize_or_zero();
        if to_goal == Vec2::ZERO {
            return to_ball;
        }
        let w = self.goal_weight.clamp(0.0, 1.0);
        let turn = |d: Vec2| {
            let e = Motion::normalize_angle((d.y.atan2(d.x) - heading) as f64);
            Motion::fold_bidirectional(e).abs()
        };
        let d = [to_goal * w + to_ball * (1.0 - w), to_goal * w - to_ball * (1.0 - w)]
            .into_iter()
            .filter(|c| c.length_squared() > 1e-9)
            .map(|c| c.normalize())
            .min_by(|a, b| turn(*a).total_cmp(&turn(*b)))
            .unwrap_or(to_goal);
        if d.dot(to_goal) < 0.0 { -d } else { d }
    }
}

impl Skill for ReceiveSkill {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        let ball = world.get_ball_state().position;
        let point = clamp_to_logical_field(self.point);
        let face = match motion.attack_dir() {
            Some(dir) => robot.position + self.facing(point, ball, dir * GOAL_X, robot.orientation as f32),
            None => ball,
        };
        motion.move_and_face(robot, point, face, world, self.kp, self.ki, self.kd)
    }

    fn current_target(&self, _world: &World) -> Option<Vec2> {
        Some(clamp_to_logical_field(self.point))
    }

    fn status(&self, robot: &RobotState, world: &World) -> SkillStatus {
        let ball = world.get_ball_state().position;
        let dist = (clamp_to_logical_field(self.point) - robot.position).length();
        let to_ball = ball - robot.position;
        let raw = Motion::normalize_angle((to_ball.y as f64).atan2(to_ball.x as f64) - robot.orientation);
        SkillStatus {
            progress: (1.0 - dist / 0.6).clamp(0.0, 1.0),
            done: false,
            feasible: true,
            face: if raw.abs() <= std::f64::consts::FRAC_PI_2 { Face::Front } else { Face::Back },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn skill(w: f32) -> ReceiveSkill {
        ReceiveSkill { goal_weight: w, ..ReceiveSkill::new(Vec2::ZERO) }
    }

    fn deg(d: Vec2) -> f32 {
        d.y.atan2(d.x).to_degrees()
    }

    const AT: Vec2 = Vec2::new(0.3, 0.3);
    const GOAL: Vec2 = Vec2::new(0.75, 0.0);

    #[test]
    fn weight_zero_faces_the_ball_and_one_faces_the_goal() {
        let ball = Vec2::new(0.0, 0.3);
        // Pelota detrás (−x): con peso 0 queda sobre la recta a la pelota, con la cara que da
        // al arco.
        assert!(deg(skill(0.0).facing(AT, ball, GOAL, 0.3)).abs() < 1e-3);
        let to_goal = (GOAL - AT).normalize();
        assert!((skill(1.0).facing(AT, ball, GOAL, 2.0) - to_goal).length() < 1e-5);
    }

    #[test]
    fn half_weight_is_the_bisector_closest_to_the_current_heading() {
        let at = Vec2::ZERO;
        let goal = Vec2::new(0.75, 0.0);
        // Pelota a −60° y arco a 0°: la mitad es −30°.
        let ball = Vec2::new(0.2, -0.2 * 3f32.sqrt());
        assert!((deg(skill(0.5).facing(at, ball, goal, 0.0)) + 30.0).abs() < 1e-3);
        // Pelota a −90°, justo de costado: ±45° valen lo mismo; elige la más cercana a la
        // orientación actual, así no salta de una a otra.
        let ball = Vec2::new(0.0, -0.4);
        assert!((deg(skill(0.5).facing(at, ball, goal, 0.6)) - 45.0).abs() < 1e-3);
        assert!((deg(skill(0.5).facing(at, ball, goal, -0.6)) + 45.0).abs() < 1e-3);
        // Pelota detrás del robot y arco adelante: misma recta.
        assert!(deg(skill(0.5).facing(at, Vec2::new(-0.3, 0.0), goal, 1.0)).abs() < 1e-3);
    }

    #[test]
    fn always_turns_the_face_toward_the_goal() {
        for w in [0.0, 0.3, 0.7, 1.0] {
            for ball in [Vec2::new(-0.5, 0.6), Vec2::new(0.6, -0.5), Vec2::new(0.2, 0.31)] {
                for heading in [-2.5, 0.0, 1.2] {
                    let d = skill(w).facing(AT, ball, GOAL, heading);
                    assert!(d.dot(GOAL - AT) >= 0.0, "w {w}, pelota {ball}: {d}");
                    assert!((d.length() - 1.0).abs() < 1e-5);
                }
            }
        }
    }
}
