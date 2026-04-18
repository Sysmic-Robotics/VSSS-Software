use crate::coach::coach_trait::Coach;
use crate::coach::observation::{FIELD_HALF_X, FIELD_HALF_Y, Observation};
use crate::coach::robot_target::RobotTarget;
use glam::Vec2;

/// Coach clásico basado en reglas. Replica la lógica de `StandardPlay` expresada
/// como targets de posición, sin llamar a motion primitives.
///
/// Roles fijos:
/// - Robot 0 → Atacante: staging point detrás de la pelota en dirección al goal rival
/// - Robot 1 → Soporte: 25cm detrás + 20cm lateral respecto a la pelota
/// - Robot 2 → Portero: x fija delante del arco propio, y siguiendo la pelota
///
/// Este coach sirve para:
/// 1. Validar que el pipeline Coach → CoachPlay → Motion funciona correctamente.
/// 2. A/B testing frente a StandardPlay para verificar paridad de comportamiento.
/// 3. Fallback clásico si el modelo RL no está disponible.
pub struct RuleBasedCoach {
    pub attack_goal: Vec2,
    pub own_goal: Vec2,
}

impl RuleBasedCoach {
    pub fn new(attack_goal: Vec2, own_goal: Vec2) -> Self {
        Self {
            attack_goal,
            own_goal,
        }
    }

    /// Desnormaliza la posición de la pelota desde la observación.
    fn ball_pos(obs: &Observation) -> Vec2 {
        Vec2::new(obs.ball.x * FIELD_HALF_X, obs.ball.y * FIELD_HALF_Y)
    }

    fn attacker_target(&self, obs: &Observation) -> RobotTarget {
        let ball = Self::ball_pos(obs);
        let ball_to_goal = (self.attack_goal - ball).normalize_or_zero();
        // Staging: 16cm detrás de la pelota (mismo valor que ApproachBallBehindSkill::staging_offset)
        let staging = ball - ball_to_goal * 0.16;
        RobotTarget {
            robot_id: 0,
            position: staging,
            face_target: Some(ball),
        }
    }

    fn support_target(&self, obs: &Observation) -> RobotTarget {
        let ball = Self::ball_pos(obs);
        let to_own = (self.own_goal - ball).normalize_or_zero();
        let lateral = Vec2::new(-to_own.y, to_own.x) * 0.20;
        let raw = ball + to_own * 0.25 + lateral;
        let pos = Vec2::new(raw.x.clamp(-0.60, 0.60), raw.y.clamp(-0.55, 0.55));
        RobotTarget {
            robot_id: 1,
            position: pos,
            face_target: Some(ball),
        }
    }

    fn goalkeeper_target(&self, obs: &Observation) -> RobotTarget {
        let ball = Self::ball_pos(obs);
        let sign = if self.own_goal.x < 0.0 { 1.0_f32 } else { -1.0 };
        let defend_x = self.own_goal.x + sign * 0.12;
        let target_y = ball.y.clamp(-0.20, 0.20);
        RobotTarget {
            robot_id: 2,
            position: Vec2::new(defend_x, target_y),
            face_target: Some(ball),
        }
    }
}

impl Coach for RuleBasedCoach {
    fn decide(&mut self, obs: &Observation) -> Vec<RobotTarget> {
        vec![
            self.attacker_target(obs),
            self.support_target(obs),
            self.goalkeeper_target(obs),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::coach::observation::{BallObs, Observation, RobotObs};

    fn make_obs(ball_x: f32, ball_y: f32) -> Observation {
        Observation {
            ball: BallObs {
                x: ball_x / FIELD_HALF_X,
                y: ball_y / FIELD_HALF_Y,
                vx: 0.0,
                vy: 0.0,
            },
            own_robots: vec![RobotObs::default(); 3],
            opp_robots: vec![RobotObs::default(); 3],
            own_team: 0,
        }
    }

    #[test]
    fn test_rule_based_returns_three_targets() {
        let mut coach = RuleBasedCoach::new(Vec2::new(0.75, 0.0), Vec2::new(-0.75, 0.0));
        let obs = make_obs(0.1, 0.0);
        let targets = coach.decide(&obs);
        assert_eq!(targets.len(), 3);
        assert_eq!(targets[0].robot_id, 0);
        assert_eq!(targets[1].robot_id, 1);
        assert_eq!(targets[2].robot_id, 2);
    }

    #[test]
    fn test_attacker_staging_behind_ball() {
        let attack_goal = Vec2::new(0.75, 0.0);
        let mut coach = RuleBasedCoach::new(attack_goal, Vec2::new(-0.75, 0.0));
        // Pelota en el centro, goal a la derecha → staging debe estar a la izquierda de la pelota
        let obs = make_obs(0.0, 0.0);
        let targets = coach.decide(&obs);
        let attacker = &targets[0];
        // staging_x = ball_x - (normalized ball_to_goal_x) * 0.16 = 0 - 1*0.16 = -0.16
        assert!(
            attacker.position.x < 0.0,
            "staging debe estar detrás (izq) de la pelota"
        );
        assert!(attacker.face_target.is_some());
    }

    #[test]
    fn test_goalkeeper_tracks_ball_y() {
        let own_goal = Vec2::new(-0.75, 0.0);
        let mut coach = RuleBasedCoach::new(Vec2::new(0.75, 0.0), own_goal);
        // Pelota en y=0.15 → portero debe ir a y=0.15 (dentro del clamp ±0.20)
        let obs = make_obs(0.0, 0.15);
        let targets = coach.decide(&obs);
        let gk = &targets[2];
        assert!(
            (gk.position.y - 0.15).abs() < 0.01,
            "portero debe seguir y de la pelota"
        );
        // defend_x = -0.75 + 1.0 * 0.12 = -0.63
        assert!((gk.position.x - (-0.63)).abs() < 0.01);
    }

    #[test]
    fn test_goalkeeper_clamps_ball_y() {
        let mut coach = RuleBasedCoach::new(Vec2::new(0.75, 0.0), Vec2::new(-0.75, 0.0));
        // Pelota en y=0.50 → portero debe clampear a ±0.20
        let obs = make_obs(0.0, 0.50);
        let targets = coach.decide(&obs);
        assert!(
            (targets[2].position.y - 0.20).abs() < 0.01,
            "debe clampear a 0.20"
        );
    }
}
