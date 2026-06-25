use crate::coach::coach_trait::Coach;
use crate::coach::observation::{FIELD_HALF_X, FIELD_HALF_Y, Observation};
use crate::coach::skill_choice::SkillChoice;
use crate::skills::SkillId;
use glam::Vec2;

/// **Baseline clásico contra el cual comparar el modelo RL** (A/B obligatorio).
///
/// Reescrito en Fase 3 del plan RL contra la nueva firma `Coach<SkillChoice>`.
/// La lógica táctica subyacente es la misma que la versión vieja
/// (`StandardPlay`): roles fijos atacante/soporte/portero. La diferencia es
/// que ahora se expresa como **selección de skill del catálogo cerrado**,
/// idéntica al output que va a producir el `RlCoach`. Esto permite
/// comparar las dos policies bajo exactamente el mismo path de ejecución
/// (mismo dispatcher, mismas skills, mismas constantes).
///
/// Roles fijos (no se reasignan dinámicamente en este baseline):
/// - **Robot 0 → Atacante**: `GoTo` al staging point detrás de la pelota
///   en dirección al goal rival; conmuta a `ChaseBall` cuando ya está cerca
///   del staging (≤10 cm). Esto reproduce el patrón "approach then chase"
///   de la striker LARC 2019.
/// - **Robot 1 → Soporte**: `GoTo` a una posición 25 cm detrás de la pelota
///   con offset lateral de 20 cm, clampeada al campo lógico.
/// - **Robot 2 → Portero**: `GoTo` a un punto sobre la línea defensiva
///   (12 cm delante del propio arco), siguiendo la `y` de la pelota
///   clampeada a ±20 cm.
///
/// **Importante**: este coach solo usa skills del catálogo (`GoTo`,
/// `ChaseBall`). No usa `FacePoint` ni `Spin` — esas las ejercerá el
/// modelo RL cuando aprenda situaciones donde son útiles. Si el RL no
/// las usa nunca, el catálogo crecía sobre suposiciones equivocadas.
pub struct RuleBasedCoach {
    pub attack_goal: Vec2,
    pub own_goal: Vec2,
    /// Distancia al staging por debajo de la cual el atacante conmuta a
    /// `ChaseBall`. 10 cm es generoso pero estable.
    pub attacker_chase_radius: f32,
    /// Offset detrás de la pelota en dirección al goal rival (m).
    /// Mismo valor que `ApproachBallBehindSkill::staging_offset`.
    pub attacker_staging_offset: f32,
}

impl RuleBasedCoach {
    pub fn new(attack_goal: Vec2, own_goal: Vec2) -> Self {
        Self {
            attack_goal,
            own_goal,
            attacker_chase_radius: 0.10,
            attacker_staging_offset: 0.16,
        }
    }

    /// Desnormaliza la posición de la pelota desde la observación.
    fn ball_pos(obs: &Observation) -> Vec2 {
        Vec2::new(obs.ball.x * FIELD_HALF_X, obs.ball.y * FIELD_HALF_Y)
    }

    /// Desnormaliza la posición del robot propio en el slot `idx` (0..3).
    fn own_robot_pos(obs: &Observation, idx: usize) -> Vec2 {
        let r = &obs.own_robots[idx];
        Vec2::new(r.x * FIELD_HALF_X, r.y * FIELD_HALF_Y)
    }

    fn attacker_choice(&self, obs: &Observation) -> SkillChoice {
        let ball = Self::ball_pos(obs);
        let robot = Self::own_robot_pos(obs, 0);
        let ball_to_goal = (self.attack_goal - ball).normalize_or_zero();
        let staging = ball - ball_to_goal * self.attacker_staging_offset;

        if (robot - staging).length() < self.attacker_chase_radius {
            // Cerca del staging → empujar la pelota persiguiéndola.
            SkillChoice::chase_ball(0)
        } else {
            // Lejos del staging → ir al staging primero.
            SkillChoice::goto(0, staging)
        }
    }

    fn support_choice(&self, obs: &Observation) -> SkillChoice {
        let ball = Self::ball_pos(obs);
        let to_own = (self.own_goal - ball).normalize_or_zero();
        let lateral = Vec2::new(-to_own.y, to_own.x) * 0.20;
        let raw = ball + to_own * 0.25 + lateral;
        let pos = Vec2::new(raw.x.clamp(-0.60, 0.60), raw.y.clamp(-0.55, 0.55));
        SkillChoice::goto(1, pos)
    }

    fn goalkeeper_choice(&self, obs: &Observation) -> SkillChoice {
        let ball = Self::ball_pos(obs);
        let sign = if self.own_goal.x < 0.0 { 1.0_f32 } else { -1.0 };
        let defend_x = self.own_goal.x + sign * 0.12;
        let target_y = ball.y.clamp(-0.20, 0.20);
        SkillChoice::goto(2, Vec2::new(defend_x, target_y))
    }
}

impl Coach for RuleBasedCoach {
    fn decide(&mut self, obs: &Observation) -> Vec<SkillChoice> {
        vec![
            self.attacker_choice(obs),
            self.support_choice(obs),
            self.goalkeeper_choice(obs),
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

    fn make_obs_with_robot(ball: Vec2, robot_idx: usize, robot_pos: Vec2) -> Observation {
        let mut obs = make_obs(ball.x, ball.y);
        obs.own_robots[robot_idx] = RobotObs {
            x: robot_pos.x / FIELD_HALF_X,
            y: robot_pos.y / FIELD_HALF_Y,
            vx: 0.0,
            vy: 0.0,
            sin_theta: 0.0,
            cos_theta: 1.0,
            omega: 0.0,
            active: 1.0,
        };
        obs
    }

    #[test]
    fn rule_based_returns_three_choices() {
        let mut coach = RuleBasedCoach::new(Vec2::new(0.75, 0.0), Vec2::new(-0.75, 0.0));
        let obs = make_obs(0.1, 0.0);
        let choices = coach.decide(&obs);
        assert_eq!(choices.len(), 3);
        assert_eq!(choices[0].robot_id, 0);
        assert_eq!(choices[1].robot_id, 1);
        assert_eq!(choices[2].robot_id, 2);
    }

    #[test]
    fn attacker_uses_goto_to_staging_when_far() {
        let attack_goal = Vec2::new(0.75, 0.0);
        let mut coach = RuleBasedCoach::new(attack_goal, Vec2::new(-0.75, 0.0));
        // Pelota en el centro, robot 0 lejos del staging.
        let obs = make_obs_with_robot(Vec2::new(0.0, 0.0), 0, Vec2::new(-0.5, 0.0));
        let choices = coach.decide(&obs);
        let attacker = &choices[0];

        assert_eq!(attacker.skill_id, SkillId::GoTo);
        // staging_x = 0 - 1*0.16 = -0.16 (detrás de la pelota respecto al goal)
        assert!(
            attacker.target.x < 0.0,
            "staging debe estar detrás (izq) de la pelota: target={:?}",
            attacker.target
        );
        assert!((attacker.target.x - (-0.16)).abs() < 0.01);
    }

    #[test]
    fn attacker_chases_ball_when_close_to_staging() {
        let attack_goal = Vec2::new(0.75, 0.0);
        let mut coach = RuleBasedCoach::new(attack_goal, Vec2::new(-0.75, 0.0));
        // Pelota en el centro → staging en (-0.16, 0). Robot ya en (-0.18, 0),
        // dentro del attacker_chase_radius=0.10.
        let obs = make_obs_with_robot(Vec2::new(0.0, 0.0), 0, Vec2::new(-0.18, 0.0));
        let choices = coach.decide(&obs);
        let attacker = &choices[0];

        assert_eq!(attacker.skill_id, SkillId::ChaseBall);
    }

    #[test]
    fn support_targets_a_lateral_position_behind_ball() {
        let mut coach = RuleBasedCoach::new(Vec2::new(0.75, 0.0), Vec2::new(-0.75, 0.0));
        let obs = make_obs(0.0, 0.0);
        let choices = coach.decide(&obs);
        let support = &choices[1];

        assert_eq!(support.skill_id, SkillId::GoTo);
        // Pelota en centro, own_goal a la izquierda → support va detrás (x<0)
        // y con offset lateral.
        assert!(support.target.x < 0.0);
    }

    #[test]
    fn goalkeeper_tracks_ball_y_via_goto() {
        let own_goal = Vec2::new(-0.75, 0.0);
        let mut coach = RuleBasedCoach::new(Vec2::new(0.75, 0.0), own_goal);
        let obs = make_obs(0.0, 0.15);
        let choices = coach.decide(&obs);
        let gk = &choices[2];

        assert_eq!(gk.skill_id, SkillId::GoTo);
        assert!((gk.target.y - 0.15).abs() < 0.01);
        // defend_x = -0.75 + 1.0 * 0.12 = -0.63
        assert!((gk.target.x - (-0.63)).abs() < 0.01);
    }

    #[test]
    fn goalkeeper_clamps_ball_y() {
        let mut coach = RuleBasedCoach::new(Vec2::new(0.75, 0.0), Vec2::new(-0.75, 0.0));
        let obs = make_obs(0.0, 0.50);
        let choices = coach.decide(&obs);
        let gk = &choices[2];

        assert!((gk.target.y - 0.20).abs() < 0.01, "debe clampear a 0.20");
    }
}
