//! Coach heurístico STP (skills → tactics → plays) para el equipo de dos caras.
//!
//! Estructura (Plan Equipo Heurístico §4):
//!
//! ```text
//! PLAY  : asignación de roles con histéresis  (Keeper fijo · Striker = el que
//!         llega antes a la pelota con cualquier cara · Support = el otro)
//! TACTIC: por rol, elige la skill por reglas de factibilidad con compromiso
//!         mínimo (no cambia de skill cada tick por ruido de percepción)
//! SKILL : catálogo del engine (`SkillId`), ejecutado por el control loop
//! ```
//!
//! Anti-ruido (decisiones sobre percepción real, no sobre sim limpio):
//! - Cambio de rol solo si el candidato es claramente mejor (`ROLE_SWITCH_GAIN`)
//!   y pasó un tiempo mínimo desde el último cambio (`ROLE_MIN_HOLD`).
//! - Cada skill elegida se mantiene al menos `SKILL_MIN_HOLD` decisiones, salvo
//!   que deje de ser factible (p.ej. `ShootPush` cuando el robot ya no está
//!   detrás de la pelota).
//! - La factibilidad de `ShootPush` usa exactamente la misma regla geométrica
//!   que la skill (`shoot_push_feasible`), así el coach nunca "cree" que
//!   empuja cuando la skill va a devolver stop.
//!
//! El coach solo ve `Observation` (contrato del engine); no toca motion ni radio.

use crate::coach::coach_trait::Coach;
use crate::coach::observation::{FIELD_HALF_X, FIELD_HALF_Y, Observation, RobotObs};
use crate::coach::skill_choice::SkillChoice;
use crate::motion::MotionConfig;
use crate::params::{CoachParams, params};
use crate::skills::{SkillId, shoot_push_feasible_now};
use glam::Vec2;
use std::f32::consts::{FRAC_PI_2, PI};

// ── Geometría del campo (LARC VSSS 3v3, m) ────────────────────────────────────
/// |x| desde el que empieza el área de arco propio/rival (área 70×15 cm).
pub const GOAL_AREA_X: f32 = 0.60;
/// Mitad del ancho del área de arco.
pub const GOAL_AREA_HALF_Y: f32 = 0.35;
/// Mitad del ancho de la boca del arco.
pub const GOAL_HALF_Y: f32 = 0.20;
/// Límite |x| para robots de campo (no entrar al área propia: solo 1 permitido).
const FIELD_ROBOT_MAX_ABS_X: f32 = 0.55;
const FIELD_ROBOT_MAX_ABS_Y: f32 = 0.55;

// Umbrales de decisión: `CoachParams` en `src/params.rs` / `config/team_params.json`
// (histéresis de rol, compromiso por skill, velocidades de referencia, bandas).

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Role {
    Keeper,
    Striker,
    Support,
}

#[derive(Debug, Clone, Copy)]
struct RobotView {
    id: i32,
    pos: Vec2,
    theta: f32,
    active: bool,
}

#[derive(Debug, Clone, Copy)]
struct Committed {
    skill: SkillId,
    held: u32,
}

pub struct HeuristicCoach {
    pub attack_goal: Vec2,
    pub own_goal: Vec2,
    /// Robot que juega de arquero (fijo; solo cambia si queda inactivo).
    pub keeper_id: i32,
    /// Costo de giro plegado a ±90° (robot de dos caras). Se toma de
    /// `MotionConfig::from_env()` para decidir igual que como se ejecuta.
    pub bidirectional: bool,
    /// Umbrales calibrables (ver `config/team_params.json`).
    pub p: CoachParams,
    striker_id: Option<i32>,
    decisions_since_role_switch: u32,
    committed: [Option<Committed>; 3],
    decision_count: u64,
}

impl HeuristicCoach {
    /// Coach de producción: parámetros del JSON vigente y modo de dos caras según
    /// `MotionConfig::from_env()` (decide igual que como se ejecuta).
    pub fn new(attack_goal: Vec2, own_goal: Vec2) -> Self {
        Self::from_params(
            attack_goal,
            own_goal,
            params().coach.clone(),
            MotionConfig::from_env().bidirectional,
        )
    }

    /// Defaults de código con arquero y modo explícitos (tests, herramientas).
    pub fn with_options(attack_goal: Vec2, own_goal: Vec2, keeper_id: i32, bidirectional: bool) -> Self {
        let p = CoachParams {
            keeper_id,
            ..CoachParams::default()
        };
        Self::from_params(attack_goal, own_goal, p, bidirectional)
    }

    pub fn from_params(attack_goal: Vec2, own_goal: Vec2, p: CoachParams, bidirectional: bool) -> Self {
        Self {
            attack_goal,
            own_goal,
            keeper_id: p.keeper_id,
            bidirectional,
            striker_id: None,
            decisions_since_role_switch: p.role_min_hold,
            committed: [None; 3],
            decision_count: 0,
            p,
        }
    }

    /// Rol vigente de cada robot (para GUI/logs). `None` si inactivo.
    pub fn role_of(&self, robot_id: i32) -> Option<Role> {
        if robot_id == self.keeper_id {
            Some(Role::Keeper)
        } else if Some(robot_id) == self.striker_id {
            Some(Role::Striker)
        } else if (0..3).contains(&robot_id) {
            Some(Role::Support)
        } else {
            None
        }
    }

    // ── Desnormalización ─────────────────────────────────────────────────────

    fn ball_pos(obs: &Observation) -> Vec2 {
        Vec2::new(obs.ball.x * FIELD_HALF_X, obs.ball.y * FIELD_HALF_Y)
    }

    fn ball_vel(obs: &Observation) -> Vec2 {
        // MAX_VEL_NORM = 1.5 en observation.rs (privada); replicado aquí.
        Vec2::new(obs.ball.vx * 1.5, obs.ball.vy * 1.5)
    }

    fn view(idx: usize, r: &RobotObs) -> RobotView {
        RobotView {
            id: idx as i32,
            pos: Vec2::new(r.x * FIELD_HALF_X, r.y * FIELD_HALF_Y),
            theta: r.sin_theta.atan2(r.cos_theta),
            active: r.active > 0.5,
        }
    }

    // ── Geometría ────────────────────────────────────────────────────────────

    fn attack_sign(&self) -> f32 {
        if self.attack_goal.x > 0.0 { 1.0 } else { -1.0 }
    }

    /// `true` si `p` está dentro del área de arco propio.
    pub fn in_own_area(&self, p: Vec2) -> bool {
        let s = self.attack_sign();
        // Área propia: x más allá de -s·GOAL_AREA_X en dirección al arco propio.
        (p.x * -s) >= GOAL_AREA_X && p.y.abs() <= GOAL_AREA_HALF_Y
    }

    /// `true` si `p` está en la mitad de cancha propia.
    fn in_own_half(&self, p: Vec2) -> bool {
        p.x * self.attack_sign() < 0.0
    }

    fn clamp_field_robot(&self, mut p: Vec2) -> Vec2 {
        p.x = p.x.clamp(-FIELD_ROBOT_MAX_ABS_X, FIELD_ROBOT_MAX_ABS_X);
        p.y = p.y.clamp(-FIELD_ROBOT_MAX_ABS_Y, FIELD_ROBOT_MAX_ABS_Y);
        p
    }

    fn normalize_angle(a: f32) -> f32 {
        let mut a = a % (2.0 * PI);
        if a > PI {
            a -= 2.0 * PI;
        } else if a < -PI {
            a += 2.0 * PI;
        }
        a
    }

    /// Tiempo estimado para que el robot llegue a `p`: traslación + giro hasta
    /// alinear una cara (plegado a ±90° si es bidireccional).
    fn time_to_reach(&self, r: &RobotView, p: Vec2) -> f32 {
        let d = p - r.pos;
        let dist = d.length();
        let mut turn = 0.0;
        if dist > 0.05 {
            let desired = d.y.atan2(d.x);
            let mut err = Self::normalize_angle(desired - r.theta).abs();
            if self.bidirectional && err > FRAC_PI_2 {
                err = PI - err;
            }
            turn = err / self.p.omega_ref;
        }
        dist / self.p.v_ref + turn
    }

    /// Punto del arco rival al que apuntar: centro, o el palo lejano si hay un
    /// rival parado en el arco.
    fn aim_point(&self, opp: &[RobotView]) -> Vec2 {
        let keeper = opp
            .iter()
            .filter(|o| o.active)
            .filter(|o| (o.pos - self.attack_goal).length() <= self.p.opp_keeper_radius)
            .min_by(|a, b| {
                (a.pos - self.attack_goal)
                    .length()
                    .partial_cmp(&(b.pos - self.attack_goal).length())
                    .unwrap()
            });
        match keeper {
            Some(k) if k.pos.y.abs() > 0.03 => {
                Vec2::new(self.attack_goal.x, -k.pos.y.signum() * (GOAL_HALF_Y - 0.06))
            }
            _ => self.attack_goal,
        }
    }

    /// Punto de despeje desde campo propio: hacia adelante y a la banda del lado
    /// donde está la pelota (saca la pelota del centro de nuestra defensa).
    fn clear_target(&self, ball: Vec2) -> Vec2 {
        let side = if ball.y >= 0.0 { 1.0 } else { -1.0 };
        Vec2::new(self.attack_sign() * 0.30, side * 0.45)
    }

    /// Pelota pegada a una banda lateral.
    fn ball_on_side_wall(&self, ball: Vec2) -> bool {
        ball.y.abs() > self.p.wall_band_y
    }

    /// Pelota en el fondo (línea de gol fuera de la boca del arco) o en la esquina.
    fn ball_on_end_wall(&self, ball: Vec2) -> bool {
        ball.x.abs() > self.p.wall_band_x && ball.y.abs() > GOAL_HALF_Y
    }

    /// Objetivo de empuje del striker según dónde está la pelota. Apuntar al
    /// arco cuando la pelota está en una pared no sirve: la línea de empuje
    /// atraviesa la pared y los robots terminan amontonados contra ella.
    fn striker_target(&self, ball: Vec2, opp: &[RobotView]) -> Vec2 {
        let s = self.attack_sign();
        let side = if ball.y >= 0.0 { 1.0 } else { -1.0 };
        let attacking_end = ball.x * s > 0.0;
        if self.ball_on_end_wall(ball) {
            return if attacking_end {
                // Fondo/esquina rival: sacarla al frente del arco (segunda pelota).
                Vec2::new(s * (GOAL_AREA_X - 0.12), -side * 0.10)
            } else {
                // Fondo/esquina propia: despejar por la banda hacia adelante.
                Vec2::new(s * 0.20, side * 0.50)
            };
        }
        if self.ball_on_side_wall(ball) {
            // Banda: conducir a lo largo de la banda con un ángulo suave hacia
            // adentro, para que la pelota se despegue de la pared camino al arco.
            return Vec2::new(ball.x + s * 0.35, side * 0.42);
        }
        if self.in_own_half(ball) && (ball.x * -s) > 0.35 {
            return self.clear_target(ball);
        }
        self.aim_point(opp)
    }

    // ── Compromiso por skill ─────────────────────────────────────────────────

    /// Aplica el compromiso mínimo: si el robot venía con `prev` y todavía es
    /// aceptable (`still_ok`), la mantiene hasta `SKILL_MIN_HOLD` decisiones.
    fn commit(&mut self, robot_id: i32, wanted: SkillId, still_ok: impl Fn(SkillId) -> bool) -> SkillId {
        let min_hold = self.p.skill_min_hold;
        let slot = &mut self.committed[robot_id as usize];
        let chosen = match slot {
            Some(c) if c.skill != wanted && c.held < min_hold && still_ok(c.skill) => c.skill,
            _ => wanted,
        };
        match slot {
            Some(c) if c.skill == chosen => c.held += 1,
            _ => *slot = Some(Committed { skill: chosen, held: 1 }),
        }
        chosen
    }

    // ── Tácticas ─────────────────────────────────────────────────────────────

    fn striker_choice(&mut self, r: &RobotView, ball: Vec2, ball_vel: Vec2, opp: &[RobotView]) -> SkillChoice {
        let target = self.striker_target(ball, opp);

        let shoot_ok = shoot_push_feasible_now(r.pos, ball, target);
        let ball_speed = ball_vel.length();
        let intercept_min_speed = self.p.intercept_min_ball_speed;
        let ball_coming = ball_speed >= intercept_min_speed
            && (r.pos - ball).normalize_or_zero().dot(ball_vel.normalize_or_zero()) > 0.3;

        let wanted = if shoot_ok {
            SkillId::ShootPush
        } else if ball_coming {
            SkillId::Intercept
        } else {
            SkillId::ApproachAligned
        };
        let skill = self.commit(r.id, wanted, |prev| match prev {
            SkillId::ShootPush => shoot_ok,
            SkillId::Intercept => ball_speed >= intercept_min_speed * 0.5,
            _ => true,
        });
        SkillChoice::new(r.id, skill, target)
    }

    fn support_choice(&mut self, r: &RobotView, ball: Vec2, striker: Option<&RobotView>) -> SkillChoice {
        let defending = self.in_own_half(ball);
        if defending {
            let skill = self.commit(r.id, SkillId::BlockLine, |_| true);
            return SkillChoice::new(r.id, skill, self.own_goal);
        }
        // Pelota en el fondo/esquina rival: el striker la saca al frente del arco;
        // el support espera ahí la "segunda pelota", del lado contrario, fuera del
        // área rival (no amontonarse contra la pared con el striker).
        let s = self.attack_sign();
        if self.ball_on_end_wall(ball) && ball.x * s > 0.0 {
            let side = if ball.y >= 0.0 { 1.0 } else { -1.0 };
            let pos = Vec2::new(s * (GOAL_AREA_X - 0.18), -side * 0.22);
            let skill = self.commit(r.id, SkillId::GoTo, |_| true);
            return SkillChoice::new(r.id, skill, pos);
        }
        // Atacando: posición de recepción/rebote detrás de la pelota, del lado
        // contrario al striker (o al lado libre si no hay striker).
        let to_own = (self.own_goal - ball).normalize_or_zero();
        let lateral_dir = Vec2::new(-to_own.y, to_own.x);
        let side = match striker {
            Some(s) => -((s.pos - ball).dot(lateral_dir)).signum(),
            None => -ball.y.signum(),
        };
        let side = if side == 0.0 { 1.0 } else { side };
        let raw = ball + to_own * 0.30 + lateral_dir * (0.25 * side);
        let mut pos = self.clamp_field_robot(raw);
        if self.in_own_area(pos) {
            pos.x = self.attack_sign() * -(GOAL_AREA_X - 0.08);
        }
        let skill = self.commit(r.id, SkillId::GoTo, |_| true);
        SkillChoice::new(r.id, skill, pos)
    }

    fn keeper_choice(&mut self, r: &RobotView, ball: Vec2, ball_vel: Vec2) -> SkillChoice {
        let ball_parked_in_area =
            self.in_own_area(ball) && ball_vel.length() <= self.p.gk_clear_max_ball_speed;
        if ball_parked_in_area {
            let target = self.clear_target(ball);
            let shoot_ok = shoot_push_feasible_now(r.pos, ball, target);
            let wanted = if shoot_ok { SkillId::ShootPush } else { SkillId::ApproachAligned };
            let skill = self.commit(r.id, wanted, |prev| prev != SkillId::ShootPush || shoot_ok);
            return SkillChoice::new(r.id, skill, target);
        }
        let skill = self.commit(r.id, SkillId::GoalKeep, |_| true);
        SkillChoice::new(r.id, skill, self.own_goal)
    }

    // ── Play: roles ──────────────────────────────────────────────────────────

    fn assign_roles(&mut self, own: &[RobotView], ball: Vec2) {
        self.decisions_since_role_switch = self.decisions_since_role_switch.saturating_add(1);

        // Arquero: fijo; si está inactivo, el activo más cercano al arco propio.
        let keeper_active = own.iter().any(|r| r.active && r.id == self.keeper_id);
        if !keeper_active {
            if let Some(k) = own
                .iter()
                .filter(|r| r.active)
                .min_by(|a, b| {
                    (a.pos - self.own_goal)
                        .length()
                        .partial_cmp(&(b.pos - self.own_goal).length())
                        .unwrap()
                })
            {
                self.keeper_id = k.id;
            }
        }

        // Striker: el jugador de campo que llega antes a la pelota, con histéresis.
        let mut field: Vec<(&RobotView, f32)> = own
            .iter()
            .filter(|r| r.active && r.id != self.keeper_id)
            .map(|r| (r, self.time_to_reach(r, ball)))
            .collect();
        field.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());

        let Some((best, best_t)) = field.first().copied() else {
            self.striker_id = None;
            return;
        };
        match self.striker_id.and_then(|id| field.iter().find(|(r, _)| r.id == id).copied()) {
            None => {
                self.striker_id = Some(best.id);
                self.decisions_since_role_switch = 0;
            }
            Some((cur, cur_t)) if cur.id != best.id => {
                let clearly_better = best_t < cur_t * self.p.role_switch_gain;
                if clearly_better && self.decisions_since_role_switch >= self.p.role_min_hold {
                    self.striker_id = Some(best.id);
                    self.decisions_since_role_switch = 0;
                }
            }
            _ => {}
        }
    }
}

impl Coach for HeuristicCoach {
    fn decide(&mut self, obs: &Observation) -> Vec<SkillChoice> {
        self.decision_count += 1;
        let ball = Self::ball_pos(obs);
        let ball_vel = Self::ball_vel(obs);
        let own: Vec<RobotView> = obs.own_robots.iter().enumerate().map(|(i, r)| Self::view(i, r)).collect();
        let opp: Vec<RobotView> = obs.opp_robots.iter().enumerate().map(|(i, r)| Self::view(i, r)).collect();

        self.assign_roles(&own, ball);
        let striker = self.striker_id.and_then(|id| own.iter().find(|r| r.id == id).copied());

        let mut choices = Vec::with_capacity(3);
        for r in own.iter().filter(|r| r.active) {
            let choice = match self.role_of(r.id) {
                Some(Role::Keeper) => self.keeper_choice(r, ball, ball_vel),
                Some(Role::Striker) => self.striker_choice(r, ball, ball_vel, &opp),
                _ => self.support_choice(r, ball, striker.as_ref()),
            };
            choices.push(choice);
        }
        // Robots inactivos: olvidar su compromiso para que arranquen limpios.
        for r in own.iter().filter(|r| !r.active) {
            self.committed[r.id as usize] = None;
        }
        choices
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::coach::observation::BallObs;

    const ATTACK: Vec2 = Vec2::new(0.75, 0.0);
    const OWN: Vec2 = Vec2::new(-0.75, 0.0);

    fn robot(pos: Vec2, theta_deg: f32) -> RobotObs {
        RobotObs {
            x: pos.x / FIELD_HALF_X,
            y: pos.y / FIELD_HALF_Y,
            vx: 0.0,
            vy: 0.0,
            sin_theta: theta_deg.to_radians().sin(),
            cos_theta: theta_deg.to_radians().cos(),
            omega: 0.0,
            active: 1.0,
        }
    }

    fn obs(ball: Vec2, ball_vel: Vec2, own: [Vec2; 3]) -> Observation {
        Observation {
            ball: BallObs {
                x: ball.x / FIELD_HALF_X,
                y: ball.y / FIELD_HALF_Y,
                vx: ball_vel.x / 1.5,
                vy: ball_vel.y / 1.5,
            },
            own_robots: own.iter().map(|p| robot(*p, 0.0)).collect(),
            opp_robots: vec![RobotObs::default(); 3],
            own_team: 0,
        }
    }

    fn coach() -> HeuristicCoach {
        HeuristicCoach::with_options(ATTACK, OWN, 2, true)
    }

    fn choice_of(choices: &[SkillChoice], id: i32) -> SkillChoice {
        *choices.iter().find(|c| c.robot_id == id).expect("robot sin choice")
    }

    #[test]
    fn closest_field_robot_becomes_striker_and_keeper_keeps_goal() {
        let mut c = coach();
        let o = obs(
            Vec2::new(0.2, 0.0),
            Vec2::ZERO,
            [Vec2::new(-0.5, 0.3), Vec2::new(0.0, -0.1), Vec2::new(-0.63, 0.0)],
        );
        let ch = c.decide(&o);
        assert_eq!(ch.len(), 3);
        assert_eq!(c.role_of(1), Some(Role::Striker));
        assert_eq!(c.role_of(0), Some(Role::Support));
        assert_eq!(choice_of(&ch, 2).skill_id, SkillId::GoalKeep);
        assert_eq!(choice_of(&ch, 2).target, OWN);
    }

    #[test]
    fn striker_shoots_when_behind_ball_else_approaches() {
        let mut c = coach();
        // Robot 1 detrás de la pelota respecto del arco rival → ShootPush.
        let o = obs(
            Vec2::new(0.2, 0.0),
            Vec2::ZERO,
            [Vec2::new(-0.5, 0.3), Vec2::new(0.08, 0.0), Vec2::new(-0.63, 0.0)],
        );
        let ch = c.decide(&o);
        assert_eq!(choice_of(&ch, 1).skill_id, SkillId::ShootPush);
        assert_eq!(choice_of(&ch, 1).target, ATTACK);

        // Robot 1 DELANTE de la pelota → no puede empujar: ApproachAligned.
        let mut c = coach();
        let o = obs(
            Vec2::new(0.2, 0.0),
            Vec2::ZERO,
            [Vec2::new(-0.5, 0.3), Vec2::new(0.32, 0.0), Vec2::new(-0.63, 0.0)],
        );
        let ch = c.decide(&o);
        assert_eq!(choice_of(&ch, 1).skill_id, SkillId::ApproachAligned);
    }

    #[test]
    fn striker_intercepts_fast_incoming_ball() {
        let mut c = coach();
        // Pelota en el medio, viniendo hacia el robot 0 a 0.8 m/s.
        let o = obs(
            Vec2::new(0.0, 0.0),
            Vec2::new(-0.8, 0.0),
            [Vec2::new(-0.4, 0.0), Vec2::new(0.5, 0.4), Vec2::new(-0.63, 0.0)],
        );
        let ch = c.decide(&o);
        assert_eq!(c.role_of(0), Some(Role::Striker));
        assert_eq!(choice_of(&ch, 0).skill_id, SkillId::Intercept);
    }

    #[test]
    fn support_blocks_when_defending_and_positions_when_attacking() {
        let mut c = coach();
        // Pelota en campo propio → support cubre la línea al arco.
        let o = obs(
            Vec2::new(-0.3, 0.2),
            Vec2::ZERO,
            [Vec2::new(-0.2, 0.2), Vec2::new(0.3, -0.3), Vec2::new(-0.63, 0.0)],
        );
        let ch = c.decide(&o);
        assert_eq!(c.role_of(0), Some(Role::Striker));
        let sup = choice_of(&ch, 1);
        assert_eq!(sup.skill_id, SkillId::BlockLine);
        assert_eq!(sup.target, OWN);

        // Pelota en campo rival → support va a una posición de apoyo (GoTo)
        // detrás de la pelota, fuera del área propia y dentro del campo.
        let mut c = coach();
        let o = obs(
            Vec2::new(0.4, 0.1),
            Vec2::ZERO,
            [Vec2::new(0.3, 0.1), Vec2::new(-0.3, -0.3), Vec2::new(-0.63, 0.0)],
        );
        let ch = c.decide(&o);
        let sup = choice_of(&ch, 1);
        assert_eq!(sup.skill_id, SkillId::GoTo);
        assert!(sup.target.x < 0.4, "apoyo detrás de la pelota: {:?}", sup.target);
        assert!(sup.target.x.abs() <= FIELD_ROBOT_MAX_ABS_X + 1e-6);
        assert!(!c.in_own_area(sup.target));
    }

    #[test]
    fn keeper_clears_parked_ball_inside_own_area() {
        let mut c = coach();
        // Pelota quieta dentro del área propia, arquero detrás de ella (más
        // cerca del arco) → ShootPush hacia la banda/adelante.
        let o = obs(
            Vec2::new(-0.62, 0.10),
            Vec2::ZERO,
            [Vec2::new(0.0, 0.3), Vec2::new(0.2, -0.3), Vec2::new(-0.70, 0.12)],
        );
        let ch = c.decide(&o);
        let gk = choice_of(&ch, 2);
        assert_eq!(gk.skill_id, SkillId::ShootPush);
        assert!(gk.target.x > -0.62, "despeje hacia adelante: {:?}", gk.target);
        assert!(gk.target.y > 0.0, "despeje al lado de la pelota");
    }

    #[test]
    fn striker_role_has_hysteresis() {
        let mut c = coach();
        // Robot 0 apenas más cerca que robot 1 → 0 es striker.
        let o = obs(
            Vec2::new(0.0, 0.0),
            Vec2::ZERO,
            [Vec2::new(-0.30, 0.0), Vec2::new(0.32, 0.0), Vec2::new(-0.63, 0.0)],
        );
        c.decide(&o);
        assert_eq!(c.role_of(0), Some(Role::Striker));
        // Ahora robot 1 queda apenas más cerca (ruido típico): NO debe cambiar.
        let o = obs(
            Vec2::new(0.0, 0.0),
            Vec2::ZERO,
            [Vec2::new(-0.32, 0.0), Vec2::new(0.30, 0.0), Vec2::new(-0.63, 0.0)],
        );
        for _ in 0..10 {
            c.decide(&o);
        }
        assert_eq!(c.role_of(0), Some(Role::Striker), "flip-flop de rol por ruido");
        // Robot 1 claramente más cerca y pasó el hold → sí cambia.
        let o = obs(
            Vec2::new(0.0, 0.0),
            Vec2::ZERO,
            [Vec2::new(-0.60, 0.0), Vec2::new(0.10, 0.0), Vec2::new(-0.63, 0.0)],
        );
        for _ in 0..CoachParams::default().role_min_hold + 1 {
            c.decide(&o);
        }
        assert_eq!(c.role_of(1), Some(Role::Striker));
    }

    #[test]
    fn skill_commitment_survives_short_flicker() {
        let mut c = coach();
        // Striker detrás de la pelota → ShootPush.
        let behind = obs(
            Vec2::new(0.2, 0.0),
            Vec2::ZERO,
            [Vec2::new(-0.5, 0.3), Vec2::new(0.08, 0.0), Vec2::new(-0.63, 0.0)],
        );
        assert_eq!(choice_of(&c.decide(&behind), 1).skill_id, SkillId::ShootPush);
        // Un tick de percepción dice que la pelota se movió delante del robot:
        // ShootPush deja de ser factible → cambia de inmediato (no se finge empuje).
        let flicker = obs(
            Vec2::new(0.2, 0.0),
            Vec2::ZERO,
            [Vec2::new(-0.5, 0.3), Vec2::new(0.30, 0.0), Vec2::new(-0.63, 0.0)],
        );
        assert_eq!(choice_of(&c.decide(&flicker), 1).skill_id, SkillId::ApproachAligned);
        // Vuelve a estar detrás: ApproachAligned se mantiene el hold mínimo
        // aunque ShootPush ya sea factible otra vez (evita el ping-pong).
        assert_eq!(choice_of(&c.decide(&behind), 1).skill_id, SkillId::ApproachAligned);
        let mut last = SkillId::ApproachAligned;
        for _ in 0..CoachParams::default().skill_min_hold {
            last = choice_of(&c.decide(&behind), 1).skill_id;
        }
        assert_eq!(last, SkillId::ShootPush);
    }

    #[test]
    fn two_faced_cost_prefers_robot_with_back_to_ball() {
        // Robot 0 está de espaldas a la pelota; robot 1 igual de lejos pero de
        // costado (90°). Con dos caras, 0 no necesita girar → llega antes.
        let mut o = obs(
            Vec2::new(0.0, 0.0),
            Vec2::ZERO,
            [Vec2::new(-0.40, 0.0), Vec2::new(0.40, 0.0), Vec2::new(-0.63, 0.0)],
        );
        o.own_robots[0] = robot(Vec2::new(-0.40, 0.0), 180.0); // mira a -x, pelota a +x
        o.own_robots[1] = robot(Vec2::new(0.40, 0.0), 90.0); // de costado
        let mut bidir = HeuristicCoach::with_options(ATTACK, OWN, 2, true);
        bidir.decide(&o);
        assert_eq!(bidir.role_of(0), Some(Role::Striker));

        let mut mono = HeuristicCoach::with_options(ATTACK, OWN, 2, false);
        mono.decide(&o);
        assert_eq!(mono.role_of(1), Some(Role::Striker), "frontal: 180° cuesta más que 90°");
    }

    #[test]
    fn yellow_team_mirrors_geometry() {
        let mut c = HeuristicCoach::with_options(OWN, ATTACK, 2, true); // ataca a -x
        let o = obs(
            Vec2::new(-0.2, 0.0),
            Vec2::ZERO,
            [Vec2::new(0.5, 0.3), Vec2::new(-0.08, 0.0), Vec2::new(0.63, 0.0)],
        );
        let ch = c.decide(&o);
        assert_eq!(choice_of(&ch, 1).skill_id, SkillId::ShootPush);
        assert_eq!(choice_of(&ch, 1).target, OWN);
        assert_eq!(choice_of(&ch, 2).target, ATTACK);
        assert!(c.in_own_area(Vec2::new(0.70, 0.1)));
        assert!(!c.in_own_area(Vec2::new(-0.70, 0.1)));
    }

    #[test]
    fn striker_drives_along_side_wall_instead_of_aiming_at_goal() {
        let c = coach();
        let ball = Vec2::new(0.0, 0.58);
        let t = c.striker_target(ball, &[]);
        assert!(t.x > ball.x, "debe empujar hacia adelante: {t:?}");
        assert!(t.y > 0.0 && t.y < ball.y, "ángulo suave hacia adentro: {t:?}");
        // Sin pared: apunta al arco.
        assert_eq!(c.striker_target(Vec2::new(0.0, 0.2), &[]), ATTACK);
    }

    #[test]
    fn attacking_corner_pulls_ball_to_goal_front_and_support_waits_opposite() {
        let mut c = coach();
        let ball = Vec2::new(0.68, 0.58);
        let t = c.striker_target(ball, &[]);
        assert!(t.x < GOAL_AREA_X && t.x > 0.3, "al frente del arco: {t:?}");
        assert!(t.y < 0.0, "hacia el lado contrario de la esquina: {t:?}");

        let o = obs(
            ball,
            Vec2::ZERO,
            [Vec2::new(0.60, 0.50), Vec2::new(0.2, 0.0), Vec2::new(-0.63, 0.0)],
        );
        let ch = c.decide(&o);
        assert_eq!(c.role_of(0), Some(Role::Striker));
        let sup = choice_of(&ch, 1);
        assert_eq!(sup.skill_id, SkillId::GoTo);
        assert!(sup.target.y < 0.0, "support del lado contrario: {:?}", sup.target);
        assert!(sup.target.x < GOAL_AREA_X, "support fuera del área rival: {:?}", sup.target);
    }

    #[test]
    fn own_corner_clears_forward_along_wall() {
        let c = coach();
        let t = c.striker_target(Vec2::new(-0.68, -0.58), &[]);
        assert!(t.x > -0.68, "despeje hacia adelante: {t:?}");
        assert!(t.y < 0.0, "por la misma banda: {t:?}");
    }

    #[test]
    fn params_drive_keeper_and_hysteresis() {
        let p = CoachParams {
            keeper_id: 0,
            role_min_hold: 0,
            role_switch_gain: 1.0,
            ..CoachParams::default()
        };
        let mut c = HeuristicCoach::from_params(ATTACK, OWN, p, true);
        let o = obs(
            Vec2::new(0.2, 0.0),
            Vec2::ZERO,
            [Vec2::new(-0.63, 0.0), Vec2::new(0.0, 0.0), Vec2::new(-0.5, 0.3)],
        );
        let ch = c.decide(&o);
        assert_eq!(c.role_of(0), Some(Role::Keeper));
        assert_eq!(choice_of(&ch, 0).skill_id, SkillId::GoalKeep);
        assert_eq!(c.role_of(1), Some(Role::Striker));
        // Sin histéresis (hold 0, gain 1.0): el rol cambia apenas otro está más cerca.
        let o = obs(
            Vec2::new(0.2, 0.0),
            Vec2::ZERO,
            [Vec2::new(-0.63, 0.0), Vec2::new(-0.5, 0.0), Vec2::new(0.1, 0.0)],
        );
        c.decide(&o);
        assert_eq!(c.role_of(2), Some(Role::Striker));
    }

    #[test]
    fn inactive_keeper_is_replaced_by_closest_to_goal() {
        let mut c = coach();
        let mut o = obs(
            Vec2::new(0.2, 0.0),
            Vec2::ZERO,
            [Vec2::new(-0.5, 0.0), Vec2::new(0.1, 0.0), Vec2::new(-0.63, 0.0)],
        );
        o.own_robots[2] = RobotObs::default(); // arquero fuera
        let ch = c.decide(&o);
        assert_eq!(ch.len(), 2);
        assert_eq!(c.role_of(0), Some(Role::Keeper));
        assert_eq!(choice_of(&ch, 0).skill_id, SkillId::GoalKeep);
    }
}
