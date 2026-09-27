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

use crate::coach::adaptation::{Adaptation, Lane};
use crate::coach::coach_trait::Coach;
use crate::coach::observation::{FIELD_HALF_X, FIELD_HALF_Y, Observation, RobotObs};
use crate::coach::plays::{Play, formation};
use crate::coach::referee::SharedReferee;
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
    /// Decisión en la que la pelota entró al área propia (para el límite de 10 s).
    ball_in_area_since: Option<u64>,
    /// Estado del árbitro (VSSReferee / operador). `None` = siempre juego abierto.
    referee: Option<SharedReferee>,
    /// Play vigente (derivada del último comando del árbitro).
    current_play: Play,
    /// Últimos scores del striker (skill, score) — para logs/GUI y tests.
    last_scores: [(SkillId, f32); 5],
    /// Contadores de adaptación al rival (éxito de tiro por carril, disputas).
    pub adaptation: Adaptation,
    /// Ya se comparó el lado configurado con la posición inicial del equipo.
    side_checked: bool,
}

fn clamp01(x: f32) -> f32 {
    x.clamp(0.0, 1.0)
}

/// Opciones que compiten por el striker (orden fijo de `last_scores`).
pub const STRIKER_OPTIONS: [SkillId; 5] = [
    SkillId::ShootPush,
    SkillId::Clear,
    SkillId::SpinKick,
    SkillId::Intercept,
    SkillId::ApproachAligned,
];

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
            ball_in_area_since: None,
            referee: None,
            current_play: Play::Open,
            last_scores: [
                (SkillId::ShootPush, 0.0),
                (SkillId::Clear, 0.0),
                (SkillId::SpinKick, 0.0),
                (SkillId::Intercept, 0.0),
                (SkillId::ApproachAligned, 0.0),
            ],
            adaptation: Adaptation::new(p.adapt_enabled, p.adapt_alpha),
            side_checked: false,
            p,
        }
    }

    /// Scores de la última decisión del striker, en el orden de `STRIKER_OPTIONS`.
    pub fn last_striker_scores(&self) -> &[(SkillId, f32); 5] {
        &self.last_scores
    }

    /// Conecta el estado del árbitro (listener de `coach::referee`).
    pub fn set_referee(&mut self, shared: SharedReferee) {
        self.referee = Some(shared);
    }

    pub fn current_play(&self) -> Play {
        self.current_play
    }

    /// Equipo propio según el lado de ataque (convención del engine: azul ataca +x).
    fn own_team_id(&self) -> i32 {
        if self.attack_goal.x > 0.0 { 0 } else { 1 }
    }

    /// Play según el último comando del árbitro (juego abierto si no hay árbitro).
    fn read_play(&self) -> Play {
        match self.referee.as_ref().and_then(|r| r.lock().ok().map(|s| s.command)) {
            Some(cmd) => Play::from_command(&cmd, self.own_team_id()),
            None => Play::Open,
        }
    }

    /// Decisiones de una play de pelota parada: cada rol va a su punto de la
    /// formación mirando a la pelota (`Mark`); el arquero sigue en la línea salvo
    /// que la play le dé un punto explícito (goal kick propio: lo saca él).
    fn set_piece_choices(&mut self, play: Play, own: &[RobotView], opp: &[RobotView]) -> Vec<SkillChoice> {
        let s = self.attack_sign();
        let aim = self.aim_point(opp);
        let staging = params().skills.approach_staging_offset;
        let Some(f) = formation(play, s, aim, staging) else {
            return Vec::new();
        };
        // El pateador es el jugador de campo que llega antes a su punto.
        self.assign_roles(own, f.striker);
        own.iter()
            .filter(|r| r.active)
            .map(|r| match self.role_of(r.id) {
                Some(Role::Keeper) => match f.keeper {
                    Some(p) => SkillChoice::new(r.id, SkillId::Mark, p),
                    None => SkillChoice::new(r.id, SkillId::GoalKeep, self.own_goal),
                },
                Some(Role::Striker) => SkillChoice::new(r.id, SkillId::Mark, f.striker),
                _ => SkillChoice::new(r.id, SkillId::Mark, f.support),
            })
            .collect()
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
        let post_y = GOAL_HALF_Y - 0.06;
        match keeper {
            Some(k) if k.pos.y.abs() > 0.03 => Vec2::new(self.attack_goal.x, -k.pos.y.signum() * post_y),
            // Sin arquero rival visible: al centro, salvo que los contadores digan
            // que por el centro no está entrando y otro carril rinde mejor.
            _ => Vec2::new(self.attack_goal.x, self.adaptation.preferred_aim_y(0.0, post_y)),
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

    /// `true` si un rival está sobre el segmento pelota→objetivo (tapa el tiro).
    fn shot_blocked(ball: Vec2, target: Vec2, opp: &[RobotView]) -> bool {
        let seg = target - ball;
        let len = seg.length();
        if len < 1e-3 {
            return false;
        }
        let dir = seg / len;
        opp.iter().filter(|o| o.active).any(|o| {
            let rel = o.pos - ball;
            let along = rel.dot(dir);
            let lateral = (rel - dir * along).length();
            along > 0.05 && along < len && lateral < 0.09
        })
    }

    /// Scores de viabilidad 0..1 de cada opción del striker (estilo TIGERs:
    /// puntajes hechos a mano, ajustables sin reescribir reglas). Orden =
    /// `STRIKER_OPTIONS`.
    fn striker_scores(
        &self,
        r: &RobotView,
        ball: Vec2,
        ball_vel: Vec2,
        opp: &[RobotView],
        target: Vec2,
    ) -> [(SkillId, f32); 5] {
        let p = &self.p;
        let s = self.attack_sign();
        let sp = &params().skills;
        let ball_speed = ball_vel.length();
        let dist_ball = (ball - r.pos).length();
        let min_opp_dist = opp
            .iter()
            .filter(|o| o.active)
            .map(|o| (o.pos - ball).length())
            .fold(f32::INFINITY, f32::min);

        // ShootPush: factible (detrás y cerca) × alineación con la línea de empuje,
        // penalizado si un rival tapa el tiro.
        let shoot = if shoot_push_feasible_now(r.pos, ball, target) {
            let dir = (target - ball).normalize_or_zero();
            let rel = r.pos - ball;
            let lateral = (rel - dir * rel.dot(dir)).length();
            let align = clamp01(1.0 - lateral / 0.10);
            let blocked = if Self::shot_blocked(ball, target, opp) {
                p.score_shot_blocked_factor
            } else {
                1.0
            };
            let lane = Lane::of(ball.y, self.adaptation.lane_half_width);
            (0.6 + 0.4 * align) * blocked * self.adaptation.shot_factor(lane)
        } else {
            0.0
        };

        // Clear: cuanto más cerca de nuestro arco está la pelota y más presión
        // rival hay, más urge sacarla. Cero en campo rival.
        let depth = ball.x * -s; // distancia "hacia nuestro arco" desde el centro
        let danger = clamp01((depth - p.clear_zone_start_x) / (p.clear_zone_full_x - p.clear_zone_start_x).max(1e-3));
        let pressure = clamp01(1.0 - min_opp_dist / 0.30);
        let clear = if self.in_own_half(ball) { danger * (0.6 + 0.4 * pressure) } else { 0.0 };

        // SpinKick: pelota quieta y disputada (rival encima) o en el fondo/esquina,
        // con el robot cerca; pierde valor si igual se puede empujar.
        let close = clamp01(1.0 - (dist_ball - 0.08) / p.spin_engage_radius.max(1e-3));
        let slow: f32 = if ball_speed < p.intercept_min_ball_speed { 1.0 } else { 0.0 };
        let contested: f32 = if min_opp_dist < p.spin_when_opponent_within { 1.0 } else { 0.0 };
        let wall: f32 = if self.ball_on_end_wall(ball) {
            1.0
        } else if self.ball_on_side_wall(ball) {
            0.5
        } else {
            0.0
        };
        let spin = close
            * slow
            * contested.max(wall)
            * if shoot > 0.0 { 0.3 } else { 1.0 }
            * self.adaptation.spin_factor();

        // Intercept: pelota rápida y viniendo hacia el robot.
        let vmin = p.intercept_min_ball_speed.max(1e-3);
        let speed_term = clamp01((ball_speed - 0.5 * vmin) / vmin);
        let coming = clamp01((r.pos - ball).normalize_or_zero().dot(ball_vel.normalize_or_zero()));
        let intercept = speed_term * coming;

        // ApproachAligned: fallback con score base (las demás deben superarlo); algo
        // menos atractivo si el staging está lejos (la pelota "se va").
        let staging_far = clamp01(dist_ball / (2.0 * sp.approach_staging_offset + 0.6));
        let approach = p.score_approach_base * (1.0 - 0.3 * staging_far);

        [
            (SkillId::ShootPush, clamp01(shoot)),
            (SkillId::Clear, clamp01(clear)),
            (SkillId::SpinKick, clamp01(spin)),
            (SkillId::Intercept, clamp01(intercept)),
            (SkillId::ApproachAligned, clamp01(approach)),
        ]
    }

    /// Elige la opción de mayor score con histéresis: la opción vigente (si sigue
    /// viable) recibe `score_hysteresis` de bono.
    fn pick_with_hysteresis(&self, scores: &[(SkillId, f32); 5], current: Option<SkillId>) -> SkillId {
        let mut best = SkillId::ApproachAligned;
        let mut best_score = f32::NEG_INFINITY;
        for (skill, score) in scores {
            let bonus = if Some(*skill) == current && *score > 0.0 {
                self.p.score_hysteresis
            } else {
                0.0
            };
            let v = score + bonus;
            if v > best_score {
                best = *skill;
                best_score = v;
            }
        }
        best
    }

    fn striker_choice(&mut self, r: &RobotView, ball: Vec2, ball_vel: Vec2, opp: &[RobotView]) -> SkillChoice {
        let target = self.striker_target(ball, opp);
        let scores = self.striker_scores(r, ball, ball_vel, opp, target);
        self.last_scores = scores;
        let current = self.committed[r.id as usize].map(|c| c.skill);
        let wanted = self.pick_with_hysteresis(&scores, current);
        // Compromiso mínimo: la opción vigente se mantiene mientras siga viable (score > 0).
        let skill = self.commit(r.id, wanted, |prev| {
            scores.iter().any(|(s, v)| *s == prev && *v > 0.0)
        });
        SkillChoice::new(r.id, skill, target)
    }

    /// Rival que amenaza en nuestra mitad sin ser el que está sobre la pelota
    /// (el "segundo atacante" de un contraataque). Se elige el más cercano a
    /// nuestro arco.
    fn threat(&self, ball: Vec2, opp: &[RobotView]) -> Option<RobotView> {
        let on_ball = opp
            .iter()
            .filter(|o| o.active)
            .min_by(|a, b| {
                (a.pos - ball)
                    .length()
                    .partial_cmp(&(b.pos - ball).length())
                    .unwrap()
            })
            .map(|o| o.id);
        opp.iter()
            .filter(|o| o.active && Some(o.id) != on_ball && self.in_own_half(o.pos))
            .min_by(|a, b| {
                (a.pos - self.own_goal)
                    .length()
                    .partial_cmp(&(b.pos - self.own_goal).length())
                    .unwrap()
            })
            .copied()
    }

    /// Punto de marca: entre el rival y nuestro arco, a `mark_distance` del rival,
    /// fuera del área propia.
    fn mark_point(&self, threat: Vec2) -> Vec2 {
        let raw = threat + (self.own_goal - threat).normalize_or_zero() * self.p.mark_distance;
        let mut pos = self.clamp_field_robot(raw);
        if self.in_own_area(pos) {
            pos.x = self.attack_sign() * -(GOAL_AREA_X - 0.08);
        }
        pos
    }

    fn support_choice(
        &mut self,
        r: &RobotView,
        ball: Vec2,
        striker: Option<&RobotView>,
        opp: &[RobotView],
    ) -> SkillChoice {
        let defending = self.in_own_half(ball);
        if defending {
            let skill = self.commit(r.id, SkillId::BlockLine, |_| true);
            return SkillChoice::new(r.id, skill, self.own_goal);
        }
        // Atacando con un rival suelto en nuestra mitad: marcarlo (entre él y el
        // arco, mirando a la pelota) en vez de subir a apoyar.
        if let Some(t) = self.threat(ball, opp) {
            let pos = self.mark_point(t.pos);
            let skill = self.commit(r.id, SkillId::Mark, |_| true);
            return SkillChoice::new(r.id, skill, pos);
        }
        // Pelota en el fondo/esquina rival: el striker la saca al frente del arco;
        // el support espera ahí la "segunda pelota", del lado contrario, fuera del
        // área rival (no amontonarse contra la pared con el striker).
        let s = self.attack_sign();
        if self.ball_on_end_wall(ball) && ball.x * s > 0.0 {
            let side = if ball.y >= 0.0 { 1.0 } else { -1.0 };
            let pos = Vec2::new(s * (GOAL_AREA_X - 0.18), -side * 0.22);
            let skill = self.commit(r.id, SkillId::Mark, |_| true);
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
        // Mark = ir al punto y quedar mirando a la pelota (listo para recibir).
        let skill = self.commit(r.id, SkillId::Mark, |_| true);
        SkillChoice::new(r.id, skill, pos)
    }

    /// Segundos que la pelota lleva dentro del área propia (0 si no está).
    pub fn ball_seconds_in_own_area(&self) -> f32 {
        self.ball_in_area_since
            .map(|t0| (self.decision_count.saturating_sub(t0)) as f32 / self.p.decision_hz.max(1e-3))
            .unwrap_or(0.0)
    }

    fn keeper_choice(&mut self, r: &RobotView, ball: Vec2, ball_vel: Vec2) -> SkillChoice {
        let in_area = self.in_own_area(ball);
        if in_area {
            if self.ball_in_area_since.is_none() {
                self.ball_in_area_since = Some(self.decision_count);
            }
        } else {
            self.ball_in_area_since = None;
        }
        // Despejar si la pelota quedó parada en el área, o sí o sí antes del
        // límite reglamentario de retención (10 s) aunque siga moviéndose.
        let parked = in_area && ball_vel.length() <= self.p.gk_clear_max_ball_speed;
        let overdue = in_area && self.ball_seconds_in_own_area() >= self.p.gk_force_clear_s;
        if parked || overdue {
            let target = self.clear_target(ball);
            let skill = self.commit(r.id, SkillId::Clear, |_| true);
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

        // Chequeo de lado (una vez, con el equipo completo visible): si nuestros
        // robots están en la mitad del arco "rival", el lado configurado está al
        // revés (VSSL_SIDE) y atacaríamos nuestro propio arco.
        if !self.side_checked {
            let active: Vec<&RobotView> = own.iter().filter(|r| r.active).collect();
            if active.len() >= 3 {
                let mean_x = active.iter().map(|r| r.pos.x).sum::<f32>() / active.len() as f32;
                if mean_x.abs() > 0.10 {
                    let on_own_side = mean_x * self.attack_sign() < 0.0;
                    if !on_own_side {
                        eprintln!(
                            "[coach] ⚠ nuestros robots están en la mitad del arco RIVAL (x medio {mean_x:.2}, \
                             atacamos hacia {:+}X): revisa VSSL_SIDE (left|right) o el lado en el simulador",
                            self.attack_sign() as i32
                        );
                    } else {
                        eprintln!("[coach] lado verificado: robots propios en su mitad (x medio {mean_x:.2})");
                    }
                    self.side_checked = true;
                }
            }
        }

        // Árbitro: al cambiar de play se olvidan los compromisos de skill para que
        // el silbato (GAME_ON) tenga efecto inmediato.
        let play = self.read_play();
        if play != self.current_play {
            self.committed = [None; 3];
            self.current_play = play;
        }
        match play {
            Play::Hold => {
                return own
                    .iter()
                    .filter(|r| r.active)
                    .map(|r| SkillChoice::new(r.id, SkillId::Hold, Vec2::ZERO))
                    .collect();
            }
            p if p.is_set_piece() => return self.set_piece_choices(p, &own, &opp),
            Play::Open => {}
            Play::KickoffOurs
            | Play::KickoffTheirs
            | Play::FreeBall(_)
            | Play::PenaltyOurs
            | Play::PenaltyTheirs
            | Play::FreeKickOurs
            | Play::FreeKickTheirs
            | Play::GoalKickOurs
            | Play::GoalKickTheirs => {}
        }

        self.assign_roles(&own, ball);
        let striker = self.striker_id.and_then(|id| own.iter().find(|r| r.id == id).copied());
        // Pelota dentro del área propia con arquero activo: la despeja él (§9.5,
        // solo un robot en el área); los de campo cubren la línea desde afuera.
        let keeper_active = own.iter().any(|r| r.active && r.id == self.keeper_id);
        let ball_in_own_area = self.in_own_area(ball) && keeper_active;

        let mut choices = Vec::with_capacity(3);
        for r in own.iter().filter(|r| r.active) {
            let choice = match self.role_of(r.id) {
                Some(Role::Keeper) => self.keeper_choice(r, ball, ball_vel),
                Some(Role::Striker) if ball_in_own_area => {
                    let skill = self.commit(r.id, SkillId::BlockLine, |_| true);
                    SkillChoice::new(r.id, skill, self.own_goal)
                }
                Some(Role::Striker) => self.striker_choice(r, ball, ball_vel, &opp),
                _ => self.support_choice(r, ball, striker.as_ref(), &opp),
            };
            choices.push(choice);
        }

        // Contadores de adaptación: qué eligió el striker y quién está sobre la pelota.
        let striker_skill = self
            .striker_id
            .and_then(|id| choices.iter().find(|c| c.robot_id == id))
            .map(|c| c.skill_id);
        let nearest = |robots: &[RobotView]| {
            robots
                .iter()
                .filter(|r| r.active)
                .map(|r| (r.pos - ball).length())
                .fold(f32::INFINITY, f32::min)
        };
        let s = self.attack_sign();
        self.adaptation.observe(
            self.decision_count,
            ball,
            ball_vel,
            s,
            striker_skill == Some(SkillId::ShootPush),
            striker_skill == Some(SkillId::SpinKick),
            nearest(&own),
            nearest(&opp),
        );
        if self.decision_count.is_multiple_of(600) && self.adaptation.enabled {
            eprintln!("[coach] adaptación: {}", self.adaptation.summary());
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

        // Pelota en campo rival → support va a una posición de apoyo (Mark: llega y
        // mira a la pelota) detrás de la pelota, fuera del área propia y en el campo.
        let mut c = coach();
        let o = obs(
            Vec2::new(0.4, 0.1),
            Vec2::ZERO,
            [Vec2::new(0.3, 0.1), Vec2::new(-0.3, -0.3), Vec2::new(-0.63, 0.0)],
        );
        let ch = c.decide(&o);
        let sup = choice_of(&ch, 1);
        assert_eq!(sup.skill_id, SkillId::Mark);
        assert!(sup.target.x < 0.4, "apoyo detrás de la pelota: {:?}", sup.target);
        assert!(sup.target.x.abs() <= FIELD_ROBOT_MAX_ABS_X + 1e-6);
        assert!(!c.in_own_area(sup.target));
    }

    #[test]
    fn keeper_clears_parked_ball_inside_own_area() {
        let mut c = coach();
        // Pelota quieta dentro del área propia → Clear hacia la banda/adelante.
        let o = obs(
            Vec2::new(-0.62, 0.10),
            Vec2::ZERO,
            [Vec2::new(0.0, 0.3), Vec2::new(0.2, -0.3), Vec2::new(-0.70, 0.12)],
        );
        let ch = c.decide(&o);
        let gk = choice_of(&ch, 2);
        assert_eq!(gk.skill_id, SkillId::Clear);
        assert!(gk.target.x > -0.62, "despeje hacia adelante: {:?}", gk.target);
        assert!(gk.target.y > 0.0, "despeje al lado de la pelota");
    }

    #[test]
    fn field_players_stay_out_when_ball_is_in_own_area() {
        let mut c = coach();
        // Pelota quieta dentro del área propia: arquero despeja, striker cubre la
        // línea desde afuera en vez de entrar (falta de área).
        let o = obs(
            Vec2::new(-0.64, 0.05),
            Vec2::ZERO,
            [Vec2::new(-0.45, 0.05), Vec2::new(0.2, -0.3), Vec2::new(-0.70, 0.0)],
        );
        let ch = c.decide(&o);
        assert_eq!(c.role_of(0), Some(Role::Striker));
        assert_eq!(choice_of(&ch, 0).skill_id, SkillId::BlockLine);
        assert_eq!(choice_of(&ch, 2).skill_id, SkillId::Clear);
        // Sin arquero activo, el striker sí va por la pelota.
        let mut o2 = o.clone();
        o2.own_robots[2] = RobotObs::default();
        let mut c2 = coach();
        let ch2 = c2.decide(&o2);
        assert_ne!(choice_of(&ch2, 0).skill_id, SkillId::BlockLine);
    }

    #[test]
    fn keeper_forces_clear_before_retention_limit() {
        let p = CoachParams {
            gk_force_clear_s: 1.0,
            decision_hz: 10.0,
            ..CoachParams::default()
        };
        let mut c = HeuristicCoach::from_params(ATTACK, OWN, p, true);
        // Pelota moviéndose dentro del área (no "parada") → GoalKeep al principio...
        let o = obs(
            Vec2::new(-0.65, 0.05),
            Vec2::new(0.0, 0.3),
            [Vec2::new(0.0, 0.3), Vec2::new(0.2, -0.3), Vec2::new(-0.70, 0.0)],
        );
        assert_eq!(choice_of(&c.decide(&o), 2).skill_id, SkillId::GoalKeep);
        // ...pero tras 1 s dentro del área, despeja igual.
        let mut last = SkillId::GoalKeep;
        for _ in 0..12 {
            last = choice_of(&c.decide(&o), 2).skill_id;
        }
        assert_eq!(last, SkillId::Clear);
        assert!(c.ball_seconds_in_own_area() >= 1.0);
        // La pelota sale del área → el contador se reinicia.
        let out = obs(
            Vec2::new(0.1, 0.0),
            Vec2::ZERO,
            [Vec2::new(0.0, 0.3), Vec2::new(0.2, -0.3), Vec2::new(-0.63, 0.0)],
        );
        c.decide(&out);
        assert_eq!(c.ball_seconds_in_own_area(), 0.0);
    }

    #[test]
    fn striker_spin_kicks_when_opponent_sits_on_ball() {
        let mut c = coach();
        let ball = Vec2::new(0.1, 0.0);
        let mut o = obs(
            ball,
            Vec2::ZERO,
            [Vec2::new(0.25, 0.05), Vec2::new(-0.4, -0.3), Vec2::new(-0.63, 0.0)],
        );
        o.opp_robots[0] = robot(Vec2::new(0.16, 0.0), 180.0); // rival encima de la pelota
        let ch = c.decide(&o);
        assert_eq!(c.role_of(0), Some(Role::Striker));
        assert_eq!(choice_of(&ch, 0).skill_id, SkillId::SpinKick);
        assert_eq!(choice_of(&ch, 0).target, ATTACK);
    }

    #[test]
    fn striker_clears_from_own_defensive_third() {
        let mut c = coach();
        let o = obs(
            Vec2::new(-0.5, 0.2),
            Vec2::ZERO,
            [Vec2::new(-0.3, 0.2), Vec2::new(0.3, -0.3), Vec2::new(-0.63, 0.0)],
        );
        let ch = c.decide(&o);
        let st = choice_of(&ch, 0);
        assert_eq!(st.skill_id, SkillId::Clear);
        assert!(st.target.x > -0.5 && st.target.y > 0.0, "despeje por la banda: {:?}", st.target);
    }

    #[test]
    fn support_marks_loose_opponent_in_own_half_while_attacking() {
        let mut c = coach();
        let ball = Vec2::new(0.4, 0.1);
        let mut o = obs(
            ball,
            Vec2::ZERO,
            [Vec2::new(0.3, 0.1), Vec2::new(-0.2, -0.3), Vec2::new(-0.63, 0.0)],
        );
        o.opp_robots[0] = robot(Vec2::new(0.45, 0.15), 180.0); // rival sobre la pelota
        o.opp_robots[1] = robot(Vec2::new(-0.35, 0.25), 180.0); // rival suelto en nuestra mitad
        let ch = c.decide(&o);
        let sup = choice_of(&ch, 1);
        assert_eq!(sup.skill_id, SkillId::Mark);
        // Entre el rival y nuestro arco: más cerca del arco que el rival, y sobre su línea.
        assert!(sup.target.x < -0.35, "{:?}", sup.target);
        let to_goal = (OWN - Vec2::new(-0.35, 0.25)).normalize();
        let to_mark = (sup.target - Vec2::new(-0.35, 0.25)).normalize();
        assert!(to_goal.dot(to_mark) > 0.99, "{:?}", sup.target);
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
        assert_eq!(sup.skill_id, SkillId::Mark);
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
    fn referee_drives_hold_set_pieces_and_game_on() {
        use crate::coach::referee::{Foul, Quadrant, RefTeam, RefereeCommand, apply_command, new_shared_referee};
        let shared = new_shared_referee();
        let mut c = coach(); // azul, ataca +x
        c.set_referee(shared.clone());
        let o = obs(
            Vec2::new(0.0, 0.0),
            Vec2::ZERO,
            [Vec2::new(-0.3, 0.1), Vec2::new(-0.4, -0.3), Vec2::new(-0.63, 0.0)],
        );
        // Sin comando: juego abierto (táctica normal).
        assert_eq!(c.current_play(), Play::Open);
        assert_ne!(choice_of(&c.decide(&o), 0).skill_id, SkillId::Hold);

        // HALT → todos quietos.
        apply_command(&shared, RefereeCommand { foul: Foul::Halt, ..RefereeCommand::GAME_ON });
        let ch = c.decide(&o);
        assert_eq!(c.current_play(), Play::Hold);
        assert!(ch.iter().all(|x| x.skill_id == SkillId::Hold));

        // KICKOFF nuestro → pateador (el más cercano) dentro del círculo detrás de la
        // pelota, support en nuestra mitad, arquero en la línea.
        apply_command(
            &shared,
            RefereeCommand { foul: Foul::Kickoff, team: RefTeam::Blue, ..RefereeCommand::GAME_ON },
        );
        let ch = c.decide(&o);
        assert_eq!(c.current_play(), Play::KickoffOurs);
        assert_eq!(c.role_of(0), Some(Role::Striker));
        let k = choice_of(&ch, 0);
        assert_eq!(k.skill_id, SkillId::Mark);
        assert!(k.target.length() < 0.2 && k.target.x < 0.0, "{:?}", k.target);
        assert_eq!(choice_of(&ch, 2).skill_id, SkillId::GoalKeep);
        assert!(choice_of(&ch, 1).target.x < 0.0);

        // FREE BALL Q1 → robot en el punto a 0.20 m de la cruz del lado nuestro.
        apply_command(
            &shared,
            RefereeCommand { foul: Foul::FreeBall, quadrant: Quadrant::Q1, ..RefereeCommand::GAME_ON },
        );
        let ch = c.decide(&o);
        let striker_id = if c.role_of(0) == Some(Role::Striker) { 0 } else { 1 };
        let k = choice_of(&ch, striker_id);
        assert_eq!(k.target, Vec2::new(0.375 - 0.20, 0.40));

        // GAME_ON → vuelve la táctica normal de inmediato (sin hold de skill).
        apply_command(&shared, RefereeCommand::GAME_ON);
        let ch = c.decide(&o);
        assert_eq!(c.current_play(), Play::Open);
        assert!(ch.iter().all(|x| x.skill_id != SkillId::Mark || x.robot_id != striker_id));
        assert!(matches!(
            choice_of(&ch, striker_id).skill_id,
            SkillId::ApproachAligned | SkillId::ShootPush | SkillId::Intercept | SkillId::Clear
        ));
    }

    #[test]
    fn yellow_team_reads_referee_colors_correctly() {
        use crate::coach::referee::{Foul, RefTeam, RefereeCommand, apply_command, new_shared_referee};
        let shared = new_shared_referee();
        let mut c = HeuristicCoach::with_options(OWN, ATTACK, 2, true); // amarillo, ataca −x
        c.set_referee(shared.clone());
        apply_command(
            &shared,
            RefereeCommand { foul: Foul::PenaltyKick, team: RefTeam::Blue, ..RefereeCommand::GAME_ON },
        );
        let o = obs(
            Vec2::new(-0.375, 0.0),
            Vec2::ZERO,
            [Vec2::new(0.3, 0.1), Vec2::new(0.4, -0.3), Vec2::new(0.63, 0.0)],
        );
        let ch = c.decide(&o);
        assert_eq!(c.current_play(), Play::PenaltyTheirs);
        // Penal en contra: jugadores de campo en la mitad rival (x < 0 para amarillo).
        for id in [0, 1] {
            assert!(choice_of(&ch, id).target.x < 0.0, "{:?}", choice_of(&ch, id));
        }
        assert_eq!(choice_of(&ch, 2).skill_id, SkillId::GoalKeep);
    }

    fn score_of(c: &HeuristicCoach, skill: SkillId) -> f32 {
        c.last_striker_scores().iter().find(|(s, _)| *s == skill).map(|(_, v)| *v).unwrap()
    }

    #[test]
    fn scores_rank_options_as_expected() {
        let mut c = coach();
        // Detrás de la pelota, alineado, tiro libre: ShootPush ≈ 1 y gana.
        let o = obs(
            Vec2::new(0.2, 0.0),
            Vec2::ZERO,
            [Vec2::new(0.08, 0.0), Vec2::new(-0.5, 0.3), Vec2::new(-0.63, 0.0)],
        );
        let ch = c.decide(&o);
        assert_eq!(choice_of(&ch, 0).skill_id, SkillId::ShootPush);
        assert!(score_of(&c, SkillId::ShootPush) > 0.95);
        assert!(score_of(&c, SkillId::Intercept) == 0.0 && score_of(&c, SkillId::Clear) == 0.0);

        // Mismo caso con un rival tapando el tiro: baja pero sigue ganándole al approach.
        let mut c = coach();
        let mut o2 = o.clone();
        o2.opp_robots[0] = robot(Vec2::new(0.45, 0.02), 180.0);
        c.decide(&o2);
        let blocked = score_of(&c, SkillId::ShootPush);
        assert!(blocked < 0.95 && blocked > score_of(&c, SkillId::ApproachAligned), "{blocked}");

        // Pelota en nuestro fondo con presión rival: Clear domina.
        let mut c = coach();
        let mut o3 = obs(
            Vec2::new(-0.55, 0.1),
            Vec2::ZERO,
            [Vec2::new(-0.35, 0.1), Vec2::new(0.3, -0.3), Vec2::new(-0.63, 0.0)],
        );
        o3.opp_robots[0] = robot(Vec2::new(-0.45, 0.15), 180.0);
        let ch = c.decide(&o3);
        assert_eq!(choice_of(&ch, 0).skill_id, SkillId::Clear);
        // danger 0.875 × (0.6 + 0.4·presión 0.63) ≈ 0.75
        assert!(score_of(&c, SkillId::Clear) > 0.6, "{}", score_of(&c, SkillId::Clear));
    }

    #[test]
    fn hysteresis_keeps_current_option_when_scores_are_close() {
        let c = coach();
        let scores = [
            (SkillId::ShootPush, 0.50),
            (SkillId::Clear, 0.0),
            (SkillId::SpinKick, 0.60),
            (SkillId::Intercept, 0.0),
            (SkillId::ApproachAligned, 0.30),
        ];
        // Sin opción vigente gana la mayor.
        assert_eq!(c.pick_with_hysteresis(&scores, None), SkillId::SpinKick);
        // Con ShootPush vigente y diferencia < histéresis (0.15), se mantiene.
        assert_eq!(c.pick_with_hysteresis(&scores, Some(SkillId::ShootPush)), SkillId::ShootPush);
        // Diferencia mayor que la histéresis: cambia.
        let mut far = scores;
        far[2].1 = 0.80;
        assert_eq!(c.pick_with_hysteresis(&far, Some(SkillId::ShootPush)), SkillId::SpinKick);
        // Una opción vigente con score 0 no recibe bono.
        assert_eq!(c.pick_with_hysteresis(&scores, Some(SkillId::Intercept)), SkillId::SpinKick);
    }

    #[test]
    fn failed_shots_lower_shoot_score_and_move_aim() {
        let mut c = coach();
        let behind = obs(
            Vec2::new(0.2, 0.0),
            Vec2::ZERO,
            [Vec2::new(0.08, 0.0), Vec2::new(-0.5, 0.3), Vec2::new(-0.63, 0.0)],
        );
        c.decide(&behind);
        let before = score_of(&c, SkillId::ShootPush);
        assert_eq!(choice_of(&c.decide(&behind), 0).target, ATTACK);

        // Tres tiros por el centro que un rival toca de inmediato.
        for _ in 0..3 {
            c.decide(&behind); // arranca episodio de tiro (ShootPush)
            let mut touched = behind.clone();
            touched.opp_robots[0] = robot(Vec2::new(0.24, 0.0), 180.0); // rival sobre la pelota
            c.decide(&touched); // cierra el episodio como fallo
            // Estado limpio para el siguiente intento (sin rival, robot detrás).
            let reset = obs(
                Vec2::new(-0.3, 0.0),
                Vec2::ZERO,
                [Vec2::new(-0.5, 0.0), Vec2::new(-0.5, 0.3), Vec2::new(-0.63, 0.0)],
            );
            for _ in 0..4 {
                c.decide(&reset);
            }
        }
        assert!(c.adaptation.shot_rate(Lane::Center).samples >= 3);
        assert!(c.adaptation.shot_rate(Lane::Center).value < 0.35);
        // Un éxito por arriba hace que el centro deje de ser el objetivo.
        let up = obs(
            Vec2::new(0.2, 0.3),
            Vec2::ZERO,
            [Vec2::new(0.08, 0.3), Vec2::new(-0.5, -0.3), Vec2::new(-0.63, 0.0)],
        );
        c.decide(&up);
        let scored = obs(
            Vec2::new(0.66, 0.15),
            Vec2::new(0.9, 0.0),
            [Vec2::new(0.3, 0.3), Vec2::new(-0.5, -0.3), Vec2::new(-0.63, 0.0)],
        );
        c.decide(&scored);
        assert_eq!(c.adaptation.last_outcome(), Some(("tiro", true)));
        c.decide(&behind);
        let after = score_of(&c, SkillId::ShootPush);
        assert!(after < before, "score de tiro por el centro debe bajar: {before} → {after}");
        assert!(choice_of(&c.decide(&behind), 0).target.y > 0.0, "apunta al palo superior");
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
