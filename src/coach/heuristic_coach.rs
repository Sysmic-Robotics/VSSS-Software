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
use crate::coach::debug::{CoachDebug, TipoObjetivo};
use crate::coach::observation::{FIELD_HALF_X, FIELD_HALF_Y, Observation, RobotObs};
use crate::coach::playbook::{Estilo, Playbook};
use crate::coach::plays::{Play, formation};
use crate::coach::referee::SharedReferee;
use crate::coach::skill_choice::SkillChoice;
use crate::motion::MotionConfig;
use crate::params::{CoachParams, params};
use crate::skills::{SkillId, shoot_push_feasible_now};
use glam::Vec2;
use std::f32::consts::{FRAC_PI_2, PI};

mod depuracion;

// ── Geometría del campo (LARC VSSS 3v3, m) ────────────────────────────────────
/// |x| desde el que empieza el área de arco propio/rival (área 70×15 cm).
pub const GOAL_AREA_X: f32 = 0.60;
/// Mitad del ancho del área de arco.
pub const GOAL_AREA_HALF_Y: f32 = 0.35;
/// Mitad del ancho de la boca del arco.
pub const GOAL_HALF_Y: f32 = 0.20;
/// Límite |x| para robots de campo (no entrar al área propia: solo 1 permitido).
/// Segundos desde el silbato de un penal propio en que el atacante patea al palo elegido.
const PENALTY_SHOT_S: f32 = 3.0;
/// Saque rival con muralla: distancia a la pelota del atacante (justo fuera del círculo central,
/// como en la formación) y del apoyo, sobre la recta pelota → arco propio (m).
const KICKOFF_WALL_DIST: f32 = crate::coach::plays::CENTER_CIRCLE_R + 0.06;
const KICKOFF_SECOND_DIST: f32 = 0.45;
/// La muralla termina apenas la pelota se mueve esto del centro (m). Con 0.12, el atacante esperaba
/// quieto mientras una arrancada en diagonal lo rodeaba (gol a los 1.3 s en el partido visible).
const KICKOFF_WALL_RELEASE: f32 = 0.04;
/// Carril de la arrancada del saque cuando un rival tapa todas las rectas al arco: el punto
/// (x hacia el arco rival, |y|) al que conduce primero para pasarle al costado (m).
const KICKOFF_LANE: Vec2 = Vec2::new(0.45, 0.25);
/// Arrancada rozando el palo: |y| en la línea de gol (el borde interior del palo) y holgura con
/// que la recta pasa a los rivales (medio robot + radio de la pelota), m.
const KICKOFF_EDGE_Y: f32 = 0.175;
const KICKOFF_EDGE_MARGIN: f32 = 0.0585;
const FIELD_ROBOT_MAX_ABS_X: f32 = 0.55;
const FIELD_ROBOT_MAX_ABS_Y: f32 = 0.55;
/// Desvío: centro del robot a la pelota cuando ella toca el medio de una cara (medio lado del
/// robot de 7.5 cm más el radio de la pelota).
const DEFLECT_FACE_TO_BALL: f32 = 0.059;

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

/// Pase y desvío en curso (`pass_deflect_max_dist`): quién pasa y quién desvía, dónde toca la
/// cara del receptor (`point`), su centro y rumbo, dónde estaba la pelota al empezar y la
/// decisión en que empezó y en que el pasador frenó.
#[derive(Debug, Clone, Copy)]
struct PassPlay {
    passer: i32,
    receiver: i32,
    point: Vec2,
    center: Vec2,
    heading: f32,
    ball0: Vec2,
    started: u64,
    released: Option<u64>,
}

/// Arrancada del saque propio (`kickoff_rush_s`): lado del carril (+1/−1 en y), decisión del
/// silbato y objetivo. El objetivo se fija al silbato y cambia una sola vez, del carril al palo de
/// ese lado cuando la pelota lo pasa: recalculado en cada decisión saltaba de un palo al otro y el
/// atacante empujaba la pelota de costado hasta la banda (0 goles en 30 saques).
#[derive(Debug, Clone, Copy)]
struct KickoffRush {
    side: f32,
    since: u64,
    target: Vec2,
    /// Amagando hacia el palo contrario (`kickoff_feint_s`).
    feint: bool,
}

/// Desvío en curso: punto del camino de la pelota donde la toca la cara, centro y rumbo del
/// robot, y decisión en que se espera el contacto.
#[derive(Debug, Clone, Copy)]
struct DeflectPlan {
    point: Vec2,
    center: Vec2,
    heading: f32,
    contact_at: u64,
}

/// Pelota trabada junto al striker (pegada al robot sin moverse: contra la pared, con un
/// rival encima). Empujando, tras `push_stall_s` se la saca con el giro; si el giro
/// tampoco la suelta en `stall_kick_max_s`, se lo prohíbe `spin_ban_s` para que pruebe
/// otra opción en vez de quedarse quieto con el giro agotado.
#[derive(Debug, Clone, Copy)]
enum Stall {
    None,
    /// Decisión en la que empezó a empujar sin que la pelota se mueva.
    Counting(u64),
    /// Decisión en la que se forzó (o empezó solo) `SpinKick`.
    Kicking(u64),
    /// Decisión desde la que `SpinKick` queda prohibido.
    SpinBanned(u64),
}

/// Qué impone el estado de pelota trabada sobre los scores del striker.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StallAction {
    Nothing,
    ForceSpin,
    BanSpin,
}

// Los tiempos del empuje trabado, las distancias de apoyo y presión y los umbrales
// del arquero activo son parámetros de estilo de juego: `CoachParams`
// (`config/team_params.json`, rangos en `config/style_ranges.json`).

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
    /// Desvío en curso por robot (`deflect_min_speed`).
    deflect_plans: [Option<DeflectPlan>; 3],
    decision_count: u64,
    /// Decisión desde la que la pelota está quieta (`spin_min_still_s`).
    ball_still_since: Option<u64>,
    /// El apoyo estaba cediendo el paso en la decisión anterior (`support_yield_dist`).
    yielding: [bool; 3],
    /// Pase y desvío en curso.
    pass_play: Option<PassPlay>,
    /// Arrancada del saque propio en curso (`kickoff_rush_s`).
    kickoff_rush: Option<KickoffRush>,
    /// Saques propios con arrancada: alterna el lado con el arquero rival centrado.
    kickoffs_taken: u32,
    /// Lado de la arrancada elegido al empezar el saque propio: fijo hasta el silbato. Elegido en
    /// cada decisión cambiaba con los rivales acomodándose y el pateador se corría de un lado al
    /// otro detrás de la pelota; al silbato todavía estaba girando.
    kickoff_side: Option<f32>,
    /// Muralla del saque rival en curso (`kickoff_guard_s`): decisión del silbato.
    kickoff_guard: Option<u64>,
    /// Decisión en la que la pelota entró al área propia (para el límite de 10 s).
    ball_in_area_since: Option<u64>,
    /// Estado del árbitro (VSSReferee / operador). `None` = siempre juego abierto.
    referee: Option<SharedReferee>,
    /// Play vigente (derivada del último comando del árbitro).
    current_play: Play,
    /// Tiro de pelota parada en juego: objetivo del atacante, decisión en que sonó el silbato
    /// y cuánto dura (s). Hoy solo el penal propio (el palo).
    penalty_shot: Option<(Vec2, u64, f32)>,
    /// Penales propios pateados (alterna el palo cuando el arquero está centrado).
    penalties_taken: u32,
    /// Penal propio con un solo robot nuestro en la cancha (tanda de penales: "apenas um
    /// jogador de cada equipe"): ese robot patea en vez de quedar de arquero. Se recalcula en
    /// cada `assign_roles`.
    pateador_unico: Option<i32>,
    /// Últimos scores del striker (skill, score) — para logs/GUI y tests.
    last_scores: [(SkillId, f32); 5],
    /// Contadores de adaptación al rival (éxito de tiro por carril, disputas).
    pub adaptation: Adaptation,
    /// Ya se comparó el lado configurado con la posición inicial del equipo.
    side_checked: bool,
    push_stall: Stall,
    /// Depuración de las decisiones (GUI y CSV de partido; no decide nada).
    depuracion: depuracion::Registro,
    /// Registro de cartas (`coach::cartas`, etapa 1 del selector de jugadas): solo el coach de
    /// producción lo abre, junto al registro de partido; no decide nada.
    cartas: Option<crate::coach::cartas::RegistroCartas>,
    /// Con `estilo = adaptativo`: elige el estilo de juego abierto durante el partido.
    playbook: Option<Playbook>,
    /// Pelota en la última decisión de juego abierto (el playbook reconoce un gol por el saque
    /// que lo sigue con la pelota junto a un arco).
    last_open_ball: Vec2,
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
        let mut c = Self::from_params(
            attack_goal,
            own_goal,
            params().coach.clone(),
            MotionConfig::from_env().bidirectional,
        );
        c.cartas = crate::coach::cartas::RegistroCartas::desde_entorno();
        c
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
            deflect_plans: [None; 3],
            decision_count: 0,
            ball_still_since: None,
            yielding: [false; 3],
            pass_play: None,
            kickoff_rush: None,
            kickoffs_taken: 0,
            kickoff_side: None,
            kickoff_guard: None,
            ball_in_area_since: None,
            referee: None,
            current_play: Play::Open,
            penalty_shot: None,
            penalties_taken: 0,
            pateador_unico: None,
            last_scores: [
                (SkillId::ShootPush, 0.0),
                (SkillId::Clear, 0.0),
                (SkillId::SpinKick, 0.0),
                (SkillId::Intercept, 0.0),
                (SkillId::ApproachAligned, 0.0),
            ],
            adaptation: Adaptation::new(p.adapt_enabled, p.adapt_alpha),
            side_checked: false,
            push_stall: Stall::None,
            depuracion: depuracion::Registro::default(),
            cartas: None,
            playbook: (p.estilo == "adaptativo").then(|| {
                let largo = (p.playbook_episodio_s * p.decision_hz).round() as u32;
                Playbook::new(largo, 0x9E37_79B9_7F4A_7C15 ^ u64::from(attack_goal.x.to_bits()))
            }),
            last_open_ball: Vec2::ZERO,
            p,
        }
    }

    /// Estilo de juego abierto vigente: el del playbook o el fijo de `coach.estilo`.
    pub fn estilo(&self) -> Estilo {
        match &self.playbook {
            Some(pb) => pb.vigente(),
            None => Estilo::from_nombre(&self.p.estilo).unwrap_or(Estilo::Equilibrado),
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

    /// Play según el último comando del árbitro (juego abierto si no hay árbitro). El equipo
    /// propio es el color de la observación (`VSSL_TEAM_COLOR`), no el lado de ataque: antes
    /// se deducía de que "el azul ataca +x", y con el azul a la derecha (o el amarillo a la
    /// izquierda, como en el segundo tiempo) todas las pelotas paradas con equipo quedaban al
    /// revés (en un penal propio se quedaba afuera y en uno rival se ponía a patear).
    fn read_play(&self, own_team: i32) -> Play {
        match self.referee.as_ref().and_then(|r| r.lock().ok().map(|s| s.command)) {
            Some(cmd) => Play::from_command(&cmd, own_team),
            None => Play::Open,
        }
    }

    /// Decisiones de una play de pelota parada: cada rol va a su punto de la
    /// formación mirando a la pelota (`Mark`); el arquero sigue en la línea salvo
    /// que la play le dé un punto explícito (goal kick propio: lo saca él).
    fn set_piece_choices(&mut self, play: Play, own: &[RobotView], opp: &[RobotView]) -> Vec<SkillChoice> {
        let s = self.attack_sign();
        // Con la arrancada, el pateador se forma ya alineado con ella: formado hacia el centro del
        // arco, al silbato tenía que rodear la pelota y el primer segundo iba a 0.1-0.2 m/s.
        let aim = if play == Play::KickoffOurs && self.p.kickoff_rush_s > 0.0 {
            let side = match self.kickoff_side {
                Some(side) => side,
                None => {
                    let side = self.kickoff_rush_side(Vec2::ZERO, opp);
                    self.kickoff_side = Some(side);
                    side
                }
            };
            self.kickoff_first_target(Vec2::ZERO, side, opp)
        } else {
            self.aim_point(opp)
        };
        let staging = params().skills.approach_staging_offset;
        let Some(mut f) = formation(play, s, aim, staging) else {
            return Vec::new();
        };
        // Muralla del saque rival: el apoyo ya formado detrás del atacante, en la recta al arco.
        if play == Play::KickoffTheirs && self.p.kickoff_guard_s > 0.0 {
            f.support = Vec2::new(-s * KICKOFF_SECOND_DIST, 0.0);
        }
        // El pateador es el jugador de campo que llega antes a su punto.
        self.assign_roles(own, f.striker, Vec2::ZERO);
        own.iter()
            .filter(|r| r.active)
            .map(|r| match self.role_of(r.id) {
                Some(Role::Keeper) => match f.keeper {
                    Some(p) => SkillChoice::new(r.id, SkillId::Mark, p),
                    None => SkillChoice::new(r.id, self.guard_skill(), self.own_goal),
                },
                Some(Role::Striker) => SkillChoice::new(r.id, SkillId::Mark, f.striker),
                _ => SkillChoice::new(r.id, SkillId::Mark, f.support),
            })
            .collect()
    }

    /// Rol vigente de cada robot (para GUI/logs). `None` si inactivo.
    pub fn role_of(&self, robot_id: i32) -> Option<Role> {
        if self.pateador_unico == Some(robot_id) {
            Some(Role::Striker)
        } else if robot_id == self.keeper_id {
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

    /// `true` si la pelota en `p` es del arquero: está dentro del área propia o a menos de
    /// un robot de su borde o de su arco, donde un jugador de campo no puede ponerse detrás
    /// de ella sin tocar el área (el guardia lo frena en la línea y la disputa con el
    /// arquero y los rivales termina con alguien empujado adentro).
    pub fn ball_is_keepers(&self, p: Vec2) -> bool {
        use crate::skills::zones::{AreaRect, ROBOT_HALF};
        const BALL_RADIUS: f32 = 0.02;
        // Contra el fondo, fuera del área, el arquero tampoco puede ponerse detrás: esa
        // pelota es del giro del striker.
        self.in_own_area(p)
            || (!self.ball_on_end_wall(p) && AreaRect::own(self.attack_sign()).touches_with(p, ROBOT_HALF + BALL_RADIUS))
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

    /// Primer instante en que `r` puede estar sobre la pelota que rueda, con el modelo de
    /// `InterceptSkill` y sus parámetros (frenado exponencial, horizonte, pasos de 0.05 s);
    /// la pelota predicha se recorta a la cancha. Sin alcanzarla en el horizonte, el tiempo
    /// hasta donde queda; con la pelota quieta, `time_to_reach`.
    fn time_to_intercept(&self, r: &RobotView, ball: Vec2, ball_vel: Vec2) -> f32 {
        let sp = &params().skills;
        if ball_vel.length() < sp.intercept_min_ball_speed {
            return self.time_to_reach(r, ball);
        }
        let k = sp.intercept_ball_decay_per_s;
        let at = |t: f32| {
            let travel = if k > 1e-6 { (1.0 - (-k * t).exp()) / k } else { t };
            let p = ball + ball_vel * travel;
            Vec2::new(p.x.clamp(-FIELD_HALF_X, FIELD_HALF_X), p.y.clamp(-FIELD_HALF_Y, FIELD_HALF_Y))
        };
        let mut t = 0.0_f32;
        while t <= sp.intercept_horizon {
            if self.time_to_reach(r, at(t)) <= t {
                return t;
            }
            t += 0.05;
        }
        self.time_to_reach(r, at(sp.intercept_horizon))
    }

    /// Palo al que patea un penal propio: el contrario al arquero rival, o alternando si está
    /// centrado. Con el arquero en el centro se apuntaba al centro y la pelota le pegaba: en
    /// la cancha a velocidad real convertíamos el 10 % contra el arquero de rule_based.
    fn penalty_corner(&mut self, opp: &[RobotView]) -> Vec2 {
        let keeper_y = opp
            .iter()
            .filter(|o| o.active && (o.pos - self.attack_goal).length() <= self.p.opp_keeper_radius)
            .min_by(|a, b| (a.pos - self.attack_goal).length().total_cmp(&(b.pos - self.attack_goal).length()))
            .map(|k| k.pos.y);
        let side = match keeper_y {
            Some(y) if y.abs() > 0.01 => -y.signum(),
            _ => if self.penalties_taken % 2 == 0 { 1.0 } else { -1.0 },
        };
        self.penalties_taken += 1;
        Vec2::new(self.attack_goal.x, side * (GOAL_HALF_Y - self.p.aim_post_margin))
    }

    /// Objetivo del tiro de pelota parada mientras dura (desde el silbato).
    fn active_penalty_shot(&self) -> Option<Vec2> {
        self.penalty_shot
            .filter(|&(_, since, secs)| (self.decision_count - since) as f32 / self.p.decision_hz.max(1e-3) < secs)
            .map(|(target, _, _)| target)
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
        let post_y = GOAL_HALF_Y - self.p.aim_post_margin;
        let (aim, tipo) = match keeper {
            Some(k) if k.pos.y.abs() > 0.03 => {
                (Vec2::new(self.attack_goal.x, -k.pos.y.signum() * post_y), TipoObjetivo::PaloLejano)
            }
            // Arquero centrado o sin arquero rival visible: al centro, salvo que los contadores
            // digan que por el centro no está entrando y otro carril rinde mejor. (Se probó, como
            // en el penal, ir al palo lejano a la pelota con el arquero centrado: contra el actual
            // +0.05 ± 0.09 goles en 120 partidos y contra rule_based +0.16 ± 0.3; sin efecto.)
            _ => (Vec2::new(self.attack_goal.x, self.adaptation.preferred_aim_y(0.0, post_y)), TipoObjetivo::Arco),
        };
        self.depuracion.tipo_objetivo(tipo);
        aim
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
        let corner = self.ball_on_end_wall(ball) && self.ball_on_side_wall(ball);
        if self.ball_on_end_wall(ball) {
            self.depuracion.tipo_objetivo(if attacking_end {
                TipoObjetivo::FondoRival
            } else {
                TipoObjetivo::FondoPropio
            });
        }
        if corner {
            // Esquina: no hay dónde ponerse detrás; el objetivo solo orienta al giro
            // (SpinKick la saca hacia ahí). Rival: al frente del arco. Propia: por la banda.
            return if attacking_end {
                Vec2::new(s * (GOAL_AREA_X - 0.12), -side * 0.10)
            } else {
                Vec2::new(s * 0.20, side * 0.50)
            };
        }
        if self.ball_on_end_wall(ball) {
            // Contra el fondo solo se puede empujar A LO LARGO de la pared: cualquier
            // objetivo hacia el campo deja el staging dentro de la pared y el robot se queda
            // frente a la pelota sin poder empujarla (`clear` congelado 20 s en los registros).
            // Fondo rival: hacia la boca del arco; propio: hacia la esquina, de donde el giro
            // la saca por la banda.
            return if attacking_end {
                Vec2::new(ball.x, side * (GOAL_HALF_Y - 0.05))
            } else {
                Vec2::new(ball.x, side * 0.70)
            };
        }
        if self.ball_on_side_wall(ball) {
            // Banda: conducir a lo largo de la pared, sin ángulo. Con 13° hacia adentro el
            // staging caía en la pared y, clampado al campo lógico, quedaba justo en la
            // tolerancia lateral del empuje: approach↔shoot sin tocar la pelota.
            self.depuracion.tipo_objetivo(TipoObjetivo::Banda);
            return Vec2::new(ball.x + s * 0.35, ball.y);
        }
        // Tiro de pelota parada en curso (penal al palo, salida del free ball): antes que el
        // despeje, porque la mitad de los free balls caen en nuestra mitad.
        if let Some(target) = self.active_penalty_shot() {
            self.depuracion.tipo_objetivo(TipoObjetivo::PaloLejano);
            return target;
        }
        if self.in_own_half(ball) && (ball.x * -s) > 0.35 {
            self.depuracion.tipo_objetivo(TipoObjetivo::Despeje);
            return self.clear_target(ball);
        }
        // (Se probó conducir por una dirección libre cercana con un rival en el camino: menos
        // conducciones contra un rival, pero el rival la cortaba igual (30 % y 30 %) y cara a
        // cara quedó +0.38 ± 0.27 para ir derecho; se descartó. Lo que más pierde la pelota es
        // empujarla descentrada en la cara. También se probó el tiro de banda: con el tiro
        // directo tapado, apuntar al punto de la banda donde el rebote medido en FIRASim la lleva
        // al arco. En motion_bench (`banda`) toca la banda donde corresponde pero no entra nunca,
        // 0/20 con el tope real y 0/10 a 0.8 m/s por rueda: empujando, el robot la aprieta contra
        // la pared y sale rasante a lo largo de ella; de giro, llega a ~0.1 m/s y muere ahí. El
        // ángulo sirve: lanzada sola a 1.5 m/s (`banda_libre_15`) rebota hacia el arco y se
        // detiene 7 cm antes de la línea; falta fuerza (en FIRASim la pelota frena mucho: 0.6 m/s
        // llegan a 0.17 a la banda) y el rebote sale más rasante que el modelo (k ≈ 0.36).)
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
        Self::line_blocked(ball, target, opp, 0.09)
    }

    /// Un rival a menos de `margin` del segmento `ball`→`target` (y por delante de la pelota).
    fn line_blocked(ball: Vec2, target: Vec2, opp: &[RobotView], margin: f32) -> bool {
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
            along > 0.05 && along < len && lateral < margin
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
        // En una pared el despeje hacia el campo no tiene staging: la saca ShootPush a lo
        // largo de la pared o el giro.
        let on_wall = self.ball_on_side_wall(ball) || self.ball_on_end_wall(ball);
        let clear = if self.in_own_half(ball) && !on_wall { danger * (0.6 + 0.4 * pressure) } else { 0.0 };

        // SpinKick: pelota quieta y disputada (rival encima) o en el fondo/esquina,
        // con el robot cerca; pierde valor si igual se puede empujar.
        let close = clamp01(1.0 - (dist_ball - 0.08) / p.spin_engage_radius.max(1e-3));
        let still_s = self
            .ball_still_since
            .map_or(0.0, |since| self.decision_count.saturating_sub(since) as f32 / p.decision_hz.max(1e-3));
        let still_enough = p.spin_min_still_s <= 0.0 || still_s >= p.spin_min_still_s;
        let slow: f32 = if ball_speed < p.intercept_min_ball_speed && still_enough { 1.0 } else { 0.0 };
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
        // menos atractivo si el staging está lejos (la pelota "se va"). Con el empuje ya
        // factible queda casi en cero: su único fin es hacerlo factible y, una vez en el
        // staging, se queda quieto (un tiro tapado vale más que no tirar). No es cero
        // para que el compromiso mínimo siga evitando el ping-pong approach↔shoot.
        let staging_far = clamp01(dist_ball / (2.0 * sp.approach_staging_offset + 0.6));
        // Ya en el staging y el empuje sigue infactible (pelota en la pared: el staging se
        // clampa al campo lógico y el robot para a más de la tolerancia lateral): seguir en
        // approach es quedarse quieto; que gane el giro.
        let at_staging = crate::skills::tactical::ApproachAlignedSkill::new(target)
            .staging_point(ball)
            .is_some_and(|st| (st - r.pos).length() <= sp.approach_pos_tol);
        let approach = p.score_approach_base
            * if shoot > 0.0 || at_staging { p.score_approach_when_feasible } else { 1.0 - 0.3 * staging_far };

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

    /// Actualiza el estado de pelota trabada y devuelve qué imponer sobre los scores:
    /// - empujando con la pelota pegada y quieta `push_stall_s` → `ForceSpin` hasta que se
    ///   suelte (se mueve o se aleja) o pasen `stall_kick_max_s`;
    /// - un giro (forzado o elegido) que no la suelta en `stall_kick_max_s` → `BanSpin`
    ///   durante `spin_ban_s` (el giro agotado deja al robot quieto; que pruebe otra cosa).
    fn update_push_stall(&mut self, current: Option<SkillId>, pos: Vec2, ball: Vec2, ball_vel: Vec2) -> StallAction {
        let p = &self.p;
        let hz = p.decision_hz.max(1e-3);
        let now = self.decision_count;
        let elapsed = |since: u64| now.saturating_sub(since) as f32 / hz;
        let dist = (ball - pos).length();
        let pinned = dist < 0.15 && ball_vel.length() < 0.05;
        let freed = ball_vel.length() > 0.15 || dist > 0.25;
        // Contar también `ApproachAligned` (que se ejecuta como `Dribble` y empuja) se probó y
        // se descartó: en la boca del arco las trabas bajaron de 63 a 23 s, pero el total no
        // cambió (9 % del juego abierto) y la pelota pasó más tiempo en las esquinas de ataque
        // (1472 contra 1306 s). Cara a cara, -0.02 ± 0.13 y +0.12 ± 0.17 (96 partidos).
        let pushing = matches!(current, Some(SkillId::ShootPush | SkillId::Clear));
        let spinning = current == Some(SkillId::SpinKick);
        self.push_stall = match self.push_stall {
            Stall::None if pushing && pinned => Stall::Counting(now),
            Stall::None if spinning && pinned => Stall::Kicking(now),
            Stall::Counting(_) if !(pushing && pinned) => Stall::None,
            Stall::Counting(since) if elapsed(since) >= p.push_stall_s => Stall::Kicking(now),
            Stall::Kicking(_) if freed => Stall::None,
            Stall::Kicking(since) if elapsed(since) >= p.stall_kick_max_s => Stall::SpinBanned(now),
            Stall::SpinBanned(since) if freed || elapsed(since) >= p.spin_ban_s => Stall::None,
            same => same,
        };
        match self.push_stall {
            Stall::Kicking(_) => StallAction::ForceSpin,
            Stall::SpinBanned(_) => StallAction::BanSpin,
            _ => StallAction::Nothing,
        }
    }

    /// Punto a `standoff_from_ball` de la pelota hacia el centro de la cancha (fuera del
    /// área propia): donde espera un robot de campo mientras el arquero juega la pelota.
    fn standoff_point(&self, ball: Vec2) -> Vec2 {
        let away = (Vec2::ZERO - ball).normalize_or_zero();
        let mut pos = self.clamp_field_robot(ball + away * self.p.standoff_from_ball);
        if self.in_own_area(pos) {
            pos.x = self.attack_sign() * -(GOAL_AREA_X - 0.08);
        }
        pos
    }

    /// `pos` alejado de la pelota hasta `standoff_from_ball` si estaba más cerca (contra
    /// la pared el clamp del campo deja el punto de apoyo encima de la pelota y el
    /// support se suma a la disputa).
    fn away_from_ball(&self, pos: Vec2, ball: Vec2) -> Vec2 {
        let standoff = self.p.standoff_from_ball;
        let rel = pos - ball;
        if rel.length() >= standoff {
            return pos;
        }
        let dir = if rel.length() > 1e-3 { rel.normalize() } else { (Vec2::ZERO - ball).normalize_or_zero() };
        self.clamp_field_robot(ball + dir * standoff)
    }

    /// Presión: `true` si un rival está encima de la pelota y el striker, más lejos que
    /// `press_distance`, debe esperarla sobre la línea pelota–arco propio en vez de ir.
    fn holds_press_line(&self, r: &RobotView, ball: Vec2, opp: &[RobotView]) -> bool {
        if self.estilo() == Estilo::Ofensivo {
            return false;
        }
        let rival_on_ball = opp
            .iter()
            .any(|o| o.active && (o.pos - ball).length() <= self.p.spin_when_opponent_within);
        rival_on_ball && (ball - r.pos).length() > self.p.press_distance
    }

    /// Punto de espera de la presión: sobre la línea pelota–arco propio, a
    /// `press_distance` de la pelota, fuera del área propia.
    fn press_point(&self, ball: Vec2) -> Vec2 {
        let to_own = (self.own_goal - ball).normalize_or_zero();
        let mut pos = self.clamp_field_robot(ball + to_own * self.p.press_distance);
        if self.in_own_area(pos) {
            pos.x = self.attack_sign() * -(GOAL_AREA_X - 0.08);
        }
        pos
    }

    /// Con el empuje trabado gira hacia el mismo objetivo, sin pasarla. (Pases descartados con
    /// datos: por empuje con el tiro tapado llegaba al receptor el 9-20 % de las veces. Con el
    /// giro hacia el compañero, que con un rival encima suelta la pelota a ~0.23 m/s, la tocaba
    /// primero el compañero el 6-11 % de las veces; además le obligaba a rodear la pelota y
    /// erraba la dirección por ~70°. En 144 partidos, girar hacia el objetivo, o sin
    /// reubicarse, nunca rindió peor que el pase, y hacia el objetivo le ganó cara a cara a
    /// ambos: +0.29 ± 0.23 y +0.46 ± 0.29 goles por partido.)
    fn striker_choice(&mut self, r: &RobotView, ball: Vec2, ball_vel: Vec2, opp: &[RobotView]) -> SkillChoice {
        self.end_deflection(r.id);
        // Desvío: con la pelota rodando libre, desviarla de primera al arco en vez de recibirla.
        if let Some(choice) = self.deflect_choice(r, ball, ball_vel, None, opp) {
            return choice;
        }
        let target = self.striker_target(ball, opp);
        let mut scores = self.striker_scores(r, ball, ball_vel, opp, target);
        let current = self.committed[r.id as usize].map(|c| c.skill);
        match self.update_push_stall(current, r.pos, ball, ball_vel) {
            // Empuje trabado: sacarla con el giro aunque los scores digan seguir empujando.
            StallAction::ForceSpin => {
                for (skill, score) in scores.iter_mut() {
                    if *skill == SkillId::SpinKick {
                        *score = 1.0;
                    }
                }
            }
            StallAction::BanSpin => {
                for (skill, score) in scores.iter_mut() {
                    if *skill == SkillId::SpinKick {
                        *score = 0.0;
                    }
                }
            }
            StallAction::Nothing => {}
        }
        self.last_scores = scores;
        let wanted = self.pick_with_hysteresis(&scores, current);
        // Compromiso mínimo: la opción vigente se mantiene mientras siga viable (score > 0).
        let skill = self.commit(r.id, wanted, |prev| {
            scores.iter().any(|(s, v)| *s == prev && *v > 0.0)
        });
        self.depuracion_atacante(r, ball, opp, target, &scores, current, wanted, skill);
        // Conducción: aproximarse y empujar como un solo movimiento (`Dribble`, campo "ir al
        // arco"), sin pararse detrás de la pelota. Las opciones y el compromiso siguen
        // decidiéndose con `ApproachAligned`/`ShootPush`. Contra aproximarse a un punto detrás
        // de la pelota y recién empujar: ganó 19-3-2 cara a cara, +1.00 ± 0.20 goles por
        // partido (48 partidos), con 74 % más empujes en contacto.
        let executed = if matches!(skill, SkillId::ApproachAligned | SkillId::ShootPush) {
            SkillId::Dribble
        } else {
            skill
        };
        SkillChoice::new(r.id, executed, target)
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

    /// Lado (+1/−1 en y) de la arrancada del saque: el del palo lejos del arquero rival; con el
    /// arquero centrado (o sin arquero visible), el del carril más despejado, y si empatan,
    /// alternando de un saque al siguiente (`kickoffs_taken`).
    fn kickoff_rush_side(&self, ball: Vec2, opp: &[RobotView]) -> f32 {
        let keeper_y = opp
            .iter()
            .filter(|o| o.active && (o.pos - self.attack_goal).length() <= self.p.opp_keeper_radius)
            .min_by(|a, b| (a.pos - self.attack_goal).length().total_cmp(&(b.pos - self.attack_goal).length()))
            .map(|k| k.pos.y);
        if let Some(y) = keeper_y.filter(|y| y.abs() > 0.01) {
            return -y.signum();
        }
        let s = self.attack_sign();
        let lane = |side: f32| {
            Self::segment_clearance(ball, Vec2::new(s * KICKOFF_LANE.x, side * KICKOFF_LANE.y), opp).min(1.0)
        };
        let (up, down) = (lane(1.0), lane(-1.0));
        if (up - down).abs() > 0.02 {
            if up > down { 1.0 } else { -1.0 }
        } else if self.kickoffs_taken % 2 == 0 {
            1.0
        } else {
            -1.0
        }
    }

    /// Primer objetivo de la arrancada: con amague (`kickoff_feint_s`), rozando el palo contrario
    /// al lado `side`; sin amague, el de `kickoff_rush_target`.
    fn kickoff_first_target(&self, ball: Vec2, side: f32, opp: &[RobotView]) -> Vec2 {
        if self.p.kickoff_feint_s > 0.0 {
            Vec2::new(self.attack_goal.x, -side * KICKOFF_EDGE_Y)
        } else {
            self.kickoff_rush_target(ball, side, opp)
        }
    }

    /// Objetivo de la arrancada desde la pelota en `ball`: el primero con la recta libre de
    /// rivales (a más de 0.09 m) entre el palo del lado `side`, el centro y el otro palo (por
    /// dentro, `aim_post_margin`). Si ninguno lo está (un rival parado delante de la pelota) y la
    /// pelota no pasó todavía el carril, el punto del carril de ese lado, para pasarle al costado;
    /// si no, el palo de ese lado.
    fn kickoff_rush_target(&self, ball: Vec2, side: f32, opp: &[RobotView]) -> Vec2 {
        let s = self.attack_sign();
        let gx = self.attack_goal.x;
        let post_y = GOAL_HALF_Y - self.p.aim_post_margin;
        let goal = [Vec2::new(gx, side * post_y), Vec2::new(gx, 0.0), Vec2::new(gx, -side * post_y)];
        if let Some(target) = goal.into_iter().find(|&g| !Self::line_blocked(ball, g, opp, 0.09)) {
            return target;
        }
        // Rozando el palo: del centro, la recta a y = 0.175 en la línea de gol (la pelota entra sin
        // tocar el palo) pasa a 5.9 cm de un rival parado justo fuera del círculo (0.26 m), y a la
        // pelota le alcanza con 5.85 (medio robot y su radio): al límite. Empujada fuerte la pelota
        // sigue derecho: por el carril llegaba al fondo 22 cm fuera del arco (y = 0.42 en 5 de 5
        // saques); por esta recta, si se escapa, va hacia adentro.
        let edge = [Vec2::new(gx, side * KICKOFF_EDGE_Y), Vec2::new(gx, -side * KICKOFF_EDGE_Y)];
        if let Some(target) = edge.into_iter().find(|&g| !Self::line_blocked(ball, g, opp, KICKOFF_EDGE_MARGIN)) {
            return target;
        }
        if ball.x * s < KICKOFF_LANE.x - 0.05 {
            return Vec2::new(s * KICKOFF_LANE.x, side * KICKOFF_LANE.y);
        }
        goal[0]
    }

    /// Saques: termina la arrancada (`kickoff_rush_s`) al acabarse su tiempo, si el atacante
    /// queda a más de 0.30 m de la pelota (empujada rápido se le adelanta), si un rival está encima de ella (a menos de 0.10 m y
    /// más cerca que el atacante) o con la pelota en el área rival; y la muralla
    /// (`kickoff_guard_s`) al acabarse su tiempo o apenas la pelota se mueve del centro
    /// (`KICKOFF_WALL_RELEASE`).
    fn update_kickoff(&mut self, striker: Option<&RobotView>, ball: Vec2, opp: &[RobotView]) {
        let hz = self.p.decision_hz.max(1e-3);
        let elapsed = |since: u64, now: u64| now.saturating_sub(since) as f32 / hz;
        let now = self.decision_count;
        // Del carril al palo de ese lado, una vez: apenas la recta al palo queda libre o la pelota
        // pasó el carril. (Esperando a llegar al punto del carril, el atacante frenaba ahí, la
        // pelota se le adelantaba y la arrancada se cortaba a los 1.2 s.)
        let (s, goal_x, post_y) = (self.attack_sign(), self.attack_goal.x, GOAL_HALF_Y - self.p.aim_post_margin);
        // Fin del amague: corta hacia el lado elegido (rozando el palo si la recta pasa, si no al
        // palo de ese lado).
        let hz = self.p.decision_hz.max(1e-3);
        if let Some(rush) = self.kickoff_rush.filter(|r| r.feint) {
            if self.decision_count.saturating_sub(rush.since) as f32 / hz >= self.p.kickoff_feint_s {
                let edge = Vec2::new(goal_x, rush.side * KICKOFF_EDGE_Y);
                let target = if Self::line_blocked(ball, edge, opp, KICKOFF_EDGE_MARGIN) {
                    Vec2::new(goal_x, rush.side * post_y)
                } else {
                    edge
                };
                self.kickoff_rush = Some(KickoffRush { target, feint: false, ..rush });
            }
        }
        if let Some(rush) = self.kickoff_rush.filter(|r| !r.feint) {
            let post = Vec2::new(goal_x, rush.side * post_y);
            let on_lane = rush.target.x * s < goal_x * s - 1e-3;
            if on_lane && (!Self::line_blocked(ball, post, opp, 0.09) || ball.x * s >= KICKOFF_LANE.x - 0.05) {
                self.kickoff_rush = Some(KickoffRush { target: post, ..rush });
            }
        }
        if let Some(KickoffRush { since, .. }) = self.kickoff_rush {
            let d_striker = striker.map_or(f32::INFINITY, |st| (st.pos - ball).length());
            let d_rival = opp
                .iter()
                .filter(|o| o.active)
                .map(|o| (o.pos - ball).length())
                .fold(f32::INFINITY, f32::min);
            let in_area = ball.x * self.attack_sign() > GOAL_AREA_X - 0.05;
            let contested = d_rival < 0.10 && d_rival < d_striker && !self.p.kickoff_rush_push;
            if elapsed(since, now) > self.p.kickoff_rush_s || d_striker > 0.30 || contested || in_area
            {
                self.kickoff_rush = None;
            }
        }
        if let Some(since) = self.kickoff_guard {
            if elapsed(since, now) > self.p.kickoff_guard_s || ball.length() > KICKOFF_WALL_RELEASE {
                self.kickoff_guard = None;
            }
        }
    }

    /// La orden de un saque en curso para `r`: en la arrancada, el atacante conduce al objetivo
    /// de la arrancada; en la muralla, el atacante espera en la recta pelota → arco propio justo
    /// fuera del círculo y el apoyo detrás, mirando la pelota.
    fn kickoff_choice(&mut self, r: &RobotView, ball: Vec2) -> Option<SkillChoice> {
        let role = self.role_of(r.id)?;
        if let Some(rush) = self.kickoff_rush {
            if role != Role::Striker {
                return None;
            }
            let skill = self.commit(r.id, SkillId::Dribble, |_| false);
            return Some(SkillChoice::new(r.id, skill, rush.target));
        }
        self.kickoff_guard?;
        let dist = match role {
            Role::Striker => KICKOFF_WALL_DIST,
            Role::Support => KICKOFF_SECOND_DIST,
            Role::Keeper => return None,
        };
        let dir = (self.own_goal - ball).normalize_or_zero();
        let skill = self.commit(r.id, SkillId::Mark, |_| false);
        Some(SkillChoice::new(r.id, skill, ball + dir * dist))
    }

    /// Fracción de la boca del arco rival (9 puntos entre ±0.18 m) que se ve desde `from` sin un
    /// rival a menos de 6 cm de la recta (medio robot y el radio de la pelota): 1 = arco abierto.
    fn open_goal_fraction(&self, from: Vec2, opp: &[RobotView]) -> f32 {
        const N: usize = 9;
        let libres = (0..N)
            .filter(|&i| {
                let y = -0.18 + 0.36 * i as f32 / (N - 1) as f32;
                !Self::line_blocked(from, Vec2::new(self.attack_goal.x, y), opp, 0.06)
            })
            .count();
        libres as f32 / N as f32
    }

    /// Una línea del registro de cartas (`coach::cartas`) con las cartas del atacante en esta
    /// decisión y lo que eligió el coach. Posiciones en el marco "atacamos hacia +x".
    #[allow(clippy::too_many_arguments)]
    fn cartas_json(
        &self,
        own: &[RobotView],
        opp: &[RobotView],
        ball: Vec2,
        ball_vel: Vec2,
        striker: Option<&RobotView>,
        support: Option<&RobotView>,
        choices: &[SkillChoice],
    ) -> serde_json::Value {
        use crate::coach::cartas::r3;
        use serde_json::json;
        let s = self.attack_sign();
        let p = |v: Vec2| [r3(v.x * s), r3(v.y)];
        let near = |robots: &[RobotView]| {
            robots
                .iter()
                .filter(|r| r.active)
                .map(|r| (r.pos - ball).length())
                .fold(f32::INFINITY, f32::min)
        };
        let campo: Vec<RobotView> = own.iter().filter(|r| r.active && r.id != self.keeper_id).copied().collect();
        let (d_nuestro, d_rival) = (near(&campo), near(opp));
        let posesion = if d_nuestro < 0.12 && d_nuestro <= d_rival {
            "nuestra"
        } else if d_rival < 0.12 {
            "rival"
        } else {
            "libre"
        };
        let t_rival = opp
            .iter()
            .filter(|o| o.active)
            .map(|o| self.time_to_intercept(o, ball, ball_vel))
            .fold(f32::INFINITY, f32::min);
        // Tiro directo: al centro, o al palo lejano si el arquero rival está corrido (como
        // `aim_point`, sin los contadores de adaptación ni tocar la depuración).
        let keeper = opp
            .iter()
            .filter(|o| o.active && (o.pos - self.attack_goal).length() <= self.p.opp_keeper_radius)
            .min_by(|a, b| (a.pos - self.attack_goal).length().total_cmp(&(b.pos - self.attack_goal).length()));
        let post_y = GOAL_HALF_Y - self.p.aim_post_margin;
        let aim = match keeper {
            Some(k) if k.pos.y.abs() > 0.03 => Vec2::new(self.attack_goal.x, -k.pos.y.signum() * post_y),
            _ => Vec2::new(self.attack_goal.x, 0.0),
        };
        let atacante = striker.map(|st| {
            let alin = (ball - st.pos).normalize_or_zero().dot((aim - ball).normalize_or_zero());
            json!({
                "id": st.id,
                "pos": p(st.pos),
                "d": r3((st.pos - ball).length()),
                "alineado": r3(alin),
                "t_llegar": r3(self.time_to_intercept(st, ball, ball_vel)),
            })
        });
        let pase = support.map(|sup| {
            json!({
                "id": sup.id,
                "pos": p(sup.pos),
                "d": r3((sup.pos - ball).length()),
                "linea_libre": r3(Self::segment_clearance(ball, sup.pos, opp).min(1.0)),
                "abierto": r3(self.open_goal_fraction(sup.pos, opp)),
                "ventana": self.pass_window(ball, striker, opp).map(p),
            })
        });
        let eleccion = |id: Option<i32>| {
            id.and_then(|id| choices.iter().find(|c| c.robot_id == id))
                .map(|c| json!({"skill": format!("{:?}", c.skill_id), "objetivo": p(c.target)}))
        };
        json!({
            "t": self.decision_count,
            "play": format!("{:?}", self.current_play),
            "pelota": p(ball),
            "vel": p(ball_vel),
            "posesion": posesion,
            "d_nuestro": r3(d_nuestro.min(9.0)),
            "d_rival": r3(d_rival.min(9.0)),
            "t_rival": r3(t_rival.min(9.0)),
            "atacante": atacante,
            "directo": {
                "objetivo_y": r3(aim.y),
                "abierto": r3(self.open_goal_fraction(ball, opp)),
                "libre": r3(Self::segment_clearance(ball, aim, opp).min(1.0)),
                "dist": r3((aim - ball).length()),
                "arquero_y": keeper.map(|k| r3(k.pos.y)),
            },
            "giro": {
                "al_alcance": striker.is_some_and(|st| (st.pos - ball).length() <= self.p.spin_engage_radius),
                "rival_encima": d_rival <= self.p.spin_when_opponent_within + 0.05,
            },
            "pase": pase,
            "puntajes": self.last_scores.iter().map(|(k, v)| json!([format!("{k:?}"), r3(*v)])).collect::<Vec<_>>(),
            "eleccion": {"atacante": eleccion(self.striker_id), "apoyo": eleccion(support.map(|r| r.id))},
            "jugadas": {"pase_y_desvio": self.pass_play.is_some(), "arrancada": self.kickoff_rush.is_some()},
        })
    }

    /// Pase y desvío (`pass_deflect_max_dist`): sigue, termina o empieza la jugada.
    /// Empieza con el atacante pegado detrás de la pelota quieta (respecto del pase), el tiro
    /// directo tapado y el apoyo en campo rival a 0.25 m a `pass_deflect_max_dist` de la
    /// pelota, con el pase y su salida al arco libres de rivales y un desvío de 15° a 100°. El
    /// pasador frena cuando la pelota ya va a 0.5 m/s o recorrió 0.2 m (o a 1 s). Termina a los
    /// 3 s, si cambian los roles, si antes de frenar el pasador pierde la pelota, o cuando, ya
    /// suelta, la pelota se frena o deja de ir hacia el receptor.
    fn update_pass_play(
        &mut self,
        striker: Option<&RobotView>,
        support: Option<&RobotView>,
        ball: Vec2,
        ball_vel: Vec2,
        opp: &[RobotView],
    ) {
        let max_dist = self.p.pass_deflect_max_dist;
        if max_dist <= 0.0 {
            self.pass_play = None;
            return;
        }
        let hz = self.p.decision_hz.max(1e-3);
        let since = |d: u64, now: u64| now.saturating_sub(d) as f32 / hz;
        if let Some(mut play) = self.pass_play {
            let now = self.decision_count;
            let roles_ok =
                striker.is_some_and(|s| s.id == play.passer) && support.is_some_and(|s| s.id == play.receiver);
            // Antes de frenar: perdió la pelota (el pasador se alejó de ella) o la pelota va al
            // revés. Con un rival cerca no se cancela: la pelota está disputada el 68 % del tiempo
            // que nuestro atacante está sobre ella.
            let dir = (play.point - play.ball0).normalize_or_zero();
            let lost = play.released.is_none()
                && (striker.map_or(true, |s| (s.pos - ball).length() > 0.18) || ball_vel.dot(dir) < -0.1);
            let over = play.released.is_some() && (ball_vel.length() < 0.1 || ball_vel.dot(play.point - ball) <= 0.0);
            if since(play.started, now) > 3.0 || !roles_ok || lost || over {
                self.end_pass_play(&play);
                return;
            }
            if play.released.is_none() {
                let fast = ball_vel.dot(dir) >= 0.5;
                let far = (ball - play.ball0).dot(dir) >= 0.2;
                if fast || far || since(play.started, now) >= 1.0 {
                    play.released = Some(now);
                }
            }
            self.pass_play = Some(play);
            return;
        }
        let (Some(st), Some(sup)) = (striker, support) else {
            return;
        };
        let s = self.attack_sign();
        let aim = self.attack_goal;
        let p = sup.pos;
        let d = (p - ball).length();
        if ball_vel.length() > 0.3
            || (st.pos - ball).length() > 0.12
            || d < 0.25
            || d > max_dist
            || p.x * s < -0.10
            || p.x * s > GOAL_AREA_X - 0.05
            || !Self::shot_blocked(ball, aim, opp)
            || Self::line_blocked(ball, p, opp, 0.09)
            || Self::line_blocked(p, aim, opp, 0.09)
        {
            return;
        }
        let inc = (p - ball) / d;
        let out = (aim - p).normalize_or_zero();
        let behind = (ball - st.pos).normalize_or_zero().dot(inc) >= std::f32::consts::FRAC_1_SQRT_2;
        if !behind || !(-0.17..=0.97).contains(&inc.dot(out)) {
            return;
        }
        let Some(ph) = Self::deflect_normal(inc, out, self.p.deflect_face_k) else {
            return;
        };
        let quarter = FRAC_PI_2;
        let off = (sup.theta - ph).rem_euclid(quarter);
        let heading = if off <= quarter / 2.0 { sup.theta - off } else { sup.theta + quarter - off };
        self.pass_play = Some(PassPlay {
            passer: st.id,
            receiver: sup.id,
            point: p,
            center: p - Vec2::from_angle(ph) * DEFLECT_FACE_TO_BALL,
            heading,
            ball0: ball,
            started: self.decision_count,
            released: None,
        });
    }

    /// Distancia del rival activo más cercano al segmento `a`→`b` (solo los que quedan a lo
    /// largo del segmento); infinito si no hay ninguno.
    fn segment_clearance(a: Vec2, b: Vec2, opp: &[RobotView]) -> f32 {
        let seg = b - a;
        let len = seg.length();
        if len < 1e-3 {
            return f32::INFINITY;
        }
        let dir = seg / len;
        opp.iter()
            .filter(|o| o.active)
            .filter_map(|o| {
                let rel = o.pos - a;
                let along = rel.dot(dir);
                (along > 0.0 && along < len).then(|| (rel - dir * along).length())
            })
            .fold(f32::INFINITY, f32::min)
    }

    /// Punto donde el apoyo arma el pase y desvío: alrededor de la pelota, a 0.30-0.50 m (sin
    /// pasar `pass_deflect_max_dist`) y girado 20-45° desde la dirección en que el atacante
    /// llega a la pelota (así pasa sin rodearla), desde 0.10 m antes de la mitad hasta el área
    /// rival, con el pase y la
    /// salida al arco libres de rivales, ninguno a menos de 0.20 m y un desvío de 15° a 100°.
    /// Entre los que quedan, el más lejos de los rivales (y el más corto si empatan). `None` sin
    /// atacante o con la pelota en nuestro tercio.
    fn pass_window(&self, ball: Vec2, striker: Option<&RobotView>, opp: &[RobotView]) -> Option<Vec2> {
        let max_dist = self.p.pass_deflect_max_dist;
        let st = striker?;
        let s = self.attack_sign();
        // Se anticipa: desde que el atacante va a la pelota (no recién cuando la tiene), con la
        // dirección en que llega, para que el pase salga en el primer toque. Esperando a que el
        // atacante estuviera a 0.25 m, el apoyo no estaba en su punto el 53 % de las veces.
        if max_dist <= 0.0 || ball.x * s < -0.20 {
            return None;
        }
        let aim = self.attack_goal;
        let push = (ball - st.pos).normalize_or_zero();
        if push.length_squared() < 0.5 {
            return None;
        }
        let mut best: Option<(f32, f32, Vec2)> = None;
        for d in [0.30_f32, 0.40, 0.50] {
            if d > max_dist {
                continue;
            }
            for deg in [-45.0_f32, -30.0, -20.0, 20.0, 30.0, 45.0] {
                let dir = Vec2::from_angle(deg.to_radians()).rotate(push);
                let p = ball + dir * d;
                let inside = p.x.abs() <= FIELD_ROBOT_MAX_ABS_X && p.y.abs() <= FIELD_ROBOT_MAX_ABS_Y;
                if !inside || p.x * s < -0.10 || p.x * s > GOAL_AREA_X - 0.05 {
                    continue;
                }
                let turn = dir.dot((aim - p).normalize_or_zero());
                let near = opp.iter().filter(|o| o.active).map(|o| (o.pos - p).length()).fold(f32::INFINITY, f32::min);
                let clear = Self::segment_clearance(ball, p, opp).min(Self::segment_clearance(p, aim, opp));
                if !(-0.17..=0.97).contains(&turn) || near < 0.20 || clear < 0.12 {
                    continue;
                }
                let score = clear.min(0.5) + near.min(0.5);
                if best.map_or(true, |b| score > b.0 + 1e-3 || ((score - b.0).abs() <= 1e-3 && d < b.1)) {
                    best = Some((score, d, p));
                }
            }
        }
        best.map(|b| b.2)
    }

    /// Fin de la jugada: se sueltan los compromisos de los dos (Hold, Dribble, GoTo y FacePoint
    /// del pase no deben seguir con el objetivo de otra rama).
    fn end_pass_play(&mut self, play: &PassPlay) {
        for id in [play.passer, play.receiver] {
            if let Some(slot) = self.committed.get_mut(id as usize) {
                *slot = None;
            }
        }
        self.pass_play = None;
    }

    /// La orden de la jugada para `r`: el pasador conduce hacia el receptor y después frena
    /// (`Hold`); el receptor va a su centro y orienta la cara (`deflect_command`).
    fn pass_play_choice(&mut self, r: &RobotView) -> Option<SkillChoice> {
        let play = self.pass_play?;
        if r.id == play.passer {
            let (wanted, target) = match play.released {
                None => (SkillId::Dribble, play.point),
                Some(_) => (SkillId::Hold, r.pos),
            };
            let skill = self.commit(r.id, wanted, |_| false);
            Some(SkillChoice::new(r.id, skill, target))
        } else if r.id == play.receiver {
            Some(self.deflect_command(r, play.center, play.heading))
        } else {
            None
        }
    }

    /// Normal de la cara (ángulo, rad) que manda hacia `out` una pelota que llega por `inc`
    /// (unitarios), con la cara devolviendo `k` de la velocidad normal respecto de la
    /// tangencial: la salida `inc − (1 + k)(inc·n) n` va por `out`. `None` si ninguna cara de
    /// frente a la pelota lo logra.
    fn deflect_normal(inc: Vec2, out: Vec2, k: f32) -> Option<f32> {
        let (al, be) = (inc.y.atan2(inc.x), out.y.atan2(out.x));
        let a = 1.0 + k;
        let f = |ph: f32| (al - be).sin() - a * (ph - al).cos() * (ph - be).sin();
        // Caras de frente a la pelota que llega: cos(ph − al) < 0.
        let (lo, hi) = (al + FRAC_PI_2 + 1e-3, al + 3.0 * FRAC_PI_2 - 1e-3);
        const N: usize = 180;
        (0..N).find_map(|i| {
            let mut x0 = lo + (hi - lo) * i as f32 / N as f32;
            let mut x1 = lo + (hi - lo) * (i + 1) as f32 / N as f32;
            if f(x0) * f(x1) > 0.0 {
                return None;
            }
            for _ in 0..30 {
                let xm = 0.5 * (x0 + x1);
                if f(x0) * f(xm) <= 0.0 {
                    x1 = xm;
                } else {
                    x0 = xm;
                }
            }
            let ph = 0.5 * (x0 + x1);
            let n = Vec2::from_angle(ph);
            let v = inc - n * (a * inc.dot(n));
            (v.dot(out) > 0.0).then_some(ph)
        })
    }

    /// Suelta el compromiso de un desvío anterior: `GoTo` y `FacePoint` solo los usa el desvío, que
    /// se recalcula en cada decisión; si no, el compromiso los mantenía con el objetivo de otra
    /// rama (por ejemplo, girando hacia nuestro arco en vez de cubrir la línea).
    fn end_deflection(&mut self, robot_id: i32) {
        let slot = &mut self.committed[robot_id as usize];
        if slot.is_some_and(|c| matches!(c.skill, SkillId::GoTo | SkillId::FacePoint)) {
            *slot = None;
        }
    }

    /// Desvío (`deflect_min_speed`): con la pelota rodando libre, el punto de su camino en la
    /// mitad rival que el robot alcanza antes que ella y con menos recorrido (el más temprano
    /// lo haría perseguir puntos cada vez más atrás sin asentarse), adonde llega a más de
    /// `deflect_min_speed` y que la desvía entre 15° y 100° hacia el centro del arco sin un
    /// rival en la salida. El robot va con `GoTo` a su centro de desvío y, ahí, gira con
    /// `FacePoint` hasta tener una cara con la normal de `deflect_normal` (el robot es cuadrado:
    /// sirve cualquiera de sus cuatro caras, la que menos gira).
    fn deflect_choice(
        &mut self,
        r: &RobotView,
        ball: Vec2,
        ball_vel: Vec2,
        striker: Option<&RobotView>,
        opp: &[RobotView],
    ) -> Option<SkillChoice> {
        let min_speed = self.p.deflect_min_speed;
        if min_speed <= 0.0 {
            return None;
        }
        // El desvío en curso se mantiene hasta el contacto: recalculado en cada decisión se
        // abandonaba a los 0.18 s (mediana de 187 intentos, 9 llegaron a su punto, ningún toque).
        let id = r.id as usize;
        if let Some(plan) = self.deflect_plans[id] {
            if self.deflect_plan_holds(&plan, ball, ball_vel, striker, opp) {
                return Some(self.deflect_command(r, plan.center, plan.heading));
            }
            self.deflect_plans[id] = None;
        }
        let speed = ball_vel.length();
        if speed < min_speed {
            return None;
        }
        // Uno solo por vez: el atacante y el apoyo calculaban el mismo punto e iban los dos.
        let hz = self.p.decision_hz.max(1e-3);
        let grace = (0.5 * hz).ceil() as u64;
        let other_active = self.deflect_plans.iter().enumerate().any(|(j, p)| {
            j != id && p.is_some_and(|p| self.decision_count <= p.contact_at + grace)
        });
        if other_active {
            return None;
        }
        // Pelota libre: ni el propio robot, ni nuestro atacante, ni un rival la llevan.
        let carried = (r.pos - ball).length() < 0.15
            || striker.is_some_and(|st| (st.pos - ball).length() < 0.15)
            || opp.iter().any(|o| o.active && (o.pos - ball).length() < 0.12);
        if carried {
            return None;
        }
        let s = self.attack_sign();
        let aim = self.attack_goal;
        let inc = ball_vel / speed;
        let sp = &params().skills;
        let k = sp.intercept_ball_decay_per_s;
        let mut best: Option<(f32, Vec2, f32, Vec2, f32)> = None; // (necesita, centro, rumbo, punto, t)
        let mut t = 0.1_f32;
        while t <= sp.intercept_horizon {
            let decay = (-k * t).exp();
            if speed * decay < min_speed {
                break;
            }
            let travel = if k > 1e-6 { (1.0 - decay) / k } else { t };
            let p = ball + ball_vel * travel;
            t += 0.05;
            if p.x * s < 0.10 || p.x * s > GOAL_AREA_X - 0.05 || p.y.abs() > FIELD_ROBOT_MAX_ABS_Y {
                continue;
            }
            let out = (aim - p).normalize_or_zero();
            let turn = inc.dot(out);
            if !(-0.17..=0.97).contains(&turn) || Self::line_blocked(p, aim, opp, 0.09) {
                continue;
            }
            let Some(ph) = Self::deflect_normal(inc, out, self.p.deflect_face_k) else {
                continue;
            };
            let center = p - Vec2::from_angle(ph) * DEFLECT_FACE_TO_BALL;
            if center.x.abs() > FIELD_ROBOT_MAX_ABS_X || center.y.abs() > FIELD_ROBOT_MAX_ABS_Y {
                continue;
            }
            // La cara que menos gira: rumbo ≡ ph módulo 90°.
            let quarter = std::f32::consts::FRAC_PI_2;
            let off = (r.theta - ph).rem_euclid(quarter);
            let (heading, rot) = if off <= quarter / 2.0 { (r.theta - off, off) } else { (r.theta + quarter - off, quarter - off) };
            let need = self.deflect_reach_time(r, center) + rot / self.p.omega_ref;
            if need + 0.2 <= t - 0.05 && best.map_or(true, |b| need < b.0) {
                best = Some((need, center, heading, p, t - 0.05));
            }
        }
        let (_, center, heading, point, t_contact) = best?;
        let contact_at = self.decision_count + (t_contact * hz).ceil() as u64;
        self.deflect_plans[id] = Some(DeflectPlan { point, center, heading, contact_at });
        Some(self.deflect_command(r, center, heading))
    }

    /// Tiempo para llegar al centro del desvío: el giro de `time_to_reach` y el recorrido con
    /// la rampa de `motion.max_linear_accel` (acelerar y frenar hasta `v_ref`). Con el recorrido
    /// a velocidad tope, como en `time_to_reach`, el robot llegaba tarde: en los registros giraba
    /// en el lugar y aceleraba mientras la pelota ya pasaba.
    fn deflect_reach_time(&self, r: &RobotView, center: Vec2) -> f32 {
        let d = (center - r.pos).length();
        let v = self.p.v_ref.max(0.05);
        let a = (params().motion.max_linear_accel as f32).max(0.1);
        let travel = if d < v * v / a { 2.0 * (d / a).sqrt() } else { d / v + v / a };
        self.time_to_reach(r, center) - d / v + travel
    }

    /// El desvío en curso sigue: la pelota no pasó el punto, su camino pasa a menos de 8 cm de
    /// él (la velocidad medida tiene ruido), sigue rodando, ni un rival ni nuestro atacante la
    /// tomaron y no pasó medio segundo del contacto previsto.
    fn deflect_plan_holds(
        &self,
        plan: &DeflectPlan,
        ball: Vec2,
        ball_vel: Vec2,
        striker: Option<&RobotView>,
        opp: &[RobotView],
    ) -> bool {
        let hz = self.p.decision_hz.max(1e-3);
        let late = self.decision_count > plan.contact_at + (0.5 * hz).ceil() as u64;
        let speed = ball_vel.length();
        let taken = striker.is_some_and(|st| (st.pos - ball).length() < 0.12)
            || opp.iter().any(|o| o.active && (o.pos - ball).length() < 0.12);
        if late || speed < 0.08 || taken {
            return false;
        }
        let dir = ball_vel / speed;
        let rel = plan.point - ball;
        let along = rel.dot(dir);
        along > -0.02 && (rel - dir * along).length() <= 0.08
    }

    /// `GoTo` al centro del desvío y, ya ahí, `FacePoint` al rumbo de la cara.
    fn deflect_command(&mut self, r: &RobotView, center: Vec2, heading: f32) -> SkillChoice {
        let (wanted, target) = if (center - r.pos).length() > 0.03 {
            (SkillId::GoTo, center)
        } else {
            (SkillId::FacePoint, center + Vec2::from_angle(heading) * 0.30)
        };
        let skill = self.commit(r.id, wanted, |_| false);
        SkillChoice::new(r.id, skill, target)
    }

    /// Ceder el paso (`support_yield_dist`): con el apoyo a menos de esa distancia del camino
    /// atacante → pelota (y el atacante a más de 0.25 m de ella), `Mark` a un punto al costado
    /// del camino, del lado en que ya está el apoyo (o del otro si ese cae en el área propia).
    fn yield_to_striker(&mut self, r: &RobotView, ball: Vec2, striker: Option<&RobotView>) -> Option<SkillChoice> {
        let id = r.id as usize;
        let choice = self.yield_point(r, ball, striker);
        match choice {
            Some(target) => {
                self.yielding[id] = true;
                let skill = self.commit(r.id, SkillId::Mark, |_| false);
                Some(SkillChoice::new(r.id, skill, target))
            }
            None => {
                // Al dejar de ceder, el compromiso no debe arrastrar ese `Mark` al objetivo de
                // otra rama (por ejemplo, ir a marcar nuestro arco en vez de cubrir la línea).
                if std::mem::take(&mut self.yielding[id]) {
                    self.committed[id] = None;
                }
                None
            }
        }
    }

    fn yield_point(&self, r: &RobotView, ball: Vec2, striker: Option<&RobotView>) -> Option<Vec2> {
        let gap = self.p.support_yield_dist;
        let st = striker?;
        if gap <= 0.0 {
            return None;
        }
        let path = ball - st.pos;
        let len = path.length();
        if len < 0.25 {
            return None;
        }
        let dir = path / len;
        let rel = r.pos - st.pos;
        let along = rel.dot(dir);
        let lateral = rel - dir * along;
        let side = lateral.length();
        if along <= -0.08 || along >= len || side >= gap {
            return None;
        }
        let normal = if side > 1e-3 { lateral / side } else { Vec2::new(-dir.y, dir.x) };
        let foot = st.pos + dir * along;
        [normal, -normal]
            .into_iter()
            .map(|n| self.clamp_field_robot(foot + n * (gap + 0.05)))
            .find(|p| !self.in_own_area(*p))
    }

    fn support_choice(
        &mut self,
        r: &RobotView,
        ball: Vec2,
        ball_vel: Vec2,
        striker: Option<&RobotView>,
        opp: &[RobotView],
    ) -> SkillChoice {
        self.end_deflection(r.id);
        if let Some(choice) = self.yield_to_striker(r, ball, striker) {
            return choice;
        }
        // Desvío con la pelota yendo hacia la mitad rival: antes que defender o marcar. Evaluado
        // después, casi no se activaba (0 y 1 intento en partidos de 3 min): el apoyo pasaba la
        // mitad del tiempo cubriendo la línea o marcando cuando pasaban las pelotas rápidas.
        if ball_vel.x * self.attack_sign() > 0.0 {
            if let Some(choice) = self.deflect_choice(r, ball, ball_vel, striker, opp) {
                return choice;
            }
        }
        // Repliegue (`support_cover_dist`): con la pelota del rival, delante de ella en la recta
        // pelota → arco propio, en cualquier mitad, para cortar la conducción o el pase.
        if let Some(pos) = self.cover_point(ball, striker, opp) {
            let skill = self.commit(r.id, SkillId::Mark, |_| true);
            return SkillChoice::new(r.id, skill, pos);
        }
        // Defiende (cubre la línea) con la pelota en nuestra mitad, salvo que el striker la
        // tenga sin rival encima: entonces se ofrece como receptor también desde ahí. Sin
        // esto, alrededor de la mitad de cancha el apoyo oscilaba entre receptor y
        // BlockLine y el pase moría a medio camino.
        let striker_has_it = striker.is_some_and(|st| (st.pos - ball).length() <= 0.20)
            && !opp.iter().any(|o| o.active && (o.pos - ball).length() <= 0.25);
        let s = self.attack_sign();
        let defending = match self.estilo() {
            Estilo::Equilibrado => self.in_own_half(ball) && !striker_has_it,
            Estilo::Defensivo => !(striker_has_it && !self.in_own_half(ball)),
            Estilo::Ofensivo => ball.x * s < -0.25 && !striker_has_it,
        };
        // (Se probó cubrir la recta pelota → palo lejano en vez de la del centro, como los
        // bloqueadores de ZJUNlict 2024 con el arquero en el centro: cara a cara 16-22-10 para
        // el actual, 23 goles contra 20.)
        if defending {
            let skill = self.commit(r.id, SkillId::BlockLine, |_| true);
            return SkillChoice::new(r.id, skill, self.own_goal);
        }
        // Marca al presionador (`support_screen_dist`): con la pelota nuestra, tapar al rival que
        // viene a quitársela al atacante.
        if let Some(pos) = self.screen_point(ball, striker, opp) {
            let skill = self.commit(r.id, SkillId::Mark, |_| true);
            return SkillChoice::new(r.id, skill, pos);
        }
        // Atacando con un rival suelto en nuestra mitad: marcarlo (entre él y el
        // arco, mirando a la pelota) en vez de subir a apoyar.
        if let Some(t) = self.threat(ball, opp) {
            let pos = self.mark_point(t.pos);
            let skill = self.commit(r.id, SkillId::Mark, |_| true);
            return SkillChoice::new(r.id, skill, pos);
        }
        // Ventana de pase y tiro (`pass_deflect_max_dist`): el apoyo se para donde el pase desde
        // la pelota y su salida al arco quedan libres de rivales, para que el atacante pueda
        // armar el pase y desvío; con un rival tapando busca otro punto en vez de esperar.
        if let Some(pos) = self.pass_window(ball, striker, opp) {
            let skill = self.commit(r.id, SkillId::Mark, |_| true);
            return SkillChoice::new(r.id, skill, pos);
        }
        // Estilo ofensivo: con la pelota disputada en la mitad rival (nuestro striker encima y
        // un rival sobre ella), el apoyo también va, desde el lado contrario al striker:
        // conduce hacia el palo del lado del striker, así su campo lo acerca por el otro lado.
        // (Como regla fija para todo el partido, el modo dos atacantes de ThunderVolt/USP dio
        // +0.06 ± 0.09 goles en 96 partidos contra el actual: sin efecto.)
        if self.estilo() == Estilo::Ofensivo && ball.x * s >= 0.0 {
            if let Some(st) = striker.filter(|st| (st.pos - ball).length() <= 0.20) {
                if opp.iter().any(|o| o.active && (o.pos - ball).length() <= 0.12) {
                    let up = if st.pos.y >= ball.y { 1.0 } else { -1.0 };
                    let skill = self.commit(r.id, SkillId::Dribble, |_| true);
                    return SkillChoice::new(r.id, skill, Vec2::new(self.attack_goal.x, up * (GOAL_HALF_Y - 0.06)));
                }
            }
        }
        // Pelota en el fondo/esquina rival: el striker la saca al frente del arco;
        // el support espera ahí la "segunda pelota", del lado contrario, fuera del
        // área rival (no amontonarse contra la pared con el striker).
        if self.ball_on_end_wall(ball) && ball.x * s > 0.0 {
            let side = if ball.y >= 0.0 { 1.0 } else { -1.0 };
            // (Se probó esperar a 0.30 m de la pelota hacia el frente del arco, para emparejar el
            // dos contra uno de la esquina: el apoyo quedó a 0.34 m en vez de 0.68, pero la
            // pelota salió al frente del arco igual (15 % y 18 %) y cara a cara 15-22-11 para
            // el actual, 21 goles contra 18.)
            let pos = self.away_from_ball(Vec2::new(s * (GOAL_AREA_X - 0.18), -side * 0.22), ball);
            let skill = self.commit(r.id, SkillId::Mark, |_| true);
            return SkillChoice::new(r.id, skill, pos);
        }
        // Atacando: receptor de pase a la par de la pelota, en el carril contrario al
        // striker (o al lado libre), con línea de tiro al arco y lejos de los rivales. En
        // los registros el apoyo esperaba detrás de la pelota y nunca era opción de pase.
        // Carriles en el marco de la cancha: en diagonal adelante (0.25 m hacia el arco
        // rival) y al otro lado en y. Es la posición de segunda pelota: rebotes y pelotas
        // sueltas del lado libre, con línea de tiro propia.
        let preferred = match striker {
            Some(st) if (st.pos.y - ball.y).abs() > 0.02 => -(st.pos.y - ball.y).signum(),
            _ => -ball.y.signum(),
        };
        let preferred = if preferred == 0.0 { 1.0 } else { preferred };
        let candidate = |side: f32| -> Option<Vec2> {
            let raw = Vec2::new(ball.x + s * 0.25, ball.y + self.p.pass_receiver_offset * side);
            // Si el carril de ese lado no existe (pelota pegada a la banda), no es opción.
            (raw.y.abs() <= FIELD_ROBOT_MAX_ABS_Y).then(|| self.receiver_point(raw, ball))
        };
        let open = |p: Vec2| -> bool {
            let clearance = opp.iter().filter(|o| o.active).map(|o| (o.pos - p).length()).fold(f32::INFINITY, f32::min);
            clearance >= 0.20 && !Self::shot_blocked(p, self.attack_goal, opp)
        };
        let first = candidate(preferred);
        let other = candidate(-preferred);
        let pos = match (first, other) {
            (Some(a), _) if open(a) => a,
            (_, Some(b)) if open(b) => b,
            (Some(a), _) => a,
            (None, Some(b)) => b,
            (None, None) => self.receiver_point(Vec2::new(ball.x + s * 0.05, ball.y), ball),
        };
        // Mark = ir al punto y quedar mirando a la pelota (listo para recibir). (Se probó
        // esperar con una cara hacia el arco o en la bisectriz pelota–arco: sin diferencia en
        // 96 partidos. El apoyo nunca recibe quieto en este punto, que se mantiene a
        // `standoff_from_ball` de la pelota: el 100 % de sus toques es en movimiento.)
        let skill = self.commit(r.id, SkillId::Mark, |_| true);
        SkillChoice::new(r.id, skill, pos)
    }

    /// Punto de repliegue (`support_cover_dist`): con un rival activo a menos de 0.15 m de la
    /// pelota y más cerca de ella que nuestro atacante, el punto a esa distancia delante de la
    /// pelota en la recta pelota → arco propio. `None` sin opción, sin el rival con la pelota, o
    /// si el punto cae a menos de `block_distance` + 0.10 m del arco o en el área (ahí cubrir la
    /// línea, como siempre).
    fn cover_point(&self, ball: Vec2, striker: Option<&RobotView>, opp: &[RobotView]) -> Option<Vec2> {
        let dist = self.p.support_cover_dist;
        if dist <= 0.0 {
            return None;
        }
        let d_rival = opp
            .iter()
            .filter(|o| o.active)
            .map(|o| (o.pos - ball).length())
            .fold(f32::INFINITY, f32::min);
        let d_striker = striker.map_or(f32::INFINITY, |st| (st.pos - ball).length());
        if d_rival > 0.15 || d_rival >= d_striker {
            return None;
        }
        let to_goal = self.own_goal - ball;
        let len = to_goal.length();
        if len - dist < params().skills.block_distance + 0.10 {
            return None;
        }
        let pos = self.clamp_field_robot(ball + to_goal / len * dist);
        (!self.in_own_area(pos)).then_some(pos)
    }

    /// Punto para tapar al presionador (`support_screen_dist`): con nuestro atacante a menos de
    /// 0.12 m de la pelota y más cerca que todos los rivales, el rival más cercano a la pelota,
    /// si está a 0.15 m a esa distancia de ella, se tapa desde 0.13 m delante de él, en la recta
    /// hacia la pelota. `None` sin opción o sin presionador, con la pelota a más de 0.25 m dentro
    /// de nuestra mitad, o si el punto queda a menos de 0.12 m del atacante, a menos de 0.10 m de
    /// la pelota o en el área propia.
    fn screen_point(&self, ball: Vec2, striker: Option<&RobotView>, opp: &[RobotView]) -> Option<Vec2> {
        let max = self.p.support_screen_dist;
        let st = striker?;
        let d_striker = (st.pos - ball).length();
        if max <= 0.0 || d_striker > 0.12 || ball.x * self.attack_sign() < -0.25 {
            return None;
        }
        let (d, rival) = opp
            .iter()
            .filter(|o| o.active)
            .map(|o| ((o.pos - ball).length(), o.pos))
            .min_by(|a, b| a.0.total_cmp(&b.0))?;
        if d <= d_striker || !(0.15..=max).contains(&d) {
            return None;
        }
        let pos = self.clamp_field_robot(rival + (ball - rival) / d * 0.13);
        let ok = (pos - st.pos).length() >= 0.12 && (pos - ball).length() >= 0.10 && !self.in_own_area(pos);
        ok.then_some(pos)
    }

    /// Punto de receptor legal: dentro de la cancha, a `standoff_from_ball` de la pelota,
    /// fuera de las dos áreas (la rival con la holgura del guardia de zonas).
    fn receiver_point(&self, raw: Vec2, ball: Vec2) -> Vec2 {
        use crate::skills::zones::{AreaRect, AREA_CLEARANCE};
        let s = self.attack_sign();
        let mut pos = self.clamp_field_robot(raw);
        if AreaRect::opp(s).touches_with(pos, AREA_CLEARANCE) {
            pos.x = s * (GOAL_AREA_X - AREA_CLEARANCE - 0.02);
        }
        if self.in_own_area(pos) {
            pos.x = s * -(GOAL_AREA_X - 0.08);
        }
        // Al final, para que ningún ajuste lo vuelva a acercar a la pelota.
        self.away_from_ball(pos, ball)
    }

    /// Segundos que la pelota lleva dentro del área propia (0 si no está).
    pub fn ball_seconds_in_own_area(&self) -> f32 {
        self.ball_in_area_since
            .map(|t0| (self.decision_count.saturating_sub(t0)) as f32 / self.p.decision_hz.max(1e-3))
            .unwrap_or(0.0)
    }

    /// Pelota en el área propia ampliada en `gk_engage_margin` (el borde desde donde un
    /// rival la mete conduciendo).
    fn near_own_area(&self, p: Vec2) -> bool {
        let s = self.attack_sign();
        let margin = self.p.gk_engage_margin;
        (p.x * -s) >= GOAL_AREA_X - margin && p.y.abs() <= GOAL_AREA_HALF_Y + margin
    }

    fn keeper_choice(&mut self, r: &RobotView, ball: Vec2, ball_vel: Vec2, opp: &[RobotView]) -> SkillChoice {
        let in_area = self.in_own_area(ball);
        if in_area {
            if self.ball_in_area_since.is_none() {
                self.ball_in_area_since = Some(self.decision_count);
            }
        } else {
            self.ball_in_area_since = None;
        }
        // Despejar si la pelota quedó parada en el área, sí o sí antes del límite
        // reglamentario de retención (10 s) aunque siga moviéndose, o si un rival la
        // lleva conduciendo (lenta, con él encima) por el área o su borde: esperarlo en
        // la línea es dejar que la empuje hasta adentro.
        let slow = ball_vel.length() <= self.p.gk_clear_max_ball_speed;
        let carried = ball_vel.length() <= self.p.gk_engage_max_ball_speed
            && opp.iter().any(|o| o.active && (o.pos - ball).length() <= self.p.gk_engage_opp_dist);
        // Parada en el área o pegada a su borde: nadie más puede jugarla.
        let parked = self.ball_is_keepers(ball) && slow;
        let overdue = in_area && self.ball_seconds_in_own_area() >= self.p.gk_force_clear_s;
        let challenge = carried && self.near_own_area(ball);
        // Bloquear antes que despejar una pelota que va hacia la boca del arco: en los
        // registros, el 39 % de los goles en contra entraba con el arquero despejando (y
        // despeja solo el 6 % del tiempo), alineándose mientras la pelota lenta entraba. Con
        // esta regla los goles en contra bajaron de 1,00 a 0,75 por partido (36 partidos).
        // (Se probó también ceder la pelota a un rival en contacto si el arquero no estaba
        // listo para pegarle: los goles en contra subieron de 0,83 a 1,19 y se descartó; con
        // RailKeeper quedó sin diferencia, 0,69 contra 0,58 y cara a cara −0,12 ± 0,27.)
        let incoming = self.ball_heads_into_own_goal(ball, ball_vel);
        // Pelota entre el riel y la línea de gol: Clear no tiene dónde ponerse detrás (el
        // arquero la rodeaba por delante y dejaba la boca libre: 22 de 41 goles en contra);
        // la barre el riel a lo largo de la línea.
        let on_goal_line = self.p.gk_rail && self.ball_on_own_goal_line(ball);
        if (parked || overdue || challenge) && !incoming && !on_goal_line {
            let target = self.clear_target(ball);
            let skill = self.commit(r.id, SkillId::Clear, |_| true);
            return SkillChoice::new(r.id, skill, target);
        }
        // Volver a cuidar el arco por esas reglas es inmediato (sin compromiso).
        let rule = incoming || on_goal_line;
        let skill = self.commit(r.id, self.guard_skill(), |_| !rule);
        SkillChoice::new(r.id, skill, self.own_goal)
    }

    /// Skill con la que el arquero cuida el arco.
    fn guard_skill(&self) -> SkillId {
        if self.p.gk_rail { SkillId::RailKeeper } else { SkillId::GoalKeep }
    }

    /// Pelota entre el riel del arquero y nuestra línea de gol, dentro del ancho del área.
    fn ball_on_own_goal_line(&self, ball: Vec2) -> bool {
        let depth = (ball.x - self.own_goal.x) * self.attack_sign();
        depth < params().skills.keeper_depth + 0.03 && ball.y.abs() <= GOAL_AREA_HALF_Y
    }

    /// La pelota avanza hacia nuestro arco a más de `gk_block_incoming_speed` y su
    /// trayectoria cruza la línea de gol dentro de la boca (con 5 cm de margen).
    fn ball_heads_into_own_goal(&self, ball: Vec2, ball_vel: Vec2) -> bool {
        let to_goal = (self.own_goal - ball).normalize_or_zero();
        if ball_vel.dot(to_goal) <= self.p.gk_block_incoming_speed || ball_vel.x.abs() < 1e-3 {
            return false;
        }
        let t = (self.own_goal.x - ball.x) / ball_vel.x;
        t > 0.0 && (ball.y + ball_vel.y * t).abs() <= GOAL_HALF_Y + 0.05
    }

    // ── Play: roles ──────────────────────────────────────────────────────────

    fn assign_roles(&mut self, own: &[RobotView], ball: Vec2, ball_vel: Vec2) {
        self.decisions_since_role_switch = self.decisions_since_role_switch.saturating_add(1);

        // Penal propio (formación o tiro) con un solo robot activo: patea él. Sin esto quedaba
        // de arquero (el fijo, o el más cercano al arco propio) y volvía a nuestro arco: en la
        // tanda de penales, con un robot por equipo, nunca pateábamos.
        let mut activos = own.iter().filter(|r| r.active);
        let unico = match (activos.next(), activos.next()) {
            (Some(r), None) => Some(r.id),
            _ => None,
        };
        let penal_propio = self.current_play == Play::PenaltyOurs || self.active_penalty_shot().is_some();
        self.pateador_unico = unico.filter(|_| penal_propio);
        if let Some(id) = self.pateador_unico {
            self.striker_id = Some(id);
            return;
        }

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

        // Striker: el jugador de campo que llega antes a la pelota, con histéresis. Llegar a
        // donde ESTARÁ la pelota, no a donde está: con la pelota rodando hacia el apoyo, este
        // pasaba a atacante 0.28-0.35 s después de que empezaba a venir; con la intercepción,
        // 0.17-0.20 s. En 95 partidos ganó 23-9-16 cara a cara y +0.56 ± 0.37 de aptitud contra
        // rule_based.
        let mut field: Vec<(&RobotView, f32)> = own
            .iter()
            .filter(|r| r.active && r.id != self.keeper_id)
            .map(|r| (r, self.time_to_intercept(r, ball, ball_vel)))
            .collect();
        field.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        self.depuracion_roles(&field);

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
    fn role(&self, robot_id: i32) -> Option<Role> {
        self.role_of(robot_id)
    }

    /// Entretiempo: los arcos se intercambian y se olvida lo que dependía del lado
    /// (compromisos, atacante, penal en curso, chequeo de lado). La adaptación por carril
    /// (arriba/centro/abajo) y el playbook no dependen del lado y se conservan.
    fn set_side(&mut self, attack_sign: f32) {
        let s = if attack_sign >= 0.0 { 1.0 } else { -1.0 };
        let x = self.attack_goal.x.abs().max(self.own_goal.x.abs());
        self.attack_goal = Vec2::new(s * x, 0.0);
        self.own_goal = Vec2::new(-s * x, 0.0);
        self.committed = [None; 3];
        self.striker_id = None;
        self.penalty_shot = None;
        self.kickoff_rush = None;
        self.kickoff_guard = None;
        self.push_stall = Stall::None;
        self.side_checked = false;
        self.ball_still_since = None;
        self.last_open_ball = Vec2::ZERO;
    }

    fn debug(&self) -> Option<CoachDebug> {
        Some(self.depuracion())
    }

    fn decide(&mut self, obs: &Observation) -> Vec<SkillChoice> {
        self.decision_count += 1;
        let ball = Self::ball_pos(obs);
        let ball_vel = Self::ball_vel(obs);
        if ball_vel.length() < self.p.intercept_min_ball_speed {
            self.ball_still_since.get_or_insert(self.decision_count);
        } else {
            self.ball_still_since = None;
        }
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
        let play = self.read_play(obs.own_team);
        if play != self.current_play && self.cartas.is_some() {
            let s = self.attack_sign();
            let linea = serde_json::json!({
                "t": self.decision_count,
                "evento": "jugada",
                "play": format!("{play:?}"),
                "antes": format!("{:?}", self.current_play),
                "ultima_pelota": [crate::coach::cartas::r3(self.last_open_ball.x * s), crate::coach::cartas::r3(self.last_open_ball.y)],
            });
            if let Some(reg) = self.cartas.as_mut() {
                reg.escribir(&linea);
            }
        }
        if play != self.current_play {
            self.committed = [None; 3];
            self.push_stall = Stall::None;
            // Pelota parada: los roles se reparten de cero, sin la histéresis que protege
            // al atacante anterior. Si no, el robot que queda junto a la pelota podía seguir
            // de apoyo y retroceder a cubrir la línea mientras el atacante venía de lejos: en
            // los free balls de nuestra mitad ganábamos el primer toque el 27 % de las veces
            // y 6 de 22 terminaban en gol en contra.
            if play.is_set_piece() {
                self.striker_id = None;
            }
            // Silbato de un penal propio: elegir el palo antes de que el pateador se mueva. (Se
            // probó amagar: empujar al centro y doblar al palo con la pelota pasada x = 0.50 o
            // 0.58. Empujando no se alcanza a desviar en lo que queda y la pelota sigue al centro,
            // contra el arquero: 12 % contra el nuestro igual que sin amague, 3 % doblando en 0.58,
            // y contra el de rule_based 5 % en vez de 78 %. El giro tampoco sirve de penal; ver los
            // casos `penal_*` de motion_bench.)
            // (Se probó, en el free ball, sacar la pelota en diagonal hacia el centro durante 1.5 s
            // en vez de empujarla de frente contra el pateador rival: para rodearla el atacante
            // llega tarde y el primer toque pasa del 70 % al 26 %; cara a cara 5-16-27, −0.71 ±
            // 0.16 goles, 21 goles en contra tras free balls. Ir de frente gana la carrera.)
            self.penalty_shot = (self.current_play == Play::PenaltyOurs && play == Play::Open)
                .then(|| (self.penalty_corner(&opp), self.decision_count, PENALTY_SHOT_S));
            // Saques (`kickoff_rush_s`, `kickoff_guard_s`): al silbato, la arrancada propia o la
            // muralla ante la rival.
            let silbato = play == Play::Open;
            self.kickoff_rush = if silbato && self.current_play == Play::KickoffOurs && self.p.kickoff_rush_s > 0.0 {
                let side = self.kickoff_side.unwrap_or_else(|| self.kickoff_rush_side(ball, &opp));
                self.kickoffs_taken += 1;
                let feint = self.p.kickoff_feint_s > 0.0;
                Some(KickoffRush { side, since: self.decision_count, target: self.kickoff_first_target(ball, side, &opp), feint })
            } else {
                None
            };
            self.kickoff_guard = (silbato && self.current_play == Play::KickoffTheirs && self.p.kickoff_guard_s > 0.0)
                .then_some(self.decision_count);
            self.kickoff_side = None;
            // Saque tras un gol: saca el equipo que lo recibió.
            let x = self.last_open_ball.x * self.attack_sign();
            let gol = match play {
                Play::KickoffTheirs if x > 0.55 => Some(true),
                Play::KickoffOurs if x < -0.55 => Some(false),
                _ => None,
            };
            if let (Some(a_favor), Some(pb)) = (gol, self.playbook.as_mut()) {
                self.last_open_ball = Vec2::ZERO;
                let antes = pb.vigente();
                if let Some(r) = pb.gol(a_favor) {
                    eprintln!(
                        "[coach] playbook: {} terminó en {r:?} → {} (pesos {:.2?})",
                        antes.nombre(),
                        pb.vigente().nombre(),
                        pb.pesos()
                    );
                }
            }
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

        let s = self.attack_sign();
        self.last_open_ball = ball;
        if let Some(pb) = self.playbook.as_mut() {
            let antes = pb.vigente();
            if let Some(r) = pb.observar(ball, s) {
                eprintln!(
                    "[coach] playbook: {} terminó en {r:?} → {} (pesos {:.2?})",
                    antes.nombre(),
                    pb.vigente().nombre(),
                    pb.pesos()
                );
            }
        }
        self.assign_roles(&own, ball, ball_vel);
        let striker = self.striker_id.and_then(|id| own.iter().find(|r| r.id == id).copied());
        // El arquero decide primero: si juega la pelota, los de campo no se le amontonan.
        // Pelota dentro del área propia con arquero activo: la despeja él (§9.5, solo un
        // robot en el área) y los de campo cubren la línea desde afuera; si la despeja
        // fuera del área, el striker se aparta a la distancia de apoyo.
        let keeper_choice = own
            .iter()
            .find(|r| r.active && r.id == self.keeper_id)
            .map(|r| self.keeper_choice(r, ball, ball_vel, &opp));
        let keeper_active = keeper_choice.is_some();
        let keeper_clearing = keeper_choice.is_some_and(|c| c.skill_id == SkillId::Clear);
        let ball_in_own_area = self.in_own_area(ball) && keeper_active;
        let ball_on_area_edge = self.ball_is_keepers(ball) && keeper_active;

        let support = own
            .iter()
            .find(|r| r.active && Some(r.id) != self.striker_id && r.id != self.keeper_id)
            .copied();
        self.update_kickoff(striker.as_ref(), ball, &opp);
        if self.kickoff_rush.is_none() {
            self.update_pass_play(striker.as_ref(), support.as_ref(), ball, ball_vel, &opp);
        }

        let mut choices = Vec::with_capacity(3);
        for r in own.iter().filter(|r| r.active) {
            if let Some(choice) = self.kickoff_choice(r, ball) {
                choices.push(choice);
                continue;
            }
            if let Some(choice) = self.pass_play_choice(r) {
                choices.push(choice);
                continue;
            }
            let choice = match self.role_of(r.id) {
                Some(Role::Keeper) => keeper_choice.expect("arquero activo"),
                Some(Role::Striker) if ball_in_own_area => {
                    let skill = self.commit(r.id, SkillId::BlockLine, |_| true);
                    SkillChoice::new(r.id, skill, self.own_goal)
                }
                Some(Role::Striker) if keeper_clearing => {
                    let skill = self.commit(r.id, SkillId::Mark, |_| true);
                    SkillChoice::new(r.id, skill, self.standoff_point(ball))
                }
                Some(Role::Striker) if ball_on_area_edge => {
                    // Pelota pegada al área y el arquero aún no la juega: regla, no
                    // preferencia (sin compromiso con la skill anterior), cubre la línea.
                    let skill = self.commit(r.id, SkillId::BlockLine, |_| false);
                    SkillChoice::new(r.id, skill, self.own_goal)
                }
                Some(Role::Striker) if self.holds_press_line(r, ball, &opp) => {
                    let skill = self.commit(r.id, SkillId::Mark, |_| true);
                    SkillChoice::new(r.id, skill, self.press_point(ball))
                }
                Some(Role::Striker) => self.striker_choice(r, ball, ball_vel, &opp),
                _ => self.support_choice(r, ball, ball_vel, striker.as_ref(), &opp),
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
            // Conduciendo, es tiro cuando el empuje ya es factible (el mismo criterio de ShootPush).
            striker_skill == Some(SkillId::ShootPush)
                || (striker_skill == Some(SkillId::Dribble) && self.last_scores[0].1 > 0.0),
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
        if self.cartas.is_some() {
            let linea = self.cartas_json(&own, &opp, ball, ball_vel, striker.as_ref(), support.as_ref(), &choices);
            if let Some(reg) = self.cartas.as_mut() {
                reg.escribir(&linea);
            }
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

    /// Lo que decidió el coach para el robot `id`: con `Dribble`, la opción del atacante
    /// (aproximarse o empujar se ejecutan conduciendo); si no, la skill emitida.
    fn decidida(c: &HeuristicCoach, ch: &[SkillChoice], id: i32) -> SkillId {
        let s = choice_of(ch, id).skill_id;
        if s == SkillId::Dribble {
            c.debug().and_then(|d| d.opciones).map(|o| o.aplicada).expect("conduce: hay opciones")
        } else {
            s
        }
    }

    /// Decide con `o` y devuelve `decidida` para el robot `id`.
    fn decidida_de(c: &mut HeuristicCoach, o: &Observation, id: i32) -> SkillId {
        let ch = c.decide(o);
        decidida(c, &ch, id)
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
        assert_eq!(choice_of(&ch, 2).skill_id, SkillId::RailKeeper);
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
        assert_eq!(decidida(&c, &ch, 1), SkillId::ShootPush);
        assert_eq!(choice_of(&ch, 1).target, ATTACK);

        // Robot 1 DELANTE de la pelota → no puede empujar: ApproachAligned.
        let mut c = coach();
        let o = obs(
            Vec2::new(0.2, 0.0),
            Vec2::ZERO,
            [Vec2::new(-0.5, 0.3), Vec2::new(0.32, 0.0), Vec2::new(-0.63, 0.0)],
        );
        let ch = c.decide(&o);
        assert_eq!(decidida(&c, &ch, 1), SkillId::ApproachAligned);
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
        // Pelota disputada en campo propio (rival encima) → support cubre la línea al arco.
        let mut o = obs(
            Vec2::new(-0.3, 0.2),
            Vec2::ZERO,
            [Vec2::new(-0.2, 0.2), Vec2::new(0.3, -0.3), Vec2::new(-0.63, 0.0)],
        );
        o.opp_robots[0] = robot(Vec2::new(-0.36, 0.24), 0.0);
        let ch = c.decide(&o);
        assert_eq!(c.role_of(0), Some(Role::Striker));
        let sup = choice_of(&ch, 1);
        assert_eq!(sup.skill_id, SkillId::BlockLine);
        assert_eq!(sup.target, OWN);

        // Pelota en campo rival → support va al punto de receptor (Mark: llega y mira a
        // la pelota): a la par de la pelota, en el carril contrario, fuera del área propia
        // y en el campo.
        let mut c = coach();
        let o = obs(
            Vec2::new(0.4, 0.1),
            Vec2::ZERO,
            [Vec2::new(0.3, 0.1), Vec2::new(-0.3, -0.3), Vec2::new(-0.63, 0.0)],
        );
        let ch = c.decide(&o);
        let sup = choice_of(&ch, 1);
        assert_eq!(sup.skill_id, SkillId::Mark);
        assert!(sup.target.x >= 0.38 && sup.target.y < 0.0, "receptor a la par, carril contrario: {:?}", sup.target);
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
    fn keeper_challenges_rival_carrying_ball_into_area() {
        // Partido 3: el rival condujo la pelota 8 s por nuestra área a 0.1-0.2 m/s y el
        // arquero esperó en la línea hasta el gol. Con un rival encima de una pelota
        // lenta en el área o su borde, sale a despejar.
        let mut c = coach();
        let mut o = obs(
            Vec2::new(-0.57, -0.25),
            Vec2::new(-0.15, -0.05),
            [Vec2::new(-0.35, -0.20), Vec2::new(0.2, -0.3), Vec2::new(-0.70, 0.0)],
        );
        o.opp_robots[0] = robot(Vec2::new(-0.49, -0.22), 180.0);
        let ch = c.decide(&o);
        assert_eq!(choice_of(&ch, 2).skill_id, SkillId::Clear);
        // Mientras el arquero la juega, el striker (robot 0) no se le amontona: espera a
        // la distancia de apoyo hacia el centro, mirando la pelota.
        let striker = choice_of(&ch, 0);
        assert_eq!(striker.skill_id, SkillId::Mark);
        assert!((striker.target - Vec2::new(-0.57, -0.25)).length() >= c.p.standoff_from_ball - 1e-3);
        assert!(!c.in_own_area(striker.target));
        // Misma pelota sin rival encima: sigue en la línea.
        let mut c = coach();
        o.opp_robots[0] = RobotObs::default();
        assert_eq!(choice_of(&c.decide(&o), 2).skill_id, SkillId::RailKeeper);
        // Rival encima pero junto a la línea de fondo fuera del margen del área: no sale
        // (ahí el rival no puede marcar y el arco quedaría vacío).
        let mut c = coach();
        let mut far = obs(
            Vec2::new(-0.73, -0.44),
            Vec2::ZERO,
            [Vec2::new(-0.35, -0.20), Vec2::new(0.2, -0.3), Vec2::new(-0.70, 0.0)],
        );
        far.opp_robots[0] = robot(Vec2::new(-0.68, -0.50), 180.0);
        assert_eq!(choice_of(&c.decide(&far), 2).skill_id, SkillId::RailKeeper);
    }

    #[test]
    fn keeper_blocks_a_ball_rolling_into_the_goal_instead_of_clearing_it() {
        let old = CoachParams { gk_block_incoming_speed: 9.9, ..CoachParams::default() };
        // Pelota lenta en el área (fuera de la franja del riel, 0.11 m) rodando hacia la boca
        // del arco: antes se despejaba ("parada" a 0.14 m/s); ahora el arquero la bloquea
        // desde la línea.
        let rolling = obs(
            Vec2::new(-0.62, 0.05),
            Vec2::new(-0.14, 0.0),
            [Vec2::new(-0.35, -0.20), Vec2::new(0.2, -0.3), Vec2::new(-0.70, -0.10)],
        );
        let mut before = HeuristicCoach::from_params(ATTACK, OWN, old, true);
        assert_eq!(choice_of(&before.decide(&rolling), 2).skill_id, SkillId::Clear);
        assert_eq!(choice_of(&coach().decide(&rolling), 2).skill_id, SkillId::RailKeeper);
        // La misma pelota rodando hacia la banda no entra: se despeja como siempre.
        let mut wide = rolling.clone();
        wide.ball.vy = 0.14 / 1.5;
        wide.ball.vx = 0.0;
        assert_eq!(choice_of(&coach().decide(&wide), 2).skill_id, SkillId::Clear);
    }

    #[test]
    fn exhausted_spin_is_banned_so_the_striker_tries_something_else() {
        // Serie espejo: el atacante quedaba 8 s quieto en SpinKick agotado contra la línea
        // de fondo porque el coach lo volvía a elegir. Girando con la pelota quieta más de
        // stall_kick_max_s, el giro queda prohibido un rato.
        let mut c = coach();
        // Esquina rival, pelota quieta, rival encima: SpinKick por scores.
        let mut o = obs(
            Vec2::new(0.70, 0.58),
            Vec2::ZERO,
            [Vec2::new(0.64, 0.52), Vec2::new(0.2, -0.2), Vec2::new(-0.63, 0.0)],
        );
        o.opp_robots[0] = robot(Vec2::new(0.66, 0.63), 180.0);
        assert_eq!(choice_of(&c.decide(&o), 0).skill_id, SkillId::SpinKick);
        let hz = CoachParams::default().decision_hz;
        let mut last = SkillId::SpinKick;
        for _ in 0..((CoachParams::default().stall_kick_max_s * hz) as usize + 2) {
            last = choice_of(&c.decide(&o), 0).skill_id;
        }
        assert_ne!(last, SkillId::SpinKick, "{last:?}");
        assert!(matches!(c.push_stall, Stall::SpinBanned(_)));
        // Pasada la prohibición vuelve a poder girar.
        for _ in 0..((CoachParams::default().spin_ban_s * hz) as usize + 2) {
            last = choice_of(&c.decide(&o), 0).skill_id;
        }
        assert!(matches!(c.push_stall, Stall::None | Stall::Kicking(_)), "{:?}", c.push_stall);
    }

    #[test]
    fn support_keeps_its_distance_from_a_ball_on_the_wall() {
        let mut c = coach();
        // Pelota en la banda, campo rival: el punto de apoyo clampeado al campo quedaría
        // encima de la pelota; debe quedar a la distancia de apoyo.
        let ball = Vec2::new(0.30, 0.60);
        let o = obs(
            ball,
            Vec2::ZERO,
            [Vec2::new(0.15, 0.55), Vec2::new(0.4, 0.3), Vec2::new(-0.63, 0.0)],
        );
        let ch = c.decide(&o);
        assert_eq!(c.role_of(0), Some(Role::Striker));
        let sup = choice_of(&ch, 1);
        assert_eq!(sup.skill_id, SkillId::Mark);
        assert!((sup.target - ball).length() >= c.p.standoff_from_ball - 1e-3, "{:?}", sup.target);
    }

    #[test]
    fn striker_holds_the_press_line_when_far_from_a_rival_with_the_ball() {
        // Rival encima de la pelota en su mitad, striker a 0.9 m: con press_distance
        // 0.6 espera sobre la línea pelota–arco propio a esa distancia, mirando la
        // pelota; con presión total (press_distance grande) va a disputarla.
        let ball = Vec2::new(0.30, 0.20);
        let mut o = obs(
            ball,
            Vec2::ZERO,
            [Vec2::new(-0.45, -0.10), Vec2::new(-0.60, 0.50), Vec2::new(-0.63, 0.0)],
        );
        o.opp_robots[0] = robot(Vec2::new(0.37, 0.22), 180.0);
        let mut c = coach();
        let ch = c.decide(&o);
        assert_eq!(c.role_of(0), Some(Role::Striker));
        let striker = choice_of(&ch, 0);
        assert_eq!(striker.skill_id, SkillId::Mark);
        let to_own = (OWN - ball).normalize();
        let expected = ball + to_own * c.p.press_distance;
        assert!((striker.target - expected).length() < 1e-3, "{:?} vs {:?}", striker.target, expected);

        let p = CoachParams {
            press_distance: 2.0,
            ..CoachParams::default()
        };
        let mut c = HeuristicCoach::from_params(ATTACK, OWN, p, true);
        assert_ne!(choice_of(&c.decide(&o), 0).skill_id, SkillId::Mark);
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
    fn ball_on_the_area_edge_belongs_to_the_keeper() {
        // Pelota 5 cm fuera de la línea del área (fuera del rectángulo estricto): un
        // jugador de campo no puede ponerse detrás sin tocar el área. Quieta: el arquero
        // sale a despejarla y el striker espera a la distancia de apoyo. En movimiento: el
        // arquero sigue en su arco y el striker cubre la línea, sin compromiso con la skill
        // que traía.
        let edge = Vec2::new(-0.55, 0.30);
        let mut c = coach();
        assert!(!c.in_own_area(edge) && c.ball_is_keepers(edge));
        assert!(!c.ball_is_keepers(Vec2::new(-0.50, 0.20)), "a 10 cm del área sigue siendo jugable");
        let robots = [Vec2::new(-0.40, 0.30), Vec2::new(0.2, -0.3), Vec2::new(-0.70, 0.0)];
        let ch = c.decide(&obs(edge, Vec2::ZERO, robots));
        assert_eq!(choice_of(&ch, 2).skill_id, SkillId::Clear);
        assert_eq!(choice_of(&ch, 0).skill_id, SkillId::Mark);
        let mut c = coach();
        let moving = obs(Vec2::new(-0.20, 0.30), Vec2::new(-0.6, 0.0), robots);
        let ch_far = c.decide(&moving);
        assert_ne!(choice_of(&ch_far, 0).skill_id, SkillId::BlockLine, "lejos del área el striker juega");
        let ch = c.decide(&obs(edge, Vec2::new(-0.6, 0.0), robots));
        assert_eq!(choice_of(&ch, 2).skill_id, SkillId::RailKeeper);
        assert_eq!(choice_of(&ch, 0).skill_id, SkillId::BlockLine, "cambio inmediato, sin compromiso");
    }

    #[test]
    fn keeper_forces_clear_before_retention_limit() {
        let p = CoachParams {
            gk_force_clear_s: 1.0,
            decision_hz: 10.0,
            ..CoachParams::default()
        };
        let mut c = HeuristicCoach::from_params(ATTACK, OWN, p, true);
        // Pelota moviéndose dentro del área (no "parada", fuera de la franja del riel) →
        // RailKeeper al principio...
        let o = obs(
            Vec2::new(-0.62, 0.05),
            Vec2::new(0.0, 0.3),
            [Vec2::new(0.0, 0.3), Vec2::new(0.2, -0.3), Vec2::new(-0.70, 0.0)],
        );
        assert_eq!(choice_of(&c.decide(&o), 2).skill_id, SkillId::RailKeeper);
        // ...pero tras 1 s dentro del área, despeja igual.
        let mut last = SkillId::RailKeeper;
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
    fn pass_and_deflect_play() {
        // Tiro directo tapado por un rival; el apoyo en campo rival con línea libre al arco y el
        // atacante pegado detrás de la pelota respecto del pase: el atacante conduce hacia el
        // apoyo (Dribble a su punto) y el apoyo va a su centro de desvío; cuando la pelota ya va
        // rápida hacia él, el atacante frena (Hold). Apagado, nada de eso.
        let ball = Vec2::new(0.10, -0.10);
        let p = Vec2::new(0.40, 0.20);
        let inc = (p - ball).normalize();
        let own = [ball - inc * 0.10, p, Vec2::new(-0.68, 0.0)];
        let mut o = obs(ball, Vec2::ZERO, own);
        o.opp_robots[0] = robot(Vec2::new(0.35, -0.06), 180.0);
        let mut off = coach();
        let ch = off.decide(&o);
        assert_ne!(choice_of(&ch, 1).skill_id, SkillId::GoTo, "apagado: {:?}", choice_of(&ch, 1));
        let mut c = coach();
        c.p.pass_deflect_max_dist = 0.6;
        let ch = c.decide(&o);
        assert_eq!(c.role_of(0), Some(Role::Striker));
        let pase = choice_of(&ch, 0);
        assert_eq!(pase.skill_id, SkillId::Dribble);
        assert!((pase.target - p).length() < 1e-4, "{pase:?}");
        let rec = choice_of(&ch, 1);
        assert!(matches!(rec.skill_id, SkillId::GoTo | SkillId::FacePoint), "{rec:?}");
        assert!((rec.target - p).length() <= DEFLECT_FACE_TO_BALL + 0.31, "{rec:?}");
        let mut rodando = obs(ball + inc * 0.08, inc * 0.6, own);
        rodando.opp_robots[0] = robot(Vec2::new(0.35, -0.06), 180.0);
        let ch = c.decide(&rodando);
        assert_eq!(choice_of(&ch, 0).skill_id, SkillId::Hold, "frenó para soltarla");
    }

    #[test]
    fn support_finds_a_pass_window_around_blockers() {
        // Atacante detrás de la pelota hacia el arco y un rival tapando el tiro directo. Con la
        // opción, el apoyo va a un punto con el pase y la salida al arco libres; si otro rival se
        // para en ese punto, elige otro. Apagado, no busca ventana.
        let ball = Vec2::new(0.10, -0.10);
        let st = RobotView { id: 0, pos: Vec2::new(0.0, -0.10), theta: 0.0, active: true };
        let tapa = RobotView { id: 0, pos: Vec2::new(0.35, -0.06), theta: 3.14, active: true };
        let mut c = coach();
        assert!(c.pass_window(ball, Some(&st), &[tapa]).is_none(), "apagado");
        c.p.pass_deflect_max_dist = 0.6;
        let p = c.pass_window(ball, Some(&st), &[tapa]).expect("ventana");
        let goal = c.attack_goal;
        assert!(!HeuristicCoach::line_blocked(ball, p, &[tapa], 0.09), "pase libre {p:?}");
        assert!(!HeuristicCoach::line_blocked(p, goal, &[tapa], 0.09), "tiro libre {p:?}");
        let d = (p - ball).length();
        assert!((0.29..=0.51).contains(&d) && p.x > 0.0, "{p:?}");
        let otro = RobotView { id: 1, pos: p, theta: 0.0, active: true };
        let q = c.pass_window(ball, Some(&st), &[tapa, otro]).expect("otra ventana");
        assert!((q - p).length() > 0.15, "{p:?} {q:?}");
    }

    #[test]
    fn support_covers_in_front_of_the_ball_when_the_rival_has_it() {
        // Rival con la pelota en su mitad y nuestro atacante detrás: apagado, el apoyo sigue de
        // receptor adelante; con la opción va delante de la pelota, en la recta al arco propio.
        let ball = Vec2::new(0.30, 0.10);
        let rival = RobotView { id: 0, pos: Vec2::new(0.38, 0.10), theta: 3.14, active: true };
        let st = RobotView { id: 0, pos: Vec2::new(0.50, 0.20), theta: 3.14, active: true };
        let r = RobotView { id: 1, pos: Vec2::new(0.55, -0.20), theta: 0.0, active: true };
        let mut c = coach();
        let antes = c.support_choice(&r, ball, Vec2::ZERO, Some(&st), &[rival]);
        assert!(antes.target.x > ball.x, "{:?}", antes.target);
        c.p.support_cover_dist = 0.35;
        let cub = c.support_choice(&r, ball, Vec2::ZERO, Some(&st), &[rival]);
        assert_eq!(cub.skill_id, SkillId::Mark);
        let to_goal = (c.own_goal - ball).normalize();
        let rel = cub.target - ball;
        assert!((rel.length() - 0.35).abs() < 1e-3 && rel.normalize().dot(to_goal) > 0.999, "{:?}", cub.target);
        // Nuestro atacante más cerca de la pelota que el rival: no se repliega.
        let encima = RobotView { pos: ball + Vec2::new(-0.05, 0.0), ..st };
        assert!(c.cover_point(ball, Some(&encima), &[rival]).is_none());
        // Pelota cerca de nuestro arco: cubre la línea como siempre (coach nuevo, sin el
        // compromiso con `Mark` de la decisión anterior).
        let cerca = Vec2::new(-0.40, 0.0);
        let rival_cerca = RobotView { pos: cerca + Vec2::new(0.08, 0.0), ..rival };
        let mut c = coach();
        c.p.support_cover_dist = 0.35;
        assert_eq!(c.support_choice(&r, cerca, Vec2::ZERO, Some(&st), &[rival_cerca]).skill_id, SkillId::BlockLine);
    }

    #[test]
    fn support_screens_the_rival_coming_to_press() {
        // Nuestro atacante con la pelota y un rival viniendo a presionar desde 0.35 m: con la
        // opción el apoyo se para entre ese rival y la pelota. Con el rival ya encima de la
        // pelota, o apagado, no.
        let ball = Vec2::new(0.10, 0.0);
        let st = RobotView { id: 0, pos: Vec2::new(0.02, 0.0), theta: 0.0, active: true };
        let rival = RobotView { id: 0, pos: Vec2::new(0.35, 0.25), theta: 3.6, active: true };
        let r = RobotView { id: 1, pos: Vec2::new(0.20, -0.30), theta: 0.0, active: true };
        let mut c = coach();
        assert!(c.screen_point(ball, Some(&st), &[rival]).is_none(), "apagado");
        c.p.support_screen_dist = 0.45;
        let ch = c.support_choice(&r, ball, Vec2::ZERO, Some(&st), &[rival]);
        assert_eq!(ch.skill_id, SkillId::Mark);
        let to_ball = (ball - rival.pos).normalize();
        let rel = ch.target - rival.pos;
        assert!((rel.length() - 0.13).abs() < 1e-3 && rel.normalize().dot(to_ball) > 0.999, "{:?}", ch.target);
        let encima = RobotView { pos: ball + Vec2::new(0.07, 0.03), ..rival };
        assert!(c.screen_point(ball, Some(&st), &[encima]).is_none(), "rival encima");
        let lejos = RobotView { pos: Vec2::new(0.60, 0.30), ..rival };
        assert!(c.screen_point(ball, Some(&st), &[lejos]).is_none(), "rival lejos");
    }

    /// Saque: pelota al centro, nuestros robots formados y los rivales con arquero centrado y uno
    /// parado delante de la pelota (como nuestra formación del saque rival).
    fn escena_saque(ball: Vec2, own: [Vec2; 3], rival_frente: Vec2) -> Observation {
        let mut o = obs(ball, Vec2::ZERO, own);
        o.opp_robots[2] = robot(Vec2::new(0.70, 0.0), 90.0);
        o.opp_robots[0] = robot(rival_frente, 180.0);
        o
    }

    #[test]
    fn kickoff_rush_goes_straight_and_hands_over_when_contested() {
        use crate::coach::referee::{Foul, RefTeam, RefereeCommand, apply_command, new_shared_referee};
        let shared = new_shared_referee();
        let mut c = coach(); // azul, ataca +x
        c.p.kickoff_rush_s = 5.0;
        c.set_referee(shared.clone());
        let own = [Vec2::new(-0.12, 0.0), Vec2::new(-0.35, 0.30), Vec2::new(-0.68, 0.0)];
        // Con un rival justo delante (0.24 m, tocando el círculo) todas las rectas al arco quedan
        // tapadas, también rozando el palo: primero el carril del costado, sin dudar entre opciones.
        let o = escena_saque(Vec2::ZERO, own, Vec2::new(0.24, 0.0));
        apply_command(&shared, RefereeCommand { foul: Foul::Kickoff, team: RefTeam::Blue, ..RefereeCommand::GAME_ON });
        let formado = choice_of(&c.decide(&o), 0);
        assert_eq!(formado.skill_id, SkillId::Mark, "formación");
        apply_command(&shared, RefereeCommand::GAME_ON);
        let ch = choice_of(&c.decide(&o), 0);
        assert_eq!(ch.skill_id, SkillId::Dribble);
        assert!((ch.target.x - KICKOFF_LANE.x).abs() < 1e-6 && (ch.target.y.abs() - KICKOFF_LANE.y).abs() < 1e-6, "{:?}", ch.target);
        let side = ch.target.y.signum();
        // Formado detrás de la pelota en la dirección de la arrancada.
        assert!(formado.target.normalize().dot(ch.target.normalize()) < -0.99, "{:?} {:?}", formado.target, ch.target);
        // Sigue por el carril mientras un rival tape la recta al palo de ese lado (aunque otra
        // recta quede libre); con la recta libre o pasado el carril, al palo de ese lado.
        let o_medio = escena_saque(Vec2::new(0.15, side * 0.08), [Vec2::new(0.06, side * 0.03), own[1], own[2]], Vec2::new(0.30, side * 0.10));
        assert_eq!(choice_of(&c.decide(&o_medio), 0).target, ch.target);
        let mut o2 = escena_saque(Vec2::new(0.42, side * 0.23), [Vec2::new(0.34, side * 0.20), own[1], own[2]], Vec2::new(0.26, 0.0));
        o2.opp_robots[0] = robot(Vec2::new(0.26, -side * 0.02), 180.0);
        let ch = choice_of(&c.decide(&o2), 0);
        assert_eq!(ch.skill_id, SkillId::Dribble);
        assert_eq!(ch.target, Vec2::new(ATTACK.x, side * (GOAL_HALF_Y - c.p.aim_post_margin)));
        // Un rival encima de la pelota: termina la arrancada y sigue la estrategia normal.
        let mut o3 = o2.clone();
        o3.opp_robots[0] = robot(Vec2::new(0.47, side * 0.23), 180.0);
        let _ = c.decide(&o3);
        assert!(c.kickoff_rush.is_none());
        // Apagada (por defecto): al silbato no hay arrancada.
        let shared = new_shared_referee();
        let mut d = coach();
        d.set_referee(shared.clone());
        apply_command(&shared, RefereeCommand { foul: Foul::Kickoff, team: RefTeam::Blue, ..RefereeCommand::GAME_ON });
        let _ = d.decide(&o);
        apply_command(&shared, RefereeCommand::GAME_ON);
        let _ = d.decide(&o);
        assert!(d.kickoff_rush.is_none());
    }

    #[test]
    fn kickoff_rush_grazes_the_post_past_a_rival_at_the_circle() {
        // Rival parado justo fuera del círculo (0.26 m) y arquero centrado: la recta al borde
        // interior del palo le pasa a 5.9 cm, y la arrancada va por ahí y no por el carril.
        let mut c = coach();
        let o = escena_saque(Vec2::ZERO, [Vec2::new(-0.12, 0.0), Vec2::new(-0.35, 0.30), Vec2::new(-0.68, 0.0)], Vec2::new(0.26, 0.0));
        let opp: Vec<RobotView> = o.opp_robots.iter().enumerate().map(|(i, r)| HeuristicCoach::view(i, r)).collect();
        c.p.kickoff_rush_s = 5.0;
        let t = c.kickoff_rush_target(Vec2::ZERO, 1.0, &opp);
        assert_eq!(t, Vec2::new(ATTACK.x, KICKOFF_EDGE_Y));
    }

    #[test]
    fn kickoff_rush_feints_then_cuts_and_can_push_through() {
        use crate::coach::referee::{Foul, RefTeam, RefereeCommand, apply_command, new_shared_referee};
        let shared = new_shared_referee();
        let mut c = coach();
        c.p.kickoff_rush_s = 2.5;
        c.p.kickoff_feint_s = 0.5;
        c.p.kickoff_rush_push = true;
        c.set_referee(shared.clone());
        let own = [Vec2::new(-0.12, 0.0), Vec2::new(-0.35, 0.30), Vec2::new(-0.68, 0.0)];
        let o = escena_saque(Vec2::ZERO, own, Vec2::new(0.26, 0.0));
        apply_command(&shared, RefereeCommand { foul: Foul::Kickoff, team: RefTeam::Blue, ..RefereeCommand::GAME_ON });
        let _ = c.decide(&o);
        apply_command(&shared, RefereeCommand::GAME_ON);
        let amague = choice_of(&c.decide(&o), 0).target;
        let side = -amague.y.signum();
        assert_eq!(amague, Vec2::new(ATTACK.x, -side * KICKOFF_EDGE_Y), "primero al palo contrario");
        // A los 0.5 s (5 decisiones) corta al lado elegido; un rival encima no la corta.
        let mut o2 = escena_saque(Vec2::new(0.10, 0.0), [Vec2::new(0.02, 0.0), own[1], own[2]], Vec2::new(0.16, 0.02));
        o2.opp_robots[0] = robot(Vec2::new(0.16, 0.02), 180.0);
        let mut corte = Vec2::ZERO;
        for _ in 0..6 {
            corte = choice_of(&c.decide(&o2), 0).target;
        }
        assert!(c.kickoff_rush.is_some(), "con rival encima sigue empujando");
        assert_eq!(corte.x, ATTACK.x);
        assert!(corte.y * side > 0.0, "cortó al lado elegido: {corte:?}");
    }

    #[test]
    fn kickoff_rush_with_a_clear_line_goes_to_the_post_away_from_the_keeper() {
        use crate::coach::referee::{Foul, RefTeam, RefereeCommand, apply_command, new_shared_referee};
        let shared = new_shared_referee();
        let mut c = coach();
        c.p.kickoff_rush_s = 5.0;
        c.set_referee(shared.clone());
        let own = [Vec2::new(-0.12, 0.0), Vec2::new(-0.35, 0.30), Vec2::new(-0.68, 0.0)];
        // Arquero rival corrido hacia +y y el otro rival lejos del centro: al palo de −y.
        let mut o = escena_saque(Vec2::ZERO, own, Vec2::new(0.30, 0.40));
        o.opp_robots[2] = robot(Vec2::new(0.70, 0.08), 90.0);
        apply_command(&shared, RefereeCommand { foul: Foul::Kickoff, team: RefTeam::Blue, ..RefereeCommand::GAME_ON });
        let _ = c.decide(&o);
        apply_command(&shared, RefereeCommand::GAME_ON);
        let ch = choice_of(&c.decide(&o), 0);
        assert_eq!((ch.skill_id, ch.target), (SkillId::Dribble, Vec2::new(ATTACK.x, -(GOAL_HALF_Y - c.p.aim_post_margin))));
    }

    #[test]
    fn kickoff_wall_waits_on_the_line_until_the_ball_leaves_the_center() {
        use crate::coach::referee::{Foul, RefTeam, RefereeCommand, apply_command, new_shared_referee};
        let shared = new_shared_referee();
        let mut c = coach();
        c.p.kickoff_guard_s = 5.0;
        c.set_referee(shared.clone());
        let own = [Vec2::new(-0.26, 0.0), Vec2::new(-0.40, 0.30), Vec2::new(-0.68, 0.0)];
        let mut o = obs(Vec2::ZERO, Vec2::ZERO, own);
        o.opp_robots[0] = robot(Vec2::new(0.12, 0.0), 180.0);
        apply_command(&shared, RefereeCommand { foul: Foul::Kickoff, team: RefTeam::Yellow, ..RefereeCommand::GAME_ON });
        let f = c.decide(&o);
        assert_eq!(choice_of(&f, 1).target, Vec2::new(-KICKOFF_SECOND_DIST, 0.0), "apoyo formado detrás");
        apply_command(&shared, RefereeCommand::GAME_ON);
        let ch = c.decide(&o);
        let (st, sup) = (choice_of(&ch, 0), choice_of(&ch, 1));
        assert_eq!((st.skill_id, sup.skill_id), (SkillId::Mark, SkillId::Mark));
        assert!((st.target - Vec2::new(-KICKOFF_WALL_DIST, 0.0)).length() < 1e-5, "{:?}", st.target);
        assert!((sup.target - Vec2::new(-KICKOFF_SECOND_DIST, 0.0)).length() < 1e-5, "{:?}", sup.target);
        // La pelota salió del centro hacia nosotros: se juega normal (el atacante va a la pelota).
        let o2 = obs(Vec2::new(-0.14, 0.0), Vec2::new(-0.6, 0.0), own);
        let ch = c.decide(&o2);
        assert!(c.kickoff_guard.is_none());
        assert_ne!(choice_of(&ch, 0).target, Vec2::new(-0.14 - KICKOFF_WALL_DIST, 0.0));
    }

    #[test]
    fn cards_describe_the_strikers_options() {
        // Atacante detrás de la pelota hacia el arco rival: sin rivales el arco está abierto;
        // con un rival delante tapando el centro, solo en parte. La línea dice qué eligió.
        let ball = Vec2::new(0.30, 0.0);
        let own = [Vec2::new(0.22, 0.0), Vec2::new(0.25, 0.30), Vec2::new(-0.68, 0.0)];
        let mut o = obs(ball, Vec2::ZERO, own);
        let mut c = coach();
        let views = |o: &Observation| -> (Vec<RobotView>, Vec<RobotView>) {
            (
                o.own_robots.iter().enumerate().map(|(i, r)| HeuristicCoach::view(i, r)).collect(),
                o.opp_robots.iter().enumerate().map(|(i, r)| HeuristicCoach::view(i, r)).collect(),
            )
        };
        let ch = c.decide(&o);
        let (ov, rv) = views(&o);
        let j = c.cartas_json(&ov, &rv, ball, Vec2::ZERO, Some(&ov[0]), Some(&ov[1]), &ch);
        assert_eq!(j["directo"]["abierto"], 1.0, "{j}");
        assert_eq!(j["posesion"], "nuestra");
        assert!(j["eleccion"]["atacante"]["skill"].is_string(), "{j}");
        assert_eq!(j["pelota"][0], 0.3);
        o.opp_robots[0] = robot(Vec2::new(0.50, 0.0), 180.0);
        let ch = c.decide(&o);
        let (ov, rv) = views(&o);
        let j = c.cartas_json(&ov, &rv, ball, Vec2::ZERO, Some(&ov[0]), Some(&ov[1]), &ch);
        let abierto = j["directo"]["abierto"].as_f64().unwrap();
        assert!(abierto > 0.0 && abierto < 1.0, "{j}");
        assert!(j["pase"]["linea_libre"].as_f64().unwrap() > 0.0);
    }

    #[test]
    fn support_steps_aside_from_the_strikers_path() {
        // Pelota en nuestra mitad (el apoyo cubriría la línea) y el apoyo parado en el camino del
        // atacante a la pelota: con la opción se corre al costado, a más de la distancia pedida.
        let ball = Vec2::new(-0.2, 0.0);
        let st = RobotView { id: 0, pos: Vec2::new(-0.55, 0.0), theta: 0.0, active: true };
        let r = RobotView { id: 1, pos: Vec2::new(-0.40, 0.02), theta: 0.0, active: true };
        let mut c = coach();
        assert_eq!(c.support_choice(&r, ball, Vec2::ZERO, Some(&st), &[]).skill_id, SkillId::BlockLine);
        c.p.support_yield_dist = 0.12;
        let aside = c.support_choice(&r, ball, Vec2::ZERO, Some(&st), &[]);
        assert_eq!(aside.skill_id, SkillId::Mark);
        assert!(aside.target.y >= 0.12 && !c.in_own_area(aside.target), "{:?}", aside.target);
        // Lejos del camino, o con el atacante ya sobre la pelota: no cede.
        let lejos = RobotView { pos: Vec2::new(-0.40, 0.30), ..r };
        assert_eq!(c.support_choice(&lejos, ball, Vec2::ZERO, Some(&st), &[]).skill_id, SkillId::BlockLine);
        let encima = RobotView { pos: ball + Vec2::new(-0.10, 0.0), ..st };
        assert_eq!(c.support_choice(&r, ball, Vec2::ZERO, Some(&encima), &[]).skill_id, SkillId::BlockLine);
    }

    #[test]
    fn spin_waits_for_the_ball_to_stay_still() {
        // Con `spin_min_still_s`, el rival encima no basta: la pelota tiene que llevar ese tiempo
        // quieta. A 10 Hz, 0.4 s son 4 decisiones; si se mueve, la cuenta vuelve a cero.
        let ball = Vec2::new(0.1, 0.0);
        let mut o = obs(ball, Vec2::ZERO, [Vec2::new(0.25, 0.05), Vec2::new(-0.4, -0.3), Vec2::new(-0.63, 0.0)]);
        o.opp_robots[0] = robot(Vec2::new(0.16, 0.0), 180.0);
        let mut c = coach();
        c.p.spin_min_still_s = 0.4;
        let first = c.decide(&o);
        assert_ne!(choice_of(&first, 0).skill_id, SkillId::SpinKick);
        let mut last = first;
        for _ in 0..5 {
            last = c.decide(&o);
        }
        assert_eq!(choice_of(&last, 0).skill_id, SkillId::SpinKick);
        let mut moving = o.clone();
        moving.ball.vx = 0.5 / 1.5;
        c.decide(&moving);
        let again = c.decide(&o);
        assert_ne!(choice_of(&again, 0).skill_id, SkillId::SpinKick, "la cuenta volvió a cero");
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
        assert_eq!(decidida_de(&mut c, &behind, 1), SkillId::ShootPush);
        // Un tick de percepción dice que la pelota se movió delante del robot:
        // ShootPush deja de ser factible → cambia de inmediato (no se finge empuje).
        let flicker = obs(
            Vec2::new(0.2, 0.0),
            Vec2::ZERO,
            [Vec2::new(-0.5, 0.3), Vec2::new(0.30, 0.0), Vec2::new(-0.63, 0.0)],
        );
        assert_eq!(decidida_de(&mut c, &flicker, 1), SkillId::ApproachAligned);
        // Vuelve a estar detrás: ApproachAligned se mantiene el hold mínimo
        // aunque ShootPush ya sea factible otra vez (evita el ping-pong).
        assert_eq!(decidida_de(&mut c, &behind, 1), SkillId::ApproachAligned);
        let mut last = SkillId::ApproachAligned;
        for _ in 0..CoachParams::default().skill_min_hold {
            last = decidida_de(&mut c, &behind, 1);
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
        assert_eq!(decidida(&c, &ch, 1), SkillId::ShootPush);
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
        assert_eq!(t.y, ball.y, "a lo largo de la pared, sin ángulo: {t:?}");
        // Sin pared: apunta al arco.
        assert_eq!(c.striker_target(Vec2::new(0.0, 0.2), &[]), ATTACK);
    }

    #[test]
    fn ball_on_end_wall_is_pushed_along_the_wall() {
        let c = coach();
        // Fondo rival (atacamos +x), fuera de la esquina: a lo largo del fondo hacia la boca.
        let t = c.striker_target(Vec2::new(0.73, 0.45), &[]);
        assert_eq!(t.x, 0.73, "{t:?}");
        assert!(t.y > 0.0 && t.y < 0.20, "hacia la boca del arco por la misma pared: {t:?}");
        // Fondo propio: a lo largo del fondo hacia la esquina (de ahí la saca el giro).
        let t = c.striker_target(Vec2::new(-0.73, -0.40), &[]);
        assert_eq!(t.x, -0.73, "{t:?}");
        assert!(t.y < -0.40, "hacia la esquina: {t:?}");
        // El staging de approach queda dentro del campo lógico y detrás de la pelota sobre
        // la línea de empuje: el empuje es factible desde ahí.
        let ball = Vec2::new(0.73, 0.45);
        let sk = crate::skills::tactical::ApproachAlignedSkill::new(c.striker_target(ball, &[]));
        let st = sk.staging_point(ball).unwrap();
        assert!(st.y > ball.y && st.x <= 0.70, "staging por detrás, dentro del campo: {st:?}");
        assert!(crate::skills::tactical::shoot_push_feasible_now(st, ball, c.striker_target(ball, &[])));
    }

    #[test]
    fn striker_parked_at_a_wall_staging_with_no_push_switches_to_spin() {
        // Traza real: pelota quieta en la banda, striker ya en el staging (clampado al campo
        // lógico, 6 cm fuera de la línea de empuje): ShootPush infactible por 1 cm y approach
        // "terminado" sin moverse 8 s. Con el empuje infactible y el staging alcanzado, el
        // giro tiene que ganar aunque la adaptación lo haya castigado.
        let p = CoachParams { score_hysteresis: 0.2, ..CoachParams::default() };
        let mut c = HeuristicCoach::from_params(ATTACK, OWN, p, true);
        let ball = Vec2::new(0.28, 0.63);
        // Primero llega desde lejos (se compromete con approach)...
        let far = [Vec2::new(-0.10, 0.30), Vec2::new(-0.20, -0.30), Vec2::new(-0.68, 0.0)];
        let ch = c.decide(&obs(ball, Vec2::ZERO, far));
        assert_eq!(c.role_of(0), Some(Role::Striker));
        assert_eq!(decidida(&c, &ch, 0), SkillId::ApproachAligned);
        // ...y ya en el staging, pasado el compromiso mínimo, cambia al giro.
        let parked = [Vec2::new(0.20, 0.57), Vec2::new(-0.20, -0.30), Vec2::new(-0.68, 0.0)];
        let mut last = SkillId::ApproachAligned;
        for _ in 0..(CoachParams::default().skill_min_hold + 2) {
            last = decidida_de(&mut c, &obs(ball, Vec2::ZERO, parked), 0);
        }
        assert_eq!(last, SkillId::SpinKick, "{:?}", c.last_striker_scores());
    }

    #[test]
    fn deflect_normal_is_the_mirror_bisector_and_bends_with_a_soft_face() {
        let inc = Vec2::new(0.664, 0.747).normalize();
        let out = Vec2::new(0.928, -0.371).normalize();
        // Espejo (k = 1): la normal es la bisectriz salida − llegada.
        let mirror = HeuristicCoach::deflect_normal(inc, out, 1.0).unwrap();
        assert!(Vec2::from_angle(mirror).dot((out - inc).normalize()) > 0.9999, "{mirror}");
        // Cara que apaga la normal (k = 0.1): la salida inc − 1.1 (inc·n) n va por `out`.
        let n = Vec2::from_angle(HeuristicCoach::deflect_normal(inc, out, 0.1).unwrap());
        let v = inc - n * (1.1 * inc.dot(n));
        assert!(inc.dot(n) < 0.0 && v.normalize().dot(out) > 0.9999);
    }

    #[test]
    fn support_deflects_a_rolling_ball_toward_the_goal() {
        // Pase rodando a 1.2 m/s desde (0, −0.45) hacia arriba a la derecha; el apoyo en
        // (0.48, 0.20) llega antes a un punto del camino: va (GoTo) al centro del robot que deja
        // una cara en ese punto y, ya ahí, gira (FacePoint) a un rumbo con esa cara.
        let ball = Vec2::new(0.0, -0.45);
        let vel = Vec2::new(0.79, 0.90);
        let st = RobotView { id: 0, pos: Vec2::new(-0.20, -0.60), theta: 0.0, active: true };
        let sup = RobotView { id: 1, pos: Vec2::new(0.48, 0.20), theta: 0.0, active: true };
        let mut c = coach();
        assert!(c.deflect_choice(&sup, ball, vel, Some(&st), &[]).is_none(), "apagado por defecto");
        c.p.deflect_min_speed = 0.3;
        let go = c.deflect_choice(&sup, ball, vel, Some(&st), &[]).expect("desvío");
        assert_eq!(go.skill_id, SkillId::GoTo);
        let dir = vel.normalize();
        let off_line = |p: Vec2| (p - ball - dir * (p - ball).dot(dir)).length();
        assert!(off_line(go.target) <= DEFLECT_FACE_TO_BALL + 1e-3 && go.target.x > 0.3, "{:?}", go.target);
        // Ya en el centro: gira; con ese rumbo, una de sus cuatro caras queda sobre el camino.
        let at = RobotView { pos: go.target, ..sup };
        let face = c.deflect_choice(&at, ball, vel, Some(&st), &[]).expect("sigue el desvío");
        assert_eq!(face.skill_id, SkillId::FacePoint);
        let d = face.target - go.target;
        let heading = d.y.atan2(d.x);
        let on_path = (0..4).any(|m| {
            let n = Vec2::from_angle(heading + m as f32 * FRAC_PI_2);
            off_line(go.target + n * DEFLECT_FACE_TO_BALL) < 2e-3
        });
        assert!(on_path, "rumbo {:?}", heading);
        // Lenta (no llega a 0.3 m/s a la mitad rival) o llevada por el atacante: no desvía.
        let mut c = coach();
        c.p.deflect_min_speed = 0.3;
        assert!(c.deflect_choice(&sup, ball, vel * 0.2, Some(&st), &[]).is_none());
        let carrying = RobotView { pos: ball + Vec2::new(-0.05, -0.05), ..st };
        assert!(c.deflect_choice(&sup, ball, vel, Some(&carrying), &[]).is_none());
    }

    #[test]
    fn deflection_keeps_its_plan_until_the_ball_arrives() {
        // Iniciado el desvío, el plan se mantiene aunque la velocidad medida de la pelota gire
        // unos grados (ruido) y el robot ya se haya movido: mismo centro. Se suelta cuando la
        // pelota pasó el punto o cuando un rival la toma.
        let ball = Vec2::new(0.0, -0.45);
        let vel = Vec2::new(0.79, 0.90);
        let st = RobotView { id: 0, pos: Vec2::new(-0.20, -0.60), theta: 0.0, active: true };
        let sup = RobotView { id: 1, pos: Vec2::new(0.48, 0.20), theta: 0.0, active: true };
        let mut c = coach();
        c.p.deflect_min_speed = 0.3;
        let first = c.deflect_choice(&sup, ball, vel, Some(&st), &[]).expect("desvío");
        let plan = c.deflect_plans[1].expect("plan");
        let ruido = Vec2::from_angle(3f32.to_radians()).rotate(vel);
        let moved = RobotView { pos: (sup.pos + first.target) * 0.5, ..sup };
        let kept = c.deflect_choice(&moved, ball + vel * 0.1, ruido, Some(&st), &[]).expect("sigue");
        assert!(matches!(kept.skill_id, SkillId::GoTo | SkillId::FacePoint));
        assert_eq!(c.deflect_plans[1].map(|p| p.center), Some(plan.center), "mismo centro");
        // Uno solo por vez: otro robot cerca del mismo camino no arma un segundo desvío.
        let otro = RobotView { id: 0, pos: Vec2::new(0.45, 0.12), theta: 0.0, active: true };
        assert!(c.deflect_choice(&otro, ball + vel * 0.1, vel, None, &[]).is_none());
        let past = plan.point + vel.normalize() * 0.10;
        let after = c.deflect_choice(&moved, past, vel * 0.5, Some(&st), &[]);
        assert!(after.map_or(true, |a| a.target != first.target), "la pelota ya pasó: {after:?}");
        let mut c = coach();
        c.p.deflect_min_speed = 0.3;
        c.deflect_choice(&sup, ball, vel, Some(&st), &[]).expect("desvío");
        let rival = RobotView { id: 0, pos: ball + vel * 0.1 + Vec2::new(0.05, 0.0), theta: 0.0, active: true };
        assert!(!c.deflect_plan_holds(&c.deflect_plans[1].unwrap(), ball + vel * 0.1, vel, Some(&st), &[rival]));
    }

    #[test]
    fn deflection_comes_before_defending_and_replaces_receiving() {
        // Pelota rodando desde nuestra mitad hacia la rival: el apoyo, que con la pelota en
        // nuestra mitad cubriría la línea, la desvía; el atacante en ese lugar también, en vez
        // de recibirla. Apagado, el apoyo cubre la línea.
        let ball = Vec2::new(-0.05, -0.45);
        let vel = Vec2::new(0.95, 1.08);
        let st = RobotView { id: 0, pos: Vec2::new(-0.30, -0.60), theta: 0.0, active: true };
        let r = RobotView { id: 1, pos: Vec2::new(0.48, 0.20), theta: 0.0, active: true };
        let mut c = coach();
        assert_eq!(c.support_choice(&r, ball, vel, Some(&st), &[]).skill_id, SkillId::BlockLine);
        c.p.deflect_min_speed = 0.2;
        let sup = c.support_choice(&r, ball, vel, Some(&st), &[]);
        assert!(matches!(sup.skill_id, SkillId::GoTo | SkillId::FacePoint), "{sup:?}");
        let atk = c.striker_choice(&r, ball, vel, &[]);
        assert!(matches!(atk.skill_id, SkillId::GoTo | SkillId::FacePoint), "{atk:?}");
        // Yendo hacia nuestro arco, el apoyo no deja la línea.
        assert_eq!(c.support_choice(&r, ball, -vel, Some(&st), &[]).skill_id, SkillId::BlockLine);
    }

    #[test]
    fn support_becomes_a_receiver_level_with_the_ball_on_the_opposite_lane() {
        let mut c = coach();
        // Atacando +x, striker (robot 0) detrás de la pelota por el carril izquierdo.
        let ball = Vec2::new(0.30, 0.20);
        let o = obs(ball, Vec2::ZERO, [Vec2::new(0.18, 0.22), Vec2::new(-0.10, -0.10), Vec2::new(-0.68, 0.0)]);
        let ch = c.decide(&o);
        assert_eq!(c.role_of(0), Some(Role::Striker));
        let sup = choice_of(&ch, 1);
        assert_eq!(sup.skill_id, SkillId::Mark);
        assert!(sup.target.y < 0.0, "carril contrario: {:?}", sup.target);
        assert!(sup.target.x >= ball.x - 0.02, "a la par o adelante, no detrás: {:?}", sup.target);
        assert!((sup.target - ball).length() >= c.p.standoff_from_ball - 1e-3);
        assert!(!c.in_own_area(sup.target) && sup.target.x < GOAL_AREA_X - 0.10, "fuera del área rival: {:?}", sup.target);
    }

    #[test]
    fn support_receives_in_own_half_when_the_striker_has_the_ball() {
        let mut c = coach();
        // Pelota apenas en nuestra mitad, striker encima y ningún rival cerca: el apoyo no
        // se va a la línea de gol, se ofrece a la par en el carril contrario.
        let ball = Vec2::new(-0.10, 0.15);
        let o = obs(ball, Vec2::ZERO, [Vec2::new(-0.20, 0.17), Vec2::new(-0.40, -0.20), Vec2::new(-0.68, 0.0)]);
        let ch = c.decide(&o);
        assert_eq!(c.role_of(0), Some(Role::Striker));
        let sup = choice_of(&ch, 1);
        assert_eq!(sup.skill_id, SkillId::Mark, "{sup:?}");
        assert!(sup.target.x >= ball.x - 0.02 && sup.target.y < 0.0, "{:?}", sup.target);
        // Con un rival sobre la pelota, vuelve a cubrir la línea.
        let mut c2 = coach();
        let mut o2 = o.clone();
        o2.opp_robots[0] = robot(Vec2::new(-0.02, 0.10), 180.0);
        assert_eq!(choice_of(&c2.decide(&o2), 1).skill_id, SkillId::BlockLine);
    }

    #[test]
    fn walls_disable_clear_and_end_wall_ball_is_not_the_keepers() {
        let mut c = coach();
        // Pelota quieta en nuestro fondo, fuera del área: Clear no tiene staging, así que
        // no puntúa; el arquero no la reclama; el striker la juega (approach o giro).
        let ball = Vec2::new(-0.73, 0.45);
        assert!(!c.ball_is_keepers(ball));
        let o = obs(ball, Vec2::ZERO, [Vec2::new(-0.40, 0.30), Vec2::new(0.2, -0.3), Vec2::new(-0.68, 0.0)]);
        let ch = c.decide(&o);
        assert_ne!(choice_of(&ch, 0).skill_id, SkillId::Clear);
        assert_ne!(choice_of(&ch, 0).skill_id, SkillId::BlockLine);
        assert_eq!(choice_of(&ch, 2).skill_id, SkillId::RailKeeper);
        let r = RobotView { id: 0, pos: Vec2::new(-0.60, 0.45), theta: 0.0, active: true };
        let scores = c.striker_scores(&r, ball, Vec2::ZERO, &[], c.striker_target(ball, &[]));
        assert_eq!(scores.iter().find(|(s, _)| *s == SkillId::Clear).unwrap().1, 0.0);
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
        assert_eq!(choice_of(&ch, 0).skill_id, SkillId::RailKeeper);
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
    fn our_penalty_goes_to_the_post_away_from_the_keeper() {
        use crate::coach::referee::{Foul, RefTeam, RefereeCommand, apply_command, new_shared_referee};
        let shared = new_shared_referee();
        let mut c = coach(); // azul, ataca +x
        c.set_referee(shared.clone());
        let post_y = GOAL_HALF_Y - 0.06;
        let mut o = obs(
            Vec2::new(0.375, 0.0),
            Vec2::ZERO,
            [Vec2::new(0.235, 0.0), Vec2::new(-0.1, 0.35), Vec2::new(-0.68, 0.0)],
        );
        let shoot = |c: &mut HeuristicCoach, o: &Observation| {
            apply_command(&shared, RefereeCommand { foul: Foul::PenaltyKick, team: RefTeam::Blue, ..RefereeCommand::GAME_ON });
            c.decide(o);
            apply_command(&shared, RefereeCommand::GAME_ON);
            choice_of(&c.decide(o), 0).target
        };
        // Arquero rival corrido hacia +y: al palo de −y.
        o.opp_robots[2] = robot(Vec2::new(0.70, 0.05), 90.0);
        assert_eq!(shoot(&mut c, &o), Vec2::new(0.75, -post_y));
        // Arquero centrado: alterna (el primero fue el 1.º penal; este, el 2.º → −y).
        o.opp_robots[2] = robot(Vec2::new(0.70, 0.0), 90.0);
        assert_eq!(shoot(&mut c, &o), Vec2::new(0.75, -post_y));
        assert_eq!(shoot(&mut c, &o), Vec2::new(0.75, post_y));
        // Pasado el tiro, vuelve al objetivo normal.
        let steps = (PENALTY_SHOT_S * c.p.decision_hz) as usize + 2;
        let mut last = Vec2::ZERO;
        for _ in 0..steps {
            last = choice_of(&c.decide(&o), 0).target;
        }
        assert_eq!(last, ATTACK);
    }

    /// Tanda de penales: un solo robot nuestro en la cancha.
    fn solo(id: usize, pos: Vec2) -> Observation {
        let mut o = obs(Vec2::new(0.375, 0.0), Vec2::ZERO, [Vec2::new(-0.3, 0.3), Vec2::new(-0.3, -0.3), Vec2::new(-0.68, 0.0)]);
        for (i, r) in o.own_robots.iter_mut().enumerate() {
            if i == id {
                *r = robot(pos, 0.0);
            } else {
                r.active = 0.0;
            }
        }
        o.opp_robots[2] = robot(Vec2::new(0.70, 0.0), 90.0);
        o
    }

    #[test]
    fn en_la_tanda_el_unico_robot_patea_nuestro_penal() {
        use crate::coach::referee::{Foul, RefTeam, RefereeCommand, apply_command, new_shared_referee};
        // Con el arquero designado (2) y con otro robot (0): antes, los dos se iban a nuestro arco.
        for id in [2usize, 0] {
            let shared = new_shared_referee();
            let mut c = coach(); // azul, ataca +x, arquero 2
            c.set_referee(shared.clone());
            let o = solo(id, Vec2::new(0.05, 0.0));
            apply_command(&shared, RefereeCommand { foul: Foul::PenaltyKick, team: RefTeam::Blue, ..RefereeCommand::GAME_ON });
            let formacion = choice_of(&c.decide(&o), id as i32);
            assert_eq!(c.role_of(id as i32), Some(Role::Striker), "robot {id}");
            assert_eq!(formacion.skill_id, SkillId::Mark, "robot {id}: va detrás de la pelota");
            assert!(formacion.target.x > 0.2 && formacion.target.x < 0.375, "robot {id}: {:?}", formacion.target);
            // Silbato: patea al palo, no vuelve al arco propio.
            apply_command(&shared, RefereeCommand::GAME_ON);
            let tiro = choice_of(&c.decide(&o), id as i32);
            assert_eq!(c.role_of(id as i32), Some(Role::Striker), "robot {id}");
            assert_eq!(tiro.target.x, ATTACK.x, "robot {id}: {tiro:?}");
            assert!(tiro.target.y.abs() > 0.0, "robot {id}: a un palo, {tiro:?}");
        }
    }

    #[test]
    fn en_la_tanda_el_unico_robot_ataja_el_penal_rival_y_con_tres_nada_cambia() {
        use crate::coach::referee::{Foul, RefTeam, RefereeCommand, apply_command, new_shared_referee};
        // Penal en contra con un solo robot: sigue de arquero.
        let shared = new_shared_referee();
        let mut c = coach();
        c.set_referee(shared.clone());
        let mut o = solo(0, Vec2::new(-0.40, 0.0));
        o.ball.x = -0.375 / FIELD_HALF_X;
        apply_command(&shared, RefereeCommand { foul: Foul::PenaltyKick, team: RefTeam::Yellow, ..RefereeCommand::GAME_ON });
        let _ = c.decide(&o);
        assert_eq!(c.role_of(0), Some(Role::Keeper));
        // Penal propio con los tres: el arquero sigue de arquero y patea un jugador de campo.
        let shared = new_shared_referee();
        let mut c = coach();
        c.set_referee(shared.clone());
        let o = obs(Vec2::new(0.375, 0.0), Vec2::ZERO, [Vec2::new(0.235, 0.0), Vec2::new(-0.1, 0.35), Vec2::new(-0.68, 0.0)]);
        apply_command(&shared, RefereeCommand { foul: Foul::PenaltyKick, team: RefTeam::Blue, ..RefereeCommand::GAME_ON });
        let _ = c.decide(&o);
        assert_eq!(c.role_of(2), Some(Role::Keeper));
        assert_eq!(c.role_of(0), Some(Role::Striker));
    }

    #[test]
    fn open_play_styles_move_the_support_back_or_forward() {
        let support = |estilo: &str, ball: Vec2, rival: Vec2, striker: Vec2| {
            let mut c = coach();
            c.p.estilo = estilo.to_string();
            let mut o = obs(ball, Vec2::ZERO, [striker, Vec2::new(-0.1, -0.35), Vec2::new(-0.63, 0.0)]);
            o.opp_robots[0] = robot(rival, 0.0);
            choice_of(&c.decide(&o), 1)
        };
        // Pelota disputada en la mitad rival: el equilibrado se ofrece, el defensivo cubre y el
        // ofensivo va a la pelota (striker abajo de ella → conduce al palo de abajo).
        let (ball, rival, st) = (Vec2::new(0.30, 0.10), Vec2::new(0.37, 0.12), Vec2::new(0.22, 0.06));
        assert_eq!(support("equilibrado", ball, rival, st).skill_id, SkillId::Mark);
        assert_eq!(support("defensivo", ball, rival, st).skill_id, SkillId::BlockLine);
        let of = support("ofensivo", ball, rival, st);
        assert_eq!((of.skill_id, of.target), (SkillId::Dribble, Vec2::new(0.75, -(GOAL_HALF_Y - 0.06))));
        // Pelota disputada en nuestra mitad, fuera del tercio: el ofensivo no vuelve a cubrir.
        let (ball, rival, st) = (Vec2::new(-0.15, 0.20), Vec2::new(-0.21, 0.24), Vec2::new(-0.08, 0.20));
        assert_eq!(support("equilibrado", ball, rival, st).skill_id, SkillId::BlockLine);
        assert_ne!(support("ofensivo", ball, rival, st).skill_id, SkillId::BlockLine);
        // Adaptativo: hay playbook y el estilo es uno de los tres.
        let p = CoachParams { estilo: "adaptativo".to_string(), ..CoachParams::default() };
        let c = HeuristicCoach::from_params(ATTACK, OWN, p, true);
        assert!(c.playbook.is_some() && Estilo::TODOS.contains(&c.estilo()));
    }

    #[test]
    fn the_playbook_counts_a_goal_by_the_kickoff_that_follows_it() {
        use crate::coach::referee::{Foul, RefTeam, RefereeCommand, apply_command, new_shared_referee};
        let shared = new_shared_referee();
        let p = CoachParams { estilo: "adaptativo".to_string(), ..CoachParams::default() };
        let mut c = HeuristicCoach::from_params(ATTACK, OWN, p, true);
        c.set_referee(shared.clone());
        apply_command(&shared, RefereeCommand::GAME_ON);
        // Pelota junto al arco rival en juego abierto; el árbitro para y saca el amarillo.
        let o = obs(Vec2::new(0.72, 0.05), Vec2::ZERO, [Vec2::new(0.6, 0.0), Vec2::new(0.0, 0.3), Vec2::new(-0.63, 0.0)]);
        c.decide(&o);
        assert_eq!(c.playbook.as_ref().unwrap().pesos(), [1.0; 3]);
        apply_command(&shared, RefereeCommand { foul: Foul::Kickoff, team: RefTeam::Yellow, ..RefereeCommand::GAME_ON });
        c.decide(&o);
        let pesos = c.playbook.as_ref().unwrap().pesos();
        assert!(pesos.iter().any(|&w| w > 1.0), "gol a favor: {pesos:?}");
    }

    #[test]
    fn switching_sides_swaps_the_goals_and_the_attack_direction() {
        let mut c = coach(); // ataca +x
        let o = obs(Vec2::new(0.5, 0.0), Vec2::ZERO, [Vec2::new(0.4, 0.0), Vec2::new(0.0, 0.3), Vec2::new(-0.63, 0.0)]);
        c.decide(&o);
        assert!(choice_of(&c.decide(&o), 0).target.x > 0.0);
        c.set_side(-1.0);
        assert_eq!((c.attack_goal, c.own_goal), (OWN, ATTACK));
        assert_eq!(c.attack_sign(), -1.0);
        assert!(c.in_own_half(Vec2::new(0.5, 0.0)));
        // Mismo mundo, ahora esa pelota está en nuestra mitad: el arquero es el robot junto a
        // nuestro arco nuevo (+x) y el atacante empuja hacia −x.
        let o = obs(Vec2::new(-0.5, 0.0), Vec2::ZERO, [Vec2::new(-0.4, 0.0), Vec2::new(0.0, 0.3), Vec2::new(0.63, 0.0)]);
        c.decide(&o);
        let ch = c.decide(&o);
        assert!(choice_of(&ch, 0).target.x < 0.0, "{:?}", choice_of(&ch, 0));
        assert_eq!(c.role_of(2), Some(Role::Keeper));
        // Volver deja todo como al principio.
        c.set_side(1.0);
        assert_eq!((c.attack_goal, c.own_goal), (ATTACK, OWN));
    }

    #[test]
    fn set_pieces_follow_the_team_color_not_the_side_it_attacks() {
        use crate::coach::referee::{Foul, RefTeam, RefereeCommand, apply_command, new_shared_referee};
        let cmd = |foul, team| RefereeCommand { foul, team, ..RefereeCommand::GAME_ON };
        let own = [Vec2::new(0.3, 0.1), Vec2::new(0.4, -0.3), Vec2::new(0.63, 0.0)];
        // Azul jugando a la derecha (segundo tiempo): ataca hacia −x.
        let shared = new_shared_referee();
        let mut blue = HeuristicCoach::with_options(Vec2::new(-0.75, 0.0), Vec2::new(0.75, 0.0), 2, true);
        blue.set_referee(shared.clone());
        let o = obs(Vec2::ZERO, Vec2::ZERO, own);
        apply_command(&shared, cmd(Foul::PenaltyKick, RefTeam::Blue));
        blue.decide(&o);
        assert_eq!(blue.current_play(), Play::PenaltyOurs);
        apply_command(&shared, cmd(Foul::Kickoff, RefTeam::Yellow));
        blue.decide(&o);
        assert_eq!(blue.current_play(), Play::KickoffTheirs);
        // Amarillo jugando a la izquierda: ataca hacia +x.
        let shared = new_shared_referee();
        let mut yellow = coach();
        yellow.set_referee(shared.clone());
        let mut o = obs(Vec2::ZERO, Vec2::ZERO, [Vec2::new(-0.3, 0.1), Vec2::new(-0.4, -0.3), Vec2::new(-0.63, 0.0)]);
        o.own_team = 1;
        apply_command(&shared, cmd(Foul::GoalKick, RefTeam::Yellow));
        yellow.decide(&o);
        assert_eq!(yellow.current_play(), Play::GoalKickOurs);
        apply_command(&shared, cmd(Foul::FreeKick, RefTeam::Blue));
        yellow.decide(&o);
        assert_eq!(yellow.current_play(), Play::FreeKickTheirs);
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
        assert_eq!(choice_of(&ch, 2).skill_id, SkillId::RailKeeper);
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
            SkillId::ApproachAligned | SkillId::ShootPush | SkillId::Dribble | SkillId::Intercept | SkillId::Clear
        ));
    }

    #[test]
    fn free_ball_striker_is_the_robot_next_to_the_ball_not_the_previous_one() {
        use crate::coach::referee::{Foul, Quadrant, RefereeCommand, apply_command, new_shared_referee};
        let shared = new_shared_referee();
        let mut c = coach(); // azul, ataca +x
        c.set_referee(shared.clone());
        // Juego abierto en campo rival: el atacante es el robot 0.
        let open = obs(
            Vec2::new(0.35, -0.30),
            Vec2::ZERO,
            [Vec2::new(0.25, -0.30), Vec2::new(-0.20, 0.30), Vec2::new(-0.68, 0.0)],
        );
        c.decide(&open);
        assert_eq!(c.role_of(0), Some(Role::Striker));
        // Free ball en Q2 (nuestra mitad): el robot 1 quedó junto a la cruz y el 0 lejos.
        apply_command(&shared, RefereeCommand { foul: Foul::FreeBall, quadrant: Quadrant::Q2, ..RefereeCommand::GAME_ON });
        let fb = obs(
            Vec2::new(-0.375, 0.40),
            Vec2::ZERO,
            [Vec2::new(0.25, -0.30), Vec2::new(-0.575, 0.40), Vec2::new(-0.68, 0.0)],
        );
        let ch = c.decide(&fb);
        assert_eq!(c.role_of(1), Some(Role::Striker), "el que está junto a la pelota");
        assert_eq!(choice_of(&ch, 1).target, Vec2::new(-0.575, 0.40));
        // Al silbato juega la pelota él (no retrocede a cubrir la línea).
        apply_command(&shared, RefereeCommand::GAME_ON);
        let ch = c.decide(&fb);
        assert_eq!(c.role_of(1), Some(Role::Striker));
        assert!(matches!(
            choice_of(&ch, 1).skill_id,
            SkillId::ApproachAligned | SkillId::ShootPush | SkillId::Dribble | SkillId::Clear | SkillId::SpinKick
        ), "{:?}", choice_of(&ch, 1));
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
        let mut o = obs(
            Vec2::new(-0.375, 0.0),
            Vec2::ZERO,
            [Vec2::new(0.3, 0.1), Vec2::new(0.4, -0.3), Vec2::new(0.63, 0.0)],
        );
        // El color sale de la observación (VSSL_TEAM_COLOR), no del lado de ataque.
        o.own_team = 1;
        let ch = c.decide(&o);
        assert_eq!(c.current_play(), Play::PenaltyTheirs);
        // Penal en contra: jugadores de campo en la mitad rival (x < 0 para amarillo).
        for id in [0, 1] {
            assert!(choice_of(&ch, id).target.x < 0.0, "{:?}", choice_of(&ch, id));
        }
        assert_eq!(choice_of(&ch, 2).skill_id, SkillId::RailKeeper);
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
        assert_eq!(decidida(&c, &ch, 0), SkillId::ShootPush);
        assert!(score_of(&c, SkillId::ShootPush) > 0.95);
        assert!(score_of(&c, SkillId::Intercept) == 0.0 && score_of(&c, SkillId::Clear) == 0.0);

        // Mismo caso con un rival tapando el tiro: baja pero sigue ganándole al approach.
        let mut c = coach();
        let mut o2 = o.clone();
        o2.opp_robots[0] = robot(Vec2::new(0.45, 0.02), 180.0);
        c.decide(&o2);
        let blocked = score_of(&c, SkillId::ShootPush);
        assert!(blocked < 0.95 && blocked > score_of(&c, SkillId::ApproachAligned), "{blocked}");

        // Pelota en nuestro fondo (fuera del margen del arquero) con presión rival:
        // Clear domina.
        let mut c = coach();
        let mut o3 = obs(
            Vec2::new(-0.50, 0.1),
            Vec2::ZERO,
            [Vec2::new(-0.30, 0.1), Vec2::new(0.3, -0.3), Vec2::new(-0.63, 0.0)],
        );
        o3.opp_robots[0] = robot(Vec2::new(-0.40, 0.15), 180.0);
        let ch = c.decide(&o3);
        assert_eq!(choice_of(&ch, 0).skill_id, SkillId::Clear);
        // danger 0.75 × (0.6 + 0.4·presión 0.63) ≈ 0.64
        assert!(score_of(&c, SkillId::Clear) > 0.6, "{}", score_of(&c, SkillId::Clear));
    }

    #[test]
    fn feasible_shot_beats_idle_approach_even_when_blocked() {
        // Partido del 7-oct: striker alineado 14 cm detrás de la pelota libre y el
        // arquero rival tapando el palo lejano; con el approach ya "terminado" el robot
        // se quedó 50 s quieto porque un tiro tapado puntuaba menos que approach + histéresis.
        let p = CoachParams {
            keeper_id: 2,
            score_shot_blocked_factor: 0.4,
            ..CoachParams::default()
        };
        let mut c = HeuristicCoach::from_params(ATTACK, OWN, p, true);
        let mut o = obs(
            Vec2::new(0.59, 0.39),
            Vec2::ZERO,
            [Vec2::new(0.51, 0.51), Vec2::new(-0.3, 0.0), Vec2::new(-0.63, 0.0)],
        );
        o.opp_robots[0] = robot(Vec2::new(0.63, 0.18), 180.0);
        // Primero llega desde adelante (approach vigente)...
        let mut before = o.clone();
        before.own_robots[0] = robot(Vec2::new(0.70, 0.45), 0.0);
        assert_eq!(decidida_de(&mut c, &before, 0), SkillId::ApproachAligned);
        // ...y ya detrás y alineado: pasado el compromiso mínimo, el tiro tapado vale
        // más que seguir "aproximando".
        let mut last = SkillId::ApproachAligned;
        for _ in 0..CoachParams::default().skill_min_hold {
            last = decidida_de(&mut c, &o, 0);
        }
        assert_eq!(last, SkillId::ShootPush);
        let shoot = score_of(&c, SkillId::ShootPush);
        assert!(shoot < 0.5, "tiro tapado: {shoot}");
        assert!(score_of(&c, SkillId::ApproachAligned) + c.p.score_hysteresis < shoot);
    }

    #[test]
    fn pinned_push_turns_into_spin_kick_after_stall() {
        // Pelota quieta en nuestra banda con un rival encima y el striker detrás sobre la
        // recta de empuje: empuja (Clear/ShootPush), la pelota no se mueve, y pasado el
        // plazo la saca con el giro.
        let mut c = coach();
        let ball = Vec2::new(-0.50, 0.58);
        let mut o = obs(
            ball,
            Vec2::ZERO,
            [Vec2::new(-0.60, 0.60), Vec2::new(0.2, -0.2), Vec2::new(-0.63, 0.0)],
        );
        o.opp_robots[0] = robot(Vec2::new(-0.42, 0.56), 180.0);
        let first = decidida_de(&mut c, &o, 0);
        assert!(matches!(first, SkillId::Clear | SkillId::ShootPush), "{first:?}");
        // El plazo corre desde la primera decisión ya empujando (la segunda).
        let d = CoachParams::default();
        let decisions = (d.push_stall_s * d.decision_hz) as usize + 2;
        let mut last = choice_of(&c.decide(&o), 0);
        for _ in 0..decisions {
            last = choice_of(&c.decide(&o), 0);
        }
        assert_eq!(last.skill_id, SkillId::SpinKick);
        assert!(matches!(c.push_stall, Stall::Kicking(_)));
        // Gira hacia el mismo objetivo del empuje (banda: a lo largo de la pared), no al
        // compañero de campo (robot 1).
        assert!((last.target - Vec2::new(-0.15, 0.58)).length() < 1e-3, "{:?}", last.target);
        // La pelota se soltó (se aleja rodando): se vuelve a decidir por scores.
        let freed = obs(
            Vec2::new(-0.20, 0.50),
            Vec2::new(0.6, -0.1),
            [Vec2::new(-0.60, 0.60), Vec2::new(0.2, -0.2), Vec2::new(-0.63, 0.0)],
        );
        c.decide(&freed);
        assert!(matches!(c.push_stall, Stall::None));
    }

    #[test]
    fn striker_dribbles_where_it_would_approach_or_push() {
        let o = obs(Vec2::new(0.2, 0.0), Vec2::ZERO, [Vec2::new(-0.2, 0.15), Vec2::new(-0.3, -0.4), Vec2::new(-0.63, 0.0)]);
        let mut c = coach();
        let ch = choice_of(&c.decide(&o), 0);
        assert_eq!(ch.skill_id, SkillId::Dribble);
        // Las opciones del atacante siguen siendo las de siempre (depuración y compromiso), y
        // conduce hacia el mismo objetivo con que se puntuó el empuje.
        let d = c.debug().unwrap();
        assert!(matches!(d.opciones.unwrap().aplicada, SkillId::ApproachAligned | SkillId::ShootPush));
        assert_eq!(ch.target, d.objetivo.unwrap().0);
    }

    #[test]
    fn rolling_ball_goes_to_whoever_reaches_its_path_first() {
        // La pelota pasa junto al robot 0 pero rueda rápido hacia el robot 1: por distancia
        // sería del 0; por intercepción es del 1, que la espera en su camino.
        let ball = Vec2::new(0.15, 0.0);
        let own = [Vec2::new(0.0, 0.0), Vec2::new(0.6, 0.02), Vec2::new(-0.63, 0.0)];
        let mut c = coach();
        c.decide(&obs(ball, Vec2::new(0.8, 0.0), own));
        assert_eq!(c.role_of(1), Some(Role::Striker));
        // Con la pelota quieta, la intercepción es la llegada de siempre.
        let mut c = coach();
        c.decide(&obs(ball, Vec2::ZERO, own));
        assert_eq!(c.role_of(0), Some(Role::Striker));
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
        assert_eq!(choice_of(&ch, 0).skill_id, SkillId::RailKeeper);
    }
}
