//! Skills tácticas del equipo heurístico (Plan Equipo Heurístico, §5).
//!
//! Diseñadas para el robot **simétrico de dos caras**: ninguna asume que la cara
//! de contacto es el frente. Con `MotionConfig::bidirectional = true` el PID de
//! heading pliega el error a ±90° y la cara la reporta `status().face`; sin el
//! modo bidireccional se comportan como skills frontales clásicas.
//!
//! Contrato común (ver `Skill`): `tick` → comando; `is_done` → terminación
//! intrínseca; `status` → progreso / factibilidad / cara. Sin estado oculto
//! entre skills (los parámetros se setean por `set_*` antes de cada tick).
//!
//! | Skill            | Param (`target`)   | Qué hace |
//! |------------------|--------------------|----------|
//! | ApproachAligned  | punto objetivo     | llega DETRÁS de la pelota sobre la línea pelota→objetivo, alineado |
//! | ShootPush        | punto objetivo     | conduce/empuja la pelota alineado hacia el objetivo; "release" al soltarla |
//! | Intercept        | (ignorado)         | va al punto de intercepción predicho (usa velocidad filtrada de la pelota) |
//! | BlockLine        | arco propio        | se ubica sobre la línea pelota→arco propio a distancia fija del arco |

use super::{clamp_to_logical_field, is_inside_logical_field, stop_cmd, Skill};
use crate::motion::{Motion, MotionCommand, MotionConfig};
use crate::params::params;
use crate::skills::zones::{AreaRect, AREA_CLEARANCE};
use crate::world::{RobotState, World};
use glam::Vec2;

const CONTROL_KP: f64 = 1.2;
const CONTROL_KI: f64 = 0.0;
const CONTROL_KD: f64 = 0.10;

/// Cara de contacto que la skill está usando/eligiendo.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Face {
    Front,
    Back,
}

/// Estado observable de una skill para la capa táctica.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkillStatus {
    /// Avance hacia el objetivo de la skill, en [0, 1].
    pub progress: f32,
    /// La skill terminó por criterio propio.
    pub done: bool,
    /// La skill tiene sentido en el estado actual (si es `false`, la táctica
    /// debería elegir otra — p.ej. ShootPush sin estar detrás de la pelota).
    pub feasible: bool,
    /// Cara con la que la skill está trabajando.
    pub face: Face,
}

impl Default for SkillStatus {
    fn default() -> Self {
        Self {
            progress: 0.0,
            done: false,
            feasible: true,
            face: Face::Front,
        }
    }
}

/// Cara más cercana a `dir` para un robot con orientación `theta`. Si el motion
/// no es bidireccional siempre es `Front` (el robot va a girar hasta alinear).
pub fn choose_face(theta: f64, dir: Vec2, motion: &Motion) -> Face {
    if !motion.config.bidirectional || dir.length_squared() < f32::EPSILON {
        return Face::Front;
    }
    let desired = (dir.y as f64).atan2(dir.x as f64);
    let err = Motion::normalize_angle(desired - theta);
    if err.abs() <= std::f64::consts::FRAC_PI_2 {
        Face::Front
    } else {
        Face::Back
    }
}

fn clamp01(x: f32) -> f32 {
    x.clamp(0.0, 1.0)
}

/// Criterio geométrico de factibilidad de `ShootPush`, compartido con la capa
/// táctica (el coach decide con la MISMA regla que la skill ejecuta):
/// el robot está detrás de la pelota respecto de `target` (proyección sobre la
/// línea de empuje ≤ `behind_tol`), sobre esa línea (distancia lateral ≤
/// `lateral_tol`: de costado, la pelota no está frente a la cara y el empuje la
/// manda de lado) y a menos de `lose_radius` de ella.
pub fn shoot_push_feasible(
    robot_pos: Vec2,
    ball: Vec2,
    target: Vec2,
    behind_tol: f32,
    lateral_tol: f32,
    lose_radius: f32,
) -> bool {
    let dir = (target - ball).normalize_or_zero();
    if dir.length_squared() < f32::EPSILON {
        return false;
    }
    let rel = robot_pos - ball;
    rel.dot(dir) <= behind_tol && rel.perp_dot(dir).abs() <= lateral_tol && rel.length() <= lose_radius
}

/// Si el segmento `from`→`to` pasa por la pelota (a menos de `clearance` y con la
/// pelota entre medio), devuelve un punto de rodeo al costado de la pelota; si no,
/// devuelve `to`. Lo usa SpinKick para llegar a su centro de contacto, que está dentro
/// de la holgura con que motion rodea la pelota (`motion::avoid`), así que ahí la regla
/// de la tangente no actúa. Las demás skills de posición rodean la pelota con motion.
pub fn route_around_ball(from: Vec2, to: Vec2, ball: Vec2, clearance: f32) -> Vec2 {
    let seg = to - from;
    let len = seg.length();
    if len < 1e-4 {
        return to;
    }
    let dir = seg / len;
    let rel = ball - from;
    let along = rel.dot(dir);
    if along <= 0.0 || along >= len {
        return to;
    }
    let lateral = rel - dir * along;
    let lat_len = lateral.length();
    if lat_len >= clearance {
        return to;
    }
    // Rodear por el lado contrario a donde la pelota se desvía del segmento (el
    // más despejado); si está justo en línea, por la izquierda.
    let perp = if lat_len > 1e-4 {
        -lateral / lat_len
    } else {
        Vec2::new(-dir.y, dir.x)
    };
    clamp_to_logical_field(ball + perp * (clearance + 0.06))
}

/// `shoot_push_feasible` con las tolerancias vigentes de `config/team_params.json`
/// (las mismas que usa `ShootPushSkill::new`). Es lo que consulta el coach.
pub fn shoot_push_feasible_now(robot_pos: Vec2, ball: Vec2, target: Vec2) -> bool {
    let p = &params().skills;
    shoot_push_feasible(robot_pos, ball, target, p.shoot_behind_tol, p.shoot_lateral_tol, p.shoot_lose_radius)
}

// ─────────────────────────────────────────────────────────────────────────────
//  ApproachAligned
// ─────────────────────────────────────────────────────────────────────────────

/// Llega a un punto de *staging* detrás de la pelota, sobre la línea
/// pelota→objetivo, y termina alineado con esa línea (con la cara que requiera
/// menos giro). Es el paso previo obligado de `ShootPush`.
pub struct ApproachAlignedSkill {
    pub aim_point: Vec2,
    /// Distancia del staging detrás de la pelota (m).
    pub staging_offset: f32,
    /// Tolerancia posicional para considerar "en staging" (m).
    pub pos_tol: f32,
    /// Tolerancia angular de alineación con la línea (rad).
    pub angle_tol: f64,
    /// Radio desde el que el robot ya empieza a mirar a la pelota (m).
    pub pre_align_radius: f32,
    /// Modo del motion con el que corre (en `tick` se toma del `Motion`): en frontal,
    /// `is_done` exige el frente; con dos caras, cualquiera de las dos.
    pub bidirectional: bool,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

impl ApproachAlignedSkill {
    pub fn new(aim_point: Vec2) -> Self {
        let p = &params().skills;
        Self {
            aim_point,
            staging_offset: p.approach_staging_offset,
            pos_tol: p.approach_pos_tol,
            angle_tol: p.approach_angle_tol,
            pre_align_radius: p.approach_pre_align_radius,
            bidirectional: MotionConfig::from_env().bidirectional,
            kp: CONTROL_KP,
            ki: CONTROL_KI,
            kd: CONTROL_KD,
        }
    }

    pub fn set_aim_point(&mut self, p: Vec2) {
        self.aim_point = p;
    }

    /// Dirección pelota→objetivo (unitaria) o `None` si es degenerada.
    fn line_dir(&self, ball: Vec2) -> Option<Vec2> {
        let d = (self.aim_point - ball).normalize_or_zero();
        (d.length_squared() > f32::EPSILON).then_some(d)
    }

    /// Punto de staging detrás de la pelota. Si cae fuera del campo lógico
    /// (pelota pegada al borde del lado contrario), se acerca a la pelota hasta
    /// que entre y, en última instancia, se clampa.
    pub fn staging_point(&self, ball: Vec2) -> Option<Vec2> {
        let dir = self.line_dir(ball)?;
        let mut offset = self.staging_offset;
        while offset > 0.08 {
            let p = ball - dir * offset;
            if is_inside_logical_field(p) {
                return Some(p);
            }
            offset -= 0.02;
        }
        Some(clamp_to_logical_field(ball - dir * 0.08))
    }
}

impl Skill for ApproachAlignedSkill {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        self.bidirectional = motion.config.bidirectional;
        let ball = world.get_ball_state().position;
        let (Some(dir), Some(staging)) = (self.line_dir(ball), self.staging_point(ball)) else {
            return stop_cmd(robot);
        };
        let dist = (staging - robot.position).length();

        if dist <= self.pos_tol {
            // Ya en staging: solo alinear con la línea (pliega a ±90° si es bidireccional).
            let desired = (dir.y as f64).atan2(dir.x as f64);
            return motion.face_to_angle(robot, desired, self.kp, self.ki, self.kd);
        }

        // Lejos: navegar al staging mirando al staging; cerca: ya mirar a la pelota
        // para llegar orientado. Si la pelota queda en el camino (el robot está
        // "delante" de ella), motion la rodea por la tangente.
        let face_target = if dist < self.pre_align_radius {
            ball
        } else {
            staging
        };
        motion.move_and_face(robot, staging, face_target, world, self.kp, self.ki, self.kd)
    }

    fn is_done(&self, robot: &RobotState, world: &World) -> bool {
        let ball = world.get_ball_state().position;
        let (Some(dir), Some(staging)) = (self.line_dir(ball), self.staging_point(ball)) else {
            return false;
        };
        // En frontal el empuje es con el frente: de espaldas no está terminado.
        let raw = Motion::normalize_angle((dir.y as f64).atan2(dir.x as f64) - robot.orientation);
        let err = if self.bidirectional { Motion::fold_bidirectional(raw) } else { raw };
        (staging - robot.position).length() <= self.pos_tol && err.abs() <= self.angle_tol
    }

    fn current_target(&self, world: &World) -> Option<Vec2> {
        self.staging_point(world.get_ball_state().position)
    }

    fn status(&self, robot: &RobotState, world: &World) -> SkillStatus {
        let ball = world.get_ball_state().position;
        let Some(dir) = self.line_dir(ball) else {
            return SkillStatus {
                feasible: false,
                ..Default::default()
            };
        };
        let dist = self
            .staging_point(ball)
            .map(|s| (s - robot.position).length())
            .unwrap_or(1.0);
        let pos_progress = clamp01(1.0 - dist / 0.6);
        let err = Motion::fold_bidirectional(Motion::normalize_angle(
            (dir.y as f64).atan2(dir.x as f64) - robot.orientation,
        ));
        let ang_progress = clamp01(1.0 - (err.abs() / std::f64::consts::FRAC_PI_2) as f32);
        // La cara la decide el error sin plegar: si la línea queda "atrás" es Back.
        let raw = Motion::normalize_angle((dir.y as f64).atan2(dir.x as f64) - robot.orientation);
        let face = if raw.abs() <= std::f64::consts::FRAC_PI_2 {
            Face::Front
        } else {
            Face::Back
        };
        SkillStatus {
            progress: 0.7 * pos_progress + 0.3 * ang_progress,
            done: self.is_done(robot, world),
            feasible: true,
            face,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  ShootPush
// ─────────────────────────────────────────────────────────────────────────────

/// Conduce y empuja la pelota alineado hacia `target` con la cara que ya esté
/// sobre la línea. Solo es factible si el robot está DETRÁS de la pelota (respecto
/// del objetivo) y cerca; si no, devuelve stop y `status().feasible = false` para
/// que la táctica vuelva a `ApproachAligned`. Termina ("release") cuando la
/// pelota se aleja del robot más rápido de lo que él la empuja.
pub struct ShootPushSkill {
    pub target: Vec2,
    /// Cuánto más allá de la pelota se apunta el movimiento (m): alto = no frena
    /// al llegar a la pelota.
    pub push_overshoot: f32,
    /// Distancia robot-pelota a partir de la cual la skill deja de ser factible (m).
    pub lose_radius: f32,
    /// Margen para considerar "detrás de la pelota" (m, proyección sobre la línea).
    pub behind_tol: f32,
    /// Distancia lateral máxima a la línea de empuje para empujar (m).
    pub lateral_tol: f32,
    /// Velocidad de la pelota hacia el objetivo que se considera "soltada" (m/s).
    pub release_ball_speed: f32,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

impl ShootPushSkill {
    pub fn new(target: Vec2) -> Self {
        let p = &params().skills;
        Self {
            target,
            push_overshoot: p.shoot_push_overshoot,
            lose_radius: p.shoot_lose_radius,
            behind_tol: p.shoot_behind_tol,
            lateral_tol: p.shoot_lateral_tol,
            release_ball_speed: p.shoot_release_ball_speed,
            kp: CONTROL_KP,
            ki: CONTROL_KI,
            kd: CONTROL_KD,
        }
    }

    pub fn set_target(&mut self, p: Vec2) {
        self.target = p;
    }

    fn push_dir(&self, ball: Vec2) -> Option<Vec2> {
        let d = (self.target - ball).normalize_or_zero();
        (d.length_squared() > f32::EPSILON).then_some(d)
    }

    /// Factible = detrás de la pelota y sobre la línea de empuje, dentro del radio de
    /// trabajo (ver `shoot_push_feasible`).
    pub fn is_feasible(&self, robot: &RobotState, ball: Vec2) -> bool {
        shoot_push_feasible(
            robot.position,
            ball,
            self.target,
            self.behind_tol,
            self.lateral_tol,
            self.lose_radius,
        )
    }

    fn ball_speed_to_target(&self, world: &World) -> f32 {
        let b = world.get_ball_state();
        match self.push_dir(b.position) {
            Some(dir) => b.velocity.dot(dir),
            None => 0.0,
        }
    }
}

impl Skill for ShootPushSkill {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        let ball = world.get_ball_state().position;
        let Some(dir) = self.push_dir(ball) else {
            return stop_cmd(robot);
        };
        if !self.is_feasible(robot, ball) {
            return stop_cmd(robot);
        }

        // Empuje con la ley del diferencial (como `Clear`), apuntando más allá de la
        // pelota para no frenar sobre ella: si el heading no está sobre la recta de
        // empuje primero gira (avance ∝ cos del error). Un vector en marco mundo
        // proyectado sobre un heading cruzado movía al robot hacia atrás y fuera de la
        // recta, la táctica volvía a `ApproachAligned` y el ciclo nunca tocaba la pelota.
        // Es un movimiento de contacto: motion no rodea la pelota que va a empujar.
        let push_point = ball + dir * self.push_overshoot;
        motion.move_and_face_contact(robot, push_point, push_point, world, self.kp, self.ki, self.kd)
    }

    fn is_done(&self, robot: &RobotState, world: &World) -> bool {
        let b = world.get_ball_state();
        let released = self.ball_speed_to_target(world) >= self.release_ball_speed
            && (b.position - robot.position).length() > 0.12;
        let arrived = (b.position - self.target).length() < 0.05;
        released || arrived
    }

    fn current_target(&self, world: &World) -> Option<Vec2> {
        let ball = world.get_ball_state().position;
        self.push_dir(ball).map(|d| ball + d * self.push_overshoot)
    }

    fn status(&self, robot: &RobotState, world: &World) -> SkillStatus {
        let ball = world.get_ball_state().position;
        let Some(dir) = self.push_dir(ball) else {
            return SkillStatus {
                feasible: false,
                ..Default::default()
            };
        };
        let raw = Motion::normalize_angle((dir.y as f64).atan2(dir.x as f64) - robot.orientation);
        let face = if raw.abs() <= std::f64::consts::FRAC_PI_2 {
            Face::Front
        } else {
            Face::Back
        };
        SkillStatus {
            progress: clamp01(self.ball_speed_to_target(world) / self.release_ball_speed),
            done: self.is_done(robot, world),
            feasible: self.is_feasible(robot, ball),
            face,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Intercept
// ─────────────────────────────────────────────────────────────────────────────

/// Va al punto donde el robot puede alcanzar la pelota lo antes posible, usando la
/// velocidad (filtrada) de la pelota y un modelo de frenado exponencial. Con
/// pelota lenta degenera en perseguirla. Rol natural del "que llega antes".
pub struct InterceptSkill {
    /// Horizonte de predicción (s).
    pub horizon: f32,
    /// Bajo esta velocidad la pelota se considera quieta (m/s).
    pub min_ball_speed: f32,
    /// Constante de frenado exponencial de la pelota (1/s). Calibrar con datos
    /// reales (`robot_calibration.json`); 0 = sin frenado.
    pub ball_decay_per_s: f32,
    /// Retardo de reacción que se suma al tiempo de viaje del robot (s).
    pub reaction_delay: f32,
    /// Distancia robot-pelota que cuenta como "la alcancé" (m).
    pub reach_radius: f32,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

impl InterceptSkill {
    pub fn new() -> Self {
        let p = &params().skills;
        Self {
            horizon: p.intercept_horizon,
            min_ball_speed: p.intercept_min_ball_speed,
            ball_decay_per_s: p.intercept_ball_decay_per_s,
            reaction_delay: p.intercept_reaction_delay,
            reach_radius: p.intercept_reach_radius,
            kp: CONTROL_KP,
            ki: CONTROL_KI,
            kd: CONTROL_KD,
        }
    }

    /// Posición de la pelota en `t` segundos con frenado exponencial.
    fn ball_at(&self, ball: Vec2, vel: Vec2, t: f32) -> Vec2 {
        let k = self.ball_decay_per_s;
        let travel = if k > 1e-6 {
            (1.0 - (-k * t).exp()) / k
        } else {
            t
        };
        ball + vel * travel
    }

    /// Primer punto de la trayectoria de la pelota al que el robot llega a tiempo.
    pub fn intercept_point(&self, robot: &RobotState, world: &World, max_speed: f32) -> Vec2 {
        let b = world.get_ball_state();
        let speed = b.velocity.length();
        if speed < self.min_ball_speed {
            return clamp_to_logical_field(b.position);
        }
        let mut t = 0.0_f32;
        let mut candidate = self.ball_at(b.position, b.velocity, self.horizon);
        while t <= self.horizon {
            let p = self.ball_at(b.position, b.velocity, t);
            let travel = (p - robot.position).length() / max_speed.max(0.05) + self.reaction_delay;
            if travel <= t {
                candidate = p;
                break;
            }
            t += 0.05;
        }
        clamp_to_logical_field(candidate)
    }
}

impl Default for InterceptSkill {
    fn default() -> Self {
        Self::new()
    }
}

impl Skill for InterceptSkill {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        let target = self.intercept_point(robot, world, motion.config.max_linear_speed as f32);
        motion.move_and_face_contact(robot, target, target, world, self.kp, self.ki, self.kd)
    }

    fn is_done(&self, robot: &RobotState, world: &World) -> bool {
        (world.get_ball_state().position - robot.position).length() <= self.reach_radius
    }

    fn current_target(&self, world: &World) -> Option<Vec2> {
        // Sin el robot no podemos calcular la intercepción exacta: mostramos la pelota.
        Some(world.get_ball_state().position)
    }

    fn status(&self, robot: &RobotState, world: &World) -> SkillStatus {
        let dist = (world.get_ball_state().position - robot.position).length();
        SkillStatus {
            progress: clamp01(1.0 - dist / 1.0),
            done: self.is_done(robot, world),
            feasible: true,
            face: Face::Front,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  BlockLine
// ─────────────────────────────────────────────────────────────────────────────

/// Se ubica sobre la línea pelota→arco propio a `distance` del arco (cobertura
/// del tiro), mirando a la pelota. Con robot bidireccional puede cubrir de
/// espaldas y despejar sin girar. Si ese punto toca el área propia (zona prohibida
/// del `ZoneGuard` para un jugador de campo), lo corre hacia afuera por la misma línea.
pub struct BlockLineSkill {
    pub own_goal: Vec2,
    /// Distancia del punto de bloqueo al centro del arco propio (m).
    pub distance: f32,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

impl BlockLineSkill {
    pub fn new(own_goal: Vec2) -> Self {
        let p = &params().skills;
        Self {
            own_goal,
            distance: p.block_distance,
            kp: CONTROL_KP,
            ki: CONTROL_KI,
            kd: CONTROL_KD,
        }
    }

    pub fn set_own_goal(&mut self, g: Vec2) {
        self.own_goal = g;
    }

    /// Punto de bloqueo: sobre la línea pelota→arco a `distance` del arco o, si queda a
    /// menos de `AREA_CLEARANCE` del área propia (el criterio del `ZoneGuard` con más
    /// margen, la holgura con que motion rodea el área), el primero hacia afuera por esa
    /// línea que ya no lo está (pasos de 1 cm, hasta 0.75 m del arco).
    pub fn block_point(&self, ball: Vec2) -> Option<Vec2> {
        let dir = (ball - self.own_goal).normalize_or_zero();
        if dir.length_squared() < f32::EPSILON {
            return None;
        }
        let area = AreaRect { side: self.own_goal.x.signum() };
        let mut s = self.distance;
        while area.touches_with(self.own_goal + dir * s, AREA_CLEARANCE) && s < 0.75 {
            s += 0.01;
        }
        Some(clamp_to_logical_field(self.own_goal + dir * s))
    }
}

impl Skill for BlockLineSkill {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        let ball = world.get_ball_state().position;
        let Some(target) = self.block_point(ball) else {
            return stop_cmd(robot);
        };
        motion.move_and_face(robot, target, ball, world, self.kp, self.ki, self.kd)
    }

    fn current_target(&self, world: &World) -> Option<Vec2> {
        self.block_point(world.get_ball_state().position)
    }

    fn status(&self, robot: &RobotState, world: &World) -> SkillStatus {
        let ball = world.get_ball_state().position;
        let Some(target) = self.block_point(ball) else {
            return SkillStatus {
                feasible: false,
                ..Default::default()
            };
        };
        let dist = (target - robot.position).length();
        SkillStatus {
            progress: clamp01(1.0 - dist / 0.5),
            done: false,
            feasible: true,
            face: Face::Front,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Clear
// ─────────────────────────────────────────────────────────────────────────────

/// Despeje: aproximación + empuje en UNA skill con tolerancias amplias. Para
/// sacar la pelota de la zona propia no hace falta precisión: se pone detrás
/// por el camino corto y empuja hacia `target` (punto de despeje que elige la
/// táctica). Termina cuando la pelota sale disparada.
pub struct ClearSkill {
    pub target: Vec2,
    pub staging_offset: f32,
    pub behind_tol: f32,
    /// Distancia lateral máxima a la recta de despeje para empujar (la de ShootPush).
    pub lateral_tol: f32,
    pub lose_radius: f32,
    pub push_overshoot: f32,
    pub release_ball_speed: f32,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    /// Dirección efectiva de despeje del último tick (`clear_direction`); `None` antes del
    /// primero.
    eff_dir: Option<Vec2>,
    /// Fase de empuje (con histéresis, ver `push_transition`).
    pushing: bool,
}

/// Error de heading máximo (rad) respecto de la recta de despeje para entrar al empuje
/// (plegado a ±90° en bidireccional). Entrando de costado, el robot gira mientras empuja,
/// se sale de la recta y manda la pelota en ángulo.
const CLEAR_ENTRY_MAX_ERR: f64 = std::f64::consts::FRAC_PI_4;
/// Distancia lateral (m) a la recta de despeje desde la que un robot que ya empuja vuelve
/// al staging (para entrar se exige `shoot_lateral_tol`, 0.05).
const CLEAR_EXIT_LATERAL: f32 = 0.08;

/// Staging de Clear: `offset` detrás de la pelota a lo largo de `dir`, acercándose de a
/// 2 cm (hasta 0.05 m) si cae fuera del campo lógico.
fn clear_staging(ball: Vec2, dir: Vec2, offset: f32) -> Vec2 {
    let mut offset = offset;
    let mut staging = ball - dir * offset;
    while !is_inside_logical_field(staging) && offset > 0.05 {
        offset -= 0.02;
        staging = ball - dir * offset;
    }
    clamp_to_logical_field(staging)
}

/// Avance mínimo hacia el arco rival (coseno con `forward`) de una dirección de despeje
/// girada: ~87°, para que la pelota salga de nuestro campo.
const CLEAR_MIN_FORWARD: f32 = 0.05;

/// Dirección efectiva de despeje: la de la pelota al objetivo o, si su staging cae a
/// `AREA_CLEARANCE` o menos de un área prohibida para el robot, la primera girando de a 1°
/// y alternando el sentido (+1°, −1°, +2°, …) cuyo staging es legal y que, con
/// `forward` (dirección hacia el arco rival), avanza hacia él. Si ningún giro lo logra, la
/// original. `None` si la pelota está sobre el objetivo.
pub fn clear_direction(
    ball: Vec2,
    target: Vec2,
    staging_offset: f32,
    forbidden: &[AreaRect],
    forward: Option<Vec2>,
) -> Option<Vec2> {
    let d0 = (target - ball).normalize_or_zero();
    if d0.length_squared() < f32::EPSILON {
        return None;
    }
    let staging_legal = |d: Vec2| {
        let staging = clear_staging(ball, d, staging_offset);
        forbidden.iter().all(|a| a.clearance(staging) > AREA_CLEARANCE)
    };
    if staging_legal(d0) {
        return Some(d0);
    }
    let legal = |d: Vec2| staging_legal(d) && forward.is_none_or(|f| d.dot(f) > CLEAR_MIN_FORWARD);
    let turns = (1..180).flat_map(|k| [k as f32, -(k as f32)]);
    let found = turns.map(|deg| Vec2::from_angle(deg.to_radians()).rotate(d0)).find(|&d| legal(d));
    Some(found.unwrap_or(d0))
}

impl ClearSkill {
    pub fn new(target: Vec2) -> Self {
        let p = &params().skills;
        Self {
            target,
            staging_offset: p.clear_staging_offset,
            behind_tol: p.clear_behind_tol,
            lateral_tol: p.shoot_lateral_tol,
            lose_radius: p.clear_lose_radius,
            push_overshoot: p.shoot_push_overshoot,
            release_ball_speed: p.clear_release_ball_speed,
            kp: CONTROL_KP,
            ki: CONTROL_KI,
            kd: CONTROL_KD,
            eff_dir: None,
            pushing: false,
        }
    }

    pub fn set_target(&mut self, p: Vec2) {
        self.target = p;
    }

    /// Dirección de despeje: la efectiva del último tick o, antes del primero, la de la
    /// pelota al objetivo.
    fn dir(&self, ball: Vec2) -> Option<Vec2> {
        self.eff_dir.or_else(|| {
            let d = (self.target - ball).normalize_or_zero();
            (d.length_squared() > f32::EPSILON).then_some(d)
        })
    }

    /// `true` si en el último tick estaba en la fase de empuje.
    pub fn is_pushing(&self) -> bool {
        self.pushing
    }

    /// Siguiente fase dado si estaba empujando (`pushing`):
    /// - para **entrar** tiene que estar detrás de la pelota sobre la recta de despeje (la
    ///   de la dirección efectiva; `shoot_push_feasible`) y con el error de heading a lo
    ///   sumo `CLEAR_ENTRY_MAX_ERR` (plegado a ±90° si es bidireccional);
    /// - empujando, **sale** si se aparta de la recta más de `CLEAR_EXIT_LATERAL`, o si deja
    ///   de estar detrás de la pelota o la pierde (`lose_radius`).
    pub fn push_transition(&self, pushing: bool, robot: &RobotState, ball: Vec2, bidirectional: bool) -> bool {
        let Some(dir) = self.dir(ball) else {
            return false;
        };
        let rel = robot.position - ball;
        if pushing {
            return rel.perp_dot(dir).abs() <= CLEAR_EXIT_LATERAL
                && rel.dot(dir) <= self.behind_tol
                && rel.length() <= self.lose_radius;
        }
        let desired = (dir.y as f64).atan2(dir.x as f64);
        let raw = Motion::normalize_angle(desired - robot.orientation);
        let err = if bidirectional { Motion::fold_bidirectional(raw) } else { raw };
        shoot_push_feasible(robot.position, ball, ball + dir, self.behind_tol, self.lateral_tol, self.lose_radius)
            && err.abs() <= CLEAR_ENTRY_MAX_ERR
    }
}

impl Skill for ClearSkill {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        let ball = world.get_ball_state().position;
        let forbidden = motion.forbidden_areas(robot, world);
        self.eff_dir = clear_direction(ball, self.target, self.staging_offset, &forbidden, motion.attack_dir());
        let Some(dir) = self.eff_dir else {
            return stop_cmd(robot);
        };
        self.pushing = self.push_transition(self.pushing, robot, ball, motion.config.bidirectional);
        if self.pushing {
            // Empuje con la ley del diferencial (avance ∝ cos del error). Entra alineado
            // (≤ 45°): al staging se llega mirando a la pelota. Es un movimiento de
            // contacto: motion no rodea la pelota que va a empujar.
            let push_point = ball + dir * self.push_overshoot;
            return motion.move_and_face_contact(robot, push_point, push_point, world, self.kp, self.ki, self.kd);
        }
        // Al staging: si la pelota queda en el camino, motion la rodea por la tangente.
        let staging = clear_staging(ball, dir, self.staging_offset);
        motion.move_and_face(robot, staging, ball, world, self.kp, self.ki, self.kd)
    }

    fn is_done(&self, robot: &RobotState, world: &World) -> bool {
        let b = world.get_ball_state();
        match self.dir(b.position) {
            Some(dir) => {
                b.velocity.dot(dir) >= self.release_ball_speed
                    && (b.position - robot.position).length() > 0.12
            }
            None => false,
        }
    }

    fn current_target(&self, world: &World) -> Option<Vec2> {
        let ball = world.get_ball_state().position;
        self.dir(ball).map(|d| ball + d * self.push_overshoot)
    }

    fn reset(&mut self) {
        self.eff_dir = None;
        self.pushing = false;
    }

    fn status(&self, robot: &RobotState, world: &World) -> SkillStatus {
        let b = world.get_ball_state();
        let Some(dir) = self.dir(b.position) else {
            return SkillStatus {
                feasible: false,
                ..Default::default()
            };
        };
        let progress = if self.pushing {
            0.5 + 0.5 * clamp01(b.velocity.dot(dir) / self.release_ball_speed)
        } else {
            0.5 * clamp01(1.0 - (b.position - robot.position).length() / 0.6)
        };
        let raw = Motion::normalize_angle((dir.y as f64).atan2(dir.x as f64) - robot.orientation);
        let face = if raw.abs() <= std::f64::consts::FRAC_PI_2 {
            Face::Front
        } else {
            Face::Back
        };
        SkillStatus {
            progress,
            done: self.is_done(robot, world),
            feasible: true,
            face,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  SpinKick
// ─────────────────────────────────────────────────────────────────────────────

/// Patada por giro (clásica de VSSS): se pone en contacto lateral con la pelota y
/// gira a velocidad máxima; la pelota sale tangencialmente hacia `target`. Sirve
/// para sacarla de la pared o de encima de un rival, donde no hay espacio para
/// ponerse detrás. La cara no importa: cualquier lado del cuerpo cuadrado
/// lanza igual. El sentido de giro (CCW/CW) se elige por el centro de contacto
/// más cercano y se mantiene hasta soltar la pelota.
/// Velocidad del robot (m/s, del tracker) bajo la que SpinKick empieza a girar: en FIRASim,
/// arrancar el giro a ~0.2 m/s empujaba la pelota hacia afuera mientras las ruedas
/// invertían su sentido. Recalibrar con la sysid del robot real.
const SPIN_START_MAX_SPEED: f32 = 0.08;

pub struct SpinKickSkill {
    pub target: Vec2,
    /// Distancia centro del robot–pelota en el contacto (m).
    pub contact_radius: f32,
    /// Velocidad angular del giro (rad/s).
    pub omega: f64,
    pub pos_tol: f32,
    pub release_ball_speed: f32,
    pub max_spin_ticks: u32,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    spin_sign: f32,
    spinning: bool,
    spin_ticks: u32,
}

impl SpinKickSkill {
    pub fn new(target: Vec2) -> Self {
        let p = &params().skills;
        Self {
            target,
            contact_radius: p.spin_contact_radius,
            omega: p.spin_omega,
            pos_tol: p.spin_pos_tol,
            release_ball_speed: p.spin_release_ball_speed,
            max_spin_ticks: (p.spin_max_time_s * 60.0).ceil() as u32,
            kp: CONTROL_KP,
            ki: CONTROL_KI,
            kd: CONTROL_KD,
            spin_sign: 0.0,
            spinning: false,
            spin_ticks: 0,
        }
    }

    pub fn set_target(&mut self, p: Vec2) {
        self.target = p;
    }

    pub fn is_spinning(&self) -> bool {
        self.spinning
    }

    fn dir(&self, ball: Vec2) -> Option<Vec2> {
        let d = (self.target - ball).normalize_or_zero();
        (d.length_squared() > f32::EPSILON).then_some(d)
    }

    /// Centros de giro que lanzan la pelota en `dir`: `(ccw, cw)`. Con giro CCW la
    /// velocidad tangencial en la pelota es ω·perp(pelota − centro), así que el
    /// centro va del lado `(dir.y, −dir.x)` de la pelota; con CW, del opuesto.
    pub fn contact_centers(&self, ball: Vec2, dir: Vec2) -> (Vec2, Vec2) {
        let ccw = ball - Vec2::new(dir.y, -dir.x) * self.contact_radius;
        let cw = ball - Vec2::new(-dir.y, dir.x) * self.contact_radius;
        (ccw, cw)
    }

    /// Centro de contacto y sentido de giro. Mientras gira, el sentido ya elegido; si no,
    /// el centro más cercano al robot entre los que quedan dentro del campo lógico (con la
    /// pelota contra la pared, uno de los dos queda detrás de ella), o el más cercano si
    /// ninguno lo está.
    fn choose_center(&self, robot_pos: Vec2, ball: Vec2, dir: Vec2) -> (Vec2, f32) {
        let (ccw, cw) = self.contact_centers(ball, dir);
        if self.spin_sign > 0.0 {
            return (ccw, 1.0);
        }
        if self.spin_sign < 0.0 {
            return (cw, -1.0);
        }
        let ccw_first = match (is_inside_logical_field(ccw), is_inside_logical_field(cw)) {
            (true, false) => true,
            (false, true) => false,
            _ => (ccw - robot_pos).length() <= (cw - robot_pos).length(),
        };
        if ccw_first { (ccw, 1.0) } else { (cw, -1.0) }
    }
}

impl Skill for SpinKickSkill {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        let ball = world.get_ball_state().position;
        let Some(dir) = self.dir(ball) else {
            return stop_cmd(robot);
        };
        // La pelota ya se fue: cerrar el episodio de giro.
        if self.spinning && (ball - robot.position).length() > self.contact_radius * 2.0 + 0.05 {
            self.reset();
        }
        // Timeout: giró `max_spin_ticks` y la pelota sigue al lado (trabada); queda quieto
        // con `is_done` hasta que la reinicien (cambio de skill) o la pelota se aleje.
        if self.spin_ticks >= self.max_spin_ticks {
            return stop_cmd(robot);
        }
        let (center, sign) = self.choose_center(robot.position, ball, dir);
        let dist = (center - robot.position).length();
        // Gira con la pelota a `contact_radius` y el robot casi quieto: más lejos las
        // esquinas no la tocan o la rozan hacia afuera; avanzando rápido, el giro tarda en
        // levantarse (torque limitado) y el robot la empuja en vez de patearla.
        // (+2 mm de tolerancia: en el centro exacto, `reach` = `contact_radius` ± redondeo.)
        let reach = (ball - robot.position).length();
        let in_reach = reach <= self.contact_radius + 0.002;
        let settled = robot.velocity.length() < SPIN_START_MAX_SPEED;
        let spin = self.spinning || (in_reach && settled);
        if !spin {
            if dist > self.pos_tol && reach > self.contact_radius + 0.01 {
                // Al centro de contacto con la ley del diferencial (también de costado),
                // rodeando la pelota si está en medio. El destino se corre
                // `arrival_threshold` más allá a lo largo de la aproximación: motion
                // detiene al robot en el centro y no antes.
                let waypoint =
                    route_around_ball(robot.position, center, ball, self.contact_radius + 0.02);
                let goal = if waypoint == center {
                    center + (center - robot.position).normalize_or_zero() * motion.config.arrival_threshold
                } else {
                    waypoint
                };
                return motion.move_and_face(robot, goal, ball, world, self.kp, self.ki, self.kd);
            }
            // En el centro sin la pelota a `contact_radius`: arrimarse a `min_linear_speed`
            // (con la latencia de la visión, a velocidad normal llega a empujarla).
            let mut cmd = motion.move_and_face(robot, ball, ball, world, self.kp, self.ki, self.kd);
            let (v, creep) = (cmd.vx.hypot(cmd.vy), motion.config.min_linear_speed);
            if v > creep {
                cmd.vx *= creep / v;
                cmd.vy *= creep / v;
            }
            return cmd;
        }
        self.spinning = true;
        self.spin_sign = sign;
        self.spin_ticks += 1;
        MotionCommand {
            id: robot.id,
            team: robot.team,
            vx: 0.0,
            vy: 0.0,
            omega: self.omega * sign as f64,
            orientation: robot.orientation,
        }
    }

    fn is_done(&self, _robot: &RobotState, world: &World) -> bool {
        let b = world.get_ball_state();
        let Some(dir) = self.dir(b.position) else {
            return false;
        };
        b.velocity.dot(dir) >= self.release_ball_speed || self.spin_ticks >= self.max_spin_ticks
    }

    fn current_target(&self, world: &World) -> Option<Vec2> {
        Some(world.get_ball_state().position)
    }

    fn reset(&mut self) {
        self.spin_sign = 0.0;
        self.spinning = false;
        self.spin_ticks = 0;
    }

    fn status(&self, robot: &RobotState, world: &World) -> SkillStatus {
        let b = world.get_ball_state();
        let dist_ball = (b.position - robot.position).length();
        let Some(dir) = self.dir(b.position) else {
            return SkillStatus {
                feasible: false,
                ..Default::default()
            };
        };
        let progress = if self.spinning {
            0.5 + 0.5 * clamp01(b.velocity.dot(dir) / self.release_ball_speed)
        } else {
            0.5 * clamp01(1.0 - dist_ball / 0.5)
        };
        SkillStatus {
            progress,
            done: self.is_done(robot, world),
            feasible: b.velocity.length() < 0.5 && dist_ball < 0.6,
            face: Face::Front,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Mark
// ─────────────────────────────────────────────────────────────────────────────

/// Posicionamiento mirando a la pelota: va a `point` (lo calcula la táctica:
/// marca entre un rival y nuestro arco, punto de apoyo para la segunda pelota,
/// etc.) y queda orientado a la pelota con la cara más cercana, listo para
/// interceptar. Nunca termina por sí sola.
pub struct MarkSkill {
    pub point: Vec2,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

impl MarkSkill {
    pub fn new(point: Vec2) -> Self {
        Self {
            point,
            kp: CONTROL_KP,
            ki: CONTROL_KI,
            kd: CONTROL_KD,
        }
    }

    pub fn set_point(&mut self, p: Vec2) {
        self.point = p;
    }
}

impl Skill for MarkSkill {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        let ball = world.get_ball_state().position;
        let point = clamp_to_logical_field(self.point);
        motion.move_and_face(robot, point, ball, world, self.kp, self.ki, self.kd)
    }

    fn current_target(&self, _world: &World) -> Option<Vec2> {
        Some(clamp_to_logical_field(self.point))
    }

    fn status(&self, robot: &RobotState, world: &World) -> SkillStatus {
        let ball = world.get_ball_state().position;
        let dist = (clamp_to_logical_field(self.point) - robot.position).length();
        let to_ball = ball - robot.position;
        let raw = Motion::normalize_angle((to_ball.y as f64).atan2(to_ball.x as f64) - robot.orientation);
        let face = if raw.abs() <= std::f64::consts::FRAC_PI_2 {
            Face::Front
        } else {
            Face::Back
        };
        SkillStatus {
            progress: clamp01(1.0 - dist / 0.6),
            done: false,
            feasible: true,
            face,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn robot_at(x: f32, y: f32, deg: f32) -> RobotState {
        let mut r = RobotState::new(0, 0);
        r.position = Vec2::new(x, y);
        r.orientation = deg.to_radians() as f64;
        r
    }

    fn bidir_motion() -> Motion {
        let mut cfg = MotionConfig::default();
        cfg.bidirectional = true;
        Motion::with_config(cfg)
    }

    #[test]
    fn approach_staging_is_behind_ball_on_line() {
        let skill = ApproachAlignedSkill::new(Vec2::new(0.75, 0.0));
        let ball = Vec2::new(0.0, 0.0);
        let staging = skill.staging_point(ball).unwrap();
        assert!(staging.x < 0.0, "staging debe quedar detrás de la pelota");
        assert!(staging.y.abs() < 1e-6);
        assert!(((staging - ball).length() - skill.staging_offset).abs() < 1e-6);
    }

    #[test]
    fn approach_staging_retreats_into_field_when_ball_on_far_border() {
        let skill = ApproachAlignedSkill::new(Vec2::new(0.75, 0.0));
        // Pelota pegada al borde -x: el staging "detrás" caería fuera del campo.
        let staging = skill.staging_point(Vec2::new(-0.69, 0.0)).unwrap();
        assert!(is_inside_logical_field(staging));
    }

    #[test]
    fn approach_done_accepts_the_back_face_only_with_two_faces() {
        // N13: en frontal el empuje es con el frente; antes `is_done` plegaba siempre y
        // daba por terminado al robot de espaldas.
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.0, 0.0), Vec2::ZERO);
        let mut skill = ApproachAlignedSkill::new(Vec2::new(0.75, 0.0));
        let staging = skill.staging_point(Vec2::ZERO).unwrap();
        let back = robot_at(staging.x, staging.y, 180.0);
        let front = robot_at(staging.x, staging.y, 0.0);
        // El modo lo toma del motion con el que corre.
        skill.tick(&back, &world, &Motion::new());
        assert!(!skill.is_done(&back, &world), "frontal, de espaldas");
        assert!(skill.is_done(&front, &world), "frontal, de frente");
        skill.tick(&back, &world, &bidir_motion());
        assert!(skill.is_done(&back, &world), "bidireccional, de espaldas");
        assert_eq!(skill.status(&back, &world).face, Face::Back);
    }

    #[test]
    fn route_around_ball_detours_only_when_ball_is_in_the_way() {
        let ball = Vec2::ZERO;
        // Pelota justo en el camino: punto de rodeo al costado, a la holgura + margen.
        let wp = route_around_ball(Vec2::new(0.25, 0.0), Vec2::new(-0.14, 0.0), ball, 0.09);
        assert!(wp.y.abs() > 0.12, "rodeo lateral: {wp:?}");
        assert!(wp.x.abs() < 0.02, "a la altura de la pelota: {wp:?}");
        // Pelota fuera del segmento (detrás del destino): sin rodeo.
        let to = Vec2::new(-0.14, 0.0);
        assert_eq!(route_around_ball(Vec2::new(-0.5, 0.0), to, ball, 0.09), to);
        // Pelota lejos de la línea (a 0.11 m del segmento): sin rodeo.
        assert_eq!(route_around_ball(Vec2::new(0.25, 0.5), to, ball, 0.09), to);
    }

    #[test]
    fn approach_from_in_front_goes_around_the_ball_not_through_it() {
        let motion = bidir_motion();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::ZERO, Vec2::ZERO);
        // Robot delante de la pelota respecto del arco (+x): el staging queda detrás.
        // Con la planta diferencial: durante el primer segundo el robot no toca la pelota
        // (rodea por el costado en vez de atravesarla). Contacto: medio robot + pelota.
        use crate::motion::test_plant::{Case, run};
        use crate::skills::SkillId;
        for skill in [SkillId::ApproachAligned, SkillId::Clear] {
            let mut case = Case::new(skill, Vec2::new(0.75, 0.0), (0.25, 0.0, 180.0), Vec2::ZERO)
                .bidirectional();
            case.ticks = 60;
            let trace = run(&case);
            let min_d = trace
                .steps
                .iter()
                .map(|s| Vec2::new(s.x, s.y).length())
                .fold(f32::MAX, f32::min);
            assert!(min_d > 0.06, "{skill:?} pasó a {min_d:.3} m de la pelota");
            let last = trace.last();
            assert!(last.y.abs() > 0.03, "{skill:?} debe desviarse lateralmente: y={:.3}", last.y);
        }
        let _ = (&motion, &world);
    }

    #[test]
    fn spin_kick_spins_when_already_touching_and_detours_when_ball_blocks_center() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::ZERO, Vec2::ZERO);
        let mut skill = SpinKickSkill::new(Vec2::new(0.75, 0.0));
        // Tocando la pelota por el lado equivocado: gira igual (no la empuja más).
        let touching = robot_at(-0.06, 0.0, 0.0);
        let cmd = skill.tick(&touching, &world, &motion);
        assert!(skill.is_spinning());
        assert_eq!(cmd.vx, 0.0);
        // Robot delante de la pelota sobre la línea de tiro: los centros de contacto
        // (0, ±0.065) quedan "detrás" de la pelota → rodeo lateral, sin girar.
        let mut skill2 = SpinKickSkill::new(Vec2::new(0.75, 0.0));
        let in_front = robot_at(0.12, 0.0, 0.0);
        let cmd = skill2.tick(&in_front, &world, &motion);
        assert!(!skill2.is_spinning());
        // El waypoint de rodeo queda al costado: el diferencial primero gira hacia él
        // (a lo sumo max_angular_speed, no el giro de la patada).
        assert!(
            cmd.omega.abs() > 0.5 && cmd.omega.abs() <= motion.config.max_angular_speed + 1e-9,
            "gira hacia el rodeo lateral: {cmd:?}"
        );
    }

    #[test]
    fn approach_moves_when_far() {
        let motion = bidir_motion();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.2, 0.1), Vec2::ZERO);
        let robot = robot_at(-0.5, -0.3, 0.0);
        let mut skill = ApproachAlignedSkill::new(Vec2::new(0.75, 0.0));
        // El avance sube con la rampa de aceleración: a los 20 ticks ya es claro.
        let mut cmd = skill.tick(&robot, &world, &motion);
        for _ in 0..20 {
            cmd = skill.tick(&robot, &world, &motion);
        }
        assert!(cmd.vx.abs() + cmd.vy.abs() > 0.1);
        assert!(!skill.is_done(&robot, &world));
    }

    #[test]
    fn shoot_push_infeasible_when_in_front_of_ball() {
        let motion = bidir_motion();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.0, 0.0), Vec2::ZERO);
        let robot = robot_at(0.10, 0.0, 0.0); // delante de la pelota respecto de +x
        let mut skill = ShootPushSkill::new(Vec2::new(0.75, 0.0));
        assert!(!skill.is_feasible(&robot, Vec2::ZERO));
        let cmd = skill.tick(&robot, &world, &motion);
        assert_eq!(cmd.vx, 0.0);
        assert_eq!(cmd.vy, 0.0);
        assert!(!skill.status(&robot, &world).feasible);
    }

    /// Comando tras dejar subir la rampa de aceleración (motion nuevo por caso).
    fn settled_cmd(skill: &mut ShootPushSkill, robot: &RobotState, world: &World) -> MotionCommand {
        let motion = bidir_motion();
        let mut cmd = skill.tick(robot, world, &motion);
        for _ in 0..25 {
            cmd = skill.tick(robot, world, &motion);
        }
        cmd
    }

    #[test]
    fn shoot_push_pushes_through_ball_with_either_face() {
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.0, 0.0), Vec2::ZERO);
        let mut skill = ShootPushSkill::new(Vec2::new(0.75, 0.0));

        let front = robot_at(-0.10, 0.0, 0.0);
        let cmd_f = settled_cmd(&mut skill, &front, &world);
        assert!(cmd_f.vx > 0.3, "empuje frontal vx={}", cmd_f.vx);
        assert_eq!(skill.status(&front, &world).face, Face::Front);

        let back = robot_at(-0.10, 0.0, 180.0);
        let cmd_b = settled_cmd(&mut skill, &back, &world);
        assert!(cmd_b.vx > 0.3, "empuje de espaldas vx={}", cmd_b.vx);
        // Alineado con la espalda: el heading plegado no pide giro.
        assert!(cmd_b.omega.abs() < 1e-6, "omega={}", cmd_b.omega);
        assert_eq!(skill.status(&back, &world).face, Face::Back);
    }

    #[test]
    fn shoot_push_turns_first_when_body_is_crossed() {
        // Serie del 7-oct: factible por posición pero con el cuerpo cruzado respecto de la
        // recta de empuje. El comando debe ser ejecutable por el diferencial (a lo largo
        // del heading), girando hacia la recta y sin alejarse de la pelota.
        let motion = bidir_motion();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::ZERO, Vec2::ZERO);
        let mut skill = ShootPushSkill::new(Vec2::new(0.75, 0.0));
        let crossed = robot_at(-0.12, 0.02, -125.0);
        assert!(skill.is_feasible(&crossed, Vec2::ZERO));
        let cmd = skill.tick(&crossed, &world, &motion);
        let th = crossed.orientation;
        let across = cmd.vx * th.sin() - cmd.vy * th.cos();
        assert!(across.abs() < 1e-6, "comando fuera del heading: {cmd:?}");
        assert!(cmd.omega < -1.0, "debe girar hacia la recta: {cmd:?}");
        assert!(cmd.vx >= 0.0, "no se aleja de la pelota: {cmd:?}");
    }

    #[test]
    fn shoot_push_releases_when_ball_runs_away() {
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.30, 0.0), Vec2::new(1.0, 0.0));
        let robot = robot_at(0.0, 0.0, 0.0);
        let skill = ShootPushSkill::new(Vec2::new(0.75, 0.0));
        assert!(skill.is_done(&robot, &world));
    }

    #[test]
    fn shoot_push_is_infeasible_from_the_side_and_stops() {
        // B3: al costado de la pelota (proyección 0 sobre la línea de empuje) ya no es
        // "detrás": empujar desde ahí manda la pelota de lado.
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::ZERO, Vec2::ZERO);
        let target = Vec2::new(0.75, 0.0);
        let mut skill = ShootPushSkill::new(target);
        for y in [0.10f32, 0.25] {
            let side = robot_at(0.0, y, -90.0);
            assert!(!shoot_push_feasible_now(side.position, Vec2::ZERO, target), "y={y}");
            assert!(!skill.is_feasible(&side, Vec2::ZERO));
            let cmd = skill.tick(&side, &world, &motion);
            assert_eq!((cmd.vx, cmd.vy, cmd.omega), (0.0, 0.0, 0.0), "y={y}: {cmd:?}");
        }
        // Detrás y casi sobre la línea: factible.
        assert!(shoot_push_feasible_now(Vec2::new(-0.12, 0.02), Vec2::ZERO, target));
    }

    // ── Clear: dirección efectiva y staging legal ─────────────────────────────

    fn angle_deg(a: Vec2, b: Vec2) -> f32 {
        a.angle_between(b).to_degrees().abs()
    }

    fn staging_is_legal(ball: Vec2, d: Vec2, areas: &[AreaRect]) -> bool {
        let st = clear_staging(ball, d, params().skills.clear_staging_offset);
        areas.iter().all(|a| a.clearance(st) > AREA_CLEARANCE)
    }

    #[test]
    fn clear_direction_is_the_minimum_legal_turn_from_the_target() {
        // Pelota frente al arco: con la dirección pedida el staging cae dentro del arco. La
        // efectiva es la primera legal (y hacia el arco rival) girando de a 1° y alternando
        // el sentido: ningún giro menor, en ningún sentido, lo es.
        let (ball, target, own) = (Vec2::new(-0.45, 0.0), Vec2::new(0.2, 0.45), [AreaRect::own(1.0)]);
        let offset = params().skills.clear_staging_offset;
        let d0 = (target - ball).normalize();
        assert!(!staging_is_legal(ball, d0, &own), "escenario: el staging pedido es ilegal");
        let d = clear_direction(ball, target, offset, &own, Some(Vec2::X)).unwrap();
        assert!(staging_is_legal(ball, d, &own));
        let a = d.y.atan2(d.x).to_degrees();
        assert!((72.0..=82.0).contains(&a), "dirección efectiva {a:.1}°");
        let k = angle_deg(d0, d).round() as i32;
        for j in 0..k {
            for sgn in [1.0f32, -1.0] {
                let dj = Vec2::from_angle((sgn * j as f32).to_radians()).rotate(d0);
                assert!(
                    !(staging_is_legal(ball, dj, &own) && dj.x > CLEAR_MIN_FORWARD),
                    "un giro de {}° ya era legal",
                    sgn * j as f32
                );
            }
        }
    }

    #[test]
    fn the_effective_clear_sends_the_ball_out_of_our_side() {
        // Para pelotas frente al área propia y objetivos en el campo rival, la dirección
        // efectiva avanza hacia el arco rival (x > 0) y la recta de empuje no entra al área.
        // El staging es legal salvo que no exista ninguna dirección hacia adelante con staging
        // legal (pelota pegada al área: la despeja el arquero).
        let own = [AreaRect::own(1.0)];
        let offset = params().skills.clear_staging_offset;
        let any_forward_legal = |ball: Vec2| {
            (0..360).map(|a| Vec2::from_angle((a as f32).to_radians())).any(|d| d.x > CLEAR_MIN_FORWARD && staging_is_legal(ball, d, &own))
        };
        let mut legal_cases = 0;
        for bx in [-0.50f32, -0.45, -0.40, -0.30] {
            for by in [-0.30f32, -0.15, 0.0, 0.15, 0.30] {
                for target in [Vec2::new(0.2, 0.45), Vec2::new(0.3, -0.2), Vec2::new(0.5, 0.0)] {
                    let ball = Vec2::new(bx, by);
                    if own[0].touches(ball) {
                        continue;
                    }
                    let d = clear_direction(ball, target, offset, &own, Some(Vec2::X)).unwrap();
                    assert!(d.x > 0.0, "ball={ball:?} target={target:?} d={d:?}");
                    if !any_forward_legal(ball) {
                        continue;
                    }
                    legal_cases += 1;
                    assert!(staging_is_legal(ball, d, &own), "ball={ball:?} target={target:?}");
                    assert!(
                        (1..=50).all(|i| !own[0].touches(ball + d * (i as f32 * 0.02))),
                        "ball={ball:?} target={target:?}: la recta de empuje entra al área"
                    );
                }
            }
        }
        assert!(legal_cases >= 40, "la grilla debe cubrir casos con staging legal: {legal_cases}");
    }

    #[test]
    fn clear_direction_keeps_a_legal_original() {
        let (ball, target) = (Vec2::new(0.0, 0.1), Vec2::new(0.5, 0.0));
        let d = clear_direction(ball, target, 0.10, &[AreaRect::own(1.0)], Some(Vec2::X)).unwrap();
        assert!((d - (target - ball).normalize()).length() < 1e-6);
    }

    #[test]
    fn the_keeper_does_not_turn_its_clear() {
        // El área propia no le está prohibida al arquero (keeper_id 2): despeja hacia
        // donde se le pide aunque el staging quede dentro del área.
        let motion = Motion::new().with_areas(1.0, 2);
        let world = World::new(3, 3);
        let mut keeper = robot_at(-0.70, 0.12, 0.0);
        keeper.id = 2;
        let forbidden = motion.forbidden_areas(&keeper, &world);
        assert!(forbidden.is_empty());
        let (ball, target) = (Vec2::new(-0.62, 0.10), Vec2::new(0.0, 0.45));
        let d = clear_direction(ball, target, 0.10, &forbidden, motion.attack_dir()).unwrap();
        assert!((d - (target - ball).normalize()).length() < 1e-6);
        // Un jugador de campo en la misma situación sí gira (o no encuentra staging legal).
        let field = motion.forbidden_areas(&robot_at(-0.70, 0.12, 0.0), &world);
        assert_eq!(field.len(), 1);
    }

    #[test]
    fn clear_from_the_side_goes_to_staging_first() {
        let ball = Vec2::new(-0.4, -0.2);
        let skill = ClearSkill::new(Vec2::new(0.3, -0.2));
        // Al costado de la pelota, perpendicular a la recta de despeje.
        assert!(!skill.push_transition(false, &robot_at(-0.4, 0.0, 0.0), ball, false));
        assert!(skill.push_transition(false, &robot_at(-0.5, -0.21, 0.0), ball, false));
    }

    #[test]
    fn clear_enters_the_push_only_aligned() {
        // Detrás de la pelota, sobre la recta de despeje (+x): con 90° de error no empieza a
        // empujar (gira primero); alineado (o a 30°) sí. En bidireccional vale la espalda.
        let ball = Vec2::new(-0.4, -0.2);
        let skill = ClearSkill::new(Vec2::new(0.3, -0.2));
        let at = |deg: f32| robot_at(-0.5, -0.2, deg);
        assert!(!skill.push_transition(false, &at(90.0), ball, false), "90° no entra");
        assert!(!skill.push_transition(false, &at(-90.0), ball, true), "90° no entra (bidir)");
        assert!(skill.push_transition(false, &at(0.0), ball, false), "alineado entra");
        assert!(skill.push_transition(false, &at(30.0), ball, false), "30° entra");
        assert!(!skill.push_transition(false, &at(180.0), ball, false), "de espaldas no entra (frontal)");
        assert!(skill.push_transition(false, &at(180.0), ball, true), "de espaldas entra (bidir)");
    }

    #[test]
    fn a_small_lateral_drift_does_not_end_the_push() {
        // Empujando, una desviación lateral de 0.06 m (más que la tolerancia de entrada, 0.05)
        // no lo hace volver al staging; 0.09 m sí. Sin empujar, a 0.06 m no entra.
        let ball = Vec2::new(-0.4, -0.2);
        let skill = ClearSkill::new(Vec2::new(0.3, -0.2));
        let drift = |lat: f32| robot_at(-0.48, -0.2 + lat, 0.0);
        assert!(skill.push_transition(true, &drift(0.06), ball, false), "sigue empujando");
        assert!(skill.push_transition(true, &drift(-0.07), ball, false), "sigue empujando");
        assert!(!skill.push_transition(true, &drift(0.09), ball, false), "vuelve al staging");
        assert!(!skill.push_transition(false, &drift(0.06), ball, false), "para entrar exige 0.05");
        // Si pasa por delante de la pelota, deja de empujar.
        assert!(!skill.push_transition(true, &robot_at(-0.30, -0.2, 0.0), ball, false));
    }

    #[test]
    fn clear_reaches_the_ball_pushing_without_oscillating() {
        // N9: desde (−0.2, 0.3) el robot pasaba por posiciones "detrás" pero de costado y
        // alternaba entre ir al staging y empujar sin llegar a la pelota. La planta no
        // tiene física de pelota: se mira hasta el primer contacto, que tiene que ser ya en
        // la fase de empuje (al rodear la pelota hacia el staging, motion no la toca). Con
        // la pelota frente al arco, ver `clear_direction_is_the_minimum_legal_turn_from_the_target`.
        use crate::motion::test_plant::{Case, run};
        use crate::skills::SkillId;
        let ball = Vec2::new(-0.30, 0.0);
        let target = Vec2::new(0.2, 0.45);
        let skill = ClearSkill::new(target);
        for case in [
            Case::new(SkillId::Clear, target, (-0.2, 0.3, 0.0), ball),
            Case::new(SkillId::Clear, target, (-0.2, 0.3, 0.0), ball).bidirectional(),
        ] {
            let t = run(&case);
            let bidir = case.motion.bidirectional;
            // Fase de cada tick: la misma transición con histéresis que usa la skill, sobre
            // la pose (con heading) de la planta.
            let mut phase = false;
            let pushing: Vec<bool> = t
                .steps
                .iter()
                .map(|s| {
                    phase = skill.push_transition(phase, &robot_at(s.x, s.y, s.th.to_degrees() as f32), ball, bidir);
                    phase
                })
                .collect();
            let contact = t.steps.iter().position(|s| (Vec2::new(s.x, s.y) - ball).length() < 0.065);
            let k = contact.unwrap_or_else(|| panic!("bidir={bidir}: no llega a la pelota en 6 s"));
            let toggles = pushing[..=k].windows(2).filter(|w| w[0] != w[1]).count();
            assert!(toggles <= 2, "bidir={bidir}: {toggles} cambios de fase antes del contacto");
            assert!(pushing[k], "bidir={bidir}: llega a la pelota sin estar empujando");
        }
    }

    #[test]
    fn intercept_leads_moving_ball() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.0, 0.3), Vec2::new(0.0, -1.0)); // baja hacia y=0
        let robot = robot_at(0.0, -0.3, 90.0);
        let skill = InterceptSkill::new();
        let p = skill.intercept_point(&robot, &world, motion.config.max_linear_speed as f32);
        // El punto de intercepción está más adelante en la trayectoria (y < 0.3).
        assert!(p.y < 0.3, "p={p:?}");
        assert!(p.x.abs() < 1e-3);
    }

    #[test]
    fn intercept_chases_slow_ball() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.2, 0.2), Vec2::new(0.01, 0.0));
        let robot = robot_at(-0.4, 0.0, 0.0);
        let skill = InterceptSkill::new();
        let p = skill.intercept_point(&robot, &world, motion.config.max_linear_speed as f32);
        assert!((p - Vec2::new(0.2, 0.2)).length() < 1e-6);
    }

    #[test]
    fn block_line_sits_between_ball_and_goal() {
        let skill = BlockLineSkill::new(Vec2::new(-0.75, 0.0));
        let ball = Vec2::new(0.3, 0.4);
        let p = skill.block_point(ball).unwrap();
        let to_ball = (ball - skill.own_goal).normalize();
        let to_p = (p - skill.own_goal).normalize();
        assert!(to_ball.dot(to_p) > 0.999, "no está sobre la línea");
        assert!(((p - skill.own_goal).length() - skill.distance).abs() < 1e-3);
        assert!(!AreaRect::own(1.0).touches(p));
    }

    /// Distancia de `p` a la recta arco → pelota.
    fn dist_to_line(goal: Vec2, ball: Vec2, p: Vec2) -> f32 {
        let d = (ball - goal).normalize();
        (p - goal).perp_dot(d).abs()
    }

    #[test]
    fn block_point_beside_the_area_moves_out_along_the_line() {
        // B2: con la pelota abierta por el costado del área, el punto a `block_distance`
        // del arco cae en la zona prohibida; se corre hacia afuera por la misma línea.
        let goal = Vec2::new(-0.75, 0.0);
        let skill = BlockLineSkill::new(goal);
        for ball in [Vec2::new(-0.45, 0.55), Vec2::new(-0.65, -0.50)] {
            let p = skill.block_point(ball).unwrap();
            assert!(!AreaRect::own(1.0).touches(p), "ball={ball:?} p={p:?} toca el área");
            // Con margen: queda a más de `AREA_CLEARANCE` (medio robot + 0.03) del área.
            assert!(!AreaRect::own(1.0).touches_with(p, AREA_CLEARANCE), "ball={ball:?} p={p:?} sin margen");
            assert!(dist_to_line(goal, ball, p) < 1e-3, "ball={ball:?} p={p:?} fuera de la línea");
            assert!((p - goal).length() >= skill.distance);
        }
        // Pelota al centro: el punto no toca y queda a `block_distance`, como antes.
        let p = skill.block_point(Vec2::new(0.2, 0.0)).unwrap();
        assert!((p - Vec2::new(-0.45, 0.0)).length() < 1e-6, "p={p:?}");
    }

    #[test]
    fn block_line_reaches_its_point_beside_the_area() {
        use crate::motion::test_plant::{Case, run};
        use crate::skills::SkillId;
        let goal = Vec2::new(-0.75, 0.0);
        let ball = Vec2::new(-0.45, 0.55);
        let p = BlockLineSkill::new(goal).block_point(ball).unwrap();
        let tol = params().motion.arrival_threshold + 0.01;
        for case in [
            Case::new(SkillId::BlockLine, goal, (-0.3, 0.0, 0.0), ball),
            Case::new(SkillId::BlockLine, goal, (-0.3, 0.0, 0.0), ball).bidirectional(),
        ] {
            let t = run(&case);
            let end = Vec2::new(t.last().x, t.last().y);
            assert!((end - p).length() <= tol, "bidir={}: final {end:?}, punto {p:?}", case.motion.bidirectional);
            assert!(t.steps.iter().all(|s| !AreaRect::own(1.0).touches(Vec2::new(s.x, s.y))));
        }
    }

    #[test]
    fn clear_approaches_then_pushes_and_finishes_when_ball_leaves() {
        let motion = bidir_motion();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(-0.5, 0.0), Vec2::ZERO);
        let mut skill = ClearSkill::new(Vec2::new(0.2, 0.45)); // despeje adelante/banda
        // Delante de la pelota (lado del despeje): no empuja, va al staging detrás.
        let front = robot_at(-0.35, 0.1, 0.0);
        assert!(!skill.push_transition(false, &front, Vec2::new(-0.5, 0.0), true));
        // El avance sube con la rampa de aceleración: a los 20 ticks ya es claro.
        let mut cmd = skill.tick(&front, &world, &motion);
        for _ in 0..20 {
            cmd = skill.tick(&front, &world, &motion);
        }
        assert!(cmd.vx.abs() + cmd.vy.abs() > 0.1);
        // Detrás (lado del arco propio): empuja hacia el objetivo. Motion fresco: el robot
        // "salta" de pose y la rampa no debe arrastrar el avance de la otra fase.
        let behind = robot_at(-0.58, -0.04, 0.0);
        assert!(skill.push_transition(false, &behind, Vec2::new(-0.5, 0.0), true));
        let motion = bidir_motion();
        let mut cmd = skill.tick(&behind, &world, &motion);
        for _ in 0..20 {
            cmd = skill.tick(&behind, &world, &motion);
        }
        assert!(cmd.vx > 0.2, "empuje hacia +x: {cmd:?}");
        // Pelota lanzada hacia el objetivo → done.
        world.update_ball(Vec2::new(-0.3, 0.12), Vec2::new(0.6, 0.4));
        assert!(skill.is_done(&behind, &world));
    }

    #[test]
    fn spin_kick_contact_centers_throw_ball_along_direction() {
        let skill = SpinKickSkill::new(Vec2::new(0.75, 0.0));
        let ball = Vec2::new(0.0, 0.0);
        for dir in [Vec2::X, Vec2::Y, Vec2::new(-0.6, 0.8)] {
            let (ccw, cw) = skill.contact_centers(ball, dir);
            // v = ω · perp(pelota − centro) con perp(v) = (−v.y, v.x).
            let r = ball - ccw;
            let v_ccw = Vec2::new(-r.y, r.x);
            assert!(v_ccw.normalize().dot(dir) > 0.999, "CCW dir={dir:?} v={v_ccw:?}");
            let r = ball - cw;
            let v_cw = -Vec2::new(-r.y, r.x);
            assert!(v_cw.normalize().dot(dir) > 0.999, "CW dir={dir:?} v={v_cw:?}");
            assert!(((ccw - ball).length() - skill.contact_radius).abs() < 1e-6);
        }
    }

    #[test]
    fn spin_kick_stops_after_its_timeout() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::ZERO, Vec2::ZERO);
        let mut skill = SpinKickSkill::new(Vec2::new(0.75, 0.0));
        let (ccw, _) = skill.contact_centers(Vec2::ZERO, Vec2::X);
        let at = robot_at(ccw.x, ccw.y, 0.0);
        for _ in 0..skill.max_spin_ticks {
            assert!(skill.tick(&at, &world, &motion).omega > 0.0);
        }
        // La pelota sigue al lado (trabada): quieto y terminado hasta el próximo reinicio.
        let cmd = skill.tick(&at, &world, &motion);
        assert_eq!((cmd.vx, cmd.vy, cmd.omega), (0.0, 0.0, 0.0), "{cmd:?}");
        assert!(skill.is_done(&at, &world));
        skill.reset();
        assert!(skill.tick(&at, &world, &motion).omega > 0.0, "tras reset vuelve a girar");
    }

    #[test]
    fn spin_kick_final_approach_from_the_side_reaches_contact_and_spins() {
        // N7: de costado a 0.10 m del centro de contacto, la aproximación final holonómica
        // (`move_direct` con ω = 0) proyectaba 0 sobre el heading: ni avanzaba ni giraba.
        use crate::motion::test_plant::{Case, run};
        use crate::skills::SkillId;
        let spin = params().skills.spin_omega;
        for case in [
            Case::new(SkillId::SpinKick, Vec2::new(0.75, 0.0), (-0.10, 0.065, 90.0), Vec2::ZERO),
            Case::new(SkillId::SpinKick, Vec2::new(0.75, 0.0), (-0.10, 0.065, 90.0), Vec2::ZERO).bidirectional(),
        ] {
            let t = run(&case);
            let k = t.steps.iter().position(|s| (s.omega.abs() - spin).abs() < 1e-6);
            let bidir = case.motion.bidirectional;
            assert!(k.is_some_and(|k| k < 120), "bidir={bidir}: no empieza a girar en 2 s ({k:?})");
        }
    }

    #[test]
    fn spin_kick_picks_a_reachable_contact_center_against_the_wall() {
        // N8: con la pelota contra la pared, el centro más cercano al robot queda detrás de
        // ella; se elige el otro, dentro del campo lógico.
        let ball = Vec2::new(0.2, -0.6);
        let tgt = Vec2::ZERO;
        let s = SpinKickSkill::new(tgt);
        let dir = (tgt - ball).normalize();
        let (ccw, cw) = s.contact_centers(ball, dir);
        let robot = Vec2::new(-0.2, -0.3);
        assert!(!is_inside_logical_field(ccw) && is_inside_logical_field(cw));
        assert!((ccw - robot).length() < (cw - robot).length(), "el inalcanzable es el más cercano");
        let (chosen, sign) = s.choose_center(robot, ball, dir);
        assert_eq!((chosen, sign), (cw, -1.0));
    }

    #[test]
    fn spin_kick_moves_to_contact_then_spins_and_times_out() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.0, 0.0), Vec2::ZERO);
        let mut skill = SpinKickSkill::new(Vec2::new(0.75, 0.0));
        // Lejos: se mueve, no gira a tope.
        let far = robot_at(-0.4, 0.2, 0.0);
        // El avance sube con la rampa de aceleración: a los 20 ticks ya es claro.
        let mut cmd = skill.tick(&far, &world, &motion);
        for _ in 0..20 {
            cmd = skill.tick(&far, &world, &motion);
        }
        assert!(cmd.vx.abs() + cmd.vy.abs() > 0.1);
        assert!(cmd.omega.abs() < skill.omega);
        assert!(!skill.is_spinning());
        // En el centro de contacto CCW (lado −y de la pelota para lanzar a +x): gira CCW.
        let (ccw, _) = skill.contact_centers(Vec2::ZERO, Vec2::X);
        let at = robot_at(ccw.x, ccw.y, 0.0);
        let cmd = skill.tick(&at, &world, &motion);
        assert!(skill.is_spinning());
        assert_eq!(cmd.vx, 0.0);
        assert!((cmd.omega - skill.omega).abs() < 1e-9, "omega={}", cmd.omega);
        // Sigue girando el mismo sentido aunque el otro centro quede más cerca.
        let (_, cw) = skill.contact_centers(Vec2::ZERO, Vec2::X);
        let near_cw = robot_at(cw.x, cw.y, 0.0);
        let cmd = skill.tick(&near_cw, &world, &motion);
        assert!(cmd.omega > 0.0);
        // Sin soltar la pelota, termina por tiempo.
        for _ in 0..skill.max_spin_ticks {
            skill.tick(&at, &world, &motion);
        }
        assert!(skill.is_done(&at, &world));
        // Pelota lanzada hacia el objetivo también termina.
        let mut skill2 = SpinKickSkill::new(Vec2::new(0.75, 0.0));
        world.update_ball(Vec2::new(0.1, 0.0), Vec2::new(0.8, 0.0));
        assert!(skill2.is_done(&at, &world));
        let _ = skill2.tick(&at, &world, &motion);
    }

    #[test]
    fn mark_holds_point_and_faces_ball() {
        let motion = bidir_motion();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.3, 0.3), Vec2::ZERO);
        let mut skill = MarkSkill::new(Vec2::new(-0.3, 0.0));
        // Lejos del punto: se mueve hacia él.
        let far = robot_at(0.2, -0.3, 0.0);
        let cmd = skill.tick(&far, &world, &motion);
        assert!(cmd.vx < 0.0, "hacia −x: {cmd:?}");
        // En el punto, de espaldas a la pelota (180° ± 45°): no se mueve, y con dos
        // caras no necesita girar. Motion nuevo: el PID de heading guarda estado por
        // robot y el tick anterior (lejos) dejaría términos I/D distintos de cero.
        let motion = bidir_motion();
        let deg_to_ball = (0.3f32 / 0.6).atan().to_degrees(); // dirección punto→pelota
        let at = robot_at(-0.3, 0.0, 180.0 + deg_to_ball);
        let cmd = skill.tick(&at, &world, &motion);
        assert_eq!(cmd.vx, 0.0);
        assert!(cmd.omega.abs() < 1e-3, "alineado por la espalda: {}", cmd.omega);
        assert_eq!(skill.status(&at, &world).face, Face::Back);
        assert!(!skill.is_done(&at, &world));
    }

    #[test]
    fn all_tactical_skills_stay_finite_under_degenerate_states() {
        let motion = bidir_motion();
        let mut world = World::new(3, 3);
        let robot = robot_at(0.0, 0.0, 30.0);
        world.update_ball(robot.position, Vec2::ZERO); // pelota sobre el robot
        let mut a = ApproachAlignedSkill::new(robot.position); // objetivo sobre la pelota
        let mut s = ShootPushSkill::new(robot.position);
        let mut i = InterceptSkill::new();
        let mut b = BlockLineSkill::new(robot.position);
        let mut c = ClearSkill::new(robot.position);
        let mut k = SpinKickSkill::new(robot.position);
        let mut m = MarkSkill::new(robot.position);
        for _ in 0..50 {
            for cmd in [
                a.tick(&robot, &world, &motion),
                s.tick(&robot, &world, &motion),
                i.tick(&robot, &world, &motion),
                b.tick(&robot, &world, &motion),
                c.tick(&robot, &world, &motion),
                k.tick(&robot, &world, &motion),
                m.tick(&robot, &world, &motion),
            ] {
                assert!(cmd.vx.is_finite() && cmd.vy.is_finite() && cmd.omega.is_finite());
            }
        }
    }
}
