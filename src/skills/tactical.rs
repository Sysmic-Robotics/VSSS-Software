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
use crate::motion::{Motion, MotionCommand};
use crate::params::params;
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
/// línea de empuje ≤ `behind_tol`) y a menos de `lose_radius` de ella.
pub fn shoot_push_feasible(
    robot_pos: Vec2,
    ball: Vec2,
    target: Vec2,
    behind_tol: f32,
    lose_radius: f32,
) -> bool {
    let dir = (target - ball).normalize_or_zero();
    if dir.length_squared() < f32::EPSILON {
        return false;
    }
    let along = (robot_pos - ball).dot(dir);
    along <= behind_tol && (ball - robot_pos).length() <= lose_radius
}

/// Holgura para rodear la pelota: media diagonal del robot (0.057) + radio de la
/// pelota (0.021) + margen.
pub const BALL_ROUTE_CLEARANCE: f32 = 0.09;

/// Si el segmento `from`→`to` pasa por la pelota (a menos de `clearance` y con la
/// pelota entre medio), devuelve un punto de rodeo al costado de la pelota; si no,
/// devuelve `to`. Evita que una skill "atraviese" la pelota para llegar a un punto
/// que queda detrás de ella: en la cancha es un empujón involuntario y en FIRASim
/// la pelota aprisionada entre dos robots hace explotar la física.
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
    shoot_push_feasible(robot_pos, ball, target, p.shoot_behind_tol, p.shoot_lose_radius)
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
        // "delante" de ella), rodearla en vez de atravesarla.
        let waypoint = route_around_ball(robot.position, staging, ball, BALL_ROUTE_CLEARANCE);
        let face_target = if dist < self.pre_align_radius {
            ball
        } else {
            waypoint
        };
        motion.move_and_face(robot, waypoint, face_target, world, self.kp, self.ki, self.kd)
    }

    fn is_done(&self, robot: &RobotState, world: &World) -> bool {
        let ball = world.get_ball_state().position;
        let (Some(dir), Some(staging)) = (self.line_dir(ball), self.staging_point(ball)) else {
            return false;
        };
        // Sin acceso al motion acá: usamos el criterio plegado (válido para ambos
        // modos, ya que en modo frontal el PID igual converge a error ~0).
        let err = Motion::fold_bidirectional(Motion::normalize_angle(
            (dir.y as f64).atan2(dir.x as f64) - robot.orientation,
        ));
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

    /// Factible = detrás de la pelota (proyección sobre la línea ≤ tolerancia) y
    /// dentro del radio de trabajo.
    pub fn is_feasible(&self, robot: &RobotState, ball: Vec2) -> bool {
        shoot_push_feasible(
            robot.position,
            ball,
            self.target,
            self.behind_tol,
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

        // Empuje directo (sin UVF: la pelota NO es obstáculo) apuntando más allá de
        // la pelota para no frenar sobre ella; heading sobre la línea de empuje.
        let push_point = ball + dir * self.push_overshoot;
        let mut cmd = motion.move_direct(robot, push_point);
        let desired = (dir.y as f64).atan2(dir.x as f64);
        let face = motion.face_to_angle(robot, desired, self.kp, self.ki, self.kd);
        cmd.omega = face.omega;
        cmd
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
        motion.move_and_face(robot, target, target, world, self.kp, self.ki, self.kd)
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
/// espaldas y despejar sin girar.
pub struct BlockLineSkill {
    pub own_goal: Vec2,
    /// Distancia del punto de bloqueo al centro del arco propio (m).
    pub distance: f32,
    /// Límite para no meterse en el área propia (|x| máximo del bloqueo).
    pub max_abs_x: f32,
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
            max_abs_x: p.block_max_abs_x,
            kp: CONTROL_KP,
            ki: CONTROL_KI,
            kd: CONTROL_KD,
        }
    }

    pub fn set_own_goal(&mut self, g: Vec2) {
        self.own_goal = g;
    }

    pub fn block_point(&self, ball: Vec2) -> Option<Vec2> {
        let dir = (ball - self.own_goal).normalize_or_zero();
        if dir.length_squared() < f32::EPSILON {
            return None;
        }
        let mut p = clamp_to_logical_field(self.own_goal + dir * self.distance);
        p.x = p.x.clamp(-self.max_abs_x, self.max_abs_x);
        Some(p)
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
    pub lose_radius: f32,
    pub push_overshoot: f32,
    pub release_ball_speed: f32,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

impl ClearSkill {
    pub fn new(target: Vec2) -> Self {
        let p = &params().skills;
        Self {
            target,
            staging_offset: p.clear_staging_offset,
            behind_tol: p.clear_behind_tol,
            lose_radius: p.clear_lose_radius,
            push_overshoot: p.shoot_push_overshoot,
            release_ball_speed: p.clear_release_ball_speed,
            kp: CONTROL_KP,
            ki: CONTROL_KI,
            kd: CONTROL_KD,
        }
    }

    pub fn set_target(&mut self, p: Vec2) {
        self.target = p;
    }

    fn dir(&self, ball: Vec2) -> Option<Vec2> {
        let d = (self.target - ball).normalize_or_zero();
        (d.length_squared() > f32::EPSILON).then_some(d)
    }

    /// `true` cuando ya está detrás de la pelota y empuja (fase 2).
    pub fn is_pushing(&self, robot: &RobotState, ball: Vec2) -> bool {
        shoot_push_feasible(robot.position, ball, self.target, self.behind_tol, self.lose_radius)
    }

    fn staging(&self, ball: Vec2, dir: Vec2) -> Vec2 {
        let mut offset = self.staging_offset;
        let mut staging = ball - dir * offset;
        while !is_inside_logical_field(staging) && offset > 0.05 {
            offset -= 0.02;
            staging = ball - dir * offset;
        }
        clamp_to_logical_field(staging)
    }
}

impl Skill for ClearSkill {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        let ball = world.get_ball_state().position;
        let Some(dir) = self.dir(ball) else {
            return stop_cmd(robot);
        };
        if self.is_pushing(robot, ball) {
            let push_point = ball + dir * self.push_overshoot;
            let mut cmd = motion.move_direct(robot, push_point);
            let desired = (dir.y as f64).atan2(dir.x as f64);
            cmd.omega = motion
                .face_to_angle(robot, desired, self.kp, self.ki, self.kd)
                .omega;
            return cmd;
        }
        let staging = self.staging(ball, dir);
        let waypoint = route_around_ball(robot.position, staging, ball, BALL_ROUTE_CLEARANCE);
        motion.move_and_face(robot, waypoint, ball, world, self.kp, self.ki, self.kd)
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

    fn status(&self, robot: &RobotState, world: &World) -> SkillStatus {
        let b = world.get_ball_state();
        let Some(dir) = self.dir(b.position) else {
            return SkillStatus {
                feasible: false,
                ..Default::default()
            };
        };
        let progress = if self.is_pushing(robot, b.position) {
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

    pub fn reset(&mut self) {
        self.spin_sign = 0.0;
        self.spinning = false;
        self.spin_ticks = 0;
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

    fn choose_center(&self, robot_pos: Vec2, ball: Vec2, dir: Vec2) -> (Vec2, f32) {
        let (ccw, cw) = self.contact_centers(ball, dir);
        if self.spin_sign > 0.0 {
            return (ccw, 1.0);
        }
        if self.spin_sign < 0.0 {
            return (cw, -1.0);
        }
        if (ccw - robot_pos).length() <= (cw - robot_pos).length() {
            (ccw, 1.0)
        } else {
            (cw, -1.0)
        }
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
        let (center, sign) = self.choose_center(robot.position, ball, dir);
        let dist = (center - robot.position).length();
        // Ya tocando la pelota (aunque no exactamente en el centro): girar igual, en
        // vez de seguir empujándola para "llegar" al punto de contacto.
        let touching = (ball - robot.position).length() <= self.contact_radius + 0.01;
        if !self.spinning && dist > self.pos_tol && !touching {
            // Si la pelota está entre el robot y el punto de contacto, rodearla.
            let waypoint =
                route_around_ball(robot.position, center, ball, self.contact_radius + 0.02);
            if dist < 0.15 && waypoint == center {
                let mut cmd = motion.move_direct(robot, center);
                cmd.omega = 0.0;
                return cmd;
            }
            return motion.move_and_face(robot, waypoint, ball, world, self.kp, self.ki, self.kd);
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
    use crate::motion::MotionConfig;

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
    fn approach_done_accepts_back_face_alignment() {
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.0, 0.0), Vec2::ZERO);
        let skill = ApproachAlignedSkill::new(Vec2::new(0.75, 0.0));
        let staging = skill.staging_point(Vec2::ZERO).unwrap();
        // Robot en staging mirando a -x (de espaldas a la línea): con dos caras cuenta.
        let robot = robot_at(staging.x, staging.y, 180.0);
        assert!(skill.is_done(&robot, &world));
        assert_eq!(skill.status(&robot, &world).face, Face::Back);
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
        let robot = robot_at(0.25, 0.0, 180.0);
        let mut skill = ApproachAlignedSkill::new(Vec2::new(0.75, 0.0));
        let cmd = skill.tick(&robot, &world, &motion);
        assert!(cmd.vy.abs() > 0.15, "debe desviarse lateralmente: {cmd:?}");
        // Clear en la misma geometría también rodea.
        let mut clear = ClearSkill::new(Vec2::new(0.75, 0.0));
        let cmd = clear.tick(&robot, &world, &motion);
        assert!(cmd.vy.abs() > 0.15, "clear debe rodear: {cmd:?}");
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
        assert!(cmd.vy.abs() > 0.03, "rodea lateralmente: {cmd:?}");
    }

    #[test]
    fn approach_moves_when_far() {
        let motion = bidir_motion();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.2, 0.1), Vec2::ZERO);
        let robot = robot_at(-0.5, -0.3, 0.0);
        let mut skill = ApproachAlignedSkill::new(Vec2::new(0.75, 0.0));
        let cmd = skill.tick(&robot, &world, &motion);
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

    #[test]
    fn shoot_push_pushes_through_ball_with_either_face() {
        let motion = bidir_motion();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.0, 0.0), Vec2::ZERO);
        let mut skill = ShootPushSkill::new(Vec2::new(0.75, 0.0));

        let front = robot_at(-0.10, 0.0, 0.0);
        let cmd_f = skill.tick(&front, &world, &motion);
        assert!(cmd_f.vx > 0.3, "empuje frontal vx={}", cmd_f.vx);
        assert_eq!(skill.status(&front, &world).face, Face::Front);

        let back = robot_at(-0.10, 0.0, 180.0);
        let cmd_b = skill.tick(&back, &world, &motion);
        assert!(cmd_b.vx > 0.3, "empuje de espaldas vx={}", cmd_b.vx);
        // Alineado con la espalda: el PID plegado no pide giro.
        assert!(cmd_b.omega.abs() < 1e-6, "omega={}", cmd_b.omega);
        assert_eq!(skill.status(&back, &world).face, Face::Back);
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
        assert!(p.x.abs() <= skill.max_abs_x + 1e-6);
    }

    #[test]
    fn clear_approaches_then_pushes_and_finishes_when_ball_leaves() {
        let motion = bidir_motion();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(-0.5, 0.0), Vec2::ZERO);
        let mut skill = ClearSkill::new(Vec2::new(0.2, 0.45)); // despeje adelante/banda
        // Delante de la pelota (lado del despeje): no empuja, va al staging detrás.
        let front = robot_at(-0.35, 0.1, 0.0);
        assert!(!skill.is_pushing(&front, Vec2::new(-0.5, 0.0)));
        let cmd = skill.tick(&front, &world, &motion);
        assert!(cmd.vx.abs() + cmd.vy.abs() > 0.1);
        // Detrás (lado del arco propio): empuja hacia el objetivo.
        let behind = robot_at(-0.58, -0.04, 0.0);
        assert!(skill.is_pushing(&behind, Vec2::new(-0.5, 0.0)));
        let cmd = skill.tick(&behind, &world, &motion);
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
    fn spin_kick_moves_to_contact_then_spins_and_times_out() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.0, 0.0), Vec2::ZERO);
        let mut skill = SpinKickSkill::new(Vec2::new(0.75, 0.0));
        // Lejos: se mueve, no gira a tope.
        let far = robot_at(-0.4, 0.2, 0.0);
        let cmd = skill.tick(&far, &world, &motion);
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
