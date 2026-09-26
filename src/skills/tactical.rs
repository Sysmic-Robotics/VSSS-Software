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

/// Valores por defecto de `ShootPushSkill` que la táctica necesita para evaluar
/// factibilidad sin instanciar la skill.
pub const SHOOT_PUSH_BEHIND_TOL: f32 = 0.03;
pub const SHOOT_PUSH_LOSE_RADIUS: f32 = 0.30;

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
        Self {
            aim_point,
            staging_offset: 0.14,
            pos_tol: 0.05,
            angle_tol: 0.15,
            pre_align_radius: 0.25,
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
        // para llegar orientado.
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
        Self {
            target,
            push_overshoot: 0.25,
            lose_radius: SHOOT_PUSH_LOSE_RADIUS,
            behind_tol: SHOOT_PUSH_BEHIND_TOL,
            release_ball_speed: 0.6,
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
        Self {
            horizon: 1.5,
            min_ball_speed: 0.08,
            ball_decay_per_s: 0.3,
            reaction_delay: 0.10,
            reach_radius: 0.09,
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
        Self {
            own_goal,
            distance: 0.30,
            max_abs_x: 0.58,
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
    fn all_tactical_skills_stay_finite_under_degenerate_states() {
        let motion = bidir_motion();
        let mut world = World::new(3, 3);
        let robot = robot_at(0.0, 0.0, 30.0);
        world.update_ball(robot.position, Vec2::ZERO); // pelota sobre el robot
        let mut a = ApproachAlignedSkill::new(robot.position); // objetivo sobre la pelota
        let mut s = ShootPushSkill::new(robot.position);
        let mut i = InterceptSkill::new();
        let mut b = BlockLineSkill::new(robot.position);
        for _ in 0..50 {
            for cmd in [
                a.tick(&robot, &world, &motion),
                s.tick(&robot, &world, &motion),
                i.tick(&robot, &world, &motion),
                b.tick(&robot, &world, &motion),
            ] {
                assert!(cmd.vx.is_finite() && cmd.vy.is_finite() && cmd.omega.is_finite());
            }
        }
    }
}
