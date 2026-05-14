pub mod catalog;
pub mod config;

pub use catalog::{SkillCatalog, SkillId};
pub use config::SkillConfig;

use crate::motion::{Motion, MotionCommand};
use crate::world::{RobotState, World};
use glam::Vec2;

const LOGICAL_FIELD_HALF_X: f32 = 0.70;
const LOGICAL_FIELD_HALF_Y: f32 = 0.60;

const CONTROL_KP: f64 = 3.0;
const CONTROL_KI: f64 = 0.08;
const CONTROL_KD: f64 = 0.20;

pub trait Skill: Send + Sync {
    /// Genera el comando de movimiento para este tick.
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand;

    /// Indica si la skill terminó intrínsecamente (no por timeout externo).
    ///
    /// Por defecto retorna `false`: la skill nunca termina sola y debe ser
    /// preempt-eada por el coach o por timeout externo. Skills con criterios
    /// naturales de terminación (e.g. GoTo cuando llega al target, FacePoint
    /// cuando alinea heading) pueden override esta función.
    ///
    /// **Estado actual (Fase 2)**: el dispatcher de Fase 3 va a ignorar
    /// `is_done` porque vamos por **opción E** del horizonte (frame-skip
    /// K=6, decisión cada 100ms sin importar status del skill). El contrato
    /// queda definido para una eventual migración a **opción C** (event-based
    /// con timeout) en una segunda generación del modelo RL, sin breaking
    /// changes en el trait.
    fn is_done(&self, _robot: &RobotState, _world: &World) -> bool {
        false
    }

    /// Punto del campo (en metros) que la skill está usando como destino
    /// visible. Solo para overlay de debug en la GUI: el cyan target dot.
    /// Las skills puramente rotacionales (Spin) o de detención (Stop) deben
    /// devolver `None`.
    fn current_target(&self, _world: &World) -> Option<Vec2> {
        None
    }
}

fn stop_cmd(robot: &RobotState) -> MotionCommand {
    MotionCommand {
        id: robot.id,
        team: robot.team,
        vx: 0.0,
        vy: 0.0,
        omega: 0.0,
        orientation: robot.orientation,
    }
}

fn clamp_to_logical_field(pos: Vec2) -> Vec2 {
    Vec2::new(
        pos.x.clamp(-LOGICAL_FIELD_HALF_X, LOGICAL_FIELD_HALF_X),
        pos.y.clamp(-LOGICAL_FIELD_HALF_Y, LOGICAL_FIELD_HALF_Y),
    )
}

fn is_inside_logical_field(pos: Vec2) -> bool {
    pos.x.abs() <= LOGICAL_FIELD_HALF_X && pos.y.abs() <= LOGICAL_FIELD_HALF_Y
}

// ─────────────────────────────────────────────────────────────────────────────
//  StuckDetector — recuperación local de skills
// ─────────────────────────────────────────────────────────────────────────────
/// Detecta cuando un robot que intenta llegar a un target deja de avanzar y
/// propone un punto de escape lateral para destrabarlo.
///
/// Migrado desde `tactics/` en Fase 1 del plan RL: aquí queda como utilidad
/// reutilizable por cualquier skill que quiera recovery local sin depender
/// de que el coach (clásico o RL) cambie de decisión.
///
/// # Uso típico
/// ```ignore
/// let escape = self.stuck.update(robot.position, target);
/// let effective_target = escape.unwrap_or(target);
/// ```
///
/// # Política
/// - Considera al robot "atascado" si lleva 30 ticks moviéndose < 6 mm/tick
///   mientras está a más de 8 cm del target.
/// - Al activarse, fuerza 35 ticks de movimiento perpendicular al target,
///   con dirección lateral elegida según el cuadrante actual del robot.
/// - El punto de escape se clampea al campo lógico (±0.68, ±0.58) para no
///   sugerir destinos fuera de cancha.
pub struct StuckDetector {
    stuck_ticks: u32,
    yield_ticks: u32,
    last_pos: Vec2,
    yield_dir: f32,
}

impl StuckDetector {
    pub fn new() -> Self {
        Self {
            stuck_ticks: 0,
            yield_ticks: 0,
            last_pos: Vec2::ZERO,
            yield_dir: 1.0,
        }
    }

    /// Devuelve `Some(escape_pos)` si el robot está atascado y debe yield-ear,
    /// `None` si puede seguir hacia el target normalmente.
    pub fn update(&mut self, robot_pos: Vec2, target: Vec2) -> Option<Vec2> {
        let dist_to_target = (target - robot_pos).length();
        let moved = (robot_pos - self.last_pos).length();
        self.last_pos = robot_pos;

        if dist_to_target > 0.08 {
            if moved < 0.006 {
                self.stuck_ticks += 1;
            } else {
                self.stuck_ticks = 0;
            }
        } else {
            self.stuck_ticks = 0;
        }

        if self.stuck_ticks >= 30 && self.yield_ticks == 0 {
            self.yield_dir = if (robot_pos.x + robot_pos.y) >= 0.0 {
                1.0
            } else {
                -1.0
            };
            self.yield_ticks = 35;
            self.stuck_ticks = 0;
        }

        if self.yield_ticks > 0 {
            self.yield_ticks -= 1;
            let to_target = (target - robot_pos).normalize_or_zero();
            let perp = Vec2::new(-to_target.y, to_target.x) * self.yield_dir;
            let escape = robot_pos + perp * 0.18;
            Some(Vec2::new(
                escape.x.clamp(-0.68, 0.68),
                escape.y.clamp(-0.58, 0.58),
            ))
        } else {
            None
        }
    }
}

impl Default for StuckDetector {
    fn default() -> Self {
        Self::new()
    }
}

pub struct StopSkill;

impl StopSkill {
    pub fn new() -> Self {
        Self
    }
}

impl Default for StopSkill {
    fn default() -> Self {
        Self::new()
    }
}

impl Skill for StopSkill {
    fn tick(&mut self, robot: &RobotState, _world: &World, _motion: &Motion) -> MotionCommand {
        stop_cmd(robot)
    }
}

pub struct GoToSkill {
    pub target: Vec2,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

impl GoToSkill {
    pub fn new(target: Vec2) -> Self {
        Self {
            target,
            kp: CONTROL_KP,
            ki: CONTROL_KI,
            kd: CONTROL_KD,
        }
    }

    pub fn set_target(&mut self, target: Vec2) {
        self.target = target;
    }
}

impl Skill for GoToSkill {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        motion.move_and_face(
            robot,
            self.target,
            self.target,
            world,
            self.kp,
            self.ki,
            self.kd,
        )
    }

    fn current_target(&self, _world: &World) -> Option<Vec2> {
        Some(self.target)
    }
}

pub struct FacePointSkill {
    pub target: Vec2,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

impl FacePointSkill {
    pub fn new(target: Vec2) -> Self {
        Self {
            target,
            kp: CONTROL_KP,
            ki: CONTROL_KI,
            kd: CONTROL_KD,
        }
    }

    pub fn set_target(&mut self, target: Vec2) {
        self.target = target;
    }
}

impl Skill for FacePointSkill {
    fn tick(&mut self, robot: &RobotState, _world: &World, motion: &Motion) -> MotionCommand {
        if (self.target - robot.position).length_squared() < f32::EPSILON {
            return stop_cmd(robot);
        }

        motion.face_to(robot, self.target, self.kp, self.ki, self.kd)
    }

    fn current_target(&self, _world: &World) -> Option<Vec2> {
        Some(self.target)
    }
}

pub struct HoldPositionSkill {
    pub position: Vec2,
    pub face: Option<Vec2>,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

impl HoldPositionSkill {
    pub fn new(position: Vec2) -> Self {
        Self {
            position,
            face: None,
            kp: CONTROL_KP,
            ki: CONTROL_KI,
            kd: CONTROL_KD,
        }
    }

    pub fn facing(mut self, face: Vec2) -> Self {
        self.face = Some(face);
        self
    }
}

impl Skill for HoldPositionSkill {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        let clamped_position = clamp_to_logical_field(self.position);
        let face_target = self.face.unwrap_or(clamped_position);
        motion.move_and_face(
            robot,
            clamped_position,
            face_target,
            world,
            self.kp,
            self.ki,
            self.kd,
        )
    }

    fn current_target(&self, _world: &World) -> Option<Vec2> {
        Some(clamp_to_logical_field(self.position))
    }
}

pub struct ChaseBallSkill {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

impl ChaseBallSkill {
    pub fn new() -> Self {
        Self {
            kp: CONTROL_KP,
            ki: CONTROL_KI,
            kd: CONTROL_KD,
        }
    }
}

impl Default for ChaseBallSkill {
    fn default() -> Self {
        Self::new()
    }
}

impl Skill for ChaseBallSkill {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        let ball = world.get_ball_state().position;
        motion.move_and_face(robot, ball, ball, world, self.kp, self.ki, self.kd)
    }

    fn current_target(&self, world: &World) -> Option<Vec2> {
        Some(world.get_ball_state().position)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  SpinSkill — rotación pura en el lugar
// ─────────────────────────────────────────────────────────────────────────────
/// Hace girar al robot en su sitio sin trasladarse.
///
/// **Por qué existe como skill atómica** (ItAndroids LARC 2019, Bassani 2020):
/// el robot diff-drive sin dribbler no puede cambiar de dirección de empuje
/// sin antes alinear su cuerpo. Cuando la policy detecta que el robot está
/// mal orientado respecto a la pelota o al target, conmutar a `Spin` por
/// algunos ticks es más rápido y más confiable que esperar a que `GoTo` o
/// `FacePoint` resuelvan el alineamiento mientras también intentan moverse.
///
/// Comando: `vx = vy = 0`, `omega = direction * spin_omega`.
///
/// **Convención de signo (CONTRATO con la policy RL)**:
/// - `direction = +1.0` → rotación CCW (omega positivo).
/// - `direction = -1.0` → rotación CW (omega negativo).
/// - `direction = 0.0` → no spin (no command movement).
///
/// La policy emite el sentido a través del signo de `target.x` cuando elige
/// `SkillId::Spin`: `target.x > 0` → CCW, `target.x < 0` → CW. El catálogo
/// hace la traducción.
pub struct SpinSkill {
    /// Sentido y magnitud relativa del giro: típicamente -1.0, 0.0, o +1.0.
    /// Valores intermedios escalan la velocidad angular linealmente.
    pub direction: f32,
    /// Velocidad angular máxima (rad/s). Default: `SkillConfig::default().spin_omega`.
    pub omega_max: f64,
}

impl SpinSkill {
    pub fn new() -> Self {
        Self {
            direction: 1.0,
            omega_max: SkillConfig::default().spin_omega,
        }
    }

    /// Construye con un `SkillConfig` específico.
    pub fn with_config(cfg: &SkillConfig) -> Self {
        Self {
            direction: 1.0,
            omega_max: cfg.spin_omega,
        }
    }

    /// Setea el sentido del spin a partir del signo de `target.x`.
    /// Esta es la traducción canónica policy → SpinSkill que usa el catálogo.
    pub fn set_direction_from(&mut self, target: Vec2) {
        self.direction = if target.x > 0.0 {
            1.0
        } else if target.x < 0.0 {
            -1.0
        } else {
            0.0
        };
    }
}

impl Default for SpinSkill {
    fn default() -> Self {
        Self::new()
    }
}

impl Skill for SpinSkill {
    fn tick(&mut self, robot: &RobotState, _world: &World, _motion: &Motion) -> MotionCommand {
        MotionCommand {
            id: robot.id,
            team: robot.team,
            vx: 0.0,
            vy: 0.0,
            omega: self.omega_max * self.direction as f64,
            orientation: robot.orientation,
        }
    }
}

pub struct DefendGoalLineSkill {
    pub defend_x: f32,
    pub goal_half_y: f32,
    pub band_half_width: f32,
    pub y_deadband: f32,
    pub prediction_horizon_max: f32,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

impl DefendGoalLineSkill {
    pub fn new(own_goal: Vec2) -> Self {
        let sign = if own_goal.x < 0.0 { 1.0_f32 } else { -1.0_f32 };
        Self {
            defend_x: own_goal.x + sign * 0.12,
            goal_half_y: 0.20,
            band_half_width: 0.03,
            y_deadband: 0.03,
            prediction_horizon_max: 0.40,
            kp: CONTROL_KP,
            ki: CONTROL_KI,
            kd: CONTROL_KD,
        }
    }

    fn band_limits(&self) -> (f32, f32) {
        (
            self.defend_x - self.band_half_width,
            self.defend_x + self.band_half_width,
        )
    }

    fn moving_toward_goal(&self, ball: Vec2, ball_velocity: Vec2) -> Option<f32> {
        let vx = ball_velocity.x;
        if vx.abs() < 0.05 {
            return None;
        }

        let time_to_line = (self.defend_x - ball.x) / vx;
        if time_to_line <= 0.0 || time_to_line > self.prediction_horizon_max {
            return None;
        }

        Some(time_to_line)
    }

    fn block_y(&self, ball: Vec2, ball_velocity: Vec2) -> f32 {
        let predicted_y = self
            .moving_toward_goal(ball, ball_velocity)
            .map(|time| ball.y + ball_velocity.y * time)
            .unwrap_or(ball.y);
        predicted_y.clamp(-self.goal_half_y, self.goal_half_y)
    }

    pub fn defend_pos(&self, ball: Vec2) -> Vec2 {
        Vec2::new(
            self.defend_x,
            ball.y.clamp(-self.goal_half_y, self.goal_half_y),
        )
    }

    fn defend_target(&self, robot: &RobotState, ball: Vec2, ball_velocity: Vec2) -> Vec2 {
        let (band_min_x, band_max_x) = self.band_limits();
        let target_x = robot.position.x.clamp(band_min_x, band_max_x);
        let block_y = self.block_y(ball, ball_velocity);
        let target_y = if (block_y - robot.position.y).abs() < self.y_deadband {
            robot.position.y
        } else {
            block_y
        };

        Vec2::new(target_x, target_y)
    }

    fn should_face_ball(&self, robot: &RobotState, move_target: Vec2) -> bool {
        let (band_min_x, band_max_x) = self.band_limits();
        let inside_band = robot.position.x >= band_min_x && robot.position.x <= band_max_x;
        let settled_y = (move_target.y - robot.position.y).abs() < self.y_deadband;
        inside_band && settled_y
    }
}

impl Skill for DefendGoalLineSkill {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        let ball = world.get_ball_state().position;
        let ball_velocity = world.get_ball_state().velocity;
        let move_target = self.defend_target(robot, ball, ball_velocity);
        let face_target = if self.should_face_ball(robot, move_target) {
            ball
        } else {
            move_target
        };

        motion.move_and_face(
            robot,
            move_target,
            face_target,
            world,
            self.kp,
            self.ki,
            self.kd,
        )
    }
}

pub struct SupportPositionSkill {
    pub own_goal: Vec2,
    pub behind_ball_offset: f32,
    pub lateral_offset: f32,
    pub field_half_x: f32,
    pub field_half_y: f32,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

impl SupportPositionSkill {
    pub fn new(own_goal: Vec2) -> Self {
        Self {
            own_goal,
            behind_ball_offset: 0.25,
            lateral_offset: 0.20,
            field_half_x: 0.60,
            field_half_y: 0.55,
            kp: CONTROL_KP,
            ki: CONTROL_KI,
            kd: CONTROL_KD,
        }
    }

    pub fn support_pos(&self, ball: Vec2) -> Vec2 {
        let to_own = (self.own_goal - ball).normalize_or_zero();
        let perp = Vec2::new(-to_own.y, to_own.x) * self.lateral_offset;
        let raw = ball + to_own * self.behind_ball_offset + perp;
        Vec2::new(
            raw.x.clamp(-self.field_half_x, self.field_half_x),
            raw.y.clamp(-self.field_half_y, self.field_half_y),
        )
    }
}

impl Skill for SupportPositionSkill {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        let ball = world.get_ball_state().position;
        let target = self.support_pos(ball);
        motion.move_and_face(robot, target, ball, world, self.kp, self.ki, self.kd)
    }
}

pub struct ApproachBallBehindSkill {
    pub aim_point: Vec2,
    pub staging_offset: f32,
    pub staging_tol: f32,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

impl ApproachBallBehindSkill {
    pub fn new(aim_point: Vec2) -> Self {
        Self {
            aim_point,
            staging_offset: 0.16,
            staging_tol: 0.08,
            kp: 1.2,
            ki: 0.0,
            kd: 0.10,
        }
    }

    pub fn set_aim_point(&mut self, point: Vec2) {
        self.aim_point = point;
    }
}

impl Skill for ApproachBallBehindSkill {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        let ball = world.get_ball_state().position;
        let ball_to_aim = (self.aim_point - ball).normalize_or_zero();
        let staging = ball - ball_to_aim * self.staging_offset;

        let dist_staging = (staging - robot.position).length();
        let pre_align_radius = self.staging_tol * 1.5;
        let face_target = if dist_staging < pre_align_radius {
            ball
        } else {
            staging
        };

        if dist_staging <= self.staging_tol {
            return motion.face_to(robot, ball, self.kp, self.ki, self.kd);
        }

        motion.move_and_face(
            robot,
            staging,
            face_target,
            world,
            self.kp,
            self.ki,
            self.kd,
        )
    }
}

pub struct PushBallSkill {
    pub push_target: Vec2,
    pub push_overshoot: f32,
    pub lose_radius: f32,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

impl PushBallSkill {
    pub fn new(push_target: Vec2) -> Self {
        Self {
            push_target,
            push_overshoot: 0.12,
            lose_radius: 0.25,
            kp: 1.2,
            ki: 0.0,
            kd: 0.10,
        }
    }

    pub fn set_push_target(&mut self, point: Vec2) {
        self.push_target = point;
    }
}

impl Skill for PushBallSkill {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        let ball = world.get_ball_state().position;
        let push_dir = (self.push_target - ball).normalize_or_zero();
        let push_point = ball + push_dir * self.push_overshoot;
        let dot = (robot.position - ball).dot(self.push_target - ball);
        let dist_ball = (ball - robot.position).length();

        if dot > 0.05 || dist_ball > self.lose_radius {
            return stop_cmd(robot);
        }

        let mut cmd = motion.move_direct(robot, push_point);
        let face = motion.face_to(robot, ball, self.kp, self.ki, self.kd);
        cmd.omega = face.omega;
        cmd
    }
}

pub struct AlignBallToTargetSkill {
    pub target_point: Vec2,
    pub stand_off: f32,
    pub pos_tol: f32,
    pub angle_tol: f64,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

impl AlignBallToTargetSkill {
    pub fn new(target_point: Vec2) -> Self {
        Self {
            target_point,
            stand_off: 0.12,
            pos_tol: 0.05,
            angle_tol: 0.10,
            kp: 1.2,
            ki: 0.0,
            kd: 0.10,
        }
    }

    pub fn set_target_point(&mut self, point: Vec2) {
        self.target_point = point;
    }

    pub fn align_pos(&self, ball: Vec2) -> Vec2 {
        let dir = (self.target_point - ball).normalize_or_zero();
        ball - dir * self.stand_off
    }
}

impl Skill for AlignBallToTargetSkill {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        let ball = world.get_ball_state().position;
        let line_dir = (self.target_point - ball).normalize_or_zero();
        if line_dir.length_squared() < f32::EPSILON {
            return stop_cmd(robot);
        }

        let align_pos = ball - line_dir * self.stand_off;
        if !is_inside_logical_field(align_pos) {
            return stop_cmd(robot);
        }

        let pos_err = (align_pos - robot.position).length();
        let desired_theta = line_dir.y.atan2(line_dir.x) as f64;
        let ang_err = Motion::normalize_angle(desired_theta - robot.orientation).abs();
        if pos_err < self.pos_tol && ang_err < self.angle_tol {
            return stop_cmd(robot);
        }

        motion.move_and_face(
            robot,
            align_pos,
            self.target_point,
            world,
            self.kp,
            self.ki,
            self.kd,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_robot(id: i32, x: f32, y: f32, orient_deg: f32) -> RobotState {
        let mut robot = RobotState::new(id, 0);
        robot.position = Vec2::new(x, y);
        robot.orientation = orient_deg.to_radians() as f64;
        robot
    }

    #[test]
    fn stop_skill_returns_zero_command() {
        let motion = Motion::new();
        let world = World::new(3, 3);
        let robot = make_robot(0, 0.0, 0.0, 45.0);
        let mut skill = StopSkill::new();

        let cmd = skill.tick(&robot, &world, &motion);

        assert_eq!(cmd.vx, 0.0);
        assert_eq!(cmd.vy, 0.0);
        assert_eq!(cmd.omega, 0.0);
        assert!((cmd.orientation - 45.0_f64.to_radians()).abs() < 1e-6);
    }

    #[test]
    fn goto_skill_moves_toward_target() {
        let motion = Motion::new();
        let world = World::new(3, 3);
        let robot = make_robot(0, 0.0, 0.0, 0.0);
        let mut skill = GoToSkill::new(Vec2::new(0.4, 0.0));

        let cmd = skill.tick(&robot, &world, &motion);

        assert!(cmd.vx > 0.0 || cmd.omega.abs() > 0.0);
    }

    #[test]
    fn face_point_skill_rotates_without_translation() {
        let motion = Motion::new();
        let world = World::new(3, 3);
        let robot = make_robot(0, 0.0, 0.0, 90.0);
        let mut skill = FacePointSkill::new(Vec2::new(0.5, 0.0));

        let cmd = skill.tick(&robot, &world, &motion);

        assert_eq!(cmd.vx, 0.0);
        assert_eq!(cmd.vy, 0.0);
        assert!(cmd.omega.abs() > 0.0);
    }

    #[test]
    fn hold_position_skill_clamps_outside_targets() {
        let motion = Motion::new();
        let world = World::new(3, 3);
        let robot = make_robot(0, 0.0, 0.0, 0.0);
        let mut skill = HoldPositionSkill::new(Vec2::new(1.0, 0.9));

        let cmd = skill.tick(&robot, &world, &motion);

        assert!(cmd.vx.abs() + cmd.vy.abs() + cmd.omega.abs() > 0.0);
    }

    #[test]
    fn chase_ball_skill_reacts_to_ball_motion() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        let robot = make_robot(0, 0.0, 0.0, 0.0);
        let mut skill = ChaseBallSkill::new();

        world.update_ball(Vec2::new(0.4, 0.0), Vec2::ZERO);
        let cmd_right = skill.tick(&robot, &world, &motion);

        world.update_ball(Vec2::new(-0.4, 0.0), Vec2::ZERO);
        let cmd_left = skill.tick(&robot, &world, &motion);

        let diff = (cmd_right.vx - cmd_left.vx).abs() + (cmd_right.omega - cmd_left.omega).abs();
        assert!(diff > 0.1);
    }

    #[test]
    fn defend_goal_line_skill_predicts_ball_trajectory_toward_goal() {
        let skill = DefendGoalLineSkill::new(Vec2::new(-0.75, 0.0));
        let ball = Vec2::new(-0.30, 0.02);
        let ball_velocity = Vec2::new(-1.0, 0.40);

        let time_to_line = (skill.defend_x - ball.x) / ball_velocity.x;
        let expected_y =
            (ball.y + ball_velocity.y * time_to_line).clamp(-skill.goal_half_y, skill.goal_half_y);

        let block_y = skill.block_y(ball, ball_velocity);

        assert!((block_y - expected_y).abs() < 1e-6);
    }

    #[test]
    fn defend_goal_line_skill_keeps_target_inside_defensive_band() {
        let skill = DefendGoalLineSkill::new(Vec2::new(-0.75, 0.0));
        let robot = make_robot(2, -0.80, 0.0, 0.0);
        let ball = Vec2::new(0.0, 0.12);

        let target = skill.defend_target(&robot, ball, Vec2::ZERO);
        let (band_min_x, band_max_x) = skill.band_limits();

        assert!((target.x - band_min_x).abs() < 1e-6);
        assert!(target.x >= band_min_x);
        assert!(target.x <= band_max_x);
        assert!((target.y - ball.y).abs() < 1e-6);
    }

    #[test]
    fn defend_goal_line_skill_prioritizes_stable_posture_over_exact_ball_tracking() {
        let skill = DefendGoalLineSkill::new(Vec2::new(-0.75, 0.0));
        let robot = make_robot(2, skill.defend_x, 0.10, 0.0);
        let ball = Vec2::new(0.0, 0.115);

        let target = skill.defend_target(&robot, ball, Vec2::ZERO);

        assert!((target.y - robot.position.y).abs() < 1e-6);
    }

    #[test]
    fn defend_goal_line_skill_faces_ball_only_when_already_stable() {
        let skill = DefendGoalLineSkill::new(Vec2::new(-0.75, 0.0));

        let stable_robot = make_robot(2, skill.defend_x, 0.10, 0.0);
        let stable_target = skill.defend_target(&stable_robot, Vec2::new(0.0, 0.115), Vec2::ZERO);
        assert!(skill.should_face_ball(&stable_robot, stable_target));

        let recovering_robot = make_robot(2, skill.defend_x + 0.08, 0.0, 0.0);
        let recovering_target =
            skill.defend_target(&recovering_robot, Vec2::new(0.0, 0.15), Vec2::ZERO);
        assert!(!skill.should_face_ball(&recovering_robot, recovering_target));
    }

    #[test]
    fn defend_goal_line_skill_generates_activity_when_recovering_shape() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.0, 0.5), Vec2::new(-0.6, -0.2));
        let robot = make_robot(2, -0.40, 0.0, 0.0);
        let mut skill = DefendGoalLineSkill::new(Vec2::new(-0.75, 0.0));

        let cmd = skill.tick(&robot, &world, &motion);

        assert!(cmd.vx.abs() + cmd.vy.abs() + cmd.omega.abs() > 0.0);
    }

    #[test]
    fn support_position_skill_stays_behind_ball() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.70, 0.60), Vec2::ZERO);
        let robot = make_robot(1, -0.4, 0.1, 0.0);
        let mut skill = SupportPositionSkill::new(Vec2::new(-0.75, 0.0));

        let target = skill.support_pos(world.get_ball_state().position);
        let cmd = skill.tick(&robot, &world, &motion);

        assert!(target.x < world.get_ball_state().position.x);
        assert!(target.x.abs() <= skill.field_half_x + 1e-6);
        assert!(target.y.abs() <= skill.field_half_y + 1e-6);
        assert!(cmd.vx.abs() + cmd.vy.abs() + cmd.omega.abs() > 0.0);
    }

    #[test]
    fn approach_ball_behind_skill_faces_ball_at_staging() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.0, 0.0), Vec2::ZERO);
        let robot = make_robot(0, -0.16, 0.0, 90.0);
        let mut skill = ApproachBallBehindSkill::new(Vec2::new(0.75, 0.0));

        let cmd = skill.tick(&robot, &world, &motion);

        assert_eq!(cmd.vx, 0.0);
        assert_eq!(cmd.vy, 0.0);
        assert!(cmd.omega.abs() > 0.0);
    }

    #[test]
    fn push_ball_skill_stops_when_ball_is_out_of_reach() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.0, 0.0), Vec2::ZERO);
        let robot = make_robot(0, -0.40, 0.0, 0.0);
        let mut skill = PushBallSkill::new(Vec2::new(0.75, 0.0));

        let cmd = skill.tick(&robot, &world, &motion);

        assert_eq!(cmd.vx, 0.0);
        assert_eq!(cmd.vy, 0.0);
        assert_eq!(cmd.omega, 0.0);
    }

    #[test]
    fn align_ball_to_target_skill_stops_when_invalid() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(-0.69, 0.0), Vec2::ZERO);
        let robot = make_robot(0, -0.40, 0.0, 0.0);
        let mut skill = AlignBallToTargetSkill::new(Vec2::new(0.75, 0.0));

        let cmd = skill.tick(&robot, &world, &motion);

        assert_eq!(cmd.vx, 0.0);
        assert_eq!(cmd.vy, 0.0);
        assert_eq!(cmd.omega, 0.0);
    }

    // ─────────────────────────────────────────────────────────────────────
    //  StuckDetector
    // ─────────────────────────────────────────────────────────────────────

    #[test]
    fn stuck_detector_does_not_yield_when_robot_is_advancing() {
        let mut detector = StuckDetector::new();
        let target = Vec2::new(0.5, 0.0);

        // El robot avanza claramente cada tick (0.02 m > umbral de 0.006 m).
        let mut pos = Vec2::new(-0.2, 0.0);
        for _ in 0..200 {
            assert!(detector.update(pos, target).is_none());
            pos.x += 0.02;
        }
    }

    #[test]
    fn stuck_detector_yields_after_thirty_static_ticks() {
        let mut detector = StuckDetector::new();
        let target = Vec2::new(0.5, 0.0);
        let robot_pos = Vec2::new(-0.2, 0.0);

        // Primera llamada inicializa last_pos; las 29 siguientes acumulan stuck.
        // En la llamada #31 (stuck_ticks alcanza 30) debe disparar el escape.
        for _ in 0..30 {
            assert!(detector.update(robot_pos, target).is_none());
        }
        let escape = detector.update(robot_pos, target);
        assert!(escape.is_some(), "el detector debe disparar a los 30 ticks");
    }

    #[test]
    fn stuck_detector_escape_position_stays_inside_field() {
        let mut detector = StuckDetector::new();
        // Target lejos del robot (>0.08 m) para que se acumule stuck_ticks.
        // El robot pegado a la esquina superior derecha del campo lógico tiene
        // que recibir un escape que NO se salga del clamp ±0.68/±0.58.
        let target = Vec2::new(-0.5, -0.5);
        let robot_pos = Vec2::new(0.65, 0.55);

        for _ in 0..30 {
            detector.update(robot_pos, target);
        }
        let escape = detector
            .update(robot_pos, target)
            .expect("debe disparar escape tras 30 ticks atascado");

        assert!(
            escape.x.abs() <= 0.68 + 1e-6 && escape.y.abs() <= 0.58 + 1e-6,
            "escape {:?} fuera del campo lógico clampeado",
            escape
        );
    }

    #[test]
    fn stuck_detector_resets_when_close_to_target() {
        let mut detector = StuckDetector::new();
        let target = Vec2::new(0.0, 0.0);
        // Robot dentro del radio de llegada (8 cm) — no debe acumular stuck
        // aunque esté quieto.
        let robot_pos = Vec2::new(0.05, 0.0);

        for _ in 0..100 {
            assert!(detector.update(robot_pos, target).is_none());
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    //  Stress tests del catálogo (Fase 2)
    //
    //  La policy RL puede emitir targets en cualquier punto del rango
    //  normalizado, incluyendo casos degenerados. Estos tests verifican que
    //  las 4 skills del catálogo no producen NaN/Inf, no divergen y no
    //  pegan al robot a las paredes en ningún caso patológico.
    // ─────────────────────────────────────────────────────────────────────

    fn assert_command_finite(cmd: &MotionCommand, label: &str) {
        assert!(cmd.vx.is_finite(), "{label}: vx no-finito ({:?})", cmd.vx);
        assert!(cmd.vy.is_finite(), "{label}: vy no-finito ({:?})", cmd.vy);
        assert!(
            cmd.omega.is_finite(),
            "{label}: omega no-finito ({:?})",
            cmd.omega
        );
        assert!(
            cmd.orientation.is_finite(),
            "{label}: orientation no-finita"
        );
    }

    #[test]
    fn goto_skill_stays_finite_with_extreme_targets() {
        let motion = Motion::new();
        let world = World::new(3, 3);
        let robot = make_robot(0, 0.0, 0.0, 0.0);

        // Targets en las cuatro esquinas del campo lógico, plus puntos fuera
        // del campo (la policy RL puede emitir cualquiera de éstos).
        let targets = [
            Vec2::new(0.70, 0.60),
            Vec2::new(-0.70, -0.60),
            Vec2::new(0.70, -0.60),
            Vec2::new(-0.70, 0.60),
            Vec2::new(2.0, 2.0),    // fuera del campo: GoTo debe sobrevivir
            Vec2::new(-5.0, -5.0),  // muy fuera del campo
        ];
        for target in targets {
            let mut skill = GoToSkill::new(target);
            for tick in 0..200 {
                let cmd = skill.tick(&robot, &world, &motion);
                assert_command_finite(&cmd, &format!("target={target:?} tick={tick}"));
            }
        }
    }

    #[test]
    fn goto_skill_target_at_robot_position_is_safe() {
        let motion = Motion::new();
        let world = World::new(3, 3);
        let robot = make_robot(0, 0.3, 0.2, 45.0);
        let mut skill = GoToSkill::new(robot.position);

        let cmd = skill.tick(&robot, &world, &motion);

        // Target = posición actual → no debe pedir velocidad significativa.
        // (Puede haber un pequeño residuo del PID de heading si el robot no
        // mira al target; lo aceptamos pero verificamos que está acotado.)
        assert_command_finite(&cmd, "target_at_robot");
        let speed = (cmd.vx * cmd.vx + cmd.vy * cmd.vy).sqrt();
        assert!(speed < 0.5, "speed {speed} demasiado alta con target sobre robot");
    }

    #[test]
    fn face_point_target_at_robot_position_is_safe() {
        let motion = Motion::new();
        let world = World::new(3, 3);
        let robot = make_robot(0, 0.2, 0.1, 30.0);
        let mut skill = FacePointSkill::new(robot.position);

        let cmd = skill.tick(&robot, &world, &motion);

        assert_command_finite(&cmd, "face_at_robot");
        // Caso degenerado: dirección indefinida → la skill debe responder
        // con un comando seguro (cero translación, omega acotado).
        assert_eq!(cmd.vx, 0.0);
        assert_eq!(cmd.vy, 0.0);
    }

    #[test]
    fn chase_ball_handles_ball_at_robot_position() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        let robot = make_robot(0, 0.2, -0.1, 0.0);
        world.update_ball(robot.position, Vec2::ZERO);
        let mut skill = ChaseBallSkill::new();

        let cmd = skill.tick(&robot, &world, &motion);

        assert_command_finite(&cmd, "ball_at_robot");
    }

    #[test]
    fn chase_ball_stable_under_200_ticks_with_static_ball() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.30, 0.10), Vec2::ZERO);
        let robot = make_robot(0, -0.20, 0.0, 0.0);
        let mut skill = ChaseBallSkill::new();

        // 200 ticks sin que cambie el robot ni la pelota: el comando
        // resultante debe permanecer finito (no debe explotar el integral del PID).
        for tick in 0..200 {
            let cmd = skill.tick(&robot, &world, &motion);
            assert_command_finite(&cmd, &format!("chase_ball tick={tick}"));
        }
    }

    #[test]
    fn spin_skill_omega_uses_config_magnitude() {
        let motion = Motion::new();
        let world = World::new(3, 3);
        let robot = make_robot(0, 0.0, 0.0, 0.0);
        let cfg = SkillConfig::default();
        let mut skill = SpinSkill::with_config(&cfg);

        skill.set_direction_from(Vec2::new(1.0, 0.0));
        let cmd = skill.tick(&robot, &world, &motion);

        assert!((cmd.omega - cfg.spin_omega).abs() < 1e-9);
        assert_eq!(cmd.vx, 0.0);
        assert_eq!(cmd.vy, 0.0);
    }

    #[test]
    fn spin_skill_set_direction_handles_zero_explicitly() {
        let mut skill = SpinSkill::new();
        skill.set_direction_from(Vec2::new(0.0, 5.0)); // x = 0 explícito
        let robot = make_robot(0, 0.0, 0.0, 0.0);
        let cmd = skill.tick(&robot, &World::new(3, 3), &Motion::new());
        assert_eq!(cmd.omega, 0.0, "x=0 debe traducir a no-spin");
    }
}
