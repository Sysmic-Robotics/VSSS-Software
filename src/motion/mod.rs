mod benchmark;
mod commands;
mod environment;
mod pid;
mod recovery;
#[cfg(test)]
pub(crate) mod test_plant;
mod uvf;

pub use benchmark::{MotionBenchmarkScenario, MotionKpi, summarize_commands};
pub use commands::{KickerCommand, MotionCommand, RobotCommand};
pub use environment::Environment;
pub use pid::PIDController;
pub use recovery::BorderRecovery;
pub use uvf::UniVectorField;

use crate::world::{RobotState, World};
use glam::Vec2;
use std::collections::HashMap;
use std::sync::Mutex;

const CONTROL_DT: f64 = 0.016; // ~60 Hz

/// Parámetros tunables del sistema de movimiento.
/// `MotionConfig::default()` toma los defaults de `params::MotionParams`;
/// `MotionConfig::from_env()` usa el JSON vigente.
#[derive(Debug, Clone)]
pub struct MotionConfig {
    /// Velocidad lineal máxima (m/s)
    pub max_linear_speed: f64,
    /// Velocidad lineal mínima — umbral bajo del perfil de frenado (m/s)
    pub min_linear_speed: f64,
    /// Velocidad angular máxima (rad/s)
    pub max_angular_speed: f64,
    /// Distancia al destino bajo la cual el robot se considera "llegado" (m)
    pub arrival_threshold: f32,
    /// Distancia desde la que empieza a frenar linealmente (m)
    pub brake_distance: f32,
    /// Radio de influencia de obstáculos en UVF (m)
    pub uvf_influence_radius: f32,
    /// Ganancia repulsiva del UVF — más alto = deflexión más brusca
    pub uvf_k_rep: f32,
    /// Ganancia del seguimiento de heading hacia la dirección del UVF (rad/s por rad).
    pub heading_gain: f64,
    /// Límite de aceleración del avance comandado, sobre el comando anterior (m/s²).
    pub max_linear_accel: f64,
    /// Robot simétrico con dos caras de contacto (frente y espalda): la orientación
    /// se trata módulo 180°. El seguimiento de heading pliega el error a ±90° y avanza
    /// de espaldas (v < 0) cuando la dirección deseada queda detrás; el PID de
    /// `face_to` apunta a la cara más cercana. Activar con `VSSL_BIDIRECTIONAL=1`
    /// (ver `MotionConfig::from_env`).
    pub bidirectional: bool,
}

impl Default for MotionConfig {
    fn default() -> Self {
        Self::from_params(&crate::params::MotionParams::default())
    }
}

impl MotionConfig {
    /// Config desde los parámetros calibrables (`config/team_params.json`).
    pub fn from_params(p: &crate::params::MotionParams) -> Self {
        Self {
            max_linear_speed: p.max_linear_speed,
            min_linear_speed: p.min_linear_speed,
            max_angular_speed: p.max_angular_speed,
            arrival_threshold: p.arrival_threshold,
            brake_distance: p.brake_distance,
            uvf_influence_radius: p.uvf_influence_radius,
            uvf_k_rep: p.uvf_k_rep,
            heading_gain: p.heading_gain,
            max_linear_accel: p.max_linear_accel,
            bidirectional: p.bidirectional,
        }
    }

    /// Parámetros del JSON vigente + overrides por variables de entorno:
    /// - `VSSL_BIDIRECTIONAL=1|true|on` (o `0|false|off`) → modo de dos caras.
    pub fn from_env() -> Self {
        let mut cfg = Self::from_params(&crate::params::params().motion);
        if let Ok(v) = std::env::var("VSSL_BIDIRECTIONAL") {
            cfg.bidirectional = matches!(v.trim(), "1" | "true" | "on");
        }
        cfg
    }
}

/// Módulo principal de control de movimiento
pub struct Motion {
    uvf: UniVectorField,
    pub config: MotionConfig,
    pid_x_by_robot: Mutex<HashMap<(i32, i32), PIDController>>,
    pid_y_by_robot: Mutex<HashMap<(i32, i32), PIDController>>,
    pid_theta_by_robot: Mutex<HashMap<(i32, i32), PIDController>>,
    /// Último avance comandado por `move_and_face` (m/s, con signo), para la rampa.
    v_prev_by_robot: Mutex<HashMap<(i32, i32), f64>>,
}

impl Motion {
    pub fn new() -> Self {
        Self::with_config(MotionConfig::default())
    }

    pub fn with_config(config: MotionConfig) -> Self {
        let mut uvf = UniVectorField::new();
        uvf.influence_radius = config.uvf_influence_radius;
        uvf.k_rep = config.uvf_k_rep;
        Self {
            uvf,
            config,
            pid_x_by_robot: Mutex::new(HashMap::new()),
            pid_y_by_robot: Mutex::new(HashMap::new()),
            pid_theta_by_robot: Mutex::new(HashMap::new()),
            v_prev_by_robot: Mutex::new(HashMap::new()),
        }
    }

    /// Normaliza un ángulo al rango [-π, π]
    /// Implementación consistente con tracker
    pub fn normalize_angle(angle: f64) -> f64 {
        let pi = std::f64::consts::PI;
        let two_pi = 2.0 * pi;
        let mut normalized = angle % two_pi;
        if normalized > pi {
            normalized -= two_pi;
        } else if normalized < -pi {
            normalized += two_pi;
        }
        normalized
    }

    /// Pliega un error de heading (ya normalizado a [-π, π]) al rango [-π/2, π/2]:
    /// para un robot de dos caras, apuntar con la espalda es tan bueno como con el
    /// frente, así que nunca conviene girar más de 90°.
    pub fn fold_bidirectional(error: f64) -> f64 {
        let half_pi = std::f64::consts::FRAC_PI_2;
        let pi = std::f64::consts::PI;
        if error > half_pi {
            error - pi
        } else if error < -half_pi {
            error + pi
        } else {
            error
        }
    }

    /// Dirección del Univector Field (rad) desde el robot hacia `target`, con los otros
    /// robots y la pelota como obstáculos, **excepto** los que están sobre el destino.
    /// Un obstáculo encima del destino haría que el UVF deflecte tangencialmente y nunca
    /// llegue ("no me esquives del lugar al que voy"): p. ej. el staging detrás de la
    /// pelota, o un fantasma de visión sentado en el target.
    ///
    /// No hay paredes virtuales: el robot va hacia su destino, que está dentro de la
    /// cancha; el borde lo cubren el `ZoneGuard` (áreas) y la recuperación de atasco.
    fn uvf_heading(&self, robot_state: &RobotState, target: Vec2, world: &World) -> f32 {
        let env = Environment::new(world, robot_state);
        let near_target_threshold = self.config.uvf_influence_radius * 1.5;
        let ball_pos = env.get_ball_position();
        let mut obstacles: Vec<Vec2> = env
            .get_robots()
            .iter()
            .copied()
            .filter(|&r| (target - r).length() >= near_target_threshold)
            .collect();
        if (target - ball_pos).length() >= near_target_threshold {
            obstacles.push(ball_pos);
        }
        self.uvf.compute(robot_state.position, target, &obstacles)
    }

    /// Navega hacia `move_target` con una ley de seguimiento de heading para robot
    /// diferencial, y al llegar se orienta hacia `face_target`.
    ///
    /// Lejos del destino:
    /// - `e` = error entre el heading y la dirección del UVF (en bidireccional se pliega
    ///   a ±90° y el avance va de espaldas);
    /// - `omega = clamp(heading_gain · e, ±max_angular_speed)`;
    /// - avance `v = v_perfil(d) · max(cos e, 0)` a lo largo del heading, limitado por la
    ///   rampa de aceleración. El comando `(vx, vy)` es paralelo al heading: el
    ///   diferencial lo ejecuta sin pérdida.
    ///
    /// `face_target` no influye mientras navega (el diferencial avanza hacia donde mira:
    /// mirar a otro lado le impedía llegar). A menos de `arrival_threshold` se anula la
    /// traslación y se orienta hacia `face_target` con el PID de heading de la skill.
    #[allow(clippy::too_many_arguments)]
    pub fn move_and_face(
        &self,
        robot_state: &RobotState,
        move_target: Vec2,
        face_target: Vec2,
        world: &World,
        kp: f64,
        ki: f64,
        kd: f64,
    ) -> MotionCommand {
        let key = (robot_state.team, robot_state.id);
        let dist = (move_target - robot_state.position).length();
        if !dist.is_finite() || dist < self.config.arrival_threshold {
            self.v_prev_by_robot
                .lock()
                .expect("v_prev lock poisoned")
                .insert(key, 0.0);
            return self.face_to(robot_state, face_target, kp, ki, kd);
        }

        let theta_d = self.uvf_heading(robot_state, move_target, world) as f64;
        let raw = Self::normalize_angle(theta_d - robot_state.orientation);
        let (err, dir) = if self.config.bidirectional && raw.abs() > std::f64::consts::FRAC_PI_2
        {
            (Self::fold_bidirectional(raw), -1.0)
        } else {
            (raw, 1.0)
        };
        let max_w = self.config.max_angular_speed;
        let omega = (self.config.heading_gain * err).clamp(-max_w, max_w);

        let normalized = (dist / self.config.brake_distance).clamp(0.0, 1.0) as f64;
        let v_profile = self.config.min_linear_speed
            + normalized * (self.config.max_linear_speed - self.config.min_linear_speed);
        let v = self.ramp(key, dir * v_profile * err.cos().max(0.0));

        let th = robot_state.orientation;
        MotionCommand {
            id: robot_state.id,
            team: robot_state.team,
            vx: v * th.cos(),
            vy: v * th.sin(),
            omega,
            orientation: th,
        }
    }

    /// Rampa del avance: cambia como máximo `max_linear_accel · dt` respecto del comando
    /// anterior del mismo robot. No usa la velocidad medida (con ruido, frames perdidos
    /// o el tracker apagado podría dejar el avance pegado abajo). Pedir más de lo que el
    /// robot acelera satura ambas ruedas y se pierde el giro.
    fn ramp(&self, key: (i32, i32), v_target: f64) -> f64 {
        let mut prev_by_robot = self.v_prev_by_robot.lock().expect("v_prev lock poisoned");
        let prev = prev_by_robot.get(&key).copied().unwrap_or(0.0);
        let dv = self.config.max_linear_accel * CONTROL_DT;
        let v = v_target.clamp(prev - dv, prev + dv);
        prev_by_robot.insert(key, v);
        v
    }

    /// Reinicia el estado de control de un robot (PID de heading y rampa del avance).
    /// Lo llama el catálogo de skills cuando el robot cambia de skill.
    pub fn reset_robot(&self, team: i32, id: i32) {
        let key = (team, id);
        self.pid_theta_by_robot
            .lock()
            .expect("pid_theta lock poisoned")
            .remove(&key);
        self.v_prev_by_robot
            .lock()
            .expect("v_prev lock poisoned")
            .remove(&key);
    }

    /// Movimiento directo sin evasión de obstáculos
    pub fn move_direct(&self, robot_state: &RobotState, target: Vec2) -> MotionCommand {
        let diff = target - robot_state.position;
        let distance = diff.length();

        // Si está muy cerca del objetivo, detenerse
        if !distance.is_finite() || distance < self.config.arrival_threshold {
            return MotionCommand {
                id: robot_state.id,
                team: robot_state.team,
                vx: 0.0,
                vy: 0.0,
                omega: 0.0,
                orientation: robot_state.orientation,
            };
        }

        let direction = diff / distance;
        // Perfil proporcional con zona de frenado: ágil lejos del objetivo y suave al aproximar.
        let normalized = (distance / self.config.brake_distance).clamp(0.0, 1.0);
        let speed = (self.config.min_linear_speed as f32)
            + normalized * ((self.config.max_linear_speed - self.config.min_linear_speed) as f32);

        MotionCommand {
            id: robot_state.id,
            team: robot_state.team,
            vx: (direction.x * speed) as f64,
            vy: (direction.y * speed) as f64,
            omega: 0.0,
            orientation: robot_state.orientation, // Guardar orientación para conversión a coordenadas locales
        }
    }

    /// Control PID personalizado para movimiento en X e Y
    #[allow(clippy::too_many_arguments)]
    pub fn motion(
        &self,
        robot_state: &RobotState,
        target: Vec2,
        _world: &World,
        kp_x: f64,
        ki_x: f64,
        kp_y: f64,
        ki_y: f64,
    ) -> MotionCommand {
        let error = target - robot_state.position;

        // Controladores PID persistentes por robot para conservar integral/derivativa entre ticks.
        let key = (robot_state.team, robot_state.id);
        let (vx, vy) = {
            let mut pid_x_map = self.pid_x_by_robot.lock().expect("pid_x lock poisoned");
            let mut pid_y_map = self.pid_y_by_robot.lock().expect("pid_y lock poisoned");
            let pid_x = pid_x_map
                .entry(key)
                .or_insert_with(|| PIDController::new(kp_x, ki_x, 0.0));
            let pid_y = pid_y_map
                .entry(key)
                .or_insert_with(|| PIDController::new(kp_y, ki_y, 0.0));
            pid_x.set_gains(kp_x, ki_x, 0.0);
            pid_y.set_gains(kp_y, ki_y, 0.0);
            (
                pid_x.compute(error.x as f64, CONTROL_DT),
                pid_y.compute(error.y as f64, CONTROL_DT),
            )
        };

        // Limitar velocidad máxima
        let max_speed = self.config.max_linear_speed;
        let speed = (vx * vx + vy * vy).sqrt();
        let (vx_limited, vy_limited) = if speed > max_speed {
            let scale = max_speed / speed;
            (vx * scale, vy * scale)
        } else {
            (vx, vy)
        };

        MotionCommand {
            id: robot_state.id,
            team: robot_state.team,
            vx: vx_limited,
            vy: vy_limited,
            omega: 0.0,
            orientation: robot_state.orientation,
        }
    }

    /// Orientar hacia un punto
    pub fn face_to(
        &self,
        robot_state: &RobotState,
        target: Vec2,
        kp: f64,
        ki: f64,
        kd: f64,
    ) -> MotionCommand {
        let direction = (target - robot_state.position).normalize_or_zero();
        if direction.length_squared() < f32::EPSILON {
            // target coincide con robot: mantener orientación actual, sin omega
            return MotionCommand {
                id: robot_state.id,
                team: robot_state.team,
                vx: 0.0,
                vy: 0.0,
                omega: 0.0,
                orientation: robot_state.orientation,
            };
        }
        let target_angle = direction.y.atan2(direction.x) as f64;
        self.face_to_angle(robot_state, target_angle, kp, ki, kd)
    }

    /// Orientar hacia un ángulo específico
    pub fn face_to_angle(
        &self,
        robot_state: &RobotState,
        target_angle: f64,
        kp: f64,
        ki: f64,
        kd: f64,
    ) -> MotionCommand {
        let mut error = Self::normalize_angle(target_angle - robot_state.orientation);
        if self.config.bidirectional {
            error = Self::fold_bidirectional(error);
        }
        let key = (robot_state.team, robot_state.id);
        let omega = {
            let mut pid_theta_map = self
                .pid_theta_by_robot
                .lock()
                .expect("pid_theta lock poisoned");
            let pid = pid_theta_map
                .entry(key)
                .or_insert_with(|| PIDController::new(kp, ki, kd));
            pid.set_gains(kp, ki, kd);
            pid.compute(error, CONTROL_DT)
        };

        // Limitar velocidad angular máxima
        let max_omega = self.config.max_angular_speed;
        let omega_limited = omega.clamp(-max_omega, max_omega);

        MotionCommand {
            id: robot_state.id,
            team: robot_state.team,
            vx: 0.0,
            vy: 0.0,
            omega: omega_limited,
            orientation: robot_state.orientation,
        }
    }
}

impl Default for Motion {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::test_plant::{Case, Plant, run};
    use super::*;
    use crate::skills::SkillId;

    const FAR_BALL: Vec2 = Vec2::new(0.6, 0.5);

    fn robot(x: f32, y: f32, theta: f64) -> RobotState {
        let mut r = RobotState::new(0, 0);
        r.position = Vec2::new(x, y);
        r.orientation = theta;
        r
    }

    fn v_body(cmd: &MotionCommand) -> f64 {
        cmd.vx * cmd.orientation.cos() + cmd.vy * cmd.orientation.sin()
    }

    fn bidir() -> Motion {
        let mut cfg = MotionConfig::default();
        cfg.bidirectional = true;
        Motion::with_config(cfg)
    }

    fn angle_diff(a: f64, b: f64) -> f64 {
        Motion::normalize_angle(a - b).abs()
    }

    #[test]
    fn test_normalize_angle() {
        let pi = std::f64::consts::PI;
        assert!((Motion::normalize_angle(0.0) - 0.0).abs() < 1e-10);
        assert!((Motion::normalize_angle(pi) - pi).abs() < 1e-10);
        // -pi normalizado sigue siendo -pi (o muy cerca)
        let normalized_neg_pi = Motion::normalize_angle(-pi);
        assert!(
            (normalized_neg_pi - (-pi)).abs() < 1e-10 || (normalized_neg_pi - pi).abs() < 1e-10
        );
        assert!((Motion::normalize_angle(2.0 * pi) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn fold_bidirectional_never_exceeds_quarter_turn() {
        let pi = std::f64::consts::PI;
        assert!((Motion::fold_bidirectional(pi) - 0.0).abs() < 1e-9);
        assert!((Motion::fold_bidirectional(-pi) - 0.0).abs() < 1e-9);
        assert!((Motion::fold_bidirectional(2.0) - (2.0 - pi)).abs() < 1e-9);
        assert!((Motion::fold_bidirectional(-2.0) - (-2.0 + pi)).abs() < 1e-9);
        assert!((Motion::fold_bidirectional(0.7) - 0.7).abs() < 1e-9);
        for e in [-3.1, -2.0, -1.0, 0.0, 1.0, 2.0, 3.1] {
            assert!(Motion::fold_bidirectional(e).abs() <= std::f64::consts::FRAC_PI_2 + 1e-9);
        }
    }

    #[test]
    fn bidirectional_face_uses_back_when_target_is_behind() {
        let motion = bidir();
        let r = robot(0.0, 0.0, std::f64::consts::PI); // mira a -x
        // Target exactamente detrás (+x): con dos caras ya está alineado → omega ≈ 0.
        let cmd = motion.face_to(&r, Vec2::new(1.0, 0.0), 3.0, 0.0, 0.0);
        assert!(cmd.omega.abs() < 1e-6, "omega={} debería ser ~0", cmd.omega);

        // Sin bidireccional, el mismo caso pide media vuelta completa.
        let motion_fwd = Motion::new();
        let cmd_fwd = motion_fwd.face_to(&r, Vec2::new(1.0, 0.0), 3.0, 0.0, 0.0);
        assert!(cmd_fwd.omega.abs() > 1.0);
    }

    // ── Ley de seguimiento de heading ──────────────────────────────────────

    #[test]
    fn command_follows_the_heading() {
        // El comando es paralelo al heading en cualquier pose: el diferencial lo
        // ejecuta sin descartar nada (antes salía en la dirección del UVF).
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.05, 0.02), Vec2::ZERO);
        world.update_robot(1, 1, Vec2::new(-0.1, 0.12), 0.0, Vec2::ZERO, 0.0);
        for motion in [Motion::new(), bidir()] {
            for (x, y, th) in [(-0.4, 0.0, 0.0), (0.3, -0.2, 2.0), (0.0, 0.3, -1.2), (-0.2, -0.4, 3.0)] {
                let r = robot(x, y, th);
                for target in [Vec2::new(0.4, 0.1), Vec2::new(-0.5, -0.3), Vec2::new(0.0, 0.45)] {
                    for _ in 0..20 {
                        let c = motion.move_and_face(&r, target, target, &world, 3.0, 0.08, 0.2);
                        let lateral = -c.vx * th.sin() + c.vy * th.cos();
                        assert!(lateral.abs() < 1e-9, "componente lateral {lateral} en {r:?}");
                    }
                }
            }
        }
    }

    #[test]
    fn bidirectional_drives_backwards_to_a_target_behind() {
        let motion = bidir();
        let world = World::new(3, 3);
        let r = robot(-0.4, 0.0, std::f64::consts::PI); // de espaldas al target (+x)
        let target = Vec2::new(0.4, 0.0);
        let mut cmd = motion.move_and_face(&r, target, target, &world, 3.0, 0.0, 0.0);
        for _ in 0..100 {
            cmd = motion.move_and_face(&r, target, target, &world, 3.0, 0.0, 0.0);
        }
        // Avanza de espaldas a velocidad plena (la rampa ya llegó) y sin girar.
        assert!(v_body(&cmd) < -0.9, "v={}", v_body(&cmd));
        assert!(cmd.omega.abs() < 1e-6, "omega={}", cmd.omega);
    }

    #[test]
    fn frontal_turns_in_place_to_a_target_behind() {
        let motion = Motion::new();
        let world = World::new(3, 3);
        let r = robot(-0.4, 0.0, std::f64::consts::PI);
        let cmd = motion.move_and_face(&r, Vec2::new(0.4, 0.0), Vec2::new(0.4, 0.0), &world, 3.0, 0.0, 0.0);
        assert_eq!((cmd.vx, cmd.vy), (0.0, 0.0), "sin traslación hasta alinear");
        assert!(cmd.omega.abs() > 1.0);
    }

    #[test]
    fn obstacle_ahead_deflects_and_omega_follows_it() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        world.update_ball(FAR_BALL, Vec2::ZERO);
        world.update_robot(0, 1, Vec2::new(0.0, 0.03), 0.0, Vec2::ZERO, 0.0);
        let r = robot(-0.15, 0.0, 0.0);
        let target = Vec2::new(0.4, 0.0);
        let h = motion.uvf_heading(&r, target, &world) as f64;
        assert!(h.abs() > 0.05, "el UVF debe desviarse: {h:.3}");
        let cmd = motion.move_and_face(&r, target, target, &world, 3.0, 0.0, 0.0);
        assert!(cmd.omega * h > 0.0, "omega persigue la dirección desviada");
    }

    #[test]
    fn obstacle_behind_does_not_deflect() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        world.update_ball(FAR_BALL, Vec2::ZERO);
        world.update_robot(0, 1, Vec2::new(-0.25, 0.0), 0.0, Vec2::ZERO, 0.0);
        let h = motion.uvf_heading(&robot(-0.15, 0.0, 0.0), Vec2::new(0.4, 0.0), &world);
        assert!(h.abs() < 0.01, "h={h:.3}");
    }

    /// Regresión: un robot obstáculo sentado encima del target no debe deflectar al UVF.
    /// Caso real: vision-sysmic emite un fantasma sobre (0,0) y el robot intenta ir ahí.
    #[test]
    fn obstacle_on_the_target_is_ignored() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        world.update_robot(0, 1, Vec2::new(0.0, 0.0), 0.0, Vec2::ZERO, 0.0);
        let h = motion.uvf_heading(&robot(-0.40, -0.30, 0.0), Vec2::ZERO, &world) as f64;
        assert!(angle_diff(h, 0.3f64.atan2(0.4)) < 0.01, "h={h:.3}");
    }

    /// Regresión de la pelota de borde: con el target pegado al borde físico (~0.72),
    /// la dirección apunta al target (no hay paredes virtuales que lo desvíen).
    #[test]
    fn ball_against_the_border_is_reachable() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.72, 0.0), Vec2::ZERO);
        let h = motion.uvf_heading(&robot(0.66, 0.0, 0.0), Vec2::new(0.72, 0.0), &world);
        assert!(h.abs() < 0.01, "h={h:.3}");
    }

    #[test]
    fn no_virtual_walls_near_the_border() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        world.update_ball(FAR_BALL, Vec2::ZERO);
        // Pegado al borde +x con el destino del otro lado: recto al destino.
        let h = motion.uvf_heading(&robot(0.69, 0.0, 0.0), Vec2::new(-0.5, 0.0), &world) as f64;
        assert!(angle_diff(h, std::f64::consts::PI) < 0.01, "h={h:.3}");
        // Pegado al borde −y mirando la pared, destino al centro: recto hacia +y.
        let h = motion.uvf_heading(&robot(0.0, -0.58, -1.57), Vec2::ZERO, &world) as f64;
        assert!(angle_diff(h, std::f64::consts::FRAC_PI_2) < 0.01, "h={h:.3}");
    }

    #[test]
    fn clustered_obstacles_still_give_a_finite_command() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        for i in 0..3 {
            let angle = (i as f32) * std::f32::consts::PI * 2.0 / 3.0;
            world.update_robot(i + 1, 0, Vec2::new(0.05 * angle.cos(), 0.05 * angle.sin()), 0.0, Vec2::ZERO, 0.0);
        }
        world.update_ball(Vec2::new(0.0, -0.5), Vec2::ZERO);
        let target = Vec2::new(0.5, 0.0);
        let cmd = motion.move_and_face(&robot(0.0, 0.0, 0.0), target, target, &world, 3.0, 0.0, 0.0);
        assert!(cmd.vx.is_finite() && cmd.vy.is_finite() && cmd.omega.is_finite());
    }

    #[test]
    fn arrival_stops_translation_and_faces_the_face_target() {
        let motion = Motion::new();
        let world = World::new(3, 3);
        let r = robot(0.0, 0.0, 0.0);
        let cmd = motion.move_and_face(&r, Vec2::new(0.02, 0.0), Vec2::new(0.0, 0.5), &world, 3.0, 0.0, 0.0);
        assert_eq!((cmd.vx, cmd.vy), (0.0, 0.0));
        assert!(cmd.omega > 0.0, "gira hacia face_target (+y)");
    }

    // ── Rampa de aceleración ───────────────────────────────────────────────

    #[test]
    fn ramp_limits_the_change_including_the_face_switch() {
        let motion = bidir();
        let world = World::new(3, 3);
        let r = robot(0.0, 0.0, 0.0);
        let dv = motion.config.max_linear_accel * CONTROL_DT;
        let mut prev = 0.0;
        let mut went_negative = false;
        for k in 0..160 {
            // Primero hacia adelante; después el destino pasa atrás (cambia la cara).
            let target = if k < 60 { Vec2::new(0.6, 0.0) } else { Vec2::new(-0.6, 0.0) };
            let v = v_body(&motion.move_and_face(&r, target, target, &world, 3.0, 0.0, 0.0));
            assert!((v - prev).abs() <= dv + 1e-12, "salto de {} en el tick {k}", v - prev);
            went_negative |= v < -0.1;
            prev = v;
        }
        assert!(went_negative, "debe terminar avanzando de espaldas");
    }

    #[test]
    fn ramp_does_not_use_the_measured_velocity() {
        // `velocity` en cero (tracker apagado): la rampa sigue el comando anterior y
        // llega al perfil (1.2 m/s, destino lejos y alineado) en 1.2 / (a·dt) ticks.
        let motion = Motion::new();
        let world = World::new(3, 3);
        let r = robot(-0.5, 0.0, 0.0);
        assert_eq!(r.velocity, Vec2::ZERO);
        let target = Vec2::new(0.6, 0.0);
        let first = v_body(&motion.move_and_face(&r, target, target, &world, 3.0, 0.0, 0.0));
        assert!((first - motion.config.max_linear_accel * CONTROL_DT).abs() < 1e-12);
        let n = (motion.config.max_linear_speed / (motion.config.max_linear_accel * CONTROL_DT)).ceil() as usize;
        let mut v = first;
        for _ in 0..n {
            v = v_body(&motion.move_and_face(&r, target, target, &world, 3.0, 0.0, 0.0));
        }
        assert!((v - motion.config.max_linear_speed).abs() < 1e-9, "v={v}");
    }

    #[test]
    fn reset_robot_restarts_the_ramp() {
        let motion = Motion::new();
        let world = World::new(3, 3);
        let r = robot(-0.5, 0.0, 0.0);
        let target = Vec2::new(0.6, 0.0);
        for _ in 0..50 {
            motion.move_and_face(&r, target, target, &world, 3.0, 0.0, 0.0);
        }
        motion.reset_robot(0, 0);
        let v = v_body(&motion.move_and_face(&r, target, target, &world, 3.0, 0.0, 0.0));
        assert!(v <= motion.config.max_linear_accel * CONTROL_DT + 1e-12, "v={v}");
    }

    // ── Con la planta diferencial (repro de la investigación del 2026-10-03) ──

    #[test]
    fn mark_reaches_its_point_while_the_ball_is_elsewhere() {
        // A1: mirando la pelota y con el destino a ~70°, el robot no avanzaba.
        let ball = Vec2::new(0.3, 0.3);
        let target = Vec2::new(-0.3, 0.0);
        let to_ball = (0.6f64).atan2(0.1).to_degrees();
        let trace = run(&Case::new(SkillId::Mark, target, (0.2, -0.3, to_ball), ball));
        let k = trace.first_within(target, 0.05).expect("Mark no llegó");
        assert!(k < 180, "tardó {:.2} s", k as f64 / 60.0);
        let last = trace.last();
        let desired = ((ball.y - last.y) as f64).atan2((ball.x - last.x) as f64);
        assert!(angle_diff(desired, last.th) < 0.1, "al final mira la pelota");
        assert!(trace.steps.iter().all(|s| !s.escaping), "sin escapes espurios");
    }

    #[test]
    fn short_lateral_goto_does_not_orbit() {
        let target = Vec2::new(0.0, 0.15);
        for case in [
            Case::new(SkillId::GoTo, target, (0.0, 0.0, 0.0), FAR_BALL),
            Case::new(SkillId::GoTo, target, (0.0, 0.0, 0.0), FAR_BALL).bidirectional(),
        ] {
            let trace = run(&case);
            let k = trace.first_within(target, 0.05).expect("no llegó");
            assert!(k < 90, "tardó {:.2} s", k as f64 / 60.0);
            assert!(trace.turn_deg(k) < 150.0, "giro {:.0}°", trace.turn_deg(k));
            assert!(trace.steps.iter().all(|s| !s.escaping));
        }
    }

    #[test]
    fn approach_aligned_finishes() {
        // N4: con arrival_threshold ≥ approach_pos_tol, el robot se detenía a 0.058 m
        // del staging y la skill nunca terminaba.
        let trace = run(&Case::new(
            SkillId::ApproachAligned,
            Vec2::new(0.75, 0.0),
            (-0.1, -0.35, 90.0),
            Vec2::ZERO,
        ));
        let k = trace.steps.iter().position(|s| s.done).expect("ApproachAligned no terminó");
        assert!(k < 300, "tardó {:.2} s", k as f64 / 60.0);
    }

    #[test]
    fn goto_from_facing_the_wall_does_not_oscillate() {
        // Pegado al borde mirando la pared, destino al centro (frontal: media vuelta).
        let target = Vec2::ZERO;
        let trace = run(&Case::new(SkillId::GoTo, target, (0.0, -0.57, -90.0), FAR_BALL));
        let k = trace.first_within(target, 0.05).expect("no llegó");
        assert!(trace.turn_deg(k) < 360.0, "giro {:.0}°", trace.turn_deg(k));
    }

    #[test]
    fn plant_approach_then_push_moves_the_ball() {
        // Banco headless: aproximación detrás de la pelota con move_and_face y empuje con
        // move_direct + face_to, sobre la planta diferencial y una física simple de pelota.
        let goal_pos = Vec2::new(0.75_f32, 0.0_f32);
        let ball_start = Vec2::new(0.1_f32, 0.2_f32);
        let (kp, ki, kd) = (1.2_f64, 0.0_f64, 0.10_f64);
        let staging_offset = 0.16_f32;
        let staging_tol = 0.08_f32;
        let contact_dist = 0.075_f32;
        let ball_friction = 0.92_f32;

        let motion = Motion::new();
        let mut plant = Plant::new(-0.3, 0.15, 60.0);
        let mut ball_pos = ball_start;
        let mut ball_vel = Vec2::ZERO;
        let mut in_capture = false;
        let mut staged_tick: Option<usize> = None;
        let mut contact_tick: Option<usize> = None;
        let mut ball_moved_m = 0.0_f32;
        for tick in 0..480 {
            let ball_to_goal = (goal_pos - ball_pos).normalize_or_zero();
            let staging = ball_pos - ball_to_goal * staging_offset;
            let mut r = robot(plant.x, plant.y, plant.th);
            r.velocity = plant.velocity();
            let pos = r.position;
            let dist_staging = (staging - pos).length();
            let along = (pos - ball_pos).dot(goal_pos - ball_pos);
            if along < 0.02 && dist_staging <= staging_tol {
                in_capture = true;
            } else if along > 0.05 {
                in_capture = false;
            }
            if staged_tick.is_none() && in_capture {
                staged_tick = Some(tick);
            }
            let mut world = World::new(3, 3);
            world.update_ball(ball_pos, Vec2::ZERO);
            let cmd = if in_capture {
                let mut c = motion.move_direct(&r, ball_pos + ball_to_goal * 0.12);
                c.omega = motion.face_to(&r, ball_pos, kp, ki, kd).omega;
                c
            } else {
                let face = if dist_staging < staging_tol * 3.0 { ball_pos } else { staging };
                motion.move_and_face(&r, staging, face, &world, kp, ki, kd)
            };
            plant.step(&cmd);
            let robot_pos = Vec2::new(plant.x, plant.y);
            if (ball_pos - robot_pos).length() < contact_dist {
                contact_tick.get_or_insert(tick);
                let forward = Vec2::new(plant.th.cos() as f32, plant.th.sin() as f32);
                ball_vel += forward * (plant.v().max(0.0) as f32) * 0.6 * test_plant::DT as f32;
            }
            ball_vel *= ball_friction;
            let prev = ball_pos;
            ball_pos += ball_vel * test_plant::DT as f32;
            ball_moved_m += (ball_pos - prev).length();
        }
        let staged = staged_tick.expect("nunca llegó al staging");
        assert!(contact_tick.is_some(), "nunca tocó la pelota");
        assert!(ball_moved_m > 0.01, "la pelota no se movió ({ball_moved_m:.4} m)");
        assert!(staged < 240, "la aproximación tardó {:.2} s", staged as f64 / 60.0);
    }

    #[test]
    fn test_move_direct() {
        let motion = Motion::new();
        let r = RobotState::new(0, 0);
        let cmd = motion.move_direct(&r, Vec2::new(1.0, 0.0));

        assert_eq!(cmd.id, 0);
        assert!(cmd.vx > 0.0);
        assert_eq!(cmd.vy, 0.0);
    }

    #[test]
    fn test_motion_pid_persists_state_between_ticks() {
        let motion = Motion::new();
        let r = RobotState::new(0, 0);
        let world = World::new(3, 3);
        let target = Vec2::new(1.0, 0.0);

        let cmd_1 = motion.motion(&r, target, &world, 1.0, 0.4, 1.0, 0.0);
        let cmd_2 = motion.motion(&r, target, &world, 1.0, 0.4, 1.0, 0.0);

        // Con componente integral no nula, el segundo tick debe acumular al menos el mismo esfuerzo.
        assert!(cmd_2.vx >= cmd_1.vx);
    }
}
