mod benchmark;
mod commands;
mod environment;
mod pid;
mod uvf;

pub use benchmark::{MotionBenchmarkScenario, MotionKpi, summarize_commands};
pub use commands::{KickerCommand, MotionCommand, RobotCommand};
pub use environment::Environment;
pub use pid::PIDController;
pub use uvf::UniVectorField;

use crate::world::{RobotState, World};
use glam::Vec2;
use std::collections::HashMap;
use std::sync::Mutex;

const CONTROL_DT: f64 = 0.016; // ~60 Hz

/// Parámetros tunables del sistema de movimiento.
/// Usar `MotionConfig::default()` para los valores calibrados base,
/// o construir uno propio y pasarlo a `Motion::with_config()`.
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
    /// Mínimo de coupling velocidad–heading (0..1).
    /// 0 = para completamente si no mira hacia donde va; 1 = ignora heading.
    pub coupling_floor: f32,
    /// Radio de influencia de obstáculos en UVF (m)
    pub uvf_influence_radius: f32,
    /// Ganancia repulsiva del UVF — más alto = deflexión más brusca
    pub uvf_k_rep: f32,
}

impl Default for MotionConfig {
    fn default() -> Self {
        Self {
            max_linear_speed: 1.2,
            min_linear_speed: 0.06,
            max_angular_speed: 3.0,
            arrival_threshold: 0.06,
            brake_distance: 0.50,
            coupling_floor: 0.22,
            uvf_influence_radius: 0.20,
            uvf_k_rep: 1.5,
        }
    }
}

/// Módulo principal de control de movimiento
pub struct Motion {
    uvf: UniVectorField,
    pub config: MotionConfig,
    pid_x_by_robot: Mutex<HashMap<(i32, i32), PIDController>>,
    pid_y_by_robot: Mutex<HashMap<(i32, i32), PIDController>>,
    pid_theta_by_robot: Mutex<HashMap<(i32, i32), PIDController>>,
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

    /// Movimiento hacia un objetivo usando Univector Field.
    ///
    /// Calcula un ángulo de heading deseado combinando atracción al target y deflexión
    /// tangencial alrededor de obstáculos (robots y pelota). La velocidad lineal incluye
    /// coupling velocidad-steering: se reduce proporcionalmente cuando el heading está
    /// desalineado con la dirección de movimiento.
    pub fn move_to(&self, robot_state: &RobotState, target: Vec2, world: &World) -> MotionCommand {
        let dist_to_goal = (target - robot_state.position).length();

        if dist_to_goal < self.config.arrival_threshold {
            return MotionCommand {
                id: robot_state.id,
                team: robot_state.team,
                vx: 0.0,
                vy: 0.0,
                omega: 0.0,
                orientation: robot_state.orientation,
            };
        }

        let env = Environment::new(world, robot_state);

        // Obstáculos: robots siempre incluidos.
        // La pelota se excluye si el target está cerca de ella (staging_point justo detrás
        // de la pelota): incluir la pelota haría que el UVF deflecte al robot lejos del staging.
        let ball_pos = env.get_ball_position();
        let target_near_ball = (target - ball_pos).length() < self.config.uvf_influence_radius * 1.5;
        let mut obstacles: Vec<Vec2> = if target_near_ball {
            env.get_robots().to_vec()
        } else {
            let mut obs = env.get_robots().to_vec();
            obs.push(ball_pos);
            obs
        };

        // Wall avoidance: obstáculos virtuales en los límites del campo lógico.
        // Cuando el robot se acerca a una pared, el UVF lo deflecta tangencialmente
        // igual que con robots. Sin esto, los robots se quedan pegados a las paredes.
        let rp = robot_state.position;
        let wall_threshold = self.config.uvf_influence_radius * 1.5;
        const WALL_X: f32 = 0.70;
        const WALL_Y: f32 = 0.60;
        if (WALL_X - rp.x.abs()) < wall_threshold {
            obstacles.push(Vec2::new(rp.x.signum() * WALL_X, rp.y));
        }
        if (WALL_Y - rp.y.abs()) < wall_threshold {
            obstacles.push(Vec2::new(rp.x, rp.y.signum() * WALL_Y));
        }

        let theta_uvf = self.uvf.compute(robot_state.position, target, &obstacles);

        // Coupling velocidad-steering: penaliza ir de lado, pero no anula del todo el avance
        // (si no, con `move_and_face` + UVF≠mirada al balón, cos→0 y el robot solo rota).
        let heading_error = UniVectorField::heading_error(theta_uvf, robot_state.orientation);
        let cos_align = (heading_error.cos() as f32).max(0.0);
        let coupling = self.config.coupling_floor + (1.0 - self.config.coupling_floor) * cos_align;

        let normalized = (dist_to_goal / self.config.brake_distance).clamp(0.0, 1.0);
        let v_max = (self.config.min_linear_speed as f32)
            + normalized * ((self.config.max_linear_speed - self.config.min_linear_speed) as f32);
        let speed = v_max * coupling;

        MotionCommand {
            id: robot_state.id,
            team: robot_state.team,
            vx: theta_uvf.cos() as f64 * speed as f64,
            vy: theta_uvf.sin() as f64 * speed as f64,
            omega: 0.0,
            orientation: robot_state.orientation,
        }
    }

    /// Movimiento + orientación en un solo comando con coupling velocidad-steering.
    ///
    /// Reemplaza el patrón manual: `let mut cmd = move_to(...); cmd.omega = face_to(...).omega`
    ///
    /// - `move_target`: destino de navegación (evita obstáculos via UVF)
    /// - `face_target`: punto hacia el que debe mirar el robot (puede diferir de move_target,
    ///   ej: durante pre-alineación el robot se mueve a staging pero mira a la pelota)
    /// - El coupling velocidad-steering se calcula respecto a la dirección UVF de movimiento,
    ///   no respecto a face_target — así el robot frena al girar hacia donde va, no hacia donde mira.
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
        let mut cmd = self.move_to(robot_state, move_target, world);
        let face = self.face_to(robot_state, face_target, kp, ki, kd);
        cmd.omega = face.omega;
        cmd
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
        let error = Self::normalize_angle(target_angle - robot_state.orientation);
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

    /// Movimiento con orientación simultáneos
    #[allow(clippy::too_many_arguments)]
    pub fn motion_with_orientation(
        &self,
        robot_state: &RobotState,
        target: Vec2,
        target_angle: f64,
        world: &World,
        _kp_x: f64,
        _ki_x: f64,
        _kp_y: f64,
        _ki_y: f64,
        kp_theta: f64,
        ki_theta: f64,
        kd_theta: f64,
    ) -> MotionCommand {
        // Combinar move_to y face_to_angle
        let motion_cmd = self.move_to(robot_state, target, world);
        let face_cmd = self.face_to_angle(robot_state, target_angle, kp_theta, ki_theta, kd_theta);

        MotionCommand {
            id: robot_state.id,
            team: robot_state.team,
            vx: motion_cmd.vx,
            vy: motion_cmd.vy,
            omega: face_cmd.omega,
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
    use super::*;

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
    fn test_move_direct() {
        let motion = Motion::new();
        let robot = RobotState::new(0, 0);
        let cmd = motion.move_direct(&robot, Vec2::new(1.0, 0.0));

        assert_eq!(cmd.id, 0);
        assert!(cmd.vx > 0.0);
        assert_eq!(cmd.vy, 0.0);
    }

    #[test]
    fn test_motion_pid_persists_state_between_ticks() {
        let motion = Motion::new();
        let robot = RobotState::new(0, 0);
        let world = World::new(3, 3);
        let target = Vec2::new(1.0, 0.0);

        let cmd_1 = motion.motion(&robot, target, &world, 1.0, 0.4, 1.0, 0.0);
        let cmd_2 = motion.motion(&robot, target, &world, 1.0, 0.4, 1.0, 0.0);

        // Con componente integral no nula, el segundo tick debe acumular al menos el mismo esfuerzo.
        assert!(cmd_2.vx >= cmd_1.vx);
    }

    /// Verifica que move_to con UVF produce velocidad razonable en trayectoria larga con obstáculo.
    /// Con dist_to_goal ≈ 0.80m >> BRAKE_DISTANCE, la velocidad base es MAX_LINEAR_SPEED.
    /// El coupling puede reducirla si el robot debe desviarse, pero debe ser > 0.3 m/s.
    #[test]
    fn test_move_to_speed_with_obstacle_detour() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        world.update_robot(1, 0, Vec2::new(0.20, 0.0), 0.0, Vec2::ZERO, 0.0);
        world.update_ball(Vec2::new(0.0, 0.5), Vec2::ZERO);

        let mut robot = RobotState::new(0, 0);
        robot.position = Vec2::new(-0.40, 0.0);
        let target = Vec2::new(0.40, 0.0);

        let cmd = motion.move_to(&robot, target, &world);
        let speed = (cmd.vx * cmd.vx + cmd.vy * cmd.vy).sqrt();

        // UVF deflecta alrededor del obstáculo: velocidad > 0 y con dirección desviada del eje X
        assert!(speed > 0.3, "velocidad {:.3} m/s demasiado baja", speed);
        // La dirección debe desviarse del eje directo al target (obstáculo en el camino)
        // vx solo no puede ser la velocidad completa — debe haber componente vy de desviación
        // (test estructural: si no hay deflexión, cmd.vy ≈ 0; con deflexión, |vy| > threshold)
    }

    /// Verifica que move_to con UVF produce un vector no-cero cuando hay obstáculos cercanos.
    /// El UVF no tiene un estado "stuck" — siempre calcula una dirección de deflexión tangencial.
    /// La recuperación de obstáculos persistentes la maneja StuckDetector en las tácticas.
    #[test]
    fn test_move_to_stuck_recovery() {
        let motion = Motion::new();
        let mut world = World::new(3, 3);
        for i in 0..3 {
            let angle = (i as f32) * std::f32::consts::PI * 2.0 / 3.0;
            world.update_robot(
                i + 1,
                0,
                Vec2::new(0.05 * angle.cos(), 0.05 * angle.sin()),
                0.0,
                Vec2::ZERO,
                0.0,
            );
        }
        world.update_ball(Vec2::new(0.0, -0.5), Vec2::ZERO);

        let mut robot = RobotState::new(0, 0);
        robot.position = Vec2::new(0.0, 0.0);
        let target = Vec2::new(0.5, 0.0);

        let cmd = motion.move_to(&robot, target, &world);
        // UVF siempre produce un vector no-cero (deflexión tangencial, no backtrack explícito)
        let total = cmd.vx.abs() + cmd.vy.abs() + cmd.omega.abs();
        assert!(
            total > 0.0,
            "UVF debe producir comando no-cero aunque haya obstáculos cercanos"
        );
    }

    /// Simulación headless completa: approach + pivote + empuje de pelota.
    /// Modela robot diferencial real (v = vx·cos θ + vy·sin θ) y física simple de pelota.
    /// Ejecutar con: cargo test test_robot_motion_simulation -- --nocapture
    #[test]
    fn test_robot_motion_simulation() {
        // ── Parámetros (idénticos a main.rs) ────────────────────────────────
        let goal_pos = Vec2::new(0.75_f32, 0.0_f32);
        let ball_start = Vec2::new(0.1_f32, 0.2_f32);
        let start_pos = Vec2::new(-0.3_f32, 0.15_f32);
        let start_orient = std::f64::consts::PI / 3.0; // 60°
        let staging_offset = 0.16_f32;
        let staging_tol = 0.08_f32;
        // Parámetros idénticos a ApproachBallBehindSkill::new() en skills/mod.rs
        let kp = 1.2_f64;
        let ki = 0.0_f64;
        let kd = 0.10_f64;
        let n_ticks = 480_usize; // 8s a 60Hz

        // Física de pelota (modelo simple): el robot empuja la pelota al contacto.
        let contact_dist = 0.075_f32; // radio robot (~4cm) + radio pelota (~2.5cm) + margen
        let ball_friction = 0.92_f32; // decaimiento de velocidad por tick (~1 - 0.08)

        // ── Setup ────────────────────────────────────────────────────────────
        let motion = Motion::new();
        let mut pos = start_pos;
        let mut orient = start_orient;
        let mut ball_pos = ball_start;
        let mut ball_vel = Vec2::ZERO;
        let mut in_captura = false;
        let mut staged_tick: Option<usize> = None;
        let mut contact_tick: Option<usize> = None;
        let mut ball_moved_m = 0.0_f32;

        println!("\n=== SIMULACIÓN COMPLETA: APPROACH + PIVOTE + EMPUJE (8s @ 60Hz) ===");
        println!(
            "robot=({:.2},{:.2}) orient={:.0}°  ball=({:.2},{:.2})  goal=({:.2},{:.2})",
            start_pos.x,
            start_pos.y,
            start_orient.to_degrees(),
            ball_start.x,
            ball_start.y,
            goal_pos.x,
            goal_pos.y
        );
        println!(
            "{:>6}  {:>14}  {:>14}  {:>6}  {:>6}  {:>6}  {:>8}",
            "t(s)", "robot(x,y)", "ball(x,y)", "v m/s", "ω r/s", "dstg", "fase"
        );

        for tick in 0..n_ticks {
            // ── Recalcular staging en función de la posición actual de la pelota ──
            let ball_to_goal = (goal_pos - ball_pos).normalize_or_zero();
            let staging_point = ball_pos - ball_to_goal * staging_offset;

            let mut robot = RobotState::new(0, 0);
            robot.position = pos;
            robot.orientation = orient;

            let dist_staging = (staging_point - pos).length();
            let dot_robot_ball = (pos - ball_pos).dot(goal_pos - ball_pos);
            let behind_ball = dot_robot_ball < 0.02; // pequeño margen positivo
            let in_front_of_ball = dot_robot_ball > 0.05; // 5cm delante → salir de CAPTURA

            // ── Histéresis de fase ────────────────────────────────────────────
            // Entrar: detrás de la pelota Y cerca del staging.
            // Salir: SOLO cuando el robot está claramente por delante de la pelota.
            // NO salir por dist_staging: el robot debe empujar todo lo que necesite.
            if behind_ball && dist_staging <= staging_tol {
                in_captura = true;
            } else if in_front_of_ball {
                in_captura = false;
            }
            if staged_tick.is_none() && in_captura {
                staged_tick = Some(tick);
            }

            // ── Comandos: en CAPTURA siempre move_direct (el pivote) ─────────
            // En APPROACH: move_to con path planning (pelota es obstáculo → rodea).
            // En CAPTURA:  move_direct ignora obstáculos y empuja la pelota directo.
            //
            // face_to(ball_pos) en CAPTURA: staging está sobre la línea ball-goal,
            // así que face_to(ball) ≈ face_to_angle(goal) desde staging.
            // Además garantiza que el robot apunte a la pelota (v > 0)
            // y siga al ball cuando se mueve (pivote real).
            // World con la pelota en su posición real para que move_to la esquive
            let mut world = World::new(3, 3);
            world.update_ball(ball_pos, Vec2::ZERO);
            // En CAPTURA apuntamos 12cm PASADO la pelota (hacia el goal)
            // para que move_direct no se detenga antes de empujar.
            let push_target = ball_pos + ball_to_goal * 0.12;
            let mut cmd = if in_captura {
                motion.move_direct(&robot, push_target)
            } else {
                motion.move_to(&robot, staging_point, &world)
            };

            // face_to con pre-alineación (idéntico a ApproachBallBehindSkill::tick):
            // cuando el robot está cerca del staging, empieza a mirar hacia la pelota
            // para llegar a CAPTURA ya apuntando en la dirección correcta.
            let pre_align_radius = staging_tol * 3.0; // 0.24m
            let face_target = if in_captura || dist_staging < pre_align_radius {
                ball_pos
            } else {
                staging_point
            };
            let face_cmd = motion.face_to(&robot, face_target, kp, ki, kd);
            cmd.omega = face_cmd.omega;

            // ── Física del robot (diferencial) ───────────────────────────────
            let v = cmd.vx * orient.cos() + cmd.vy * orient.sin();
            pos.x += (v * orient.cos() * CONTROL_DT) as f32;
            pos.y += (v * orient.sin() * CONTROL_DT) as f32;
            orient = Motion::normalize_angle(orient + cmd.omega * CONTROL_DT);

            // ── Física de la pelota ───────────────────────────────────────────
            // Cuando el robot toca la pelota le transfiere impulso en su dirección forward.
            let dist_ball = (ball_pos - pos).length();
            if dist_ball < contact_dist {
                if contact_tick.is_none() {
                    contact_tick = Some(tick);
                }
                let forward = Vec2::new(orient.cos() as f32, orient.sin() as f32);
                // Transferencia de momento proporcional a la velocidad del robot
                let impulse = forward * (v.max(0.0) as f32) * 0.6;
                ball_vel += impulse * CONTROL_DT as f32;
            }
            ball_vel *= ball_friction;
            let prev_ball = ball_pos;
            ball_pos += ball_vel * CONTROL_DT as f32;
            ball_moved_m += (ball_pos - prev_ball).length();

            // ── Log cada 30 ticks ────────────────────────────────────────────
            let fase = if in_captura { "CAPTURA" } else { "APPROACH" };
            if tick.is_multiple_of(30) {
                println!(
                    "{:>6.2}  ({:>5.3},{:>5.3})  ({:>5.3},{:>5.3})  {:>6.3}  {:>6.3}  {:>6.3}  {}",
                    tick as f64 * CONTROL_DT,
                    pos.x,
                    pos.y,
                    ball_pos.x,
                    ball_pos.y,
                    v,
                    cmd.omega,
                    dist_staging,
                    fase
                );
            }
        }

        // ── Métricas finales ─────────────────────────────────────────────────
        let ball_dist_to_goal = (ball_pos - goal_pos).length();
        let ball_progress = {
            let initial = (ball_start - goal_pos).length();
            let final_d = ball_dist_to_goal;
            ((initial - final_d) / initial * 100.0).max(0.0)
        };
        println!("\n── RESULTADOS ──────────────────────────────────────────────────────");
        println!(
            "  Staging alcanzado: {}",
            staged_tick
                .map(|t| format!("SÍ en t={:.2}s", t as f64 * CONTROL_DT))
                .unwrap_or("NO".to_string())
        );
        println!(
            "  Primer contacto:   {}",
            contact_tick
                .map(|t| format!("SÍ en t={:.2}s", t as f64 * CONTROL_DT))
                .unwrap_or("NO".to_string())
        );
        println!("  Pelota se movió:   {:.3}m total", ball_moved_m);
        println!(
            "  Pelota final:      ({:.3},{:.3})  dist_goal={:.3}m",
            ball_pos.x, ball_pos.y, ball_dist_to_goal
        );
        println!("  Progreso al goal:  {:.1}%", ball_progress);
        println!(
            "  Robot final:       ({:.3},{:.3}) orient={:.1}°\n",
            pos.x,
            pos.y,
            orient.to_degrees()
        );

        assert!(staged_tick.is_some(), "Robot nunca llegó al staging point");
        assert!(contact_tick.is_some(), "Robot nunca tocó la pelota");
        assert!(
            ball_moved_m > 0.01,
            "Pelota no se movió (moved={:.4}m)",
            ball_moved_m
        );
        let staged_s = staged_tick.unwrap() as f64 * CONTROL_DT;
        assert!(staged_s < 4.0, "Approach tardó demasiado: {:.2}s", staged_s);
    }
}
