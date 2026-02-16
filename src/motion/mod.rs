mod commands;
mod benchmark;
mod environment;
mod path_planner;
mod pid;

pub use commands::{MotionCommand, KickerCommand, RobotCommand};
pub use benchmark::{MotionBenchmarkScenario, MotionKpi, summarize_commands};
pub use environment::Environment;
pub use path_planner::FastPathPlanner;
pub use pid::PIDController;

use crate::world::{World, RobotState};
use glam::Vec2;
use std::collections::HashMap;
use std::sync::Mutex;

const CONTROL_DT: f64 = 0.016; // ~60 Hz
const MAX_LINEAR_SPEED: f64 = 1.8; // m/s
const MIN_LINEAR_SPEED: f64 = 0.08; // m/s
const ARRIVAL_THRESHOLD: f32 = 0.06; // m
const BRAKE_DISTANCE: f32 = 0.50; // m
const MAX_ANGULAR_SPEED: f64 = 3.0; // rad/s

/// Módulo principal de control de movimiento
pub struct Motion {
    path_planner: FastPathPlanner,
    pid_x_by_robot: Mutex<HashMap<(i32, i32), PIDController>>,
    pid_y_by_robot: Mutex<HashMap<(i32, i32), PIDController>>,
    pid_theta_by_robot: Mutex<HashMap<(i32, i32), PIDController>>,
}

impl Motion {
    /// Crea un nuevo sistema de movimiento
    pub fn new() -> Self {
        Self {
            path_planner: FastPathPlanner::new(5), // max_depth = 5
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
    
    /// Movimiento hacia un objetivo usando path planner
    pub fn move_to(
        &self,
        robot_state: &RobotState,
        target: Vec2,
        world: &World,
    ) -> MotionCommand {
        let env = Environment::new(world, robot_state);
        let path = self.path_planner.get_path(robot_state.position, target, &env);
        
        // Calcular velocidad hacia el siguiente punto del path
        if path.len() > 1 {
            let next_point = path[1];
            let diff = next_point - robot_state.position;
            let distance = diff.length();
            if !distance.is_finite() || distance <= f32::EPSILON {
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
            let speed = MAX_LINEAR_SPEED;
            
            MotionCommand {
                id: robot_state.id,
                team: robot_state.team,
                vx: direction.x as f64 * speed,
                vy: direction.y as f64 * speed,
                omega: 0.0,
                orientation: robot_state.orientation,
            }
        } else {
            // Ya está en el objetivo o path inválido
            MotionCommand {
                id: robot_state.id,
                team: robot_state.team,
                vx: 0.0,
                vy: 0.0,
                omega: 0.0,
                orientation: robot_state.orientation,
            }
        }
    }
    
    /// Movimiento directo sin evasión de obstáculos
    pub fn move_direct(
        &self,
        robot_state: &RobotState,
        target: Vec2,
    ) -> MotionCommand {
        let diff = target - robot_state.position;
        let distance = diff.length();
        
        // Si está muy cerca del objetivo, detenerse
        if !distance.is_finite() || distance < ARRIVAL_THRESHOLD {
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
        let normalized = (distance / BRAKE_DISTANCE).clamp(0.0, 1.0);
        let speed = (MIN_LINEAR_SPEED as f32)
            + normalized * ((MAX_LINEAR_SPEED - MIN_LINEAR_SPEED) as f32);
        
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
        let max_speed = MAX_LINEAR_SPEED;
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
        let direction = (target - robot_state.position).normalize();
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
            let mut pid_theta_map = self.pid_theta_by_robot.lock().expect("pid_theta lock poisoned");
            let pid = pid_theta_map
                .entry(key)
                .or_insert_with(|| PIDController::new(kp, ki, kd));
            pid.set_gains(kp, ki, kd);
            pid.compute(error, CONTROL_DT)
        };
        
        // Limitar velocidad angular máxima
        let max_omega = MAX_ANGULAR_SPEED;
        let omega_limited = omega.max(-max_omega).min(max_omega);
        
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
    pub fn motion_with_orientation(
        &self,
        robot_state: &RobotState,
        target: Vec2,
        target_angle: f64,
        world: &World,
        kp_x: f64,
        ki_x: f64,
        kp_y: f64,
        ki_y: f64,
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
        assert!((normalized_neg_pi - (-pi)).abs() < 1e-10 || (normalized_neg_pi - pi).abs() < 1e-10);
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
}
