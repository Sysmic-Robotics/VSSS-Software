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
const MAX_LINEAR_SPEED: f64 = 1.2; // m/s
const MIN_LINEAR_SPEED: f64 = 0.06; // m/s
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
            // Perfil de frenado proporcional igual que move_direct(): ágil lejos, suave al llegar.
            let dist_to_next = distance; // distancia al próximo waypoint
            let dist_to_goal = (target - robot_state.position).length();
            let brake_ref = dist_to_next.min(dist_to_goal);
            let normalized = (brake_ref / BRAKE_DISTANCE).clamp(0.0, 1.0);
            let speed = (MIN_LINEAR_SPEED as f32)
                + normalized * ((MAX_LINEAR_SPEED - MIN_LINEAR_SPEED) as f32);

            MotionCommand {
                id: robot_state.id,
                team: robot_state.team,
                vx: direction.x as f64 * speed as f64,
                vy: direction.y as f64 * speed as f64,
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

    /// Simulación headless completa: approach + pivote + empuje de pelota.
    /// Modela robot diferencial real (v = vx·cos θ + vy·sin θ) y física simple de pelota.
    /// Ejecutar con: cargo test test_robot_motion_simulation -- --nocapture
    #[test]
    fn test_robot_motion_simulation() {
        // ── Parámetros (idénticos a main.rs) ────────────────────────────────
        let goal_pos       = Vec2::new(0.75_f32, 0.0_f32);
        let ball_start     = Vec2::new(0.1_f32,  0.2_f32);
        let start_pos      = Vec2::new(-0.3_f32, 0.15_f32);
        let start_orient   = std::f64::consts::PI / 3.0;  // 60°
        let staging_offset = 0.16_f32;
        let staging_tol    = 0.08_f32;
        let kp = 1.8_f64; let ki = 0.0_f64; let kd = 0.05_f64;
        let n_ticks        = 480_usize; // 8s a 60Hz

        // Física de pelota (modelo simple): el robot empuja la pelota al contacto.
        let contact_dist   = 0.075_f32; // radio robot (~4cm) + radio pelota (~2.5cm) + margen
        let ball_friction  = 0.92_f32;  // decaimiento de velocidad por tick (~1 - 0.08)

        // ── Setup ────────────────────────────────────────────────────────────
        let motion = Motion::new();
        let mut pos        = start_pos;
        let mut orient     = start_orient;
        let mut ball_pos   = ball_start;
        let mut ball_vel   = Vec2::ZERO;
        let mut in_captura = false;
        let mut staged_tick: Option<usize>  = None;
        let mut contact_tick: Option<usize> = None;
        let mut ball_moved_m = 0.0_f32;

        println!("\n=== SIMULACIÓN COMPLETA: APPROACH + PIVOTE + EMPUJE (8s @ 60Hz) ===");
        println!("robot=({:.2},{:.2}) orient={:.0}°  ball=({:.2},{:.2})  goal=({:.2},{:.2})",
            start_pos.x, start_pos.y, start_orient.to_degrees(),
            ball_start.x, ball_start.y, goal_pos.x, goal_pos.y);
        println!("{:>6}  {:>14}  {:>14}  {:>6}  {:>6}  {:>6}  {:>8}",
            "t(s)", "robot(x,y)", "ball(x,y)", "v m/s", "ω r/s", "dstg", "fase");

        for tick in 0..n_ticks {
            // ── Recalcular staging en función de la posición actual de la pelota ──
            let ball_to_goal  = (goal_pos - ball_pos).normalize_or_zero();
            let staging_point = ball_pos - ball_to_goal * staging_offset;

            let mut robot = RobotState::new(0, 0);
            robot.position    = pos;
            robot.orientation = orient;

            let dist_staging     = (staging_point - pos).length();
            let dot_robot_ball   = (pos - ball_pos).dot(goal_pos - ball_pos);
            let behind_ball      = dot_robot_ball < 0.02;   // pequeño margen positivo
            let in_front_of_ball = dot_robot_ball > 0.05;   // 5cm delante → salir de CAPTURA

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

            let heading_goal_angle = {
                let d = goal_pos - pos;
                f64::atan2(d.y as f64, d.x as f64)
            };

            // ── Comandos: en CAPTURA siempre move_direct (el pivote) ─────────
            // En APPROACH: move_to con path planning (pelota es obstáculo → rodea).
            // En CAPTURA:  move_direct ignora obstáculos y empuja la pelota directo.
            //
            // face_to(ball_pos) en CAPTURA: staging está sobre la línea ball-goal,
            // así que face_to(ball) ≈ face_to_angle(goal) desde staging.
            // Además garantiza que el robot apunte a la pelota (v > 0)
            // y siga al ball cuando se mueve (pivote real).
            let world = World::new(3, 3);
            // En CAPTURA apuntamos 12cm PASADO la pelota (hacia el goal)
            // para que move_direct no se detenga antes de empujar.
            let push_target = ball_pos + ball_to_goal * 0.12;
            let mut cmd = if in_captura {
                motion.move_direct(&robot, push_target)
            } else {
                motion.move_to(&robot, staging_point, &world)
            };

            // face_to(ball_pos): sigue a la pelota mientras empuja (pivote real).
            let face_cmd = if in_captura {
                motion.face_to(&robot, ball_pos, kp, ki, kd)
            } else {
                motion.face_to(&robot, staging_point, kp, ki, kd)
            };
            cmd.omega = face_cmd.omega;

            // ── Física del robot (diferencial) ───────────────────────────────
            let v = cmd.vx * orient.cos() + cmd.vy * orient.sin();
            pos.x  += (v * orient.cos() * CONTROL_DT) as f32;
            pos.y  += (v * orient.sin() * CONTROL_DT) as f32;
            orient  = Motion::normalize_angle(orient + cmd.omega * CONTROL_DT);

            // ── Física de la pelota ───────────────────────────────────────────
            // Cuando el robot toca la pelota le transfiere impulso en su dirección forward.
            let dist_ball = (ball_pos - pos).length();
            if dist_ball < contact_dist {
                if contact_tick.is_none() { contact_tick = Some(tick); }
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
            if tick % 30 == 0 {
                println!("{:>6.2}  ({:>5.3},{:>5.3})  ({:>5.3},{:>5.3})  {:>6.3}  {:>6.3}  {:>6.3}  {}",
                    tick as f64 * CONTROL_DT,
                    pos.x, pos.y, ball_pos.x, ball_pos.y,
                    v, cmd.omega, dist_staging, fase);
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
        println!("  Staging alcanzado: {}",
            staged_tick.map(|t| format!("SÍ en t={:.2}s", t as f64 * CONTROL_DT))
                       .unwrap_or("NO".to_string()));
        println!("  Primer contacto:   {}",
            contact_tick.map(|t| format!("SÍ en t={:.2}s", t as f64 * CONTROL_DT))
                        .unwrap_or("NO".to_string()));
        println!("  Pelota se movió:   {:.3}m total", ball_moved_m);
        println!("  Pelota final:      ({:.3},{:.3})  dist_goal={:.3}m",
            ball_pos.x, ball_pos.y, ball_dist_to_goal);
        println!("  Progreso al goal:  {:.1}%", ball_progress);
        println!("  Robot final:       ({:.3},{:.3}) orient={:.1}°\n",
            pos.x, pos.y, orient.to_degrees());

        assert!(staged_tick.is_some(), "Robot nunca llegó al staging point");
        assert!(contact_tick.is_some(), "Robot nunca tocó la pelota");
        assert!(ball_moved_m > 0.01, "Pelota no se movió (moved={:.4}m)", ball_moved_m);
        let staged_s = staged_tick.unwrap() as f64 * CONTROL_DT;
        assert!(staged_s < 4.0, "Approach tardó demasiado: {:.2}s", staged_s);
    }
}
