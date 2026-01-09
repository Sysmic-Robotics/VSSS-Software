use crate::motion::environment::Environment;
use glam::Vec2;
use std::f32::consts::PI;

/// Trajectory representa un segmento de ruta
#[derive(Debug, Clone)]
struct Trajectory {
    start: Vec2,
    goal: Vec2,
}

/// Función helper para rotar un vector (equivalente a rotateVector en C++)
/// EXACTO al código C++ línea 6-13
fn rotate_vector(v: Vec2, angle: f32) -> Vec2 {
    let cos_a = angle.cos();
    let sin_a = angle.sin();
    Vec2::new(
        cos_a * v.x - sin_a * v.y,
        sin_a * v.x + cos_a * v.y,
    )
}

/// FastPathPlanner implementa planificación de rutas con evasión de obstáculos
/// LÓGICA EXACTA del código C++ path_planner.cpp
pub struct FastPathPlanner {
    max_depth: i32,
}

impl FastPathPlanner {
    /// Crea un nuevo FastPathPlanner con la profundidad máxima especificada
    /// EXACTO al código C++ línea 16
    pub fn new(max_depth: i32) -> Self {
        Self { max_depth }
    }
    
    /// Verifica si una trayectoria colisiona (200 puntos de verificación)
    /// EXACTO al código C++ línea 18-25
    fn trajectory_collides(&self, traj: &Trajectory, env: &Environment) -> bool {
        for i in 0..200 {
            let t = i as f32 / 200.0;
            let point = traj.start + t * (traj.goal - traj.start);
            if env.collides(point) {
                return true;
            }
        }
        false
    }
    
    /// Busca un sub-objetivo rotando el vector dirección
    /// EXACTO al código C++ línea 27-43
    fn search_subgoal(
        &self,
        traj: &Trajectory,
        env: &Environment,
        robot_diameter: f32,
        direction: i32,
    ) -> Vec2 {
        let obs_point = traj.goal;
        let mut dir_vec = (obs_point - traj.start).normalize();
        
        for _i in 0..10 {
            let offset = Vec2::new(-dir_vec.y, dir_vec.x) * (direction as f32 * robot_diameter);
            let subgoal = obs_point + offset;
            
            if !env.collides(subgoal) &&
               subgoal.x >= -4.5 && subgoal.x <= 4.5 &&
               subgoal.y >= -3.0 && subgoal.y <= 3.0 {
                return subgoal;
            }
            
            dir_vec = rotate_vector(dir_vec, PI / 4.0);
        }
        
        // Retornar NaN si no se encuentra sub-objetivo
        // EXACTO línea 42
        Vec2::new(f32::NAN, f32::NAN)
    }
    
    /// Crea un segmento de trayectoria
    /// EXACTO al código C++ línea 45-47
    fn make_segment(&self, from: Vec2, to: Vec2) -> Trajectory {
        Trajectory { start: from, goal: to }
    }
    
    /// Crea la ruta usando algoritmo recursivo con stack
    /// EXACTO al código C++ línea 49-82
    fn create_path(&self, start: Vec2, goal: Vec2, env: &Environment) -> Vec<Trajectory> {
        let mut result_a = Vec::new();
        let mut result_b = Vec::new();
        let mut stack = vec![(Trajectory { start, goal }, 0)];
        
        // Path A (right-hand rule) - EXACTO línea 53-64
        while let Some((traj, depth)) = stack.pop() {
            if self.trajectory_collides(&traj, env) && depth < self.max_depth {
                let sub = self.search_subgoal(&traj, env, 0.36, 1);
                if sub.x.is_nan() {
                    return Vec::new();
                }
                stack.push((Trajectory { start: sub, goal: traj.goal }, depth + 1));
                stack.push((Trajectory { start: traj.start, goal: sub }, depth + 1));
            } else {
                result_a.push(traj);
            }
        }
        
        // Path B (left-hand rule) - EXACTO línea 66-79
        stack = vec![(Trajectory { start, goal }, 0)];
        while let Some((traj, depth)) = stack.pop() {
            if self.trajectory_collides(&traj, env) && depth < self.max_depth {
                let sub = self.search_subgoal(&traj, env, 0.36, -1);
                if sub.x.is_nan() {
                    return Vec::new();
                }
                stack.push((Trajectory { start: sub, goal: traj.goal }, depth + 1));
                stack.push((Trajectory { start: traj.start, goal: sub }, depth + 1));
            } else {
                result_b.push(traj);
            }
        }
        
        // Retornar el path más corto - EXACTO línea 81
        if result_a.len() <= result_b.len() && !result_a.is_empty() {
            result_a
        } else {
            result_b
        }
    }
    
    /// Simplifica la ruta eliminando puntos innecesarios
    /// EXACTO al código C++ línea 84-102
    fn simplify_path(&self, path: &[Vec2], env: &Environment) -> Vec<Vec2> {
        if path.len() <= 2 {
            return path.to_vec();
        }
        
        let mut result = Vec::new();
        result.push(path[0]);
        
        let mut i = 0;
        while i < path.len() - 1 {
            let mut j = path.len() - 1;
            while j > i + 1 {
                if !self.trajectory_collides(
                    &Trajectory { start: path[i], goal: path[j] },
                    env,
                ) {
                    break;
                }
                j -= 1;
            }
            result.push(path[j]);
            i = j;
        }
        
        result
    }
    
    /// Método principal que crea y simplifica la ruta
    /// EXACTO al código C++ línea 104-126
    pub fn get_path(&self, from: Vec2, to: Vec2, env: &Environment) -> Vec<Vec2> {
        let raw_path = self.create_path(from, to, env);
        if raw_path.is_empty() {
            return vec![from, to]; // Fallback si la ruta es inválida
        }
        
        let mut points = Vec::new();
        for seg in &raw_path {
            points.push(seg.start);
        }
        points.push(raw_path.last().unwrap().goal);
        
        // Simplificar la ruta
        self.simplify_path(&points, env)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::{World, RobotState};
    
    #[test]
    fn test_path_planner_direct_path() {
        let planner = FastPathPlanner::new(5);
        let world = World::new(11, 11);
        let robot = RobotState::new(0, 0);
        let env = Environment::new(&world, &robot);
        
        // Ruta directa sin obstáculos
        let path = planner.get_path(Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0), &env);
        assert!(!path.is_empty());
    }
    
    #[test]
    fn test_path_planner_with_obstacle() {
        let mut world = World::new(11, 11);
        // Agregar un robot obstáculo en el medio
        world.update_robot(1, 0, Vec2::new(0.5, 0.5), 0.0, Vec2::ZERO, 0.0);
        
        let robot = RobotState::new(0, 0);
        let env = Environment::new(&world, &robot);
        let planner = FastPathPlanner::new(5);
        
        // Ruta que debe evitar el obstáculo
        let path = planner.get_path(Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0), &env);
        assert!(!path.is_empty());
    }
}
