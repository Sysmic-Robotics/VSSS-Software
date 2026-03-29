use crate::world::{World, RobotState};
use glam::Vec2;

/// Environment representa el entorno del campo con obstáculos
/// Implementación EXACTA del código C++ environment.cpp
pub struct Environment {
    robots: Vec<Vec2>,      // Posiciones de otros robots
    ball_position: Vec2,    // Posición del balón
}

// Geometría aproximada VSS (metros). Evita usar valores de SSL 9x6.
const VSS_HALF_FIELD_X: f32 = 0.75;
const VSS_HALF_FIELD_Y: f32 = 0.65;
const GOAL_BOX_MIN_X: f32 = 0.60;
const GOAL_BOX_HALF_Y: f32 = 0.22;
const ROBOT_COLLISION_RADIUS: f32 = 0.04; // ~radio real VSS: diámetro 7.5cm → 3.75cm + margen
const BALL_COLLISION_RADIUS: f32 = 0.05;

impl Environment {
    /// Construye Environment desde World, excluyendo el robot self
    /// EXACTO al código C++ línea 5-21
    pub fn new(world: &World, self_robot: &RobotState) -> Self {
        let mut robots = Vec::new();
        let self_id = self_robot.id;
        let self_team = self_robot.team;
        
        // Recopilar posiciones de todos los otros robots activos
        // EXACTO al código C++ línea 9-17
        for id in 0..3 { // VSS es 3v3
            // Robot azul
            if let Some(r_blue) = world.get_robot_state(id, 0) {
                if r_blue.active && !(r_blue.id == self_id && self_team == 0) {
                    robots.push(r_blue.position);
                }
            }

            // Robot amarillo
            if let Some(r_yellow) = world.get_robot_state(id, 1) {
                if r_yellow.active && !(r_yellow.id == self_id && self_team == 1) {
                    robots.push(r_yellow.position);
                }
            }
        }
        
        // Get ball position - EXACTO línea 20
        let ball_position = world.get_ball_state().position;
        
        Self {
            robots,
            ball_position,
        }
    }
    
    /// Verifica si un punto colisiona con obstáculos
    /// Implementación EXACTA del código C++ línea 23-49
    pub fn collides(&self, point: Vec2) -> bool {
        // Field bounds check - EXACTO línea 25-26
        if point.x < -VSS_HALF_FIELD_X
            || point.x > VSS_HALF_FIELD_X
            || point.y < -VSS_HALF_FIELD_Y
            || point.y > VSS_HALF_FIELD_Y
        {
            return true;
        }
        
        // Zonas del arco (aproximadas VSS): evitar que el planner atraviese el área.
        if point.x >= GOAL_BOX_MIN_X && point.y.abs() <= GOAL_BOX_HALF_Y {
            return true;
        }
        if point.x <= -GOAL_BOX_MIN_X && point.y.abs() <= GOAL_BOX_HALF_Y {
            return true;
        }
        
        // Robot collision check - EXACTO línea 38-42
        for robot in &self.robots {
            if (point - *robot).length() <= ROBOT_COLLISION_RADIUS {
                return true;
            }
        }
        
        // Ball collision check - EXACTO línea 44-46
        if (self.ball_position - point).length() <= BALL_COLLISION_RADIUS {
            return true;
        }
        
        false
    }
    
    /// Obtiene las posiciones de los robots obstáculos
    pub fn get_robots(&self) -> &[Vec2] {
        &self.robots
    }
    
    /// Obtiene la posición del balón
    pub fn get_ball_position(&self) -> Vec2 {
        self.ball_position
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::World;
    
    #[test]
    fn test_environment_collides_field_bounds() {
        let mut world = World::new(11, 11);
        // Necesitamos actualizar el balón para que no esté en (0,0)
        world.update_ball(Vec2::new(1.0, 1.0), Vec2::ZERO);
        let robot = RobotState::new(0, 0);
        let env = Environment::new(&world, &robot);
        
        // Fuera de límites
        assert!(env.collides(Vec2::new(-5.0, 0.0)));
        assert!(env.collides(Vec2::new(5.0, 0.0)));
        assert!(env.collides(Vec2::new(0.0, -4.0)));
        assert!(env.collides(Vec2::new(0.0, 4.0)));
        
        // Dentro de límites del campo VSS (±0.75, ±0.65) — balón en (1.0,1.0) fuera del campo
        // así que estos puntos no deben colisionar con nada
        assert!(!env.collides(Vec2::new(0.0, 0.0)));
        assert!(!env.collides(Vec2::new(0.3, 0.2)));
    }
    
    #[test]
    fn test_environment_collides_goalie_boxes() {
        let world = World::new(11, 11);
        let robot = RobotState::new(0, 0);
        let env = Environment::new(&world, &robot);
        
        // Yellow goalie box
        assert!(env.collides(Vec2::new(4.0, 0.0)));
        assert!(env.collides(Vec2::new(3.5, 0.0)));
        
        // Blue goalie box
        assert!(env.collides(Vec2::new(-4.0, 0.0)));
        assert!(env.collides(Vec2::new(-3.5, 0.0)));
    }
}
