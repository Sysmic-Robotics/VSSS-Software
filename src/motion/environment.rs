use crate::world::{World, RobotState};
use glam::Vec2;

/// Environment representa el entorno del campo con obstáculos
/// Implementación EXACTA del código C++ environment.cpp
pub struct Environment {
    robots: Vec<Vec2>,      // Posiciones de otros robots
    ball_position: Vec2,    // Posición del balón
}

impl Environment {
    /// Construye Environment desde World, excluyendo el robot self
    /// EXACTO al código C++ línea 5-21
    pub fn new(world: &World, self_robot: &RobotState) -> Self {
        let mut robots = Vec::new();
        let self_id = self_robot.id;
        
        // Recopilar posiciones de todos los otros robots activos
        // EXACTO al código C++ línea 9-17
        for id in 0..12 {
            if id == self_id {
                continue;
            }
            
            // Robot azul
            if let Some(r_blue) = world.get_robot_state(id, 0) {
                if r_blue.active {
                    robots.push(r_blue.position);
                }
            }
            
            // Robot amarillo
            if let Some(r_yellow) = world.get_robot_state(id, 1) {
                if r_yellow.active {
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
        if point.x < -4.5 || point.x > 4.5 || point.y < -3.0 || point.y > 3.0 {
            return true;
        }
        
        // Yellow goalie box (x: 3.5 to 4.5, y: -1 to 1) - EXACTO línea 28-31
        if point.x >= 3.5 && point.x <= 4.5 &&
           point.y >= -1.0 && point.y <= 1.0 {
            return true;
        }
        
        // Blue goalie box (x: -4.5 to -3.5, y: -1 to 1) - EXACTO línea 33-36
        if point.x >= -4.5 && point.x <= -3.5 &&
           point.y >= -1.0 && point.y <= 1.0 {
            return true;
        }
        
        // Robot collision check - EXACTO línea 38-42
        for robot in &self.robots {
            if (point - *robot).length() <= 0.2 {  // Robot radius 0.2m
                return true;
            }
        }
        
        // Ball collision check - EXACTO línea 44-46
        if (self.ball_position - point).length() <= 0.1 {  // Ball radius 0.1m
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
        
        // Dentro de límites (pero verificar que no colisiona con el balón)
        // El balón está en (1.0, 1.0), así que (0.0, 0.0) debería estar bien
        assert!(!env.collides(Vec2::new(0.0, 0.0)));
        assert!(!env.collides(Vec2::new(4.0, 2.0)));
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
