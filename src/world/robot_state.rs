use glam::Vec2;
use serde::{Serialize, Serializer};
use std::time::SystemTime;

/// Estado de un robot en el campo
#[derive(Debug, Clone)]
pub struct RobotState {
    pub id: i32,
    pub team: i32, // 0 = azul, 1 = amarillo
    pub position: Vec2,
    pub velocity: Vec2,
    pub orientation: f64,
    pub angular_velocity: f64,
    pub active: bool,
    pub last_update: SystemTime,
}

// Implementación manual de Serialize para RobotState
impl Serialize for RobotState {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        use serde::ser::SerializeStruct;
        let mut state = serializer.serialize_struct("RobotState", 7)?;
        state.serialize_field("id", &self.id)?;
        state.serialize_field("team", &self.team)?;
        state.serialize_field(
            "position",
            &serde_json::json!({
                "x": self.position.x,
                "y": self.position.y
            }),
        )?;
        state.serialize_field(
            "velocity",
            &serde_json::json!({
                "x": self.velocity.x,
                "y": self.velocity.y
            }),
        )?;
        state.serialize_field("orientation", &self.orientation)?;
        state.serialize_field("angular_velocity", &self.angular_velocity)?;
        state.serialize_field("active", &self.active)?;
        state.end()
    }
}

impl RobotState {
    /// Crea un nuevo estado de robot
    pub fn new(id: i32, team: i32) -> Self {
        Self {
            id,
            team,
            position: Vec2::ZERO,
            velocity: Vec2::ZERO,
            orientation: 0.0,
            angular_velocity: 0.0,
            active: true,
            last_update: SystemTime::now(),
        }
    }

    /// Actualiza el estado del robot con nuevos datos
    pub fn update(
        &mut self,
        position: Vec2,
        orientation: f64,
        velocity: Vec2,
        angular_velocity: f64,
    ) {
        self.position = position;
        self.orientation = orientation;
        self.velocity = velocity;
        self.angular_velocity = angular_velocity;
        self.active = true;
        self.last_update = SystemTime::now();
    }

    /// Marca el robot como inactivo
    pub fn mark_inactive(&mut self) {
        self.active = false;
    }

    /// Verifica si el robot está inactivo (última actualización hace más de threshold segundos)
    pub fn is_inactive(&self, threshold_seconds: u64) -> bool {
        if let Ok(elapsed) = self.last_update.elapsed() {
            elapsed.as_secs() > threshold_seconds
        } else {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_robot_state_new() {
        let state = RobotState::new(1, 0);
        assert_eq!(state.id, 1);
        assert_eq!(state.team, 0);
        assert!(state.active);
    }

    #[test]
    fn test_robot_state_update() {
        let mut state = RobotState::new(1, 0);
        let pos = Vec2::new(1.5, 2.3);
        let vel = Vec2::new(0.5, -0.2);

        state.update(pos, 1.57, vel, 0.1);

        assert_eq!(state.position, pos);
        assert_eq!(state.velocity, vel);
        assert!((state.orientation - 1.57).abs() < 1e-10);
        assert!((state.angular_velocity - 0.1).abs() < 1e-10);
        assert!(state.active);
    }

    #[test]
    fn test_robot_state_inactive() {
        let mut state = RobotState::new(1, 0);
        state.mark_inactive();
        assert!(!state.active);
    }
}
