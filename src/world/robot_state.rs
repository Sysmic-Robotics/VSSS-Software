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

    /// Si el último dato tiene más de `umbral_s` segundos, con decimales: en segundos
    /// enteros, un umbral de 2 s se volvía 3 s.
    pub fn is_inactive(&self, umbral_s: f64) -> bool {
        self.last_update
            .elapsed()
            .is_ok_and(|elapsed| elapsed.as_secs_f64() > umbral_s)
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

    #[test]
    fn inactivity_compares_seconds_with_decimals() {
        let mut state = RobotState::new(1, 0);
        state.last_update = SystemTime::now() - std::time::Duration::from_secs_f64(2.5);
        assert!(state.is_inactive(2.0), "2.5 s > 2.0 s (en segundos enteros daba 2, no > 2)");
        state.last_update = SystemTime::now() - std::time::Duration::from_secs_f64(1.5);
        assert!(!state.is_inactive(2.0));
    }
}
