use glam::Vec2;
use serde::{Serialize, Serializer};

/// Estado del balón en el campo
#[derive(Debug, Clone)]
pub struct BallState {
    pub position: Vec2,
    pub velocity: Vec2,
}

// Implementación manual de Serialize para BallState
impl Serialize for BallState {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        use serde::ser::SerializeStruct;
        let mut state = serializer.serialize_struct("BallState", 2)?;
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
        state.end()
    }
}

impl BallState {
    /// Crea un nuevo estado del balón
    pub fn new() -> Self {
        Self {
            position: Vec2::ZERO,
            velocity: Vec2::ZERO,
        }
    }

    /// Actualiza el estado del balón
    pub fn update(&mut self, position: Vec2, velocity: Vec2) {
        self.position = position;
        self.velocity = velocity;
    }

    /// Verifica si el balón se está moviendo (velocidad mayor que threshold)
    pub fn is_moving(&self, threshold: f32) -> bool {
        self.velocity.length() > threshold
    }
}

impl Default for BallState {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ball_state_new() {
        let state = BallState::new();
        assert_eq!(state.position, Vec2::ZERO);
        assert_eq!(state.velocity, Vec2::ZERO);
    }

    #[test]
    fn test_ball_state_update() {
        let mut state = BallState::new();
        let pos = Vec2::new(0.5, 0.3);
        let vel = Vec2::new(0.1, 0.05);

        state.update(pos, vel);

        assert_eq!(state.position, pos);
        assert_eq!(state.velocity, vel);
    }

    #[test]
    fn test_ball_state_is_moving() {
        let mut state = BallState::new();
        assert!(!state.is_moving(0.01));

        state.update(Vec2::ZERO, Vec2::new(0.1, 0.0));
        assert!(state.is_moving(0.01));
        assert!(!state.is_moving(0.2));
    }
}
