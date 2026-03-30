use glam::Vec2;

/// Decisión del Coach para un robot en un tick.
/// La capa de ejecución (CoachPlay) traduce esto a MotionCommands via motion primitives.
#[derive(Debug, Clone)]
pub struct RobotTarget {
    /// ID del robot al que aplica este target (0, 1 o 2 en VSS 3v3).
    pub robot_id: i32,

    /// Posición a la que debe navegar el robot (coordenadas de campo, NO normalizadas).
    /// El motion layer (UVF + PID) maneja la evasión de obstáculos.
    pub position: Vec2,

    /// Punto hacia el que debe orientarse el robot.
    /// None → mirar en la dirección de movimiento (default para la mayoría de roles).
    /// Some(p) → siempre mirar hacia p (ej: portero siempre mira la pelota).
    pub face_target: Option<Vec2>,
}
