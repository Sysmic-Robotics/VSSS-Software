use glam::Vec2;

/// Comando de movimiento para un robot
#[derive(Debug, Clone)]
pub struct MotionCommand {
    pub id: i32,
    pub team: i32,  // 0 = azul, 1 = amarillo
    pub vx: f64,    // Velocidad global en X (m/s)
    pub vy: f64,    // Velocidad global en Y (m/s)
    pub omega: f64, // Velocidad angular (rad/s)
    pub orientation: f64, // Orientación del robot (rad) - necesaria para conversión a coordenadas locales
}

/// Comando de kicker para un robot
#[derive(Debug, Clone)]
pub struct KickerCommand {
    pub id: i32,
    pub team: i32,
    pub kick_x: bool,   // Kick en X
    pub kick_z: bool,   // Kick en Z
    pub dribbler: f64,  // Velocidad del dribbler (0.0 - 10.0)
}

/// Comando completo para un robot (movimiento + kicker)
#[derive(Debug, Clone)]
pub struct RobotCommand {
    pub id: i32,
    pub team: i32,
    pub motion: MotionCommand,
    pub kicker: KickerCommand,
}
