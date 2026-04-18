use crate::world::World;

// ── Constantes de normalización ───────────────────────────────────────────────
// Estas constantes son el CONTRATO entre el engine Rust y el trainer Python.
// Si cambian aquí, deben cambiar en el script de entrenamiento.

/// Mitad del campo en X (m). Posiciones x se normalizan dividiendo por esto.
pub const FIELD_HALF_X: f32 = 0.75;
/// Mitad del campo en Y (m). Posiciones y se normalizan dividiendo por esto.
pub const FIELD_HALF_Y: f32 = 0.65;
/// Velocidad máxima de normalización (m/s). Pequeño headroom sobre MAX_LINEAR_SPEED=1.2.
const MAX_VEL_NORM: f32 = 1.5;
/// Velocidad angular máxima de normalización (rad/s).
const MAX_OMEGA_NORM: f32 = std::f32::consts::PI;

// ── Structs de observación ────────────────────────────────────────────────────

/// Estado normalizado del balón.
/// Todos los campos en [-1, 1] aprox.
#[derive(Debug, Clone, Default)]
pub struct BallObs {
    pub x: f32,  // position.x / FIELD_HALF_X
    pub y: f32,  // position.y / FIELD_HALF_Y
    pub vx: f32, // velocity.x / MAX_VEL_NORM
    pub vy: f32, // velocity.y / MAX_VEL_NORM
}

/// Estado normalizado de un robot.
/// La orientación se codifica como (sin θ, cos θ) para evitar la discontinuidad en ±π
/// que rompe los gradientes de redes neuronales.
#[derive(Debug, Clone, Default)]
pub struct RobotObs {
    pub x: f32,         // position.x / FIELD_HALF_X
    pub y: f32,         // position.y / FIELD_HALF_Y
    pub vx: f32,        // velocity.x / MAX_VEL_NORM
    pub vy: f32,        // velocity.y / MAX_VEL_NORM
    pub sin_theta: f32, // sin(orientation) ∈ [-1, 1]
    pub cos_theta: f32, // cos(orientation) ∈ [-1, 1]
    pub omega: f32,     // angular_velocity / MAX_OMEGA_NORM
    pub active: f32,    // 1.0 si activo y visible, 0.0 si inactivo o no detectado
}

/// Observación completa del estado del juego para un tick.
///
/// Siempre contiene exactamente 3 entradas en `own_robots` y `opp_robots`,
/// una por ID de robot (0, 1, 2). Robots inactivos o no detectados tienen
/// todos los campos en 0.0 y `active = 0.0`.
///
/// El tamaño de `to_flat_vec()` es siempre `FLAT_SIZE = 52` floats.
#[derive(Debug, Clone)]
pub struct Observation {
    pub ball: BallObs,
    /// Robots propios ordenados por ID (0, 1, 2). Siempre 3 entradas.
    pub own_robots: Vec<RobotObs>,
    /// Robots rivales ordenados por ID (0, 1, 2). Siempre 3 entradas.
    pub opp_robots: Vec<RobotObs>,
    /// Equipo propio: 0 = azul, 1 = amarillo.
    pub own_team: i32,
}

impl Observation {
    /// Número de floats retornados por `to_flat_vec()`.
    /// Contrato fijo: cambiar esto rompe el modelo RL entrenado.
    ///
    /// Layout:
    /// ```text
    /// [0..3]   ball:         x, y, vx, vy                    (4 floats)
    /// [4..11]  own_robots[0]: x,y,vx,vy,sin,cos,omega,active (8 floats)
    /// [12..19] own_robots[1]: ...                             (8 floats)
    /// [20..27] own_robots[2]: ...                             (8 floats)
    /// [28..35] opp_robots[0]: ...                             (8 floats)
    /// [36..43] opp_robots[1]: ...                             (8 floats)
    /// [44..51] opp_robots[2]: ...                             (8 floats)
    /// Total: 4 + 8*6 = 52 floats
    /// ```
    pub const FLAT_SIZE: usize = 4 + 8 * 6;

    /// Construye una observación normalizada desde el estado actual del mundo.
    ///
    /// `own_team`: 0 = azul (blue), 1 = amarillo (yellow).
    pub fn from_world(world: &World, own_team: i32) -> Self {
        let opp_team = 1 - own_team;

        let ball = {
            let b = world.get_ball_state();
            BallObs {
                x: b.position.x / FIELD_HALF_X,
                y: b.position.y / FIELD_HALF_Y,
                vx: b.velocity.x / MAX_VEL_NORM,
                vy: b.velocity.y / MAX_VEL_NORM,
            }
        };

        let own_robots = Self::build_robot_obs(world, own_team);
        let opp_robots = Self::build_robot_obs(world, opp_team);

        Self {
            ball,
            own_robots,
            opp_robots,
            own_team,
        }
    }

    /// Serializa la observación a un Vec<f32> plano para pasarle al modelo.
    /// El layout es fijo (ver `FLAT_SIZE`).
    pub fn to_flat_vec(&self) -> Vec<f32> {
        let mut v = Vec::with_capacity(Self::FLAT_SIZE);
        v.push(self.ball.x);
        v.push(self.ball.y);
        v.push(self.ball.vx);
        v.push(self.ball.vy);
        for r in &self.own_robots {
            Self::push_robot_obs(&mut v, r);
        }
        for r in &self.opp_robots {
            Self::push_robot_obs(&mut v, r);
        }
        debug_assert_eq!(v.len(), Self::FLAT_SIZE, "to_flat_vec tamaño incorrecto");
        v
    }

    // ── helpers privados ──────────────────────────────────────────────────────

    /// Construye un Vec de 3 RobotObs (IDs 0, 1, 2) para un equipo.
    /// Robots inactivos o no encontrados → RobotObs con active = 0.0, resto = 0.0.
    fn build_robot_obs(world: &World, team: i32) -> Vec<RobotObs> {
        (0..3)
            .map(|id| {
                match world.get_robot_state(id, team) {
                    Some(r) if r.active => RobotObs {
                        x: r.position.x / FIELD_HALF_X,
                        y: r.position.y / FIELD_HALF_Y,
                        vx: r.velocity.x / MAX_VEL_NORM,
                        vy: r.velocity.y / MAX_VEL_NORM,
                        sin_theta: r.orientation.sin() as f32,
                        cos_theta: r.orientation.cos() as f32,
                        omega: r.angular_velocity as f32 / MAX_OMEGA_NORM,
                        active: 1.0,
                    },
                    _ => RobotObs::default(), // active = 0.0, todo en 0.0
                }
            })
            .collect()
    }

    fn push_robot_obs(v: &mut Vec<f32>, r: &RobotObs) {
        v.push(r.x);
        v.push(r.y);
        v.push(r.vx);
        v.push(r.vy);
        v.push(r.sin_theta);
        v.push(r.cos_theta);
        v.push(r.omega);
        v.push(r.active);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::World;

    #[test]
    fn test_observation_flat_size() {
        let world = World::new(3, 3);
        let obs = Observation::from_world(&world, 0);
        assert_eq!(
            obs.to_flat_vec().len(),
            Observation::FLAT_SIZE,
            "to_flat_vec debe tener exactamente {} floats",
            Observation::FLAT_SIZE
        );
    }

    #[test]
    fn test_observation_inactive_robots_are_zero() {
        let world = World::new(3, 3);
        let obs = Observation::from_world(&world, 0);
        // Ningún robot actualizado → todos inactivos → active = 0.0
        for r in &obs.own_robots {
            assert_eq!(r.active, 0.0, "robot inactivo debe tener active=0.0");
        }
    }

    #[test]
    fn test_observation_active_robot_normalized() {
        let mut world = World::new(3, 3);
        // Robot en la esquina del campo (posición máxima)
        world.update_robot(
            0,
            0,
            glam::Vec2::new(0.75, 0.65),
            0.0,
            glam::Vec2::ZERO,
            0.0,
        );
        let obs = Observation::from_world(&world, 0);
        let r = &obs.own_robots[0];
        assert_eq!(r.active, 1.0);
        assert!(
            (r.x - 1.0).abs() < 0.01,
            "x debería ser ~1.0 en el borde del campo"
        );
        assert!(
            (r.y - 1.0).abs() < 0.01,
            "y debería ser ~1.0 en el borde del campo"
        );
    }

    #[test]
    fn test_observation_ball_normalized() {
        let mut world = World::new(3, 3);
        world.update_ball(glam::Vec2::new(0.375, 0.325), glam::Vec2::ZERO); // mitad del half
        let obs = Observation::from_world(&world, 0);
        assert!((obs.ball.x - 0.5).abs() < 0.01);
        assert!((obs.ball.y - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_observation_own_team_perspective() {
        let mut world = World::new(3, 3);
        world.update_robot(0, 0, glam::Vec2::new(0.3, 0.0), 0.0, glam::Vec2::ZERO, 0.0);
        world.update_robot(0, 1, glam::Vec2::new(-0.3, 0.0), 0.0, glam::Vec2::ZERO, 0.0);

        let obs_blue = Observation::from_world(&world, 0);
        let obs_yellow = Observation::from_world(&world, 1);

        // Para azul: own_robots[0] es el robot azul en x=0.3 → x_norm ≈ 0.4
        assert!(obs_blue.own_robots[0].active > 0.5);
        assert!((obs_blue.own_robots[0].x - 0.4).abs() < 0.01);

        // Para amarillo: own_robots[0] es el robot amarillo en x=-0.3 → x_norm ≈ -0.4
        assert!(obs_yellow.own_robots[0].active > 0.5);
        assert!((obs_yellow.own_robots[0].x - (-0.4)).abs() < 0.01);
    }
}
