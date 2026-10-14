mod ball_state;
mod robot_state;

pub use ball_state::BallState;
pub use robot_state::RobotState;

use glam::Vec2;
use serde_json::{Value, json};
use std::collections::HashMap;

/// Un robot sin datos de visión durante más de esto (s) pasa a inactivo.
pub const INACTIVO_TRAS_S: f64 = 2.0;

/// Modelo del mundo que almacena el estado de todos los robots y el balón
pub struct World {
    blue_robots: HashMap<i32, RobotState>,
    yellow_robots: HashMap<i32, RobotState>,
    ball: BallState,
    blue_team_size: usize,
    yellow_team_size: usize,
}

impl World {
    /// Crea un nuevo mundo con los tamaños de equipo especificados
    pub fn new(blue_team_size: usize, yellow_team_size: usize) -> Self {
        Self {
            blue_robots: HashMap::new(),
            yellow_robots: HashMap::new(),
            ball: BallState::new(),
            blue_team_size,
            yellow_team_size,
        }
    }

    /// Devuelve los tamaños configurados para cada equipo.
    pub fn team_sizes(&self) -> (usize, usize) {
        (self.blue_team_size, self.yellow_team_size)
    }

    /// Obtiene el estado de un robot específico
    pub fn get_robot_state(&self, id: i32, team: i32) -> Option<&RobotState> {
        match team {
            0 => self.blue_robots.get(&id),
            1 => self.yellow_robots.get(&id),
            _ => None,
        }
    }

    /// Hora del último dato de visión de algún robot del equipo `team` (0 azul, 1
    /// amarillo); `None` si nunca se vio ninguno. Sirve para el corte por pérdida de
    /// visión del lazo, con un umbral más corto que el de inactividad.
    pub fn ultimo_dato_de(&self, team: i32) -> Option<std::time::SystemTime> {
        let robots = match team {
            0 => &self.blue_robots,
            1 => &self.yellow_robots,
            _ => return None,
        };
        robots.values().map(|r| r.last_update).max()
    }

    /// Obtiene el estado del balón
    pub fn get_ball_state(&self) -> &BallState {
        &self.ball
    }

    /// Actualiza el estado de un robot
    pub fn update_robot(
        &mut self,
        id: i32,
        team: i32,
        position: Vec2,
        orientation: f64,
        velocity: Vec2,
        omega: f64,
    ) {
        match team {
            0 => {
                let robot = self
                    .blue_robots
                    .entry(id)
                    .or_insert_with(|| RobotState::new(id, team));
                robot.update(position, orientation, velocity, omega);
            }
            1 => {
                let robot = self
                    .yellow_robots
                    .entry(id)
                    .or_insert_with(|| RobotState::new(id, team));
                robot.update(position, orientation, velocity, omega);
            }
            _ => {}
        }
    }

    /// Actualiza el estado del balón
    pub fn update_ball(&mut self, position: Vec2, velocity: Vec2) {
        self.ball.update(position, velocity);
    }

    /// Obtiene todos los robots del equipo azul
    pub fn get_blue_team_state(&self) -> Vec<&RobotState> {
        self.blue_robots.values().collect()
    }

    /// Obtiene todos los robots del equipo amarillo
    pub fn get_yellow_team_state(&self) -> Vec<&RobotState> {
        self.yellow_robots.values().collect()
    }

    /// Obtiene todos los robots activos del equipo azul
    pub fn get_blue_team_active(&self) -> Vec<&RobotState> {
        self.blue_robots.values().filter(|r| r.active).collect()
    }

    /// Obtiene todos los robots activos del equipo amarillo
    pub fn get_yellow_team_active(&self) -> Vec<&RobotState> {
        self.yellow_robots.values().filter(|r| r.active).collect()
    }

    /// Actualiza el estado del mundo, marcando robots inactivos si no han sido actualizados recientemente
    ///
    /// Un robot se marca como inactivo si su última actualización fue hace más de
    /// `INACTIVO_TRAS_S` segundos.
    pub fn update(&mut self) {
        let threshold = INACTIVO_TRAS_S;

        // Marcar robots azules inactivos
        for robot in self.blue_robots.values_mut() {
            if robot.is_inactive(threshold) {
                robot.mark_inactive();
            }
        }

        // Marcar robots amarillos inactivos
        for robot in self.yellow_robots.values_mut() {
            if robot.is_inactive(threshold) {
                robot.mark_inactive();
            }
        }
    }

    /// Limpia robots inactivos del mundo (opcional, para limpieza de memoria)
    pub fn cleanup_inactive_robots(&mut self) {
        self.blue_robots.retain(|_, robot| robot.active);
        self.yellow_robots.retain(|_, robot| robot.active);
    }

    /// Obtiene el número de robots activos por equipo
    pub fn get_active_counts(&self) -> (usize, usize) {
        let blue_active = self.blue_robots.values().filter(|r| r.active).count();
        let yellow_active = self.yellow_robots.values().filter(|r| r.active).count();
        (blue_active, yellow_active)
    }

    /// Serializa el estado del mundo a JSON
    ///
    /// Formato compatible con el código C++:
    /// {
    ///   "robots": [
    ///     {
    ///       "id": 0,
    ///       "team": "blue" | "yellow",
    ///       "position": {"x": 1.5, "y": 0.3},
    ///       "velocity": {"x": 0.5, "y": -0.2},
    ///       "orientation": 1.57,
    ///       "angular_velocity": 0.1,
    ///       "active": true
    ///     }
    ///   ],
    ///   "ball": {
    ///     "position": {"x": 0.0, "y": 0.0},
    ///     "velocity": {"x": 0.1, "y": 0.05}
    ///   }
    /// }
    pub fn to_json(&self) -> Value {
        let mut robots = Vec::new();

        // Agregar robots azules
        for robot in self.blue_robots.values() {
            robots.push(json!({
                "id": robot.id,
                "team": "blue",
                "position": {
                    "x": robot.position.x,
                    "y": robot.position.y
                },
                "velocity": {
                    "x": robot.velocity.x,
                    "y": robot.velocity.y
                },
                "orientation": robot.orientation,
                "angular_velocity": robot.angular_velocity,
                "active": robot.active
            }));
        }

        // Agregar robots amarillos
        for robot in self.yellow_robots.values() {
            robots.push(json!({
                "id": robot.id,
                "team": "yellow",
                "position": {
                    "x": robot.position.x,
                    "y": robot.position.y
                },
                "velocity": {
                    "x": robot.velocity.x,
                    "y": robot.velocity.y
                },
                "orientation": robot.orientation,
                "angular_velocity": robot.angular_velocity,
                "active": robot.active
            }));
        }

        json!({
            "robots": robots,
            "ball": {
                "position": {
                    "x": self.ball.position.x,
                    "y": self.ball.position.y
                },
                "velocity": {
                    "x": self.ball.velocity.x,
                    "y": self.ball.velocity.y
                }
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, SystemTime};

    /// Mundo con los robots propios (azules) 0, 1 y 2, el último dato de cada uno hace
    /// `edades_s`, y un rival visto ahora.
    fn mundo_con_edades(edades_s: [f64; 3]) -> World {
        let mut w = World::new(3, 3);
        for (id, edad) in edades_s.iter().enumerate() {
            w.update_robot(id as i32, 0, Vec2::ZERO, 0.0, Vec2::ZERO, 0.0);
            w.blue_robots.get_mut(&(id as i32)).unwrap().last_update =
                SystemTime::now() - Duration::from_secs_f64(*edad);
        }
        w.update_robot(0, 1, Vec2::ZERO, 0.0, Vec2::ZERO, 0.0);
        w
    }

    #[test]
    fn update_marks_inactive_after_2_0_s_not_3() {
        let mut w = mundo_con_edades([2.5, 1.5, 0.0]);
        w.update();
        assert!(!w.get_robot_state(0, 0).unwrap().active, "2.5 s sin datos: inactivo");
        assert!(w.get_robot_state(1, 0).unwrap().active, "1.5 s: sigue activo");
        assert!(w.get_robot_state(2, 0).unwrap().active);
    }

    #[test]
    fn last_data_of_a_team_is_its_freshest_robot() {
        let w = mundo_con_edades([1.0, 1.0, 0.1]);
        let edad = w.ultimo_dato_de(0).unwrap().elapsed().unwrap().as_secs_f64();
        assert!((0.1..0.2).contains(&edad), "{edad}");
        // Los rivales tienen su propio último dato; un equipo sin robots, ninguno.
        assert!(w.ultimo_dato_de(1).unwrap().elapsed().unwrap() < Duration::from_millis(100));
        assert!(World::new(3, 3).ultimo_dato_de(0).is_none());
    }

    #[test]
    fn test_world_new() {
        let world = World::new(11, 11);
        assert_eq!(world.team_sizes(), (11, 11));
    }

    #[test]
    fn test_world_update_robot() {
        let mut world = World::new(11, 11);
        let pos = Vec2::new(1.5, 2.3);
        let vel = Vec2::new(0.5, -0.2);

        world.update_robot(1, 0, pos, 1.57, vel, 0.1);

        let robot = world.get_robot_state(1, 0).unwrap();
        assert_eq!(robot.id, 1);
        assert_eq!(robot.team, 0);
        assert_eq!(robot.position, pos);
        assert!(robot.active);
    }

    #[test]
    fn test_world_update_ball() {
        let mut world = World::new(11, 11);
        let pos = Vec2::new(0.5, 0.3);
        let vel = Vec2::new(0.1, 0.05);

        world.update_ball(pos, vel);

        let ball = world.get_ball_state();
        assert_eq!(ball.position, pos);
        assert_eq!(ball.velocity, vel);
    }

    #[test]
    fn test_world_multiple_robots() {
        let mut world = World::new(11, 11);

        world.update_robot(1, 0, Vec2::new(1.0, 2.0), 0.0, Vec2::ZERO, 0.0);
        world.update_robot(2, 0, Vec2::new(2.0, 3.0), 0.0, Vec2::ZERO, 0.0);
        world.update_robot(1, 1, Vec2::new(3.0, 4.0), 0.0, Vec2::ZERO, 0.0);

        assert_eq!(world.get_blue_team_state().len(), 2);
        assert_eq!(world.get_yellow_team_state().len(), 1);
    }

    #[test]
    fn test_world_active_counts() {
        let mut world = World::new(11, 11);

        world.update_robot(1, 0, Vec2::new(1.0, 2.0), 0.0, Vec2::ZERO, 0.0);
        world.update_robot(2, 0, Vec2::new(2.0, 3.0), 0.0, Vec2::ZERO, 0.0);
        world.update_robot(1, 1, Vec2::new(3.0, 4.0), 0.0, Vec2::ZERO, 0.0);

        let (blue_active, yellow_active) = world.get_active_counts();
        assert_eq!(blue_active, 2);
        assert_eq!(yellow_active, 1);
    }

    #[test]
    fn test_world_serialization() {
        let mut world = World::new(11, 11);

        world.update_robot(1, 0, Vec2::new(1.5, 2.3), 1.57, Vec2::new(0.5, -0.2), 0.1);
        world.update_ball(Vec2::new(0.5, 0.3), Vec2::new(0.1, 0.05));

        let json = world.to_json();

        // Verificar que es JSON válido
        assert!(json.is_object());

        // Verificar que tiene robots y ball
        assert!(json.get("robots").is_some());
        assert!(json.get("ball").is_some());

        // Verificar estructura del robot
        let robots = json.get("robots").unwrap().as_array().unwrap();
        assert_eq!(robots.len(), 1);
        let robot = &robots[0];
        assert_eq!(robot["id"], 1);
        assert_eq!(robot["team"], "blue");

        // Verificar estructura del balón
        let ball = json.get("ball").unwrap();
        assert!(ball.get("position").is_some());
        assert!(ball.get("velocity").is_some());
    }
}
