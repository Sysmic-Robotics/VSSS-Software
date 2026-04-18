use crate::coach::{Coach, Observation, RobotTarget};
use crate::motion::{Motion, MotionCommand};
use crate::plays::Play;
use crate::world::World;

/// CoachPlay: puente entre la capa de estrategia (Coach) y la capa de ejecución (Motion).
///
/// Flujo por tick:
///   World → Observation::from_world() → coach.decide() → Vec<RobotTarget>
///   Vec<RobotTarget> → motion.move_and_face() → Vec<MotionCommand>
///
/// Este es el ÚNICO lugar donde el output del Coach toca las primitivas de movimiento.
/// El Coach solo decide "dónde ir"; CoachPlay decide "cómo llegar".
///
/// # Integración del modelo RL
/// Para conectar el modelo entrenado, reemplazar `RuleBasedCoach` con `RlCoach`:
/// ```ignore
/// CoachPlay::new(Box::new(RlCoach::load("model.onnx")), 0)
/// ```
/// El seam sigue disponible, pero hoy requiere cablear un entry point que use
/// `CoachPlay`; el `main` por defecto corre una base headless de skills simples.
///
/// # StuckDetector
/// Deliberadamente omitido en esta fase. El modelo RL aprenderá a evitar deadlocks
/// como parte de su política. Imponer escape lateral encima del RL podría
/// interferir con su comportamiento aprendido.
/// TODO: evaluar si StuckDetector es necesario en producción con RL.
pub struct CoachPlay {
    coach: Box<dyn Coach>,
    /// Ganancias PID para el control de heading (face_to).
    kp: f64,
    ki: f64,
    kd: f64,
    /// Equipo que controla este play: 0 = azul, 1 = amarillo.
    own_team: i32,
}

impl CoachPlay {
    /// Crea un CoachPlay con ganancias PID por defecto (mismas que ApproachBallBehindSkill).
    /// `own_team`: 0 = azul, 1 = amarillo.
    pub fn new(coach: Box<dyn Coach>, own_team: i32) -> Self {
        Self {
            coach,
            kp: 1.2,
            ki: 0.0,
            kd: 0.10,
            own_team,
        }
    }
}

impl Play for CoachPlay {
    fn tick(&mut self, world: &World, motion: &Motion) -> Vec<MotionCommand> {
        // 1. Construir observación normalizada (una sola vez por tick)
        let obs = Observation::from_world(world, self.own_team);

        // 2. Consultar al Coach por los targets (capa de estrategia)
        let targets = self.coach.decide(&obs);

        // 3. Obtener robots activos del equipo propio
        let robots: Vec<_> = if self.own_team == 0 {
            world.get_blue_team_active().into_iter().cloned().collect()
        } else {
            world
                .get_yellow_team_active()
                .into_iter()
                .cloned()
                .collect()
        };

        // 4. Ejecutar cada target con motion primitives (capa de ejecución)
        robots
            .iter()
            .map(|robot| {
                let target = find_target(&targets, robot.id);
                match target {
                    Some(t) => {
                        let face = t.face_target.unwrap_or(t.position);
                        motion.move_and_face(
                            robot, t.position, face, world, self.kp, self.ki, self.kd,
                        )
                    }
                    // Si el Coach no emitió target para este robot → mantener posición
                    None => MotionCommand {
                        id: robot.id,
                        team: robot.team,
                        vx: 0.0,
                        vy: 0.0,
                        omega: 0.0,
                        orientation: robot.orientation,
                    },
                }
            })
            .collect()
    }
}

fn find_target(targets: &[RobotTarget], robot_id: i32) -> Option<&RobotTarget> {
    targets.iter().find(|t| t.robot_id == robot_id)
}
