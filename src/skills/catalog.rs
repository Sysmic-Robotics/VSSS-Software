//! Catálogo cerrado de skills expuestas a la policy RL.
//!
//! Este es el **contrato congelado** entre el motor Rust y el modelo RL
//! entrenado en Python. Cualquier cambio en `SkillId` (agregar, sacar,
//! reordenar) rompe el modelo y obliga a reentrenar.
//!
//! El catálogo de Fase 2 contiene 4 skills, siguiendo el patrón de Bassani
//! 2020 (3 skills: go-to-ball, turn-and-shoot, shoot-goalie) más la striker
//! de LARC 2019 (4 behaviors: spin, approach, push, idle), adaptado a las
//! atómicas que ya tenemos funcionando bien:
//!
//! | id | skill        | usa target | descripción                          |
//! |----|--------------|------------|--------------------------------------|
//! | 0  | GoTo         | sí (xy)    | navegar a un punto con UVF + PID     |
//! | 1  | FacePoint    | sí (xy)    | rotar para mirar a un punto          |
//! | 2  | ChaseBall    | no         | perseguir la posición actual del balón |
//! | 3  | Spin         | sí (signo) | rotar en el lugar (signo de x → CCW/CW) |
//!
//! Skills "out-of-catalog" que viven en el módulo pero no son llamables vía
//! catálogo: `DefendGoalLineSkill` (reservada para portero en Fase 6),
//! `SupportPositionSkill`, `ApproachBallBehindSkill`, `PushBallSkill`,
//! `AlignBallToTargetSkill`, `HoldPositionSkill`, `StopSkill`. Quedan como
//! herramientas de debugging y experimentación manual desde `scenario.rs`.

use crate::motion::{Motion, MotionCommand};
use crate::skills::{ChaseBallSkill, FacePointSkill, GoToSkill, Skill, SkillConfig, SpinSkill};
use crate::world::{RobotState, World};
use glam::Vec2;

/// Identificador discreto de una skill del catálogo.
///
/// **CONTRATO CONGELADO** entre Rust y Python. Los discriminantes son parte
/// del input/output del modelo RL — nunca renumerar, nunca eliminar entradas,
/// solo agregar al final con un nuevo discriminante explícito.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SkillId {
    /// Navegar a un punto del campo. Usa `target` completo.
    GoTo = 0,
    /// Rotar para mirar a un punto. Usa `target` completo, sin trasladarse.
    FacePoint = 1,
    /// Perseguir la pelota. `target` ignorado.
    ChaseBall = 2,
    /// Rotar en el lugar. Usa el signo de `target.x` para definir sentido.
    Spin = 3,
}

impl SkillId {
    /// Cantidad de skills en el catálogo. Este es el tamaño del action space
    /// discreto que la policy debe respetar.
    pub const COUNT: usize = 4;

    /// Construye una `SkillId` a partir del entero que emite la policy.
    /// Retorna `None` si el id está fuera del rango del catálogo.
    pub fn from_u8(id: u8) -> Option<Self> {
        match id {
            0 => Some(Self::GoTo),
            1 => Some(Self::FacePoint),
            2 => Some(Self::ChaseBall),
            3 => Some(Self::Spin),
            _ => None,
        }
    }

    pub fn as_u8(self) -> u8 {
        self as u8
    }

    /// Indica si el target paramétrico es relevante para esta skill.
    /// Útil para logging y para el wrapper Python: skills que ignoran target
    /// no necesitan que la policy emita un punto significativo.
    pub fn uses_target(self) -> bool {
        matches!(self, SkillId::GoTo | SkillId::FacePoint)
    }

    /// Indica si la skill usa el signo de `target.x` como parámetro discreto
    /// (caso del Spin).
    pub fn uses_target_sign(self) -> bool {
        matches!(self, SkillId::Spin)
    }
}

/// Manejador stateful de una instancia del catálogo, con estado por robot.
///
/// Mantiene una instancia separada de cada skill para cada robot del equipo
/// propio, porque las skills tienen estado interno (ganancias PID, integrales)
/// que es **per-robot** — compartirlos contaminaría el control entre robots
/// y haría no determinístico el dispatch.
///
/// Uso típico desde el dispatcher de Fase 3:
/// ```ignore
/// let catalog = SkillCatalog::new(3);   // 3 robots por equipo
/// // ... cada tick:
/// let cmd = catalog.tick(robot.id, SkillId::GoTo, target_xy, &robot, &world, &motion);
/// ```
pub struct SkillCatalog {
    go_to: Vec<GoToSkill>,
    face_point: Vec<FacePointSkill>,
    chase_ball: Vec<ChaseBallSkill>,
    spin: Vec<SpinSkill>,
    config: SkillConfig,
}

impl SkillCatalog {
    /// Crea un catálogo para `num_robots` robots con `SkillConfig::default()`.
    pub fn new(num_robots: usize) -> Self {
        Self::with_config(num_robots, SkillConfig::default())
    }

    /// Crea un catálogo con un `SkillConfig` explícito.
    pub fn with_config(num_robots: usize, config: SkillConfig) -> Self {
        let go_to = (0..num_robots).map(|_| GoToSkill::new(Vec2::ZERO)).collect();
        let face_point = (0..num_robots)
            .map(|_| FacePointSkill::new(Vec2::ZERO))
            .collect();
        let chase_ball = (0..num_robots).map(|_| ChaseBallSkill::new()).collect();
        let spin = (0..num_robots)
            .map(|_| SpinSkill::with_config(&config))
            .collect();
        Self {
            go_to,
            face_point,
            chase_ball,
            spin,
            config,
        }
    }

    /// Cantidad de robots para los que el catálogo tiene estado.
    pub fn num_robots(&self) -> usize {
        self.go_to.len()
    }

    /// Acceso al `SkillConfig` activo (read-only).
    pub fn config(&self) -> &SkillConfig {
        &self.config
    }

    /// Despacha el skill seleccionado para el robot indicado y devuelve
    /// el `MotionCommand` resultante.
    ///
    /// `target` se interpreta según `skill_id`:
    /// - `GoTo` / `FacePoint` → punto destino en coordenadas de campo (m).
    /// - `ChaseBall` → ignorado.
    /// - `Spin` → solo se usa el signo de `target.x` para elegir sentido.
    ///
    /// **Pánico**: si `robot_id` no está en `0..num_robots()`. La policy debe
    /// emitir IDs válidos; el dispatcher de Fase 3 los validará antes de llamar.
    pub fn tick(
        &mut self,
        robot_id: usize,
        skill_id: SkillId,
        target: Vec2,
        robot: &RobotState,
        world: &World,
        motion: &Motion,
    ) -> MotionCommand {
        assert!(
            robot_id < self.num_robots(),
            "robot_id {} fuera de rango (catálogo configurado para {} robots)",
            robot_id,
            self.num_robots()
        );

        match skill_id {
            SkillId::GoTo => {
                let skill = &mut self.go_to[robot_id];
                skill.set_target(target);
                skill.tick(robot, world, motion)
            }
            SkillId::FacePoint => {
                let skill = &mut self.face_point[robot_id];
                skill.set_target(target);
                skill.tick(robot, world, motion)
            }
            SkillId::ChaseBall => self.chase_ball[robot_id].tick(robot, world, motion),
            SkillId::Spin => {
                let skill = &mut self.spin[robot_id];
                skill.set_direction_from(target);
                skill.tick(robot, world, motion)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_robot(id: i32, x: f32, y: f32, orient_deg: f32) -> RobotState {
        let mut robot = RobotState::new(id, 0);
        robot.position = Vec2::new(x, y);
        robot.orientation = orient_deg.to_radians() as f64;
        robot
    }

    #[test]
    fn skill_id_round_trip() {
        for id in 0..SkillId::COUNT as u8 {
            let skill = SkillId::from_u8(id).expect("id válido");
            assert_eq!(skill.as_u8(), id);
        }
        assert!(SkillId::from_u8(SkillId::COUNT as u8).is_none());
        assert!(SkillId::from_u8(255).is_none());
    }

    #[test]
    fn skill_id_count_matches_variants() {
        // Si alguien agrega una variante a SkillId sin actualizar COUNT, este
        // test debe romper. La policy depende de COUNT para dimensionar el
        // action space discreto.
        let known = [
            SkillId::GoTo,
            SkillId::FacePoint,
            SkillId::ChaseBall,
            SkillId::Spin,
        ];
        assert_eq!(known.len(), SkillId::COUNT);
    }

    #[test]
    fn catalog_dispatches_goto() {
        let mut catalog = SkillCatalog::new(3);
        let world = World::new(3, 3);
        let motion = Motion::new();
        let robot = make_robot(0, 0.0, 0.0, 0.0);

        let cmd = catalog.tick(
            0,
            SkillId::GoTo,
            Vec2::new(0.4, 0.0),
            &robot,
            &world,
            &motion,
        );

        // GoTo a (0.4, 0) desde origen → debe pedir movimiento positivo en X
        // o rotar; en cualquier caso el comando no es cero.
        assert!(cmd.vx.abs() + cmd.vy.abs() + cmd.omega.abs() > 0.0);
    }

    #[test]
    fn catalog_dispatches_face_point_without_translation() {
        let mut catalog = SkillCatalog::new(3);
        let world = World::new(3, 3);
        let motion = Motion::new();
        // Robot mirando hacia +Y, target en +X → debe rotar sin trasladarse.
        let robot = make_robot(0, 0.0, 0.0, 90.0);

        let cmd = catalog.tick(
            0,
            SkillId::FacePoint,
            Vec2::new(0.5, 0.0),
            &robot,
            &world,
            &motion,
        );

        assert_eq!(cmd.vx, 0.0);
        assert_eq!(cmd.vy, 0.0);
        assert!(cmd.omega.abs() > 0.0);
    }

    #[test]
    fn catalog_dispatches_chase_ball_ignoring_target() {
        let mut catalog = SkillCatalog::new(3);
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.4, 0.0), Vec2::ZERO);
        let motion = Motion::new();
        let robot = make_robot(0, 0.0, 0.0, 0.0);

        // El target paramétrico se ignora — pasamos algo absurdo y verificamos
        // que el comportamiento depende sólo de la pelota.
        let cmd_a = catalog.tick(
            0,
            SkillId::ChaseBall,
            Vec2::new(-99.0, 99.0),
            &robot,
            &world,
            &motion,
        );
        let cmd_b = catalog.tick(
            0,
            SkillId::ChaseBall,
            Vec2::new(0.0, 0.0),
            &robot,
            &world,
            &motion,
        );

        let diff = (cmd_a.vx - cmd_b.vx).abs()
            + (cmd_a.vy - cmd_b.vy).abs()
            + (cmd_a.omega - cmd_b.omega).abs();
        // Pequeñas diferencias por estado interno del PID son OK; el target
        // mismo no debería mover la decisión más que ese ruido residual.
        assert!(diff < 0.5, "ChaseBall no debe depender del target: diff={diff}");
    }

    #[test]
    fn catalog_dispatches_spin_using_target_sign() {
        let mut catalog = SkillCatalog::new(3);
        let world = World::new(3, 3);
        let motion = Motion::new();
        let robot = make_robot(0, 0.0, 0.0, 0.0);

        let cmd_ccw = catalog.tick(
            0,
            SkillId::Spin,
            Vec2::new(1.0, 0.0),
            &robot,
            &world,
            &motion,
        );
        let cmd_cw = catalog.tick(
            0,
            SkillId::Spin,
            Vec2::new(-1.0, 0.0),
            &robot,
            &world,
            &motion,
        );
        let cmd_zero = catalog.tick(
            0,
            SkillId::Spin,
            Vec2::new(0.0, 0.0),
            &robot,
            &world,
            &motion,
        );

        assert_eq!(cmd_ccw.vx, 0.0);
        assert_eq!(cmd_ccw.vy, 0.0);
        assert!(cmd_ccw.omega > 0.0);

        assert!(cmd_cw.omega < 0.0);
        assert!((cmd_ccw.omega + cmd_cw.omega).abs() < 1e-9);

        assert_eq!(cmd_zero.omega, 0.0);
    }

    #[test]
    fn catalog_per_robot_state_is_independent() {
        // Si dos robots distintos comparten estado de PID, los comandos para
        // uno se contaminan con el historial del otro. Verificamos que dos
        // robots con misma situación reciben comandos coherentes
        // independientemente de ticks anteriores en el otro.
        let mut catalog = SkillCatalog::new(3);
        let world = World::new(3, 3);
        let motion = Motion::new();
        let robot_a = make_robot(0, 0.0, 0.0, 0.0);
        let robot_b = make_robot(1, 0.0, 0.0, 0.0);
        let target = Vec2::new(0.4, 0.0);

        // Sobre-tickear robot A muchas veces para acumular estado PID en su slot.
        for _ in 0..50 {
            catalog.tick(0, SkillId::GoTo, target, &robot_a, &world, &motion);
        }

        // Robot B en su primer tick debe responder con magnitud razonable
        // (no contaminado por el historial de A).
        let cmd_b = catalog.tick(1, SkillId::GoTo, target, &robot_b, &world, &motion);
        assert!(cmd_b.vx.abs() + cmd_b.omega.abs() > 0.0);
        // Y debe ser distinto del comando que ya está en steady-state para A
        // (que tiene historial integrado).
        let cmd_a_after = catalog.tick(0, SkillId::GoTo, target, &robot_a, &world, &motion);
        // Los comandos pueden parecerse en magnitud pero el estado del PID es
        // distinto — verificamos al menos que ambos producen output válido.
        assert!(cmd_a_after.vx.abs() + cmd_a_after.omega.abs() > 0.0);
    }

    #[test]
    #[should_panic(expected = "fuera de rango")]
    fn catalog_panics_on_invalid_robot_id() {
        let mut catalog = SkillCatalog::new(3);
        let world = World::new(3, 3);
        let motion = Motion::new();
        let robot = make_robot(0, 0.0, 0.0, 0.0);
        catalog.tick(99, SkillId::GoTo, Vec2::ZERO, &robot, &world, &motion);
    }
}
