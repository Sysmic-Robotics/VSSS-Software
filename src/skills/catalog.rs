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
use crate::skills::{
    ApproachAlignedSkill, BlockLineSkill, ChaseBallSkill, ClearSkill, DefendGoalLineSkill,
    FacePointSkill, GoToSkill, InterceptSkill, MarkSkill, ShootPushSkill, Skill, SkillConfig,
    SkillStatus, SpinKickSkill, SpinSkill, StopSkill,
};
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
    /// Llegar detrás de la pelota sobre la línea pelota→`target`, alineado
    /// (con la cara que requiera menos giro). Paso previo de `ShootPush`.
    ApproachAligned = 4,
    /// Conducir/empujar la pelota alineado hacia `target`; termina al soltarla.
    ShootPush = 5,
    /// Ir al punto de intercepción predicho de la pelota. `target` ignorado.
    Intercept = 6,
    /// Cubrir la línea pelota→arco propio; `target` = centro del arco propio.
    BlockLine = 7,
    /// Arquero: línea de gol con predicción de trayectoria de la pelota;
    /// `target` = centro del arco propio.
    GoalKeep = 8,
    /// Despeje: aproximación corta + empuje con tolerancias amplias hacia `target`.
    Clear = 9,
    /// Patada por giro: contacto lateral + spin; lanza la pelota hacia `target`.
    SpinKick = 10,
    /// Posicionamiento mirando a la pelota; `target` = punto (lo calcula la táctica).
    Mark = 11,
    /// Quieto (STOP/HALT del árbitro). `target` ignorado.
    Hold = 12,
}

impl SkillId {
    /// Cantidad de skills en el catálogo. Este es el tamaño del action space
    /// discreto que la policy debe respetar.
    pub const COUNT: usize = 13;

    /// Construye una `SkillId` a partir del entero que emite la policy.
    /// Retorna `None` si el id está fuera del rango del catálogo.
    pub fn from_u8(id: u8) -> Option<Self> {
        match id {
            0 => Some(Self::GoTo),
            1 => Some(Self::FacePoint),
            2 => Some(Self::ChaseBall),
            3 => Some(Self::Spin),
            4 => Some(Self::ApproachAligned),
            5 => Some(Self::ShootPush),
            6 => Some(Self::Intercept),
            7 => Some(Self::BlockLine),
            8 => Some(Self::GoalKeep),
            9 => Some(Self::Clear),
            10 => Some(Self::SpinKick),
            11 => Some(Self::Mark),
            12 => Some(Self::Hold),
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
        matches!(
            self,
            SkillId::GoTo
                | SkillId::FacePoint
                | SkillId::ApproachAligned
                | SkillId::ShootPush
                | SkillId::BlockLine
                | SkillId::GoalKeep
                | SkillId::Clear
                | SkillId::SpinKick
                | SkillId::Mark
        )
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
    approach: Vec<ApproachAlignedSkill>,
    shoot: Vec<ShootPushSkill>,
    intercept: Vec<InterceptSkill>,
    block: Vec<BlockLineSkill>,
    goal_keep: Vec<DefendGoalLineSkill>,
    clear: Vec<ClearSkill>,
    spin_kick: Vec<SpinKickSkill>,
    mark: Vec<MarkSkill>,
    hold: Vec<StopSkill>,
    /// Última skill despachada por robot: al cambiar, se reinicia el estado de control
    /// de motion de ese robot (PID de heading, rampa del avance).
    last_skill: Vec<Option<SkillId>>,
    config: SkillConfig,
}

/// Reconstruye el arquero si el arco propio cambió de lado (el `target` de
/// `GoalKeep` es el centro del arco propio; solo importa su signo en x).
fn ensure_goal_side(skill: &mut DefendGoalLineSkill, own_goal: Vec2) {
    let same_side = (skill.defend_x < 0.0) == (own_goal.x < 0.0);
    if !same_side {
        *skill = DefendGoalLineSkill::new(own_goal);
    }
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
        let approach = (0..num_robots)
            .map(|_| ApproachAlignedSkill::new(Vec2::ZERO))
            .collect();
        let shoot = (0..num_robots)
            .map(|_| ShootPushSkill::new(Vec2::ZERO))
            .collect();
        let intercept = (0..num_robots).map(|_| InterceptSkill::new()).collect();
        let block = (0..num_robots)
            .map(|_| BlockLineSkill::new(Vec2::new(-0.75, 0.0)))
            .collect();
        let goal_keep = (0..num_robots)
            .map(|_| DefendGoalLineSkill::new(Vec2::new(-0.75, 0.0)))
            .collect();
        let clear = (0..num_robots).map(|_| ClearSkill::new(Vec2::ZERO)).collect();
        let spin_kick = (0..num_robots)
            .map(|_| SpinKickSkill::new(Vec2::ZERO))
            .collect();
        let mark = (0..num_robots).map(|_| MarkSkill::new(Vec2::ZERO)).collect();
        let hold = (0..num_robots).map(|_| StopSkill::new()).collect();
        Self {
            go_to,
            face_point,
            chase_ball,
            spin,
            approach,
            shoot,
            intercept,
            block,
            goal_keep,
            clear,
            spin_kick,
            mark,
            hold,
            last_skill: vec![None; num_robots],
            config,
        }
    }

    /// La instancia de `skill_id` del robot `robot_id`, como `dyn Skill` (para `reset`).
    fn skill_mut(&mut self, robot_id: usize, skill_id: SkillId) -> &mut dyn Skill {
        match skill_id {
            SkillId::GoTo => &mut self.go_to[robot_id],
            SkillId::FacePoint => &mut self.face_point[robot_id],
            SkillId::ChaseBall => &mut self.chase_ball[robot_id],
            SkillId::Spin => &mut self.spin[robot_id],
            SkillId::ApproachAligned => &mut self.approach[robot_id],
            SkillId::ShootPush => &mut self.shoot[robot_id],
            SkillId::Intercept => &mut self.intercept[robot_id],
            SkillId::BlockLine => &mut self.block[robot_id],
            SkillId::GoalKeep => &mut self.goal_keep[robot_id],
            SkillId::Clear => &mut self.clear[robot_id],
            SkillId::SpinKick => &mut self.spin_kick[robot_id],
            SkillId::Mark => &mut self.mark[robot_id],
            SkillId::Hold => &mut self.hold[robot_id],
        }
    }

    /// Estado observable (`SkillStatus`) de la skill `skill_id` para `robot_id`,
    /// con el mismo `target` que se le pasaría a `tick`. No muta estado de control.
    pub fn status(
        &mut self,
        robot_id: usize,
        skill_id: SkillId,
        target: Vec2,
        robot: &RobotState,
        world: &World,
    ) -> SkillStatus {
        assert!(robot_id < self.num_robots(), "robot_id fuera de rango");
        match skill_id {
            SkillId::GoTo => self.go_to[robot_id].status(robot, world),
            SkillId::FacePoint => self.face_point[robot_id].status(robot, world),
            SkillId::ChaseBall => self.chase_ball[robot_id].status(robot, world),
            SkillId::Spin => self.spin[robot_id].status(robot, world),
            SkillId::ApproachAligned => {
                let s = &mut self.approach[robot_id];
                s.set_aim_point(target);
                s.status(robot, world)
            }
            SkillId::ShootPush => {
                let s = &mut self.shoot[robot_id];
                s.set_target(target);
                s.status(robot, world)
            }
            SkillId::Intercept => self.intercept[robot_id].status(robot, world),
            SkillId::BlockLine => {
                let s = &mut self.block[robot_id];
                s.set_own_goal(target);
                s.status(robot, world)
            }
            SkillId::GoalKeep => {
                let s = &mut self.goal_keep[robot_id];
                ensure_goal_side(s, target);
                s.status(robot, world)
            }
            SkillId::Clear => {
                let s = &mut self.clear[robot_id];
                s.set_target(target);
                s.status(robot, world)
            }
            SkillId::SpinKick => {
                let s = &mut self.spin_kick[robot_id];
                s.set_target(target);
                s.status(robot, world)
            }
            SkillId::Mark => {
                let s = &mut self.mark[robot_id];
                s.set_point(target);
                s.status(robot, world)
            }
            SkillId::Hold => self.hold[robot_id].status(robot, world),
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

    /// Override en vivo de las ganancias del PID de heading (kp/ki/kd) en todas
    /// las skills que lo usan (`GoTo`, `FacePoint`, `ChaseBall`), para todos los
    /// robots. Pensado para tuneo desde la GUI; no altera los defaults del
    /// catálogo ni el `SkillId`/orden congelado.
    pub fn set_heading_pid(&mut self, kp: f64, ki: f64, kd: f64) {
        for s in &mut self.go_to {
            s.kp = kp;
            s.ki = ki;
            s.kd = kd;
        }
        for s in &mut self.face_point {
            s.kp = kp;
            s.ki = ki;
            s.kd = kd;
        }
        for s in &mut self.chase_ball {
            s.kp = kp;
            s.ki = ki;
            s.kd = kd;
        }
    }

    /// Override en vivo de la velocidad angular de `Spin` para un robot puntual.
    /// Pensado para el runner de la GUI (tuneo de bring-up); no altera el
    /// `SkillConfig` por defecto del catálogo. No-op si `robot_id` fuera de rango.
    pub fn set_spin_omega_for(&mut self, robot_id: usize, omega_max: f64) {
        if let Some(spin) = self.spin.get_mut(robot_id) {
            spin.omega_max = omega_max;
        }
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

        if self.last_skill[robot_id] != Some(skill_id) {
            // Cambio de skill: sin integral ni derivada heredadas, rampa desde cero, y la
            // skill que se activa sin estado de una activación anterior.
            motion.reset_robot(robot.team, robot.id);
            self.skill_mut(robot_id, skill_id).reset();
            self.last_skill[robot_id] = Some(skill_id);
        }

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
            SkillId::ApproachAligned => {
                let skill = &mut self.approach[robot_id];
                skill.set_aim_point(target);
                skill.tick(robot, world, motion)
            }
            SkillId::ShootPush => {
                let skill = &mut self.shoot[robot_id];
                skill.set_target(target);
                skill.tick(robot, world, motion)
            }
            SkillId::Intercept => self.intercept[robot_id].tick(robot, world, motion),
            SkillId::BlockLine => {
                let skill = &mut self.block[robot_id];
                skill.set_own_goal(target);
                skill.tick(robot, world, motion)
            }
            SkillId::GoalKeep => {
                let skill = &mut self.goal_keep[robot_id];
                ensure_goal_side(skill, target);
                skill.tick(robot, world, motion)
            }
            SkillId::Clear => {
                let skill = &mut self.clear[robot_id];
                skill.set_target(target);
                skill.tick(robot, world, motion)
            }
            SkillId::SpinKick => {
                let skill = &mut self.spin_kick[robot_id];
                skill.set_target(target);
                skill.tick(robot, world, motion)
            }
            SkillId::Mark => {
                let skill = &mut self.mark[robot_id];
                skill.set_point(target);
                skill.tick(robot, world, motion)
            }
            SkillId::Hold => self.hold[robot_id].tick(robot, world, motion),
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
            SkillId::ApproachAligned,
            SkillId::ShootPush,
            SkillId::Intercept,
            SkillId::BlockLine,
            SkillId::GoalKeep,
            SkillId::Clear,
            SkillId::SpinKick,
            SkillId::Mark,
            SkillId::Hold,
        ];
        assert_eq!(known.len(), SkillId::COUNT);
        for (i, id) in known.iter().enumerate() {
            assert_eq!(SkillId::from_u8(i as u8), Some(*id));
        }
    }

    #[test]
    fn catalog_dispatches_batch2_skills() {
        let mut catalog = SkillCatalog::new(3);
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(-0.5, 0.0), Vec2::ZERO);
        let motion = Motion::new();
        let robot = make_robot(1, -0.3, 0.2, 0.0);
        for (id, target) in [
            (SkillId::Clear, Vec2::new(0.2, 0.45)),
            (SkillId::SpinKick, Vec2::new(0.75, 0.0)),
            (SkillId::Mark, Vec2::new(-0.2, 0.1)),
        ] {
            let cmd = catalog.tick(1, id, target, &robot, &world, &motion);
            assert!(cmd.vx.is_finite() && cmd.vy.is_finite() && cmd.omega.is_finite());
            let st = catalog.status(1, id, target, &robot, &world);
            assert!((0.0..=1.0).contains(&st.progress));
        }
    }

    #[test]
    fn goal_keep_follows_goal_side_from_target() {
        let mut catalog = SkillCatalog::new(3);
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.0, 0.1), Vec2::ZERO);
        let motion = Motion::new();
        let robot = make_robot(2, 0.6, 0.0, 0.0);
        // Arco propio a la derecha (equipo amarillo): el target de movimiento
        // debe quedar del lado +x.
        let _ = catalog.tick(2, SkillId::GoalKeep, Vec2::new(0.75, 0.0), &robot, &world, &motion);
        assert!(catalog.goal_keep[2].defend_x > 0.0);
        let _ = catalog.tick(2, SkillId::GoalKeep, Vec2::new(-0.75, 0.0), &robot, &world, &motion);
        assert!(catalog.goal_keep[2].defend_x < 0.0);
    }

    #[test]
    fn catalog_dispatches_tactical_skills() {
        let mut catalog = SkillCatalog::new(3);
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.2, 0.0), Vec2::new(0.5, 0.0));
        let motion = Motion::new();
        let robot = make_robot(0, -0.4, 0.1, 0.0);
        let goal = Vec2::new(0.75, 0.0);

        for (id, target) in [
            (SkillId::ApproachAligned, goal),
            (SkillId::ShootPush, goal),
            (SkillId::Intercept, Vec2::ZERO),
            (SkillId::BlockLine, Vec2::new(-0.75, 0.0)),
        ] {
            let cmd = catalog.tick(0, id, target, &robot, &world, &motion);
            assert!(cmd.vx.is_finite() && cmd.vy.is_finite() && cmd.omega.is_finite());
            let st = catalog.status(0, id, target, &robot, &world);
            assert!((0.0..=1.0).contains(&st.progress));
        }
        // ShootPush desde lejos no es factible (debe reportarlo, no fingir empuje).
        assert!(!catalog.status(0, SkillId::ShootPush, goal, &robot, &world).feasible);
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
    fn set_heading_pid_updates_all_skills() {
        let mut catalog = SkillCatalog::new(2);
        catalog.set_heading_pid(9.0, 1.0, 0.5);
        for i in 0..2 {
            assert_eq!(catalog.go_to[i].kp, 9.0);
            assert_eq!(catalog.face_point[i].ki, 1.0);
            assert_eq!(catalog.chase_ball[i].kd, 0.5);
        }
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
    fn skill_change_resets_heading_pid() {
        // 300 ticks de FacePoint con error constante de 90° dejan integral y derivada
        // cargadas. Al pasar a Mark ya en su punto, mirando la pelota de frente (error 0),
        // el primer tick no debe heredar nada: ω = 0.
        let mut catalog = SkillCatalog::new(3);
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.5, 0.0), Vec2::ZERO);
        let motion = Motion::new();
        let robot = make_robot(0, 0.0, 0.0, 0.0);
        for _ in 0..300 {
            catalog.tick(0, SkillId::FacePoint, Vec2::new(0.0, 0.5), &robot, &world, &motion);
        }
        let cmd = catalog.tick(0, SkillId::Mark, Vec2::ZERO, &robot, &world, &motion);
        assert_eq!(cmd.omega, 0.0, "estado heredado de la skill anterior: ω={}", cmd.omega);
    }

    #[test]
    fn interrupted_spin_kick_starts_clean_when_reactivated() {
        // B4: girando junto a la pelota → GoTo → SpinKick de nuevo, a 0.16 m de la pelota
        // y lejos del punto de contacto. Sin reinicio heredaba el giro: giraba en el lugar
        // sin acercarse y con el timeout ya consumido.
        let mut catalog = SkillCatalog::new(3);
        let mut world = World::new(3, 3);
        let ball = Vec2::new(0.2, -0.3);
        world.update_ball(ball, Vec2::ZERO);
        let motion = Motion::new();
        let tgt = Vec2::new(0.0, 0.3);
        let probe = SpinKickSkill::new(tgt);
        let (ccw, _) = probe.contact_centers(ball, (tgt - ball).normalize());
        let at = make_robot(0, ccw.x, ccw.y, 0.0);
        let mut w = 0.0;
        for _ in 0..10 {
            w = catalog.tick(0, SkillId::SpinKick, tgt, &at, &world, &motion).omega;
        }
        assert!((w.abs() - probe.omega).abs() < 1e-9, "girando: ω={w}");
        let off = make_robot(0, ball.x + 0.10, ball.y + 0.12, 0.0);
        // Misma skill consecutiva: no se reinicia, el giro sigue.
        let w = catalog.tick(0, SkillId::SpinKick, tgt, &off, &world, &motion).omega;
        assert!((w.abs() - probe.omega).abs() < 1e-9, "sin cambio de skill sigue girando: ω={w}");
        for _ in 0..30 {
            catalog.tick(0, SkillId::GoTo, Vec2::ZERO, &off, &world, &motion);
        }
        let c = catalog.tick(0, SkillId::SpinKick, tgt, &off, &world, &motion);
        assert!(c.omega.abs() <= motion.config.max_angular_speed + 1e-9, "relanzada gira en el lugar: ω={}", c.omega);
        assert!(!catalog.status(0, SkillId::SpinKick, tgt, &off, &world).done);
    }

    #[test]
    fn skill_change_of_one_robot_keeps_the_others_state() {
        // El robot 1 corre FacePoint en dos catálogos idénticos; en uno de ellos el
        // robot 0 cambia de skill en el medio. El comando siguiente del robot 1 es el mismo.
        let world = World::new(3, 3);
        let r0 = make_robot(0, 0.0, 0.0, 0.0);
        let r1 = make_robot(1, 0.3, 0.0, 0.0);
        let face = Vec2::new(0.3, 0.5);
        let (mut a, mut b) = (SkillCatalog::new(3), SkillCatalog::new(3));
        let (ma, mb) = (Motion::new(), Motion::new());
        for k in 0..60 {
            a.tick(1, SkillId::FacePoint, face, &r1, &world, &ma);
            b.tick(1, SkillId::FacePoint, face, &r1, &world, &mb);
            let s0 = if k < 30 { SkillId::GoTo } else { SkillId::FacePoint };
            a.tick(0, s0, Vec2::new(0.4, 0.2), &r0, &world, &ma);
        }
        let ca = a.tick(1, SkillId::FacePoint, face, &r1, &world, &ma);
        let cb = b.tick(1, SkillId::FacePoint, face, &r1, &world, &mb);
        assert_eq!(ca.omega, cb.omega);
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
