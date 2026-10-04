//! Zonas prohibidas por reglamento (LARC VSSS 2026 §9.5) como restricción DURA
//! sobre los comandos, independiente de lo que decida la táctica:
//!
//! - **Área propia (70 × 15 cm):** solo el arquero. Dos robots propios adentro
//!   es "falta de área" → penal. Un jugador de campo nunca entra, aunque la
//!   pelota esté adentro (la despeja el arquero).
//! - **Área rival:** como máximo un atacante. El segundo que intente entrar se
//!   queda en el borde.
//!
//! El guardia trabaja sobre la velocidad que el robot diferencial realmente ejecuta
//! (la proyección del comando sobre su heading): si ese avance tocaría la zona
//! dentro del look-ahead, anula la traslación y deja el giro de la skill, para que
//! el robot pueda seguir girando hacia otro lado. Si un robot ya está adentro sin
//! permiso (lo empujaron), sale por la cara más alineada con "afuera", o gira en el
//! lugar hasta tener una. Se aplica en el control loop antes y después de la
//! recuperación de atasco (`control_loop::apply_reflexes`).

use crate::motion::{Motion, MotionCommand};
use crate::world::World;
use glam::Vec2;
use std::collections::HashSet;

/// |x| desde el que empieza el área (frente del área).
pub const AREA_X: f32 = 0.60;
/// Mitad del ancho del área.
pub const AREA_HALF_Y: f32 = 0.35;
/// Medio lado del robot: el centro debe quedar a esta distancia del borde para
/// que el cuerpo no toque la zona.
pub const ROBOT_HALF: f32 = 0.04;
/// Horizonte con el que se anticipa la entrada (s). El robot frena con aceleración
/// limitada y la pose llega con latencia (90 ms en el proxy de ruido): en FIRASim con
/// ruido, 0.08 s penetraba 0.10–0.12 m, 0.25 s todavía 2–21 mm en 7 de 12 corridas, y
/// 0.30 s, 0 en 18 de 18. Recalibrar con la latencia medida del robot real.
const LOOK_AHEAD_S: f32 = 0.30;
/// Velocidad de salida cuando un robot quedó adentro sin permiso (m/s).
const EXIT_SPEED: f32 = 0.30;
/// Coseno del ángulo máximo entre una cara y "afuera" para salir sin girar (60°).
const EXIT_ALIGN_COS: f32 = 0.5;
/// Ganancia y tope del giro en el lugar hacia la cara más alineada con "afuera".
const EXIT_TURN_GAIN: f64 = 4.0;
const EXIT_TURN_MAX: f64 = 6.0;

/// Lado que defendemos, desde `VSSL_SIDE` (`left|right`); por defecto azul
/// izquierda (ataca +X), amarillo derecha. Compartido por `main` y el guardia.
pub fn defend_left_from_env(own_team: i32) -> bool {
    let side = std::env::var("VSSL_SIDE")
        .map(|s| s.trim().to_ascii_lowercase())
        .unwrap_or_default();
    match side.as_str() {
        "left" | "izquierda" | "izq" => true,
        "right" | "derecha" | "der" => false,
        _ => own_team == 0,
    }
}

/// Signo de ataque (+1 = atacamos hacia +X) según `VSSL_SIDE`.
pub fn attack_sign_from_env(own_team: i32) -> f32 {
    if defend_left_from_env(own_team) { 1.0 } else { -1.0 }
}

/// Rectángulo del área frente al arco del lado `side` (signo de x del arco).
#[derive(Debug, Clone, Copy)]
pub struct AreaRect {
    pub side: f32,
}

impl AreaRect {
    pub fn own(attack_sign: f32) -> Self {
        Self { side: -attack_sign }
    }

    pub fn opp(attack_sign: f32) -> Self {
        Self { side: attack_sign }
    }

    /// `true` si un robot centrado en `p` tocaría el área (margen de medio robot).
    pub fn touches(&self, p: Vec2) -> bool {
        p.x * self.side >= AREA_X - ROBOT_HALF && p.y.abs() <= AREA_HALF_Y + ROBOT_HALF
    }

    /// `true` si el centro `p` está dentro del área (sin margen; criterio del auditor).
    pub fn contains(&self, p: Vec2) -> bool {
        p.x * self.side >= AREA_X && p.y.abs() <= AREA_HALF_Y
    }

    /// Restringe `cmd` para un robot diferencial en `p` con heading `theta`:
    /// - si ya toca la zona: sale a `EXIT_SPEED` por la cara alineada con "afuera"
    ///   (< 60°) sin girar, o gira en el lugar hacia la cara más alineada;
    /// - si su avance a lo largo del heading la tocaría dentro de `LOOK_AHEAD_S`:
    ///   anula la traslación y conserva `omega`;
    /// - si no: el comando queda intacto.
    pub fn restrict(&self, p: Vec2, theta: f64, cmd: &mut MotionCommand) {
        let h = Vec2::new(theta.cos() as f32, theta.sin() as f32);
        if self.touches(p) {
            let out = Vec2::new(-self.side, 0.0);
            let align = h.dot(out);
            if align.abs() > EXIT_ALIGN_COS {
                let v = EXIT_SPEED * align.signum();
                cmd.vx = (h.x * v) as f64;
                cmd.vy = (h.y * v) as f64;
                cmd.omega = 0.0;
            } else {
                let out_angle = (out.y as f64).atan2(out.x as f64);
                let err = Motion::fold_bidirectional(Motion::normalize_angle(out_angle - theta));
                cmd.vx = 0.0;
                cmd.vy = 0.0;
                cmd.omega = (EXIT_TURN_GAIN * err).clamp(-EXIT_TURN_MAX, EXIT_TURN_MAX);
            }
            return;
        }
        let v = cmd.vx as f32 * h.x + cmd.vy as f32 * h.y;
        if self.touches(p + h * v * LOOK_AHEAD_S) {
            cmd.vx = 0.0;
            cmd.vy = 0.0;
        }
    }
}

pub struct ZoneGuard {
    own_area: AreaRect,
    opp_area: AreaRect,
    keeper_id: i32,
}

impl ZoneGuard {
    pub fn new(attack_sign: f32, keeper_id: i32) -> Self {
        Self {
            own_area: AreaRect::own(attack_sign),
            opp_area: AreaRect::opp(attack_sign),
            keeper_id,
        }
    }

    pub fn own_area(&self) -> AreaRect {
        self.own_area
    }

    pub fn opp_area(&self) -> AreaRect {
        self.opp_area
    }

    /// Recorta los comandos del equipo propio. `manual` = robots bajo control
    /// manual de la GUI (no se tocan).
    pub fn guard_commands(
        &self,
        cmds: &mut [MotionCommand],
        world: &World,
        own_team: i32,
        manual: &HashSet<(i32, i32)>,
    ) {
        let own: Vec<_> = if own_team == 0 {
            world.get_blue_team_active()
        } else {
            world.get_yellow_team_active()
        };
        for cmd in cmds.iter_mut() {
            if cmd.team != own_team || manual.contains(&(cmd.team, cmd.id)) {
                continue;
            }
            let Some(robot) = own.iter().find(|r| r.id == cmd.id) else {
                continue;
            };
            let (p, theta) = (robot.position, robot.orientation);
            if cmd.id != self.keeper_id {
                self.own_area.restrict(p, theta, cmd);
            }
            let another_inside = own
                .iter()
                .any(|r| r.id != cmd.id && self.opp_area.touches(r.position));
            if another_inside {
                self.opp_area.restrict(p, theta, cmd);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cmd(id: i32, vx: f64, vy: f64) -> MotionCommand {
        MotionCommand {
            id,
            team: 0,
            vx,
            vy,
            omega: 0.5,
            orientation: 0.0,
        }
    }

    /// Azul defiende la izquierda (ataca +X): área propia en x ≤ −0.60.
    /// Tuplas `(equipo, id, x, y, orientación)`.
    fn world_with(positions: &[(i32, i32, f32, f32, f32)]) -> World {
        let mut w = World::new(3, 3);
        for (team, id, x, y, theta) in positions {
            // Firma de World: (id, team, ...).
            w.update_robot(*id, *team, Vec2::new(*x, *y), *theta as f64, Vec2::ZERO, 0.0);
        }
        w
    }

    #[test]
    fn field_robot_cannot_enter_own_area_and_keeps_turning() {
        let guard = ZoneGuard::new(1.0, 2);
        // Robot 0 justo frente al área propia, heading 0, retrocediendo hacia el arco:
        // el avance que ejecuta (la proyección al heading) la tocaría → sin traslación.
        let w = world_with(&[(0, 0, -0.52, 0.10, 0.0)]);
        let mut cmds = vec![cmd(0, -1.0, 0.3)];
        guard.guard_commands(&mut cmds, &w, 0, &HashSet::new());
        assert_eq!((cmds[0].vx, cmds[0].vy), (0.0, 0.0), "traslación anulada");
        assert_eq!(cmds[0].omega, 0.5, "el giro de la skill no se toca");
    }

    #[test]
    fn diagonal_entry_is_blocked_on_the_executed_velocity() {
        // Heading −150° (hacia el área) y comando que, proyectado al heading, avanza.
        // El guardia viejo recortaba vx en mundo y el vy restante igual entraba.
        let guard = ZoneGuard::new(1.0, 2);
        let th = (-150f32).to_radians();
        let w = world_with(&[(0, 0, -0.51, 0.10, th)]);
        let mut c = cmd(0, -0.9, -0.5);
        c.orientation = th as f64;
        let mut cmds = vec![c];
        guard.guard_commands(&mut cmds, &w, 0, &HashSet::new());
        assert_eq!((cmds[0].vx, cmds[0].vy), (0.0, 0.0));
        assert_eq!(cmds[0].omega, 0.5);
    }

    #[test]
    fn moving_parallel_to_the_area_front_is_untouched() {
        let guard = ZoneGuard::new(1.0, 2);
        // Frente al área (x = −0.53) con heading 90°, avanzando en +y.
        let w = world_with(&[(0, 0, -0.53, -0.2, std::f32::consts::FRAC_PI_2)]);
        let mut c = cmd(0, 0.0, 0.8);
        c.orientation = std::f64::consts::FRAC_PI_2;
        let mut cmds = vec![c.clone()];
        guard.guard_commands(&mut cmds, &w, 0, &HashSet::new());
        assert_eq!(cmds[0], c);
    }

    #[test]
    fn far_from_the_area_the_command_is_identical() {
        let guard = ZoneGuard::new(1.0, 2);
        let w = world_with(&[(0, 0, 0.0, 0.1, 3.0)]);
        let c = cmd(0, -1.0, 0.3);
        let mut cmds = vec![c.clone()];
        guard.guard_commands(&mut cmds, &w, 0, &HashSet::new());
        assert_eq!(cmds[0], c);
    }

    #[test]
    fn inside_facing_out_leaves_forward_without_turning() {
        let guard = ZoneGuard::new(1.0, 2);
        let w = world_with(&[(0, 0, -0.66, 0.05, 0.1)]); // mira a la cancha (~+x)
        let mut cmds = vec![cmd(0, -0.5, 0.2)];
        guard.guard_commands(&mut cmds, &w, 0, &HashSet::new());
        let v = cmds[0].vx * 0.1f64.cos() + cmds[0].vy * 0.1f64.sin();
        assert!((v - 0.30).abs() < 1e-6, "avance 0.30 a lo largo del heading: {v}");
        assert_eq!(cmds[0].omega, 0.0);
    }

    #[test]
    fn inside_facing_goal_leaves_in_reverse() {
        let guard = ZoneGuard::new(1.0, 2);
        let th = std::f32::consts::PI - 0.1; // mira al propio arco (~−x)
        let w = world_with(&[(0, 0, -0.66, 0.05, th)]);
        let mut cmds = vec![cmd(0, -0.5, 0.2)];
        guard.guard_commands(&mut cmds, &w, 0, &HashSet::new());
        let v = cmds[0].vx * (th as f64).cos() + cmds[0].vy * (th as f64).sin();
        assert!((v + 0.30).abs() < 1e-6, "reversa 0.30: {v}");
        assert!(cmds[0].vx > 0.0, "y eso lo aleja del arco");
        assert_eq!(cmds[0].omega, 0.0);
    }

    #[test]
    fn inside_sideways_turns_in_place() {
        // B1a: de costado dentro del área con Hold (comando cero).
        let guard = ZoneGuard::new(1.0, 2);
        let w = world_with(&[(0, 0, -0.66, 0.0, std::f32::consts::FRAC_PI_2)]);
        let mut cmds = vec![MotionCommand { omega: 0.0, ..cmd(0, 0.0, 0.0) }];
        guard.guard_commands(&mut cmds, &w, 0, &HashSet::new());
        assert_eq!((cmds[0].vx, cmds[0].vy), (0.0, 0.0));
        assert!(cmds[0].omega.abs() > 1.0, "gira hacia una cara de salida: {}", cmds[0].omega);
    }

    #[test]
    fn keeper_may_enter_own_area_and_manual_robots_are_untouched() {
        let guard = ZoneGuard::new(1.0, 2);
        let w = world_with(&[(0, 2, -0.52, 0.0, 0.0), (0, 1, -0.52, 0.0, 0.0)]);
        let mut cmds = vec![cmd(2, -1.0, 0.0), cmd(1, -1.0, 0.0)];
        let manual: HashSet<(i32, i32)> = [(0, 1)].into_iter().collect();
        guard.guard_commands(&mut cmds, &w, 0, &manual);
        assert_eq!(cmds[0].vx, -1.0, "el arquero entra a su área");
        assert_eq!(cmds[1].vx, -1.0, "control manual no se recorta");
    }

    #[test]
    fn field_robot_already_inside_is_pushed_out() {
        let guard = ZoneGuard::new(1.0, 2);
        let w = world_with(&[(0, 0, -0.66, 0.05, 0.0)]);
        let mut cmds = vec![cmd(0, -0.5, 0.2)];
        guard.guard_commands(&mut cmds, &w, 0, &HashSet::new());
        assert!(cmds[0].vx > 0.0, "sale hacia el frente: {:?}", cmds[0]);
        assert_eq!(cmds[0].vy, 0.0);
    }

    // ── Con la planta diferencial ──────────────────────────────────────────

    #[test]
    fn hold_sideways_inside_the_area_gets_out() {
        use crate::motion::test_plant::{run, Case};
        use crate::skills::SkillId;
        let mut case = Case::new(SkillId::Hold, Vec2::ZERO, (-0.66, 0.0, 90.0), Vec2::new(0.6, 0.5));
        case.ticks = 120;
        let trace = run(&case);
        let area = AreaRect::own(1.0);
        let out = trace
            .steps
            .iter()
            .position(|s| !area.touches(Vec2::new(s.x, s.y)))
            .expect("no salió del área");
        assert!(out < 90, "tardó {:.2} s en salir", out as f64 / 60.0);
    }

    #[test]
    fn diagonal_goto_into_own_area_never_touches_it() {
        use crate::motion::test_plant::{run, Case};
        use crate::skills::SkillId;
        let area = AreaRect::own(1.0);
        for case in [
            Case::new(SkillId::GoTo, Vec2::new(-0.72, -0.2), (-0.3, 0.35, 0.0), Vec2::new(0.6, 0.5)),
            Case::new(SkillId::GoTo, Vec2::new(-0.72, -0.2), (-0.3, 0.35, 0.0), Vec2::new(0.6, 0.5))
                .bidirectional(),
        ] {
            let trace = run(&case);
            let touching = trace.steps.iter().filter(|s| area.touches(Vec2::new(s.x, s.y))).count();
            assert_eq!(touching, 0, "tocó el área durante {touching} ticks");
            assert!(trace.steps.iter().all(|s| !s.escaping), "escape espurio junto al área");
        }
    }

    #[test]
    fn second_attacker_is_kept_out_of_opponent_area() {
        let guard = ZoneGuard::new(1.0, 2);
        // Robot 1 ya está en el área rival (x ≥ 0.60); robot 0 quiere entrar.
        let w = world_with(&[(0, 1, 0.65, 0.0, 0.0), (0, 0, 0.52, 0.0, 0.0)]);
        let mut cmds = vec![cmd(0, 1.0, 0.0), cmd(1, 1.0, 0.0)];
        guard.guard_commands(&mut cmds, &w, 0, &HashSet::new());
        assert_eq!(cmds[0].vx, 0.0, "segundo atacante bloqueado");
        // Robot 1 (adentro, el único) no se recorta: puede seguir jugando adentro.
        assert_eq!(cmds[1].vx, 1.0);
        // Con el área rival vacía, robot 0 sí puede entrar.
        let w2 = world_with(&[(0, 1, 0.0, 0.3, 0.0), (0, 0, 0.52, 0.0, 0.0)]);
        let mut cmds2 = vec![cmd(0, 1.0, 0.0)];
        guard.guard_commands(&mut cmds2, &w2, 0, &HashSet::new());
        assert_eq!(cmds2[0].vx, 1.0);
    }

    #[test]
    fn mirrored_side_uses_the_other_goal() {
        let guard = ZoneGuard::new(-1.0, 2); // defendemos la derecha
        assert!(guard.own_area().contains(Vec2::new(0.65, 0.1)));
        assert!(!guard.own_area().contains(Vec2::new(-0.65, 0.1)));
        assert!(guard.opp_area().contains(Vec2::new(-0.65, 0.1)));
        let w = world_with(&[(0, 0, 0.52, 0.0, 0.0)]);
        let mut cmds = vec![cmd(0, 1.0, 0.0)];
        guard.guard_commands(&mut cmds, &w, 0, &HashSet::new());
        assert_eq!(cmds[0].vx, 0.0);
    }

    #[test]
    fn side_from_env_defaults_by_color() {
        // Sin VSSL_SIDE (no se setea en tests): azul izquierda, amarillo derecha.
        if std::env::var("VSSL_SIDE").is_err() {
            assert_eq!(attack_sign_from_env(0), 1.0);
            assert_eq!(attack_sign_from_env(1), -1.0);
        }
    }
}
