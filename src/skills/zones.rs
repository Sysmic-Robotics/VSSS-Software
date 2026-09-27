//! Zonas prohibidas por reglamento (LARC VSSS 2026 §9.5) como restricción DURA
//! sobre los comandos, independiente de lo que decida la táctica:
//!
//! - **Área propia (70 × 15 cm):** solo el arquero. Dos robots propios adentro
//!   es "falta de área" → penal. Un jugador de campo nunca entra, aunque la
//!   pelota esté adentro (la despeja el arquero).
//! - **Área rival:** como máximo un atacante. El segundo que intente entrar se
//!   queda en el borde.
//!
//! El guardia recorta la componente de velocidad que entraría a la zona y deja
//! la que corre a lo largo del borde (el robot "resbala" por la línea). Si un
//! robot ya está adentro sin permiso (lo empujaron), solo se le permite salir.
//! Se aplica en el control loop después de las skills y de la recuperación de
//! atasco, igual que el resto de los reflejos de bajo nivel.

use crate::motion::MotionCommand;
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
/// Horizonte con el que se anticipa la entrada (s): a 1 m/s frena ~8 cm antes.
const LOOK_AHEAD_S: f32 = 0.08;
/// Velocidad de salida cuando un robot quedó adentro sin permiso (m/s).
const EXIT_SPEED: f32 = 0.30;

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

    /// Velocidad permitida desde `p` con velocidad deseada `v`: sin la componente
    /// que entraría a la zona; si ya está adentro, hacia afuera.
    pub fn block_entry(&self, p: Vec2, v: Vec2) -> Vec2 {
        if self.touches(p) {
            // Ya adentro: salir hacia el centro de la cancha (frente del área).
            let out = Vec2::new(-self.side, 0.0);
            let along_out = v.dot(out).max(EXIT_SPEED);
            return out * along_out;
        }
        let dt = LOOK_AHEAD_S;
        if !self.touches(p + v * dt) {
            return v;
        }
        let vy_only = Vec2::new(0.0, v.y);
        if !self.touches(p + vy_only * dt) {
            return vy_only;
        }
        let vx_only = Vec2::new(v.x, 0.0);
        if !self.touches(p + vx_only * dt) {
            return vx_only;
        }
        Vec2::ZERO
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
            let p = robot.position;
            let mut v = Vec2::new(cmd.vx as f32, cmd.vy as f32);
            if cmd.id != self.keeper_id {
                v = self.own_area.block_entry(p, v);
            }
            let another_inside = own
                .iter()
                .any(|r| r.id != cmd.id && self.opp_area.touches(r.position));
            if another_inside {
                v = self.opp_area.block_entry(p, v);
            }
            cmd.vx = v.x as f64;
            cmd.vy = v.y as f64;
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
    fn field_robot_cannot_enter_own_area_but_slides_along_its_front() {
        let guard = ZoneGuard::new(1.0, 2);
        // Robot 0 justo frente al área propia yendo hacia el arco (−x) y un poco en y.
        let w = world_with(&[(0, 0, -0.52, 0.10, 0.0)]);
        let mut cmds = vec![cmd(0, -1.0, 0.3)];
        guard.guard_commands(&mut cmds, &w, 0, &HashSet::new());
        assert_eq!(cmds[0].vx, 0.0, "componente hacia el área recortada");
        assert!((cmds[0].vy - 0.3).abs() < 1e-6, "resbala por el frente del área");
        assert_eq!(cmds[0].omega, 0.5, "el giro no se toca");
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
