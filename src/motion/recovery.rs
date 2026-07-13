//! Capa de recuperación de atasco (wrapper del control loop).
//!
//! Envuelve la salida de las skills (`MotionCommand`) sin modificarlas: si el robot se
//! atasca (comandado a moverse pero sin avanzar durante N ticks — p. ej. un empujón de
//! rival), ejecuta una maniobra de escape wall-aware (reversa alejándose de la pared +
//! giro hacia el interior). La evasión de pared en operación normal la hace la navegación
//! (UVF), que conoce el target; esta capa es solo recuperación de último recurso.
//!
//! Es un reflejo de bajo nivel común a coach clásico y RL; conmutable por
//! `VSSL_BORDER_RECOVERY` (default on). No toca manual ni estop.

use std::collections::HashMap;

use glam::Vec2;

use super::commands::MotionCommand;
use crate::world::RobotState;

// Geometría física del campo (m): el centro del robot puede acercarse hasta
// ~pared − radio. Se mide "cerca del borde" con una franja `BORDER_BAND`.
const FIELD_HALF_X: f32 = 0.75;
const FIELD_HALF_Y: f32 = 0.65;
/// Franja (m) desde la pared física dentro de la cual se considera "junto al borde".
const BORDER_BAND: f32 = 0.10;

/// Ticks moviéndose por debajo del umbral (mientras se le comanda avanzar) para
/// declarar atasco (~0.5 s @ 60 Hz).
const STUCK_TICKS: u32 = 30;
/// Duración de la maniobra de escape una vez disparada (ticks).
const RECOVERY_TICKS: u32 = 25;
/// Movimiento medido por tick por debajo del cual se cuenta como "no avanza" (m).
const STUCK_MOVE_EPS: f32 = 0.006;
/// Rapidez comandada mínima (m/s) para considerar que "se le pidió moverse".
const CMD_MOVE_EPS: f64 = 0.05;
/// Rapidez (m/s) de la reversa de escape (world frame, alejándose de la pared).
const ESCAPE_SPEED: f64 = 0.4;
/// Velocidad angular (rad/s) del giro de escape hacia el interior.
const ESCAPE_OMEGA: f64 = 6.0;

/// Estado de recuperación por robot.
#[derive(Default, Clone, Copy)]
struct RecoveryState {
    stuck_ticks: u32,
    recovery_ticks: u32,
    last_pos: Vec2,
    /// Dirección de escape fija durante la ventana de recuperación (unit, world).
    escape_dir: Vec2,
    /// `last_pos` ya inicializado.
    seen: bool,
}

/// Dirección unitaria de alejamiento de la pared más cercana. En esquina combina
/// ambas paredes; si no está junto a ninguna pared (atasco por rival), empuja hacia
/// el centro del campo.
pub fn outward_from_nearest_wall(pos: Vec2) -> Vec2 {
    let mut dir = Vec2::ZERO;
    if pos.x > FIELD_HALF_X - BORDER_BAND {
        dir.x -= 1.0;
    }
    if pos.x < -FIELD_HALF_X + BORDER_BAND {
        dir.x += 1.0;
    }
    if pos.y > FIELD_HALF_Y - BORDER_BAND {
        dir.y -= 1.0;
    }
    if pos.y < -FIELD_HALF_Y + BORDER_BAND {
        dir.y += 1.0;
    }
    if dir == Vec2::ZERO {
        // No está junto a una pared → alejarse del punto actual hacia el centro.
        dir = -pos;
    }
    dir.normalize_or_zero()
}

/// Envuelve un ángulo (rad) a `[-π, π]`.
fn wrap_angle(a: f64) -> f64 {
    let mut a = a % (2.0 * std::f64::consts::PI);
    if a > std::f64::consts::PI {
        a -= 2.0 * std::f64::consts::PI;
    } else if a < -std::f64::consts::PI {
        a += 2.0 * std::f64::consts::PI;
    }
    a
}

/// Capa de recuperación con estado por `(team, id)`. Conmutable.
pub struct BorderRecovery {
    states: HashMap<(i32, i32), RecoveryState>,
    enabled: bool,
}

impl BorderRecovery {
    pub fn new(enabled: bool) -> Self {
        Self {
            states: HashMap::new(),
            enabled,
        }
    }

    /// Lee el flag `VSSL_BORDER_RECOVERY` (default habilitado; `=0` lo apaga).
    pub fn from_env() -> Self {
        let enabled = std::env::var("VSSL_BORDER_RECOVERY").unwrap_or_default() != "0";
        Self::new(enabled)
    }

    /// Aplica prevención + recuperación a un comando, dado el estado medido del robot.
    /// Modifica `cmd` en el lugar. Función central, testeable sin el loop.
    pub fn guard(&mut self, cmd: &mut MotionCommand, robot: &RobotState) {
        if !self.enabled {
            return;
        }
        let pos = robot.position;
        let st = self.states.entry((robot.team, robot.id)).or_default();

        // Primera observación: inicializa `last_pos` sin declarar atasco.
        if !st.seen {
            st.seen = true;
            st.last_pos = pos;
        }

        // Ventana de escape activa: forzar la maniobra, ignorar el comando de la skill.
        if st.recovery_ticks > 0 {
            st.recovery_ticks -= 1;
            apply_escape(cmd, st.escape_dir, robot);
            st.last_pos = pos;
            return;
        }

        // Detección por resultado: se le pide moverse pero no avanza.
        let cmd_speed = (cmd.vx * cmd.vx + cmd.vy * cmd.vy).sqrt();
        let moved = (pos - st.last_pos).length();
        st.last_pos = pos;
        if cmd_speed > CMD_MOVE_EPS && moved < STUCK_MOVE_EPS {
            st.stuck_ticks += 1;
        } else {
            st.stuck_ticks = 0;
        }

        if st.stuck_ticks >= STUCK_TICKS {
            st.stuck_ticks = 0;
            st.recovery_ticks = RECOVERY_TICKS;
            st.escape_dir = outward_from_nearest_wall(pos);
            apply_escape(cmd, st.escape_dir, robot);
        }
        // Sin atasco: pass-through. La evasión de pared es responsabilidad de la
        // navegación (UVF), que conoce el target (p. ej. pelota pegada al borde); una
        // prevención aquí sería target-ciega y bloquearía ese acercamiento.
    }

    /// Aplica `guard` a todos los comandos del equipo propio con comando autónomo,
    /// saltando los robots en control manual y los que no tienen estado de visión.
    pub fn guard_commands(
        &mut self,
        commands: &mut [MotionCommand],
        world: &crate::world::World,
        manual_keys: &std::collections::HashSet<(i32, i32)>,
        own_team: i32,
    ) {
        if !self.enabled {
            return;
        }
        for cmd in commands.iter_mut() {
            if cmd.team != own_team || manual_keys.contains(&(cmd.team, cmd.id)) {
                continue;
            }
            if let Some(robot) = world.get_robot_state(cmd.id, cmd.team) {
                self.guard(cmd, robot);
            }
        }
    }
}

/// Fija en `cmd` la maniobra de escape: velocidad world alejándose de la pared
/// (reversa relativa al heading) + giro hacia esa dirección.
fn apply_escape(cmd: &mut MotionCommand, escape_dir: Vec2, robot: &RobotState) {
    cmd.vx = escape_dir.x as f64 * ESCAPE_SPEED;
    cmd.vy = escape_dir.y as f64 * ESCAPE_SPEED;
    // Girar el heading hacia la dirección de escape (para que la reversa se vuelva
    // avance una vez despegado).
    let desired = (escape_dir.y as f64).atan2(escape_dir.x as f64);
    let err = wrap_angle(desired - robot.orientation);
    cmd.omega = ESCAPE_OMEGA * err.signum();
    cmd.orientation = robot.orientation;
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::SystemTime;

    fn robot_at(pos: Vec2) -> RobotState {
        RobotState {
            id: 0,
            team: 0,
            position: pos,
            velocity: Vec2::ZERO,
            orientation: 0.0,
            angular_velocity: 0.0,
            active: true,
            last_update: SystemTime::UNIX_EPOCH,
        }
    }

    fn cmd_at(vx: f64, vy: f64) -> MotionCommand {
        MotionCommand {
            id: 0,
            team: 0,
            vx,
            vy,
            omega: 0.0,
            orientation: 0.0,
        }
    }

    // ── Sin prevención target-ciega ───────────────────────────────────────────

    #[test]
    fn near_border_not_stuck_is_passthrough() {
        // Junto al borde +x con velocidad hacia la pared, pero avanzando (no atascado):
        // el comando pasa SIN recorte (la evasión de pared la hace el UVF, no esta capa).
        let mut rec = BorderRecovery::new(true);
        let mut pos = Vec2::new(0.66, 0.0);
        for _ in 0..10 {
            let robot = robot_at(pos);
            let mut cmd = cmd_at(0.5, 0.2);
            rec.guard(&mut cmd, &robot);
            assert_eq!(cmd.vx, 0.5, "sin atasco no debe recortar la velocidad");
            assert_eq!(cmd.vy, 0.2);
            pos.x += 0.02; // avanza (no se atasca)
        }
    }

    #[test]
    fn outward_points_into_field_from_corner() {
        // Esquina superior-derecha (+x,+y): escape hacia (-x,-y).
        let d = outward_from_nearest_wall(Vec2::new(FIELD_HALF_X - 0.02, FIELD_HALF_Y - 0.02));
        assert!(d.x < 0.0 && d.y < 0.0);
        assert!((d.length() - 1.0).abs() < 1e-5);
    }

    // ── Detección + escape ──────────────────────────────────────────────────

    #[test]
    fn stuck_triggers_escape_away_from_wall() {
        let mut rec = BorderRecovery::new(true);
        let pos = Vec2::new(FIELD_HALF_X - 0.03, 0.0); // pegado a la pared +x
        let robot = robot_at(pos);

        // Se le comanda avanzar hacia +x pero no se mueve.
        for _ in 0..(STUCK_TICKS + 1) {
            let mut cmd = cmd_at(0.6, 0.0);
            rec.guard(&mut cmd, &robot);
        }
        // Tras superar el umbral, el comando es de escape: reversa (vx<0, alejándose).
        let mut cmd = cmd_at(0.6, 0.0);
        rec.guard(&mut cmd, &robot);
        assert!(cmd.vx < 0.0, "el escape debe alejar de la pared +x: {}", cmd.vx);
    }

    #[test]
    fn advancing_robot_is_passthrough() {
        let mut rec = BorderRecovery::new(true);
        // Avanza pero se mantiene lejos del borde (x sube de -0.2 a 0.4 < franja).
        let mut pos = Vec2::new(-0.2, 0.0);
        for _ in 0..30 {
            let robot = robot_at(pos);
            let mut cmd = cmd_at(0.6, 0.0);
            rec.guard(&mut cmd, &robot);
            // Lejos del borde y avanzando: comando intacto, sin escape.
            assert_eq!(cmd.vx, 0.6);
            assert_eq!(cmd.omega, 0.0);
            pos.x += 0.02;
        }
    }

    #[test]
    fn zero_command_is_passthrough_even_at_wall() {
        // Comando en cero (p. ej. estop): no hay "comandado a moverse" → sin escape,
        // y la proyección no recorta nada (velocidad nula).
        let mut rec = BorderRecovery::new(true);
        let robot = robot_at(Vec2::new(FIELD_HALF_X - 0.01, 0.0));
        for _ in 0..(STUCK_TICKS + 5) {
            let mut cmd = cmd_at(0.0, 0.0);
            rec.guard(&mut cmd, &robot);
            assert_eq!((cmd.vx, cmd.vy, cmd.omega), (0.0, 0.0, 0.0));
        }
    }

    // ── Exclusión de robots en manual (nivel guard_commands) ──────────────────

    #[test]
    fn guard_commands_skips_manual_robots() {
        use crate::world::World;
        let mut world = World::new(3, 3);
        // Robot propio pegado a la pared +x, comandado hacia la pared.
        world.update_robot(0, 0, Vec2::new(FIELD_HALF_X - 0.03, 0.0), 0.0, Vec2::ZERO, 0.0);

        let mut rec = BorderRecovery::new(true);
        let manual: std::collections::HashSet<(i32, i32)> =
            [(0, 0)].into_iter().collect();
        let mut cmds = vec![cmd_at(0.6, 0.0)];
        rec.guard_commands(&mut cmds, &world, &manual, 0);
        // Está en manual → no se toca (ni proyección ni escape), pese a estar al borde.
        assert_eq!(cmds[0].vx, 0.6, "robot manual no debe modificarse");
    }

    // ── Flag off ─────────────────────────────────────────────────────────────

    #[test]
    fn disabled_is_pure_passthrough() {
        let mut rec = BorderRecovery::new(false);
        let pos = Vec2::new(FIELD_HALF_X - 0.01, 0.0);
        let robot = robot_at(pos);
        for _ in 0..(STUCK_TICKS + 5) {
            let mut cmd = cmd_at(0.6, 0.3);
            rec.guard(&mut cmd, &robot);
            assert_eq!(cmd.vx, 0.6, "deshabilitado no debe modificar el comando");
            assert_eq!(cmd.vy, 0.3);
        }
    }
}
