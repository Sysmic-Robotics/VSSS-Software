//! Capa de recuperación de atasco (wrapper del control loop).
//!
//! Envuelve la salida de las skills (`MotionCommand`) sin modificarlas: si el robot se
//! atasca (se le comanda avanzar pero su pose medida no avanza durante una ventana de
//! ticks — p. ej. un empujón de rival o la pared física), ejecuta una maniobra de escape
//! wall-aware (reversa alejándose de la pared + giro hacia el interior). Es solo
//! recuperación de último recurso: la navegación ya va hacia el destino.
//!
//! Recibe el comando ya filtrado por el `ZoneGuard` y su salida vuelve a pasar por él
//! (`control_loop::apply_reflexes`).
//!
//! Es un reflejo de bajo nivel común a coach clásico y RL; conmutable por
//! `VSSL_BORDER_RECOVERY` (default on). No toca manual ni estop.

use std::collections::{HashMap, VecDeque};

use glam::Vec2;

use super::commands::MotionCommand;
use crate::world::RobotState;

// Geometría física del campo (m): el centro del robot puede acercarse hasta
// ~pared − radio. Se mide "cerca del borde" con una franja `BORDER_BAND`.
const FIELD_HALF_X: f32 = 0.75;
const FIELD_HALF_Y: f32 = 0.65;
/// Franja (m) desde la pared física dentro de la cual se considera "junto al borde".
const BORDER_BAND: f32 = 0.10;

/// Ventana de detección (ticks, ~0.5 s @ 60 Hz): en cada uno de estos ticks se le
/// comandó avanzar y el desplazamiento neto de la pose medida en la ventana fue chico.
const STUCK_TICKS: usize = 30;
/// Duración de la maniobra de escape una vez disparada (ticks).
const RECOVERY_TICKS: u32 = 25;
/// Velocidad del cuerpo comandada (proyección del comando al heading, m/s) desde la que
/// "se le pidió avanzar". Girar en el lugar no cuenta, aunque el comando tenga
/// componente en mundo.
const CMD_BODY_EPS: f64 = 0.10;
/// Desplazamiento NETO máximo de la pose medida en la ventana (m) para declarar atasco:
/// 0.03 m en 0.5 s = 0.06 m/s. Es la distancia entre la primera y la última pose de la
/// ventana, no el largo del camino: el jitter del ruido de cámara no se acumula. Se
/// mide sobre la pose y no sobre la velocidad del EKF, que con el tracker apagado
/// (`VSSL_TRACKER=off`) vale cero.
const STUCK_NET_DISP: f32 = 0.03;
/// Rapidez (m/s) de la reversa de escape (world frame, alejándose de la pared).
const ESCAPE_SPEED: f64 = 0.4;
/// Velocidad angular (rad/s) del giro de escape hacia el interior.
const ESCAPE_OMEGA: f64 = 6.0;

/// Estado de recuperación por robot.
#[derive(Default, Clone)]
struct RecoveryState {
    /// Poses medidas de los últimos ticks consecutivos con avance comandado.
    window: VecDeque<Vec2>,
    recovery_ticks: u32,
    /// Dirección de escape fija durante la ventana de recuperación (unit, world).
    escape_dir: Vec2,
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

    /// Aplica detección + recuperación a un comando, dado el estado medido del robot.
    /// Modifica `cmd` en el lugar y devuelve `true` si en este tick lo reemplazó por la
    /// maniobra de escape. Función central, testeable sin el loop.
    pub fn guard(&mut self, cmd: &mut MotionCommand, robot: &RobotState) -> bool {
        if !self.enabled {
            return false;
        }
        let pos = robot.position;
        let st = self.states.entry((robot.team, robot.id)).or_default();

        // Ventana de escape activa: forzar la maniobra, ignorar el comando de la skill.
        if st.recovery_ticks > 0 {
            st.recovery_ticks -= 1;
            apply_escape(cmd, st.escape_dir, robot);
            return true;
        }

        // Detección por resultado: avance comandado en el heading, pose que no avanza.
        let th = robot.orientation;
        let v_body = (cmd.vx * th.cos() + cmd.vy * th.sin()).abs();
        if !v_body.is_finite() || v_body <= CMD_BODY_EPS {
            st.window.clear();
            return false;
        }
        st.window.push_back(pos);
        if st.window.len() > STUCK_TICKS {
            st.window.pop_front();
        }
        let stuck = st.window.len() == STUCK_TICKS
            && (pos - st.window[0]).length() < STUCK_NET_DISP;
        if !stuck {
            // Sin atasco: pass-through.
            return false;
        }
        st.window.clear();
        st.recovery_ticks = RECOVERY_TICKS;
        st.escape_dir = outward_from_nearest_wall(pos);
        apply_escape(cmd, st.escape_dir, robot);
        true
    }

    /// Aplica `guard` a todos los comandos del equipo propio con comando autónomo,
    /// saltando los robots en control manual y los que no tienen estado de visión.
    /// Devuelve, paralelo a `commands`, si cada uno quedó en maniobra de escape.
    pub fn guard_commands(
        &mut self,
        commands: &mut [MotionCommand],
        world: &crate::world::World,
        manual_keys: &std::collections::HashSet<(i32, i32)>,
        own_team: i32,
    ) -> Vec<bool> {
        let mut escaping = vec![false; commands.len()];
        if !self.enabled {
            return escaping;
        }
        for (cmd, esc) in commands.iter_mut().zip(escaping.iter_mut()) {
            if cmd.team != own_team || manual_keys.contains(&(cmd.team, cmd.id)) {
                continue;
            }
            if let Some(robot) = world.get_robot_state(cmd.id, cmd.team) {
                *esc = self.guard(cmd, robot);
            }
        }
        escaping
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

    /// Corre `ticks` ticks con el comando `(vx, vy)` y la pose que devuelve `pose(k)`;
    /// devuelve el primer tick con escape, si hubo.
    fn first_escape(
        ticks: usize,
        vx: f64,
        vy: f64,
        mut pose: impl FnMut(usize) -> Vec2,
    ) -> Option<usize> {
        let mut rec = BorderRecovery::new(true);
        (0..ticks).find(|&k| {
            let robot = robot_at(pose(k));
            let mut cmd = cmd_at(vx, vy);
            rec.guard(&mut cmd, &robot)
        })
    }

    // ── Sin prevención target-ciega ───────────────────────────────────────────

    #[test]
    fn near_border_not_stuck_is_passthrough() {
        // Junto al borde +x con velocidad hacia la pared, pero avanzando (no atascado):
        // el comando pasa SIN recorte.
        let mut rec = BorderRecovery::new(true);
        let mut pos = Vec2::new(0.66, 0.0);
        for _ in 0..10 {
            let robot = robot_at(pos);
            let mut cmd = cmd_at(0.5, 0.2);
            assert!(!rec.guard(&mut cmd, &robot));
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
        assert!(rec.guard(&mut cmd, &robot));
        assert!(cmd.vx < 0.0, "el escape debe alejar de la pared +x: {}", cmd.vx);
    }

    #[test]
    fn blocked_robot_escapes_after_the_window() {
        // Bloqueado con v del cuerpo 0.3 m/s: el escape llega en el tick 30 (índice 29).
        let k = first_escape(60, 0.3, 0.0, |_| Vec2::new(-0.2, 0.1));
        assert_eq!(k, Some(STUCK_TICKS - 1));
    }

    #[test]
    fn advancing_robot_is_passthrough() {
        // Comando 0.3 m/s, la pose avanza a 0.2 m/s durante 60 ticks: sin escape.
        assert_eq!(
            first_escape(60, 0.3, 0.0, |k| Vec2::new(-0.2 + 0.2 * k as f32 / 60.0, 0.0)),
            None
        );
    }

    #[test]
    fn turning_in_place_is_not_stuck() {
        // Comando con componente en mundo (0.264 m/s, el coupling viejo) pero
        // perpendicular al heading: no hay avance comandado, no hay atasco.
        assert_eq!(first_escape(90, 0.0, 0.264, |_| Vec2::new(0.1, 0.1)), None);
    }

    #[test]
    fn accelerating_from_rest_is_not_stuck() {
        // Comando 1.2 m/s, la pose acelera a 1.2 m/s² (lo medido en FIRASim).
        assert_eq!(
            first_escape(60, 1.2, 0.0, |k| {
                let t = k as f32 / 60.0;
                Vec2::new(-0.5 + 0.5 * 1.2 * t * t, 0.0)
            }),
            None
        );
    }

    #[test]
    fn tracker_off_does_not_cause_escapes() {
        // `velocity` del estado en cero (tracker apagado) pero la pose avanza a 0.3 m/s:
        // el detector mira la pose, no la velocidad del EKF.
        let mut rec = BorderRecovery::new(true);
        for k in 0..90 {
            let robot = robot_at(Vec2::new(-0.3 + 0.3 * k as f32 / 60.0, 0.0));
            assert_eq!(robot.velocity, Vec2::ZERO);
            let mut cmd = cmd_at(0.3, 0.0);
            assert!(!rec.guard(&mut cmd, &robot), "escape espurio en el tick {k}");
        }
    }

    #[test]
    fn camera_noise_does_not_hide_a_stuck_robot() {
        // Pose fija + ruido gaussiano con el σ del proxy (1.85 mm por eje): el
        // desplazamiento NETO de la ventana sigue chico y el atasco se detecta. Con el
        // largo del camino, el jitter sumaría varios cm y lo taparía.
        let sigma = crate::params::VisionParams::default().proxy_sigma_pos_m as f32;
        let mut rng = 0x2545_F491_4F6C_DD1Du64;
        let mut gauss = move || {
            // xorshift64* + Box-Muller (semilla fija → determinista).
            let mut next = || {
                rng ^= rng >> 12;
                rng ^= rng << 25;
                rng ^= rng >> 27;
                ((rng.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64)
                    .max(1e-12)
            };
            let (u1, u2) = (next(), next());
            ((-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()) as f32
        };
        let mut path = 0.0f32;
        let mut prev = Vec2::new(-0.2, 0.1);
        let k = first_escape(60, 0.3, 0.0, |_| {
            let p = Vec2::new(-0.2 + sigma * gauss(), 0.1 + sigma * gauss());
            path += (p - prev).length();
            prev = p;
            p
        });
        assert_eq!(k, Some(STUCK_TICKS - 1), "el ruido no debe tapar el atasco");
        assert!(path > STUCK_NET_DISP, "el largo del camino ({path:.3} m) sí lo habría tapado");
    }

    #[test]
    fn zero_command_is_passthrough_even_at_wall() {
        // Comando en cero (p. ej. estop): no hay "comandado a moverse" → sin escape.
        let mut rec = BorderRecovery::new(true);
        let robot = robot_at(Vec2::new(FIELD_HALF_X - 0.01, 0.0));
        for _ in 0..(STUCK_TICKS + 5) {
            let mut cmd = cmd_at(0.0, 0.0);
            assert!(!rec.guard(&mut cmd, &robot));
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
        let manual: std::collections::HashSet<(i32, i32)> = [(0, 0)].into_iter().collect();
        let mut cmds = vec![cmd_at(0.6, 0.0)];
        let esc = rec.guard_commands(&mut cmds, &world, &manual, 0);
        // Está en manual → no se toca, pese a estar al borde.
        assert_eq!(cmds[0].vx, 0.6, "robot manual no debe modificarse");
        assert_eq!(esc, vec![false]);
    }

    // ── Flag off ─────────────────────────────────────────────────────────────

    #[test]
    fn disabled_is_pure_passthrough() {
        let mut rec = BorderRecovery::new(false);
        let pos = Vec2::new(FIELD_HALF_X - 0.01, 0.0);
        let robot = robot_at(pos);
        for _ in 0..(STUCK_TICKS + 5) {
            let mut cmd = cmd_at(0.6, 0.3);
            assert!(!rec.guard(&mut cmd, &robot));
            assert_eq!(cmd.vx, 0.6, "deshabilitado no debe modificar el comando");
            assert_eq!(cmd.vy, 0.3);
        }
    }
}
