//! Los dos jugadores del modo manual (`gui-manual-control`): el estado de cada uno
//! (rampa y giro corto en curso), el mapeo arcade de sus teclas, el sentido del giro
//! corto según la pelota y la regla "un robot, un jugador". Todo puro: `DebugGui` lo
//! cablea y lo prueban los tests sin iced.
//!
//! El robot de cada jugador vive en `DebugGui`: el del jugador 1 es el robot
//! seleccionado y el del jugador 2, un robot propio opcional.

use super::keys::{Jugador, Tecla};
use super::slew;
use crate::control_loop::TickMode;
use glam::Vec2;
use std::collections::HashSet;
use std::time::{Duration, Instant};

/// Robot de un jugador: `(equipo, id)`.
pub type RobotJugador = (i32, i32);

/// Qué deja hacer el tick a los jugadores, según la foto y la parada local.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModoJuego {
    Juego,
    /// STOP del árbitro: el lazo anula la traslación; se gira con las teclas.
    Stop,
    /// Corte sin visión: el manual sigue, sin giro corto.
    SinVision,
    /// Parada de emergencia o HALT: todo en cero.
    Detenido,
}

/// El modo del tick para los jugadores: la parada local manda aunque la foto todavía no
/// la traiga.
pub fn modo_juego(modo_foto: TickMode, parada_local: bool) -> ModoJuego {
    if parada_local {
        return ModoJuego::Detenido;
    }
    match modo_foto {
        TickMode::Play => ModoJuego::Juego,
        TickMode::Stop => ModoJuego::Stop,
        TickMode::SinVision => ModoJuego::SinVision,
        TickMode::Halt => ModoJuego::Detenido,
    }
}

/// Escalas y paso de la rampa, comunes a los dos jugadores.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Escalas {
    /// Velocidad lineal máxima (m/s).
    pub lin: f64,
    /// Velocidad angular máxima (rad/s).
    pub ang: f64,
    /// Paso máximo de la rampa por tick: lineal (m/s) y angular (rad/s).
    pub paso_lin: f64,
    pub paso_ang: f64,
}

/// Intención de un jugador: adelante (+1), atrás (−1) y giro CCW (+1) o CW (−1). Las
/// teclas opuestas se cancelan.
pub fn intencion(teclas: &HashSet<Tecla>, jugador: Jugador) -> (f64, f64) {
    let t = jugador.teclas();
    let eje = |pos: Tecla, neg: Tecla| {
        f64::from(u8::from(teclas.contains(&pos))) - f64::from(u8::from(teclas.contains(&neg)))
    };
    (eje(t.adelante, t.atras), eje(t.giro_ccw, t.giro_cw))
}

/// Marco robot (arcade): avance según el heading `theta`, convertido a marco mundo.
/// `vx = v·cosθ`, `vy = v·sinθ`; con θ no finita, θ = 0.
pub fn arcade(adelante: f64, giro: f64, lin: f64, ang: f64, theta: f64) -> (f64, f64, f64) {
    let theta = if theta.is_finite() { theta } else { 0.0 };
    let v = adelante * lin;
    (v * theta.cos(), v * theta.sin(), giro * ang)
}

/// Dirección del ataque: la del arco que ataca el equipo del robot. Para el equipo
/// propio, el opuesto del arco propio; para un rival (en simulación), el arco propio.
pub fn arco_atacado(equipo_robot: i32, own_team: i32, own_goal: Vec2) -> Vec2 {
    if equipo_robot == own_team { -own_goal } else { own_goal }
}

/// Sentido del giro corto (+1 = CCW, −1 = CW) para mandar la pelota hacia `arco`: con
/// `d` la dirección de la pelota al arco, `signo((pelota − robot).perp_dot(d))`, con el
/// empate en CCW. Es la geometría de `SpinKickSkill::contact_centers`: con giro CCW, el
/// punto del robot que toca la pelota la empuja con ω·perp(pelota − centro).
pub fn sentido_giro(robot: Vec2, pelota: Vec2, arco: Vec2) -> f64 {
    let d = (arco - pelota).try_normalize().unwrap_or(Vec2::new(signo(arco.x), 0.0));
    if (pelota - robot).perp_dot(d) >= 0.0 { 1.0 } else { -1.0 }
}

fn signo(x: f32) -> f32 {
    if x < 0.0 { -1.0 } else { 1.0 }
}

/// Giro corto en curso.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Giro {
    /// rad/s, con signo.
    omega: f64,
    hasta: Instant,
}

/// Estado de un jugador: su rampa `(vx, vy, ω)` y el giro corto en curso.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct EstadoJugador {
    rampa: (f64, f64, f64),
    giro: Option<Giro>,
}

impl EstadoJugador {
    /// Empieza un giro corto de `omega` (rad/s, con signo) durante `duracion`. Lo llama
    /// `update` en el flanco de E o Enter, si se cumplen las condiciones de inicio.
    pub fn iniciar_giro(&mut self, omega: f64, ahora: Instant, duracion: Duration) {
        self.giro = Some(Giro { omega, hasta: ahora + duracion });
        self.rampa = (0.0, 0.0, 0.0);
    }

    #[cfg(test)]
    pub fn girando(&self) -> bool {
        self.giro.is_some()
    }

    /// Suelta todo: rampa en cero y sin giro (pérdida de foco, salida del modo manual,
    /// cambio de robot).
    pub fn soltar(&mut self) {
        *self = Self::default();
    }

    /// El comando del tick, en marco mundo, con la rampa propia del jugador.
    pub fn paso(
        &mut self,
        intencion: (f64, f64),
        escalas: &Escalas,
        theta: f64,
        ahora: Instant,
        modo: ModoJuego,
    ) -> (f64, f64, f64) {
        match modo {
            ModoJuego::Detenido => {
                self.soltar();
                return (0.0, 0.0, 0.0);
            }
            ModoJuego::Stop | ModoJuego::SinVision => {
                if self.giro.take().is_some() {
                    self.rampa = (0.0, 0.0, 0.0);
                }
            }
            ModoJuego::Juego => {}
        }
        if let Some(g) = self.giro {
            if ahora < g.hasta {
                return (0.0, 0.0, g.omega);
            }
            // Terminó: ω a cero y la rampa arranca desde cero.
            self.giro = None;
            self.rampa = (0.0, 0.0, 0.0);
        }
        let (adelante, giro) = intencion;
        let (mut tx, mut ty, tw) = arcade(adelante, giro, escalas.lin, escalas.ang, theta);
        let (cx, cy, cw) = self.rampa;
        let (mut cx, mut cy) = (cx, cy);
        if modo == ModoJuego::Stop {
            // El lazo anula la traslación: que al volver al juego arranque desde cero.
            (tx, ty, cx, cy) = (0.0, 0.0, 0.0, 0.0);
        }
        self.rampa = (
            slew(cx, tx, escalas.paso_lin),
            slew(cy, ty, escalas.paso_lin),
            slew(cw, tw, escalas.paso_ang),
        );
        self.rampa
    }
}

/// "Un robot, un jugador", por `(equipo, id)`. Devuelve los robots nuevos de los dos
/// jugadores:
/// - el jugador 1 toma `robot`; si era del jugador 2, el 2 queda sin robot;
/// - el jugador 2 toma `robot`, salvo que sea el del jugador 1: entonces no cambia nada.
pub fn asignar(
    j1: RobotJugador,
    j2: Option<RobotJugador>,
    jugador: Jugador,
    robot: RobotJugador,
) -> (RobotJugador, Option<RobotJugador>) {
    match jugador {
        Jugador::Uno => (robot, j2.filter(|r| *r != robot)),
        Jugador::Dos if robot == j1 => (j1, j2),
        Jugador::Dos => (j1, Some(robot)),
    }
}

/// Los robots que dejaron de tener jugador al pasar de `antes` a `despues`.
pub fn soltados(
    antes: (RobotJugador, Option<RobotJugador>),
    despues: (RobotJugador, Option<RobotJugador>),
) -> Vec<RobotJugador> {
    let nuevos = [Some(despues.0), despues.1];
    [Some(antes.0), antes.1]
        .into_iter()
        .flatten()
        .filter(|r| !nuevos.contains(&Some(*r)))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::keys::Flecha;
    use std::f64::consts::{FRAC_PI_2, PI};

    const ESC: Escalas = Escalas { lin: 0.3, ang: 3.0, paso_lin: 0.066, paso_ang: 0.396 };

    fn teclas(t: &[Tecla]) -> HashSet<Tecla> {
        t.iter().copied().collect()
    }

    fn paso(j: &mut EstadoJugador, int: (f64, f64), theta: f64, modo: ModoJuego) -> (f64, f64, f64) {
        j.paso(int, &ESC, theta, Instant::now(), modo)
    }

    #[test]
    fn each_player_reads_only_its_keys() {
        let t = teclas(&[Tecla::Letra('w'), Tecla::Flecha(Flecha::Izquierda)]);
        assert_eq!(intencion(&t, Jugador::Uno), (1.0, 0.0));
        assert_eq!(intencion(&t, Jugador::Dos), (0.0, 1.0));
        let opuestas = teclas(&[Tecla::Letra('w'), Tecla::Letra('s'), Tecla::Letra('a'), Tecla::Letra('d')]);
        assert_eq!(intencion(&opuestas, Jugador::Uno), (0.0, 0.0));
        assert_eq!(intencion(&HashSet::new(), Jugador::Dos), (0.0, 0.0));
        let t = teclas(&[Tecla::Letra('d'), Tecla::Flecha(Flecha::Abajo)]);
        assert_eq!(intencion(&t, Jugador::Uno), (0.0, -1.0));
        assert_eq!(intencion(&t, Jugador::Dos), (-1.0, 0.0));
    }

    #[test]
    fn arcade_follows_the_heading_and_ignores_a_bad_theta() {
        let (vx, vy, w) = arcade(1.0, 0.0, 0.5, 3.0, 0.0);
        assert!((vx - 0.5).abs() < 1e-9 && vy.abs() < 1e-9 && w == 0.0);
        let (vx, vy, _) = arcade(1.0, 0.0, 0.5, 3.0, FRAC_PI_2);
        assert!(vx.abs() < 1e-9 && (vy - 0.5).abs() < 1e-9);
        assert_eq!(arcade(0.0, 1.0, 0.5, 3.0, 1.2).2, 3.0, "A y la flecha izquierda: CCW");
        let (vx, vy, w) = arcade(1.0, -1.0, 0.5, 3.0, f64::NAN);
        assert!(vx.is_finite() && vy.is_finite() && w.is_finite());
        assert!((vx - 0.5).abs() < 1e-9, "θ no finita → 0");
    }

    #[test]
    fn a_frozen_theta_is_used_as_is() {
        // El robot dejó de verse mirando a −X: la GUI usa esa θ, la misma del lazo, y el
        // avance proyectado sobre el heading (command_to_vw) sale positivo.
        let mut j = EstadoJugador::default();
        let mut c = (0.0, 0.0, 0.0);
        for _ in 0..20 {
            c = paso(&mut j, (1.0, 0.0), PI, ModoJuego::SinVision);
        }
        let v = c.0 * PI.cos() + c.1 * PI.sin();
        assert!((v - ESC.lin).abs() < 1e-9, "{c:?}");
    }

    #[test]
    fn independent_ramps() {
        let mut j1 = EstadoJugador::default();
        let mut j2 = EstadoJugador::default();
        for _ in 0..20 {
            paso(&mut j1, (1.0, 0.0), 0.0, ModoJuego::Juego);
        }
        let a = paso(&mut j1, (1.0, 0.0), 0.0, ModoJuego::Juego);
        assert!((a.0 - ESC.lin).abs() < 1e-9, "el 1 llegó al tope");
        let b = paso(&mut j2, (1.0, 0.0), 0.0, ModoJuego::Juego);
        assert!((b.0 - ESC.paso_lin).abs() < 1e-9, "el 2 arranca desde cero: {b:?}");
        assert_eq!(paso(&mut j1, (1.0, 0.0), 0.0, ModoJuego::Juego), a, "el 1 no cambia");
    }

    #[test]
    fn releasing_brakes_by_the_ramp() {
        let mut j = EstadoJugador::default();
        for _ in 0..20 {
            paso(&mut j, (1.0, 0.0), 0.0, ModoJuego::Juego);
        }
        let c = paso(&mut j, (0.0, 0.0), 0.0, ModoJuego::Juego);
        assert!((c.0 - (ESC.lin - ESC.paso_lin)).abs() < 1e-9, "decrece por la rampa: {c:?}");
        let mut ultimo = c;
        for _ in 0..20 {
            ultimo = paso(&mut j, (0.0, 0.0), 0.0, ModoJuego::Juego);
        }
        assert_eq!(ultimo, (0.0, 0.0, 0.0));
    }

    #[test]
    fn stop_or_halt_keep_the_ramp_at_zero() {
        let mut j = EstadoJugador::default();
        for _ in 0..20 {
            paso(&mut j, (1.0, 1.0), 0.0, ModoJuego::Juego);
        }
        assert_eq!(paso(&mut j, (1.0, 1.0), 0.0, ModoJuego::Detenido), (0.0, 0.0, 0.0));
        let primero = paso(&mut j, (1.0, 0.0), 0.0, ModoJuego::Juego);
        assert!((primero.0 - ESC.paso_lin).abs() < 1e-9, "al soltar la parada, desde cero");

        let mut j = EstadoJugador::default();
        for _ in 0..20 {
            paso(&mut j, (1.0, 1.0), 0.0, ModoJuego::Juego);
        }
        let stop = paso(&mut j, (1.0, 1.0), 0.0, ModoJuego::Stop);
        assert_eq!((stop.0, stop.1), (0.0, 0.0), "en STOP la traslación queda en cero");
        assert_eq!(stop.2, ESC.ang, "el giro con las teclas sigue");
        let vuelta = paso(&mut j, (1.0, 0.0), 0.0, ModoJuego::Juego);
        assert!((vuelta.0 - ESC.paso_lin).abs() < 1e-9, "al volver al juego, desde cero");
    }

    #[test]
    fn short_spin_is_a_fixed_pulse_without_ramp() {
        let t0 = Instant::now();
        let dur = Duration::from_millis(300);
        let mut j = EstadoJugador::default();
        for _ in 0..20 {
            paso(&mut j, (1.0, 0.0), 0.0, ModoJuego::Juego);
        }
        j.iniciar_giro(12.0, t0, dur);
        for ms in [0, 100, 299] {
            let c = j.paso((1.0, 1.0), &ESC, 0.0, t0 + Duration::from_millis(ms), ModoJuego::Juego);
            assert_eq!(c, (0.0, 0.0, 12.0), "a {ms} ms las teclas se ignoran");
        }
        let fin = j.paso((1.0, 0.0), &ESC, 0.0, t0 + dur, ModoJuego::Juego);
        assert_eq!(fin.2, 0.0, "ω vuelve a 0");
        assert!((fin.0 - ESC.paso_lin).abs() < 1e-9, "la rampa arranca desde cero: {fin:?}");
        assert!(!j.girando());
    }

    #[test]
    fn the_spin_is_cut_by_stop_blind_cut_halt_and_release() {
        let t0 = Instant::now();
        for modo in [ModoJuego::Stop, ModoJuego::SinVision, ModoJuego::Detenido] {
            let mut j = EstadoJugador::default();
            j.iniciar_giro(-12.0, t0, Duration::from_millis(300));
            let c = j.paso((0.0, 0.0), &ESC, 0.0, t0, modo);
            assert_eq!(c, (0.0, 0.0, 0.0), "{modo:?}");
            assert!(!j.girando(), "{modo:?}");
        }
        let mut j = EstadoJugador::default();
        j.iniciar_giro(12.0, t0, Duration::from_millis(300));
        j.soltar();
        assert!(!j.girando());
        assert_eq!(j.paso((0.0, 0.0), &ESC, 0.0, t0, ModoJuego::Juego), (0.0, 0.0, 0.0));
    }

    #[test]
    fn one_player_spins_while_the_other_drives() {
        let t0 = Instant::now();
        let mut j1 = EstadoJugador::default();
        let mut j2 = EstadoJugador::default();
        let antes: Vec<_> = (0..3).map(|_| paso(&mut j2, (1.0, 0.0), 0.0, ModoJuego::Juego)).collect();
        j1.iniciar_giro(12.0, t0, Duration::from_millis(300));
        let _ = j1.paso((0.0, 0.0), &ESC, 0.0, t0, ModoJuego::Juego);
        let siguiente = paso(&mut j2, (1.0, 0.0), 0.0, ModoJuego::Juego);
        assert!(siguiente.0 > antes[2].0, "el 2 sigue acelerando con su rampa");
    }

    #[test]
    fn modes_from_the_snapshot_and_the_local_stop() {
        assert_eq!(modo_juego(TickMode::Play, false), ModoJuego::Juego);
        assert_eq!(modo_juego(TickMode::Play, true), ModoJuego::Detenido);
        assert_eq!(modo_juego(TickMode::Stop, false), ModoJuego::Stop);
        assert_eq!(modo_juego(TickMode::SinVision, false), ModoJuego::SinVision);
        assert_eq!(modo_juego(TickMode::Halt, false), ModoJuego::Detenido);
    }

    #[test]
    fn spin_direction_by_ball_side() {
        // Atacando hacia +X: pelota a la derecha (−y) → CCW; a la izquierda → CW.
        let arco = Vec2::new(0.75, 0.0);
        assert_eq!(sentido_giro(Vec2::ZERO, Vec2::new(0.0, -0.06), arco), 1.0);
        assert_eq!(sentido_giro(Vec2::ZERO, Vec2::new(0.0, 0.06), arco), -1.0);
        // Atacando hacia −X se invierte.
        let arco = Vec2::new(-0.75, 0.0);
        assert_eq!(sentido_giro(Vec2::ZERO, Vec2::new(0.0, -0.06), arco), -1.0);
        assert_eq!(sentido_giro(Vec2::ZERO, Vec2::new(0.0, 0.06), arco), 1.0);
        // Empate (pelota alineada con el arco) → CCW.
        assert_eq!(sentido_giro(Vec2::ZERO, Vec2::new(0.06, 0.0), Vec2::new(0.75, 0.0)), 1.0);
        // Dirección degenerada (pelota en el arco): d = (signo de ataque, 0), y vale lo
        // mismo que atacando hacia −X: la pelota en −y respecto del robot → CW.
        let arco = Vec2::new(-0.75, 0.0);
        assert_eq!(sentido_giro(Vec2::new(-0.75, 0.06), arco, arco), -1.0);
        assert_eq!(sentido_giro(Vec2::new(-0.75, -0.06), arco, arco), 1.0);
    }

    #[test]
    fn spin_direction_matches_spinkick_contact_centers() {
        let sk = crate::skills::tactical::SpinKickSkill::new(Vec2::ZERO);
        for (pelota, arco) in [
            (Vec2::new(0.1, 0.2), Vec2::new(0.75, 0.0)),
            (Vec2::new(-0.3, -0.4), Vec2::new(-0.75, 0.0)),
            (Vec2::new(0.5, 0.5), Vec2::new(0.75, 0.0)),
        ] {
            let d = (arco - pelota).normalize();
            let (ccw, cw) = sk.contact_centers(pelota, d);
            assert_eq!(sentido_giro(ccw, pelota, arco), 1.0, "robot en el centro CCW");
            assert_eq!(sentido_giro(cw, pelota, arco), -1.0, "robot en el centro CW");
        }
    }

    #[test]
    fn a_rival_robot_attacks_our_goal() {
        let own_goal = Vec2::new(-0.75, 0.0);
        assert_eq!(arco_atacado(0, 0, own_goal), Vec2::new(0.75, 0.0));
        assert_eq!(arco_atacado(1, 0, own_goal), own_goal);
        // El rival, con la pelota a su derecha mirando a nuestro arco (−X), gira CCW.
        let arco = arco_atacado(1, 0, own_goal);
        assert_eq!(sentido_giro(Vec2::ZERO, Vec2::new(0.0, 0.06), arco), 1.0);
    }

    #[test]
    fn one_robot_one_player() {
        let (a, b, c) = ((0, 0), (0, 1), (0, 2));
        assert_eq!(asignar(a, Some(b), Jugador::Uno, b), (b, None), "el 1 le quita el robot al 2");
        assert_eq!(asignar(a, Some(b), Jugador::Uno, c), (c, Some(b)));
        assert_eq!(asignar(a, Some(b), Jugador::Dos, a), (a, Some(b)), "Shift sobre el del 1: nada");
        assert_eq!(asignar(a, None, Jugador::Dos, c), (a, Some(c)));
        // El mismo id en el otro equipo no choca.
        assert_eq!(asignar(a, Some(b), Jugador::Uno, (1, 1)), ((1, 1), Some(b)));
    }

    #[test]
    fn released_robots() {
        let (a, b, c) = ((0, 0), (0, 1), (0, 2));
        assert_eq!(soltados((a, Some(b)), (b, None)), vec![a], "el 2 perdió b, pero b sigue con el 1");
        assert_eq!(soltados((a, Some(b)), (c, Some(b))), vec![a]);
        assert_eq!(soltados((a, Some(b)), (a, Some(c))), vec![b]);
        assert!(soltados((a, Some(b)), (a, Some(b))).is_empty());
    }
}
