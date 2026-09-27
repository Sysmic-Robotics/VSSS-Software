//! Adaptación al rival por contadores (TIGERs: estadísticas online en vez de
//! aprendizaje en vivo). El coach registra qué pasa después de cada decisión y
//! ajusta en caliente:
//!
//! - **Éxito de tiro por carril** (superior / centro / inferior según la `y` de
//!   la pelota al iniciar el `ShootPush`): un tiro "sale bien" si la pelota llega
//!   frente al arco rival o entra; "sale mal" si un rival la toca, se aleja del
//!   arco o pasa el tiempo. La tasa (media móvil exponencial con prior 0.5)
//!   multiplica el score de `ShootPush` (`0.7..1.3`) y desplaza el punto de tiro
//!   hacia el carril que mejor funciona cuando el centro está fallando.
//! - **Resultado de las disputas** con `SpinKick`: quién se queda con la pelota
//!   1.5 s después; la tasa multiplica el score de `SpinKick`.
//!
//! Sin estado oculto raro: todo es observable (`summary()`) y se puede apagar
//! (`adapt_enabled = false` → factores 1.0).

use glam::Vec2;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Lane {
    Upper,
    Center,
    Lower,
}

impl Lane {
    pub fn of(y: f32, half_width: f32) -> Self {
        if y > half_width {
            Lane::Upper
        } else if y < -half_width {
            Lane::Lower
        } else {
            Lane::Center
        }
    }

    fn index(self) -> usize {
        match self {
            Lane::Upper => 0,
            Lane::Center => 1,
            Lane::Lower => 2,
        }
    }
}

/// Tasa de éxito con media móvil exponencial y conteo.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rate {
    pub value: f32,
    pub samples: u32,
}

impl Rate {
    pub const PRIOR: f32 = 0.5;

    fn new() -> Self {
        Self {
            value: Self::PRIOR,
            samples: 0,
        }
    }

    fn observe(&mut self, success: bool, alpha: f32) {
        let x = if success { 1.0 } else { 0.0 };
        self.value += alpha * (x - self.value);
        self.samples += 1;
    }
}

#[derive(Debug, Clone, Copy)]
enum Episode {
    Shot { lane: Lane, started: u64, moved: bool },
    Duel { started: u64 },
}

#[derive(Debug, Clone)]
pub struct Adaptation {
    pub enabled: bool,
    /// Peso de cada observación nueva (0..1).
    pub alpha: f32,
    /// Decisiones antes de dar un tiro por fallido (a 10 Hz: 25 = 2.5 s).
    pub shot_timeout: u64,
    /// Decisiones tras las que se evalúa una disputa (a 10 Hz: 15 = 1.5 s).
    pub duel_eval_after: u64,
    /// Mitad del ancho del carril central (m).
    pub lane_half_width: f32,
    shots: [Rate; 3],
    duels: Rate,
    episode: Option<Episode>,
    last_outcome: Option<(&'static str, bool)>,
}

impl Adaptation {
    pub fn new(enabled: bool, alpha: f32) -> Self {
        Self {
            enabled,
            alpha: alpha.clamp(0.01, 1.0),
            shot_timeout: 25,
            duel_eval_after: 15,
            lane_half_width: 0.15,
            shots: [Rate::new(); 3],
            duels: Rate::new(),
            episode: None,
            last_outcome: None,
        }
    }

    pub fn shot_rate(&self, lane: Lane) -> Rate {
        self.shots[lane.index()]
    }

    pub fn duel_rate(&self) -> Rate {
        self.duels
    }

    /// Último episodio cerrado: ("tiro"|"disputa", éxito).
    pub fn last_outcome(&self) -> Option<(&'static str, bool)> {
        self.last_outcome
    }

    /// Multiplicador del score de `ShootPush` para un tiro desde `lane`: 0.7..1.3.
    pub fn shot_factor(&self, lane: Lane) -> f32 {
        if !self.enabled {
            return 1.0;
        }
        0.7 + 0.6 * self.shots[lane.index()].value
    }

    /// Multiplicador del score de `SpinKick` en disputas: 0.7..1.3.
    pub fn spin_factor(&self) -> f32 {
        if !self.enabled {
            return 1.0;
        }
        0.7 + 0.6 * self.duels.value
    }

    /// Punto de tiro adaptado: si el carril por defecto viene fallando (tasa < 0.35
    /// con al menos 2 muestras) y otro carril rinde claramente mejor, apunta al
    /// palo de ese carril (`post_y`). Si no, deja `default_y`.
    pub fn preferred_aim_y(&self, default_y: f32, post_y: f32) -> f32 {
        if !self.enabled {
            return default_y;
        }
        let current = Lane::of(default_y, self.lane_half_width);
        let cur = self.shots[current.index()];
        if cur.samples < 2 || cur.value >= 0.35 {
            return default_y;
        }
        let mut best = current;
        let mut best_v = cur.value;
        for lane in [Lane::Upper, Lane::Center, Lane::Lower] {
            let r = self.shots[lane.index()];
            if r.value > best_v + 0.2 {
                best = lane;
                best_v = r.value;
            }
        }
        match best {
            Lane::Upper => post_y.abs(),
            Lane::Lower => -post_y.abs(),
            Lane::Center => 0.0,
        }
    }

    /// Registra el estado tras una decisión de juego abierto.
    ///
    /// - `striker_shooting`: el striker eligió `ShootPush` este tick.
    /// - `striker_spinning`: eligió `SpinKick`.
    /// - `own_nearest`, `opp_nearest`: distancia del robot propio / rival más
    ///   cercano a la pelota.
    /// - `s`: signo de ataque; `now`: contador de decisiones.
    #[allow(clippy::too_many_arguments)]
    pub fn observe(
        &mut self,
        now: u64,
        ball: Vec2,
        ball_vel: Vec2,
        s: f32,
        striker_shooting: bool,
        striker_spinning: bool,
        own_nearest: f32,
        opp_nearest: f32,
    ) {
        if !self.enabled {
            return;
        }
        let alpha = self.alpha;
        match self.episode {
            None => {
                if striker_shooting && ball.x * s > 0.0 {
                    self.episode = Some(Episode::Shot {
                        lane: Lane::of(ball.y, self.lane_half_width),
                        started: now,
                        moved: false,
                    });
                } else if striker_spinning {
                    self.episode = Some(Episode::Duel { started: now });
                }
            }
            Some(Episode::Shot { lane, started, moved }) => {
                let toward_goal = ball_vel.x * s;
                let moved = moved || toward_goal > 0.25;
                let in_front_of_goal = ball.x * s > 0.60 && ball.y.abs() < 0.30;
                let goal = ball.x * s > 0.75 && ball.y.abs() < 0.20;
                let opp_touch = opp_nearest < 0.10;
                let rolling_back = moved && toward_goal < -0.10;
                let timeout = now.saturating_sub(started) >= self.shot_timeout;
                let result = if goal || in_front_of_goal {
                    Some(true)
                } else if opp_touch || rolling_back || timeout {
                    Some(false)
                } else {
                    None
                };
                match result {
                    Some(ok) => {
                        self.shots[lane.index()].observe(ok, alpha);
                        self.last_outcome = Some(("tiro", ok));
                        self.episode = None;
                    }
                    None => {
                        self.episode = Some(Episode::Shot {
                            lane,
                            started,
                            moved,
                        });
                    }
                }
            }
            Some(Episode::Duel { started }) => {
                if now.saturating_sub(started) >= self.duel_eval_after {
                    let ours = own_nearest < 0.15 && own_nearest < opp_nearest;
                    let theirs = opp_nearest < 0.15 && opp_nearest <= own_nearest;
                    if ours || theirs {
                        self.duels.observe(ours, alpha);
                        self.last_outcome = Some(("disputa", ours));
                    }
                    self.episode = None;
                }
            }
        }
    }

    pub fn summary(&self) -> String {
        format!(
            "tiros sup {:.2} ({}) · centro {:.2} ({}) · inf {:.2} ({}) · disputas {:.2} ({})",
            self.shots[0].value,
            self.shots[0].samples,
            self.shots[1].value,
            self.shots[1].samples,
            self.shots[2].value,
            self.shots[2].samples,
            self.duels.value,
            self.duels.samples
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn adapt() -> Adaptation {
        Adaptation::new(true, 0.3)
    }

    #[test]
    fn lanes_split_by_y() {
        assert_eq!(Lane::of(0.3, 0.15), Lane::Upper);
        assert_eq!(Lane::of(0.0, 0.15), Lane::Center);
        assert_eq!(Lane::of(-0.2, 0.15), Lane::Lower);
    }

    #[test]
    fn successful_shot_raises_factor_and_failed_lowers_it() {
        let mut a = adapt();
        assert!((a.shot_factor(Lane::Center) - 1.0).abs() < 1e-6);
        // Tiro desde el centro que llega frente al arco → éxito.
        a.observe(0, Vec2::new(0.3, 0.0), Vec2::ZERO, 1.0, true, false, 0.05, 1.0);
        a.observe(1, Vec2::new(0.45, 0.0), Vec2::new(0.8, 0.0), 1.0, true, false, 0.2, 1.0);
        a.observe(2, Vec2::new(0.65, 0.05), Vec2::new(0.8, 0.0), 1.0, false, false, 0.4, 1.0);
        assert_eq!(a.last_outcome(), Some(("tiro", true)));
        assert!(a.shot_factor(Lane::Center) > 1.0);
        assert_eq!(a.shot_rate(Lane::Center).samples, 1);
        // Tiro desde arriba que toca un rival → fallo.
        a.observe(3, Vec2::new(0.3, 0.3), Vec2::ZERO, 1.0, true, false, 0.05, 1.0);
        a.observe(4, Vec2::new(0.4, 0.3), Vec2::new(0.5, 0.0), 1.0, true, false, 0.2, 0.05);
        assert_eq!(a.last_outcome(), Some(("tiro", false)));
        assert!(a.shot_factor(Lane::Upper) < 1.0);
        // Tiro que se queda sin resolver → fallo por tiempo.
        a.observe(10, Vec2::new(0.3, -0.3), Vec2::ZERO, 1.0, true, false, 0.05, 1.0);
        for t in 11..40 {
            a.observe(t, Vec2::new(0.35, -0.3), Vec2::ZERO, 1.0, false, false, 0.3, 1.0);
        }
        assert_eq!(a.last_outcome(), Some(("tiro", false)));
        assert!(a.shot_factor(Lane::Lower) < 1.0);
    }

    #[test]
    fn aim_moves_to_better_lane_when_center_fails() {
        let mut a = adapt();
        // Dos fallos por el centro, un éxito por arriba.
        for i in 0..2 {
            let t0 = i * 10;
            a.observe(t0, Vec2::new(0.3, 0.0), Vec2::ZERO, 1.0, true, false, 0.05, 1.0);
            a.observe(t0 + 1, Vec2::new(0.35, 0.0), Vec2::new(0.4, 0.0), 1.0, true, false, 0.2, 0.05);
        }
        a.observe(50, Vec2::new(0.3, 0.3), Vec2::ZERO, 1.0, true, false, 0.05, 1.0);
        a.observe(51, Vec2::new(0.65, 0.2), Vec2::new(0.8, 0.0), 1.0, false, false, 0.4, 1.0);
        assert!(a.shot_rate(Lane::Center).value < 0.35);
        assert!(a.shot_rate(Lane::Upper).value > a.shot_rate(Lane::Center).value + 0.2);
        assert_eq!(a.preferred_aim_y(0.0, 0.14), 0.14);
        // Desactivada: no cambia nada.
        let mut off = a.clone();
        off.enabled = false;
        assert_eq!(off.preferred_aim_y(0.0, 0.14), 0.0);
        assert_eq!(off.shot_factor(Lane::Center), 1.0);
    }

    #[test]
    fn duel_outcome_drives_spin_factor() {
        let mut a = adapt();
        a.observe(0, Vec2::ZERO, Vec2::ZERO, 1.0, false, true, 0.08, 0.09);
        for t in 1..16 {
            a.observe(t, Vec2::ZERO, Vec2::ZERO, 1.0, false, false, 0.08, 0.5);
        }
        assert_eq!(a.last_outcome(), Some(("disputa", true)));
        assert!(a.spin_factor() > 1.0);
        a.observe(20, Vec2::ZERO, Vec2::ZERO, 1.0, false, true, 0.08, 0.09);
        for t in 21..36 {
            a.observe(t, Vec2::ZERO, Vec2::ZERO, 1.0, false, false, 0.5, 0.08);
        }
        assert_eq!(a.last_outcome(), Some(("disputa", false)));
        assert_eq!(a.duel_rate().samples, 2);
    }
}
