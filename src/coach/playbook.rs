//! Estilos de juego abierto y su selección adaptativa durante el partido.
//!
//! Es el playbook de CMDragons (Bowling, Browning y Veloso, *Plays as Effective Multiagent
//! Plans Enabling Opponent-Adaptive Play Selection*, ICAPS 2004; campeones del American Open
//! 2003): no se modela al rival, se mide cómo le va a cada estilo contra él. La selección es de
//! expertos (Exp3): se elige un estilo con probabilidad proporcional a su peso y, al terminar
//! su episodio, el peso se multiplica por `m^(1/P(elegido))` y se normaliza. Multiplicadores de
//! competencia de CMDragons: 3/2 gol a favor, 2/3 gol en contra, 11/10 episodio completado,
//! 10/11 abortado. Aquí, completado = la pelota pasó más de la mitad del episodio en la mitad
//! rival.

use glam::Vec2;

/// Mitad del ancho de la boca del arco (m).
const GOAL_HALF_Y: f32 = 0.20;
/// |x| desde el que la pelota está dentro del arco (m).
const GOAL_LINE_X: f32 = 0.76;
/// Tope del exponente 1/P: con una probabilidad muy chica un solo resultado borraría al resto.
const MAX_EXPONENT: f64 = 10.0;
/// Peso mínimo tras normalizar (de una suma de 3): ningún estilo desaparece del todo.
const MIN_WEIGHT: f64 = 0.05;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Estilo {
    /// El de siempre: el apoyo cubre con la pelota en nuestra mitad y se ofrece en la rival.
    Equilibrado,
    /// El apoyo cubre la línea salvo que nuestro atacante tenga la pelota limpia en la mitad rival.
    Defensivo,
    /// El apoyo solo cubre en nuestro tercio, el atacante presiona siempre y, con la pelota
    /// disputada en la mitad rival, el apoyo también va a ella.
    Ofensivo,
}

impl Estilo {
    pub const TODOS: [Estilo; 3] = [Estilo::Equilibrado, Estilo::Defensivo, Estilo::Ofensivo];

    pub fn nombre(self) -> &'static str {
        match self {
            Estilo::Equilibrado => "equilibrado",
            Estilo::Defensivo => "defensivo",
            Estilo::Ofensivo => "ofensivo",
        }
    }

    pub fn from_nombre(s: &str) -> Option<Self> {
        Self::TODOS.into_iter().find(|e| e.nombre() == s)
    }
}

/// Cómo terminó el episodio de un estilo.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Resultado {
    GolAFavor,
    GolEnContra,
    Completado,
    Abortado,
}

impl Resultado {
    fn multiplicador(self) -> f64 {
        match self {
            Resultado::GolAFavor => 1.5,
            Resultado::GolEnContra => 2.0 / 3.0,
            Resultado::Completado => 1.1,
            Resultado::Abortado => 1.0 / 1.1,
        }
    }
}

pub struct Playbook {
    pesos: [f64; 3],
    vigente: usize,
    /// Probabilidad con que se eligió el vigente.
    prob: f64,
    /// Decisiones de juego abierto del episodio, y cuántas con la pelota en la mitad rival.
    decisiones: u32,
    en_ataque: u32,
    /// Decisiones que dura un episodio sin goles.
    largo: u32,
    /// Ya se contó el gol de la pelota que está en el arco (hasta que vuelva a la cancha).
    gol_contado: bool,
    rng: u64,
}

impl Playbook {
    pub fn new(largo_decisiones: u32, semilla: u64) -> Self {
        let mut pb = Self {
            pesos: [1.0; 3],
            vigente: 0,
            prob: 1.0 / 3.0,
            decisiones: 0,
            en_ataque: 0,
            largo: largo_decisiones.max(1),
            gol_contado: false,
            rng: semilla | 1,
        };
        pb.elegir();
        pb
    }

    pub fn vigente(&self) -> Estilo {
        Estilo::TODOS[self.vigente]
    }

    pub fn pesos(&self) -> [f64; 3] {
        self.pesos
    }

    /// Una decisión de juego abierto. `s` = signo de ataque. Si el episodio terminó, devuelve
    /// cómo (y ya eligió el estilo siguiente).
    pub fn observar(&mut self, ball: Vec2, s: f32) -> Option<Resultado> {
        let x = ball.x * s;
        if self.gol_contado {
            if x.abs() < 0.5 {
                self.gol_contado = false;
            }
            return None;
        }
        let resultado = if x.abs() > GOAL_LINE_X && ball.y.abs() < GOAL_HALF_Y {
            self.gol_contado = true;
            Some(if x > 0.0 { Resultado::GolAFavor } else { Resultado::GolEnContra })
        } else {
            self.decisiones += 1;
            if x > 0.0 {
                self.en_ataque += 1;
            }
            (self.decisiones >= self.largo).then(|| {
                if 2 * self.en_ataque > self.decisiones { Resultado::Completado } else { Resultado::Abortado }
            })
        };
        if let Some(r) = resultado {
            self.actualizar(r);
            self.elegir();
        }
        resultado
    }

    fn actualizar(&mut self, r: Resultado) {
        let exp = (1.0 / self.prob).min(MAX_EXPONENT);
        self.pesos[self.vigente] *= r.multiplicador().powf(exp);
        let suma: f64 = self.pesos.iter().sum();
        for w in &mut self.pesos {
            *w = (*w * 3.0 / suma).max(MIN_WEIGHT);
        }
        self.decisiones = 0;
        self.en_ataque = 0;
    }

    fn elegir(&mut self) {
        // xorshift64
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 7;
        self.rng ^= self.rng << 17;
        let u = (self.rng >> 11) as f64 / (1u64 << 53) as f64;
        let suma: f64 = self.pesos.iter().sum();
        let mut acum = 0.0;
        self.vigente = self.pesos.len() - 1;
        for (i, w) in self.pesos.iter().enumerate() {
            acum += w / suma;
            if u < acum {
                self.vigente = i;
                break;
            }
        }
        self.prob = self.pesos[self.vigente] / suma;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_goal_for_raises_the_style_and_a_goal_against_lowers_it() {
        let mut pb = Playbook::new(200, 7);
        let v = pb.vigente;
        let antes = pb.pesos()[v];
        assert_eq!(pb.observar(Vec2::new(0.78, 0.0), 1.0), Some(Resultado::GolAFavor));
        assert!(pb.pesos[v] > antes, "{:?}", pb.pesos);
        // La pelota sigue en el arco: no se cuenta dos veces.
        assert_eq!(pb.observar(Vec2::new(0.78, 0.0), 1.0), None);
        // Vuelve al centro (saque) y entra en el nuestro.
        assert_eq!(pb.observar(Vec2::ZERO, 1.0), None);
        let v = pb.vigente;
        let antes = pb.pesos()[v];
        assert_eq!(pb.observar(Vec2::new(-0.78, 0.05), 1.0), Some(Resultado::GolEnContra));
        assert!(pb.pesos[v] < antes, "{:?}", pb.pesos);
    }

    #[test]
    fn episodes_end_by_time_completed_in_the_rival_half_and_aborted_in_ours() {
        let mut pb = Playbook::new(10, 3);
        let r: Vec<_> = (0..10).map(|_| pb.observar(Vec2::new(0.3, 0.0), 1.0)).collect();
        assert_eq!(r[9], Some(Resultado::Completado));
        assert!(r[..9].iter().all(Option::is_none));
        let r: Vec<_> = (0..10).map(|_| pb.observar(Vec2::new(0.3, 0.0), -1.0)).collect();
        assert_eq!(r[9], Some(Resultado::Abortado));
    }

    #[test]
    fn a_style_that_keeps_conceding_is_dropped_but_never_vanishes() {
        // Arena: el estilo defensivo recibe goles, los otros completan sus episodios.
        let mut pb = Playbook::new(5, 11);
        let mut elegido = [0u32; 3];
        for _ in 0..300 {
            elegido[pb.vigente] += 1;
            if pb.vigente() == Estilo::Defensivo {
                pb.observar(Vec2::new(-0.78, 0.0), 1.0);
                pb.observar(Vec2::ZERO, 1.0);
            } else {
                for _ in 0..5 {
                    pb.observar(Vec2::new(0.3, 0.0), 1.0);
                }
            }
        }
        assert!(elegido[1] < 30, "{elegido:?}");
        assert!(pb.pesos().iter().all(|&w| w >= MIN_WEIGHT * 0.99), "{:?}", pb.pesos());
    }

    #[test]
    fn style_names_round_trip() {
        for e in Estilo::TODOS {
            assert_eq!(Estilo::from_nombre(e.nombre()), Some(e));
        }
        assert_eq!(Estilo::from_nombre("adaptativo"), None);
    }
}
