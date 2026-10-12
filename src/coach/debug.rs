//! Depuración del coach por decisión: lo que el coach ya calcula para decidir, para la
//! foto del tick, el CSV de partido y el panel "Por qué" de la GUI. Tipos puros: no
//! deciden nada (ver `heuristic_coach::depuracion`, que los arma).

use super::adaptation::{Lane, Rate};
use super::{Play, Role};
use crate::skills::SkillId;
use glam::Vec2;

/// Lo que explica la última decisión del coach.
#[derive(Debug, Clone, PartialEq)]
pub struct CoachDebug {
    /// Número de la decisión (el coach decide cada `COACH_DECISION_PERIOD` ticks).
    pub decision: u64,
    pub jugada: Play,
    pub atacante: Option<i32>,
    pub arquero: i32,
    /// Las 5 opciones del atacante, si en esta decisión eligió entre ellas.
    pub opciones: Option<OpcionesAtacante>,
    /// Skill aplicada al atacante en esta decisión (la completa `CoachDecider`).
    pub skill_atacante: Option<SkillId>,
    /// Objetivo del atacante (punto y tipo), si en esta decisión eligió entre las opciones.
    pub objetivo: Option<(Vec2, TipoObjetivo)>,
    /// Rol de cada robot propio y su tiempo de llegada (s), si se midió en esta decisión.
    pub roles: Vec<RolRobot>,
    /// Condición para cambiar de atacante, si hay atacante y otro candidato.
    pub cambio: Option<CambioAtacante>,
    pub trabada: Trabada,
    pub adaptacion: AdaptacionDebug,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RolRobot {
    pub id: i32,
    pub rol: Option<Role>,
    pub llegada_s: Option<f32>,
}

/// Las 5 opciones del atacante en una decisión.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OpcionesAtacante {
    /// En el orden de `STRIKER_OPTIONS`, después del ajuste por pelota trabada.
    pub puntajes: [(SkillId, f32); 5],
    /// La de mayor puntaje con el bono por seguir igual.
    pub ganadora: SkillId,
    /// La que quedó después del compromiso mínimo.
    pub aplicada: SkillId,
    /// Bono por seguir igual: opción y valor (solo si esa opción tenía puntaje > 0).
    pub bono: Option<(SkillId, f32)>,
    pub compromiso: Compromiso,
    pub tiro: Tiro,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Compromiso {
    pub skill: SkillId,
    /// Decisiones que le quedan antes de poder cambiar (0 = puede cambiar).
    pub restantes: u32,
    pub minimo: u32,
    /// El compromiso retuvo la opción anterior frente a la ganadora.
    pub retuvo: bool,
}

/// Factores del tiro (ShootPush) con el objetivo elegido.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tiro {
    pub factible: bool,
    pub tapado: bool,
    /// Factor del puntaje por línea tapada (1 si está libre).
    pub factor: f32,
    pub carril: Lane,
}

/// De dónde salió el objetivo del atacante.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TipoObjetivo {
    /// Palo lejano del arco rival (arquero rival corrido).
    PaloLejano,
    /// Arco rival (centro, o el carril que prefiere la adaptación).
    Arco,
    /// Pelota en el fondo rival: sacarla al frente del arco.
    FondoRival,
    /// Pelota en el fondo propio: despejar por la banda.
    FondoPropio,
    /// Pelota en la banda: conducir a lo largo.
    Banda,
    /// Pelota en nuestra mitad: despeje.
    Despeje,
    /// Pelota trabada: pase al compañero.
    Pase,
}

/// Condición para cambiar de atacante: otro jugador de campo llega claramente antes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CambioAtacante {
    pub actual: i32,
    pub t_actual: f32,
    pub mejor: i32,
    pub t_mejor: f32,
    /// Cambia si `t_mejor < t_actual · ganancia`.
    pub ganancia: f32,
    /// Decisiones desde el último cambio, y el mínimo para poder cambiar.
    pub decisiones: u32,
    pub minimo: u32,
}

impl CambioAtacante {
    /// Tiempo por debajo del cual otro jugador pasa a ser el atacante (s).
    pub fn umbral_s(&self) -> f32 {
        self.t_actual * self.ganancia
    }
}

/// Pelota trabada junto al atacante, con los segundos en ese estado.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Trabada {
    Libre,
    /// Empuja sin que la pelota se mueva.
    Contando(f32),
    /// Sacándola con el giro.
    Pateando(f32),
    /// Giro prohibido después de uno que no la soltó.
    GiroProhibido(f32),
}

/// Adaptación al rival: tasas de éxito con su número de muestras.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AdaptacionDebug {
    pub activa: bool,
    /// Tiros por carril: arriba, centro, abajo.
    pub tiros: [Rate; 3],
    pub duelos: Rate,
}
