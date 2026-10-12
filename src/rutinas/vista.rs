//! Vista de una rutina para la foto de la GUI y el CSV, y las frases de estado (funciones
//! puras, con tests).

use super::formato::{Repeticiones, Rutina};
use super::resumen::{Resultado, ResumenCorrida};
use glam::Vec2;
use std::sync::Arc;

/// Fase de una corrida (D4 del design).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fase {
    Preparando,
    Lista,
    Ejecutando,
    Pausada,
    Terminada,
    Abortada,
}

impl Fase {
    /// Todavía mueve robots (no terminó ni se abortó).
    pub fn activa(self) -> bool {
        !matches!(self, Fase::Terminada | Fase::Abortada)
    }

    /// Para el CSV y el resumen.
    pub fn etiqueta(self) -> &'static str {
        match self {
            Fase::Preparando => "preparando",
            Fase::Lista => "lista",
            Fase::Ejecutando => "ejecutando",
            Fase::Pausada => "pausada",
            Fase::Terminada => "terminada",
            Fase::Abortada => "abortada",
        }
    }
}

/// Lo primero que le falta a la preparación para quedar lista.
#[derive(Debug, Clone, PartialEq)]
pub enum Falta {
    /// El robot todavía navega a su inicio.
    VaASuPuesto { robot: i32, puesto: String },
    /// Está en su inicio pero no orientado.
    SeOrienta { robot: i32 },
    /// En posición, pero todavía se mueve (o la ventana de quietud no se llenó).
    Quieto { robot: i32 },
    /// Un robot sin puesto está demasiado cerca del recorrido de un puesto.
    CercaDelRecorrido { robot: i32, distancia: f32, puesto: String },
}

impl Falta {
    /// En presente, para la frase de la preparación.
    pub fn presente(&self) -> String {
        match self {
            Falta::VaASuPuesto { robot, puesto } => format!("el robot {robot} va a su puesto ({puesto})"),
            Falta::SeOrienta { robot } => format!("el robot {robot} se orienta en su puesto"),
            Falta::Quieto { robot } => format!("esperando que el robot {robot} quede quieto"),
            Falta::CercaDelRecorrido { robot, distancia, puesto } => {
                format!("el robot {robot} está a {distancia:.2} m del recorrido de {puesto}")
            }
        }
    }

    /// Causa de aborto cuando la preparación no termina en `t` segundos.
    pub fn vencida(&self, t: f64) -> String {
        let t = segundos(t);
        match self {
            Falta::VaASuPuesto { robot, .. } => format!("el robot {robot} no llegó a su puesto en {t} s"),
            Falta::SeOrienta { robot } => format!("el robot {robot} no se orientó en su puesto en {t} s"),
            Falta::Quieto { robot } => format!("el robot {robot} no quedó quieto en su puesto en {t} s"),
            Falta::CercaDelRecorrido { .. } => format!("no quedó lista en {t} s: {}", self.presente()),
        }
    }
}

/// Segundos para una frase: sin decimales si es casi entero, con uno si no.
pub fn segundos(t: f64) -> String {
    if (t - t.round()).abs() < 0.05 { format!("{:.0}", t) } else { format!("{:.1}", t) }
}

/// "1 vuelta", "2 vueltas", "1.5 vueltas".
pub fn vueltas(v: f64) -> String {
    let n = if (v - v.round()).abs() < 1e-9 { format!("{:.0}", v) } else { format!("{v}") };
    if (v - 1.0).abs() < 1e-9 { "1 vuelta".to_string() } else { format!("{n} vueltas") }
}

/// Un robot asignado en el tick.
#[derive(Debug, Clone, PartialEq)]
pub struct RobotEnRutina {
    pub robot: i32,
    /// Índice del puesto en `rutina.puestos`.
    pub puesto: usize,
    /// Paso en curso (desde 0), en ejecución o en pausa; `None` en la preparación y
    /// cuando el puesto ya hizo su vuelta.
    pub paso: Option<usize>,
    pub total_pasos: usize,
    /// Tipo del paso en curso ("ir", "girar"…).
    pub accion: Option<&'static str>,
    /// Destino o punto mirado del paso en curso (en la preparación, el inicio).
    pub destino: Option<Vec2>,
}

/// Vista inmutable de la rutina en un tick.
#[derive(Debug, Clone, PartialEq)]
pub struct RutinaVista {
    pub rutina: Arc<Rutina>,
    pub fase: Fase,
    pub frase: String,
    pub velocidad_pct: u32,
    /// Repetición en curso (desde 1); 0 antes de empezar.
    pub repeticion: u32,
    pub repeticiones: Repeticiones,
    /// Un elemento por puesto en uso, en orden.
    pub robots: Vec<RobotEnRutina>,
    /// Al terminar o abortar.
    pub resumen: Option<Arc<ResumenCorrida>>,
}

impl RutinaVista {
    /// El robot asignado `id`, si lo está.
    pub fn de_robot(&self, id: i32) -> Option<&RobotEnRutina> {
        self.robots.iter().find(|r| r.robot == id)
    }

    /// Nombre del puesto `k`.
    pub fn nombre_puesto(&self, k: usize) -> &str {
        self.rutina.puestos.get(k).map_or("?", |p| p.nombre.as_str())
    }
}

/// "Vuelta 2 de 5" o "Vuelta 2 (bucle)".
fn vuelta_de(rep: u32, reps: Repeticiones) -> String {
    match reps {
        Repeticiones::Veces(n) => format!("Vuelta {rep} de {n}"),
        Repeticiones::Bucle => format!("Vuelta {rep} (bucle)"),
    }
}

/// Lo que hace el robot en el paso `paso` (desde 0), en frase corta.
pub fn verbo(accion: &str, paso: usize, orientando: bool) -> String {
    let n = paso + 1;
    match (accion, orientando) {
        ("ir", false) => format!("va al punto {n}"),
        ("ir", true) => format!("se orienta en el punto {n}"),
        ("mirar", _) => format!("mira al punto {n}"),
        ("girar", _) => "gira".to_string(),
        ("esperar", _) => "espera".to_string(),
        (otro, _) => otro.to_string(),
    }
}

/// Frase de la ejecución: "Vuelta 2 de 5, paso 3 de 8: el robot 0 gira".
pub fn frase_ejecucion(rep: u32, reps: Repeticiones, paso: usize, total: usize, robot: i32, que: &str) -> String {
    format!("{}, paso {} de {total}: el robot {robot} {que}", vuelta_de(rep, reps), paso + 1)
}

/// Frase de la pausa: "Pausada en la vuelta 2, paso 3 de 8".
pub fn frase_pausa(rep: u32, paso: usize, total: usize) -> String {
    format!("Pausada en la vuelta {rep}, paso {} de {total}", paso + 1)
}

/// Frase de la preparación.
pub fn frase_preparacion(falta: Option<&Falta>) -> String {
    match falta {
        Some(f) => format!("Preparando: {}", f.presente()),
        None => "Lista: robots en su puesto y quietos".to_string(),
    }
}

/// Frase del cierre, con el resumen.
pub fn frase_cierre(resumen: &ResumenCorrida) -> String {
    let n = resumen.vueltas.len();
    let tras = match n {
        0 => "sin vueltas completas".to_string(),
        1 => "tras 1 vuelta".to_string(),
        n => format!("tras {n} vueltas"),
    };
    match resumen.resultado {
        Resultado::Completada => match (n, resumen.duraciones().0) {
            (1, Some(m)) => format!("Terminada: 1 vuelta en {m:.1} s"),
            (_, Some(m)) => format!("Terminada: {n} vueltas, {m:.1} s por vuelta"),
            (_, None) => "Terminada".to_string(),
        },
        Resultado::Detenida => format!("Detenida por el usuario {tras}"),
        Resultado::Abortada => match resumen.causa.as_deref() {
            Some(CAUSA_USUARIO) | None => "Abortada por el usuario".to_string(),
            Some(c) => format!("Abortada: {c}"),
        },
    }
}

/// Causa del aborto del usuario (fuera de bucle).
pub const CAUSA_USUARIO: &str = "abortada por el usuario";
/// Causa de la detención del usuario en bucle.
pub const CAUSA_DETENIDA: &str = "detenida por el usuario";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn running_phrase() {
        assert_eq!(
            frase_ejecucion(2, Repeticiones::Veces(5), 2, 8, 0, &verbo("girar", 2, false)),
            "Vuelta 2 de 5, paso 3 de 8: el robot 0 gira"
        );
        assert_eq!(
            frase_ejecucion(3, Repeticiones::Bucle, 0, 3, 1, &verbo("ir", 0, false)),
            "Vuelta 3 (bucle), paso 1 de 3: el robot 1 va al punto 1"
        );
        assert_eq!(verbo("ir", 3, true), "se orienta en el punto 4");
        assert_eq!(verbo("mirar", 1, false), "mira al punto 2");
        assert_eq!(frase_pausa(2, 2, 8), "Pausada en la vuelta 2, paso 3 de 8");
    }

    #[test]
    fn preparation_phrases() {
        let f = Falta::VaASuPuesto { robot: 1, puesto: "A".to_string() };
        assert_eq!(frase_preparacion(Some(&f)), "Preparando: el robot 1 va a su puesto (A)");
        assert_eq!(f.vencida(20.0), "el robot 1 no llegó a su puesto en 20 s");
        let c = Falta::CercaDelRecorrido { robot: 2, distancia: 0.12, puesto: "A".to_string() };
        assert_eq!(c.presente(), "el robot 2 está a 0.12 m del recorrido de A");
        assert_eq!(c.vencida(33.3), "no quedó lista en 33.3 s: el robot 2 está a 0.12 m del recorrido de A");
        assert_eq!(frase_preparacion(None), "Lista: robots en su puesto y quietos");
    }

    #[test]
    fn seconds_and_turns() {
        assert_eq!(segundos(6.0), "6");
        assert_eq!(segundos(13.333), "13.3");
        assert_eq!(vueltas(1.0), "1 vuelta");
        assert_eq!(vueltas(2.0), "2 vueltas");
        assert_eq!(vueltas(1.5), "1.5 vueltas");
    }

    #[test]
    fn phase_labels() {
        assert!(Fase::Pausada.activa() && !Fase::Abortada.activa() && !Fase::Terminada.activa());
        assert_eq!(Fase::Ejecutando.etiqueta(), "ejecutando");
    }
}
