//! Formato de los archivos de rutina (`config/rutinas/*.json`) y su carga.
//!
//! Un archivo por rutina; el id es el nombre del archivo sin `.json`. La lectura va en
//! dos pasos: primero a tipos planos con `deny_unknown_fields` (una errata como
//! `"tolerancia"` es un error que nombra el campo) y después a los tipos de la rutina,
//! con mensajes en español que llevan el puesto y el número de paso. Cada paso se lee
//! aparte para poder decir dónde está el error.
//!
//! La carga devuelve la rutina aunque sea inválida (la GUI la muestra en gris con su
//! razón) junto con todas las razones: las de la lectura y las de la validación
//! (`super::validacion`). La GUI y la consola usan las mismas funciones.

use super::validacion::{ContextoValidacion, validar};
use glam::Vec2;
use serde::Deserialize;
use std::path::{Path, PathBuf};
use std::sync::Arc;

/// Carpeta de las rutinas (relativa al directorio de trabajo, como `config/team_params.json`).
pub const CARPETA_POR_DEFECTO: &str = "config/rutinas";
/// Variable de entorno que cambia la carpeta.
pub const CARPETA_ENV: &str = "VSSL_RUTINAS_DIR";

/// Los ocho tipos de paso, en el orden de la especificación.
pub const TIPOS_DE_PASO: [&str; 8] = [
    "ir",
    "mirar",
    "girar",
    "esperar",
    "seguir",
    "sincronizar",
    "empujar",
    "recibir",
];

/// Carpeta vigente: `VSSL_RUTINAS_DIR` o `config/rutinas`.
pub fn carpeta() -> PathBuf {
    match std::env::var(CARPETA_ENV) {
        Ok(p) if !p.trim().is_empty() => PathBuf::from(p.trim()),
        _ => PathBuf::from(CARPETA_POR_DEFECTO),
    }
}

/// Grupo de la rutina (sección de la lista en la GUI).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Grupo {
    Circuito,
    Coreografia,
    ConPelota,
}

impl Grupo {
    pub const TODOS: [Grupo; 3] = [Grupo::Circuito, Grupo::Coreografia, Grupo::ConPelota];

    fn de_texto(s: &str) -> Option<Self> {
        match s {
            "circuito" => Some(Grupo::Circuito),
            "coreografia" => Some(Grupo::Coreografia),
            "con_pelota" => Some(Grupo::ConPelota),
            _ => None,
        }
    }

    /// Título de la sección en la GUI.
    pub fn titulo(self) -> &'static str {
        match self {
            Grupo::Circuito => "Circuitos",
            Grupo::Coreografia => "Coreografías",
            Grupo::ConPelota => "Con pelota",
        }
    }
}

/// Cuántas veces se recorre la lista de pasos.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Repeticiones {
    Veces(u32),
    /// Hasta que el usuario la detiene.
    Bucle,
}

impl Repeticiones {
    /// Desde el texto de la consola: un entero ≥ 1 o `bucle`.
    pub fn de_texto(s: &str) -> Result<Self, String> {
        let s = s.trim();
        if s == "bucle" {
            return Ok(Repeticiones::Bucle);
        }
        match s.parse::<u32>() {
            Ok(n) if n >= 1 => Ok(Repeticiones::Veces(n)),
            _ => Err(format!("repeticiones `{s}`: un entero ≥ 1 o `bucle`")),
        }
    }

    pub fn texto(self) -> String {
        match self {
            Repeticiones::Veces(n) => n.to_string(),
            Repeticiones::Bucle => "bucle".to_string(),
        }
    }

    /// Más de una vuelta: el robot vuelve del último destino al primero.
    pub fn mas_de_una(self) -> bool {
        !matches!(self, Repeticiones::Veces(1))
    }
}

/// Posición y orientación (rad) en coordenadas de la cancha.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    pub pos: Vec2,
    pub orientacion: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sentido {
    Antihorario,
    Horario,
}

impl Sentido {
    /// +1 antihorario, −1 horario (convención de Spin: `target.x > 0` → CCW).
    pub fn signo(self) -> f64 {
        match self {
            Sentido::Antihorario => 1.0,
            Sentido::Horario => -1.0,
        }
    }
}

/// Duración de `girar` o `seguir`: vueltas o segundos (uno de los dos).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Duracion {
    Vueltas(f64),
    Tiempo(f64),
}

/// Figura de `seguir` (fase 2).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Figura {
    Circulo { centro: Vec2, radio: f32 },
    IdaVuelta { desde: Vec2, hasta: Vec2 },
}

impl Figura {
    /// Puntos de la figura para validarla (la circunferencia muestreada, o los extremos).
    pub fn muestras(&self) -> Vec<Vec2> {
        match *self {
            Figura::Circulo { centro, radio } => (0..72)
                .map(|i| {
                    let a = i as f32 * std::f32::consts::TAU / 72.0;
                    centro + radio * Vec2::new(a.cos(), a.sin())
                })
                .collect(),
            Figura::IdaVuelta { desde, hasta } => vec![desde, hasta],
        }
    }
}

/// Destino de `empujar` (fase 3).
#[derive(Debug, Clone, PartialEq)]
pub enum Hacia {
    Punto(Vec2),
    Puesto(String),
}

/// Lo que hace un paso.
#[derive(Debug, Clone, PartialEq)]
pub enum Accion {
    /// GoTo al punto y, con orientación (rad), FacePoint al final.
    Ir { punto: Vec2, orientacion: Option<f64> },
    /// FacePoint hacia el punto.
    Mirar { punto: Vec2 },
    /// Spin en el lugar.
    Girar { sentido: Sentido, duracion: Duracion },
    /// Hold.
    Esperar { tiempo_s: f64 },
    /// Fase 2: punto móvil paramétrico con el reloj común (desfase en rad).
    Seguir { figura: Figura, periodo_s: f64, duracion: Duracion, desfase: f64 },
    /// Fase 2: barrera con nombre.
    Sincronizar { barrera: String },
    /// Fase 3: la pelota hacia un punto o un compañero.
    Empujar { hacia: Hacia },
    /// Fase 3: esperar la pelota en un punto y controlarla.
    Recibir { punto: Vec2 },
}

impl Accion {
    /// Nombre del tipo, como en el archivo.
    pub fn tipo(&self) -> &'static str {
        match self {
            Accion::Ir { .. } => "ir",
            Accion::Mirar { .. } => "mirar",
            Accion::Girar { .. } => "girar",
            Accion::Esperar { .. } => "esperar",
            Accion::Seguir { .. } => "seguir",
            Accion::Sincronizar { .. } => "sincronizar",
            Accion::Empujar { .. } => "empujar",
            Accion::Recibir { .. } => "recibir",
        }
    }

    /// Fase de implementación en la que el paso se ejecuta (1, 2 o 3).
    pub fn fase(&self) -> u8 {
        match self {
            Accion::Ir { .. } | Accion::Mirar { .. } | Accion::Girar { .. } | Accion::Esperar { .. } => 1,
            Accion::Seguir { .. } | Accion::Sincronizar { .. } => 2,
            Accion::Empujar { .. } | Accion::Recibir { .. } => 3,
        }
    }

    /// Punto al que el paso lleva al robot, si lo traslada a un punto fijo.
    pub fn destino(&self) -> Option<Vec2> {
        match self {
            Accion::Ir { punto, .. } | Accion::Recibir { punto } => Some(*punto),
            _ => None,
        }
    }
}

/// Un paso con sus valores propios (`None` = el de `team_params.json → rutinas`).
#[derive(Debug, Clone, PartialEq)]
pub struct Paso {
    pub accion: Accion,
    pub tolerancia_m: Option<f32>,
    pub tolerancia_deg: Option<f64>,
    pub tiempo_max_s: Option<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Puesto {
    pub nombre: String,
    pub inicio: Pose,
    pub arquero: bool,
    pub pasos: Vec<Paso>,
}

impl Puesto {
    /// Destinos de los pasos que trasladan, en orden.
    pub fn destinos(&self) -> impl Iterator<Item = Vec2> + '_ {
        self.pasos.iter().filter_map(|p| p.accion.destino())
    }

    /// Recorrido ideal: el inicio y los destinos en orden. Con `vuelta` (más de una
    /// repetición) suma el tramo del último destino al primero.
    pub fn recorrido(&self, vuelta: bool) -> Vec<Vec2> {
        let mut pts = vec![self.inicio.pos];
        pts.extend(self.destinos());
        if vuelta && let Some(primero) = self.destinos().next() {
            pts.push(primero);
        }
        pts
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Rutina {
    pub id: String,
    pub nombre: String,
    pub grupo: Grupo,
    pub descripcion: String,
    pub robots_min: usize,
    pub robots_max: usize,
    /// Marca de la pelota (fase 3); `None` = sin pelota.
    pub pelota: Option<Vec2>,
    pub repeticiones: Repeticiones,
    pub puestos: Vec<Puesto>,
}

impl Rutina {
    /// "1 robot", "2 robots" o "2–3 robots".
    pub fn texto_robots(&self) -> String {
        match (self.robots_min, self.robots_max) {
            (1, 1) => "1 robot".to_string(),
            (a, b) if a == b => format!("{a} robots"),
            (a, b) => format!("{a}–{b} robots"),
        }
    }

    /// Los `empujar` de la rutina con su receptor y sus puntos. El n-ésimo `empujar` con
    /// `a_puesto` B (en el orden de los puestos y de sus pasos) se empareja con el n-ésimo
    /// `recibir` de B; la misma función sirve a la validación, al decisor y a la GUI.
    pub fn pases(&self) -> Vec<Pase> {
        let indice = |n: &str| self.puestos.iter().position(|p| p.nombre == n);
        let recibir_de = |j: usize| -> Vec<usize> {
            self.puestos[j]
                .pasos
                .iter()
                .enumerate()
                .filter(|(_, p)| matches!(p.accion, Accion::Recibir { .. }))
                .map(|(i, _)| i)
                .collect()
        };
        let punto_recibir = |(j, i): (usize, usize)| match self.puestos[j].pasos[i].accion {
            Accion::Recibir { punto } => punto,
            _ => unreachable!("emparejado con un `recibir`"),
        };
        let mut usados: Vec<usize> = vec![0; self.puestos.len()];
        let mut out = Vec::new();
        for (k, puesto) in self.puestos.iter().enumerate() {
            for (i, paso) in puesto.pasos.iter().enumerate() {
                let Accion::Empujar { hacia } = &paso.accion else { continue };
                let receptor = match hacia {
                    Hacia::Puesto(n) => indice(n).filter(|j| *j != k).and_then(|j| {
                        let r = recibir_de(j).get(usados[j]).map(|i| (j, *i));
                        usados[j] += 1;
                        r
                    }),
                    Hacia::Punto(_) => None,
                };
                let objetivo = match hacia {
                    Hacia::Punto(p) => Some(*p),
                    Hacia::Puesto(_) => receptor.map(punto_recibir),
                };
                // La pelota sale del último `recibir` del pasador antes del empuje, o de la
                // marca en la primera vuelta; desde la segunda, del último `recibir` de la
                // vuelta anterior.
                let recibe = |p: &Paso| match p.accion {
                    Accion::Recibir { punto } => Some(punto),
                    _ => None,
                };
                let antes = puesto.pasos[..i].iter().rev().find_map(recibe);
                let mut origenes: Vec<Vec2> = Vec::new();
                match antes {
                    Some(o) => origenes.push(o),
                    None => {
                        origenes.extend(self.pelota);
                        origenes.extend(puesto.pasos[i..].iter().rev().find_map(recibe));
                    }
                }
                out.push(Pase { de: (k, i), receptor, origenes, objetivo });
            }
        }
        out
    }
}

/// Un `empujar` de la rutina.
#[derive(Debug, Clone, PartialEq)]
pub struct Pase {
    /// Puesto y paso del `empujar`.
    pub de: (usize, usize),
    /// Puesto y paso del `recibir` emparejado (`None` con `hacia` o si falta el `recibir`).
    pub receptor: Option<(usize, usize)>,
    /// De dónde puede salir la pelota: la marca o un punto de `recibir` del pasador.
    pub origenes: Vec<Vec2>,
    /// Hacia dónde: el punto de `hacia` o el del `recibir` emparejado.
    pub objetivo: Option<Vec2>,
}

/// Distancia de `p` al tramo `a`–`b` (al punto si el tramo es nulo).
pub fn distancia_a_tramo(p: Vec2, a: Vec2, b: Vec2) -> f32 {
    let ab = b - a;
    let l2 = ab.length_squared();
    if l2 <= f32::EPSILON {
        return (p - a).length();
    }
    let t = ((p - a).dot(ab) / l2).clamp(0.0, 1.0);
    (p - (a + ab * t)).length()
}

/// Distancia de `p` a una polilínea (infinita si no tiene puntos).
pub fn distancia_a_recorrido(p: Vec2, pts: &[Vec2]) -> f32 {
    match pts {
        [] => f32::INFINITY,
        [a] => (p - *a).length(),
        _ => pts
            .windows(2)
            .map(|w| distancia_a_tramo(p, w[0], w[1]))
            .fold(f32::INFINITY, f32::min),
    }
}

/// Un archivo leído: la rutina (aunque sea inválida) y las razones por las que no se
/// puede correr. Sin razones, es válida.
#[derive(Debug, Clone)]
pub struct Entrada {
    pub id: String,
    pub ruta: PathBuf,
    /// `None` si el archivo no se pudo leer como rutina.
    pub rutina: Option<Arc<Rutina>>,
    pub razones: Vec<String>,
}

impl Entrada {
    /// La rutina, solo si es válida.
    pub fn valida(&self) -> Option<&Arc<Rutina>> {
        if self.razones.is_empty() { self.rutina.as_ref() } else { None }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Tipos del archivo (planos, sin campos desconocidos)
// ─────────────────────────────────────────────────────────────────────────────

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RutinaArchivo {
    nombre: String,
    grupo: String,
    descripcion: String,
    robots: RobotsArchivo,
    #[serde(default)]
    pelota: Option<PelotaArchivo>,
    #[serde(default)]
    repeticiones: Option<serde_json::Value>,
    puestos: Vec<PuestoArchivo>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RobotsArchivo {
    min: usize,
    max: usize,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PelotaArchivo {
    marca: PuntoArchivo,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
struct PuntoArchivo {
    x: f32,
    y: f32,
}

impl PuntoArchivo {
    fn vec(self) -> Vec2 {
        Vec2::new(self.x, self.y)
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PoseArchivo {
    x: f32,
    y: f32,
    orientacion_deg: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PuestoArchivo {
    nombre: String,
    inicio: PoseArchivo,
    #[serde(default)]
    arquero: bool,
    /// Cada paso se lee aparte (para ubicar el error).
    pasos: Vec<serde_json::Value>,
}

/// Un paso tal cual viene: el tipo y todos los campos posibles. La conversión verifica
/// que estén los del tipo y que no sobre ninguno.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PasoArchivo {
    paso: String,
    x: Option<f32>,
    y: Option<f32>,
    orientacion_deg: Option<f64>,
    sentido: Option<String>,
    vueltas: Option<f64>,
    tiempo_s: Option<f64>,
    figura: Option<String>,
    centro: Option<PuntoArchivo>,
    radio: Option<f32>,
    desde: Option<PuntoArchivo>,
    hasta: Option<PuntoArchivo>,
    periodo_s: Option<f64>,
    desfase_deg: Option<f64>,
    barrera: Option<String>,
    hacia: Option<PuntoArchivo>,
    a_puesto: Option<String>,
    tolerancia_m: Option<f32>,
    tolerancia_deg: Option<f64>,
    tiempo_max_s: Option<f64>,
}

impl PasoArchivo {
    /// Campos propios del tipo que vinieron (los comunes no cuentan).
    fn presentes(&self) -> Vec<&'static str> {
        [
            ("x", self.x.is_some()),
            ("y", self.y.is_some()),
            ("orientacion_deg", self.orientacion_deg.is_some()),
            ("sentido", self.sentido.is_some()),
            ("vueltas", self.vueltas.is_some()),
            ("tiempo_s", self.tiempo_s.is_some()),
            ("figura", self.figura.is_some()),
            ("centro", self.centro.is_some()),
            ("radio", self.radio.is_some()),
            ("desde", self.desde.is_some()),
            ("hasta", self.hasta.is_some()),
            ("periodo_s", self.periodo_s.is_some()),
            ("desfase_deg", self.desfase_deg.is_some()),
            ("barrera", self.barrera.is_some()),
            ("hacia", self.hacia.is_some()),
            ("a_puesto", self.a_puesto.is_some()),
        ]
        .into_iter()
        .filter_map(|(n, si)| si.then_some(n))
        .collect()
    }
}

/// Error de serde en español: "campo desconocido `x`", "falta el campo `x`" o el
/// mensaje original, con la línea y la columna si las hay.
fn traducir(e: &serde_json::Error) -> String {
    let m = e.to_string();
    let m = m.split(" at line ").next().unwrap_or(&m).to_string();
    let campo = |prefijo: &str| {
        m.strip_prefix(prefijo)
            .and_then(|r| r.split('`').nth(1))
            .map(str::to_string)
    };
    let base = if let Some(c) = campo("unknown field ") {
        format!("campo desconocido `{c}`")
    } else if let Some(c) = campo("missing field ") {
        format!("falta el campo `{c}`")
    } else {
        m.clone()
    };
    if e.line() > 0 {
        format!("{base} (línea {}, columna {})", e.line(), e.column())
    } else {
        base
    }
}

fn punto_de(tipo: &str, x: Option<f32>, y: Option<f32>) -> Result<Vec2, String> {
    match (x, y) {
        (Some(x), Some(y)) => Ok(Vec2::new(x, y)),
        _ => Err(format!("`{tipo}` necesita `x` e `y`")),
    }
}

fn duracion_de(tipo: &str, vueltas: Option<f64>, tiempo_s: Option<f64>) -> Result<Duracion, String> {
    match (vueltas, tiempo_s) {
        (Some(v), None) => Ok(Duracion::Vueltas(v)),
        (None, Some(t)) => Ok(Duracion::Tiempo(t)),
        (Some(_), Some(_)) => Err(format!("`{tipo}` lleva `vueltas` o `tiempo_s`: va uno solo de los dos")),
        (None, None) => Err(format!("`{tipo}` necesita `vueltas` o `tiempo_s`")),
    }
}

/// Convierte un paso plano al tipo de la rutina.
fn paso_de(p: PasoArchivo) -> Result<Paso, String> {
    let permitidos: &[&str] = match p.paso.as_str() {
        "ir" => &["x", "y", "orientacion_deg"],
        "mirar" | "recibir" => &["x", "y"],
        "girar" => &["sentido", "vueltas", "tiempo_s"],
        "esperar" => &["tiempo_s"],
        "seguir" => &[
            "figura", "centro", "radio", "desde", "hasta", "periodo_s", "vueltas", "tiempo_s", "desfase_deg",
        ],
        "sincronizar" => &["barrera"],
        "empujar" => &["hacia", "a_puesto"],
        otro => {
            return Err(format!(
                "el paso `{otro}` no existe; los pasos son: {}",
                TIPOS_DE_PASO.join(", ")
            ));
        }
    };
    if let Some(c) = p.presentes().into_iter().find(|c| !permitidos.contains(c)) {
        return Err(format!("el campo `{c}` no va en `{}`", p.paso));
    }
    let PasoArchivo {
        paso,
        x,
        y,
        orientacion_deg,
        sentido,
        vueltas,
        tiempo_s,
        figura,
        centro,
        radio,
        desde,
        hasta,
        periodo_s,
        desfase_deg,
        barrera,
        hacia,
        a_puesto,
        tolerancia_m,
        tolerancia_deg,
        tiempo_max_s,
    } = p;
    let tipo = paso.as_str();
    let accion = match tipo {
        "ir" => Accion::Ir {
            punto: punto_de(tipo, x, y)?,
            orientacion: orientacion_deg.map(f64::to_radians),
        },
        "mirar" => Accion::Mirar { punto: punto_de(tipo, x, y)? },
        "recibir" => Accion::Recibir { punto: punto_de(tipo, x, y)? },
        "girar" => {
            let sentido = match sentido.as_deref() {
                Some("antihorario") => Sentido::Antihorario,
                Some("horario") => Sentido::Horario,
                Some(o) => return Err(format!("`sentido` es `antihorario` u `horario`, no `{o}`")),
                None => return Err("`girar` necesita `sentido`".to_string()),
            };
            Accion::Girar { sentido, duracion: duracion_de(tipo, vueltas, tiempo_s)? }
        }
        "esperar" => Accion::Esperar {
            tiempo_s: tiempo_s.ok_or_else(|| "`esperar` necesita `tiempo_s`".to_string())?,
        },
        "seguir" => {
            let figura = match figura.as_deref() {
                Some("circulo") => match (centro, radio) {
                    (Some(c), Some(r)) => Figura::Circulo { centro: c.vec(), radio: r },
                    _ => return Err("la figura `circulo` necesita `centro` y `radio`".to_string()),
                },
                Some("ida_vuelta") => match (desde, hasta) {
                    (Some(a), Some(b)) => Figura::IdaVuelta { desde: a.vec(), hasta: b.vec() },
                    _ => return Err("la figura `ida_vuelta` necesita `desde` y `hasta`".to_string()),
                },
                Some(o) => return Err(format!("`figura` es `circulo` o `ida_vuelta`, no `{o}`")),
                None => return Err("`seguir` necesita `figura`".to_string()),
            };
            Accion::Seguir {
                figura,
                periodo_s: periodo_s.ok_or_else(|| "`seguir` necesita `periodo_s`".to_string())?,
                duracion: duracion_de(tipo, vueltas, tiempo_s)?,
                desfase: desfase_deg.unwrap_or(0.0).to_radians(),
            }
        }
        "sincronizar" => Accion::Sincronizar {
            barrera: barrera.ok_or_else(|| "`sincronizar` necesita `barrera`".to_string())?,
        },
        "empujar" => {
            let hacia = match (hacia, a_puesto) {
                (Some(h), None) => Hacia::Punto(h.vec()),
                (None, Some(n)) => Hacia::Puesto(n),
                (Some(_), Some(_)) => {
                    return Err("`empujar` lleva `hacia` o `a_puesto`: va uno solo de los dos".to_string());
                }
                (None, None) => return Err("`empujar` necesita `hacia` o `a_puesto`".to_string()),
            };
            Accion::Empujar { hacia }
        }
        _ => unreachable!("tipo verificado arriba"),
    };
    Ok(Paso { accion, tolerancia_m, tolerancia_deg, tiempo_max_s })
}

/// `repeticiones` del archivo: ausente = 1, un entero ≥ 1 o `"bucle"`.
fn repeticiones_de(v: Option<serde_json::Value>) -> Result<Repeticiones, String> {
    match v {
        None => Ok(Repeticiones::Veces(1)),
        Some(serde_json::Value::String(s)) if s == "bucle" => Ok(Repeticiones::Bucle),
        Some(serde_json::Value::Number(n)) => match n.as_u64() {
            Some(k) if (1..=u32::MAX as u64).contains(&k) => Ok(Repeticiones::Veces(k as u32)),
            _ => Err(format!("`repeticiones` {n}: un entero ≥ 1 o \"bucle\"")),
        },
        Some(otro) => Err(format!("`repeticiones` {otro}: un entero ≥ 1 o \"bucle\"")),
    }
}

/// Lee el texto de un archivo de rutina, sin validar la geometría. Devuelve la rutina
/// (si el JSON se pudo leer) y los errores de lectura.
pub fn leer(texto: &str, id: &str) -> (Option<Rutina>, Vec<String>) {
    let texto = texto.strip_prefix('\u{feff}').unwrap_or(texto);
    let archivo: RutinaArchivo = match serde_json::from_str(texto) {
        Ok(a) => a,
        Err(e) => return (None, vec![format!("el archivo no se puede leer: {}", traducir(&e))]),
    };
    let mut errores = Vec::new();
    let grupo = Grupo::de_texto(&archivo.grupo).unwrap_or_else(|| {
        errores.push(format!(
            "grupo `{}` no existe: circuito, coreografia o con_pelota",
            archivo.grupo
        ));
        Grupo::Circuito
    });
    let repeticiones = repeticiones_de(archivo.repeticiones).unwrap_or_else(|e| {
        errores.push(e);
        Repeticiones::Veces(1)
    });
    let mut puestos = Vec::with_capacity(archivo.puestos.len());
    for pa in archivo.puestos {
        let mut pasos = Vec::with_capacity(pa.pasos.len());
        for (i, v) in pa.pasos.into_iter().enumerate() {
            let donde = format!("puesto {}, paso {}", pa.nombre, i + 1);
            match serde_json::from_value::<PasoArchivo>(v) {
                Ok(p) => match paso_de(p) {
                    Ok(paso) => pasos.push(paso),
                    Err(e) => errores.push(format!("{donde}: {e}")),
                },
                Err(e) => errores.push(format!("{donde}: {}", traducir(&e))),
            }
        }
        puestos.push(Puesto {
            nombre: pa.nombre,
            inicio: Pose {
                pos: Vec2::new(pa.inicio.x, pa.inicio.y),
                orientacion: pa.inicio.orientacion_deg.to_radians(),
            },
            arquero: pa.arquero,
            pasos,
        });
    }
    let rutina = Rutina {
        id: id.to_string(),
        nombre: archivo.nombre,
        grupo,
        descripcion: archivo.descripcion,
        robots_min: archivo.robots.min,
        robots_max: archivo.robots.max,
        pelota: archivo.pelota.map(|p| p.marca.vec()),
        repeticiones,
        puestos,
    };
    (Some(rutina), errores)
}

/// `true` si el id cumple `[a-z0-9_]+`.
pub fn id_valido(id: &str) -> bool {
    !id.is_empty() && id.chars().all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_')
}

/// Lee y valida un texto como la rutina `id`.
pub fn cargar_texto(texto: &str, id: &str, ruta: PathBuf, ctx: &ContextoValidacion) -> Entrada {
    let mut razones = Vec::new();
    if !id_valido(id) {
        razones.push(format!(
            "el id `{id}` (nombre del archivo) solo admite minúsculas, dígitos y `_`"
        ));
    }
    let (rutina, errores) = leer(texto, id);
    razones.extend(errores);
    if let Some(r) = &rutina {
        razones.extend(validar(r, ctx));
    }
    Entrada { id: id.to_string(), ruta, rutina: rutina.map(Arc::new), razones }
}

/// Lee y valida un archivo.
pub fn cargar_archivo(ruta: &Path, ctx: &ContextoValidacion) -> Entrada {
    let id = ruta
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_default();
    match std::fs::read_to_string(ruta) {
        Ok(texto) => cargar_texto(&texto, &id, ruta.to_path_buf(), ctx),
        Err(e) => Entrada {
            id,
            ruta: ruta.to_path_buf(),
            rutina: None,
            razones: vec![format!("no se puede abrir {}: {e}", ruta.display())],
        },
    }
}

/// Todos los `.json` de la carpeta, ordenados por id.
pub fn cargar_carpeta(dir: &Path, ctx: &ContextoValidacion) -> Result<Vec<Entrada>, String> {
    let lectura = std::fs::read_dir(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    let mut rutas: Vec<PathBuf> = lectura
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "json"))
        .collect();
    rutas.sort();
    Ok(rutas.iter().map(|r| cargar_archivo(r, ctx)).collect())
}

/// Una rutina por nombre: un id de la carpeta vigente o una ruta a un `.json`.
pub fn buscar(nombre: &str, ctx: &ContextoValidacion) -> Result<Entrada, String> {
    let como_ruta = Path::new(nombre);
    let ruta = if nombre.ends_with(".json") || como_ruta.components().count() > 1 {
        como_ruta.to_path_buf()
    } else {
        carpeta().join(format!("{nombre}.json"))
    };
    if !ruta.exists() {
        return Err(format!(
            "no existe la rutina `{nombre}` ({}); `--listar-rutinas` muestra las que hay",
            ruta.display()
        ));
    }
    Ok(cargar_archivo(&ruta, ctx))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ctx() -> ContextoValidacion {
        ContextoValidacion::por_defecto(1.0)
    }

    /// Rutina mínima válida de un robot (2 repeticiones), con los pasos dados (JSON de la lista).
    fn con_pasos(pasos: &str) -> String {
        format!(
            r#"{{
              "nombre": "Prueba", "grupo": "circuito", "descripcion": "Para los tests.",
              "robots": {{ "min": 1, "max": 1 }},
              "repeticiones": 2,
              "puestos": [ {{ "nombre": "A", "inicio": {{ "x": -0.3, "y": -0.25, "orientacion_deg": 0 }},
                             "pasos": [ {pasos} ] }} ]
            }}"#
        )
    }

    fn entrada(texto: &str) -> Entrada {
        cargar_texto(texto, "prueba", PathBuf::from("prueba.json"), &ctx())
    }

    #[test]
    fn valid_routine_reads_every_field() {
        let e = entrada(&con_pasos(
            r#"{ "paso": "ir", "x": 0.3, "y": -0.25 },
               { "paso": "ir", "x": 0.0, "y": 0.27, "orientacion_deg": 90, "tolerancia_m": 0.06 },
               { "paso": "mirar", "x": 0.75, "y": 0.0 },
               { "paso": "girar", "sentido": "horario", "vueltas": 1.5 },
               { "paso": "esperar", "tiempo_s": 2 }"#,
        ));
        assert!(e.razones.is_empty(), "{:?}", e.razones);
        let r = e.valida().expect("válida");
        assert_eq!(r.id, "prueba");
        assert_eq!(r.grupo, Grupo::Circuito);
        assert_eq!(r.repeticiones, Repeticiones::Veces(2));
        assert_eq!(r.texto_robots(), "1 robot");
        let pasos = &r.puestos[0].pasos;
        assert_eq!(pasos.len(), 5);
        assert_eq!(pasos[1].tolerancia_m, Some(0.06));
        match pasos[1].accion {
            Accion::Ir { orientacion: Some(o), .. } => assert!((o - std::f64::consts::FRAC_PI_2).abs() < 1e-12),
            ref a => panic!("{a:?}"),
        }
        assert_eq!(
            pasos[3].accion,
            Accion::Girar { sentido: Sentido::Horario, duracion: Duracion::Vueltas(1.5) }
        );
    }

    #[test]
    fn repetitions_default_to_one() {
        let texto = con_pasos(r#"{ "paso": "ir", "x": 0.3, "y": -0.25 }"#).replace(r#""repeticiones": 2,"#, "");
        let e = entrada(&texto);
        assert_eq!(e.rutina.unwrap().repeticiones, Repeticiones::Veces(1));
    }

    #[test]
    fn loop_and_bad_repetitions() {
        let bucle = con_pasos(r#"{ "paso": "ir", "x": 0.3, "y": -0.25 }"#).replace("\"repeticiones\": 2", "\"repeticiones\": \"bucle\"");
        assert_eq!(entrada(&bucle).rutina.unwrap().repeticiones, Repeticiones::Bucle);
        let cero = con_pasos(r#"{ "paso": "ir", "x": 0.3, "y": -0.25 }"#).replace("\"repeticiones\": 2", "\"repeticiones\": 0");
        assert!(entrada(&cero).razones.iter().any(|r| r.contains("repeticiones")));
    }

    #[test]
    fn typo_in_a_step_names_the_field_and_the_place() {
        let e = entrada(&con_pasos(
            r#"{ "paso": "ir", "x": 0.3, "y": -0.25 }, { "paso": "ir", "x": 0.0, "y": 0.27, "tolerancia": 0.05 }"#,
        ));
        assert!(
            e.razones.iter().any(|r| r == "puesto A, paso 2: campo desconocido `tolerancia`"),
            "{:?}",
            e.razones
        );
        assert!(e.valida().is_none());
    }

    #[test]
    fn unknown_field_at_the_top_is_an_error() {
        let texto = con_pasos(r#"{ "paso": "ir", "x": 0.3, "y": -0.25 }"#).replacen("\"nombre\"", "\"autor\": \"x\", \"nombre\"", 1);
        let e = entrada(&texto);
        assert!(e.rutina.is_none());
        assert!(e.razones[0].contains("campo desconocido `autor`"), "{:?}", e.razones);
    }

    #[test]
    fn unknown_step_type_lists_the_eight() {
        let e = entrada(&con_pasos(r#"{ "paso": "saltar" }"#));
        let r = e.razones.iter().find(|r| r.contains("saltar")).expect("razón");
        for t in TIPOS_DE_PASO {
            assert!(r.contains(t), "{r}");
        }
    }

    #[test]
    fn turn_takes_turns_or_time_not_both() {
        let e = entrada(&con_pasos(r#"{ "paso": "girar", "sentido": "antihorario", "vueltas": 1, "tiempo_s": 2 }"#));
        assert!(e.razones.iter().any(|r| r.contains("va uno solo de los dos")), "{:?}", e.razones);
        let e = entrada(&con_pasos(r#"{ "paso": "girar", "sentido": "a la derecha", "vueltas": 1 }"#));
        assert!(e.razones.iter().any(|r| r.contains("`sentido`")), "{:?}", e.razones);
    }

    #[test]
    fn a_field_of_another_type_is_rejected() {
        let e = entrada(&con_pasos(r#"{ "paso": "ir", "x": 0.3, "y": -0.25, "sentido": "horario" }"#));
        assert!(
            e.razones.iter().any(|r| r == "puesto A, paso 1: el campo `sentido` no va en `ir`"),
            "{:?}",
            e.razones
        );
    }

    #[test]
    fn missing_point_is_reported() {
        let e = entrada(&con_pasos(r#"{ "paso": "ir", "x": 0.3 }"#));
        assert!(e.razones.iter().any(|r| r == "puesto A, paso 1: `ir` necesita `x` e `y`"), "{:?}", e.razones);
    }

    #[test]
    fn id_must_be_lowercase() {
        let e = cargar_texto(
            &con_pasos(r#"{ "paso": "ir", "x": 0.3, "y": -0.25 }"#),
            "Triangulo",
            PathBuf::from("Triangulo.json"),
            &ctx(),
        );
        assert!(e.razones.iter().any(|r| r.contains("solo admite minúsculas, dígitos y `_`")), "{:?}", e.razones);
        assert!(id_valido("cuadrado_giros") && !id_valido("ocho-2") && !id_valido(""));
    }

    #[test]
    fn reserved_steps_are_read_with_their_fields() {
        let e = entrada(&con_pasos(
            r#"{ "paso": "seguir", "figura": "circulo", "centro": { "x": 0, "y": 0 }, "radio": 0.25, "periodo_s": 4, "vueltas": 2 },
               { "paso": "sincronizar", "barrera": "medio" },
               { "paso": "empujar", "hacia": { "x": 0.75, "y": 0 } },
               { "paso": "recibir", "x": 0.2, "y": 0.1 }"#,
        ));
        let r = e.rutina.expect("leída");
        let tipos: Vec<&str> = r.puestos[0].pasos.iter().map(|p| p.accion.tipo()).collect();
        assert_eq!(tipos, ["seguir", "sincronizar", "empujar", "recibir"]);
        assert_eq!(r.puestos[0].pasos[0].accion.fase(), 2);
        assert_eq!(r.puestos[0].pasos[2].accion.fase(), 3);
    }

    #[test]
    fn route_with_return_leg() {
        let e = entrada(&con_pasos(r#"{ "paso": "ir", "x": 0.3, "y": -0.25 }, { "paso": "mirar", "x": 0, "y": 0 }, { "paso": "ir", "x": 0.0, "y": 0.27 }"#));
        let p = &e.rutina.unwrap().puestos[0];
        assert_eq!(p.recorrido(false), vec![Vec2::new(-0.3, -0.25), Vec2::new(0.3, -0.25), Vec2::new(0.0, 0.27)]);
        assert_eq!(p.recorrido(true).last(), Some(&Vec2::new(0.3, -0.25)));
    }

    #[test]
    fn distance_to_a_route() {
        let pts = [Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)];
        assert!((distancia_a_recorrido(Vec2::new(0.5, 0.2), &pts) - 0.2).abs() < 1e-6);
        assert!((distancia_a_recorrido(Vec2::new(1.3, 0.4), &pts) - 0.5).abs() < 1e-6);
        assert!((distancia_a_tramo(Vec2::new(0.3, 0.4), Vec2::ZERO, Vec2::ZERO) - 0.5).abs() < 1e-6);
        assert_eq!(distancia_a_recorrido(Vec2::ZERO, &[]), f32::INFINITY);
    }

    #[test]
    fn bom_is_accepted() {
        let e = entrada(&format!("\u{feff}{}", con_pasos(r#"{ "paso": "ir", "x": 0.3, "y": -0.25 }"#)));
        assert!(e.razones.is_empty(), "{:?}", e.razones);
    }

    #[test]
    fn search_by_id_or_path() {
        let ctx = ctx();
        let err = buscar("no_existe", &ctx).unwrap_err();
        assert!(err.contains("no existe la rutina `no_existe`"), "{err}");
        let e = buscar("triangulo", &ctx).unwrap();
        assert!(e.valida().is_some(), "{:?}", e.razones);
        let ruta = Path::new(env!("CARGO_MANIFEST_DIR")).join("config/rutinas/ocho.json");
        assert_eq!(buscar(ruta.to_str().unwrap(), &ctx).unwrap().id, "ocho");
    }

    #[test]
    fn repetitions_from_the_console() {
        assert_eq!(Repeticiones::de_texto("10"), Ok(Repeticiones::Veces(10)));
        assert_eq!(Repeticiones::de_texto("bucle"), Ok(Repeticiones::Bucle));
        assert!(Repeticiones::de_texto("0").is_err() && Repeticiones::de_texto("dos").is_err());
    }
}
