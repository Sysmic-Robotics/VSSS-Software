//! `RoutineDecider`: una corrida de una rutina, como un decisor más del lazo común (D1 y
//! D4–D10 del design).
//!
//! Solo elige skill y objetivo por robot y por tick (GoTo, FacePoint, Spin y Hold; con la
//! pelota, ApproachAligned, ShootPush, Mark e Intercept); navegación, guardia, recuperación
//! de atasco, HALT, parada, corte sin visión, CSV y capas siguen siendo los del lazo. Las causas de aborto que dependen del lazo
//! (HALT, parada, STOP, corte, manual, teleport de la GUI) llegan por `observar`, que el
//! lazo llama al final de cada tick, también cuando no consulta al decisor.
//!
//! El reloj de la rutina es `tick · CONTROL_DT`: determinista en los tests (el CSV tiene
//! el `t_ms` de pared).

use super::asignacion::{Eleccion, asignar};
use super::formato::{
    Accion, Duracion, Hacia, Paso, Pose, Repeticiones, Rutina, distancia_a_recorrido, distancia_a_tramo,
};
use super::validacion::nombre_area;
use super::resumen::{Pases, Resultado, ResumenCorrida, TipoAborto, Vuelta, VueltaPuesto};
use super::vista::{
    CAUSA_DETENIDA, CAUSA_USUARIO, Falta, Fase, RobotEnRutina, RutinaVista, frase_cierre, frase_ejecucion,
    frase_pausa, frase_preparacion, segundos, verbo, vueltas,
};
use crate::coach::{FIELD_HALF_X, SkillChoice};
use crate::control_loop::{ContextoTick, PerfilVelocidad, TickDecider, TickMode, sin_vision};
use crate::motion::{CONTROL_DT, Motion, MotionConfig};
use crate::params::{RutinasParams, params};
use crate::radio::TeleportItem;
use crate::skills::zones::{AREA_CLEARANCE, AreaRect};
use crate::skills::{ApproachAlignedSkill, ShootPushSkill, Skill, SkillConfig, SkillId};
use crate::world::{RobotState, World};
use glam::Vec2;
use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::Arc;
use std::time::{Duration, SystemTime};

/// Distancia del punto que mira FacePoint para orientar a un ángulo (m): lejos, el
/// desvío por estar a unos cm del destino es despreciable.
const MIRA_LEJOS_M: f32 = 10.0;
/// Lugares fuera de la cancha donde el teleport de la preparación deja a los robots sin
/// puesto (y = 0.90, separados 0.20 m) y a la pelota (y = −0.90), como el perfil de sysid.
const PARQUE_Y: f64 = 0.90;
const PARQUE_X0: f64 = -0.60;
const PARQUE_DX: f64 = 0.20;

/// `recibir`: ticks seguidos con la pelota controlada para darla por recibida (un rebote
/// no cuenta como control).
const TICKS_CONTROL: u32 = 3;

/// Frase antes de asignar robots: la rutina espera el juego y la visión.
const FRASE_ESPERA: &str = "Preparando: esperando la visión de los robots propios y el juego";

/// Porcentaje de velocidad válido: de 20 a 100 en pasos de 10.
pub fn velocidad_valida(pct: u32) -> bool {
    (20..=100).contains(&pct) && pct.is_multiple_of(10)
}

/// Opciones de una corrida.
#[derive(Debug, Clone, PartialEq)]
pub struct OpcionesCorrida {
    pub eleccion: Eleccion,
    pub velocidad_pct: u32,
    pub repeticiones: Repeticiones,
    /// Teleport de la preparación (solo en simulación).
    pub teletransportar: bool,
    pub simulacion: bool,
    /// Arranca sola al quedar lista (consola).
    pub inicio_automatico: bool,
}

impl OpcionesCorrida {
    /// Robots automáticos, la velocidad por defecto del transporte, las repeticiones del
    /// archivo, sin teleport ni inicio automático.
    pub fn por_defecto(rutina: &Rutina, simulacion: bool) -> Self {
        let p = &params().rutinas;
        Self {
            eleccion: Eleccion::Automatica,
            velocidad_pct: if simulacion { p.velocidad_sim_pct } else { p.velocidad_real_pct },
            repeticiones: rutina.repeticiones,
            teletransportar: false,
            simulacion,
            inicio_automatico: false,
        }
    }
}

/// Lo que el decisor toma de afuera; `vigente` lo arma con los parámetros del proceso.
#[derive(Debug, Clone)]
pub struct Entorno {
    pub own_team: i32,
    pub num_robots: usize,
    pub keeper_id: i32,
    pub bidireccional: bool,
    pub blind_stop_s: f64,
    /// Giro base de Spin (`SkillConfig::default().spin_omega`).
    pub spin_omega: f64,
    pub params: RutinasParams,
    /// Pelota controlada: a lo sumo a esta distancia del robot (`skills.intercept_reach_radius`).
    pub alcance_m: f32,
    /// Pelota quieta: más lenta que esto (`skills.intercept_min_ball_speed`, m/s).
    pub pelota_lenta: f32,
}

impl Entorno {
    pub fn vigente(own_team: i32, num_robots: usize) -> Self {
        let p = params();
        Self {
            own_team,
            num_robots,
            keeper_id: p.coach.keeper_id,
            bidireccional: MotionConfig::from_env().bidirectional,
            blind_stop_s: p.vision.blind_stop_s,
            spin_omega: SkillConfig::default().spin_omega,
            params: p.rutinas.clone(),
            alcance_m: p.skills.intercept_reach_radius,
            pelota_lenta: p.skills.intercept_min_ball_speed,
        }
    }
}

/// Órdenes del usuario sobre una corrida (la de preparar es crearla).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OrdenCorrida {
    Iniciar,
    Pausar,
    Reanudar,
    Abortar,
}

/// `recibir`: después del contacto o del cruce de su línea, el receptor va por la pelota
/// que rueda más lenta que esto (m/s) dentro de `recepcion_radio_m` (D3). Medido en FIRASim:
/// tras pegar en una esquina del receptor, la pelota se escapa a 0.13–0.3 m/s.
const PELOTA_ESCAPA_MAX: f32 = 0.3;

/// Una causa de aborto: su tipo, como dato, y la frase para mostrar.
#[derive(Debug, Clone, PartialEq)]
struct Aborto {
    tipo: TipoAborto,
    frase: String,
}

fn aborto(tipo: TipoAborto, frase: impl Into<String>) -> Aborto {
    Aborto { tipo, frase: frase.into() }
}

/// Estado de un puesto en la vuelta en curso.
#[derive(Debug, Clone)]
struct EnCurso {
    paso: usize,
    /// Inicio del paso (s, corrido por las pausas).
    t_paso: f64,
    /// `ir` con orientación: ya llegó y se orienta.
    orientando: bool,
    /// `girar`: giro acumulado (rad) y la última orientación vista.
    giro: f64,
    theta_prev: Option<f64>,
    /// Terminó sus pasos de esta vuelta y espera a los demás.
    hecho: bool,
    /// `empujar` y la recuperación: empujando, desde esta posición del robot (antes,
    /// alineándose detrás de la pelota).
    empuje_desde: Option<Vec2>,
    /// `recibir`: ticks seguidos con la pelota controlada.
    control: u32,
    /// `recibir`: ticks seguidos con la pelota quieta fuera de `recepcion_radio_m`.
    afuera: u32,
    /// `recibir`: recuperando la pelota tras una recepción fallida, desde este instante.
    recuperando: Option<f64>,
    /// `recibir`: la pelota ya llegó al robot o cruzó su línea.
    contacto: bool,
    err_suma: f64,
    err_n: u32,
    err_max: f64,
    escapes: u32,
    guardia: u32,
}

impl EnCurso {
    fn nuevo(t: f64) -> Self {
        Self {
            paso: 0,
            t_paso: t,
            orientando: false,
            giro: 0.0,
            theta_prev: None,
            hecho: false,
            empuje_desde: None,
            control: 0,
            afuera: 0,
            recuperando: None,
            contacto: false,
            err_suma: 0.0,
            err_n: 0,
            err_max: 0.0,
            escapes: 0,
            guardia: 0,
        }
    }

    fn siguiente_paso(&mut self, t: f64, total: usize) {
        self.paso += 1;
        self.t_paso = t;
        self.orientando = false;
        self.giro = 0.0;
        self.theta_prev = None;
        self.empuje_desde = None;
        self.control = 0;
        self.afuera = 0;
        self.recuperando = None;
        self.contacto = false;
        self.hecho = self.paso >= total;
    }
}

/// Resultado de evaluar el paso en curso de un puesto.
enum Evaluacion {
    Sigue(SkillChoice),
    Termina,
}

/// Una corrida de una rutina.
pub struct RoutineDecider {
    rutina: Arc<Rutina>,
    op: OpcionesCorrida,
    ent: Entorno,
    escala: f64,
    /// Recorrido ideal de cada puesto (con la vuelta si hay más de una repetición).
    recorridos: Vec<Vec<Vec2>>,
    fase: Fase,
    /// Robot de cada puesto en uso; vacío hasta asignar.
    asignados: Vec<i32>,
    /// Reloj de la rutina en el último tick visto (s).
    t: f64,
    t_prep: Option<f64>,
    ventanas: Vec<VecDeque<(Vec2, f64)>>,
    falta: Option<Falta>,
    teleport: Vec<TeleportItem>,
    en_curso: Vec<EnCurso>,
    repeticion: u32,
    t_vuelta: f64,
    pausa_desde: Option<f64>,
    pausa_en_vuelta: f64,
    preparacion_s: Option<f64>,
    vueltas: Vec<Vuelta>,
    /// Último modo del lazo (por `observar`).
    modo: TickMode,
    previo_escape: HashSet<i32>,
    previo_guardia: HashSet<i32>,
    causa: Option<String>,
    tipo_aborto: Option<TipoAborto>,
    fase_aborto: Option<Fase>,
    resumen: Option<Arc<ResumenCorrida>>,
    vista: Arc<RutinaVista>,
    /// Pases: objetivo de cada `empujar` (puesto, paso) y su `recibir` emparejado.
    objetivos: HashMap<(usize, usize), Vec2>,
    receptor_de: HashMap<(usize, usize), (usize, usize)>,
    emisor_de: HashMap<(usize, usize), (usize, usize)>,
    /// `recibir` (puesto, paso) cuyo pase se hizo en la vuelta en curso, con el instante y
    /// dónde estaba la pelota (el origen del pase, para la línea del receptor).
    pases_hechos: HashMap<(usize, usize), (f64, Vec2)>,
    /// Por puesto que pasa: (intentados, logrados) en la corrida, y en la vuelta en curso.
    pases_puesto: Vec<(u32, u32)>,
    pases_vuelta: (u32, u32),
    recuperaciones: u32,
    fallidas_seguidas: u32,
    /// Preparación con pelota: posiciones recientes de la pelota, y el tiempo que la
    /// preparación esperó solo a la pelota (no cuenta para el tiempo máximo).
    ventana_pelota: VecDeque<Vec2>,
    espera_pelota: f64,
    t_prev: Option<f64>,
    /// Cuándo pidió la rutina su teleport: durante `quieto_s` el lazo todavía puede ver
    /// las poses de antes y el casi choque no cuenta.
    t_teleport: Option<f64>,
    /// Tests del conmutador: entra en pánico en el próximo `decide` u `observar`.
    #[cfg(test)]
    pub(crate) panico: bool,
}

/// La pelota quieta en `m` (teleport).
fn pelota_en(m: Vec2) -> TeleportItem {
    TeleportItem::Ball { x: f64::from(m.x), y: f64::from(m.y), vx: 0.0, vy: 0.0 }
}

fn choice(robot_id: i32, skill_id: SkillId, target: Vec2) -> SkillChoice {
    SkillChoice { robot_id, skill_id, target }
}

/// Error de heading como lo calcula motion: normalizado y, en bidireccional, plegado.
fn error_heading(objetivo: f64, actual: f64, bidireccional: bool) -> f64 {
    let e = Motion::normalize_angle(objetivo - actual);
    if bidireccional { Motion::fold_bidirectional(e) } else { e }
}

/// Punto lejano en la dirección `theta` desde `p`, para orientar con FacePoint.
/// Las dos áreas: ningún robot de una rutina es arquero.
fn areas() -> [AreaRect; 2] {
    [AreaRect::own(1.0), AreaRect::opp(1.0)]
}

/// Dónde espera el receptor después del pase (D3): en su `punto` o, mientras la pelota viene
/// hacia él (se acerca a su línea a `lenta` o más), donde va a cruzarla. Su línea es la
/// perpendicular al pase (de `origen`, la pelota al hacerse el pase, a `punto`) por su punto.
/// El corrimiento no pasa de `radio`, y se achica hasta que el robot no toque ningún área con
/// la holgura de la validación.
fn punto_de_espera(punto: Vec2, origen: Vec2, pelota: Vec2, vel: Vec2, lenta: f32, radio: f32) -> Vec2 {
    let n = (punto - origen).normalize_or_zero();
    let falta = (punto - pelota).dot(n);
    let acerca = vel.dot(n);
    if n == Vec2::ZERO || acerca < lenta || falta <= 0.0 {
        return punto;
    }
    let lado = n.perp();
    let cruce = (pelota + vel * (falta / acerca) - punto).dot(lado).clamp(-radio, radio);
    const PASOS: u32 = 25;
    (0..=PASOS)
        .map(|j| punto + lado * cruce * (1.0 - j as f32 / PASOS as f32))
        .find(|q| !areas().iter().any(|a| a.touches_with(*q, AREA_CLEARANCE)))
        .unwrap_or(punto)
}

/// Por qué no hay lugar detrás de la pelota para el `empujar` (D2), o `None`. El lugar es el
/// de ApproachAligned (`staging_point`, la misma función de la skill): con la pelota contra la
/// pared, la skill lo acerca a la pelota y, si ni así entra al campo lógico, lo recorta contra
/// el borde, fuera de la línea del pase. Junto a un área, alcanza con que el robot quepa sin
/// tocarla en el punto más cercano a la pelota que la skill acepta (`pos_tol` hacia ella).
fn sin_lugar_detras(alinear: &ApproachAlignedSkill, pelota: Vec2) -> Option<String> {
    let dir = (alinear.aim_point - pelota).normalize_or_zero();
    let lugar = alinear.staging_point(pelota)?;
    let atras = pelota - lugar;
    if atras.length() < 1e-4 || (atras.dot(dir) - atras.length()).abs() > 1e-3 {
        return Some("está contra la pared".to_string());
    }
    let mas_cerca = lugar + dir * alinear.pos_tol;
    areas().iter().find(|a| a.touches(mas_cerca)).map(|a| format!("el lugar toca {}", nombre_area(a)))
}

fn mira_hacia(p: Vec2, theta: f64) -> Vec2 {
    p + MIRA_LEJOS_M * Vec2::new(theta.cos() as f32, theta.sin() as f32)
}

/// Pose media de un tramo de la ventana (orientación por media circular).
fn media<'a>(muestras: impl Iterator<Item = &'a (Vec2, f64)>) -> (Vec2, f64) {
    let (mut p, mut s, mut c, mut n) = (Vec2::ZERO, 0.0, 0.0, 0.0_f32);
    for (q, th) in muestras {
        p += *q;
        s += th.sin();
        c += th.cos();
        n += 1.0;
    }
    (p / n.max(1.0), f64::atan2(s, c))
}

/// Causa de aborto de un paso que superó su tiempo máximo `max` (s).
fn frase_fuera_de_tiempo(robot: i32, i: usize, paso: &Paso, max: f64, orientando: bool) -> String {
    let (n, t) = (i + 1, segundos(max));
    match &paso.accion {
        Accion::Ir { .. } if orientando => format!("el robot {robot} no se orientó en el punto {n} en {t} s"),
        Accion::Ir { .. } => format!("el robot {robot} no llegó al punto {n} en {t} s"),
        Accion::Mirar { .. } => format!("el robot {robot} no se orientó al punto {n} en {t} s"),
        Accion::Girar { duracion: Duracion::Vueltas(v), .. } => {
            format!("el robot {robot} no completó {} en {t} s", vueltas(*v))
        }
        Accion::Empujar { hacia: Hacia::Puesto(_) } => format!("el robot {robot} no pasó la pelota en {t} s"),
        Accion::Empujar { .. } => format!("el robot {robot} no empujó la pelota en {t} s"),
        otro => format!("el robot {robot} no terminó el paso {n} ({}) en {t} s", otro.tipo()),
    }
}

/// Cómo está un robot respecto de su inicio.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Quietud {
    FueraDePosicion,
    Moviendose,
    Lista,
}

impl RoutineDecider {
    pub fn new(rutina: Arc<Rutina>, op: OpcionesCorrida, ent: Entorno) -> Self {
        let escala = f64::from(op.velocidad_pct.clamp(20, 100)) / 100.0;
        let vuelta = op.repeticiones.mas_de_una();
        let recorridos = rutina.puestos.iter().map(|p| p.recorrido(vuelta)).collect();
        let mut objetivos = HashMap::new();
        let mut receptor_de = HashMap::new();
        let mut emisor_de = HashMap::new();
        for pase in rutina.pases() {
            if let Some(o) = pase.objetivo {
                objetivos.insert(pase.de, o);
            }
            if let Some(r) = pase.receptor {
                receptor_de.insert(pase.de, r);
                emisor_de.insert(r, pase.de);
            }
        }
        let n_puestos = rutina.puestos.len();
        let vista = Arc::new(RutinaVista {
            rutina: rutina.clone(),
            fase: Fase::Preparando,
            frase: "Preparando".to_string(),
            velocidad_pct: op.velocidad_pct,
            repeticion: 0,
            repeticiones: op.repeticiones,
            robots: Vec::new(),
            resumen: None,
        });
        Self {
            rutina,
            op,
            ent,
            escala,
            recorridos,
            fase: Fase::Preparando,
            asignados: Vec::new(),
            t: 0.0,
            t_prep: None,
            ventanas: Vec::new(),
            falta: None,
            teleport: Vec::new(),
            en_curso: Vec::new(),
            repeticion: 0,
            t_vuelta: 0.0,
            pausa_desde: None,
            pausa_en_vuelta: 0.0,
            preparacion_s: None,
            vueltas: Vec::new(),
            modo: TickMode::Play,
            previo_escape: HashSet::new(),
            previo_guardia: HashSet::new(),
            causa: None,
            tipo_aborto: None,
            fase_aborto: None,
            resumen: None,
            vista,
            objetivos,
            receptor_de,
            emisor_de,
            pases_hechos: HashMap::new(),
            pases_puesto: vec![(0, 0); n_puestos],
            pases_vuelta: (0, 0),
            recuperaciones: 0,
            fallidas_seguidas: 0,
            ventana_pelota: VecDeque::new(),
            espera_pelota: 0.0,
            t_prev: None,
            t_teleport: None,
            #[cfg(test)]
            panico: false,
        }
    }

    /// Con los parámetros vigentes del proceso.
    pub fn vigente(rutina: Arc<Rutina>, op: OpcionesCorrida, own_team: i32, num_robots: usize) -> Self {
        Self::new(rutina, op, Entorno::vigente(own_team, num_robots))
    }

    pub fn fase(&self) -> Fase {
        self.fase
    }

    pub fn causa(&self) -> Option<&str> {
        self.causa.as_deref()
    }

    pub fn tipo_aborto(&self) -> Option<TipoAborto> {
        self.tipo_aborto
    }

    pub fn resumen(&self) -> Option<&Arc<ResumenCorrida>> {
        self.resumen.as_ref()
    }

    /// Robot de cada puesto en uso (vacío antes de asignar).
    pub fn asignados(&self) -> &[i32] {
        &self.asignados
    }

    pub fn vista(&self) -> &Arc<RutinaVista> {
        &self.vista
    }

    fn p(&self) -> &RutinasParams {
        &self.ent.params
    }

    // ─── Órdenes ─────────────────────────────────────────────────────────────

    /// Aplica una orden del usuario. Las que no corresponden a la fase se ignoran.
    pub fn ordenar(&mut self, orden: OrdenCorrida) {
        match (orden, self.fase) {
            (OrdenCorrida::Iniciar, Fase::Lista) => self.iniciar(),
            (OrdenCorrida::Pausar, Fase::Ejecutando) => {
                self.fase = Fase::Pausada;
                self.pausa_desde = Some(self.t);
            }
            (OrdenCorrida::Reanudar, Fase::Pausada) => {
                let d = self.pausa_desde.take().map_or(0.0, |t0| (self.t - t0).max(0.0));
                for e in &mut self.en_curso {
                    e.t_paso += d;
                    e.theta_prev = None;
                    if let Some(t0) = &mut e.recuperando {
                        *t0 += d;
                    }
                }
                for (t0, _) in self.pases_hechos.values_mut() {
                    *t0 += d;
                }
                self.pausa_en_vuelta += d;
                self.fase = Fase::Ejecutando;
            }
            (OrdenCorrida::Abortar, f) if f.activa() => {
                if self.op.repeticiones == Repeticiones::Bucle {
                    self.cerrar(Resultado::Detenida, Some(aborto(TipoAborto::Usuario, CAUSA_DETENIDA)));
                } else {
                    self.cerrar(Resultado::Abortada, Some(aborto(TipoAborto::Usuario, CAUSA_USUARIO)));
                }
            }
            _ => {}
        }
        self.refrescar_vista();
    }

    /// Aborta con una causa propia (p. ej. el tope de `--dur` de la consola).
    pub fn abortar(&mut self, tipo: TipoAborto, causa: &str) {
        self.cerrar(Resultado::Abortada, Some(aborto(tipo, causa)));
        self.refrescar_vista();
    }

    fn iniciar(&mut self) {
        self.fase = Fase::Ejecutando;
        self.preparacion_s = self.t_prep.map(|t0| self.t - t0);
        self.repeticion = 1;
        self.t_vuelta = self.t;
        self.pausa_en_vuelta = 0.0;
        self.en_curso = (0..self.asignados.len()).map(|_| EnCurso::nuevo(self.t)).collect();
    }

    /// Termina la corrida: Hold a todos, la causa y el resumen. Sin efecto si ya terminó.
    fn cerrar(&mut self, resultado: Resultado, causa: Option<Aborto>) {
        if !self.fase.activa() {
            return;
        }
        if resultado != Resultado::Completada {
            self.fase_aborto = Some(self.fase);
        }
        self.fase = if resultado == Resultado::Completada { Fase::Terminada } else { Fase::Abortada };
        self.tipo_aborto = causa.as_ref().map(|a| a.tipo);
        self.causa = causa.map(|a| a.frase);
        self.resumen = Some(Arc::new(ResumenCorrida {
            rutina: self.rutina.id.clone(),
            nombre: self.rutina.nombre.clone(),
            velocidad_pct: self.op.velocidad_pct,
            repeticiones: self.op.repeticiones,
            asignacion: self
                .asignados
                .iter()
                .enumerate()
                .map(|(k, r)| (self.rutina.puestos[k].nombre.clone(), *r))
                .collect(),
            resultado,
            causa: self.causa.clone(),
            tipo_aborto: self.tipo_aborto,
            fase_aborto: self.fase_aborto,
            preparacion_s: self.preparacion_s,
            vueltas: self.vueltas.clone(),
            pases: self.rutina.pelota.map(|_| Pases {
                intentados: self.pases_puesto.iter().map(|p| p.0).sum(),
                logrados: self.pases_puesto.iter().map(|p| p.1).sum(),
                recuperaciones: self.recuperaciones,
                por_puesto: self
                    .rutina
                    .puestos
                    .iter()
                    .zip(&self.pases_puesto)
                    .filter(|(_, p)| p.0 > 0)
                    .map(|(a, p)| (a.nombre.clone(), p.0, p.1))
                    .collect(),
            }),
        }));
    }

    fn abortar_por(&mut self, causa: Aborto) {
        self.cerrar(Resultado::Abortada, Some(causa));
    }

    // ─── Tick ────────────────────────────────────────────────────────────────

    /// `decide` con el reloj de pared explícito (para los tests).
    pub fn decidir(&mut self, tick: u32, world: &World, ahora: SystemTime) -> Vec<SkillChoice> {
        #[cfg(test)]
        assert!(!self.panico, "pánico de prueba en decide");
        self.t = f64::from(tick) * CONTROL_DT;
        let mut out = self.decidir_fase(world, ahora);
        self.completar_con_hold(&mut out, world);
        self.refrescar_vista();
        out
    }

    fn decidir_fase(&mut self, world: &World, ahora: SystemTime) -> Vec<SkillChoice> {
        if self.fase.activa()
            && self.asignados.is_empty()
            && let Err(e) = self.preparar_asignacion(world)
        {
            self.abortar_por(aborto(TipoAborto::Asignacion, format!("no se pudo preparar: {e}")));
        }
        if self.fase.activa()
            && let Some(c) = self.chequeo_vivo(world, ahora)
        {
            self.abortar_por(c);
        }
        match self.fase {
            Fase::Preparando | Fase::Lista => self.preparar(world),
            Fase::Ejecutando => self.ejecutar(world),
            Fase::Pausada | Fase::Terminada | Fase::Abortada => Vec::new(),
        }
    }

    /// Robots propios activos con id < `num_robots`, ordenados por id.
    fn propios(&self, world: &World) -> Vec<RobotState> {
        let todos = if self.ent.own_team == 0 {
            world.get_blue_team_active()
        } else {
            world.get_yellow_team_active()
        };
        let mut v: Vec<RobotState> = todos
            .into_iter()
            .filter(|r| r.id >= 0 && (r.id as usize) < self.ent.num_robots)
            .cloned()
            .collect();
        v.sort_by_key(|r| r.id);
        v
    }

    /// El robot `id` visto y con un dato reciente.
    fn robot(&self, world: &World, id: i32, ahora: SystemTime) -> Option<RobotState> {
        let umbral = Duration::from_secs_f64(self.ent.blind_stop_s);
        world
            .get_robot_state(id, self.ent.own_team)
            .filter(|r| r.active && !sin_vision(Some(r.last_update), ahora, umbral))
            .cloned()
    }

    /// Hold para cada robot propio visto sin elección.
    fn completar_con_hold(&self, out: &mut Vec<SkillChoice>, world: &World) {
        for r in self.propios(world) {
            if !out.iter().any(|c| c.robot_id == r.id) {
                out.push(choice(r.id, SkillId::Hold, Vec2::ZERO));
            }
        }
    }

    fn preparar_asignacion(&mut self, world: &World) -> Result<(), String> {
        let vistos: Vec<(i32, Vec2)> = self.propios(world).iter().map(|r| (r.id, r.position)).collect();
        self.asignados = asignar(&self.rutina, &vistos, self.ent.keeper_id, &self.op.eleccion)?;
        self.t_prep = Some(self.t);
        self.ventanas = vec![VecDeque::new(); self.asignados.len()];
        if self.op.teletransportar && self.op.simulacion {
            self.teleport = self.armar_teleport(world);
        } else if self.op.simulacion
            && let Some(m) = self.rutina.pelota
        {
            // Con pelota, en simulación la pelota va sola a su marca (D5).
            self.teleport = vec![pelota_en(m)];
        }
        if !self.teleport.is_empty() {
            self.t_teleport = Some(self.t);
        }
        Ok(())
    }

    /// Asignados a su inicio; los demás robots (propios y rivales) y la pelota, fuera de
    /// la cancha. `theta` en grados, como espera el replacement del simulador.
    fn armar_teleport(&self, world: &World) -> Vec<TeleportItem> {
        let team = self.ent.own_team as u32;
        let mut items: Vec<TeleportItem> = self
            .asignados
            .iter()
            .enumerate()
            .map(|(k, id)| {
                let ini = self.rutina.puestos[k].inicio;
                TeleportItem::Robot {
                    team,
                    id: *id as u32,
                    x: f64::from(ini.pos.x),
                    y: f64::from(ini.pos.y),
                    theta: ini.orientacion.to_degrees(),
                }
            })
            .collect();
        let mut otros: Vec<(i32, i32)> = world
            .get_blue_team_active()
            .into_iter()
            .chain(world.get_yellow_team_active())
            .map(|r| (r.team, r.id))
            .filter(|(t, id)| !(*t == self.ent.own_team && self.asignados.contains(id)))
            .collect();
        otros.sort();
        for (j, (t, id)) in otros.into_iter().enumerate() {
            items.push(TeleportItem::Robot {
                team: t as u32,
                id: id as u32,
                x: PARQUE_X0 + PARQUE_DX * j as f64,
                y: PARQUE_Y,
                theta: 0.0,
            });
        }
        items.push(match self.rutina.pelota {
            Some(m) => pelota_en(m),
            None => TeleportItem::Ball { x: 0.0, y: -PARQUE_Y, vx: 0.0, vy: 0.0 },
        });
        items
    }

    /// Causas de aborto que se ven en el `World`: un asignado sin visión, o dos propios a
    /// menos de `distancia_choque_m` (salvo durante `quieto_s` tras el teleport de la
    /// rutina, cuando el lazo todavía puede ver las poses de antes).
    fn chequeo_vivo(&self, world: &World, ahora: SystemTime) -> Option<Aborto> {
        for &id in &self.asignados {
            if self.robot(world, id, ahora).is_none() {
                return Some(aborto(TipoAborto::RobotSinVision, format!("el robot {id} dejó de verse")));
            }
        }
        if self.t_teleport.is_some_and(|t0| self.t - t0 < self.p().quieto_s) {
            return None;
        }
        let mut propios: Vec<&RobotState> = if self.ent.own_team == 0 {
            world.get_blue_team_active()
        } else {
            world.get_yellow_team_active()
        };
        propios.sort_by_key(|r| r.id);
        for (i, a) in propios.iter().enumerate() {
            for b in &propios[i + 1..] {
                let d = (a.position - b.position).length();
                if d < self.p().distancia_choque_m {
                    return Some(aborto(TipoAborto::Choque, format!("los robots {} y {} quedaron a {d:.2} m", a.id, b.id)));
                }
            }
        }
        None
    }

    // ─── Preparación ─────────────────────────────────────────────────────────

    fn largo_ventana(&self) -> usize {
        ((self.p().quieto_s / CONTROL_DT).round() as usize).max(2)
    }

    /// Posición y quietud de un robot respecto de su inicio (D6).
    fn quietud(&self, ventana: &VecDeque<(Vec2, f64)>, inicio: Pose) -> Quietud {
        let p = self.p();
        let n = self.largo_ventana();
        let (tol, tol_ang) = (p.tolerancia_m, p.tolerancia_deg.to_radians());
        let en_posicion = |pos: Vec2, th: f64| {
            (pos - inicio.pos).length() <= tol
                && error_heading(inicio.orientacion, th, self.ent.bidireccional).abs() <= tol_ang
        };
        if ventana.len() < n {
            return match ventana.back() {
                Some((pos, th)) if en_posicion(*pos, *th) => Quietud::Moviendose,
                _ => Quietud::FueraDePosicion,
            };
        }
        let h = n / 2;
        let (p1, a1) = media(ventana.iter().take(h));
        let (p2, a2) = media(ventana.iter().skip(n - h));
        if !en_posicion(p2, a2) {
            return Quietud::FueraDePosicion;
        }
        let quieto = (p2 - p1).length() <= p.quieto_m
            && Motion::normalize_angle(a2 - a1).abs() <= p.quieto_deg.to_radians();
        if quieto { Quietud::Lista } else { Quietud::Moviendose }
    }

    /// Un robot sin puesto a menos de `distancia_choque_m + tolerancia_m` del recorrido.
    fn cerca_del_recorrido(&self, world: &World) -> Option<Falta> {
        let umbral = self.p().distancia_choque_m + self.p().tolerancia_m;
        for r in self.propios(world) {
            if self.asignados.contains(&r.id) {
                continue;
            }
            for k in 0..self.asignados.len() {
                let d = distancia_a_recorrido(r.position, &self.recorridos[k]);
                if d < umbral {
                    return Some(Falta::CercaDelRecorrido {
                        robot: r.id,
                        distancia: d,
                        puesto: self.rutina.puestos[k].nombre.clone(),
                    });
                }
            }
        }
        None
    }

    /// Con pelota: lo que le falta a la pelota para estar quieta en su marca, con la misma
    /// ventana que los robots (D5).
    fn falta_pelota(&mut self, world: &World, marca: Vec2) -> Option<Falta> {
        let n = self.largo_ventana();
        let b = world.get_ball_state().position;
        if !b.is_finite() {
            self.ventana_pelota.clear();
            return Some(Falta::PelotaNoSeVe);
        }
        self.ventana_pelota.push_back(b);
        while self.ventana_pelota.len() > n {
            self.ventana_pelota.pop_front();
        }
        let tol = self.p().tolerancia_m;
        if self.ventana_pelota.len() < n {
            return Some(if (b - marca).length() > tol {
                Falta::PelotaFueraDeMarca { distancia: (b - marca).length() }
            } else {
                Falta::PelotaRueda
            });
        }
        let h = n / 2;
        let m1 = self.ventana_pelota.iter().take(h).sum::<Vec2>() / h as f32;
        let m2 = self.ventana_pelota.iter().skip(n - h).sum::<Vec2>() / h as f32;
        let d = (m2 - marca).length();
        if d > tol {
            return Some(Falta::PelotaFueraDeMarca { distancia: d });
        }
        let lapso = (n - h) as f64 * CONTROL_DT;
        if f64::from((m2 - m1).length()) / lapso >= f64::from(self.ent.pelota_lenta) {
            return Some(Falta::PelotaRueda);
        }
        None
    }

    fn preparar(&mut self, world: &World) -> Vec<SkillChoice> {
        let n = self.largo_ventana();
        let tol = self.p().tolerancia_m;
        let mut out = Vec::new();
        let mut falta: Option<Falta> = None;
        let falta_pelota = self.rutina.pelota.and_then(|m| self.falta_pelota(world, m));
        for k in 0..self.asignados.len() {
            let id = self.asignados[k];
            let Some(r) = world.get_robot_state(id, self.ent.own_team).filter(|r| r.active) else { continue };
            let puesto = &self.rutina.puestos[k];
            let ventana = &mut self.ventanas[k];
            ventana.push_back((r.position, r.orientation));
            while ventana.len() > n {
                ventana.pop_front();
            }
            if (r.position - puesto.inicio.pos).length() > tol {
                out.push(choice(id, SkillId::GoTo, puesto.inicio.pos));
                falta.get_or_insert(Falta::VaASuPuesto { robot: id, puesto: puesto.nombre.clone() });
                continue;
            }
            out.push(choice(id, SkillId::FacePoint, mira_hacia(puesto.inicio.pos, puesto.inicio.orientacion)));
            match self.quietud(&self.ventanas[k], puesto.inicio) {
                Quietud::Lista => {}
                Quietud::FueraDePosicion => {
                    falta.get_or_insert(Falta::SeOrienta { robot: id });
                }
                Quietud::Moviendose => {
                    falta.get_or_insert(Falta::Quieto { robot: id });
                }
            }
        }
        if falta.is_none() {
            falta = self.cerca_del_recorrido(world);
        }
        if falta.is_none() {
            falta = falta_pelota;
        }
        // Esperar solo a la pelota no corre el tiempo de la preparación: la persona tiene
        // que poder colocarla.
        let dt = self.t_prev.map_or(0.0, |t0| (self.t - t0).max(0.0));
        self.t_prev = Some(self.t);
        if falta.as_ref().is_some_and(Falta::es_de_la_pelota) {
            self.espera_pelota += dt;
        }
        let lista = falta.is_none() && self.modo == TickMode::Play;
        self.falta = falta;
        match (self.fase, lista) {
            (Fase::Preparando, true) => {
                self.fase = Fase::Lista;
                if self.op.inicio_automatico {
                    self.iniciar();
                    return self.ejecutar(world);
                }
            }
            (Fase::Lista, false) => {
                // Se salió: vuelve a preparar, con el tiempo máximo desde ahora.
                self.fase = Fase::Preparando;
                self.t_prep = Some(self.t);
                self.espera_pelota = 0.0;
            }
            _ => {}
        }
        if self.fase == Fase::Preparando
            && let Some(t0) = self.t_prep
        {
            let max = self.p().preparacion_max_s / self.escala;
            if self.t - t0 - self.espera_pelota > max {
                let causa = self.falta.as_ref().map_or_else(
                    || format!("no quedó lista en {} s", segundos(max)),
                    |f| f.vencida(max),
                );
                self.abortar_por(aborto(TipoAborto::TiempoVencido, causa));
                return Vec::new();
            }
        }
        out
    }

    // ─── Ejecución ───────────────────────────────────────────────────────────

    fn ejecutar(&mut self, world: &World) -> Vec<SkillChoice> {
        let mut out = Vec::new();
        // A lo sumo un cierre de vuelta por tick.
        for _ in 0..2 {
            out.clear();
            for k in 0..self.asignados.len() {
                let id = self.asignados[k];
                let Some(r) = world.get_robot_state(id, self.ent.own_team).filter(|r| r.active).cloned() else {
                    continue;
                };
                match self.paso_puesto(k, &r, world) {
                    Ok(Some(c)) => out.push(c),
                    Ok(None) => {}
                    Err(causa) => {
                        self.abortar_por(causa);
                        return Vec::new();
                    }
                }
            }
            if self.en_curso.is_empty() || !self.en_curso.iter().all(|e| e.hecho) {
                break;
            }
            self.cerrar_vuelta();
            if self.fase != Fase::Ejecutando {
                return Vec::new();
            }
        }
        out
    }

    fn tolerancias(&self, paso: &Paso) -> (f32, f64) {
        (
            paso.tolerancia_m.unwrap_or(self.p().tolerancia_m),
            paso.tolerancia_deg.unwrap_or(self.p().tolerancia_deg).to_radians(),
        )
    }

    /// Tiempo máximo del paso (s, ya dividido por la escala); `None` para los que duran lo
    /// que piden (`esperar` y `girar` por tiempo) y para `recibir`, que lo cuenta desde el
    /// pase (lo maneja `evaluar_recibir`).
    fn tiempo_max(&self, paso: &Paso) -> Option<f64> {
        match paso.accion {
            Accion::Esperar { .. } | Accion::Girar { duracion: Duracion::Tiempo(_), .. } | Accion::Recibir { .. } => None,
            _ => Some(self.tiempo_base(paso)),
        }
    }

    /// `tiempo_max_s` del paso (o el de los parámetros), dividido por la escala.
    fn tiempo_base(&self, paso: &Paso) -> f64 {
        paso.tiempo_max_s.unwrap_or(self.p().tiempo_max_s) / self.escala
    }

    /// Origen del tramo ideal del paso `i`: el destino anterior, o el inicio en la primera
    /// vuelta, o el último destino desde la segunda.
    fn origen_del_tramo(&self, k: usize, i: usize) -> Vec2 {
        let puesto = &self.rutina.puestos[k];
        if let Some(d) = puesto.pasos[..i].iter().rev().find_map(|p| p.accion.destino()) {
            return d;
        }
        if self.repeticion > 1
            && let Some(d) = puesto.destinos().last()
        {
            return d;
        }
        puesto.inicio.pos
    }

    /// Avanza los pasos del puesto `k` que terminan en este tick y devuelve la elección del
    /// paso en curso (`None` si ya hizo su vuelta), o la causa de aborto.
    fn paso_puesto(&mut self, k: usize, r: &RobotState, world: &World) -> Result<Option<SkillChoice>, Aborto> {
        let rutina = self.rutina.clone();
        let puesto = &rutina.puestos[k];
        let total = puesto.pasos.len();
        for _ in 0..=total {
            if self.en_curso[k].hecho {
                return Ok(None);
            }
            let i = self.en_curso[k].paso;
            let paso = &puesto.pasos[i];
            match self.evaluar(k, i, paso, r, world)? {
                Evaluacion::Termina => self.en_curso[k].siguiente_paso(self.t, total),
                Evaluacion::Sigue(c) => {
                    let e = &self.en_curso[k];
                    let (dt, orientando) = (self.t - e.t_paso, e.orientando);
                    if let Some(max) = self.tiempo_max(paso)
                        && dt > max
                    {
                        return Err(aborto(TipoAborto::TiempoVencido, frase_fuera_de_tiempo(r.id, i, paso, max, orientando)));
                    }
                    return Ok(Some(c));
                }
            }
        }
        Ok(None)
    }

    fn evaluar(&mut self, k: usize, i: usize, paso: &Paso, r: &RobotState, world: &World) -> Result<Evaluacion, Aborto> {
        match &paso.accion {
            Accion::Empujar { hacia } => return self.evaluar_empujar(k, i, hacia, r, world),
            Accion::Recibir { punto } => return self.evaluar_recibir(k, i, *punto, paso, r, world),
            _ => {}
        }
        let (tol, tol_ang) = self.tolerancias(paso);
        let bidir = self.ent.bidireccional;
        let t = self.t;
        let origen = self.origen_del_tramo(k, i);
        let e = &mut self.en_curso[k];
        let dt = t - e.t_paso;
        Ok(match &paso.accion {
            Accion::Ir { punto, orientacion } => {
                if !e.orientando {
                    if (r.position - *punto).length() > tol {
                        let err = f64::from(distancia_a_tramo(r.position, origen, *punto));
                        e.err_suma += err;
                        e.err_n += 1;
                        e.err_max = e.err_max.max(err);
                        return Ok(Evaluacion::Sigue(choice(r.id, SkillId::GoTo, *punto)));
                    }
                    if orientacion.is_none() {
                        return Ok(Evaluacion::Termina);
                    }
                    e.orientando = true;
                }
                let th = orientacion.unwrap_or(r.orientation);
                if error_heading(th, r.orientation, bidir).abs() <= tol_ang {
                    Evaluacion::Termina
                } else {
                    Evaluacion::Sigue(choice(r.id, SkillId::FacePoint, mira_hacia(*punto, th)))
                }
            }
            Accion::Mirar { punto } => {
                let dir = *punto - r.position;
                if dir.length_squared() < f32::EPSILON {
                    return Ok(Evaluacion::Termina);
                }
                let th = f64::from(dir.y).atan2(f64::from(dir.x));
                if error_heading(th, r.orientation, bidir).abs() <= tol_ang {
                    Evaluacion::Termina
                } else {
                    Evaluacion::Sigue(choice(r.id, SkillId::FacePoint, *punto))
                }
            }
            Accion::Girar { sentido, duracion } => {
                if let Some(prev) = e.theta_prev {
                    e.giro += Motion::normalize_angle(r.orientation - prev);
                }
                e.theta_prev = Some(r.orientation);
                let signo = sentido.signo();
                let hecho = match duracion {
                    Duracion::Vueltas(v) => e.giro * signo >= std::f64::consts::TAU * v,
                    Duracion::Tiempo(s) => dt + 1e-9 >= *s,
                };
                if hecho {
                    Evaluacion::Termina
                } else {
                    Evaluacion::Sigue(choice(r.id, SkillId::Spin, Vec2::new(signo as f32, 0.0)))
                }
            }
            Accion::Esperar { tiempo_s } => {
                if dt + 1e-9 >= *tiempo_s {
                    Evaluacion::Termina
                } else {
                    Evaluacion::Sigue(choice(r.id, SkillId::Hold, Vec2::ZERO))
                }
            }
            otro => {
                return Err(aborto(
                    TipoAborto::PasoNoDisponible,
                    format!("el paso `{}` no está disponible en esta fase", otro.tipo()),
                ));
            }
        })
    }

    // ─── Pasos con pelota ────────────────────────────────────────────────────

    /// `empujar`: se alinea detrás de la pelota y empuja hacia el objetivo hasta soltarla o
    /// hasta avanzar `pase_empuje_max_m` empujando (D2). Al terminar, el pase queda hecho
    /// para el `recibir` emparejado. Mientras se alinea, si no hay lugar detrás de la pelota,
    /// aborta enseguida.
    fn evaluar_empujar(
        &mut self,
        k: usize,
        i: usize,
        hacia: &Hacia,
        r: &RobotState,
        world: &World,
    ) -> Result<Evaluacion, Aborto> {
        let Some(&objetivo) = self.objetivos.get(&(k, i)) else {
            // `a_puesto` sin `recibir` emparejado: la validación no deja correr la rutina.
            return Ok(Evaluacion::Termina);
        };
        if self.en_curso[k].empuje_desde.is_none()
            && let Some(razon) = sin_lugar_detras(&ApproachAlignedSkill::new(objetivo), world.get_ball_state().position)
        {
            let para = match hacia {
                Hacia::Puesto(n) => format!("pasarla a {n}"),
                Hacia::Punto(_) => "empujarla".to_string(),
            };
            return Err(aborto(TipoAborto::SinLugar, format!("no hay lugar detrás de la pelota para {para}: {razon}")));
        }
        Ok(match self.empujar_hacia(k, objetivo, r, world) {
            Some(c) => Evaluacion::Sigue(c),
            None => {
                if let Some(&receptor) = self.receptor_de.get(&(k, i)) {
                    self.pases_hechos.insert(receptor, (self.t, world.get_ball_state().position));
                    self.pases_puesto[k].0 += 1;
                    self.pases_vuelta.0 += 1;
                }
                Evaluacion::Termina
            }
        })
    }

    /// ApproachAligned hasta quedar detrás de la pelota y alineado; después ShootPush, que
    /// vuelve a ApproachAligned si deja de ser factible. `None` cuando terminó de empujar:
    /// ShootPush soltó la pelota (o llegó), o el robot avanzó `pase_empuje_max_m` empujando,
    /// medido hacia el objetivo (un robot que se corre de la línea no cuenta como empuje). Las
    /// skills se evalúan con instancias propias: sus `is_done` y su factibilidad son
    /// funciones de la pose, la pelota y el objetivo.
    fn empujar_hacia(
        &mut self,
        k: usize,
        objetivo: Vec2,
        r: &RobotState,
        world: &World,
    ) -> Option<SkillChoice> {
        let ball = world.get_ball_state().position;
        let mut alinear = ApproachAlignedSkill::new(objetivo);
        alinear.bidirectional = self.ent.bidireccional;
        let empujar = ShootPushSkill::new(objetivo);
        let e = &mut self.en_curso[k];
        let Some(desde) = e.empuje_desde else {
            if alinear.is_done(r, world) {
                e.empuje_desde = Some(r.position);
                return Some(choice(r.id, SkillId::ShootPush, objetivo));
            }
            return Some(choice(r.id, SkillId::ApproachAligned, objetivo));
        };
        if empujar.is_done(r, world) {
            return None;
        }
        if !empujar.is_feasible(r, ball) {
            e.empuje_desde = None;
            return Some(choice(r.id, SkillId::ApproachAligned, objetivo));
        }
        let avance = (r.position - desde).dot((objetivo - desde).normalize_or_zero());
        if avance >= self.p().pase_empuje_max_m {
            return None;
        }
        Some(choice(r.id, SkillId::ShootPush, objetivo))
    }

    /// `recibir`: Mark mirando a la pelota, en su punto o, mientras la pelota viene hacia él,
    /// donde va a cruzar su línea (`punto_de_espera`); cuando se detuvo a `recepcion_radio_m`
    /// o menos del punto, Intercept hasta controlarla. La recepción falla si la pelota se
    /// detiene fuera del radio o se vence el tiempo, contado desde el pase. Al fallar, controla
    /// la pelota donde quedó, si puede (`pelota_perdida`), y sigue con su paso siguiente, con un
    /// tope de recepciones fallidas seguidas (D3 y D4).
    fn evaluar_recibir(
        &mut self,
        k: usize,
        i: usize,
        punto: Vec2,
        paso: &Paso,
        r: &RobotState,
        world: &World,
    ) -> Result<Evaluacion, Aborto> {
        let max = self.tiempo_base(paso);
        let b = world.get_ball_state();
        let quieta = b.velocity.length() < self.ent.pelota_lenta;
        let controla = (b.position - r.position).length() <= self.ent.alcance_m && quieta;
        if let Some(t0) = self.en_curso[k].recuperando {
            if let Some(causa) = self.pelota_perdida(k, i, punto, b.position) {
                return Err(causa);
            }
            if self.t - t0 > max {
                return Err(aborto(
                    TipoAborto::TiempoVencido,
                    format!("el robot {} no pudo recuperar la pelota en {} s", r.id, segundos(max)),
                ));
            }
            let e = &mut self.en_curso[k];
            e.control = if controla { e.control + 1 } else { 0 };
            if e.control >= TICKS_CONTROL {
                self.recuperaciones += 1;
                return Ok(Evaluacion::Termina);
            }
            return Ok(Evaluacion::Sigue(choice(r.id, SkillId::Intercept, b.position)));
        }
        let Some(&(t_pase, origen)) = self.pases_hechos.get(&(k, i)) else {
            return Ok(Evaluacion::Sigue(choice(r.id, SkillId::Mark, punto)));
        };
        let radio = self.p().recepcion_radio_m;
        let en_radio = (b.position - punto).length() <= radio;
        let n = (punto - origen).normalize_or_zero();
        let llego = (b.position - r.position).length() <= self.ent.alcance_m;
        let cruzo = n != Vec2::ZERO && (punto - b.position).dot(n) <= 0.0;
        let e = &mut self.en_curso[k];
        e.contacto |= llego || cruzo;
        let contacto = e.contacto;
        e.control = if controla { e.control + 1 } else { 0 };
        e.afuera = if quieta && !en_radio { e.afuera + 1 } else { 0 };
        let afuera = e.afuera;
        if e.control >= TICKS_CONTROL {
            if let Some(&(emisor, _)) = self.emisor_de.get(&(k, i)) {
                self.pases_puesto[emisor].1 += 1;
            }
            self.pases_vuelta.1 += 1;
            self.fallidas_seguidas = 0;
            return Ok(Evaluacion::Termina);
        }
        if self.t - t_pase > max || afuera >= TICKS_CONTROL {
            self.fallidas_seguidas += 1;
            if self.fallidas_seguidas >= self.p().reintentos_max {
                let n = self.fallidas_seguidas;
                let pases = if n == 1 { "1 pase".to_string() } else { format!("{n} pases seguidos") };
                return Err(aborto(TipoAborto::LimiteReintentos, format!("el robot {} no controló la pelota en {pases}", r.id)));
            }
            if let Some(causa) = self.pelota_perdida(k, i, punto, b.position) {
                return Err(causa);
            }
            let e = &mut self.en_curso[k];
            e.recuperando = Some(self.t);
            e.control = 0;
            return Ok(Evaluacion::Sigue(choice(r.id, SkillId::Intercept, b.position)));
        }
        // Va por la pelota cuando se detuvo dentro del radio o, después del contacto, cuando se
        // le escapa despacio dentro del radio; si no, la espera de frente (no sale a buscarla
        // ni la persigue fuera de su zona).
        let escapa = contacto && b.velocity.length() < PELOTA_ESCAPA_MAX;
        if en_radio && (quieta || escapa) {
            Ok(Evaluacion::Sigue(choice(r.id, SkillId::Intercept, b.position)))
        } else {
            let espera = punto_de_espera(punto, origen, b.position, b.velocity, self.ent.pelota_lenta, radio);
            Ok(Evaluacion::Sigue(choice(r.id, SkillId::Mark, espera)))
        }
    }

    /// Por qué el receptor no puede recuperar la pelota: quedó en la otra mitad (la del que
    /// pasó), en un arco o en un área (D4). `None` si puede ir a controlarla.
    fn pelota_perdida(&self, k: usize, i: usize, punto: Vec2, pelota: Vec2) -> Option<Aborto> {
        if pelota.x * punto.x < 0.0 {
            let de = self.emisor_de.get(&(k, i)).map(|(ke, _)| self.rutina.puestos[*ke].nombre.as_str());
            return Some(aborto(
                TipoAborto::MitadRival,
                match de {
                    Some(n) => format!("la pelota quedó en la mitad de {n}"),
                    None => "la pelota quedó en la otra mitad".to_string(),
                },
            ));
        }
        if pelota.x.abs() > FIELD_HALF_X {
            let lado = if pelota.x < 0.0 { "izquierdo" } else { "derecho" };
            return Some(aborto(TipoAborto::EnArco, format!("la pelota quedó en el arco {lado}")));
        }
        areas()
            .iter()
            .find(|a| a.touches_with(pelota, 0.0))
            .map(|a| aborto(TipoAborto::EnArea, format!("la pelota quedó en {}", nombre_area(a))))
    }

    /// Cierra la vuelta en curso (todos los puestos hicieron sus pasos).
    fn cerrar_vuelta(&mut self) {
        let puestos: Vec<VueltaPuesto> = self
            .en_curso
            .iter()
            .enumerate()
            .map(|(k, e)| VueltaPuesto {
                puesto: self.rutina.puestos[k].nombre.clone(),
                robot: self.asignados[k],
                error_medio_m: (e.err_n > 0).then(|| e.err_suma / f64::from(e.err_n)),
                error_max_m: (e.err_n > 0).then_some(e.err_max),
                escapes: e.escapes,
                guardia: e.guardia,
            })
            .collect();
        self.vueltas.push(Vuelta {
            n: self.repeticion,
            duracion_s: self.t - self.t_vuelta - self.pausa_en_vuelta,
            puestos,
            pases: self.rutina.pelota.map(|_| self.pases_vuelta),
        });
        self.pases_vuelta = (0, 0);
        self.pases_hechos.clear();
        let ultima = matches!(self.op.repeticiones, Repeticiones::Veces(n) if self.repeticion >= n);
        if ultima {
            self.cerrar(Resultado::Completada, None);
        } else {
            self.repeticion += 1;
            self.t_vuelta = self.t;
            self.pausa_en_vuelta = 0.0;
            self.en_curso = (0..self.asignados.len()).map(|_| EnCurso::nuevo(self.t)).collect();
        }
    }

    // ─── Lo que llega del lazo ───────────────────────────────────────────────

    fn observar_contexto(&mut self, ctx: &ContextoTick<'_>) {
        #[cfg(test)]
        assert!(!self.panico, "pánico de prueba en observar");
        self.t = f64::from(ctx.tick) * CONTROL_DT;
        self.modo = ctx.modo;
        let escape: HashSet<i32> = ctx.escapando.iter().copied().collect();
        let guardia: HashSet<i32> = ctx.guardia.iter().copied().collect();
        if self.fase == Fase::Ejecutando {
            for (k, id) in self.asignados.iter().enumerate() {
                let e = &mut self.en_curso[k];
                if escape.contains(id) && !self.previo_escape.contains(id) {
                    e.escapes += 1;
                }
                if guardia.contains(id) && !self.previo_guardia.contains(id) {
                    e.guardia += 1;
                }
            }
        }
        self.previo_escape = escape;
        self.previo_guardia = guardia;
        // Antes de asignar, la rutina espera: el lazo arranca en corte sin visión hasta el
        // primer cuadro, y solo asigna en un tick en que consulta al decisor (juego o STOP).
        if !self.fase.activa() || self.asignados.is_empty() {
            return;
        }
        let causa = if ctx.estop {
            Some(aborto(TipoAborto::Parada, "parada de emergencia"))
        } else {
            match ctx.modo {
                TickMode::Halt => Some(aborto(TipoAborto::Halt, "HALT del árbitro")),
                TickMode::Stop => Some(aborto(TipoAborto::Stop, "STOP del árbitro")),
                TickMode::SinVision => Some(aborto(TipoAborto::CorteSinVision, "corte sin visión de los robots propios")),
                TickMode::Play => None,
            }
        };
        let causa = causa
            .or_else(|| {
                self.asignados
                    .iter()
                    .find(|id| ctx.intervenidos.contains(id))
                    .map(|id| aborto(TipoAborto::Manual, format!("el robot {id} pasó a control manual")))
            })
            .or_else(|| {
                (ctx.teleport_gui && matches!(self.fase, Fase::Ejecutando | Fase::Pausada))
                    .then(|| aborto(TipoAborto::Teleport, "teleport durante la ejecución"))
            });
        if let Some(c) = causa {
            self.abortar_por(c);
        }
    }

    // ─── Vista ───────────────────────────────────────────────────────────────

    fn frase(&self) -> String {
        match self.fase {
            Fase::Preparando if self.asignados.is_empty() => FRASE_ESPERA.to_string(),
            Fase::Preparando | Fase::Lista => frase_preparacion(self.falta.as_ref()),
            Fase::Ejecutando => {
                let reps = self.op.repeticiones;
                self.en_curso
                    .iter()
                    .enumerate()
                    .find(|(_, e)| !e.hecho)
                    .map(|(k, e)| {
                        let paso = &self.rutina.puestos[k].pasos[e.paso];
                        let total = self.rutina.puestos[k].pasos.len();
                        let que = match &paso.accion {
                            Accion::Empujar { hacia: Hacia::Puesto(n) } => format!("pasa la pelota a {n}"),
                            Accion::Empujar { .. } => "empuja la pelota".to_string(),
                            Accion::Recibir { .. } if e.recuperando.is_some() => format!(
                                "recupera la pelota ({} de {})",
                                self.fallidas_seguidas,
                                self.p().reintentos_max
                            ),
                            Accion::Recibir { .. } if self.pases_hechos.contains_key(&(k, e.paso)) => {
                                "recibe la pelota".to_string()
                            }
                            Accion::Recibir { .. } => "espera la pelota".to_string(),
                            otro => verbo(otro.tipo(), e.paso, e.orientando),
                        };
                        frase_ejecucion(self.repeticion, reps, e.paso, total, self.asignados[k], &que)
                    })
                    .unwrap_or_else(|| format!("Vuelta {}: cerrando la vuelta", self.repeticion))
            }
            Fase::Pausada => {
                let (paso, total) = self
                    .en_curso
                    .iter()
                    .enumerate()
                    .find(|(_, e)| !e.hecho)
                    .map_or((0, 0), |(k, e)| (e.paso, self.rutina.puestos[k].pasos.len()));
                frase_pausa(self.repeticion, paso, total)
            }
            Fase::Terminada | Fase::Abortada => match &self.resumen {
                Some(r) => frase_cierre(r),
                None => "Abortada".to_string(),
            },
        }
    }

    fn robots_vista(&self) -> Vec<RobotEnRutina> {
        self.asignados
            .iter()
            .enumerate()
            .map(|(k, id)| {
                let puesto = &self.rutina.puestos[k];
                let total = puesto.pasos.len();
                let en_paso = match self.fase {
                    Fase::Ejecutando | Fase::Pausada => self.en_curso.get(k).filter(|e| !e.hecho).map(|e| e.paso),
                    _ => None,
                };
                let (accion, destino) = match en_paso {
                    Some(i) => {
                        let a = &puesto.pasos[i].accion;
                        let destino = match a {
                            Accion::Ir { punto, .. } | Accion::Mirar { punto } | Accion::Recibir { punto } => Some(*punto),
                            Accion::Empujar { .. } => self.objetivos.get(&(k, i)).copied(),
                            _ => None,
                        };
                        (Some(a.tipo()), destino)
                    }
                    None if matches!(self.fase, Fase::Preparando | Fase::Lista) => (None, Some(puesto.inicio.pos)),
                    None => (None, None),
                };
                RobotEnRutina { robot: *id, puesto: k, paso: en_paso, total_pasos: total, accion, destino }
            })
            .collect()
    }

    fn refrescar_vista(&mut self) {
        let nueva = RutinaVista {
            rutina: self.rutina.clone(),
            fase: self.fase,
            frase: self.frase(),
            velocidad_pct: self.op.velocidad_pct,
            repeticion: self.repeticion,
            repeticiones: self.op.repeticiones,
            robots: self.robots_vista(),
            resumen: self.resumen.clone(),
        };
        if nueva != *self.vista {
            self.vista = Arc::new(nueva);
        }
    }
}

impl TickDecider for RoutineDecider {
    fn decide(&mut self, tick: u32, world: &World) -> Vec<SkillChoice> {
        self.decidir(tick, world, SystemTime::now())
    }

    fn observar(&mut self, ctx: &ContextoTick<'_>) {
        self.observar_contexto(ctx);
        self.refrescar_vista();
    }

    /// Desde Preparar hasta que termina o se aborta (D7).
    fn perfil_velocidad(&self) -> Option<PerfilVelocidad> {
        self.fase.activa().then(|| PerfilVelocidad {
            escala: self.escala,
            omega_giro: self.ent.spin_omega.min(self.p().giro_max_rad_s) * self.escala,
        })
    }

    fn tomar_teleport(&mut self) -> Vec<TeleportItem> {
        std::mem::take(&mut self.teleport)
    }

    fn rutina(&self) -> Option<Arc<RutinaVista>> {
        Some(self.vista.clone())
    }
}

/// Hold para cada robot propio activo con id < `num_robots` (lo usa el conmutador cuando
/// la rutina falló).
pub(crate) fn hold_a_todos(world: &World, own_team: i32, num_robots: usize) -> Vec<SkillChoice> {
    let propios = if own_team == 0 { world.get_blue_team_active() } else { world.get_yellow_team_active() };
    let mut ids: Vec<i32> = propios
        .into_iter()
        .map(|r| r.id)
        .filter(|id| *id >= 0 && (*id as usize) < num_robots)
        .collect();
    ids.sort();
    ids.into_iter().map(|id| choice(id, SkillId::Hold, Vec2::ZERO)).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rutinas::prueba;
    use crate::vision_tools::XorShift64;

    const INICIO: Vec2 = Vec2::new(-0.3, -0.25);

    fn entorno(bidireccional: bool) -> Entorno {
        Entorno {
            own_team: 0,
            num_robots: 3,
            keeper_id: 2,
            bidireccional,
            blind_stop_s: 0.4,
            spin_omega: 20.0,
            params: RutinasParams::default(),
            alcance_m: 0.09,
            pelota_lenta: 0.08,
        }
    }

    fn opciones(inicio_automatico: bool) -> OpcionesCorrida {
        OpcionesCorrida {
            eleccion: Eleccion::Automatica,
            velocidad_pct: 100,
            repeticiones: Repeticiones::Veces(1),
            teletransportar: false,
            simulacion: true,
            inicio_automatico,
        }
    }

    fn con_texto(texto: &str, op: OpcionesCorrida, bidir: bool) -> RoutineDecider {
        RoutineDecider::new(prueba::rutina(texto), op, entorno(bidir))
    }

    fn decisor(pasos: &str, op: OpcionesCorrida) -> RoutineDecider {
        con_texto(&prueba::un_puesto(pasos), op, false)
    }

    /// Mundo con robots propios (azules): (id, x, y, θ en grados).
    fn mundo(robots: &[(i32, f32, f32, f64)]) -> World {
        let mut w = World::new(3, 3);
        for &(id, x, y, deg) in robots {
            w.update_robot(id, 0, Vec2::new(x, y), deg.to_radians(), Vec2::ZERO, 0.0);
        }
        w.update_ball(Vec2::new(0.0, -0.55), Vec2::ZERO);
        w
    }

    fn en_inicio() -> World {
        mundo(&[(0, INICIO.x, INICIO.y, 0.0)])
    }

    /// `n` ticks desde `t0` con el mundo `w`; las elecciones del último.
    fn correr(d: &mut RoutineDecider, w: &World, t0: u32, n: u32) -> Vec<SkillChoice> {
        let ahora = SystemTime::now();
        let mut out = Vec::new();
        for t in t0..t0 + n {
            out = d.decidir(t, w, ahora);
        }
        out
    }

    /// Corre con el robot quieto en el inicio hasta que arranca (inicio automático) y
    /// devuelve el tick del arranque.
    fn hasta_ejecutar(d: &mut RoutineDecider, w: &World) -> u32 {
        let ahora = SystemTime::now();
        for t in 0..200 {
            d.decidir(t, w, ahora);
            if d.fase() == Fase::Ejecutando {
                return t;
            }
        }
        panic!("no arrancó: {}", d.vista().frase);
    }

    fn ctx<'a>(tick: u32, modo: TickMode, estop: bool, intervenidos: &'a [i32], teleport_gui: bool) -> ContextoTick<'a> {
        ContextoTick { tick, modo, estop, intervenidos, teleport_gui, escapando: &[], guardia: &[] }
    }

    const IR: &str = r#"{ "paso": "ir", "x": 0.3, "y": -0.25 }"#;

    #[test]
    fn prepares_with_goto_then_facepoint_and_gets_ready() {
        let mut d = decisor(IR, opciones(false));
        let c = correr(&mut d, &mundo(&[(0, 0.3, 0.3, 0.0)]), 0, 1);
        assert_eq!(c, vec![choice(0, SkillId::GoTo, INICIO)]);
        assert_eq!(d.vista().frase, "Preparando: el robot 0 va a su puesto (A)");
        let c = correr(&mut d, &mundo(&[(0, INICIO.x, INICIO.y, 90.0)]), 1, 1);
        assert_eq!(c, vec![choice(0, SkillId::FacePoint, INICIO + Vec2::new(MIRA_LEJOS_M, 0.0))]);
        assert_eq!(d.vista().frase, "Preparando: el robot 0 se orienta en su puesto");
        correr(&mut d, &en_inicio(), 2, 40);
        assert_eq!(d.fase(), Fase::Lista);
        assert_eq!(d.vista().frase, "Lista: robots en su puesto y quietos");
        // En la GUI espera a Iniciar.
        correr(&mut d, &en_inicio(), 42, 100);
        assert_eq!(d.fase(), Fase::Lista);
        d.ordenar(OrdenCorrida::Iniciar);
        let c = correr(&mut d, &en_inicio(), 142, 1);
        assert_eq!(c, vec![choice(0, SkillId::GoTo, Vec2::new(0.3, -0.25))]);
        assert_eq!(d.vista().frase, "Vuelta 1 de 1, paso 1 de 1: el robot 0 va al punto 1");
    }

    #[test]
    fn quiet_with_the_proxy_noise() {
        let mut d = decisor(IR, opciones(false));
        let mut rng = XorShift64::new(7);
        let ahora = SystemTime::now();
        for t in 0..60 {
            let x = INICIO.x + (0.00185 * rng.gauss()) as f32;
            let y = INICIO.y + (0.00185 * rng.gauss()) as f32;
            let th = (0.031 * rng.gauss()).to_degrees();
            d.decidir(t, &mundo(&[(0, x, y, th)]), ahora);
        }
        assert_eq!(d.fase(), Fase::Lista, "{}", d.vista().frase);
    }

    #[test]
    fn moving_slowly_is_not_quiet() {
        let mut d = decisor(IR, opciones(false));
        let ahora = SystemTime::now();
        for t in 0..90 {
            // 3 cm/s, siempre dentro de la tolerancia del inicio.
            let x = INICIO.x - 0.02 + 0.03 * (t as f32 * 0.016);
            d.decidir(t, &mundo(&[(0, x, INICIO.y, 0.0)]), ahora);
        }
        assert_eq!(d.fase(), Fase::Preparando);
        assert_eq!(d.vista().frase, "Preparando: esperando que el robot 0 quede quieto");
    }

    #[test]
    fn backwards_is_in_position_only_in_bidirectional() {
        let atras = mundo(&[(0, INICIO.x, INICIO.y, 180.0)]);
        let mut bidir = con_texto(&prueba::un_puesto(IR), opciones(false), true);
        correr(&mut bidir, &atras, 0, 40);
        assert_eq!(bidir.fase(), Fase::Lista);
        let mut frontal = con_texto(&prueba::un_puesto(IR), opciones(false), false);
        correr(&mut frontal, &atras, 0, 40);
        assert_eq!(frontal.vista().frase, "Preparando: el robot 0 se orienta en su puesto");
    }

    #[test]
    fn unassigned_robot_near_the_route_blocks_without_aborting() {
        let mut d = decisor(IR, opciones(true));
        // A 0.12 m del recorrido (abajo del tramo), sin puesto.
        let w = mundo(&[(0, INICIO.x, INICIO.y, 0.0), (1, 0.0, -0.37, 0.0)]);
        correr(&mut d, &w, 0, 100);
        assert_eq!(d.fase(), Fase::Preparando);
        assert_eq!(d.vista().frase, "Preparando: el robot 1 está a 0.12 m del recorrido de A");
        // Con el robot lejos, arranca.
        correr(&mut d, &mundo(&[(0, INICIO.x, INICIO.y, 0.0), (1, 0.0, 0.3, 0.0)]), 100, 2);
        assert_eq!(d.fase(), Fase::Ejecutando);
    }

    #[test]
    fn preparation_timeout_scales_with_speed() {
        let lejos = mundo(&[(0, 0.3, 0.3, 0.0)]);
        let mut d = decisor(IR, opciones(false));
        correr(&mut d, &lejos, 0, 1250);
        assert_eq!(d.fase(), Fase::Preparando, "antes de 20 s no aborta");
        correr(&mut d, &lejos, 1250, 3);
        assert_eq!(d.causa(), Some("el robot 0 no llegó a su puesto en 20 s"));
        // Al 60 %: 20 / 0.6 = 33.3 s.
        let mut d = decisor(IR, OpcionesCorrida { velocidad_pct: 60, ..opciones(false) });
        correr(&mut d, &lejos, 0, 2080);
        assert_eq!(d.fase(), Fase::Preparando, "a los 33.3 s, no a los 20");
        correr(&mut d, &lejos, 2080, 6);
        assert_eq!(d.causa(), Some("el robot 0 no llegó a su puesto en 33.3 s"));
    }

    #[test]
    fn a_step_that_does_not_arrive_aborts_with_its_phrase() {
        let mut d = decisor(IR, opciones(true));
        let w = en_inicio();
        let t0 = hasta_ejecutar(&mut d, &w);
        correr(&mut d, &w, t0 + 1, 498);
        assert_eq!(d.fase(), Fase::Ejecutando, "antes de 8 s no aborta");
        correr(&mut d, &w, t0 + 499, 5);
        assert_eq!(d.causa(), Some("el robot 0 no llegó al punto 1 en 8 s"));
        assert_eq!(d.vista().frase, "Abortada: el robot 0 no llegó al punto 1 en 8 s");
        // Abortada: Hold y sin perfil.
        assert_eq!(correr(&mut d, &w, t0 + 504, 1), vec![choice(0, SkillId::Hold, Vec2::ZERO)]);
        assert!(d.perfil_velocidad().is_none());
    }

    #[test]
    fn look_and_turn_time_out_with_their_phrases() {
        let mut d = decisor(r#"{ "paso": "mirar", "x": -0.3, "y": 0.5, "tiempo_max_s": 2 }"#, opciones(true));
        let w = en_inicio();
        let t0 = hasta_ejecutar(&mut d, &w);
        correr(&mut d, &w, t0 + 1, 130);
        assert_eq!(d.causa(), Some("el robot 0 no se orientó al punto 1 en 2 s"));
        let mut d = decisor(r#"{ "paso": "girar", "sentido": "horario", "vueltas": 2, "tiempo_max_s": 3 }"#, opciones(true));
        let t0 = hasta_ejecutar(&mut d, &w);
        correr(&mut d, &w, t0 + 1, 200);
        assert_eq!(d.causa(), Some("el robot 0 no completó 2 vueltas en 3 s"));
    }

    #[test]
    fn turn_counts_full_turns_across_pi() {
        let texto = prueba::un_puesto(r#"{ "paso": "girar", "sentido": "antihorario", "vueltas": 2 }"#)
            .replace("\"orientacion_deg\": 0", "\"orientacion_deg\": 170");
        let mut d = con_texto(&texto, opciones(true), false);
        let t0 = hasta_ejecutar(&mut d, &mundo(&[(0, INICIO.x, INICIO.y, 170.0)]));
        let ahora = SystemTime::now();
        for k in 1..=62u32 {
            let th = 170f64.to_radians() + 0.2 * f64::from(k);
            let c = d.decidir(t0 + k, &mundo(&[(0, INICIO.x, INICIO.y, th.to_degrees())]), ahora);
            assert_eq!(c, vec![choice(0, SkillId::Spin, Vec2::new(1.0, 0.0))], "k = {k}");
        }
        assert_eq!(d.fase(), Fase::Ejecutando, "a 4π le falta poco");
        let th = 170f64.to_radians() + 0.2 * 63.0;
        d.decidir(t0 + 63, &mundo(&[(0, INICIO.x, INICIO.y, th.to_degrees())]), ahora);
        assert_eq!(d.fase(), Fase::Terminada);
    }

    #[test]
    fn wait_takes_its_time() {
        let mut d = decisor(r#"{ "paso": "esperar", "tiempo_s": 2 }"#, opciones(true));
        let w = en_inicio();
        let t0 = hasta_ejecutar(&mut d, &w);
        let c = correr(&mut d, &w, t0 + 1, 124);
        assert_eq!(c, vec![choice(0, SkillId::Hold, Vec2::ZERO)]);
        assert_eq!(d.fase(), Fase::Ejecutando);
        correr(&mut d, &w, t0 + 125, 1);
        assert_eq!(d.fase(), Fase::Terminada);
        assert_eq!(d.vista().frase, "Terminada: 1 vuelta en 2.0 s");
    }

    #[test]
    fn pause_freezes_the_step_time() {
        let mut d = decisor(IR, opciones(true));
        let w = en_inicio();
        let t0 = hasta_ejecutar(&mut d, &w);
        correr(&mut d, &w, t0 + 1, 312);
        d.ordenar(OrdenCorrida::Pausar);
        assert_eq!(d.fase(), Fase::Pausada);
        assert_eq!(d.vista().frase, "Pausada en la vuelta 1, paso 1 de 1");
        let c = correr(&mut d, &w, t0 + 313, 625);
        assert_eq!(c, vec![choice(0, SkillId::Hold, Vec2::ZERO)]);
        d.ordenar(OrdenCorrida::Reanudar);
        correr(&mut d, &w, t0 + 938, 180);
        assert_eq!(d.fase(), Fase::Ejecutando, "la pausa no consume el tiempo del paso");
        correr(&mut d, &w, t0 + 1118, 20);
        assert_eq!(d.causa(), Some("el robot 0 no llegó al punto 1 en 8 s"));
    }

    #[test]
    fn a_robot_out_of_sight_aborts() {
        let mut d = decisor(IR, opciones(false));
        let w = en_inicio();
        correr(&mut d, &w, 0, 1);
        d.decidir(1, &w, SystemTime::now() + Duration::from_secs(1));
        assert_eq!(d.causa(), Some("el robot 0 dejó de verse"));
        assert_eq!(d.vista().frase, "Abortada: el robot 0 dejó de verse");
    }

    #[test]
    fn near_collision_aborts_and_a_crossing_does_not() {
        let mut d = decisor(IR, opciones(false));
        correr(&mut d, &mundo(&[(0, INICIO.x, INICIO.y, 0.0), (2, INICIO.x + 0.15, INICIO.y, 0.0)]), 0, 20);
        assert!(d.fase().activa(), "0.15 m no aborta: {}", d.vista().frase);
        correr(&mut d, &mundo(&[(0, INICIO.x, INICIO.y, 0.0), (2, INICIO.x + 0.08, INICIO.y, 0.0)]), 20, 1);
        assert_eq!(d.causa(), Some("los robots 0 y 2 quedaron a 0.08 m"));
    }

    #[test]
    fn loop_causes_arrive_through_observar() {
        let w = en_inicio();
        // (modo, parada, intervenidos, teleport de la GUI, causa esperada)
        type Caso<'a> = (TickMode, bool, &'a [i32], bool, Option<&'a str>);
        let casos: [Caso; 8] = [
            (TickMode::Halt, false, &[], false, Some("HALT del árbitro")),
            (TickMode::Halt, true, &[], false, Some("parada de emergencia")),
            (TickMode::Stop, false, &[], false, Some("STOP del árbitro")),
            (TickMode::SinVision, false, &[], false, Some("corte sin visión de los robots propios")),
            (TickMode::Play, false, &[0], false, Some("el robot 0 pasó a control manual")),
            (TickMode::Play, false, &[1], false, None),
            (TickMode::Play, false, &[], true, Some("teleport durante la ejecución")),
            (TickMode::Play, false, &[], false, None),
        ];
        for (modo, estop, interv, tp, causa) in casos {
            let mut d = decisor(IR, opciones(true));
            let t0 = hasta_ejecutar(&mut d, &w);
            d.observar(&ctx(t0, modo, estop, interv, tp));
            assert_eq!(d.causa(), causa, "{modo:?} estop={estop} {interv:?} tp={tp}");
            if causa.is_some() {
                assert_eq!(d.fase(), Fase::Abortada);
                assert_eq!(d.resumen().unwrap().fase_aborto, Some(Fase::Ejecutando));
            }
        }
    }

    #[test]
    fn before_assigning_it_waits_for_vision_and_play() {
        // El lazo arranca en corte sin visión hasta el primer cuadro, y en HALT no consulta
        // al decisor: la rutina espera, sin abortar.
        let mut d = decisor(IR, opciones(true));
        assert_eq!(d.vista().frase, "Preparando");
        d.observar(&ctx(0, TickMode::SinVision, false, &[], false));
        d.observar(&ctx(1, TickMode::Halt, false, &[], false));
        assert_eq!(d.fase(), Fase::Preparando);
        assert_eq!(d.vista().frase, FRASE_ESPERA);
        // Con el primer tick de juego asigna; desde ahí, el corte aborta.
        correr(&mut d, &en_inicio(), 2, 1);
        assert_eq!(d.asignados(), &[0]);
        d.observar(&ctx(2, TickMode::SinVision, false, &[], false));
        assert_eq!(d.causa(), Some("corte sin visión de los robots propios"));
    }

    #[test]
    fn gui_teleport_while_preparing_does_not_abort() {
        let mut d = decisor(IR, opciones(false));
        correr(&mut d, &en_inicio(), 0, 5);
        d.observar(&ctx(5, TickMode::Play, false, &[], true));
        assert_eq!(d.fase(), Fase::Preparando);
        // STOP en la preparación sí aborta.
        d.observar(&ctx(6, TickMode::Stop, false, &[], false));
        assert_eq!(d.causa(), Some("STOP del árbitro"));
        assert_eq!(d.resumen().unwrap().fase_aborto, Some(Fase::Preparando));
    }

    #[test]
    fn user_abort_and_loop_stop() {
        let w = en_inicio();
        let mut d = decisor(IR, opciones(true));
        let t0 = hasta_ejecutar(&mut d, &w);
        d.ordenar(OrdenCorrida::Abortar);
        assert_eq!(d.resumen().unwrap().resultado, Resultado::Abortada);
        assert_eq!(d.vista().frase, "Abortada por el usuario");
        let mut b = decisor(IR, OpcionesCorrida { repeticiones: Repeticiones::Bucle, ..opciones(true) });
        hasta_ejecutar(&mut b, &w);
        b.ordenar(OrdenCorrida::Abortar);
        assert_eq!(b.resumen().unwrap().resultado, Resultado::Detenida);
        assert_eq!(b.vista().frase, "Detenida por el usuario sin vueltas completas");
        // Una orden fuera de fase no hace nada.
        let mut e = decisor(IR, opciones(false));
        correr(&mut e, &w, t0, 1);
        e.ordenar(OrdenCorrida::Pausar);
        e.ordenar(OrdenCorrida::Iniciar);
        assert_eq!(e.fase(), Fase::Preparando);
    }

    #[test]
    fn profile_while_active_only() {
        let mut d = decisor(IR, OpcionesCorrida { velocidad_pct: 60, ..opciones(false) });
        let p = d.perfil_velocidad().expect("desde Preparar");
        assert_eq!(p.escala, 0.6);
        assert_eq!(p.omega_giro, 20f64.min(12.0) * 0.6);
        d.ordenar(OrdenCorrida::Abortar);
        assert!(d.perfil_velocidad().is_none());
        assert!(velocidad_valida(60) && !velocidad_valida(65) && !velocidad_valida(10) && !velocidad_valida(110));
    }

    #[test]
    fn teleport_once_and_only_in_simulation() {
        let texto = prueba::un_puesto(IR).replace("\"orientacion_deg\": 0", "\"orientacion_deg\": 90");
        let mut w = mundo(&[(0, 0.3, 0.3, 0.0), (1, 0.6, -0.5, 0.0)]);
        w.update_robot(0, 1, Vec2::ZERO, 0.0, Vec2::ZERO, 0.0);
        let mut d = con_texto(&texto, OpcionesCorrida { teletransportar: true, ..opciones(false) }, false);
        correr(&mut d, &w, 0, 1);
        let items = d.tomar_teleport();
        assert_eq!(items.len(), 4, "{items:?}");
        match items[0] {
            TeleportItem::Robot { team: 0, id: 0, x, y, theta } => {
                assert!((x + 0.3).abs() < 1e-6 && (y + 0.25).abs() < 1e-6);
                assert!((theta - 90.0).abs() < 1e-9, "en grados: {theta}");
            }
            ref o => panic!("{o:?}"),
        }
        assert!(matches!(items[1], TeleportItem::Robot { team: 0, id: 1, y, .. } if y == PARQUE_Y));
        assert!(matches!(items[2], TeleportItem::Robot { team: 1, id: 0, y, .. } if y == PARQUE_Y));
        assert!(matches!(items[3], TeleportItem::Ball { y, .. } if y == -PARQUE_Y));
        assert!(d.tomar_teleport().is_empty(), "una sola vez");
        let mut real = con_texto(&texto, OpcionesCorrida { teletransportar: true, simulacion: false, ..opciones(false) }, false);
        correr(&mut real, &w, 0, 1);
        assert!(real.tomar_teleport().is_empty(), "con el robot real no hay teleport");
    }

    #[test]
    fn summary_error_from_the_ideal_route() {
        let mut d = decisor(IR, opciones(true));
        let t0 = hasta_ejecutar(&mut d, &mundo(&[(0, INICIO.x, INICIO.y + 0.02, 0.0)]));
        let ahora = SystemTime::now();
        let mut k = 1;
        while d.fase() == Fase::Ejecutando && k < 100 {
            let x = INICIO.x + 0.01 * k as f32;
            d.decidir(t0 + k, &mundo(&[(0, x, INICIO.y + 0.02, 0.0)]), ahora);
            k += 1;
        }
        assert_eq!(d.fase(), Fase::Terminada);
        let r = d.resumen().unwrap();
        let a = &r.vueltas[0].puestos[0];
        assert!((a.error_medio_m.unwrap() - 0.02).abs() < 1e-4, "{a:?}");
        assert!((a.error_max_m.unwrap() - 0.02).abs() < 1e-4, "{a:?}");
        assert_eq!(r.asignacion, vec![("A".to_string(), 0)]);
        assert!(r.preparacion_s.is_some());
    }

    #[test]
    fn escapes_and_guard_count_episodes() {
        let mut d = decisor(r#"{ "paso": "esperar", "tiempo_s": 1 }"#, opciones(true));
        let w = en_inicio();
        let t0 = hasta_ejecutar(&mut d, &w);
        let ahora = SystemTime::now();
        for k in 1..=70u32 {
            if d.fase() != Fase::Ejecutando {
                break;
            }
            d.decidir(t0 + k, &w, ahora);
            let escapa: &[i32] = if (5..30).contains(&k) { &[0] } else { &[] };
            let frena: &[i32] = if k == 10 || k == 20 { &[0] } else { &[] };
            d.observar(&ContextoTick { escapando: escapa, guardia: frena, ..ctx(t0 + k, TickMode::Play, false, &[], false) });
        }
        let r = d.resumen().expect("terminó");
        assert_eq!(r.resultado, Resultado::Completada);
        assert_eq!((r.vueltas[0].puestos[0].escapes, r.vueltas[0].puestos[0].guardia), (1, 2));
    }

    #[test]
    fn no_robots_means_it_cannot_prepare() {
        let mut d = decisor(IR, opciones(false));
        let c = correr(&mut d, &World::new(3, 3), 0, 1);
        assert!(c.is_empty());
        assert_eq!(d.causa(), Some("no se pudo preparar: se ven 0 robots propios; la rutina pide 1 robot"));
    }

    #[test]
    fn unassigned_robots_hold_and_only_catalog_skills_are_used() {
        let texto = prueba::un_puesto(
            r#"{ "paso": "ir", "x": 0.3, "y": -0.25, "orientacion_deg": 90 }, { "paso": "mirar", "x": 0.3, "y": 0.5 },
               { "paso": "girar", "sentido": "horario", "tiempo_s": 0.5 }, { "paso": "esperar", "tiempo_s": 0.5 }"#,
        );
        let mut d = con_texto(&texto, opciones(true), false);
        let otro = (2, 0.3, 0.4, 0.0);
        let mut usadas = HashSet::new();
        let ahora = SystemTime::now();
        // Recorre los pasos moviendo el robot a mano: inicio, punto 1 (mirando al norte), etc.
        let poses = [(INICIO.x, INICIO.y, 0.0, 40), (0.3, -0.25, 0.0, 5), (0.3, -0.25, 90.0, 5), (0.3, -0.25, 90.0, 80)];
        let mut t = 0;
        for (x, y, th, n) in poses {
            for _ in 0..n {
                for c in d.decidir(t, &mundo(&[(0, x, y, th), otro]), ahora) {
                    if c.robot_id == 2 {
                        assert_eq!(c.skill_id, SkillId::Hold, "el robot sin puesto hace Hold");
                    }
                    usadas.insert(c.skill_id);
                }
                t += 1;
            }
        }
        assert_eq!(d.fase(), Fase::Terminada, "{}", d.vista().frase);
        let permitidas = [SkillId::GoTo, SkillId::FacePoint, SkillId::Spin, SkillId::Hold];
        assert!(usadas.iter().all(|s| permitidas.contains(s)), "{usadas:?}");
        assert_eq!(usadas.len(), 4, "usó las cuatro: {usadas:?}");
    }

    // ─── Pases ───────────────────────────────────────────────────────────────

    const A: Vec2 = Vec2::new(-0.40, 0.0);
    const B: Vec2 = Vec2::new(0.40, 0.0);
    const MARCA: Vec2 = Vec2::new(-0.28, 0.0);

    /// El robot 0 (puesto A) y el 1 (puesto B), con su orientación en grados, y la pelota.
    fn cancha(a: (Vec2, f64), b: (Vec2, f64), pelota: Vec2, vel: Vec2) -> World {
        let mut w = World::new(3, 3);
        w.update_robot(0, 0, a.0, a.1.to_radians(), Vec2::ZERO, 0.0);
        w.update_robot(1, 0, b.0, b.1.to_radians(), Vec2::ZERO, 0.0);
        w.update_ball(pelota, vel);
        w
    }

    fn pases(reintentos: u32, op: OpcionesCorrida) -> RoutineDecider {
        let mut ent = entorno(false);
        ent.params.reintentos_max = reintentos;
        RoutineDecider::new(prueba::rutina(&prueba::pases()), op, ent)
    }

    fn de(c: &[SkillChoice], id: i32) -> SkillChoice {
        *c.iter().find(|c| c.robot_id == id).unwrap_or_else(|| panic!("sin elección para {id}: {c:?}"))
    }

    /// Las skills que una rutina con pelota puede elegir.
    fn solo_permitidas(c: &[SkillChoice]) {
        let permitidas = [
            SkillId::GoTo,
            SkillId::FacePoint,
            SkillId::Spin,
            SkillId::Hold,
            SkillId::ApproachAligned,
            SkillId::ShootPush,
            SkillId::Mark,
            SkillId::Intercept,
        ];
        assert!(c.iter().all(|c| permitidas.contains(&c.skill_id)), "{c:?}");
    }

    /// Corre `n` ticks y revisa que todas las elecciones sean de las permitidas.
    fn correr_p(d: &mut RoutineDecider, w: &World, t0: u32, n: u32) -> Vec<SkillChoice> {
        let ahora = SystemTime::now();
        let mut out = Vec::new();
        for t in t0..t0 + n {
            out = d.decidir(t, w, ahora);
            solo_permitidas(&out);
        }
        out
    }

    /// Hasta la ejecución con los dos en su inicio y la pelota en la marca; después, A
    /// empuja y pasa el corte: el pase queda hecho. Devuelve el tick siguiente.
    fn hasta_el_pase(d: &mut RoutineDecider) -> u32 {
        let t = hasta_ejecutar(d, &cancha((A, 0.0), (B, 180.0), MARCA, Vec2::ZERO));
        let c = correr_p(d, &cancha((A, 0.0), (B, 180.0), MARCA, Vec2::ZERO), t + 1, 1);
        assert_eq!(de(&c, 0), choice(0, SkillId::ShootPush, B), "A ya está detrás de la pelota y alineado");
        assert_eq!(de(&c, 1), choice(1, SkillId::Mark, B), "B espera de frente a la pelota");
        // A avanzó 0.16 m empujando, sin que ShootPush suelte (la pelota a 0.4 m/s).
        let a_empujando = A + Vec2::new(0.16, 0.0);
        let c = correr_p(d, &cancha((a_empujando, 0.0), (B, 180.0), Vec2::new(-0.12, 0.0), Vec2::new(0.4, 0.0)), t + 2, 1);
        assert_eq!(de(&c, 0), choice(0, SkillId::GoTo, A), "el empuje terminó por el corte y A vuelve");
        assert_eq!(de(&c, 1), choice(1, SkillId::Mark, B), "con la pelota rodando, B la espera de frente");
        t + 3
    }

    #[test]
    fn a_pass_is_made_and_received() {
        let mut d = pases(3, opciones(true));
        let t = hasta_el_pase(&mut d);
        // La pelota llega a B y queda quieta a 0.05 m: tres ticks de control.
        let llega = cancha((A, 0.0), (B, 180.0), B - Vec2::new(0.05, 0.0), Vec2::ZERO);
        let c = correr_p(&mut d, &llega, t, 2);
        assert_eq!(de(&c, 1).skill_id, SkillId::Intercept, "dos ticks no alcanzan");
        let c = correr_p(&mut d, &llega, t + 2, 1);
        // Recibió: B sigue con su `empujar` hacia el punto de A.
        let b = de(&c, 1);
        assert!(matches!(b.skill_id, SkillId::ApproachAligned | SkillId::ShootPush) && b.target == A, "{b:?}");
        assert_eq!((d.pases_puesto[0], d.pases_vuelta), ((1, 1), (1, 1)));
        assert_eq!(d.fallidas_seguidas, 0);
        // A, ya en su punto, espera la pelota.
        assert_eq!(de(&c, 0), choice(0, SkillId::Mark, A));
        assert_eq!(d.vista().frase, "Vuelta 1 de 1, paso 3 de 3: el robot 0 espera la pelota");
    }

    #[test]
    fn the_receiver_has_no_time_limit_before_the_pass_but_the_passer_does() {
        let mut d = pases(3, opciones(true));
        let t = hasta_ejecutar(&mut d, &cancha((A, 0.0), (B, 180.0), MARCA, Vec2::ZERO));
        // A queda lejos de la pelota y nunca se alinea: B espera sin abortar por su
        // `recibir`, y la rutina se aborta por el `empujar` de A a los 8 s.
        let lejos = cancha((Vec2::new(-0.40, 0.30), 90.0), (B, 180.0), MARCA, Vec2::ZERO);
        correr_p(&mut d, &lejos, t + 1, 499);
        assert_eq!(d.fase(), Fase::Ejecutando);
        correr_p(&mut d, &lejos, t + 500, 2);
        assert_eq!(d.causa(), Some("el robot 0 no pasó la pelota en 8 s"));
    }

    /// Fuera del radio de recepción de B (0.30 m de su punto) y en su mitad.
    const CORTA: Vec2 = Vec2::new(0.10, 0.0);

    /// Después del pase, la pelota se detiene en `CORTA`: tres ticks quieta fuera del radio
    /// y la recepción falla; B va a controlarla donde quedó.
    fn recepcion_fallida(d: &mut RoutineDecider) -> u32 {
        let t = hasta_el_pase(d);
        let quieta = cancha((A, 0.0), (B, 180.0), CORTA, Vec2::ZERO);
        let c = correr_p(d, &quieta, t, 2);
        assert_eq!(de(&c, 1), choice(1, SkillId::Mark, B), "fuera del radio no va por ella");
        let c = correr_p(d, &quieta, t + 2, 1);
        assert_eq!(de(&c, 1), choice(1, SkillId::Intercept, CORTA), "B va por la pelota donde quedó");
        t + 3
    }

    /// B controla la pelota en su punto, desde el costado (sin quedar detrás de ella hacia A):
    /// tres ticks y la recuperación termina.
    fn b_recupera(d: &mut RoutineDecider, t: u32) -> Vec<SkillChoice> {
        let al_lado = cancha((A, 0.0), (B + Vec2::new(0.0, 0.06), 270.0), B, Vec2::ZERO);
        let c = correr_p(d, &al_lado, t, 2);
        assert_eq!(de(&c, 1), choice(1, SkillId::Intercept, B), "dos ticks de control no alcanzan");
        assert_eq!(d.recuperaciones, 0);
        correr_p(d, &al_lado, t + 2, 1)
    }

    #[test]
    fn a_failed_reception_recovers_and_goes_on_with_its_own_pass() {
        let mut d = pases(3, opciones(true));
        let t = recepcion_fallida(&mut d);
        assert_eq!(d.fallidas_seguidas, 1);
        assert_eq!(d.pases_puesto[0], (1, 0), "el pase cuenta como intentado y no logrado");
        // La pelota rueda: B la sigue con Intercept, sin volver a su punto.
        let rueda = cancha((A, 0.0), (B, 180.0), CORTA, Vec2::new(0.0, 0.2));
        let c = correr_p(&mut d, &rueda, t, 1);
        assert_eq!(de(&c, 1), choice(1, SkillId::Intercept, CORTA));
        // La controla: recuperada, y B sigue con su pase a A desde donde quedó.
        let c = b_recupera(&mut d, t + 1);
        assert_eq!(d.recuperaciones, 1);
        assert_eq!(de(&c, 1), choice(1, SkillId::ApproachAligned, A), "B pasa desde donde controló la pelota");
        d.abortar(TipoAborto::Usuario, "fin de la prueba");
        let p = d.resumen().unwrap().pases.clone().expect("con pelota");
        assert_eq!((p.intentados, p.logrados, p.recuperaciones), (1, 0, 1));
        assert_eq!(p.por_puesto, vec![("A".to_string(), 1, 0)]);
    }

    #[test]
    fn a_recovery_out_of_time_aborts() {
        let mut d = pases(3, opciones(true));
        let t = recepcion_fallida(&mut d);
        let lejos = cancha((A, 0.0), (B, 180.0), CORTA, Vec2::ZERO);
        correr_p(&mut d, &lejos, t, 510);
        assert_eq!(d.causa(), Some("el robot 1 no pudo recuperar la pelota en 8 s"));
    }

    /// B recupera la pelota y le pasa a A (pase hecho); devuelve el tick siguiente.
    fn b_le_pasa_a_a(d: &mut RoutineDecider, t: u32) -> u32 {
        b_recupera(d, t);
        let t = t + 3;
        let b_detras = B + Vec2::new(0.14, 0.0);
        let c = correr_p(d, &cancha((A, 0.0), (b_detras, 180.0), B, Vec2::ZERO), t, 2);
        assert_eq!(de(&c, 1), choice(1, SkillId::ShootPush, A), "B detrás de la pelota y alineado");
        let b_empujando = b_detras - Vec2::new(0.16, 0.0);
        let rodando = cancha((A, 0.0), (b_empujando, 180.0), Vec2::new(0.26, 0.0), Vec2::new(-0.4, 0.0));
        let c = correr_p(d, &rodando, t + 2, 1);
        // El empuje de B terminó por el corte; B quedó a 2 cm de su punto, así que su `ir`
        // también termina y espera a A con Hold.
        assert_eq!(de(&c, 1), choice(1, SkillId::Hold, Vec2::ZERO));
        // A se evalúa antes que B en cada tick: desde el tick siguiente ve el pase hecho y,
        // con la pelota rodando, la espera de frente en su punto.
        let c = correr_p(d, &rodando, t + 3, 1);
        assert_eq!(de(&c, 0), choice(0, SkillId::Mark, A), "hecho el pase de B, A lo espera");
        assert!(d.pases_hechos.contains_key(&(0, 2)), "el pase a A quedó hecho");
        t + 4
    }

    #[test]
    fn the_retry_limit_aborts_with_its_phrase() {
        // Con `reintentos_max` 2: falla B (1), recupera, le pasa a A y falla A (2).
        let mut d = pases(2, opciones(true));
        let t = recepcion_fallida(&mut d);
        let t = b_le_pasa_a_a(&mut d, t);
        // La pelota se detiene en la mitad de A, fuera de su radio.
        let corta_a = cancha((A, 0.0), (B, 180.0), -CORTA, Vec2::ZERO);
        correr_p(&mut d, &corta_a, t, 2);
        assert_eq!(d.fase(), Fase::Ejecutando);
        correr_p(&mut d, &corta_a, t + 2, 1);
        assert_eq!(d.causa(), Some("el robot 0 no controló la pelota en 2 pases seguidos"));
        assert_eq!(d.fase(), Fase::Abortada);
        // Con `reintentos_max` 1, la primera falla aborta, en singular.
        let mut d = pases(1, opciones(true));
        let t = hasta_el_pase(&mut d);
        correr_p(&mut d, &cancha((A, 0.0), (B, 180.0), CORTA, Vec2::ZERO), t, 3);
        assert_eq!(d.causa(), Some("el robot 1 no controló la pelota en 1 pase"));
    }

    #[test]
    fn a_received_pass_resets_the_count() {
        let mut d = pases(2, opciones(true));
        let t = recepcion_fallida(&mut d);
        let t = b_le_pasa_a_a(&mut d, t);
        assert_eq!(d.fallidas_seguidas, 1);
        // A controla la pelota: la cuenta vuelve a cero.
        let en_a = cancha((A, 0.0), (B, 180.0), A + Vec2::new(0.05, 0.0), Vec2::ZERO);
        correr_p(&mut d, &en_a, t, 3);
        assert_eq!(d.fallidas_seguidas, 0);
        assert_eq!(d.pases_puesto, vec![(1, 0), (1, 1)]);
    }

    #[test]
    fn the_receiver_waits_for_a_rolling_ball_and_goes_for_it_once_stopped() {
        let mut d = pases(3, opciones(true));
        let t = hasta_el_pase(&mut d);
        // Rueda hacia B a 0.3 m/s, ya dentro del radio: B no sale a buscarla.
        let rodando = cancha((A, 0.0), (B, 180.0), B - Vec2::new(0.20, 0.0), Vec2::new(0.3, 0.0));
        let c = correr_p(&mut d, &rodando, t, 3);
        assert_eq!(de(&c, 1), choice(1, SkillId::Mark, B));
        // Rebotó y quedó quieta a 0.15 m de su punto: va por ella.
        let quieta = cancha((A, 0.0), (B, 180.0), B - Vec2::new(0.15, 0.0), Vec2::ZERO);
        let c = correr_p(&mut d, &quieta, t + 3, 1);
        assert_eq!(de(&c, 1).skill_id, SkillId::Intercept);
        assert_eq!(d.fase(), Fase::Ejecutando, "dentro del radio no falla");
    }

    #[test]
    fn a_ball_left_in_the_other_half_aborts() {
        let mut d = pases(3, opciones(true));
        let t = hasta_el_pase(&mut d);
        // La pelota de A se detiene en la mitad de A, fuera del radio de B.
        let en_a = cancha((A, 0.0), (B, 180.0), -CORTA, Vec2::ZERO);
        correr_p(&mut d, &en_a, t, 3);
        assert_eq!(d.causa(), Some("la pelota quedó en la mitad de A"));
        // Y si cruza a la otra mitad durante la recuperación, también.
        let mut d = pases(3, opciones(true));
        let t = recepcion_fallida(&mut d);
        correr_p(&mut d, &en_a, t, 1);
        assert_eq!(d.causa(), Some("la pelota quedó en la mitad de A"));
    }

    #[test]
    fn the_receiver_steps_onto_the_line_of_a_ball_coming_to_it() {
        let mut d = pases(3, opciones(true));
        let t = hasta_el_pase(&mut d);
        let rueda = |pelota: Vec2, vel: Vec2| cancha((A, 0.0), (B, 180.0), pelota, vel);
        // Viene desviada: cruza x = 0.40 en y = 0.04 (0.8 s a 0.5 m/s). B espera ahí, mirándola.
        let c = correr_p(&mut d, &rueda(Vec2::ZERO, Vec2::new(0.5, 0.05)), t, 1);
        let b = de(&c, 1);
        assert_eq!(b.skill_id, SkillId::Mark);
        assert!((b.target - Vec2::new(0.40, 0.04)).length() < 1e-4, "{b:?}");
        // Muy desviada: no sale a más de `recepcion_radio_m` de su punto.
        let c = correr_p(&mut d, &rueda(Vec2::ZERO, Vec2::new(0.5, 0.4)), t + 1, 1);
        assert!((de(&c, 1).target - Vec2::new(0.40, 0.25)).length() < 1e-4, "{:?}", de(&c, 1));
        // Ruido de la visión (más lenta que `intercept_min_ball_speed`), una pelota que se aleja
        // o una que ya pasó su línea: se queda en su punto.
        for (pelota, vel) in [
            (Vec2::ZERO, Vec2::new(0.05, 0.03)),
            (Vec2::ZERO, Vec2::new(-0.3, 0.1)),
            (Vec2::new(0.45, 0.10), Vec2::new(0.3, 0.1)),
        ] {
            let c = correr_p(&mut d, &rueda(pelota, vel), t + 2, 1);
            assert_eq!(de(&c, 1), choice(1, SkillId::Mark, B), "{pelota:?} {vel:?}");
        }
        assert_eq!(d.fase(), Fase::Ejecutando);
    }

    #[test]
    fn the_receiver_does_not_step_into_an_area() {
        // Pase a lo largo de y hacia (0.40, 0): la línea del receptor es y = 0, hacia el arco
        // derecho. La pelota cruzaría en x = 0.52, que toca el área con la holgura.
        let punto = Vec2::new(0.40, 0.0);
        let crudo = Vec2::new(0.52, 0.0);
        assert!(areas().iter().any(|a| a.touches_with(crudo, AREA_CLEARANCE)));
        let q = punto_de_espera(punto, Vec2::new(0.40, -0.40), Vec2::new(0.40, -0.20), Vec2::new(0.3, 0.5), 0.08, 0.25);
        assert!(!areas().iter().any(|a| a.touches_with(q, AREA_CLEARANCE)), "{q:?}");
        assert!(q.y.abs() < 1e-6 && q.x > punto.x && q.x < crudo.x, "entre su punto y el cruce: {q:?}");
    }

    #[test]
    fn a_ball_left_in_a_goal_or_an_area_aborts_at_once() {
        // El pase de A pasa a B y se detiene dentro del arco derecho, o en el área derecha.
        for (pelota, causa) in [
            (Vec2::new(0.78, 0.05), "la pelota quedó en el arco derecho"),
            (Vec2::new(0.65, 0.20), "la pelota quedó en el área derecha"),
        ] {
            let mut d = pases(3, opciones(true));
            let t = hasta_el_pase(&mut d);
            correr_p(&mut d, &cancha((A, 0.0), (B, 180.0), pelota, Vec2::ZERO), t, 3);
            assert_eq!(d.causa(), Some(causa));
            assert_eq!(d.recuperaciones, 0);
        }
        // Y si la recuperación la mete en el área, también.
        let mut d = pases(3, opciones(true));
        let t = recepcion_fallida(&mut d);
        correr_p(&mut d, &cancha((A, 0.0), (B, 180.0), Vec2::new(0.65, 0.20), Vec2::ZERO), t, 1);
        assert_eq!(d.causa(), Some("la pelota quedó en el área derecha"));
    }

    #[test]
    fn a_ball_escaping_after_contact_is_intercepted_inside_the_radius() {
        let mut d = pases(3, opciones(true));
        let t = hasta_el_pase(&mut d);
        let en = |pelota: Vec2, vel: Vec2| cancha((A, 0.0), (B, 180.0), pelota, vel);
        // Antes del contacto, una pelota lenta que se acerca: B la espera de frente.
        let c = correr_p(&mut d, &en(B - Vec2::new(0.20, 0.0), Vec2::new(0.15, 0.0)), t, 1);
        assert_eq!(de(&c, 1), choice(1, SkillId::Mark, B));
        // Llega a B (a 0.07 m) y se escapa de costado a 0.13 m/s: va por ella.
        let escapa = B + Vec2::new(-0.07, -0.02);
        let c = correr_p(&mut d, &en(escapa, Vec2::new(0.0, -0.13)), t + 1, 1);
        assert_eq!(de(&c, 1), choice(1, SkillId::Intercept, escapa));
        // Ya lejos del robot, dentro del radio y lenta: la sigue.
        let mas_alla = B + Vec2::new(0.07, -0.12);
        let c = correr_p(&mut d, &en(mas_alla, Vec2::new(0.1, -0.1)), t + 2, 1);
        assert_eq!(de(&c, 1), choice(1, SkillId::Intercept, mas_alla));
        // Rápida, o fuera del radio: espera en su punto.
        let c = correr_p(&mut d, &en(mas_alla, Vec2::new(0.3, -0.4)), t + 3, 1);
        assert_eq!(de(&c, 1), choice(1, SkillId::Mark, B));
        let c = correr_p(&mut d, &en(B + Vec2::new(0.0, -0.30), Vec2::new(0.0, -0.13)), t + 4, 1);
        assert_eq!(de(&c, 1), choice(1, SkillId::Mark, B));
        assert_eq!(d.fase(), Fase::Ejecutando);
    }

    #[test]
    fn without_room_behind_the_ball_the_pass_aborts_at_once() {
        // A tiene que pasarle a B con la pelota en la esquina, o con el lugar detrás de la
        // pelota pegado al arco del área izquierda.
        for (pelota, causa) in [
            (Vec2::new(-0.64, 0.63), "no hay lugar detrás de la pelota para pasarla a B: está contra la pared"),
            (Vec2::new(-0.44, 0.0), "no hay lugar detrás de la pelota para pasarla a B: el lugar toca el área izquierda"),
        ] {
            let mut d = pases(3, opciones(true));
            let t = hasta_ejecutar(&mut d, &cancha((A, 0.0), (B, 180.0), MARCA, Vec2::ZERO));
            correr_p(&mut d, &cancha((A, 0.0), (B, 180.0), pelota, Vec2::ZERO), t + 1, 2);
            assert_eq!(d.causa(), Some(causa));
            assert_eq!(d.tipo_aborto(), Some(TipoAborto::SinLugar));
        }
        // Desde el punto de B hacia A, el lugar queda junto al arco del área derecha, pero el
        // robot cabe a `approach_pos_tol` de él: no aborta.
        let alinear = ApproachAlignedSkill::new(A);
        assert!(areas().iter().any(|a| a.touches(alinear.staging_point(B).unwrap())));
        assert_eq!(sin_lugar_detras(&alinear, B), None);
    }

    #[test]
    fn every_abort_cause_has_its_type() {
        let w = en_inicio();
        let lejos = mundo(&[(0, 0.3, 0.3, 0.0)]);
        let fin = |d: &RoutineDecider| (d.causa().map(str::to_string), d.tipo_aborto(), d.resumen().and_then(|r| r.tipo_aborto));
        let mut casos: Vec<(RoutineDecider, &str, TipoAborto)> = Vec::new();
        // Tiempos vencidos: un paso, la preparación, el `empujar` y la recuperación.
        let mut d = decisor(IR, opciones(true));
        let t0 = hasta_ejecutar(&mut d, &w);
        correr(&mut d, &w, t0 + 1, 510);
        casos.push((d, "el robot 0 no llegó al punto 1 en 8 s", TipoAborto::TiempoVencido));
        let mut d = decisor(IR, opciones(false));
        correr(&mut d, &lejos, 0, 1253);
        casos.push((d, "el robot 0 no llegó a su puesto en 20 s", TipoAborto::TiempoVencido));
        let mut d = pases(3, opciones(true));
        let t = hasta_ejecutar(&mut d, &cancha((A, 0.0), (B, 180.0), MARCA, Vec2::ZERO));
        correr_p(&mut d, &cancha((Vec2::new(-0.40, 0.30), 90.0), (B, 180.0), MARCA, Vec2::ZERO), t + 1, 510);
        casos.push((d, "el robot 0 no pasó la pelota en 8 s", TipoAborto::TiempoVencido));
        let mut d = pases(3, opciones(true));
        let t = recepcion_fallida(&mut d);
        correr_p(&mut d, &cancha((A, 0.0), (B, 180.0), CORTA, Vec2::ZERO), t, 510);
        casos.push((d, "el robot 1 no pudo recuperar la pelota en 8 s", TipoAborto::TiempoVencido));
        // Visión y casi choque.
        let mut d = decisor(IR, opciones(false));
        correr(&mut d, &w, 0, 1);
        d.decidir(1, &w, SystemTime::now() + Duration::from_secs(1));
        casos.push((d, "el robot 0 dejó de verse", TipoAborto::RobotSinVision));
        let mut d = decisor(IR, opciones(false));
        correr(&mut d, &mundo(&[(0, INICIO.x, INICIO.y, 0.0), (2, INICIO.x + 0.08, INICIO.y, 0.0)]), 0, 1);
        casos.push((d, "los robots 0 y 2 quedaron a 0.08 m", TipoAborto::Choque));
        // El usuario: abortar, y detener en bucle.
        let mut d = decisor(IR, opciones(true));
        hasta_ejecutar(&mut d, &w);
        d.ordenar(OrdenCorrida::Abortar);
        casos.push((d, CAUSA_USUARIO, TipoAborto::Usuario));
        let mut d = decisor(IR, OpcionesCorrida { repeticiones: Repeticiones::Bucle, ..opciones(true) });
        hasta_ejecutar(&mut d, &w);
        d.ordenar(OrdenCorrida::Abortar);
        casos.push((d, CAUSA_DETENIDA, TipoAborto::Usuario));
        // Las que llegan por `observar`.
        for (modo, estop, interv, tp, causa, tipo) in [
            (TickMode::Halt, false, &[][..], false, "HALT del árbitro", TipoAborto::Halt),
            (TickMode::Halt, true, &[], false, "parada de emergencia", TipoAborto::Parada),
            (TickMode::Stop, false, &[], false, "STOP del árbitro", TipoAborto::Stop),
            (TickMode::SinVision, false, &[], false, "corte sin visión de los robots propios", TipoAborto::CorteSinVision),
            (TickMode::Play, false, &[0], false, "el robot 0 pasó a control manual", TipoAborto::Manual),
            (TickMode::Play, false, &[], true, "teleport durante la ejecución", TipoAborto::Teleport),
        ] {
            let mut d = decisor(IR, opciones(true));
            let t0 = hasta_ejecutar(&mut d, &w);
            d.observar(&ctx(t0, modo, estop, interv, tp));
            casos.push((d, causa, tipo));
        }
        // Sin robots para asignar.
        let mut d = decisor(IR, opciones(false));
        correr(&mut d, &World::new(3, 3), 0, 1);
        casos.push((d, "no se pudo preparar: se ven 0 robots propios; la rutina pide 1 robot", TipoAborto::Asignacion));
        // Un paso de la fase 2 (la validación no deja cargarlo; se arma a mano).
        let mut r = (*prueba::rutina(&prueba::un_puesto(IR))).clone();
        r.puestos[0].pasos[0].accion = Accion::Sincronizar { barrera: "b".to_string() };
        let mut d = RoutineDecider::new(Arc::new(r), opciones(true), entorno(false));
        correr(&mut d, &w, 0, 200);
        casos.push((d, "el paso `sincronizar` no está disponible en esta fase", TipoAborto::PasoNoDisponible));
        // Pases: tope de reintentos, mitad, arco, área y sin lugar detrás.
        let mut d = pases(1, opciones(true));
        let t = hasta_el_pase(&mut d);
        correr_p(&mut d, &cancha((A, 0.0), (B, 180.0), CORTA, Vec2::ZERO), t, 3);
        casos.push((d, "el robot 1 no controló la pelota en 1 pase", TipoAborto::LimiteReintentos));
        for (pelota, causa, tipo) in [
            (-CORTA, "la pelota quedó en la mitad de A", TipoAborto::MitadRival),
            (Vec2::new(0.78, 0.05), "la pelota quedó en el arco derecho", TipoAborto::EnArco),
            (Vec2::new(0.65, 0.20), "la pelota quedó en el área derecha", TipoAborto::EnArea),
        ] {
            let mut d = pases(3, opciones(true));
            let t = hasta_el_pase(&mut d);
            correr_p(&mut d, &cancha((A, 0.0), (B, 180.0), pelota, Vec2::ZERO), t, 3);
            casos.push((d, causa, tipo));
        }
        let mut d = pases(3, opciones(true));
        let t = hasta_ejecutar(&mut d, &cancha((A, 0.0), (B, 180.0), MARCA, Vec2::ZERO));
        correr_p(&mut d, &cancha((A, 0.0), (B, 180.0), Vec2::new(-0.64, 0.63), Vec2::ZERO), t + 1, 2);
        casos.push((d, "no hay lugar detrás de la pelota para pasarla a B: está contra la pared", TipoAborto::SinLugar));

        for (d, causa, tipo) in &casos {
            assert_eq!(fin(d), (Some(causa.to_string()), Some(*tipo), Some(*tipo)), "{causa}");
        }
        let cubiertos: HashSet<TipoAborto> = casos.iter().map(|c| c.2).collect();
        let faltan: Vec<_> = TipoAborto::TODOS.iter().filter(|t| !cubiertos.contains(t)).collect();
        assert!(faltan.is_empty(), "tipos sin una causa probada: {faltan:?}");
        // Completada: sin causa ni tipo.
        let mut d = decisor(IR, opciones(true));
        let t0 = hasta_ejecutar(&mut d, &w);
        correr(&mut d, &mundo(&[(0, 0.3, -0.25, 0.0)]), t0 + 1, 2);
        assert_eq!(d.fase(), Fase::Terminada);
        assert_eq!(fin(&d), (None, None, None));
    }

    #[test]
    fn the_near_collision_check_waits_after_the_routine_teleport() {
        // Los robots quedaron a 0.09 m (como al terminar una corrida anterior); el teleport
        // de la rutina todavía no llegó: durante `quieto_s` no aborta.
        let juntos = cancha((A, 0.0), (A + Vec2::new(0.09, 0.0), 180.0), MARCA, Vec2::ZERO);
        let mut d = pases(3, OpcionesCorrida { teletransportar: true, ..opciones(false) });
        correr_p(&mut d, &juntos, 0, 30);
        assert!(d.fase().activa(), "{:?}", d.causa());
        assert!(!d.tomar_teleport().is_empty());
        // Si después de `quieto_s` siguen juntos, sí.
        correr_p(&mut d, &juntos, 30, 3);
        assert_eq!(d.causa(), Some("los robots 0 y 1 quedaron a 0.09 m"));
        // Sin teleport (el robot real), aborta desde el primer tick.
        let mut real = pases(3, OpcionesCorrida { simulacion: false, ..opciones(false) });
        correr_p(&mut real, &juntos, 0, 1);
        assert_eq!(real.causa(), Some("los robots 0 y 1 quedaron a 0.09 m"));
    }

    #[test]
    fn in_real_the_preparation_waits_for_the_ball_without_aborting() {
        let real = OpcionesCorrida { simulacion: false, ..opciones(false) };
        let mut d = pases(3, real);
        let lejos = cancha((A, 0.0), (B, 180.0), MARCA + Vec2::new(0.12, 0.0), Vec2::ZERO);
        // 30 s (más que los 20 s de la preparación al 100 %).
        correr_p(&mut d, &lejos, 0, 1875);
        assert_eq!(d.fase(), Fase::Preparando);
        assert_eq!(d.vista().frase, "Preparando: coloca la pelota en la marca: está a 0.12 m");
        assert!(d.tomar_teleport().is_empty(), "con el robot real no hay teleport");
        let en_marca = cancha((A, 0.0), (B, 180.0), MARCA, Vec2::ZERO);
        correr_p(&mut d, &en_marca, 1875, 40);
        assert_eq!(d.fase(), Fase::Lista, "{}", d.vista().frase);
    }

    #[test]
    fn in_simulation_the_ball_goes_to_its_mark_alone() {
        let mut d = pases(3, opciones(false));
        correr_p(&mut d, &cancha((A, 0.0), (B, 180.0), Vec2::ZERO, Vec2::ZERO), 0, 1);
        let items = d.tomar_teleport();
        assert_eq!(items.len(), 1, "solo la pelota, sin `teletransportar`: {items:?}");
        assert!(matches!(items[0], TeleportItem::Ball { x, y, vx, vy }
            if (x + 0.28).abs() < 1e-6 && y.abs() < 1e-6 && vx == 0.0 && vy == 0.0));
        // Con `teletransportar`, los robots y la pelota en la marca.
        let mut d = pases(3, OpcionesCorrida { teletransportar: true, ..opciones(false) });
        correr_p(&mut d, &cancha((A, 0.0), (B, 180.0), Vec2::ZERO, Vec2::ZERO), 0, 1);
        let items = d.tomar_teleport();
        assert!(matches!(items.last(), Some(TeleportItem::Ball { x, .. }) if (*x + 0.28).abs() < 1e-6), "{items:?}");
    }

    #[test]
    fn own_skill_checks_match_the_catalog() {
        let catalogo = crate::skills::SkillCatalog::new(3);
        let mut cat = catalogo;
        for (robot, pelota) in [
            ((Vec2::new(-0.42, 0.0), 0.0), MARCA),
            ((Vec2::new(-0.30, 0.20), 90.0), MARCA),
            ((Vec2::new(-0.24, 0.0), 0.0), Vec2::new(-0.12, 0.0)),
        ] {
            let w = cancha(robot, (B, 180.0), pelota, Vec2::ZERO);
            let r = w.get_robot_state(0, 0).unwrap().clone();
            let mut alinear = ApproachAlignedSkill::new(B);
            alinear.bidirectional = false;
            let st = cat.status(0, SkillId::ApproachAligned, B, &r, &w);
            assert_eq!(alinear.is_done(&r, &w), st.done, "ApproachAligned en {robot:?}");
            let empujar = ShootPushSkill::new(B);
            let st = cat.status(0, SkillId::ShootPush, B, &r, &w);
            assert_eq!(empujar.is_feasible(&r, pelota), st.feasible, "ShootPush en {robot:?}");
        }
    }
}
