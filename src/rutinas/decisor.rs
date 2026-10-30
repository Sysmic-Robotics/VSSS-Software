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
    Accion, AccionPelota, Duracion, Hacia, Paso, Pose, Repeticiones, Rutina, distancia_a_recorrido, distancia_a_tramo,
};
use super::pelota::{
    DESPEGUE_ALCANCE_M, FueraDeJuego, areas, despegue_legal, direccion_despegue, distancia_a_pared, dueno_de_zona,
    fuera_de_juego, mas_cercano, punto_resguardo, sin_lugar_detras,
};
use super::resumen::{Eventos, Pases, Resultado, ResumenCorrida, TipoAborto, Vuelta, VueltaPuesto};
use super::vista::{
    CAUSA_DETENIDA, CAUSA_USUARIO, Falta, Fase, RobotEnRutina, RutinaVista, frase_cierre, frase_ejecucion,
    frase_pausa, frase_pelota, frase_preparacion, segundos, verbo, vueltas,
};
use crate::coach::SkillChoice;
use crate::control_loop::{ContextoTick, PerfilVelocidad, TickDecider, TickMode, sin_vision};
use crate::motion::{CONTROL_DT, Motion, MotionConfig};
use crate::params::{RutinasParams, params};
use crate::radio::TeleportItem;
use crate::skills::zones::AREA_CLEARANCE;
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

/// Rapidez de la pelota: también su desplazamiento en esta ventana (s). Al cortar el empuje,
/// el tracker puede reiniciar la pelota con velocidad cero aunque siga rodando (punto de
/// control de la etapa 1): con la velocidad sola, parecía quieta fuera del radio.
const VENTANA_RAPIDEZ_S: f64 = 0.25;
/// Tope de la gracia del casi choque tras el teleport de la preparación (s): dura hasta ver a
/// los robots en su inicio, y a lo sumo esto.
const TELEPORT_MAX_S: f64 = 2.0;
/// Despegue que oscila (D15): `OSCILA_MAX` idas y vueltas entre alinearse y empujar dentro de
/// `OSCILA_VENTANA_S` lo dan por fallido. En el segundo punto de control, el robot alternaba
/// en cada tick mientras empujaba la pelota de a poco por la pared.
const OSCILA_MAX: usize = 4;
const OSCILA_VENTANA_S: f64 = 1.0;

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
        self.hecho = self.paso >= total;
    }

    /// Vuelve (o salta) al paso `paso`, como si empezara (resincronización, D4).
    fn volver_a(&mut self, paso: usize, t: f64) {
        self.paso = paso;
        self.t_paso = t;
        self.orientando = false;
        self.giro = 0.0;
        self.theta_prev = None;
        self.empuje_desde = None;
        self.control = 0;
        self.afuera = 0;
        self.hecho = false;
    }
}

/// Estado de la pelota en una rutina con pelota (D2).
#[derive(Debug, Clone)]
enum EstadoPelota {
    /// La secuencia manda: cada puesto corre sus pasos.
    EnJuego,
    /// Nadie la controla: el rescate (D3).
    Suelta(Rescate),
    /// Pegada a la pared: el despegue (D5).
    Despegue(Despegando),
    /// En un arco, en un área o pegada sin despegue: reposición en sim, pausa en real (D6).
    Fuera(FueraEstado),
}

#[derive(Debug, Clone)]
struct Rescate {
    /// La acción de la secuencia que quedó pendiente (la que falló o la que tocaba).
    pendiente: usize,
    /// Cuándo quedó suelta.
    desde: f64,
    /// El rescatador (puesto) y cuándo partió.
    rescatador: Option<(usize, f64)>,
    /// Puestos que no llegaron a tiempo (no se vuelven a elegir en este rescate).
    excluidos: Vec<usize>,
    /// El receptor del pase que falló: si rescata él, es una recuperación.
    receptor_que_fallo: Option<usize>,
    /// Ticks seguidos con la pelota controlada por el rescatador.
    control: u32,
}

#[derive(Debug, Clone)]
struct Despegando {
    puesto: usize,
    pendiente: usize,
    objetivo: Vec2,
    desde: f64,
    /// Cuándo el robot entró al empuje y volvió a alinearse (las del último segundo).
    vueltas: Vec<f64>,
}

#[derive(Debug, Clone)]
struct FueraEstado {
    pendiente: usize,
    motivo: FueraDeJuego,
    desde: f64,
    /// Dónde se repone en sim: la marca.
    punto: Vec2,
    /// Sim: cuándo se pidió el teleport.
    repuesta: Option<f64>,
    /// Real: ticks seguidos con la pelota quieta en una zona válida.
    quieta: usize,
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
    /// Patrones sin comandos (identidad de los robots): la asignación automática no los
    /// elige. Vale al preparar: si cambia en la corrida, la asignación no se rehace.
    sin_comandos: Vec<i32>,
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
    /// Con pelota (D2–D7): el estado, la secuencia de la vuelta (y la acción de cada
    /// `empujar` y de cada `recibir`), los pasos salteados por una resincronización (en
    /// esta vuelta y en la siguiente), lo resuelto, el último progreso y desde cuándo los
    /// pasos están congelados por el estado de la pelota.
    estado: EstadoPelota,
    acciones: Vec<AccionPelota>,
    accion_de_actor: HashMap<(usize, usize), usize>,
    accion_de_receptor: HashMap<(usize, usize), usize>,
    /// Los puntos de referencia de cada puesto con pelota (su inicio y sus `recibir`): su
    /// zona son los puntos más cerca de alguno de ellos (D15).
    referencias: Vec<(usize, Vec<Vec2>)>,
    salteados: HashSet<(usize, usize)>,
    salteados_siguiente: HashSet<(usize, usize)>,
    eventos: Eventos,
    t_progreso: f64,
    t_salida_juego: Option<f64>,
    /// Despegue: la distancia a la pared al empezar el anterior, y los fallidos seguidos.
    despegue_previo: Option<f32>,
    despegues_fallidos_seguidos: u32,
    /// Preparación con pelota: posiciones recientes de la pelota, y el tiempo que la
    /// preparación esperó solo a la pelota (no cuenta para el tiempo máximo).
    ventana_pelota: VecDeque<Vec2>,
    espera_pelota: f64,
    t_prev: Option<f64>,
    /// Cuándo pidió la rutina su teleport: durante `quieto_s`, y hasta ver a los robots en
    /// el destino del teleport (a lo sumo `TELEPORT_MAX_S`), el lazo todavía puede ver las
    /// poses de antes y el casi choque no cuenta.
    t_teleport: Option<f64>,
    destinos_teleport: Vec<(i32, Vec2)>,
    teleport_visto: bool,
    /// Posiciones recientes de la pelota en la ejecución (para su rapidez).
    historial_pelota: VecDeque<(f64, Vec2)>,
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

/// Punto lejano en la dirección `theta` desde `p`, para orientar con FacePoint.
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
        let acciones = rutina.secuencia_pelota().unwrap_or_default();
        let accion_de_actor = acciones.iter().enumerate().map(|(j, a)| (a.actor, j)).collect();
        let accion_de_receptor = acciones.iter().enumerate().filter_map(|(j, a)| a.receptor.map(|r| (r, j))).collect();
        let referencias = rutina
            .puestos
            .iter()
            .enumerate()
            .filter(|(_, p)| p.pasos.iter().any(|x| x.accion.con_pelota()))
            .map(|(k, p)| {
                let recibe = p.pasos.iter().filter_map(|x| match x.accion {
                    Accion::Recibir { punto } => Some(punto),
                    _ => None,
                });
                (k, std::iter::once(p.inicio.pos).chain(recibe).collect())
            })
            .collect();
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
            sin_comandos: Vec::new(),
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
            estado: EstadoPelota::EnJuego,
            acciones,
            accion_de_actor,
            accion_de_receptor,
            referencias,
            salteados: HashSet::new(),
            salteados_siguiente: HashSet::new(),
            eventos: Eventos::default(),
            t_progreso: 0.0,
            t_salida_juego: None,
            despegue_previo: None,
            despegues_fallidos_seguidos: 0,
            ventana_pelota: VecDeque::new(),
            espera_pelota: 0.0,
            t_prev: None,
            t_teleport: None,
            destinos_teleport: Vec::new(),
            teleport_visto: true,
            historial_pelota: VecDeque::new(),
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
                }
                for (t0, _) in self.pases_hechos.values_mut() {
                    *t0 += d;
                }
                self.correr_relojes_pelota(d);
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
        self.estado = EstadoPelota::EnJuego;
        self.t_progreso = self.t;
        self.historial_pelota.clear();
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
            eventos: self.rutina.pelota.map(|_| self.eventos.clone()),
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
        if !self.teleport_visto {
            let tol = self.p().tolerancia_m;
            self.teleport_visto = self.destinos_teleport.iter().all(|(id, p)| {
                world.get_robot_state(*id, self.ent.own_team).is_some_and(|r| (r.position - *p).length() <= tol)
            });
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
        self.asignados = asignar(&self.rutina, &vistos, self.ent.keeper_id, &self.op.eleccion, &self.sin_comandos)?;
        self.t_prep = Some(self.t);
        self.ventanas = vec![VecDeque::new(); self.asignados.len()];
        if self.op.teletransportar && self.op.simulacion {
            self.teleport = self.armar_teleport(world);
            self.destinos_teleport = self
                .asignados
                .iter()
                .enumerate()
                .map(|(k, id)| (*id, self.rutina.puestos[k].inicio.pos))
                .collect();
        } else if self.op.simulacion
            && let Some(m) = self.rutina.pelota
        {
            // Con pelota, en simulación la pelota va sola a su marca (D5).
            self.teleport = vec![pelota_en(m)];
        }
        if !self.teleport.is_empty() {
            self.t_teleport = Some(self.t);
            self.teleport_visto = self.destinos_teleport.is_empty();
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
        if let Some(t0) = self.t_teleport {
            let dt = self.t - t0;
            if dt < self.p().quieto_s || (dt < TELEPORT_MAX_S && !self.teleport_visto) {
                return None;
            }
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
        if self.rutina.pelota.is_some() {
            self.registrar_pelota(world);
            if let Some(causa) = self.vigilante() {
                self.abortar_por(causa);
                return Vec::new();
            }
            if !matches!(self.estado, EstadoPelota::EnJuego) {
                return self.supervisar(world);
            }
        }
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
                // Un paso sacó a la pelota de la secuencia: desde ahora manda su estado.
                if !matches!(self.estado, EstadoPelota::EnJuego) {
                    return self.supervisar(world);
                }
            }
            if self.en_curso.is_empty() || !self.en_curso.iter().all(|e| e.hecho) {
                self.resguardo_en_juego(world, &mut out);
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
            // Salteado por una resincronización: hecho, sin pase (D4).
            if self.salteados.contains(&(k, i)) {
                self.en_curso[k].siguiente_paso(self.t, total);
                continue;
            }
            match self.evaluar(k, i, paso, r, world)? {
                Evaluacion::Termina => {
                    if !paso.accion.con_pelota() {
                        self.progreso();
                    }
                    self.en_curso[k].siguiente_paso(self.t, total);
                }
                Evaluacion::Sigue(c) => {
                    let e = &self.en_curso[k];
                    let (dt, orientando) = (self.t - e.t_paso, e.orientando);
                    if let Some(max) = self.tiempo_max(paso)
                        && dt > max
                    {
                        if self.rutina.pelota.is_none() {
                            return Err(aborto(TipoAborto::TiempoVencido, frase_fuera_de_tiempo(r.id, i, paso, max, orientando)));
                        }
                        // Con pelota, el tiempo vencido replanifica (D7).
                        self.eventos.replanificaciones += 1;
                        if matches!(paso.accion, Accion::Empujar { .. }) {
                            let pendiente = self.accion_de_actor.get(&(k, i)).copied().unwrap_or(0);
                            self.soltar(pendiente, None);
                            return Ok(None);
                        }
                        self.en_curso[k].siguiente_paso(self.t, total);
                        continue;
                    }
                    return Ok(Some(c));
                }
            }
        }
        Ok(None)
    }

    fn evaluar(&mut self, k: usize, i: usize, paso: &Paso, r: &RobotState, world: &World) -> Result<Evaluacion, Aborto> {
        match &paso.accion {
            Accion::Empujar { .. } => return self.evaluar_empujar(k, i, r, world),
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
    /// empieza el despegue (D5).
    fn evaluar_empujar(&mut self, k: usize, i: usize, r: &RobotState, world: &World) -> Result<Evaluacion, Aborto> {
        let Some(&objetivo) = self.objetivos.get(&(k, i)) else {
            // `a_puesto` sin `recibir` emparejado: la validación no deja correr la rutina.
            return Ok(Evaluacion::Termina);
        };
        let pelota = world.get_ball_state().position;
        if self.en_curso[k].empuje_desde.is_none()
            && sin_lugar_detras(&ApproachAlignedSkill::new(objetivo), pelota).is_some()
        {
            let pendiente = self.accion_de_actor.get(&(k, i)).copied().unwrap_or(0);
            self.iniciar_despegue(k, pendiente, pelota, Some(objetivo));
            return Ok(Evaluacion::Sigue(choice(r.id, SkillId::Hold, Vec2::ZERO)));
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
    /// detiene fuera del radio o se vence el tiempo, contado desde el pase: la pelota queda
    /// suelta (el rescate) o fuera de juego (D3 y D6 de `rutinas-pelota-resiliente`).
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
        // Quieta, para la recepción fallida y para ir por ella, con la rapidez robusta; el
        // control (la pelota pegada al robot) con la velocidad del tick.
        let quieta = self.rapidez_pelota(world) < self.ent.pelota_lenta;
        let controla =
            (b.position - r.position).length() <= self.ent.alcance_m && b.velocity.length() < self.ent.pelota_lenta;
        let Some(&(t_pase, origen)) = self.pases_hechos.get(&(k, i)) else {
            return Ok(Evaluacion::Sigue(choice(r.id, SkillId::Mark, punto)));
        };
        let radio = self.p().recepcion_radio_m;
        let en_radio = (b.position - punto).length() <= radio;
        let e = &mut self.en_curso[k];
        e.control = if controla { e.control + 1 } else { 0 };
        e.afuera = if quieta && !en_radio { e.afuera + 1 } else { 0 };
        let afuera = e.afuera;
        if e.control >= TICKS_CONTROL {
            if let Some(&(emisor, _)) = self.emisor_de.get(&(k, i)) {
                self.pases_puesto[emisor].1 += 1;
            }
            self.pases_vuelta.1 += 1;
            self.despegue_previo = None;
            self.despegues_fallidos_seguidos = 0;
            self.progreso();
            return Ok(Evaluacion::Termina);
        }
        if self.t - t_pase > max || afuera >= TICKS_CONTROL {
            // Recepción fallida: el pase queda intentado y no logrado (D3).
            let pendiente = self.accion_de_receptor.get(&(k, i)).copied().unwrap_or(0);
            match fuera_de_juego(b.position) {
                Some(motivo) => self.sacar_de_juego(pendiente, motivo),
                None => self.soltar(pendiente, Some(k)),
            }
            return Ok(Evaluacion::Sigue(choice(r.id, SkillId::Hold, Vec2::ZERO)));
        }
        // Va por la pelota cuando se detuvo dentro del radio; si no, la espera de frente (no
        // sale a buscarla ni la persigue: la que se le escapa la toma el rescate, D8).
        if en_radio && quieta {
            Ok(Evaluacion::Sigue(choice(r.id, SkillId::Intercept, b.position)))
        } else {
            let espera = punto_de_espera(punto, origen, b.position, b.velocity, self.ent.pelota_lenta, radio);
            Ok(Evaluacion::Sigue(choice(r.id, SkillId::Mark, espera)))
        }
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
        self.salteados = std::mem::take(&mut self.salteados_siguiente);
        self.progreso();
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

    // ─── La pelota fuera de la secuencia (D2–D7) ─────────────────────────────

    fn progreso(&mut self) {
        self.t_progreso = self.t;
    }

    /// Guarda la posición de la pelota del tick, y descarta las de más de `VENTANA_RAPIDEZ_S`.
    fn registrar_pelota(&mut self, world: &World) {
        let p = world.get_ball_state().position;
        if p.is_finite() {
            self.historial_pelota.push_back((self.t, p));
        }
        while self.historial_pelota.front().is_some_and(|(t, _)| self.t - t > VENTANA_RAPIDEZ_S) {
            self.historial_pelota.pop_front();
        }
    }

    /// Rapidez de la pelota (m/s): la mayor entre su velocidad y su desplazamiento medio en
    /// la ventana. Un reinicio del tracker con velocidad cero no la hace parecer quieta.
    fn rapidez_pelota(&self, world: &World) -> f32 {
        let b = world.get_ball_state();
        let media = match self.historial_pelota.front() {
            Some(&(t0, p0)) if self.t - t0 >= 2.0 * CONTROL_DT && b.position.is_finite() => {
                (b.position - p0).length() / (self.t - t0) as f32
            }
            _ => 0.0,
        };
        b.velocity.length().max(media)
    }

    /// El vigilante: sin progreso en `sin_progreso_s`, aborta (D7). En la pausa real no
    /// corre (su tiempo se descuenta al reanudar).
    fn vigilante(&self) -> Option<Aborto> {
        if matches!(self.estado, EstadoPelota::Fuera(_)) && !self.op.simulacion {
            return None;
        }
        let max = self.p().sin_progreso_s;
        (self.t - self.t_progreso > max)
            .then(|| aborto(TipoAborto::SinProgreso, format!("sin progreso en {} s: {}", segundos(max), self.situacion())))
    }

    /// Lo último que estaba intentando la rutina, para la frase del vigilante.
    fn situacion(&self) -> String {
        let robot = |k: usize| self.asignados.get(k).copied().unwrap_or(-1);
        match &self.estado {
            EstadoPelota::Suelta(Rescate { rescatador: Some((k, _)), .. }) => {
                format!("el robot {} no logra controlar la pelota", robot(*k))
            }
            EstadoPelota::Suelta(_) => "la pelota sigue suelta".to_string(),
            EstadoPelota::Despegue(d) => format!("el robot {} no logra despegar la pelota", robot(d.puesto)),
            EstadoPelota::Fuera(f) => f.motivo.que_hacer(),
            EstadoPelota::EnJuego => self
                .en_curso
                .iter()
                .enumerate()
                .find(|(_, e)| !e.hecho)
                .map(|(k, e)| match self.rutina.puestos[k].pasos[e.paso].accion {
                    Accion::Empujar { .. } => format!("el robot {} no logra pasar la pelota", robot(k)),
                    Accion::Recibir { .. } => format!("el robot {} no recibe la pelota", robot(k)),
                    _ => format!("el robot {} no termina el paso {}", robot(k), e.paso + 1),
                })
                .unwrap_or_else(|| "la vuelta no se cierra".to_string()),
        }
    }

    /// Qué pasa con la pelota, para la frase de la vista.
    fn que_hace_la_pelota(&self) -> String {
        let robot = |k: usize| self.asignados.get(k).copied().unwrap_or(-1);
        match &self.estado {
            EstadoPelota::Suelta(Rescate { rescatador: Some((k, _)), .. }) => {
                format!("el robot {} va por la pelota suelta", robot(*k))
            }
            EstadoPelota::Suelta(_) => "la pelota está suelta".to_string(),
            EstadoPelota::Despegue(d) => format!("el robot {} despega la pelota de la pared", robot(d.puesto)),
            EstadoPelota::Fuera(_) if self.op.simulacion => "repone la pelota en la marca".to_string(),
            EstadoPelota::Fuera(f) => format!("en pausa: {}", f.motivo.que_hacer()),
            EstadoPelota::EnJuego => String::new(),
        }
    }

    /// Los pasos quedan congelados desde ahora (la primera vez).
    fn salir_del_juego(&mut self) {
        if self.t_salida_juego.is_none() {
            self.t_salida_juego = Some(self.t);
        }
    }

    /// Vuelve a la secuencia, corriendo los relojes de los pasos lo que estuvieron
    /// congelados.
    fn volver_al_juego(&mut self) {
        if let Some(t0) = self.t_salida_juego.take() {
            let d = (self.t - t0).max(0.0);
            for e in &mut self.en_curso {
                e.t_paso += d;
            }
            for (t, _) in self.pases_hechos.values_mut() {
                *t += d;
            }
        }
        self.estado = EstadoPelota::EnJuego;
    }

    /// La pausa del usuario corre también los relojes de la pelota.
    fn correr_relojes_pelota(&mut self, d: f64) {
        self.t_progreso += d;
        if let Some(t0) = &mut self.t_salida_juego {
            *t0 += d;
        }
        match &mut self.estado {
            EstadoPelota::Suelta(r) => {
                r.desde += d;
                if let Some((_, t)) = &mut r.rescatador {
                    *t += d;
                }
            }
            EstadoPelota::Despegue(x) => x.desde += d,
            EstadoPelota::Fuera(f) => {
                f.desde += d;
                if let Some(t) = &mut f.repuesta {
                    *t += d;
                }
            }
            EstadoPelota::EnJuego => {}
        }
    }

    /// La pelota queda suelta: el rescate (D3).
    fn soltar(&mut self, pendiente: usize, receptor_que_fallo: Option<usize>) {
        self.salir_del_juego();
        self.estado = EstadoPelota::Suelta(Rescate {
            pendiente,
            desde: self.t,
            rescatador: None,
            excluidos: Vec::new(),
            receptor_que_fallo,
            control: 0,
        });
    }

    /// La pelota fuera de juego: reposición en sim, pausa en real (D6).
    fn sacar_de_juego(&mut self, pendiente: usize, motivo: FueraDeJuego) {
        self.salir_del_juego();
        if !self.op.simulacion {
            self.eventos.pausas += 1;
        }
        self.estado = EstadoPelota::Fuera(FueraEstado {
            pendiente,
            motivo,
            desde: self.t,
            punto: self.rutina.pelota.unwrap_or(Vec2::ZERO),
            repuesta: None,
            quieta: 0,
        });
    }

    /// Empieza un despegue del puesto `k` (D5), o saca la pelota de juego si no hay
    /// dirección legal o si los dos despegues anteriores no la alejaron de la pared.
    fn iniciar_despegue(&mut self, k: usize, pendiente: usize, pelota: Vec2, hacia: Option<Vec2>) {
        let pared = distancia_a_pared(pelota);
        if let Some(antes) = self.despegue_previo {
            if pared < antes + 0.05 {
                self.eventos.despegues_fallidos += 1;
                self.despegues_fallidos_seguidos += 1;
            } else {
                self.despegues_fallidos_seguidos = 0;
            }
        }
        if self.despegues_fallidos_seguidos >= 2 {
            self.despegue_previo = None;
            self.despegues_fallidos_seguidos = 0;
            return self.sacar_de_juego(pendiente, FueraDeJuego::Pegada);
        }
        match direccion_despegue(pelota, hacia, self.p().despegue_max_deg, |d| despegue_legal(pelota, d)) {
            Some(d) => {
                self.salir_del_juego();
                self.despegue_previo = Some(pared);
                self.en_curso[k].empuje_desde = None;
                self.estado = EstadoPelota::Despegue(Despegando {
                    puesto: k,
                    pendiente,
                    objetivo: pelota + d * DESPEGUE_ALCANCE_M,
                    desde: self.t,
                    vueltas: Vec::new(),
                });
            }
            None => self.sacar_de_juego(pendiente, FueraDeJuego::Pegada),
        }
    }

    /// Mark del puesto `k` lejos de la pelota (`centro`): en su referencia (el destino del
    /// paso actual) o a `resguardo_m` de ella.
    fn apartar(&self, k: usize, r: &RobotState, centro: Vec2) -> SkillChoice {
        let referencia = self
            .en_curso
            .get(k)
            .filter(|e| !e.hecho)
            .and_then(|e| self.rutina.puestos[k].pasos.get(e.paso))
            .and_then(|p| p.accion.destino())
            .unwrap_or(r.position);
        let q = punto_resguardo(referencia, r.position, centro, self.p().resguardo_m, self.p().zona_segura_m);
        choice(r.id, SkillId::Mark, q)
    }

    /// Resguardo en juego (D15): mientras un puesto está en su `empujar`, los demás cuyo
    /// destino (un `ir`, o la espera de un `recibir` antes del pase) queda a menos de
    /// `resguardo_m` de la pelota esperan en el punto de resguardo.
    fn resguardo_en_juego(&self, world: &World, out: &mut [SkillChoice]) {
        if self.rutina.pelota.is_none() {
            return;
        }
        let en_empujar = |k: usize| {
            let e = &self.en_curso[k];
            !e.hecho && matches!(self.rutina.puestos[k].pasos[e.paso].accion, Accion::Empujar { .. })
        };
        let Some(pasador) = (0..self.en_curso.len()).find(|&k| en_empujar(k)) else { return };
        let pelota = world.get_ball_state().position;
        let resguardo = self.p().resguardo_m;
        for c in out.iter_mut() {
            let Some(k) = self.asignados.iter().position(|id| *id == c.robot_id) else { continue };
            let e = &self.en_curso[k];
            if k == pasador || e.hecho || !matches!(c.skill_id, SkillId::GoTo | SkillId::Mark) {
                continue;
            }
            let espera = match self.rutina.puestos[k].pasos[e.paso].accion {
                Accion::Ir { .. } => true,
                Accion::Recibir { .. } => !self.pases_hechos.contains_key(&(k, e.paso)),
                _ => false,
            };
            if espera
                && (c.target - pelota).length() < resguardo
                && let Some(r) = world.get_robot_state(c.robot_id, self.ent.own_team)
            {
                *c = self.apartar(k, r, pelota);
            }
        }
    }

    /// Con la pelota fuera de la secuencia, las elecciones de todos y las transiciones (D2).
    fn supervisar(&mut self, world: &World) -> Vec<SkillChoice> {
        let b = world.get_ball_state().clone();
        let robots: Vec<(usize, RobotState)> = (0..self.asignados.len())
            .filter_map(|k| {
                world.get_robot_state(self.asignados[k], self.ent.own_team).filter(|r| r.active).map(|r| (k, r.clone()))
            })
            .collect();
        // Suelta o despegando, la pelota puede irse al arco o al área, o dejar de verse.
        let pendiente = match &self.estado {
            EstadoPelota::Suelta(r) => Some(r.pendiente),
            EstadoPelota::Despegue(d) => Some(d.pendiente),
            _ => None,
        };
        if let Some(pendiente) = pendiente {
            if !b.position.is_finite() {
                if !self.op.simulacion {
                    self.sacar_de_juego(pendiente, FueraDeJuego::NoSeVe);
                }
            } else if let Some(motivo) = fuera_de_juego(b.position) {
                self.sacar_de_juego(pendiente, motivo);
            }
        }
        let vel = self.rapidez_pelota(world);
        let mut out = Vec::new();
        match self.estado.clone() {
            EstadoPelota::EnJuego => {}
            EstadoPelota::Suelta(mut res) => {
                if let Some((kr, t0)) = res.rescatador
                    && self.t - t0 > self.p().tiempo_max_s / self.escala
                {
                    // No llegó a tiempo: otro (D3).
                    res.excluidos.push(kr);
                    res.rescatador = None;
                    res.control = 0;
                    self.eventos.replanificaciones += 1;
                }
                let elegido = match res.rescatador {
                    Some((k, _)) => Some(k),
                    None => {
                        let mut cands: Vec<(usize, Vec2)> =
                            robots.iter().filter(|(k, _)| !res.excluidos.contains(k)).map(|(k, r)| (*k, r.position)).collect();
                        if cands.is_empty() {
                            res.excluidos.clear();
                            cands = robots.iter().map(|(k, r)| (*k, r.position)).collect();
                        }
                        // El dueño de la zona; si no puede (quedó excluido), el más cercano (D15).
                        let dueno = dueno_de_zona(b.position, &self.referencias)
                            .filter(|k| cands.iter().any(|(j, _)| j == k));
                        let elegido = dueno.or_else(|| mas_cercano(b.position, &cands));
                        let parte = vel < self.p().rescate_vel_m_s || self.t - res.desde >= self.p().rescate_espera_s;
                        if parte {
                            res.rescatador = elegido.map(|k| (k, self.t));
                        }
                        elegido
                    }
                };
                for (k, r) in &robots {
                    out.push(match (Some(*k) == elegido, res.rescatador.is_some()) {
                        (true, true) => choice(r.id, SkillId::Intercept, b.position),
                        (true, false) => choice(r.id, SkillId::Mark, r.position),
                        (false, _) => self.apartar(*k, r, b.position),
                    });
                }
                if let Some((kr, _)) = res.rescatador {
                    let controla = robots.iter().find(|(k, _)| *k == kr).is_some_and(|(_, r)| {
                        (b.position - r.position).length() <= self.ent.alcance_m
                            && b.velocity.length() < self.ent.pelota_lenta
                    });
                    res.control = if controla { res.control + 1 } else { 0 };
                    if res.control >= TICKS_CONTROL {
                        self.resincronizar(kr, res.pendiente, res.receptor_que_fallo);
                        return out;
                    }
                }
                self.estado = EstadoPelota::Suelta(res);
            }
            EstadoPelota::Despegue(mut d) => {
                // Vence, u oscila entre alinearse y empujar (D15): las idas y vueltas del último
                // segundo, aunque la pelota avance de a poco por la pared.
                d.vueltas.retain(|t| self.t - t <= OSCILA_VENTANA_S);
                let oscila = d.vueltas.len() >= OSCILA_MAX;
                if self.t - d.desde > self.p().tiempo_max_s / self.escala || oscila {
                    self.eventos.despegues_fallidos += 1;
                    self.en_curso[d.puesto].empuje_desde = None;
                    self.sacar_de_juego(d.pendiente, FueraDeJuego::Pegada);
                }
                for (k, r) in &robots {
                    if *k != d.puesto {
                        out.push(self.apartar(*k, r, b.position));
                    } else if !matches!(self.estado, EstadoPelota::Despegue(_)) {
                        out.push(choice(r.id, SkillId::Hold, Vec2::ZERO));
                    } else {
                        let empujaba = self.en_curso[*k].empuje_desde.is_some();
                        match self.empujar_hacia(*k, d.objetivo, r, world) {
                            Some(c) => {
                                if empujaba && self.en_curso[*k].empuje_desde.is_none() {
                                    d.vueltas.push(self.t);
                                }
                                if let EstadoPelota::Despegue(x) = &mut self.estado {
                                    x.vueltas = d.vueltas.clone();
                                }
                                out.push(c);
                            }
                            None => {
                                self.eventos.despegues += 1;
                                self.en_curso[*k].empuje_desde = None;
                                self.soltar(d.pendiente, None);
                                out.push(choice(r.id, SkillId::Mark, r.position));
                            }
                        }
                    }
                }
            }
            EstadoPelota::Fuera(mut f) if self.op.simulacion => {
                match f.repuesta {
                    None => {
                        // Primero se apartan los que están sobre la marca; después, el teleport.
                        let cerca = robots.iter().any(|(_, r)| (r.position - f.punto).length() < self.p().resguardo_m);
                        if !cerca || self.t - f.desde > 2.0 {
                            self.teleport.push(pelota_en(f.punto));
                            self.t_teleport = Some(self.t);
                            self.destinos_teleport.clear();
                            self.teleport_visto = true;
                            f.repuesta = Some(self.t);
                            self.eventos.reposiciones += 1;
                        }
                    }
                    Some(t0) => {
                        let llego = (b.position - f.punto).length() <= self.p().tolerancia_m;
                        if (llego && vel < self.ent.pelota_lenta) || self.t - t0 > 2.0 {
                            for (k, r) in &robots {
                                out.push(self.apartar(*k, r, f.punto));
                            }
                            self.soltar(f.pendiente, None);
                            return out;
                        }
                    }
                }
                for (k, r) in &robots {
                    out.push(self.apartar(*k, r, f.punto));
                }
                self.estado = EstadoPelota::Fuera(f);
            }
            EstadoPelota::Fuera(mut f) => {
                // Pausa real: todos quietos hasta que la persona deja la pelota quieta en una
                // zona válida (D6).
                let p = b.position;
                let valida = p.is_finite()
                    && fuera_de_juego(p).is_none()
                    && !areas().iter().any(|a| a.touches_with(p, AREA_CLEARANCE))
                    && self.hay_lugar(p, f.pendiente);
                f.quieta = if valida && vel < self.ent.pelota_lenta { f.quieta + 1 } else { 0 };
                for (_, r) in &robots {
                    out.push(choice(r.id, SkillId::Hold, Vec2::ZERO));
                }
                if f.quieta >= self.largo_ventana() {
                    let d = (self.t - f.desde).max(0.0);
                    self.eventos.pausa_s += d;
                    self.t_progreso += d;
                    self.soltar(f.pendiente, None);
                } else {
                    self.estado = EstadoPelota::Fuera(f);
                }
            }
        }
        out
    }

    /// Si hay lugar detrás de la pelota en `p` para la acción `pendiente`, o un despegue.
    fn hay_lugar(&self, p: Vec2, pendiente: usize) -> bool {
        let objetivo = self.acciones.get(pendiente).and_then(|a| self.objetivos.get(&a.actor)).copied();
        objetivo.is_some_and(|o| sin_lugar_detras(&ApproachAlignedSkill::new(o), p).is_none())
            || direccion_despegue(p, objetivo, self.p().despegue_max_deg, |d| despegue_legal(p, d)).is_some()
    }

    /// El puesto `z` controló la pelota: la secuencia sigue desde su próxima acción (D4).
    /// Las acciones entre la pendiente y esa se saltean (su paso con pelota y el `recibir`
    /// emparejado quedan hechos, sin pase); si es la misma y `z` la hizo, la repite.
    fn resincronizar(&mut self, z: usize, pendiente: usize, receptor_que_fallo: Option<usize>) {
        if receptor_que_fallo == Some(z) {
            self.recuperaciones += 1;
        } else {
            self.eventos.cambios_de_rol += 1;
        }
        let n = self.acciones.len();
        let Some(j) = (pendiente..pendiente + n).find(|&j| self.acciones[j % n].actor.0 == z) else {
            self.volver_al_juego();
            return;
        };
        for idx in pendiente..j {
            let a = self.acciones[idx % n];
            let destino = if idx < n { &mut self.salteados } else { &mut self.salteados_siguiente };
            destino.insert(a.actor);
            destino.extend(a.receptor);
        }
        if j < n {
            let a = self.acciones[j];
            let e = &mut self.en_curso[z];
            if e.hecho || e.paso > a.actor.1 {
                e.volver_a(a.actor.1, self.t);
            }
            if let Some(r) = a.receptor {
                self.pases_hechos.remove(&r);
                let er = &mut self.en_curso[r.0];
                if !er.hecho && er.paso == r.1 {
                    er.volver_a(r.1, self.t);
                }
            }
        }
        self.volver_al_juego();
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
            Fase::Ejecutando if !matches!(self.estado, EstadoPelota::EnJuego) => {
                frase_pelota(self.repeticion, self.op.repeticiones, &self.que_hace_la_pelota())
            }
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

    fn usar_sin_comandos(&mut self, patrones: &[i32]) {
        self.sin_comandos = patrones.to_vec();
    }

    /// El robot asignado al puesto `arquero` (D4 de `arquero`: por la asignación, el
    /// `keeper_id` del entorno). Sin ese puesto, o antes de asignar, no opina.
    fn arquero(&self) -> Option<i32> {
        self.rutina.puestos.iter().zip(&self.asignados).find(|(p, _)| p.arquero).map(|(_, id)| *id)
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

    /// El reloj de los tests: el último dato de visión del mundo (sus robots se actualizan
    /// al armarlo). Con el reloj de pared, un mundo reusado después de una tanda larga de
    /// ticks envejecía más que `blind_stop_s` con la máquina cargada, y la rutina abortaba
    /// por "dejó de verse".
    fn ahora_de(w: &World) -> SystemTime {
        w.get_blue_team_state()
            .into_iter()
            .chain(w.get_yellow_team_state())
            .map(|r| r.last_update)
            .max()
            .unwrap_or_else(SystemTime::now)
    }

    /// `n` ticks desde `t0` con el mundo `w`; las elecciones del último.
    fn correr(d: &mut RoutineDecider, w: &World, t0: u32, n: u32) -> Vec<SkillChoice> {
        let ahora = ahora_de(w);
        let mut out = Vec::new();
        for t in t0..t0 + n {
            out = d.decidir(t, w, ahora);
        }
        out
    }

    /// Corre con el robot quieto en el inicio hasta que arranca (inicio automático) y
    /// devuelve el tick del arranque.
    fn hasta_ejecutar(d: &mut RoutineDecider, w: &World) -> u32 {
        let ahora = ahora_de(w);
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
    fn preparar_usa_la_lista_sin_comandos() {
        // Dos robots; el más cercano al inicio (el 0) no recibe comandos: va el 1.
        let w = mundo(&[(0, INICIO.x, INICIO.y, 0.0), (1, 0.3, 0.3, 0.0)]);
        let mut sin = decisor(IR, opciones(false));
        correr(&mut sin, &w, 0, 3);
        assert_eq!(sin.asignados(), &[0]);
        let mut con = decisor(IR, opciones(false));
        con.usar_sin_comandos(&[0]);
        correr(&mut con, &w, 0, 3);
        assert_eq!(con.asignados(), &[1]);
    }

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
    fn a_reused_world_does_not_age_with_the_wall_clock() {
        // Regresión: con la máquina cargada, una tanda larga de ticks tardaba más que
        // `blind_stop_s` (0.4 s), y el mismo mundo, reusado, parecía sin visión.
        let w = en_inicio();
        let mut d = decisor(IR, opciones(true));
        let t0 = hasta_ejecutar(&mut d, &w);
        std::thread::sleep(Duration::from_millis(500));
        correr(&mut d, &w, t0 + 1, 5);
        assert_eq!(d.causa(), None);
        assert_eq!(d.fase(), Fase::Ejecutando);
    }

    #[test]
    fn a_robot_out_of_sight_aborts() {
        let mut d = decisor(IR, opciones(false));
        let w = en_inicio();
        correr(&mut d, &w, 0, 1);
        d.decidir(1, &w, ahora_de(&w) + Duration::from_secs(1));
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

    fn pases(op: OpcionesCorrida) -> RoutineDecider {
        RoutineDecider::new(prueba::rutina(&prueba::pases()), op, entorno(false))
    }

    fn real() -> OpcionesCorrida {
        OpcionesCorrida { simulacion: false, ..opciones(true) }
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
        let ahora = ahora_de(w);
        let mut out = Vec::new();
        for t in t0..t0 + n {
            out = d.decidir(t, w, ahora);
            solo_permitidas(&out);
        }
        out
    }

    fn pasa_a(c: SkillChoice, objetivo: Vec2) -> bool {
        matches!(c.skill_id, SkillId::ApproachAligned | SkillId::ShootPush) && (c.target - objetivo).length() < 1e-5
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
        let mut d = pases(opciones(true));
        let t = hasta_el_pase(&mut d);
        // La pelota llega a B y queda quieta a 0.05 m: tres ticks de control.
        let llega = cancha((A, 0.0), (B, 180.0), B - Vec2::new(0.05, 0.0), Vec2::ZERO);
        correr_p(&mut d, &llega, t, 2);
        assert_eq!(d.pases_vuelta, (1, 0), "dos ticks no alcanzan");
        let c = correr_p(&mut d, &llega, t + 2, 1);
        // Recibió: B sigue con su `empujar` hacia el punto de A.
        assert!(pasa_a(de(&c, 1), A), "{:?}", de(&c, 1));
        assert_eq!((d.pases_puesto[0], d.pases_vuelta), ((1, 1), (1, 1)));
        // A, ya en su punto, espera la pelota.
        assert_eq!(de(&c, 0), choice(0, SkillId::Mark, A));
        assert_eq!(d.vista().frase, "Vuelta 1 de 1, paso 3 de 3: el robot 0 espera la pelota");
    }

    /// Fuera del radio de recepción de B (0.30 m de su punto), más cerca de B que de A.
    const CORTA: Vec2 = Vec2::new(0.10, 0.0);

    /// Después del pase, la pelota se detiene en `pelota`: tres ticks quieta fuera del radio
    /// y la recepción falla; la pelota queda suelta. Devuelve el tick siguiente y las
    /// elecciones del tick de la falla.
    fn recepcion_fallida(d: &mut RoutineDecider, pelota: Vec2) -> (u32, Vec<SkillChoice>) {
        let t = hasta_el_pase(d);
        let quieta = cancha((A, 0.0), (B, 180.0), pelota, Vec2::ZERO);
        let c = correr_p(d, &quieta, t, 2);
        assert_eq!(de(&c, 1), choice(1, SkillId::Mark, B), "fuera del radio no va por ella");
        // Falla cuando la pelota lleva la ventana de la rapidez quieta y 3 ticks más.
        for i in 2..40 {
            let c = correr_p(d, &quieta, t + i, 1);
            if matches!(d.estado, EstadoPelota::Suelta(_)) {
                assert_eq!(d.pases_puesto[0], (1, 0), "el pase cuenta como intentado y no logrado");
                return (t + i + 1, c);
            }
        }
        panic!("la recepción no falló: {:?}", d.estado);
    }

    #[test]
    fn a_failed_reception_is_rescued_by_the_receiver() {
        let mut d = pases(opciones(true));
        let (t, c) = recepcion_fallida(&mut d, CORTA);
        // B, el más cercano, va por la pelota (está quieta: parte enseguida); A sigue en su punto.
        assert_eq!(de(&c, 1), choice(1, SkillId::Intercept, CORTA));
        assert_eq!(de(&c, 0), choice(0, SkillId::Mark, A));
        assert_eq!(d.vista().frase, "Vuelta 1 de 1: el robot 1 va por la pelota suelta");
        // B la controla desde el costado: tres ticks, y la rutina se resincroniza.
        let b_en_pelota = cancha((A, 0.0), (CORTA + Vec2::new(0.0, 0.06), 270.0), CORTA, Vec2::ZERO);
        let c = correr_p(&mut d, &b_en_pelota, t, 2);
        assert_eq!(de(&c, 1), choice(1, SkillId::Intercept, CORTA));
        let c = correr_p(&mut d, &b_en_pelota, t + 2, 2);
        assert!(matches!(d.estado, EstadoPelota::EnJuego));
        assert_eq!((d.recuperaciones, d.eventos.cambios_de_rol), (1, 0));
        // B pasa desde donde controló la pelota; A la espera.
        assert!(pasa_a(de(&c, 1), A), "{:?}", de(&c, 1));
        assert_eq!(de(&c, 0), choice(0, SkillId::Mark, A));
        d.abortar(TipoAborto::Usuario, "fin de la prueba");
        let r = d.resumen().unwrap();
        let p = r.pases.clone().expect("con pelota");
        assert_eq!((p.intentados, p.logrados, p.recuperaciones), (1, 0, 1));
        assert_eq!(r.eventos.clone().expect("con pelota").cambios_de_rol, 0);
    }

    #[test]
    fn a_ball_left_in_the_passers_half_is_rescued_and_passed_again() {
        let mut d = pases(opciones(true));
        // Se detiene en la mitad de A, más cerca de A (0.30 m) que de B (0.50 m).
        let (t, c) = recepcion_fallida(&mut d, -CORTA);
        assert_eq!(de(&c, 0), choice(0, SkillId::Intercept, -CORTA), "va A, no el dueño del paso");
        assert_eq!(de(&c, 1), choice(1, SkillId::Mark, B), "B espera en su punto, a más de 0.30 m");
        // A la controla: repite el pase a B, que vuelve a esperarlo.
        let a_en_pelota = cancha((-CORTA - Vec2::new(0.06, 0.0), 0.0), (B, 180.0), -CORTA, Vec2::ZERO);
        let c = correr_p(&mut d, &a_en_pelota, t, 4);
        assert_eq!((d.recuperaciones, d.eventos.cambios_de_rol), (0, 1));
        assert!(pasa_a(de(&c, 0), B), "{:?}", de(&c, 0));
        assert_eq!(de(&c, 1), choice(1, SkillId::Mark, B));
        assert!(!d.pases_hechos.contains_key(&(1, 0)), "el `recibir` de B se reinició");
        assert_eq!(d.fase(), Fase::Ejecutando);
    }

    #[test]
    fn the_rescuer_waits_for_a_fast_ball_until_it_slows_down_or_for_a_while() {
        // A 0.3 m/s espera donde está; a los 2 s parte aunque siga rodando.
        let mut d = pases(opciones(true));
        let t = hasta_el_pase(&mut d);
        d.soltar(0, Some(1));
        let pelota = Vec2::new(0.20, 0.0);
        let rapida = cancha((A, 0.0), (B, 180.0), pelota, Vec2::new(0.0, 0.3));
        let c = correr_p(&mut d, &rapida, t, 120);
        assert_eq!(de(&c, 1), choice(1, SkillId::Mark, B), "B, el más cercano, espera donde está");
        let c = correr_p(&mut d, &rapida, t + 120, 10);
        assert_eq!(de(&c, 1), choice(1, SkillId::Intercept, pelota), "a los 2 s parte");
        // Más lenta que 0.15 m/s: parte enseguida.
        let mut d = pases(opciones(true));
        let t = hasta_el_pase(&mut d);
        d.soltar(0, Some(1));
        let lenta = cancha((A, 0.0), (B, 180.0), pelota, Vec2::new(0.0, 0.1));
        let c = correr_p(&mut d, &lenta, t, 20);
        assert_eq!(de(&c, 1), choice(1, SkillId::Intercept, pelota), "pasada la ventana del salto de la pelota");
    }

    #[test]
    fn the_zone_owner_rescues_even_if_another_is_closer() {
        // La pelota repuesta en la marca, en la mitad de A, con B más cerca: va A (D15).
        let mut d = pases(opciones(true));
        let t = hasta_el_pase(&mut d);
        d.soltar(0, None);
        let w = cancha((Vec2::new(-0.40, 0.40), 0.0), (Vec2::new(-0.20, 0.12), 180.0), MARCA, Vec2::ZERO);
        let c = correr_p(&mut d, &w, t, 20);
        assert_eq!(de(&c, 0), choice(0, SkillId::Intercept, MARCA));
        assert_eq!(de(&c, 1), choice(1, SkillId::Mark, B), "B espera en su punto");
    }

    #[test]
    fn while_passing_the_others_keep_their_distance() {
        // B pasa desde la mitad de A hacia el punto de A, a 0.12 m de la pelota; A vuelve con
        // su `ir` a ese punto: espera a 0.30 m de la pelota hasta que B suelta el pase (D15).
        let mut d = pases(opciones(true));
        let t = hasta_ejecutar(&mut d, &cancha((A, 0.0), (B, 180.0), MARCA, Vec2::ZERO));
        d.en_curso[0].volver_a(1, d.t);
        d.en_curso[1].volver_a(1, d.t);
        let pelota = Vec2::new(-0.30, 0.05);
        let w = cancha((Vec2::new(0.0, 0.0), 0.0), (Vec2::new(-0.16, 0.05), 180.0), pelota, Vec2::ZERO);
        let c = correr_p(&mut d, &w, t + 1, 1);
        assert!(pasa_a(de(&c, 1), A), "{:?}", de(&c, 1));
        let a = de(&c, 0);
        assert_eq!(a.skill_id, SkillId::Mark, "en lugar de GoTo a su punto: {a:?}");
        assert!((a.target - pelota).length() >= 0.30 - 1e-4, "{a:?}");
        // Sin nadie empujando, A va a su punto.
        d.en_curso[1].volver_a(2, d.t);
        let c = correr_p(&mut d, &w, t + 2, 1);
        assert_eq!(de(&c, 0), choice(0, SkillId::GoTo, A));
    }

    #[test]
    fn a_rescuer_that_does_not_arrive_is_replaced() {
        let mut d = pases(opciones(true));
        let t = hasta_el_pase(&mut d);
        d.soltar(0, Some(1));
        let pelota = Vec2::new(0.20, 0.0);
        let w = cancha((A, 0.0), (B, 180.0), pelota, Vec2::ZERO);
        let c = correr_p(&mut d, &w, t, 500);
        assert_eq!(de(&c, 1), choice(1, SkillId::Intercept, pelota), "B va, y no llega");
        let c = correr_p(&mut d, &w, t + 500, 30);
        assert_eq!(de(&c, 0), choice(0, SkillId::Intercept, pelota), "a los 8 s va A, el más cercano");
        assert_eq!(d.eventos.replanificaciones, 1);
        // B, el dueño de la zona, se aparta: su punto está a 0.20 m de la pelota.
        let b = de(&c, 1);
        assert_eq!(b.skill_id, SkillId::Mark);
        assert!((b.target - pelota).length() >= 0.30 - 1e-4, "{b:?}");
        assert_eq!(d.fase(), Fase::Ejecutando);
    }

    #[test]
    fn a_push_out_of_time_replans_instead_of_aborting() {
        let mut d = pases(opciones(true));
        let t = hasta_ejecutar(&mut d, &cancha((A, 0.0), (B, 180.0), MARCA, Vec2::ZERO));
        // A queda lejos y nunca se alinea: a los 8 s la pelota queda suelta, sin abortar.
        let lejos = cancha((Vec2::new(-0.40, 0.30), 90.0), (B, 180.0), MARCA, Vec2::ZERO);
        correr_p(&mut d, &lejos, t + 1, 499);
        assert!(matches!(d.estado, EstadoPelota::EnJuego));
        let c = correr_p(&mut d, &lejos, t + 500, 3);
        assert!(matches!(d.estado, EstadoPelota::Suelta(_)), "{:?}", d.estado);
        assert_eq!(d.eventos.replanificaciones, 1);
        assert_eq!(de(&c, 0), choice(0, SkillId::Intercept, MARCA), "A sigue siendo el más cercano");
        assert_eq!(d.fase(), Fase::Ejecutando);
    }

    #[test]
    fn a_move_out_of_time_is_skipped_in_a_ball_routine() {
        let mut d = pases(opciones(true));
        let t = hasta_ejecutar(&mut d, &cancha((A, 0.0), (B, 180.0), MARCA, Vec2::ZERO));
        // A, en su `ir` de vuelta (paso 2), no llega en 8 s: se saltea y pasa a recibir.
        d.en_curso[0].volver_a(1, d.t);
        let lejos = cancha((Vec2::new(-0.40, 0.30), 90.0), (B, 180.0), MARCA, Vec2::ZERO);
        let c = correr_p(&mut d, &lejos, t + 1, 499);
        assert_eq!(de(&c, 0), choice(0, SkillId::GoTo, A));
        let c = correr_p(&mut d, &lejos, t + 500, 2);
        assert_eq!(de(&c, 0), choice(0, SkillId::Mark, A), "salteado: espera la pelota");
        assert_eq!(d.eventos.replanificaciones, 1);
        assert_eq!(d.fase(), Fase::Ejecutando);
    }

    #[test]
    fn without_progress_the_watchdog_aborts() {
        let mut d = pases(opciones(true));
        let t = hasta_ejecutar(&mut d, &cancha((A, 0.0), (B, 180.0), MARCA, Vec2::ZERO));
        // Nadie llega a la pelota: replanificaciones y rescates, sin progreso.
        let lejos = cancha((Vec2::new(-0.40, 0.30), 90.0), (B, 180.0), MARCA, Vec2::ZERO);
        correr_p(&mut d, &lejos, t + 1, 1860);
        assert_eq!(d.fase(), Fase::Ejecutando, "antes de 30 s no aborta");
        correr_p(&mut d, &lejos, t + 1861, 30);
        assert_eq!(d.tipo_aborto(), Some(TipoAborto::SinProgreso));
        let causa = d.causa().unwrap();
        assert!(causa.starts_with("sin progreso en 30 s: el robot ") && causa.ends_with("no logra controlar la pelota"), "{causa}");
    }

    #[test]
    fn in_simulation_a_ball_in_the_goal_is_put_back_on_the_mark() {
        let mut d = pases(opciones(true));
        let en_arco = Vec2::new(0.78, 0.05);
        let (t, c) = {
            let t = hasta_el_pase(&mut d);
            let _ = d.tomar_teleport(); // el de la preparación: el lazo lo toma en cada tick
            let w = cancha((A, 0.0), (B, 180.0), en_arco, Vec2::ZERO);
            let c = correr_p(&mut d, &w, t, 30);
            (t + 30, c)
        };
        assert!(matches!(d.estado, EstadoPelota::Fuera(_)), "{:?}", d.estado);
        assert_eq!(d.vista().frase, "Vuelta 1 de 1: repone la pelota en la marca");
        // A está a 0.12 m de la marca: primero se aparta.
        let a = de(&c, 0);
        assert!((a.target - MARCA).length() >= 0.30 - 1e-4, "{a:?}");
        assert!(d.tomar_teleport().is_empty(), "todavía no");
        let apartados = cancha((Vec2::new(-0.40, 0.35), 0.0), (B, 180.0), en_arco, Vec2::ZERO);
        correr_p(&mut d, &apartados, t, 1);
        let items = d.tomar_teleport();
        assert!(matches!(items.as_slice(), [TeleportItem::Ball { x, y, .. }] if (*x + 0.28).abs() < 1e-6 && y.abs() < 1e-6), "{items:?}");
        assert_eq!(d.eventos.reposiciones, 1);
        // La pelota ya en la marca: suelta, y A (el más cercano) va por ella.
        // (Espera a verla quieta en la marca: la ventana de la rapidez, por el salto.)
        let en_marca = cancha((Vec2::new(-0.40, 0.35), 0.0), (B, 180.0), MARCA, Vec2::ZERO);
        let c = correr_p(&mut d, &en_marca, t + 1, 25);
        assert!(matches!(d.estado, EstadoPelota::Suelta(_)), "{:?}", d.estado);
        assert_eq!(de(&c, 0), choice(0, SkillId::Intercept, MARCA));
    }

    #[test]
    fn in_real_a_ball_in_the_goal_pauses_and_the_pause_does_not_count() {
        let mut d = pases(real());
        let t = hasta_el_pase(&mut d);
        let en_arco = cancha((A, 0.0), (B, 180.0), Vec2::new(0.78, 0.05), Vec2::ZERO);
        let c = correr_p(&mut d, &en_arco, t, 30);
        assert!(c.iter().all(|c| c.skill_id == SkillId::Hold), "{c:?}");
        assert_eq!(d.vista().frase, "Vuelta 1 de 1: en pausa: saca la pelota del arco derecho");
        assert_eq!(d.eventos.pausas, 1);
        // Un minuto de pausa: el vigilante no aborta.
        correr_p(&mut d, &en_arco, t + 30, 3750);
        assert_eq!(d.fase(), Fase::Ejecutando);
        // La persona deja la pelota quieta en (0.20, 0): la rutina sigue sola.
        let afuera = cancha((A, 0.0), (B, 180.0), Vec2::new(0.20, 0.0), Vec2::ZERO);
        correr_p(&mut d, &afuera, t + 3780, 60);
        assert!(matches!(d.estado, EstadoPelota::Suelta(_)), "{:?}", d.estado);
        assert!((60.0..62.0).contains(&d.eventos.pausa_s), "{}", d.eventos.pausa_s);
        // Y el vigilante corre desde la reanudación, no desde antes de la pausa.
        correr_p(&mut d, &afuera, t + 3840, 10);
        assert_eq!(d.fase(), Fase::Ejecutando);
    }

    #[test]
    fn a_ball_near_the_side_wall_is_pushed_off_it() {
        let mut d = pases(opciones(true));
        let t = hasta_ejecutar(&mut d, &cancha((A, 0.0), (B, 180.0), MARCA, Vec2::ZERO));
        // Al pasar, la pelota está junto a la banda: no hay lugar detrás hacia B. (En el primer
        // tick ShootPush deja de ser factible y A vuelve a alinearse; ahí se ve.)
        let pelota = Vec2::new(-0.27, -0.57);
        let c = correr_p(&mut d, &cancha((A, 0.0), (B, 180.0), pelota, Vec2::ZERO), t + 1, 2);
        let EstadoPelota::Despegue(desp) = d.estado.clone() else { panic!("{:?}", d.estado) };
        let dir = (desp.objetivo - pelota).normalize();
        let adentro = dir.y.atan2(dir.x.abs()).to_degrees();
        assert!(adentro > 0.0 && adentro <= 10.0 && dir.x > 0.0, "{adentro:.1}°: {dir:?}");
        assert_eq!(de(&c, 0), choice(0, SkillId::ApproachAligned, desp.objetivo));
        assert_eq!(d.vista().frase, "Vuelta 1 de 1: el robot 0 despega la pelota de la pared");
        // A en el lugar detrás, alineado: empuja; tras 0.16 m, la pelota queda suelta.
        let lugar = ApproachAlignedSkill::new(desp.objetivo).staging_point(pelota).unwrap();
        let th = f64::from(dir.y).atan2(f64::from(dir.x)).to_degrees();
        let c = correr_p(&mut d, &cancha((lugar, th), (B, 180.0), pelota, Vec2::ZERO), t + 3, 1);
        assert_eq!(de(&c, 0), choice(0, SkillId::ShootPush, desp.objetivo));
        let avance = dir * 0.16;
        correr_p(&mut d, &cancha((lugar + avance, th), (B, 180.0), pelota + avance, dir * 0.3), t + 4, 1);
        assert_eq!(d.eventos.despegues, 1);
        assert!(matches!(d.estado, EstadoPelota::Suelta(_)), "{:?}", d.estado);
    }

    /// Hasta el despegue con la pelota junto a la banda; devuelve el tick siguiente, la
    /// pelota y el despegue.
    fn hasta_el_despegue(d: &mut RoutineDecider) -> (u32, Vec2, Despegando) {
        let t = hasta_ejecutar(d, &cancha((A, 0.0), (B, 180.0), MARCA, Vec2::ZERO));
        let pelota = Vec2::new(-0.27, -0.57);
        correr_p(d, &cancha((A, 0.0), (B, 180.0), pelota, Vec2::ZERO), t + 1, 2);
        let EstadoPelota::Despegue(desp) = d.estado.clone() else { panic!("{:?}", d.estado) };
        (t + 3, pelota, desp)
    }

    /// Una ida y vuelta del despegue: A en el lugar detrás de `pelota`, alineado (empuja), y
    /// después corrido 0.08 m al costado (deja de ser factible y vuelve a alinearse).
    fn oscilar(d: &mut RoutineDecider, t: u32, pelota: Vec2, desp: &Despegando) {
        let dir = (desp.objetivo - pelota).normalize();
        let lugar = ApproachAlignedSkill::new(desp.objetivo).staging_point(pelota).unwrap();
        let th = f64::from(dir.y).atan2(f64::from(dir.x)).to_degrees();
        let c = correr_p(d, &cancha((lugar, th), (B, 180.0), pelota, Vec2::ZERO), t, 1);
        assert_eq!(de(&c, 0), choice(0, SkillId::ShootPush, desp.objetivo));
        let c = correr_p(d, &cancha((lugar + dir.perp() * 0.08, th), (B, 180.0), pelota, Vec2::ZERO), t + 1, 1);
        assert_eq!(de(&c, 0), choice(0, SkillId::ApproachAligned, desp.objetivo));
    }

    #[test]
    fn a_despegue_that_oscillates_fails_at_once() {
        // Cuatro idas y vueltas en un segundo, aunque la pelota avance 1 cm en cada una por la
        // pared: el despegue falla sin esperar los 8 s, y la pelota queda fuera de juego (D15).
        let mut d = pases(opciones(true));
        let (mut t, mut pelota, desp) = hasta_el_despegue(&mut d);
        let dir = (desp.objetivo - pelota).normalize();
        for _ in 0..4 {
            pelota += dir * 0.01;
            oscilar(&mut d, t, pelota, &desp);
            t += 2;
        }
        correr_p(&mut d, &cancha((A, 0.0), (B, 180.0), pelota, Vec2::ZERO), t, 1);
        assert!(matches!(&d.estado, EstadoPelota::Fuera(f) if f.motivo == FueraDeJuego::Pegada), "{:?}", d.estado);
        assert_eq!(d.eventos.despegues_fallidos, 1);
        // Las mismas cuatro, repartidas en más de un segundo (una cada 0.4 s): sigue.
        let mut d = pases(opciones(true));
        let (mut t, pelota, desp) = hasta_el_despegue(&mut d);
        let dir = (desp.objetivo - pelota).normalize();
        let lugar = ApproachAlignedSkill::new(desp.objetivo).staging_point(pelota).unwrap();
        let th = f64::from(dir.y).atan2(f64::from(dir.x)).to_degrees();
        let corrido = cancha((lugar + dir.perp() * 0.08, th), (B, 180.0), pelota, Vec2::ZERO);
        for _ in 0..4 {
            oscilar(&mut d, t, pelota, &desp);
            correr_p(&mut d, &corrido, t + 2, 23);
            t += 25;
        }
        assert!(matches!(d.estado, EstadoPelota::Despegue(_)), "{:?}", d.estado);
        assert_eq!(d.eventos.despegues_fallidos, 0);
    }

    #[test]
    fn a_ball_in_a_corner_is_out_of_play() {
        let mut d = pases(opciones(true));
        let t = hasta_ejecutar(&mut d, &cancha((A, 0.0), (B, 180.0), MARCA, Vec2::ZERO));
        correr_p(&mut d, &cancha((A, 0.0), (B, 180.0), Vec2::new(-0.66, 0.56), Vec2::ZERO), t + 1, 2);
        assert!(
            matches!(&d.estado, EstadoPelota::Fuera(f) if f.motivo == FueraDeJuego::Pegada),
            "{:?}",
            d.estado
        );
        assert_eq!(d.fase(), Fase::Ejecutando);
    }

    #[test]
    fn a_ball_that_rolls_away_from_the_receiver_is_not_chased() {
        // Sin la regla 10 (retirada): tras tocar a B, la pelota rueda despacio dentro del radio y B la
        // espera; si se detiene fuera del radio, la recepción falla y la toma el rescate.
        let mut d = pases(opciones(true));
        let t = hasta_el_pase(&mut d);
        let escapa = B + Vec2::new(-0.07, -0.02);
        let c = correr_p(&mut d, &cancha((A, 0.0), (B, 180.0), escapa, Vec2::new(0.0, -0.13)), t, 1);
        assert_eq!(de(&c, 1), choice(1, SkillId::Mark, B));
        let afuera = B + Vec2::new(0.0, -0.30);
        correr_p(&mut d, &cancha((A, 0.0), (B, 180.0), afuera, Vec2::ZERO), t + 1, 30);
        assert!(matches!(d.estado, EstadoPelota::Suelta(_)), "{:?}", d.estado);
    }

    // ─── Triángulo: resincronización con tres robots ─────────────────────────

    const TA: Vec2 = Vec2::new(-0.40, -0.25);
    const TB: Vec2 = Vec2::new(0.0, 0.30);
    const TC: Vec2 = Vec2::new(0.40, -0.25);

    fn triangulo(pelota: Vec2, a: Vec2, b: Vec2, c: Vec2) -> World {
        let mut w = World::new(3, 3);
        w.update_robot(0, 0, a, 54f64.to_radians(), Vec2::ZERO, 0.0);
        w.update_robot(1, 0, b, (-54f64).to_radians(), Vec2::ZERO, 0.0);
        w.update_robot(2, 0, c, std::f64::consts::PI, Vec2::ZERO, 0.0);
        w.update_ball(pelota, Vec2::ZERO);
        w
    }

    fn tri(repeticiones: u32) -> (RoutineDecider, u32) {
        let op = OpcionesCorrida { repeticiones: Repeticiones::Veces(repeticiones), ..opciones(true) };
        let mut d = RoutineDecider::new(prueba::rutina(&prueba::triangulo()), op, entorno(false));
        let t = hasta_ejecutar(&mut d, &triangulo(Vec2::new(-0.32, -0.14), TA, TB, TC));
        assert_eq!(d.asignados(), &[0, 1, 2]);
        (d, t)
    }

    #[test]
    fn a_third_robot_rescues_and_the_pass_in_between_is_skipped() {
        let (mut d, t) = tri(1);
        // Falla A → B y la pelota queda junto a C, que la rescata.
        d.soltar(0, Some(1));
        let pelota = Vec2::new(0.30, -0.20);
        let c = correr_p(&mut d, &triangulo(pelota, TA, TB, TC), t + 1, 20);
        assert_eq!(de(&c, 2), choice(2, SkillId::Intercept, pelota));
        let c_en_pelota = triangulo(pelota, TA, TB, pelota + Vec2::new(0.0, -0.06));
        let c = correr_p(&mut d, &c_en_pelota, t + 21, 4);
        assert_eq!(d.eventos.cambios_de_rol, 1);
        // Se saltean A → B (el `empujar` de A y el `recibir` de B) y B → C; sigue C → A.
        for paso in [(0, 0), (1, 0), (1, 1), (2, 0)] {
            assert!(d.salteados.contains(&paso), "{paso:?} en {:?}", d.salteados);
        }
        assert!(pasa_a(de(&c, 2), TA), "{:?}", de(&c, 2));
        assert_eq!(de(&c, 0), choice(0, SkillId::Mark, TA), "A espera el pase de C");
    }

    #[test]
    fn a_resync_can_cross_into_the_next_lap() {
        let (mut d, t) = tri(2);
        // A ya pasó (espera en su `recibir`) y B le pasó a C, que no la recibió; A la rescata.
        d.en_curso[0].volver_a(2, d.t);
        d.en_curso[1].volver_a(2, d.t);
        d.soltar(1, Some(2));
        let pelota = TA + Vec2::new(0.10, 0.05);
        let a_en_pelota = triangulo(pelota, pelota - Vec2::new(0.06, 0.0), TB, TC);
        // Tres ticks de control y se resincroniza.
        correr_p(&mut d, &a_en_pelota, t + 1, 3);
        assert!(matches!(d.estado, EstadoPelota::EnJuego));
        // Se saltean B → C y C → A de esta vuelta; la siguiente empieza con A → B.
        for paso in [(1, 1), (2, 0), (2, 1), (0, 2)] {
            assert!(d.salteados.contains(&paso), "{paso:?} en {:?}", d.salteados);
        }
        assert!(d.salteados_siguiente.is_empty());
        let c = correr_p(&mut d, &a_en_pelota, t + 4, 3);
        assert_eq!(d.repeticion, 2, "la vuelta se cerró");
        assert!(pasa_a(de(&c, 0), TB), "{:?}", de(&c, 0));
    }

    #[test]
    fn every_abort_cause_has_its_type() {
        let w = en_inicio();
        let lejos = mundo(&[(0, 0.3, 0.3, 0.0)]);
        let fin = |d: &RoutineDecider| (d.causa().map(str::to_string), d.tipo_aborto(), d.resumen().and_then(|r| r.tipo_aborto));
        let mut casos: Vec<(RoutineDecider, &str, TipoAborto)> = Vec::new();
        // Tiempos vencidos (sin pelota): un paso y la preparación.
        let mut d = decisor(IR, opciones(true));
        let t0 = hasta_ejecutar(&mut d, &w);
        correr(&mut d, &w, t0 + 1, 510);
        casos.push((d, "el robot 0 no llegó al punto 1 en 8 s", TipoAborto::TiempoVencido));
        let mut d = decisor(IR, opciones(false));
        correr(&mut d, &lejos, 0, 1253);
        casos.push((d, "el robot 0 no llegó a su puesto en 20 s", TipoAborto::TiempoVencido));
        // Visión y casi choque.
        let mut d = decisor(IR, opciones(false));
        correr(&mut d, &w, 0, 1);
        d.decidir(1, &w, ahora_de(&w) + Duration::from_secs(1));
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
        // Con pelota, el vigilante.
        let mut d = pases(opciones(true));
        let t = hasta_ejecutar(&mut d, &cancha((A, 0.0), (B, 180.0), MARCA, Vec2::ZERO));
        correr_p(&mut d, &cancha((Vec2::new(-0.40, 0.30), 90.0), (B, 180.0), MARCA, Vec2::ZERO), t + 1, 1900);
        let causa = d.causa().unwrap_or_default().to_string();
        assert!(causa.starts_with("sin progreso en 30 s"), "{causa}");
        let causa: &'static str = Box::leak(causa.into_boxed_str());
        casos.push((d, causa, TipoAborto::SinProgreso));

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
    fn the_near_collision_check_waits_until_the_routine_teleport_is_seen() {
        // Los robots quedaron a 0.09 m (como al terminar una corrida anterior) y el teleport de
        // la rutina todavía no se ve: mientras no se vea, hasta 2 s, no aborta (en el punto de
        // control tardó más que los 0.5 s de `quieto_s`).
        let juntos = cancha((A, 0.0), (A + Vec2::new(0.09, 0.0), 180.0), MARCA, Vec2::ZERO);
        let mut d = pases(OpcionesCorrida { teletransportar: true, ..opciones(false) });
        correr_p(&mut d, &juntos, 0, 100);
        assert!(d.fase().activa(), "a 1.6 s, sin ver el teleport: {:?}", d.causa());
        assert!(!d.tomar_teleport().is_empty());
        // A los 2 s, si siguen juntos (el teleport no llegó), sí.
        correr_p(&mut d, &juntos, 100, 30);
        assert_eq!(d.causa(), Some("los robots 0 y 1 quedaron a 0.09 m"));
        // Visto el teleport (los dos en su inicio), la gracia termina: un casi choque después
        // cuenta enseguida.
        let mut d = pases(OpcionesCorrida { teletransportar: true, ..opciones(false) });
        correr_p(&mut d, &juntos, 0, 1);
        correr_p(&mut d, &cancha((A, 0.0), (B, 180.0), MARCA, Vec2::ZERO), 1, 40);
        assert!(d.fase().activa(), "{:?}", d.causa());
        correr_p(&mut d, &juntos, 41, 1);
        assert_eq!(d.causa(), Some("los robots 0 y 1 quedaron a 0.09 m"));
        // Sin teleport (el robot real), aborta desde el primer tick.
        let mut real = pases(OpcionesCorrida { simulacion: false, ..opciones(false) });
        correr_p(&mut real, &juntos, 0, 1);
        assert_eq!(real.causa(), Some("los robots 0 y 1 quedaron a 0.09 m"));
    }

    #[test]
    fn a_tracker_reset_after_the_pass_is_not_a_failed_reception() {
        // Al cortar el empuje, el tracker reinicia la pelota: salta 0.13 m y su velocidad queda
        // en cero aunque sigue rodando (punto de control de la etapa 1). Con el desplazamiento
        // de la ventana no parece quieta: no hay recepción fallida ni rescate.
        let mut d = pases(opciones(true));
        let t = hasta_el_pase(&mut d);
        let mut x = -0.12;
        for i in 0..10 {
            x += 0.008;
            correr_p(&mut d, &cancha((A, 0.0), (B, 180.0), Vec2::new(x, 0.0), Vec2::new(0.5, 0.0)), t + i, 1);
        }
        let saltada = Vec2::new(x + 0.13, 0.0);
        let c = correr_p(&mut d, &cancha((A, 0.0), (B, 180.0), saltada, Vec2::ZERO), t + 10, 5);
        assert!(matches!(d.estado, EstadoPelota::EnJuego), "{:?}", d.estado);
        assert_eq!(de(&c, 1), choice(1, SkillId::Mark, B));
        // Si de verdad queda quieta ahí, fuera del radio, la recepción falla.
        correr_p(&mut d, &cancha((A, 0.0), (B, 180.0), saltada, Vec2::ZERO), t + 15, 25);
        assert!(matches!(d.estado, EstadoPelota::Suelta(_)), "{:?}", d.estado);
    }

    #[test]
    fn in_real_the_preparation_waits_for_the_ball_without_aborting() {
        let real = OpcionesCorrida { simulacion: false, ..opciones(false) };
        let mut d = pases(real);
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
        let mut d = pases(opciones(false));
        correr_p(&mut d, &cancha((A, 0.0), (B, 180.0), Vec2::ZERO, Vec2::ZERO), 0, 1);
        let items = d.tomar_teleport();
        assert_eq!(items.len(), 1, "solo la pelota, sin `teletransportar`: {items:?}");
        assert!(matches!(items[0], TeleportItem::Ball { x, y, vx, vy }
            if (x + 0.28).abs() < 1e-6 && y.abs() < 1e-6 && vx == 0.0 && vy == 0.0));
        // Con `teletransportar`, los robots y la pelota en la marca.
        let mut d = pases(OpcionesCorrida { teletransportar: true, ..opciones(false) });
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
