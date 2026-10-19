//! GUI de depuración (`VSSL_DEBUG_GUI=1`): la interfaz se ve como la mesa de la
//! salita y lo único con color es el juego.
//!
//! Distribución: barra de estado en frases arriba, planilla de robots
//! a la izquierda, cancha al centro, panel con pestañas a la derecha y gráficos de
//! velocidad y giro abajo; con menos de 1100 px de ancho, todo en una columna.
//!
//! Datos: la GUI solo lee la foto que publica el lazo en cada tick (`crate::snapshot`,
//! canal de "último valor": nunca frena el lazo) y manda comandos explícitos (manual,
//! skill, PID, teleport, parada) por los mismos canales de siempre.

mod capas;
mod charts;
mod field;
mod fonts;
mod format;
mod inspector;
mod jugadores;
mod keys;
mod por_que;
#[cfg(test)]
mod render_tests;
mod robot_rows;
mod rutinas;
pub mod run_info;
mod status_line;
mod tag;
mod theme;
mod tools;

pub use run_info::{CoachKind, RunInfo};

use charts::MuestraSenal;
use field::{Fieltro, FieldCanvas, OrientacionVista};
use jugadores::{EstadoJugador, Escalas, RobotJugador};
use keys::{AccionTecla, EventoTeclado, Jugador, Layer, Tecla};
use status_line::Salud;
use tag::MarcasRobot;

use crate::control_loop::{GuiSkillCommand, HeadingPid, ManualCommand};
use crate::radio::TeleportItem;
use crate::rutinas::OrdenRutina;
use crate::skills::SkillId;
use crate::snapshot::{LoopSnapshot, SnapshotReceiver};
use glam::Vec2;
use iced::futures::SinkExt;
use iced::stream;
use iced::widget::canvas::Cache;
use iced::widget::{
    Canvas, Space, button, column, container, horizontal_rule, row, scrollable, text,
};
use iced::{Alignment, Element, Length, Size, Subscription, Task, Theme};
use serde::{Deserialize, Serialize};
use std::collections::{HashSet, VecDeque};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tokio::sync::mpsc;

// Las velocidades máximas del control manual (arranque y tope, por transporte) y el
// giro corto salen de `team_params.json` → `manual`. Giro positivo → CCW (omega > 0).
/// Aceleración lineal máx de la rampa del control manual (m/s²).
const MANUAL_LIN_ACCEL: f64 = 2.0;
/// Aceleración angular máx de la rampa del control manual (rad/s²).
const MANUAL_ANG_ACCEL: f64 = 12.0;
/// Paso temporal del `ManualTick` (s). Debe coincidir con el intervalo de la
/// suscripción `manual_tick` (33 ms).
const MANUAL_TICK_DT: f64 = 0.033;
/// Retención máxima de muestras de telemetría (rueda/motion/error). ~30 s @ 60 Hz,
/// suficiente para cubrir la ventana de tiempo máxima seleccionable.
const TELEMETRY_HISTORY_MAX: usize = 1800;
/// Presets de ventana de tiempo de la telemetría (segundos mostrados).
const TELEMETRY_WINDOWS_S: [f64; 3] = [5.0, 15.0, 30.0];
/// Ventana de tiempo por defecto (s).
const TELEMETRY_WINDOW_DEFAULT_S: f64 = 5.0;
/// Velocidad angular de Spin por defecto en la GUI (rad/s). Coincide con
/// `SkillConfig::default().spin_omega`.
const SPIN_OMEGA_DEFAULT: f64 = 20.0;
/// Ganancias PID de heading por defecto (coinciden con `SkillConfig::default`).
const PID_KP_DEFAULT: f64 = 3.0;
const PID_KI_DEFAULT: f64 = 0.08;
const PID_KD_DEFAULT: f64 = 0.20;

/// Conjunto de parámetros de tuning, serializable a JSON para presets.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TuningPreset {
    pub lin_ms: f64,
    pub ang_rads: f64,
    pub lin_accel: f64,
    pub ang_accel: f64,
    pub spin_omega: f64,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

/// Recorta un preset al tope del transporte y lo valida como los campos: escalas
/// finitas en `[0, tope]` (por encima, el tope), aceleraciones y giro de Spin finitos y
/// mayores que cero, y PID finito y no negativo. Un valor inválido conserva el actual.
pub fn preset_valido(p: &TuningPreset, actual: &TuningPreset, tope_lin: f64, tope_ang: f64) -> TuningPreset {
    let escala = |v: f64, actual: f64, tope: f64| {
        if v.is_finite() && v >= 0.0 { v.min(tope) } else { actual }
    };
    let positivo = |v: f64, actual: f64| if v.is_finite() && v > 0.0 { v } else { actual };
    let no_negativo = |v: f64, actual: f64| if v.is_finite() && v >= 0.0 { v } else { actual };
    TuningPreset {
        lin_ms: escala(p.lin_ms, actual.lin_ms, tope_lin),
        ang_rads: escala(p.ang_rads, actual.ang_rads, tope_ang),
        lin_accel: positivo(p.lin_accel, actual.lin_accel),
        ang_accel: positivo(p.ang_accel, actual.ang_accel),
        spin_omega: positivo(p.spin_omega, actual.spin_omega),
        kp: no_negativo(p.kp, actual.kp),
        ki: no_negativo(p.ki, actual.ki),
        kd: no_negativo(p.kd, actual.kd),
    }
}

/// Una escala del manual escrita en su campo: válida si es finita y está en `[0, tope]`.
pub fn escala_valida(texto: &str, tope: f64) -> Option<f64> {
    texto.parse::<f64>().ok().filter(|v| v.is_finite() && *v >= 0.0 && *v <= tope)
}

/// Rampa por componente: acerca `current` a `target` sin pasarlo, con paso máximo
/// `max_step`. Función pura y testeable.
pub fn slew(current: f64, target: f64, max_step: f64) -> f64 {
    let delta = target - current;
    if delta.abs() <= max_step {
        target
    } else {
        current + max_step * delta.signum()
    }
}

/// Para el runner de `Spin`: devuelve un target cuyo `x` lleva el signo de
/// `(skill_target_x − robot_x)`, de modo que el sentido del giro dependa del lado
/// del robot donde se clickeó. Empate/`0` → `+1` (siempre gira). `y = 0`.
/// Función pura y testeable. Unidades: ambos en metros.
pub fn spin_relative_target(skill_target_x: f32, robot_x_m: f32) -> Vec2 {
    let sign = if skill_target_x - robot_x_m < 0.0 {
        -1.0
    } else {
        1.0
    };
    Vec2::new(sign, 0.0)
}

/// Botones del runner de skills, agrupados: `(título, skills en orden)`. Es la
/// única fuente de los botones de la sección Skills; el test
/// `every_catalog_skill_has_button_and_help` falla si una skill del catálogo no
/// aparece exactamente una vez.
const SKILL_GROUPS: [(&str, &[SkillId]); 4] = [
    (
        "Navegación",
        &[SkillId::GoTo, SkillId::FacePoint, SkillId::Spin],
    ),
    (
        "Ataque",
        &[
            SkillId::ChaseBall,
            SkillId::ApproachAligned,
            SkillId::ShootPush,
            SkillId::Clear,
            SkillId::SpinKick,
            SkillId::Dribble,
        ],
    ),
    (
        "Defensa",
        &[
            SkillId::Intercept,
            SkillId::BlockLine,
            SkillId::GoalKeep,
            SkillId::RailKeeper,
            SkillId::Mark,
        ],
    ),
    ("Quieto", &[SkillId::Hold]),
];

/// El botón "ninguna" de la sección Skills y su ayuda: con el coach en pausa, el robot
/// sin skill queda quieto (`coach-pausa`).
pub fn texto_ninguna(coach_en_pausa: bool) -> (&'static str, &'static str) {
    if coach_en_pausa {
        (
            "ninguna (quieto)",
            "Ninguna: el coach está en pausa y el robot queda quieto. Elige una skill y haz click en la cancha.",
        )
    } else {
        ("ninguna (coach)", skill_help(None))
    }
}

/// Aviso de Herramientas cuando un jugador maneja al arquero del coach con el coach
/// activo (`coach-pausa`): el coach no lo reemplaza y el arco queda solo.
pub fn aviso_arquero_manual(jugadores: &[Option<(i32, i32)>], own_team: i32, keeper_id: i32, coach_activo: bool) -> Option<String> {
    (coach_activo && jugadores.contains(&Some((own_team, keeper_id))))
        .then(|| format!("el robot {keeper_id} es el arquero del coach: el arco queda solo"))
}

/// Línea de ayuda de la skill activa: qué significa el click para esa skill (el
/// `target` de `SkillCatalog::tick`) o que no se usa. `None` = sin skill (coach).
pub fn skill_help(skill: Option<SkillId>) -> &'static str {
    match skill {
        None => "Ninguna: el coach controla el robot. Elige una skill y haz click en la cancha.",
        Some(SkillId::GoTo) => "click = destino.",
        Some(SkillId::FacePoint) => "click = punto a mirar (gira en el lugar).",
        Some(SkillId::Spin) => "click = sentido de giro: con x mayor que la del robot gira antihorario (CCW), con x menor, horario (CW).",
        Some(SkillId::ChaseBall) => "el click no se usa: persigue la pelota.",
        Some(SkillId::ApproachAligned) => {
            "click = hacia dónde apuntar: se pone detrás de la pelota, alineado con la \
             línea de la pelota al click."
        }
        Some(SkillId::ShootPush) => {
            "click = a dónde mandar la pelota. Solo empuja si ya está detrás de la \
             pelota; si no, queda quieto (usa ApproachAligned antes)."
        }
        Some(SkillId::Clear) => {
            "click = punto de despeje: se pone detrás de la pelota por el camino corto \
             y empuja."
        }
        Some(SkillId::SpinKick) => {
            "click = hacia dónde lanzar la pelota (contacto lateral + giro, con el giro de \
             team_params, no la de Parámetros)."
        }
        Some(SkillId::Intercept) => {
            "el click no se usa: va al punto de intercepción predicho de la pelota."
        }
        Some(SkillId::BlockLine) => {
            "el click no se usa: cubre la línea de la pelota al arco propio (lado según VSSL_SIDE)."
        }
        Some(SkillId::GoalKeep) => {
            "el click no se usa: arquero en la línea del arco propio (VSSL_SIDE). Solo el \
             robot coach.keeper_id puede entrar al área."
        }
        Some(SkillId::RailKeeper) => {
            "el click no se usa: arquero sobre un riel a 5 cm del arco propio, orientado a lo \
             largo de él, anticipando el empuje rival (VSSL_SIDE). Solo coach.keeper_id entra al área."
        }
        Some(SkillId::Mark) => "click = dónde pararse; queda mirando a la pelota.",
        Some(SkillId::Dribble) => "click = hacia dónde llevar la pelota (campo univectorial).",
        Some(SkillId::Hold) => "el click no se usa: quieto (sin avance ni giro).",
    }
}

/// Centro del arco propio (m, marco mundo) según el lado que defendemos. Mismo
/// punto que `goals_for_team` de `main`.
pub fn own_goal_center(defend_left: bool) -> Vec2 {
    let x = if defend_left {
        -crate::coach::FIELD_HALF_X
    } else {
        crate::coach::FIELD_HALF_X
    };
    Vec2::new(x, 0.0)
}

/// Target que la GUI manda al loop para `skill`. `Spin`: sentido relativo al
/// robot (`spin_relative_target`). `BlockLine`/`GoalKeep`: el arco propio, porque
/// su `target` es el centro del arco propio y no un punto que se pueda clickear.
/// Resto: el click (las skills que ignoran el target lo reciben igual).
pub fn effective_skill_target(skill: SkillId, click: Vec2, robot_x_m: f32, own_goal: Vec2) -> Vec2 {
    match skill {
        SkillId::Spin => spin_relative_target(click.x, robot_x_m),
        SkillId::BlockLine | SkillId::GoalKeep | SkillId::RailKeeper => own_goal,
        _ => click,
    }
}

/// Dónde dibujar el marcador del target en la cancha: el arco propio en
/// `BlockLine`/`GoalKeep`, el click en las skills que lo usan (incluida `Spin`,
/// que usa su lado) y ninguno en las que lo ignoran (`ChaseBall`, `Intercept`, `Hold`).
pub fn skill_marker(skill: SkillId, click: Vec2, own_goal: Vec2) -> Option<Vec2> {
    match skill {
        SkillId::BlockLine | SkillId::GoalKeep | SkillId::RailKeeper => Some(own_goal),
        s if s.uses_target() || s.uses_target_sign() => Some(click),
        _ => None,
    }
}

/// Aviso para `GoalKeep` en un robot que no es el arquero: el `ZoneGuard` del
/// loop solo deja entrar al área propia a `coach.keeper_id`, y la GUI no se
/// exime del guardia (mismo comportamiento que `main` y `skill_test`).
pub fn keeper_warning(
    skill: Option<SkillId>,
    selected_robot: u32,
    keeper_id: i32,
) -> Option<String> {
    (matches!(skill, Some(SkillId::GoalKeep | SkillId::RailKeeper)) && selected_robot as i32 != keeper_id).then(|| {
        format!(
            "este robot no es el arquero (keeper_id = {keeper_id} en team_params.json): \
             el guardia no lo deja entrar al área"
        )
    })
}

/// Agrega una muestra a una serie de telemetría, descartando las más viejas al
/// superar la ventana de retención máxima.
fn push_capped<T>(q: &mut VecDeque<T>, item: T) {
    q.push_back(item);
    while q.len() > TELEMETRY_HISTORY_MAX {
        q.pop_front();
    }
}

/// Envuelve un ángulo (rad) al rango `[-π, π]`.
fn wrap_angle(a: f32) -> f32 {
    let mut a = a % (2.0 * std::f32::consts::PI);
    if a > std::f32::consts::PI {
        a -= 2.0 * std::f32::consts::PI;
    } else if a < -std::f32::consts::PI {
        a += 2.0 * std::f32::consts::PI;
    }
    a
}

/// Tope de cuadros por segundo de la GUI: entre dos fotos pasan al menos 1/30 s. Con
/// 60, la medición del lazo con la GUI abierta en el notebook (WSLg, Vulkan por
/// software) no cumplía el criterio: la GUI le quitaba CPU al lazo y a FIRASim.
pub const GUI_MAX_FPS: f32 = 30.0;
/// Debajo de este ancho (px), todo va en una columna con la cancha primero.
const ANCHO_ANGOSTO: f32 = 1100.0;
/// Tamaño inicial de la ventana (px).
const VENTANA_INICIAL: (f32, f32) = (1440.0, 900.0);
/// Ancho de la planilla de robots y del panel de pestañas, y alto de los gráficos.
const ANCHO_ROBOTS: f32 = 300.0;
const ANCHO_PANEL: f32 = 300.0;
/// Espaciado y margen lateral de la barra de pestañas (cinco en `ANCHO_PANEL`).
const BARRA_PESTANAS_SP: f32 = theme::SP_XS;
const BARRA_PESTANAS_MARGEN: f32 = 4.0;
const ALTO_GRAFICOS: f32 = 168.0;

/// Pestañas del panel derecho. La barra muestra solo las implementadas.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tab {
    /// Decisión del coach (fase 2).
    PorQue,
    Inspector,
    /// Comandos del árbitro (fase 4).
    Arbitro,
    /// Rutinas: lista, opciones, botones por fase y estado (capability `gui-rutinas`).
    Rutinas,
    Parametros,
    /// Provisional hasta la fase 4: control manual, skills de la GUI y teleport.
    Herramientas,
}

impl Tab {
    /// Las implementadas en esta fase.
    pub const VISIBLES: [Tab; 5] = [Tab::PorQue, Tab::Inspector, Tab::Rutinas, Tab::Parametros, Tab::Herramientas];

    pub fn titulo(self) -> &'static str {
        match self {
            Tab::PorQue => "Por qué",
            Tab::Inspector => "Inspector",
            Tab::Arbitro => "Árbitro",
            Tab::Rutinas => "Rutinas",
            Tab::Parametros => "Parámetros",
            Tab::Herramientas => "Herramientas",
        }
    }
}

#[derive(Debug, Clone)]
pub enum Message {
    /// Foto del tick (la última publicada por el lazo).
    Snapshot(Arc<LoopSnapshot>),
    /// El lazo terminó (se cerró el canal de fotos): la GUI se cierra, y el hilo
    /// principal sale después de la parada (`parada::con_gui`).
    LazoTerminado,
    /// Nuevo tamaño de la ventana.
    Resized(Size),
    /// Teclado y foco de la ventana (ver `keys::evento_de` y `keys::ruta_tecla`).
    Teclado(EventoTeclado),
    /// Tick rápido que recomputa y envía el comando manual o la skill de la GUI.
    ManualTick,
    SelectTab(Tab),
    /// Click en una fila: un robot propio (con Shift en el modo manual, el jugador 2).
    SelectRow(u32),
    SelectRobot(u32),
    SelectTeam(u32),
    ToggleManual(bool),
    /// Click corto sobre un robot propio en la cancha, dentro del modo manual.
    ClicRobot { id: i32, shift: bool },
    /// Suelta al jugador 2.
    SoltarJugador2,
    /// Alterna el coach entre activo y en pausa (botón de la barra).
    ToggleCoach,
    ManualLinChanged(String),
    ManualAngChanged(String),
    ManualLinAccelChanged(String),
    ManualAngAccelChanged(String),
    SelectSkill(Option<SkillId>),
    FieldClicked(Vec2),
    /// Encender o apagar una capa (botón de la barra de capas).
    ToggleCapa(Layer),
    /// Teleport arrastrando en la cancha (solo en simulación).
    TeleportArrastre(Vec<TeleportItem>),
    SpinOmegaChanged(String),
    PidKpChanged(String),
    PidKiChanged(String),
    PidKdChanged(String),
    PresetNameChanged(String),
    SavePreset,
    LoadPreset,
    ToggleEstop,
    TpRobotXChanged(String),
    TpRobotYChanged(String),
    TpRobotThetaChanged(String),
    TpBallXChanged(String),
    TpBallYChanged(String),
    TeleportRobot,
    TeleportBall,
    ToggleTelemetryFreeze(bool),
    SetTelemetryWindow(f64),
    SetFieltro(Fieltro),
    SetOrientacion(OrientacionVista),
    SetMarcas(MarcasRobot),
    /// Pestaña Rutinas.
    Rutina(rutinas::MensajeRutina),
}

/// Cuadros por segundo de la propia GUI (fotos procesadas por segundo).
#[derive(Debug)]
struct FpsGui {
    inicio: Instant,
    cuenta: u32,
    valor: f32,
}

impl FpsGui {
    fn new() -> Self {
        Self {
            inicio: Instant::now(),
            cuenta: 0,
            valor: 0.0,
        }
    }

    fn registrar(&mut self, ahora: Instant) {
        self.cuenta += 1;
        let dt = ahora.duration_since(self.inicio).as_secs_f32();
        if dt >= 1.0 {
            self.valor = self.cuenta as f32 / dt;
            self.cuenta = 0;
            self.inicio = ahora;
        }
    }

    fn valor(&self) -> f32 {
        self.valor
    }
}

/// Parámetros de arranque de la GUI: canales y configuración de la corrida.
pub struct GuiSetup {
    /// Fotos del lazo (canal de "último valor"; ver `crate::snapshot`).
    pub snapshot_rx: SnapshotReceiver,
    /// Canal de comandos manuales GUI → lazo.
    pub manual_tx: Option<mpsc::Sender<ManualCommand>>,
    /// Canal de skills de la GUI → lazo.
    pub skill_tx: Option<mpsc::Sender<GuiSkillCommand>>,
    /// Canal de tuneo del PID de heading → lazo.
    pub pid_tx: Option<mpsc::Sender<HeadingPid>>,
    /// Canal de teleport (sim) → lazo.
    pub teleport_tx: Option<mpsc::Sender<Vec<TeleportItem>>>,
    /// Órdenes de las rutinas → conmutador de `main`. `None` (p. ej. `scenario`): la
    /// pestaña muestra la lista y la vista previa, sin poder preparar.
    pub rutina_tx: Option<mpsc::Sender<OrdenRutina>>,
    /// Flag de parada de emergencia compartido con el lazo.
    pub estop: Arc<AtomicBool>,
    /// Interruptor del coach (en pausa = `true`), compartido con el conmutador de `main`.
    /// `None` sin coach que pausar (`VSSL_COACH=none`, `scenario`): no hay interruptor.
    pub coach_pausa: Option<Arc<AtomicBool>>,
    /// Configuración de la corrida (solo lectura).
    pub run: RunInfo,
}

pub struct DebugGui {
    // Datos: la última foto y la salud calculada de sus contadores.
    snap: Arc<LoopSnapshot>,
    snapshot_rx: Arc<Mutex<Option<SnapshotReceiver>>>,
    run: RunInfo,
    salud: Salud,
    fps_gui: FpsGui,
    // Vista.
    ancho: f32,
    alto: f32,
    tab: Tab,
    fieltro: Fieltro,
    orientacion: OrientacionVista,
    marcas: MarcasRobot,
    cache_base: Cache,
    /// Capas pedidas por tecla. Fase 2: la cancha las dibuja; en esta fase no hay.
    capas: HashSet<Layer>,
    // Robot seleccionado (herramientas, Inspector, gráficos, marca en la cancha). Es
    // también el robot del jugador 1 en el modo manual.
    selected_robot: u32,
    selected_team: u32,
    // Control manual: dos jugadores.
    manual_enabled: bool,
    /// Robot del jugador 2: propio, o ninguno.
    robot_j2: Option<RobotJugador>,
    /// Rampa y giro corto de cada jugador (`Jugador::indice`).
    jugadores: [EstadoJugador; 2],
    /// Teclas apretadas (de los dos jugadores y P).
    teclas: HashSet<Tecla>,
    /// Shift apretado, para Shift y click en una fila (la cancha lleva el suyo).
    shift: bool,
    /// Topes (y arranque) de las escalas del manual según el transporte.
    tope_lin: f64,
    tope_ang: f64,
    manual_lin_ms: f64,
    manual_ang_rads: f64,
    manual_lin_str: String,
    manual_ang_str: String,
    manual_lin_accel: f64,
    manual_ang_accel: f64,
    manual_lin_accel_str: String,
    manual_ang_accel_str: String,
    manual_tx: Option<mpsc::Sender<ManualCommand>>,
    /// Interruptor del coach (ver `GuiSetup::coach_pausa`).
    coach_pausa: Option<Arc<AtomicBool>>,
    // Skills de la GUI.
    active_skill: Option<SkillId>,
    /// Target de la skill activa (m, marco mundo), fijado con click.
    skill_target: Vec2,
    skill_tx: Option<mpsc::Sender<GuiSkillCommand>>,
    /// Centro del arco propio (m), desde `VSSL_SIDE`.
    own_goal: Vec2,
    /// Único robot que el `ZoneGuard` deja entrar al área propia (`coach.keeper_id`).
    keeper_id: i32,
    // Tuning.
    spin_omega: f64,
    spin_omega_str: String,
    pid_kp: f64,
    pid_ki: f64,
    pid_kd: f64,
    pid_kp_str: String,
    pid_ki_str: String,
    pid_kd_str: String,
    pid_tx: Option<mpsc::Sender<HeadingPid>>,
    preset_name: String,
    preset_status: String,
    // Teleport.
    teleport_tx: Option<mpsc::Sender<Vec<TeleportItem>>>,
    tp_robot_x: String,
    tp_robot_y: String,
    tp_robot_theta: String,
    tp_ball_x: String,
    tp_ball_y: String,
    estop: Arc<AtomicBool>,
    // Telemetría del robot seleccionado.
    serie_v: VecDeque<MuestraSenal>,
    serie_w: VecDeque<MuestraSenal>,
    cache_v: Cache,
    cache_w: Cache,
    /// `(t, error de heading, distancia)` respecto del target.
    skill_err_history: VecDeque<(f64, f32, f32)>,
    cache_err: Cache,
    /// Cuadros de visión por segundo, último minuto.
    cuadros_hist: VecDeque<(u64, u64)>,
    /// Segundo y cuenta de cuadros del inicio del segundo en curso.
    cuadros_base: Option<(u64, u64)>,
    cache_cuadros: Cache,
    telemetry_frozen: bool,
    telemetry_window_s: f64,
    // Trazas de la capa T (últimos segundos de cada robot y de la pelota).
    trazas: capas::Trazas,
    // Pestaña Rutinas.
    rutinas: rutinas::EstadoRutinas,
}

impl DebugGui {
    fn new(setup: GuiSetup) -> (Self, Task<Message>) {
        let own_goal = own_goal_center(setup.run.defend_left);
        let selected_team = setup.run.own_team.max(0) as u32;
        let manual = &crate::params::params().manual;
        let tope_lin = manual.velocidad_m_s(setup.run.is_base_station());
        let tope_ang = manual.giro_rad_s;
        let rutinas = rutinas::EstadoRutinas::new(
            setup.rutina_tx,
            setup.run.own_team,
            !setup.run.is_base_station(),
        );
        (
            DebugGui {
                snap: Arc::new(LoopSnapshot::default()),
                snapshot_rx: Arc::new(Mutex::new(Some(setup.snapshot_rx))),
                salud: Salud::default(),
                fps_gui: FpsGui::new(),
                ancho: VENTANA_INICIAL.0,
                alto: VENTANA_INICIAL.1,
                tab: Tab::PorQue,
                fieltro: Fieltro::default(),
                orientacion: OrientacionVista::from_env(),
                marcas: MarcasRobot::from_env(),
                cache_base: Cache::default(),
                capas: capas::CAPAS_INICIALES.into_iter().collect(),
                trazas: capas::Trazas::default(),
                selected_robot: 0,
                selected_team,
                manual_enabled: false,
                robot_j2: None,
                jugadores: Default::default(),
                teclas: HashSet::new(),
                shift: false,
                tope_lin,
                tope_ang,
                manual_lin_ms: tope_lin,
                manual_ang_rads: tope_ang,
                manual_lin_str: format!("{tope_lin}"),
                manual_ang_str: format!("{tope_ang}"),
                manual_lin_accel: MANUAL_LIN_ACCEL,
                manual_ang_accel: MANUAL_ANG_ACCEL,
                manual_lin_accel_str: format!("{MANUAL_LIN_ACCEL}"),
                manual_ang_accel_str: format!("{MANUAL_ANG_ACCEL}"),
                manual_tx: setup.manual_tx,
                coach_pausa: setup.coach_pausa,
                active_skill: None,
                skill_target: Vec2::ZERO,
                skill_tx: setup.skill_tx,
                own_goal,
                keeper_id: crate::params::params().coach.keeper_id,
                spin_omega: SPIN_OMEGA_DEFAULT,
                spin_omega_str: format!("{SPIN_OMEGA_DEFAULT}"),
                pid_kp: PID_KP_DEFAULT,
                pid_ki: PID_KI_DEFAULT,
                pid_kd: PID_KD_DEFAULT,
                pid_kp_str: format!("{PID_KP_DEFAULT}"),
                pid_ki_str: format!("{PID_KI_DEFAULT}"),
                pid_kd_str: format!("{PID_KD_DEFAULT}"),
                pid_tx: setup.pid_tx,
                preset_name: "tuning.json".to_string(),
                preset_status: String::new(),
                teleport_tx: setup.teleport_tx,
                tp_robot_x: "0.0".to_string(),
                tp_robot_y: "0.0".to_string(),
                tp_robot_theta: "0.0".to_string(),
                tp_ball_x: "0.0".to_string(),
                tp_ball_y: "0.0".to_string(),
                estop: setup.estop,
                serie_v: VecDeque::new(),
                serie_w: VecDeque::new(),
                cache_v: Cache::default(),
                cache_w: Cache::default(),
                skill_err_history: VecDeque::new(),
                cache_err: Cache::default(),
                cuadros_hist: VecDeque::new(),
                cuadros_base: None,
                cache_cuadros: Cache::default(),
                telemetry_frozen: false,
                telemetry_window_s: TELEMETRY_WINDOW_DEFAULT_S,
                run: setup.run,
                rutinas,
            },
            Task::none(),
        )
    }

    fn title(&self) -> String {
        String::from("Sysmic VSSS, depuración")
    }

    fn theme(&self) -> Theme {
        theme::theme()
    }

    fn estop_activa(&self) -> bool {
        self.estop.load(Ordering::Relaxed)
    }

    fn update(&mut self, message: Message) -> Task<Message> {
        match message {
            Message::LazoTerminado => return iced::exit(),
            Message::Snapshot(snap) => {
                self.fps_gui.registrar(Instant::now());
                self.salud.push(snap.t_ms, snap.vision, snap.radio);
                self.trazas.registrar(&snap);
                self.snap = snap;
                self.registrar_cuadros();
                if !self.telemetry_frozen {
                    self.registrar_series();
                }
            }
            Message::Resized(size) => {
                self.ancho = size.width;
                self.alto = size.height;
            }
            Message::Teclado(evento) => self.teclado(evento),
            Message::ManualTick => self.manual_tick(),
            Message::ToggleCapa(capa) => self.alternar_capa(capa),
            Message::TeleportArrastre(items) => {
                if let Some(tx) = &self.teleport_tx {
                    let _ = tx.try_send(items);
                }
            }
            Message::SelectTab(t) => self.tab = t,
            Message::SelectRow(id) => {
                let robot = (self.run.own_team, id as i32);
                let jugador = if self.manual_enabled && self.shift { Jugador::Dos } else { Jugador::Uno };
                self.asignar(jugador, robot);
            }
            Message::SelectRobot(id) => self.asignar(Jugador::Uno, (self.selected_team as i32, id as i32)),
            Message::SelectTeam(team) => self.asignar(Jugador::Uno, (team as i32, self.selected_robot as i32)),
            Message::ToggleManual(on) => self.set_manual(on),
            Message::ClicRobot { id, shift } => {
                if self.manual_enabled {
                    let jugador = if shift { Jugador::Dos } else { Jugador::Uno };
                    self.asignar(jugador, (self.run.own_team, id));
                }
            }
            Message::SoltarJugador2 => {
                let antes = (self.robot_j1(), self.robot_j2);
                self.reasignar(antes, (antes.0, None));
            }
            Message::ToggleCoach => self.alternar_coach(),
            Message::ManualLinChanged(s) => {
                if let Some(v) = escala_valida(&s, self.tope_lin) {
                    self.manual_lin_ms = v;
                }
                self.manual_lin_str = s;
            }
            Message::ManualAngChanged(s) => {
                if let Some(v) = escala_valida(&s, self.tope_ang) {
                    self.manual_ang_rads = v;
                }
                self.manual_ang_str = s;
            }
            Message::ManualLinAccelChanged(s) => {
                if let Ok(v) = s.parse::<f64>()
                    && v.is_finite()
                    && v > 0.0
                {
                    self.manual_lin_accel = v;
                }
                self.manual_lin_accel_str = s;
            }
            Message::ManualAngAccelChanged(s) => {
                if let Ok(v) = s.parse::<f64>()
                    && v.is_finite()
                    && v > 0.0
                {
                    self.manual_ang_accel = v;
                }
                self.manual_ang_accel_str = s;
            }
            Message::SelectSkill(skill) => self.active_skill = skill,
            Message::FieldClicked(mundo) => {
                // El click fija el target de la skill activa (si hay). Dentro del modo
                // manual la cancha no lo emite: el click asigna robots a los jugadores.
                if self.active_skill.is_some() && !self.manual_enabled {
                    self.skill_target = mundo;
                }
            }
            Message::SpinOmegaChanged(s) => {
                if let Ok(v) = s.parse::<f64>()
                    && v.is_finite()
                    && v > 0.0
                {
                    self.spin_omega = v;
                }
                self.spin_omega_str = s;
            }
            Message::PidKpChanged(s) => {
                if let Ok(v) = s.parse::<f64>()
                    && v.is_finite()
                    && v >= 0.0
                {
                    self.pid_kp = v;
                    self.send_pid();
                }
                self.pid_kp_str = s;
            }
            Message::PidKiChanged(s) => {
                if let Ok(v) = s.parse::<f64>()
                    && v.is_finite()
                    && v >= 0.0
                {
                    self.pid_ki = v;
                    self.send_pid();
                }
                self.pid_ki_str = s;
            }
            Message::PidKdChanged(s) => {
                if let Ok(v) = s.parse::<f64>()
                    && v.is_finite()
                    && v >= 0.0
                {
                    self.pid_kd = v;
                    self.send_pid();
                }
                self.pid_kd_str = s;
            }
            Message::PresetNameChanged(s) => self.preset_name = s,
            Message::SavePreset => {
                self.preset_status = match self.save_preset() {
                    Ok(()) => format!("Guardado en {}.", self.preset_name),
                    Err(e) => format!("No se pudo guardar: {e}"),
                };
            }
            Message::LoadPreset => {
                self.preset_status = match self.load_preset() {
                    Ok(()) => {
                        self.send_pid();
                        format!("Cargado de {}.", self.preset_name)
                    }
                    Err(e) => format!("No se pudo cargar: {e}"),
                };
            }
            Message::ToggleEstop => {
                // Enclava o libera.
                let ahora = self.estop.load(Ordering::Relaxed);
                self.estop.store(!ahora, Ordering::Relaxed);
            }
            Message::TpRobotXChanged(s) => self.tp_robot_x = s,
            Message::TpRobotYChanged(s) => self.tp_robot_y = s,
            Message::TpRobotThetaChanged(s) => self.tp_robot_theta = s,
            Message::TpBallXChanged(s) => self.tp_ball_x = s,
            Message::TpBallYChanged(s) => self.tp_ball_y = s,
            Message::TeleportRobot => {
                if let (Ok(x), Ok(y), Ok(theta)) = (
                    self.tp_robot_x.parse::<f64>(),
                    self.tp_robot_y.parse::<f64>(),
                    self.tp_robot_theta.parse::<f64>(),
                ) && let Some(tx) = &self.teleport_tx
                {
                    let _ = tx.try_send(vec![TeleportItem::Robot {
                        team: self.selected_team,
                        id: self.selected_robot,
                        x,
                        y,
                        theta,
                    }]);
                }
            }
            Message::TeleportBall => {
                if let (Ok(x), Ok(y)) = (self.tp_ball_x.parse::<f64>(), self.tp_ball_y.parse::<f64>())
                    && let Some(tx) = &self.teleport_tx
                {
                    let _ = tx.try_send(vec![TeleportItem::Ball {
                        x,
                        y,
                        vx: 0.0,
                        vy: 0.0,
                    }]);
                }
            }
            Message::ToggleTelemetryFreeze(on) => self.telemetry_frozen = on,
            Message::SetTelemetryWindow(s) => {
                self.telemetry_window_s = s;
                self.clear_chart_caches();
            }
            Message::SetFieltro(f) => {
                self.fieltro = f;
                self.cache_base.clear();
            }
            Message::SetOrientacion(o) => {
                self.orientacion = o;
                self.cache_base.clear();
            }
            Message::SetMarcas(m) => self.marcas = m,
            Message::Rutina(m) => self.mensaje_rutina(m),
        }
        Task::none()
    }

    /// Enciende o apaga una capa de esta fase (la V, de la fase 4, no tiene efecto).
    fn alternar_capa(&mut self, capa: Layer) {
        if !capas::CAPAS.contains(&capa) {
            return;
        }
        if !self.capas.remove(&capa) {
            self.capas.insert(capa);
        }
    }

    /// Entra o sale del modo manual. Al salir, cada robot manejado recibe un comando cero
    /// (en FIRASim, un robot que deja de recibir comandos sigue a su última velocidad) y
    /// se vacían las teclas, las rampas y los giros. Las asignaciones quedan. Una skill de
    /// la GUI elegida no se envía durante el modo manual y vuelve al salir, sobre el robot
    /// seleccionado (el del jugador 1), como antes de los dos jugadores.
    fn set_manual(&mut self, on: bool) {
        if !on && self.manual_enabled {
            for robot in self.robots_jugadores().into_iter().flatten() {
                self.enviar_cero(robot);
            }
        }
        self.manual_enabled = on;
        self.teclas.clear();
        for j in &mut self.jugadores {
            j.soltar();
        }
    }

    /// Robot del jugador 1: el seleccionado.
    fn robot_j1(&self) -> RobotJugador {
        (self.selected_team as i32, self.selected_robot as i32)
    }

    /// Robots de los dos jugadores, por `Jugador::indice`.
    fn robots_jugadores(&self) -> [Option<RobotJugador>; 2] {
        [Some(self.robot_j1()), self.robot_j2]
    }

    /// Asigna `robot` a `jugador` con la regla "un robot, un jugador".
    fn asignar(&mut self, jugador: Jugador, robot: RobotJugador) {
        let antes = (self.robot_j1(), self.robot_j2);
        let despues = jugadores::asignar(antes.0, antes.1, jugador, robot);
        self.reasignar(antes, despues);
    }

    /// Aplica las asignaciones nuevas. Con el modo manual activo, cada robot que se queda
    /// sin jugador recibe un comando cero y el jugador que cambió de robot arranca con la
    /// rampa en cero. Fuera del modo manual no se envía nada: un cero marcaría al robot
    /// como intervenido (aborta una rutina).
    fn reasignar(&mut self, antes: (RobotJugador, Option<RobotJugador>), despues: (RobotJugador, Option<RobotJugador>)) {
        if self.manual_enabled {
            for robot in jugadores::soltados(antes, despues) {
                self.enviar_cero(robot);
            }
            if antes.0 != despues.0 {
                self.jugadores[Jugador::Uno.indice()].soltar();
            }
            if antes.1 != despues.1 {
                self.jugadores[Jugador::Dos.indice()].soltar();
            }
        }
        if despues.0 != antes.0 {
            self.selected_team = despues.0.0.max(0) as u32;
            self.selected_robot = despues.0.1.max(0) as u32;
            self.clear_telemetry();
        }
        self.robot_j2 = despues.1;
    }

    /// Un `ManualCommand` cero para `robot`, en el mismo `update`.
    fn enviar_cero(&self, (team, id): RobotJugador) {
        if let Some(tx) = &self.manual_tx {
            let _ = tx.try_send(ManualCommand { team, id, vx: 0.0, vy: 0.0, omega: 0.0 });
        }
    }

    /// El coach está en pausa (solo si hay interruptor).
    fn coach_en_pausa(&self) -> bool {
        self.coach_pausa.as_ref().is_some_and(|f| f.load(Ordering::Relaxed))
    }

    /// Hay un coach activo que decide (con interruptor y sin pausa).
    fn coach_activo(&self) -> bool {
        self.coach_pausa.as_ref().is_some_and(|f| !f.load(Ordering::Relaxed))
    }

    fn alternar_coach(&mut self) {
        if let Some(f) = &self.coach_pausa {
            f.fetch_xor(true, Ordering::Relaxed);
        }
    }

    /// Una tecla, un "soltar", Shift o la pérdida de foco.
    fn teclado(&mut self, evento: EventoTeclado) {
        match evento {
            EventoTeclado::Apretada(t) => match keys::ruta_tecla(t, self.manual_enabled) {
                AccionTecla::Parada => {
                    self.estop.store(true, Ordering::Relaxed);
                    for j in &mut self.jugadores {
                        j.soltar();
                    }
                }
                AccionTecla::SalirManual => self.set_manual(false),
                AccionTecla::AlternarCoach => {
                    // Por flanco: la autorrepetición no la alterna otra vez.
                    if self.teclas.insert(t) {
                        self.alternar_coach();
                    }
                }
                AccionTecla::Mover { tecla, .. } => {
                    self.teclas.insert(tecla);
                }
                AccionTecla::Giro(jugador) => {
                    if self.teclas.insert(t) {
                        self.intentar_giro(jugador);
                    }
                }
                AccionTecla::Capa(capa) => self.alternar_capa(capa),
                AccionTecla::Nada => {}
            },
            EventoTeclado::Soltada(t) => {
                self.teclas.remove(&t);
            }
            EventoTeclado::Shift(s) => self.shift = s,
            EventoTeclado::FocoPerdido => {
                self.teclas.clear();
                self.shift = false;
                for j in &mut self.jugadores {
                    j.soltar();
                }
            }
        }
    }

    /// Empieza el giro corto del jugador si se puede: con robot visible, en juego y sin
    /// la parada. El sentido sale de la pelota de la foto (su última posición conocida).
    fn intentar_giro(&mut self, jugador: Jugador) {
        let Some((team, id)) = self.robots_jugadores()[jugador.indice()] else {
            return;
        };
        let modo = jugadores::modo_juego(self.snap.mode, self.estop_activa());
        let robot = self.snap.robot(team, id).copied();
        let (Some(robot), Some(pelota)) = (robot.filter(|r| robot_rows::visible(Some(r))), self.snap.ball) else {
            return;
        };
        if modo != jugadores::ModoJuego::Juego {
            return;
        }
        let arco = jugadores::arco_atacado(team, self.run.own_team, self.own_goal);
        let signo = jugadores::sentido_giro(robot.position, pelota.position, arco);
        let m = &crate::params::params().manual;
        self.jugadores[jugador.indice()].iniciar_giro(
            signo * m.spin_rad_s,
            Instant::now(),
            Duration::from_secs_f64(m.spin_s),
        );
    }

    /// Mientras el modo manual está activo, manda el comando de cada jugador con robot,
    /// con su rampa (también cero al soltar las teclas). Si no, streamea la skill de la GUI.
    fn manual_tick(&mut self) {
        if self.manual_enabled && self.manual_tx.is_some() {
            let modo = jugadores::modo_juego(self.snap.mode, self.estop_activa());
            let escalas = Escalas {
                lin: self.manual_lin_ms,
                ang: self.manual_ang_rads,
                paso_lin: self.manual_lin_accel * MANUAL_TICK_DT,
                paso_ang: self.manual_ang_accel * MANUAL_TICK_DT,
            };
            let ahora = Instant::now();
            for jugador in Jugador::AMBOS {
                let Some((team, id)) = self.robots_jugadores()[jugador.indice()] else {
                    continue;
                };
                // θ de la foto aunque el robot no se vea: la misma que usa el lazo.
                let theta = self.snap.robot(team, id).map(|r| r.orientation).unwrap_or(0.0);
                let intencion = jugadores::intencion(&self.teclas, jugador);
                let (vx, vy, omega) =
                    self.jugadores[jugador.indice()].paso(intencion, &escalas, theta, ahora, modo);
                if let Some(tx) = &self.manual_tx {
                    let _ = tx.try_send(ManualCommand { team, id, vx, vy, omega });
                }
            }
        }
        let robot = self
            .snap
            .robot(self.selected_team as i32, self.selected_robot as i32)
            .copied();
        if !self.manual_enabled
            && let (Some(skill), Some(tx)) = (self.active_skill, &self.skill_tx)
        {
            // Target efectivo: Spin elige el sentido relativo al robot,
            // BlockLine/GoalKeep usan el arco propio y el resto el click.
            let robot_x_m = robot.map(|r| r.position.x).unwrap_or(0.0);
            let target = effective_skill_target(skill, self.skill_target, robot_x_m, self.own_goal);
            let _ = tx.try_send(GuiSkillCommand {
                team: self.selected_team as i32,
                id: self.selected_robot as i32,
                skill_id: skill,
                target,
                spin_omega: self.spin_omega,
            });
        }
    }

    /// Agrega a las series del robot seleccionado lo pedido y lo medido de la foto.
    fn registrar_series(&mut self) {
        let snap = self.snap.clone();
        let t = snap.t_ms as f64 / 1000.0;
        let (team, id) = (self.selected_team as i32, self.selected_robot as i32);
        let robot = snap
            .robot(team, id)
            .filter(|r| robot_rows::visible(Some(*r)));
        let propio = (team == self.run.own_team)
            .then(|| snap.own_robot(id))
            .flatten();
        let cmd = propio.and_then(|o| o.command);
        push_capped(
            &mut self.serie_v,
            MuestraSenal {
                t,
                pedida: cmd.map(|c| f32::from(c.v_mm_s) / 1000.0),
                medida: robot.map(|r| r.forward_speed()),
            },
        );
        push_capped(
            &mut self.serie_w,
            MuestraSenal {
                t,
                pedida: cmd.map(|c| f32::from(c.w_deg_s).to_radians()),
                medida: robot.map(|r| r.angular_velocity as f32),
            },
        );
        if let (Some(target), Some(r)) = (propio.and_then(|o| o.target), robot) {
            let al_target = target - r.position;
            let error = wrap_angle(al_target.y.atan2(al_target.x) - r.orientation as f32);
            push_capped(&mut self.skill_err_history, (t, error, al_target.length()));
        }
        self.clear_chart_caches();
    }

    /// Cuadros de visión por segundo (con el contador monótono de la foto).
    fn registrar_cuadros(&mut self) {
        let (seg, cuadros) = (self.snap.t_ms / 1000, self.snap.vision.frames);
        match self.cuadros_base {
            Some((s0, c0)) if seg > s0 => {
                self.cuadros_hist.push_back((s0, cuadros.saturating_sub(c0)));
                while self.cuadros_hist.len() > 60 {
                    self.cuadros_hist.pop_front();
                }
                self.cuadros_base = Some((seg, cuadros));
                self.cache_cuadros.clear();
            }
            Some((s0, _)) if seg >= s0 => {}
            // Primera foto, o el lazo arrancó de nuevo.
            _ => self.cuadros_base = Some((seg, cuadros)),
        }
    }

    fn clear_chart_caches(&self) {
        self.cache_v.clear();
        self.cache_w.clear();
        self.cache_err.clear();
    }

    /// Reinicia las series (al cambiar de robot o de equipo).
    fn clear_telemetry(&mut self) {
        self.serie_v.clear();
        self.serie_w.clear();
        self.skill_err_history.clear();
        self.clear_chart_caches();
    }

    /// Envía las ganancias PID actuales al lazo (si hay canal).
    fn send_pid(&self) {
        if let Some(tx) = &self.pid_tx {
            let _ = tx.try_send(HeadingPid {
                kp: self.pid_kp,
                ki: self.pid_ki,
                kd: self.pid_kd,
            });
        }
    }

    /// Serializa el tuning actual al archivo `preset_name` (JSON).
    fn save_preset(&self) -> Result<(), String> {
        let preset = self.preset_actual();
        let json = serde_json::to_string_pretty(&preset).map_err(|e| e.to_string())?;
        std::fs::write(&self.preset_name, json).map_err(|e| e.to_string())
    }

    /// El tuning vigente, como preset.
    fn preset_actual(&self) -> TuningPreset {
        TuningPreset {
            lin_ms: self.manual_lin_ms,
            ang_rads: self.manual_ang_rads,
            lin_accel: self.manual_lin_accel,
            ang_accel: self.manual_ang_accel,
            spin_omega: self.spin_omega,
            kp: self.pid_kp,
            ki: self.pid_ki,
            kd: self.pid_kd,
        }
    }

    /// Carga el tuning desde `preset_name` (no envía el PID; el caller decide). Pasa por
    /// la misma validación que los campos, con las escalas recortadas al tope.
    fn load_preset(&mut self) -> Result<(), String> {
        let data = std::fs::read_to_string(&self.preset_name).map_err(|e| e.to_string())?;
        let leido: TuningPreset = serde_json::from_str(&data).map_err(|e| e.to_string())?;
        self.aplicar_preset(&leido);
        Ok(())
    }

    fn aplicar_preset(&mut self, leido: &TuningPreset) {
        let p = preset_valido(leido, &self.preset_actual(), self.tope_lin, self.tope_ang);
        self.manual_lin_ms = p.lin_ms;
        self.manual_lin_str = format!("{}", p.lin_ms);
        self.manual_ang_rads = p.ang_rads;
        self.manual_ang_str = format!("{}", p.ang_rads);
        self.manual_lin_accel = p.lin_accel;
        self.manual_lin_accel_str = format!("{}", p.lin_accel);
        self.manual_ang_accel = p.ang_accel;
        self.manual_ang_accel_str = format!("{}", p.ang_accel);
        self.spin_omega = p.spin_omega;
        self.spin_omega_str = format!("{}", p.spin_omega);
        self.pid_kp = p.kp;
        self.pid_kp_str = format!("{}", p.kp);
        self.pid_ki = p.ki;
        self.pid_ki_str = format!("{}", p.ki);
        self.pid_kd = p.kd;
        self.pid_kd_str = format!("{}", p.kd);
    }

    fn subscription(&self) -> Subscription<Message> {
        let rx = self.snapshot_rx.clone();
        // Fotos: siempre la última. Si la GUI se atrasa, el `send` de este stream
        // espera en el ejecutor de iced (nunca en el del lazo) y el `watch` se queda
        // con la foto más nueva.
        let fotos = Subscription::run_with_id(
            "fotos",
            stream::channel(1, move |mut output| async move {
                let receptor = rx.lock().ok().and_then(|mut r| r.take());
                if let Some(rx) = receptor {
                    let entre_fotos = Duration::from_secs_f32(1.0 / GUI_MAX_FPS);
                    reenviar_fotos(rx, &mut output, entre_fotos).await;
                }
                std::future::pending::<()>().await;
            }),
        );
        let tamano = iced::window::resize_events().map(|(_id, size)| Message::Resized(size));
        // Todo el teclado y el foco por una sola suscripción: también ve los eventos que
        // capturó un widget, para no perder nunca un "soltar" (`keys::evento_de`).
        let teclado = iced::event::listen_with(|evento, estado, _ventana| {
            keys::evento_de(&evento, estado == iced::event::Status::Captured).map(Message::Teclado)
        });
        // El tick del manual y de la skill de la GUI solo hace falta mientras hay algo
        // que enviar: cada mensaje redibuja la ventana, y un tick permanente de 30 Hz
        // duplicaba el costo de dibujo (medido con la GUI abierta).
        let manual_tick = if self.manual_enabled || self.active_skill.is_some() {
            iced::time::every(Duration::from_millis(33)).map(|_| Message::ManualTick)
        } else {
            Subscription::none()
        };
        Subscription::batch([fotos, tamano, teclado, manual_tick])
    }

    fn vista_cancha(&self) -> Element<'_, Message> {
        let canvas = Canvas::new(FieldCanvas {
            snap: self.snap.as_ref(),
            base: &self.cache_base,
            fieltro: self.fieltro,
            orientacion: self.orientacion,
            marcas: self.marcas,
            own_team: self.run.own_team,
            selected: Some((self.selected_team as i32, self.selected_robot as i32)),
            skill_target: self
                .active_skill
                .and_then(|s| skill_marker(s, self.skill_target, self.own_goal)),
            radio_slots: self.run.radio_slots(),
            capas: &self.capas,
            trazas: &self.trazas,
            sim: !self.run.is_base_station(),
            rutina: self.dibujo_rutina(),
            modo_manual: self.manual_enabled,
            jugador2: self.robot_j2,
            num_robots: self.run.num_robots,
        })
        .width(Length::Fill)
        .height(Length::Fill);
        // La barra de capas va sobre la cancha, arriba a la izquierda (mockup).
        let barra = container(capas::barra_capas(&self.capas)).padding(theme::SP_SM);
        container(iced::widget::stack![canvas, barra])
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
    }

    /// Panel derecho: barra de pestañas y la pestaña activa (con scroll).
    fn vista_panel(&self) -> Element<'_, Message> {
        // Cinco pestañas en 300 px: espaciado y margen chicos, texto de 12 px.
        let mut barra = row![].spacing(BARRA_PESTANAS_SP).align_y(Alignment::End);
        for t in Tab::VISIBLES {
            let activa = t == self.tab;
            let boton = button(
                text(t.titulo())
                    .font(fonts::TITULO_MEDIO)
                    .size(theme::TXT_PESTANA),
            )
            .padding([4, 0])
            .style(theme::pestana(activa))
            .on_press(Message::SelectTab(t));
            // El subrayado toma el ancho del botón (va después en la columna).
            let marca: Element<'_, Message> = if activa {
                horizontal_rule(2).style(theme::subrayado).into()
            } else {
                Space::with_height(Length::Fixed(2.0)).into()
            };
            barra = barra.push(column![boton, marca].width(Length::Shrink));
        }
        let contenido = match self.tab {
            Tab::Herramientas => self.pestana_herramientas(),
            Tab::Inspector => self.pestana_inspector(),
            Tab::Rutinas => self.pestana_rutinas(),
            Tab::Parametros => self.pestana_parametros(),
            Tab::PorQue => self.pestana_por_que(),
            Tab::Arbitro => text("Llega en una fase posterior.").into(),
        };
        container(
            column![
                container(barra).padding([8.0, BARRA_PESTANAS_MARGEN]),
                horizontal_rule(1).style(theme::filete),
                scrollable(container(contenido).padding([10, 14]).width(Length::Fill))
                    .height(Length::Fill),
            ]
            .height(Length::Fill),
        )
        .width(Length::Fill)
        .height(Length::Fill)
        .style(theme::panel)
        .into()
    }

    fn view(&self) -> Element<'_, Message> {
        let sep = theme::SP_MD;
        if self.ancho < ANCHO_ANGOSTO {
            // Una columna con scroll, la cancha primero; alturas fijas (dentro de un
            // scroll nada puede llenar el alto).
            let alto_cancha = (self.alto * 0.7).max(320.0);
            let alto_robots = 90.0 + 140.0 * self.run.num_robots as f32;
            let contenido = column![
                self.vista_estado(),
                container(self.vista_cancha()).height(Length::Fixed(alto_cancha)),
                container(self.vista_robots()).height(Length::Fixed(alto_robots)),
                container(self.vista_panel()).height(Length::Fixed(640.0)),
                container(self.vista_graficos()).height(Length::Fixed(ALTO_GRAFICOS)),
            ]
            .spacing(sep)
            .padding(sep);
            return scrollable(contenido).height(Length::Fill).into();
        }
        column![
            self.vista_estado(),
            row![
                container(self.vista_robots())
                    .width(Length::Fixed(ANCHO_ROBOTS))
                    .height(Length::Fill),
                column![
                    row![
                        self.vista_cancha(),
                        container(self.vista_panel())
                            .width(Length::Fixed(ANCHO_PANEL))
                            .height(Length::Fill),
                    ]
                    .spacing(sep)
                    .height(Length::Fill),
                    container(self.vista_graficos()).height(Length::Fixed(ALTO_GRAFICOS)),
                ]
                .spacing(sep)
                .width(Length::Fill)
                .height(Length::Fill),
            ]
            .spacing(sep)
            .height(Length::Fill),
        ]
        .spacing(sep)
        .padding(sep)
        .into()
    }
}

/// Reenvía a la GUI cada foto nueva, como máximo una cada `entre_fotos`, y avisa con
/// `Message::LazoTerminado` cuando el lazo termina (se suelta el emisor de fotos).
async fn reenviar_fotos(
    mut rx: SnapshotReceiver,
    output: &mut iced::futures::channel::mpsc::Sender<Message>,
    entre_fotos: Duration,
) {
    while rx.changed().await.is_ok() {
        // Clonar el `Arc` suelta el lock del canal al instante.
        let foto = rx.borrow_and_update().clone();
        if output.send(Message::Snapshot(foto)).await.is_err() {
            return;
        }
        tokio::time::sleep(entre_fotos).await;
    }
    let _ = output.send(Message::LazoTerminado).await;
}

pub fn run_gui(setup: GuiSetup) -> iced::Result {
    let mut app = iced::application(DebugGui::title, DebugGui::update, DebugGui::view)
        .subscription(DebugGui::subscription)
        .theme(DebugGui::theme)
        .settings(iced::Settings {
            default_text_size: theme::TXT.into(),
            ..iced::Settings::default()
        })
        .default_font(fonts::TEXTO)
        // Sin MSAA: en WSLg, wgpu dibuja con Vulkan por software (lavapipe) y el MSAA
        // multiplicaba el costo de cada cuadro (medido con la GUI abierta).
        .antialiasing(false)
        .window_size(VENTANA_INICIAL);
    for bytes in fonts::TODAS {
        app = app.font(bytes);
    }
    app.run_with(move || DebugGui::new(setup))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_five_tabs_fit_in_the_panel() {
        assert_eq!(Tab::VISIBLES[2], Tab::Rutinas, "Rutinas va después de Inspector");
        let titulos: f32 = Tab::VISIBLES
            .iter()
            .map(|t| field::ancho_texto(t.titulo(), theme::TXT_PESTANA))
            .sum();
        let total = titulos
            + BARRA_PESTANAS_SP * (Tab::VISIBLES.len() - 1) as f32
            + 2.0 * BARRA_PESTANAS_MARGEN;
        assert!(total <= ANCHO_PANEL, "la barra mide {total} px");
    }

    #[tokio::test]
    async fn the_gui_hears_when_the_loop_ends() {
        use iced::futures::StreamExt;
        let (tx, rx) = crate::snapshot::snapshot_channel();
        crate::snapshot::publish(&tx, LoopSnapshot { tick: 7, ..LoopSnapshot::default() });
        drop(tx);
        let (mut output, entrada) = iced::futures::channel::mpsc::channel(4);
        reenviar_fotos(rx, &mut output, Duration::ZERO).await;
        drop(output);
        let mensajes: Vec<Message> = entrada.collect().await;
        assert!(
            matches!(mensajes.as_slice(), [Message::Snapshot(f), Message::LazoTerminado] if f.tick == 7),
            "la última foto y después el aviso"
        );
    }

    /// Una GUI de prueba, con el canal manual y el interruptor del coach.
    fn gui_de_prueba() -> (DebugGui, mpsc::Receiver<ManualCommand>, Arc<AtomicBool>) {
        let (g, manual_rx, _skills, pausa) = gui_con_skills();
        (g, manual_rx, pausa)
    }

    /// Como `gui_de_prueba`, con el canal de las skills de la GUI.
    fn gui_con_skills()
    -> (DebugGui, mpsc::Receiver<ManualCommand>, mpsc::Receiver<GuiSkillCommand>, Arc<AtomicBool>) {
        let (_fotos, snapshot_rx) = crate::snapshot::snapshot_channel();
        let (manual_tx, manual_rx) = mpsc::channel(64);
        let (skill_tx, skill_rx) = mpsc::channel(64);
        let pausa = Arc::new(AtomicBool::new(false));
        let setup = GuiSetup {
            snapshot_rx,
            manual_tx: Some(manual_tx),
            skill_tx: Some(skill_tx),
            pid_tx: None,
            teleport_tx: None,
            rutina_tx: None,
            estop: Arc::new(AtomicBool::new(false)),
            coach_pausa: Some(pausa.clone()),
            run: run_info::run_info_de_prueba(),
        };
        (DebugGui::new(setup).0, manual_rx, skill_rx, pausa)
    }

    fn recibidos(rx: &mut mpsc::Receiver<ManualCommand>) -> Vec<ManualCommand> {
        std::iter::from_fn(|| rx.try_recv().ok()).collect()
    }

    fn tecla(g: &mut DebugGui, t: Tecla) {
        let _ = g.update(Message::Teclado(EventoTeclado::Apretada(t)));
    }

    fn es_cero(c: &ManualCommand) -> bool {
        c.vx == 0.0 && c.vy == 0.0 && c.omega == 0.0
    }

    #[test]
    fn leaving_manual_sends_one_zero_per_driven_robot() {
        let (mut g, mut rx, _) = gui_de_prueba();
        let _ = g.update(Message::ToggleManual(true));
        let _ = g.update(Message::ClicRobot { id: 1, shift: true });
        assert_eq!(g.robot_j2, Some((0, 1)));
        recibidos(&mut rx);
        tecla(&mut g, Tecla::Escape);
        assert!(!g.manual_enabled);
        let ceros = recibidos(&mut rx);
        let ids: Vec<i32> = ceros.iter().map(|c| c.id).collect();
        assert_eq!(ids, [0, 1], "un cero por robot manejado, sin un ManualTick");
        assert!(ceros.iter().all(es_cero));
        assert_eq!(g.robot_j2, Some((0, 1)), "las asignaciones quedan");

        let _ = g.update(Message::ToggleManual(true));
        let _ = g.update(Message::ToggleManual(false));
        assert_eq!(recibidos(&mut rx).len(), 2, "con el botón, lo mismo");
    }

    #[test]
    fn changing_robot_inside_manual_zeroes_the_previous_one() {
        let (mut g, mut rx, _) = gui_de_prueba();
        let _ = g.update(Message::ToggleManual(true));
        let _ = g.update(Message::SelectRow(2));
        let c = recibidos(&mut rx);
        assert_eq!(c.len(), 1);
        assert!(c[0].id == 0 && es_cero(&c[0]), "{c:?}");
        assert_eq!(g.selected_robot, 2);
        let _ = g.update(Message::ClicRobot { id: 1, shift: true });
        let _ = g.update(Message::SoltarJugador2);
        let c = recibidos(&mut rx);
        assert!(c.len() == 1 && c[0].id == 1 && es_cero(&c[0]), "soltar al jugador 2: {c:?}");
        assert_eq!(g.robot_j2, None);
    }

    #[test]
    fn outside_manual_selection_sends_nothing() {
        let (mut g, mut rx, _) = gui_de_prueba();
        let _ = g.update(Message::ToggleManual(true));
        let _ = g.update(Message::ClicRobot { id: 1, shift: true });
        let _ = g.update(Message::ToggleManual(false));
        recibidos(&mut rx);
        for m in [
            Message::SelectRow(2),
            Message::SelectRobot(0),
            Message::SelectTeam(1),
            Message::SelectTeam(0),
            Message::SoltarJugador2,
            Message::ClicRobot { id: 2, shift: false },
        ] {
            let _ = g.update(m);
        }
        assert!(recibidos(&mut rx).is_empty(), "un cero marcaría al robot como intervenido");
        assert_eq!(g.robot_j2, None, "soltar sí cambia la asignación");
        assert_eq!(g.selected_robot, 0, "el click en la cancha fuera del manual no selecciona");
    }

    #[test]
    fn shift_assigns_player_two_and_focus_loss_releases_it() {
        let (mut g, _rx, _) = gui_de_prueba();
        let _ = g.update(Message::ToggleManual(true));
        let _ = g.update(Message::Teclado(EventoTeclado::Shift(true)));
        let _ = g.update(Message::SelectRow(1));
        assert_eq!((g.selected_robot, g.robot_j2), (0, Some((0, 1))), "Shift y click en la fila");
        let _ = g.update(Message::SelectRow(0));
        assert_eq!(g.robot_j2, Some((0, 1)), "Shift sobre el robot del jugador 1: nada");
        let _ = g.update(Message::Teclado(EventoTeclado::FocoPerdido));
        let _ = g.update(Message::SelectRow(2));
        assert_eq!((g.selected_robot, g.robot_j2), (2, Some((0, 1))), "sin Shift: el jugador 1");
        let _ = g.update(Message::SelectRow(1));
        assert_eq!((g.selected_robot, g.robot_j2), (1, None), "el 1 le quita el robot al 2");
    }

    #[test]
    fn both_players_drive_at_once() {
        let (mut g, mut rx, _) = gui_de_prueba();
        let _ = g.update(Message::ToggleManual(true));
        let _ = g.update(Message::ClicRobot { id: 2, shift: true });
        recibidos(&mut rx);
        tecla(&mut g, Tecla::Letra('w'));
        tecla(&mut g, Tecla::Flecha(keys::Flecha::Arriba));
        let _ = g.update(Message::ManualTick);
        let c = recibidos(&mut rx);
        assert_eq!(c.iter().map(|c| c.id).collect::<Vec<_>>(), [0, 2]);
        assert!(c.iter().all(|c| c.vx > 0.0 && c.omega == 0.0), "los dos avanzan: {c:?}");
        let _ = g.update(Message::Teclado(EventoTeclado::Soltada(Tecla::Letra('w'))));
        for _ in 0..30 {
            let _ = g.update(Message::ManualTick);
        }
        let ultimos = recibidos(&mut rx);
        let (j1, j2): (Vec<_>, Vec<_>) = ultimos.iter().partition(|c| c.id == 0);
        assert!(es_cero(j1.last().unwrap()), "el 1 soltó y frenó");
        assert!(j2.last().unwrap().vx > 0.0, "el 2 sigue");
    }

    #[test]
    fn the_stop_resets_both_ramps() {
        let (mut g, mut rx, _) = gui_de_prueba();
        let _ = g.update(Message::ToggleManual(true));
        let _ = g.update(Message::ClicRobot { id: 1, shift: true });
        tecla(&mut g, Tecla::Letra('w'));
        tecla(&mut g, Tecla::Flecha(keys::Flecha::Arriba));
        for _ in 0..30 {
            let _ = g.update(Message::ManualTick);
        }
        tecla(&mut g, Tecla::Espacio);
        assert!(g.estop_activa());
        let _ = g.update(Message::ManualTick);
        let _ = g.update(Message::ToggleEstop);
        recibidos(&mut rx);
        let _ = g.update(Message::ManualTick);
        let c = recibidos(&mut rx);
        let paso = g.manual_lin_accel * MANUAL_TICK_DT;
        assert_eq!(c.iter().map(|c| c.id).collect::<Vec<_>>(), [0, 1]);
        assert!(c.iter().all(|c| (c.vx - paso).abs() < 1e-9), "al soltar la parada, los dos desde cero: {c:?}");
    }

    #[test]
    fn focus_loss_releases_keys_ramps_and_the_spin() {
        use crate::control_loop::TickMode;
        let (mut g, mut rx, _) = gui_de_prueba();
        let _ = g.update(Message::ToggleManual(true));
        let _ = g.update(Message::ClicRobot { id: 1, shift: true });
        tecla(&mut g, Tecla::Flecha(keys::Flecha::Arriba));
        for _ in 0..10 {
            let _ = g.update(Message::ManualTick);
        }
        assert!(recibidos(&mut rx).iter().any(|c| c.id == 1 && c.vx > 0.0));
        g.snap = foto_giro(TickMode::Play, 0.0);
        tecla(&mut g, Tecla::Letra('e'));
        assert!(g.jugadores[0].girando());
        let _ = g.update(Message::Teclado(EventoTeclado::Shift(true)));
        let _ = g.update(Message::Teclado(EventoTeclado::FocoPerdido));
        assert!(g.teclas.is_empty() && !g.shift);
        assert!(!g.jugadores[0].girando(), "el giro se corta");
        for _ in 0..3 {
            let _ = g.update(Message::ManualTick);
            let c = recibidos(&mut rx);
            assert!(c.iter().all(es_cero), "sin avance hasta volver a apretar: {c:?}");
        }
    }

    #[test]
    fn leaving_manual_resumes_the_gui_skill_only_on_player_one() {
        let (mut g, mut rx, mut skills, _) = gui_con_skills();
        let _ = g.update(Message::SelectSkill(Some(SkillId::GoTo)));
        let _ = g.update(Message::ManualTick);
        assert_eq!(std::iter::from_fn(|| skills.try_recv().ok()).count(), 1, "la skill corre");
        let _ = g.update(Message::ToggleManual(true));
        let _ = g.update(Message::ClicRobot { id: 1, shift: true });
        let _ = g.update(Message::ManualTick);
        assert_eq!(std::iter::from_fn(|| skills.try_recv().ok()).count(), 0, "en el modo manual no se envía");
        recibidos(&mut rx);
        tecla(&mut g, Tecla::Escape);
        let ceros = recibidos(&mut rx);
        assert_eq!(ceros.iter().map(|c| c.id).collect::<Vec<_>>(), [0, 1], "primero los ceros");
        let _ = g.update(Message::ManualTick);
        let enviadas: Vec<GuiSkillCommand> = std::iter::from_fn(|| skills.try_recv().ok()).collect();
        assert_eq!(enviadas.iter().map(|s| s.id).collect::<Vec<_>>(), [0], "solo el robot del jugador 1");
    }

    #[test]
    fn p_toggles_the_coach_once_per_press() {
        let (mut g, _rx, pausa) = gui_de_prueba();
        for _ in 0..5 {
            tecla(&mut g, Tecla::Letra('p'));
        }
        assert!(pausa.load(Ordering::Relaxed), "la autorrepetición no la alterna otra vez");
        let _ = g.update(Message::Teclado(EventoTeclado::Soltada(Tecla::Letra('p'))));
        tecla(&mut g, Tecla::Letra('p'));
        assert!(!pausa.load(Ordering::Relaxed));
        let _ = g.update(Message::ToggleCoach);
        assert!(g.coach_en_pausa() && !g.coach_activo());
        let _ = g.update(Message::ToggleManual(true));
        let _ = g.update(Message::Teclado(EventoTeclado::Soltada(Tecla::Letra('p'))));
        tecla(&mut g, Tecla::Letra('p'));
        assert!(!g.coach_en_pausa(), "P también dentro del modo manual");
    }

    /// Una foto con el robot 0 visible en el origen y la pelota a su derecha (atacando
    /// hacia +X, porque `run_info_de_prueba` defiende la izquierda).
    fn foto_giro(modo: crate::control_loop::TickMode, edad_s: f32) -> Arc<LoopSnapshot> {
        let robot = crate::snapshot::RobotView {
            team: 0,
            id: 0,
            position: Vec2::ZERO,
            orientation: 0.0,
            velocity: Vec2::ZERO,
            angular_velocity: 0.0,
            active: true,
            age_s: edad_s,
        };
        Arc::new(LoopSnapshot {
            mode: modo,
            robots: vec![robot],
            ball: Some(crate::snapshot::BallView { position: Vec2::new(0.0, -0.06), velocity: Vec2::ZERO }),
            ..LoopSnapshot::default()
        })
    }

    #[test]
    fn short_spin_starts_on_the_press_edge_toward_the_rival_goal() {
        use crate::control_loop::TickMode;
        let (mut g, mut rx, _) = gui_de_prueba();
        let _ = g.update(Message::ToggleManual(true));
        g.snap = foto_giro(TickMode::Play, 0.0);
        tecla(&mut g, Tecla::Letra('e'));
        assert!(g.jugadores[0].girando());
        let _ = g.update(Message::ManualTick);
        let c = recibidos(&mut rx);
        assert_eq!((c[0].vx, c[0].vy), (0.0, 0.0));
        assert_eq!(c[0].omega, crate::params::params().manual.spin_rad_s, "pelota a la derecha: CCW");
        // La autorrepetición no reinicia el pulso.
        let antes = g.jugadores[0].clone();
        tecla(&mut g, Tecla::Letra('e'));
        assert_eq!(g.jugadores[0], antes);
    }

    #[test]
    fn no_short_spin_out_of_play_or_without_the_robot_in_sight() {
        use crate::control_loop::TickMode;
        for (modo, edad, motivo) in [
            (TickMode::Stop, 0.0, "STOP"),
            (TickMode::SinVision, 0.0, "corte sin visión"),
            (TickMode::Play, 2.0, "robot sin visión"),
        ] {
            let (mut g, _rx, _) = gui_de_prueba();
            let _ = g.update(Message::ToggleManual(true));
            g.snap = foto_giro(modo, edad);
            tecla(&mut g, Tecla::Letra('e'));
            assert!(!g.jugadores[0].girando(), "{motivo}");
        }
        let (mut g, _rx, _) = gui_de_prueba();
        let _ = g.update(Message::ToggleManual(true));
        g.snap = foto_giro(TickMode::Play, 0.0);
        tecla(&mut g, Tecla::Espacio);
        tecla(&mut g, Tecla::Letra('e'));
        assert!(!g.jugadores[0].girando(), "con la parada");
        let (mut g, _rx, _) = gui_de_prueba();
        let _ = g.update(Message::ToggleManual(true));
        g.snap = foto_giro(TickMode::Play, 0.0);
        tecla(&mut g, Tecla::Enter);
        assert!(!g.jugadores[1].girando(), "el jugador 2 sin robot");
    }

    #[test]
    fn player_two_spins_with_enter() {
        use crate::control_loop::TickMode;
        let (mut g, mut rx, _) = gui_de_prueba();
        let _ = g.update(Message::ToggleManual(true));
        let _ = g.update(Message::SelectRow(2));
        let _ = g.update(Message::ClicRobot { id: 0, shift: true });
        assert_eq!(g.robot_j2, Some((0, 0)));
        g.snap = foto_giro(TickMode::Play, 0.0);
        recibidos(&mut rx);
        tecla(&mut g, Tecla::Enter);
        assert!(g.jugadores[1].girando() && !g.jugadores[0].girando());
        let _ = g.update(Message::ManualTick);
        let c = recibidos(&mut rx);
        let j2 = c.iter().find(|c| c.id == 0).expect("el robot del jugador 2");
        assert_eq!((j2.vx, j2.vy), (0.0, 0.0));
        assert_eq!(j2.omega, crate::params::params().manual.spin_rad_s, "pelota a la derecha: CCW");
    }

    #[test]
    fn manual_scales_come_from_params_with_a_ceiling() {
        let (mut g, _rx, _) = gui_de_prueba();
        let tope = crate::params::params().manual.velocidad_sim_m_s;
        assert_eq!((g.tope_lin, g.manual_lin_ms), (tope, tope), "FIRASim: el tope de simulación");
        let _ = g.update(Message::ManualLinChanged("0.2".to_string()));
        assert_eq!(g.manual_lin_ms, 0.2);
        let _ = g.update(Message::ManualLinChanged(format!("{}", tope + 0.3)));
        assert_eq!(g.manual_lin_ms, 0.2, "por encima del tope se ignora");
        let _ = g.update(Message::ManualLinChanged("x".to_string()));
        assert_eq!(g.manual_lin_ms, 0.2);
        assert_eq!(escala_valida("0.3", 0.3), Some(0.3));
        assert_eq!(escala_valida("-0.1", 0.3), None);
        assert_eq!(escala_valida("NaN", 0.3), None);
    }

    #[test]
    fn presets_are_validated_and_clamped() {
        let actual = TuningPreset {
            lin_ms: 0.2,
            ang_rads: 2.0,
            lin_accel: 2.0,
            ang_accel: 12.0,
            spin_omega: 20.0,
            kp: 3.0,
            ki: 0.08,
            kd: 0.2,
        };
        let de_sim = TuningPreset { lin_ms: 0.5, ang_rads: 4.0, ..actual.clone() };
        let p = preset_valido(&de_sim, &actual, 0.3, 3.0);
        assert_eq!((p.lin_ms, p.ang_rads), (0.3, 3.0), "recortado al tope");
        let malo = TuningPreset {
            lin_ms: -1.0,
            ang_rads: f64::NAN,
            lin_accel: 0.0,
            ang_accel: -3.0,
            spin_omega: 0.0,
            kp: -1.0,
            ki: f64::INFINITY,
            kd: 0.5,
        };
        let p = preset_valido(&malo, &actual, 0.3, 3.0);
        assert_eq!(p, TuningPreset { kd: 0.5, ..actual }, "lo inválido conserva lo anterior");
    }

    #[test]
    fn none_skill_text_follows_the_pause() {
        assert_eq!(texto_ninguna(false).0, "ninguna (coach)");
        assert_eq!(texto_ninguna(true).0, "ninguna (quieto)");
        assert!(texto_ninguna(true).1.contains("queda quieto"));
    }

    #[test]
    fn warning_when_a_player_drives_the_coach_keeper() {
        let j = [Some((0, 0)), Some((0, 2))];
        assert_eq!(
            aviso_arquero_manual(&j, 0, 2, true).as_deref(),
            Some("el robot 2 es el arquero del coach: el arco queda solo")
        );
        assert_eq!(aviso_arquero_manual(&j, 0, 2, false), None, "con el coach en pausa no");
        assert_eq!(aviso_arquero_manual(&[Some((0, 0)), None], 0, 2, true), None);
        assert_eq!(aviso_arquero_manual(&[Some((1, 2)), None], 0, 2, true), None, "rival");
    }

    /// 3.5 — La rampa no sobrepasa, converge y decae hacia cero.
    #[test]
    fn slew_ramps_without_overshoot() {
        // No sobrepasa el objetivo.
        assert_eq!(slew(0.0, 1.0, 0.3), 0.3);
        // Si el paso alcanza, aterriza exacto en el objetivo.
        assert_eq!(slew(0.9, 1.0, 0.3), 1.0);
        // Converge en pasos finitos.
        let mut v = 0.0;
        for _ in 0..100 {
            v = slew(v, 1.0, 0.1);
        }
        assert!((v - 1.0).abs() < 1e-9);
        // Decae hacia cero (frenado).
        assert_eq!(slew(1.0, 0.0, 0.3), 0.7);
    }

    /// Spin en el runner: el signo del target depende del lado del robot.
    #[test]
    fn spin_relative_sign_by_side() {
        // Click a la derecha del robot (x mayor) → +1 (CCW).
        assert_eq!(spin_relative_target(0.5, 0.2).x, 1.0);
        // Click a la izquierda (x menor) → -1 (CW).
        assert_eq!(spin_relative_target(-0.5, 0.2).x, -1.0);
        // Empate → +1 determinista (siempre gira).
        assert_eq!(spin_relative_target(0.2, 0.2).x, 1.0);
        // y siempre 0 (giro puro).
        assert_eq!(spin_relative_target(0.5, 0.0).y, 0.0);
    }

    /// Cada skill del catálogo tiene exactamente un botón y texto de ayuda: una
    /// skill que se agregue a `SkillId` sin botón en la GUI rompe este test.
    #[test]
    fn every_catalog_skill_has_button_and_help() {
        let buttons: Vec<SkillId> = SKILL_GROUPS
            .iter()
            .flat_map(|(_, skills)| skills.iter().copied())
            .collect();
        for n in 0..SkillId::COUNT as u8 {
            let id = SkillId::from_u8(n).expect("id válido");
            let count = buttons.iter().filter(|&&b| b == id).count();
            assert_eq!(
                count, 1,
                "{id:?} tiene {count} botones en SKILL_GROUPS (debe tener 1)"
            );
            assert!(
                !skill_help(Some(id)).is_empty(),
                "{id:?} sin texto de ayuda"
            );
        }
        assert_eq!(buttons.len(), SkillId::COUNT);
        assert!(!skill_help(None).is_empty());
    }

    /// BlockLine/GoalKeep mandan el arco propio, Spin el sentido y el resto el click.
    #[test]
    fn effective_target_per_skill() {
        let own_goal = Vec2::new(-0.75, 0.0);
        let click = Vec2::new(0.3, -0.2);
        for id in [SkillId::BlockLine, SkillId::GoalKeep] {
            assert_eq!(effective_skill_target(id, click, 0.0, own_goal), own_goal);
            assert_eq!(
                effective_skill_target(id, Vec2::new(0.7, 0.5), 0.0, own_goal),
                own_goal
            );
        }
        assert_eq!(
            effective_skill_target(SkillId::Spin, click, 0.0, own_goal),
            Vec2::new(1.0, 0.0)
        );
        assert_eq!(
            effective_skill_target(SkillId::Spin, click, 0.5, own_goal),
            Vec2::new(-1.0, 0.0)
        );
        for id in [SkillId::GoTo, SkillId::ShootPush, SkillId::Mark] {
            assert_eq!(effective_skill_target(id, click, 0.0, own_goal), click);
        }
    }

    /// El marcador muestra el target efectivo y nada en las skills que lo ignoran.
    #[test]
    fn skill_marker_shows_effective_target() {
        let own_goal = Vec2::new(0.75, 0.0);
        let click = Vec2::new(-0.1, 0.4);
        for id in [SkillId::ChaseBall, SkillId::Intercept, SkillId::Hold] {
            assert_eq!(skill_marker(id, click, own_goal), None, "{id:?}");
        }
        for id in [SkillId::BlockLine, SkillId::GoalKeep] {
            assert_eq!(skill_marker(id, click, own_goal), Some(own_goal), "{id:?}");
        }
        for id in [SkillId::GoTo, SkillId::Spin, SkillId::SpinKick] {
            assert_eq!(skill_marker(id, click, own_goal), Some(click), "{id:?}");
        }
    }

    #[test]
    fn own_goal_center_by_side() {
        assert_eq!(own_goal_center(true), Vec2::new(-0.75, 0.0));
        assert_eq!(own_goal_center(false), Vec2::new(0.75, 0.0));
    }

    /// El aviso aparece solo con GoalKeep en un robot que no es el arquero.
    #[test]
    fn keeper_warning_only_for_goalkeep_off_keeper() {
        let w = keeper_warning(Some(SkillId::GoalKeep), 1, 2).expect("robot 1 no es el arquero");
        assert!(w.contains("keeper_id = 2"), "{w}");
        assert_eq!(keeper_warning(Some(SkillId::GoalKeep), 2, 2), None);
        assert_eq!(keeper_warning(Some(SkillId::BlockLine), 1, 2), None);
        assert_eq!(keeper_warning(None, 1, 2), None);
    }

    /// Round-trip del preset de tuning (serialize → deserialize preserva valores).
    #[test]
    fn tuning_preset_round_trip() {
        let p = TuningPreset {
            lin_ms: 0.6,
            ang_rads: 4.0,
            lin_accel: 2.5,
            ang_accel: 15.0,
            spin_omega: 25.0,
            kp: 3.5,
            ki: 0.1,
            kd: 0.25,
        };
        let json = serde_json::to_string(&p).unwrap();
        let q: TuningPreset = serde_json::from_str(&json).unwrap();
        assert_eq!(q.lin_ms, 0.6);
        assert_eq!(q.spin_omega, 25.0);
        assert_eq!(q.kp, 3.5);
        assert_eq!(q.kd, 0.25);
    }

    /// 4.5 — La ventana de telemetría descarta muestras viejas (misma lógica
    /// que `DebugGui::registrar_series`).
    #[test]
    fn vw_history_respects_window() {
        let mut hist: VecDeque<(f64, i16, i16)> = VecDeque::new();
        for i in 0..(TELEMETRY_HISTORY_MAX + 50) {
            hist.push_back((i as f64, i as i16, -(i as i16)));
            while hist.len() > TELEMETRY_HISTORY_MAX {
                hist.pop_front();
            }
        }
        assert_eq!(hist.len(), TELEMETRY_HISTORY_MAX);
        // Las 50 muestras más viejas fueron descartadas.
        assert!(hist.front().unwrap().0 >= 50.0);
    }
}
