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

mod charts;
mod field;
mod fonts;
mod format;
mod inspector;
mod keys;
#[cfg(test)]
mod render_tests;
mod robot_rows;
pub mod run_info;
mod status_line;
mod tag;
mod theme;
mod tools;

pub use run_info::{CoachKind, RunInfo};

use charts::MuestraSenal;
use field::{Fieltro, FieldCanvas, OrientacionVista};
use keys::{AccionTecla, Layer, Tecla};
use status_line::Salud;
use tag::MarcasRobot;

use crate::control_loop::{GuiSkillCommand, HeadingPid, ManualCommand};
use crate::radio::TeleportItem;
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

/// Velocidad lineal máx (m/s) por defecto del control manual (ajustable en la GUI).
const MANUAL_LIN_MS_DEFAULT: f64 = 0.5;
/// Velocidad angular máx (rad/s) por defecto del control manual (ajustable en la GUI).
/// Convención: giro positivo → CCW (omega > 0), alineada con la skill Spin del catálogo.
const MANUAL_ANG_RADS_DEFAULT: f64 = 3.0;
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
#[derive(Debug, Clone, Serialize, Deserialize)]
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

/// Marco de referencia del control manual.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameMode {
    /// W/S = ±Y mundo, A/D = ±X mundo, Q/E = giro.
    World,
    /// W/S = adelante/atrás según heading, A/D = giro (arcade drive).
    Robot,
}

/// Traduce las teclas presionadas al comando objetivo en **marco mundo**
/// `(vx, vy, omega)`, según el modo de marco, las escalas y la orientación `theta`
/// del robot (rad). Función pura y testeable (θ explícito).
///
/// - `World`: W/S = ±Y mundo, A/D = ±X mundo, Q/E = giro CCW/CW.
/// - `Robot`: W/S = adelante/atrás según heading (`vx=v_fwd·cosθ`, `vy=v_fwd·sinθ`),
///   A/D = giro (arcade drive). Q/E también giran (equivalentes a A/D).
pub fn manual_world_target(
    keys: &HashSet<char>,
    frame: FrameMode,
    lin: f64,
    ang: f64,
    theta: f64,
) -> (f64, f64, f64) {
    let axis = |pos: char, neg: char, scale: f64| -> f64 {
        let mut v = 0.0;
        if keys.contains(&pos) {
            v += scale;
        }
        if keys.contains(&neg) {
            v -= scale;
        }
        v
    };

    match frame {
        FrameMode::World => {
            let vx = axis('d', 'a', lin);
            let vy = axis('w', 's', lin);
            let omega = axis('q', 'e', ang);
            (vx, vy, omega)
        }
        FrameMode::Robot => {
            let v_fwd = axis('w', 's', lin);
            // A/D giran (arcade); Q/E también, sumados.
            let omega = axis('a', 'd', ang) + axis('q', 'e', ang);
            let (c, s) = (theta.cos(), theta.sin());
            (v_fwd * c, v_fwd * s, omega)
        }
    }
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
        ],
    ),
    (
        "Defensa",
        &[
            SkillId::Intercept,
            SkillId::BlockLine,
            SkillId::GoalKeep,
            SkillId::Mark,
        ],
    ),
    ("Quieto", &[SkillId::Hold]),
];

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
        Some(SkillId::Mark) => "click = dónde pararse; queda mirando a la pelota.",
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
        SkillId::BlockLine | SkillId::GoalKeep => own_goal,
        _ => click,
    }
}

/// Dónde dibujar el marcador del target en la cancha: el arco propio en
/// `BlockLine`/`GoalKeep`, el click en las skills que lo usan (incluida `Spin`,
/// que usa su lado) y ninguno en las que lo ignoran (`ChaseBall`, `Intercept`, `Hold`).
pub fn skill_marker(skill: SkillId, click: Vec2, own_goal: Vec2) -> Option<Vec2> {
    match skill {
        SkillId::BlockLine | SkillId::GoalKeep => Some(own_goal),
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
    (skill == Some(SkillId::GoalKeep) && selected_robot as i32 != keeper_id).then(|| {
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
const ALTO_GRAFICOS: f32 = 168.0;

/// Pestañas del panel derecho. La barra muestra solo las implementadas.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tab {
    /// Decisión del coach (fase 2).
    PorQue,
    Inspector,
    /// Comandos del árbitro (fase 4).
    Arbitro,
    Parametros,
    /// Provisional hasta la fase 4: control manual, skills de la GUI y teleport.
    Herramientas,
}

impl Tab {
    /// Las implementadas en esta fase.
    pub const VISIBLES: [Tab; 3] = [Tab::Herramientas, Tab::Inspector, Tab::Parametros];

    pub fn titulo(self) -> &'static str {
        match self {
            Tab::PorQue => "Por qué",
            Tab::Inspector => "Inspector",
            Tab::Arbitro => "Árbitro",
            Tab::Parametros => "Parámetros",
            Tab::Herramientas => "Herramientas",
        }
    }
}

#[derive(Debug, Clone)]
pub enum Message {
    /// Foto del tick (la última publicada por el lazo).
    Snapshot(Arc<LoopSnapshot>),
    /// Nuevo tamaño de la ventana.
    Resized(Size),
    /// Tecla apretada (ver `keys::ruta_tecla`).
    Tecla(Tecla),
    /// Letra soltada.
    TeclaSoltada(char),
    /// Tick rápido que recomputa y envía el comando manual o la skill de la GUI.
    ManualTick,
    SelectTab(Tab),
    /// Click en una fila: un robot propio.
    SelectRow(u32),
    SelectRobot(u32),
    SelectTeam(u32),
    ToggleManual(bool),
    SetFrame(FrameMode),
    ManualLinChanged(String),
    ManualAngChanged(String),
    ManualLinAccelChanged(String),
    ManualAngAccelChanged(String),
    SelectSkill(Option<SkillId>),
    FieldClicked(Vec2),
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
    /// Flag de parada de emergencia compartido con el lazo.
    pub estop: Arc<AtomicBool>,
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
    // Robot seleccionado (herramientas, Inspector, gráficos, marca en la cancha).
    selected_robot: u32,
    selected_team: u32,
    // Control manual.
    manual_enabled: bool,
    manual_frame: FrameMode,
    manual_lin_ms: f64,
    manual_ang_rads: f64,
    manual_lin_str: String,
    manual_ang_str: String,
    manual_lin_accel: f64,
    manual_ang_accel: f64,
    manual_lin_accel_str: String,
    manual_ang_accel_str: String,
    /// Último comando (vx, vy, omega) enviado: base de la rampa de aceleración.
    applied_cmd: (f64, f64, f64),
    keys: HashSet<char>,
    manual_tx: Option<mpsc::Sender<ManualCommand>>,
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
}

impl DebugGui {
    fn new(setup: GuiSetup) -> (Self, Task<Message>) {
        let own_goal = own_goal_center(setup.run.defend_left);
        let selected_team = setup.run.own_team.max(0) as u32;
        (
            DebugGui {
                snap: Arc::new(LoopSnapshot::default()),
                snapshot_rx: Arc::new(Mutex::new(Some(setup.snapshot_rx))),
                salud: Salud::default(),
                fps_gui: FpsGui::new(),
                ancho: VENTANA_INICIAL.0,
                alto: VENTANA_INICIAL.1,
                tab: Tab::Herramientas,
                fieltro: Fieltro::default(),
                orientacion: OrientacionVista::from_env(),
                marcas: MarcasRobot::from_env(),
                cache_base: Cache::default(),
                capas: HashSet::new(),
                selected_robot: 0,
                selected_team,
                manual_enabled: false,
                manual_frame: FrameMode::Robot,
                manual_lin_ms: MANUAL_LIN_MS_DEFAULT,
                manual_ang_rads: MANUAL_ANG_RADS_DEFAULT,
                manual_lin_str: format!("{MANUAL_LIN_MS_DEFAULT}"),
                manual_ang_str: format!("{MANUAL_ANG_RADS_DEFAULT}"),
                manual_lin_accel: MANUAL_LIN_ACCEL,
                manual_ang_accel: MANUAL_ANG_ACCEL,
                manual_lin_accel_str: format!("{MANUAL_LIN_ACCEL}"),
                manual_ang_accel_str: format!("{MANUAL_ANG_ACCEL}"),
                applied_cmd: (0.0, 0.0, 0.0),
                keys: HashSet::new(),
                manual_tx: setup.manual_tx,
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
            Message::Snapshot(snap) => {
                self.fps_gui.registrar(Instant::now());
                self.salud.push(snap.t_ms, snap.vision, snap.radio);
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
            Message::Tecla(t) => match keys::ruta_tecla(t, self.manual_enabled) {
                AccionTecla::Parada => self.estop.store(true, Ordering::Relaxed),
                AccionTecla::SalirManual => self.set_manual(false),
                AccionTecla::Mover(c) => {
                    self.keys.insert(c);
                }
                AccionTecla::Capa(capa) => {
                    if !self.capas.remove(&capa) {
                        self.capas.insert(capa);
                    }
                }
                AccionTecla::Nada => {}
            },
            Message::TeclaSoltada(c) => {
                self.keys.remove(&c);
            }
            Message::ManualTick => self.manual_tick(),
            Message::SelectTab(t) => self.tab = t,
            Message::SelectRow(id) => {
                let team = self.run.own_team.max(0) as u32;
                if (team, id) != (self.selected_team, self.selected_robot) {
                    self.selected_team = team;
                    self.selected_robot = id;
                    self.clear_telemetry();
                }
            }
            Message::SelectRobot(id) => {
                if id != self.selected_robot {
                    self.selected_robot = id;
                    self.clear_telemetry();
                }
            }
            Message::SelectTeam(team) => {
                if team != self.selected_team {
                    self.selected_team = team;
                    self.clear_telemetry();
                }
            }
            Message::ToggleManual(on) => self.set_manual(on),
            Message::SetFrame(f) => self.manual_frame = f,
            Message::ManualLinChanged(s) => {
                if let Ok(v) = s.parse::<f64>()
                    && v.is_finite()
                    && v >= 0.0
                {
                    self.manual_lin_ms = v;
                }
                self.manual_lin_str = s;
            }
            Message::ManualAngChanged(s) => {
                if let Ok(v) = s.parse::<f64>()
                    && v.is_finite()
                    && v >= 0.0
                {
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
                // El click fija el target de la skill activa (si hay).
                if self.active_skill.is_some() {
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
        }
        Task::none()
    }

    /// Entra o sale del modo manual. Al salir se vacían las teclas y la rampa (para no
    /// arrancar con inercia la próxima vez); el lazo frena al robot cuando el comando
    /// manual expira.
    fn set_manual(&mut self, on: bool) {
        self.manual_enabled = on;
        if !on {
            self.keys.clear();
            self.applied_cmd = (0.0, 0.0, 0.0);
        }
    }

    /// Mientras el modo manual está activo, recomputa el objetivo, aplica la rampa y
    /// reenvía (también cero al soltar las teclas). Si no, streamea la skill de la GUI.
    fn manual_tick(&mut self) {
        let robot = self
            .snap
            .robot(self.selected_team as i32, self.selected_robot as i32)
            .copied();
        if self.manual_enabled
            && let Some(tx) = &self.manual_tx
        {
            // Orientación del robot seleccionado (para el marco robot).
            let theta = robot.map(|r| r.orientation).unwrap_or(0.0);
            let (tx_v, ty_v, tw_v) = manual_world_target(
                &self.keys,
                self.manual_frame,
                self.manual_lin_ms,
                self.manual_ang_rads,
                theta,
            );
            let lin_step = self.manual_lin_accel * MANUAL_TICK_DT;
            let ang_step = self.manual_ang_accel * MANUAL_TICK_DT;
            let (cx, cy, co) = self.applied_cmd;
            self.applied_cmd = (
                slew(cx, tx_v, lin_step),
                slew(cy, ty_v, lin_step),
                slew(co, tw_v, ang_step),
            );
            let (vx, vy, omega) = self.applied_cmd;
            let _ = tx.try_send(ManualCommand {
                team: self.selected_team as i32,
                id: self.selected_robot as i32,
                vx,
                vy,
                omega,
            });
        }
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
        let preset = TuningPreset {
            lin_ms: self.manual_lin_ms,
            ang_rads: self.manual_ang_rads,
            lin_accel: self.manual_lin_accel,
            ang_accel: self.manual_ang_accel,
            spin_omega: self.spin_omega,
            kp: self.pid_kp,
            ki: self.pid_ki,
            kd: self.pid_kd,
        };
        let json = serde_json::to_string_pretty(&preset).map_err(|e| e.to_string())?;
        std::fs::write(&self.preset_name, json).map_err(|e| e.to_string())
    }

    /// Carga el tuning desde `preset_name` (no envía el PID; el caller decide).
    fn load_preset(&mut self) -> Result<(), String> {
        let data = std::fs::read_to_string(&self.preset_name).map_err(|e| e.to_string())?;
        let p: TuningPreset = serde_json::from_str(&data).map_err(|e| e.to_string())?;
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
        Ok(())
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
                if let Some(mut rx) = receptor {
                    let entre_fotos = Duration::from_secs_f32(1.0 / GUI_MAX_FPS);
                    while rx.changed().await.is_ok() {
                        // Clonar el `Arc` suelta el lock del canal al instante.
                        let foto = rx.borrow_and_update().clone();
                        if output.send(Message::Snapshot(foto)).await.is_err() {
                            break;
                        }
                        tokio::time::sleep(entre_fotos).await;
                    }
                }
                std::future::pending::<()>().await;
            }),
        );
        let tamano = iced::window::resize_events().map(|(_id, size)| Message::Resized(size));
        let teclas = iced::keyboard::on_key_press(|key, _mods| keys::tecla_de(&key).map(Message::Tecla));
        let sueltas = iced::keyboard::on_key_release(|key, _mods| match keys::tecla_de(&key) {
            Some(Tecla::Letra(c)) => Some(Message::TeclaSoltada(c)),
            _ => None,
        });
        // El tick del manual y de la skill de la GUI solo hace falta mientras hay algo
        // que enviar: cada mensaje redibuja la ventana, y un tick permanente de 30 Hz
        // duplicaba el costo de dibujo (medido con la GUI abierta).
        let manual_tick = if self.manual_enabled || self.active_skill.is_some() {
            iced::time::every(Duration::from_millis(33)).map(|_| Message::ManualTick)
        } else {
            Subscription::none()
        };
        Subscription::batch([fotos, tamano, teclas, sueltas, manual_tick])
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
        })
        .width(Length::Fill)
        .height(Length::Fill);
        container(canvas)
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
    }

    /// Panel derecho: barra de pestañas y la pestaña activa (con scroll).
    fn vista_panel(&self) -> Element<'_, Message> {
        let mut barra = row![].spacing(theme::SP_MD).align_y(Alignment::End);
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
            Tab::Parametros => self.pestana_parametros(),
            Tab::PorQue | Tab::Arbitro => text("Llega en una fase posterior.").into(),
        };
        container(
            column![
                container(barra).padding([8, 12]),
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

    /// 1.4 — Mapeo marco mundo determinista y escalado.
    #[test]
    fn world_frame_mapping_is_deterministic_and_scaled() {
        let lin = MANUAL_LIN_MS_DEFAULT;
        let ang = MANUAL_ANG_RADS_DEFAULT;

        // Avance (W): +Y = lin, sin giro.
        let mut keys = HashSet::new();
        keys.insert('w');
        let (vx, vy, omega) = manual_world_target(&keys, FrameMode::World, lin, ang, 0.0);
        assert_eq!(vx, 0.0);
        assert_eq!(vy, lin);
        assert_eq!(omega, 0.0);

        // Escala distinta afecta la magnitud.
        let (_, vy2, _) = manual_world_target(&keys, FrameMode::World, 1.2, ang, 0.0);
        assert_eq!(vy2, 1.2);

        // Q → CCW (omega > 0).
        let mut k = HashSet::new();
        k.insert('q');
        assert!(manual_world_target(&k, FrameMode::World, lin, ang, 0.0).2 > 0.0);

        // Teclas opuestas se cancelan; sin teclas = freno.
        let mut opp = HashSet::new();
        for c in ['w', 's', 'a', 'd'] {
            opp.insert(c);
        }
        assert_eq!(
            manual_world_target(&opp, FrameMode::World, lin, ang, 0.0),
            (0.0, 0.0, 0.0)
        );
        assert_eq!(
            manual_world_target(&HashSet::new(), FrameMode::World, lin, ang, 0.0),
            (0.0, 0.0, 0.0)
        );
    }

    /// 2.3 — Marco robot: avance según heading, A/D giran, sin NaN.
    #[test]
    fn robot_frame_converts_forward_by_heading() {
        let lin = 0.5;
        let ang = 3.0;
        let mut w = HashSet::new();
        w.insert('w');

        // θ = 0 → avanza en +X.
        let (vx, vy, _) = manual_world_target(&w, FrameMode::Robot, lin, ang, 0.0);
        assert!((vx - 0.5).abs() < 1e-9);
        assert!(vy.abs() < 1e-9);

        // θ = π/2 → avanza en +Y.
        let (vx2, vy2, _) =
            manual_world_target(&w, FrameMode::Robot, lin, ang, std::f64::consts::FRAC_PI_2);
        assert!(vx2.abs() < 1e-9);
        assert!((vy2 - 0.5).abs() < 1e-9);

        // A gira (omega > 0); todo finito.
        let mut a = HashSet::new();
        a.insert('a');
        let (rx, ry, romega) = manual_world_target(&a, FrameMode::Robot, lin, ang, 1.234);
        assert!(rx.is_finite() && ry.is_finite() && romega.is_finite());
        assert!(romega > 0.0);
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
