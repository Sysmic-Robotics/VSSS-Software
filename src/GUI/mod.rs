mod field;
mod motion_chart;
mod radio_panel;
mod theme;
mod vision_status;
mod vw_chart;

use glam::Vec2;
use iced::futures::SinkExt;
use iced::stream;
use iced::widget::canvas::Cache;
use iced::{
    Element, Length, Subscription, Task, Theme,
    widget::{
        Canvas, button, column, container, horizontal_space, row, scrollable, slider, text,
        text_input,
    },
};
use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Instant;
use tokio::sync::mpsc;

use crate::control_loop::{GuiSkillCommand, HeadingPid, ManualCommand};
use crate::radio::TeleportItem;
use crate::skills::SkillId;
use serde::{Deserialize, Serialize};
use field::FieldCanvas;
use motion_chart::LineChart;
use vw_chart::VwChart;
pub use crate::vision::StatusUpdate;

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
/// Cantidad máxima de puntos de traza del robot seleccionado.
const TRACE_MAX: usize = 150;
/// Umbral (s) para considerar un robot "activo" según su último dato de visión.
const ACTIVE_THRESHOLD_S: f64 = 0.5;
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

/// Extrae un carácter en minúscula de una tecla de iced (solo teclas de carácter).
fn key_to_char(key: &iced::keyboard::Key) -> Option<char> {
    match key {
        iced::keyboard::Key::Character(s) => s.chars().next().map(|c| c.to_ascii_lowercase()),
        _ => None,
    }
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
        Some(SkillId::Spin) => "click = sentido de giro: x mayor que el robot → CCW, x menor → CW.",
        Some(SkillId::ChaseBall) => "el click no se usa: persigue la pelota.",
        Some(SkillId::ApproachAligned) => {
            "click = hacia dónde apuntar: se pone detrás de la pelota, alineado con la \
             línea pelota→click."
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
            "click = hacia dónde lanzar la pelota (contacto lateral + giro, con la ω de \
             team_params, no la de Tuning)."
        }
        Some(SkillId::Intercept) => {
            "el click no se usa: va al punto de intercepción predicho de la pelota."
        }
        Some(SkillId::BlockLine) => {
            "el click no se usa: cubre la línea pelota→arco propio (lado según VSSL_SIDE)."
        }
        Some(SkillId::GoalKeep) => {
            "el click no se usa: arquero en la línea del arco propio (VSSL_SIDE). Solo el \
             robot coach.keeper_id puede entrar al área."
        }
        Some(SkillId::Mark) => "click = dónde pararse; queda mirando a la pelota.",
        Some(SkillId::Hold) => "el click no se usa: quieto (v = 0, ω = 0).",
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
            "⚠ este robot no es el arquero (keeper_id = {keeper_id} en team_params.json): \
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

/// Botón de selección de skill; resaltado si es la skill activa.
fn skill_button<'a>(
    label: String,
    sk: Option<SkillId>,
    active: Option<SkillId>,
) -> iced::widget::Button<'a, Message> {
    button(text(label).size(12))
        .padding([4, 8])
        .style(if active == sk {
            button::primary
        } else {
            button::secondary
        })
        .on_press(Message::SelectSkill(sk))
}

/// Datos de motion de un robot para debug visual.
/// Se envía desde el control loop al GUI cada tick.
#[derive(Debug, Clone)]
pub struct RobotMotionDebug {
    pub team: u32,
    pub id: u32,
    /// Velocidad en frame mundial (m/s)
    pub vx: f32,
    pub vy: f32,
    /// Velocidad angular COMANDADA (rad/s), copiada de `MotionCommand.omega`.
    /// Solo informativa (comparar contra la ω medida por visión).
    pub omega: f32,
    /// Destino del skill activo (metros, frame mundial). None si no aplica.
    pub target: Option<Vec2>,
    /// Lo que LLEGA al robot: velocidad lineal (mm/s) y angular (grados/s) del
    /// frame `V,W`, ya clampadas. Mismo cálculo que el CSV de auditoría (`command_to_vw`).
    pub v_mm_s: i16,
    pub w_deg_s: i16,
}

/// Ritmo de refresco del campo en GUI. La visión/control siguen a tasa completa;
/// solo la pintura del mapa se limita para evitar trabajo visual redundante.
const GUI_FIELD_UPDATE_INTERVAL_MS: u64 = 50;

/// Secciones colapsables del sidebar.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Section {
    Control,
    Skills,
    Inspector,
    Tuning,
    Teleport,
    Radio,
    Vision,
    Telemetry,
}

impl Section {
    const ALL: [Section; 8] = [
        Section::Control,
        Section::Skills,
        Section::Inspector,
        Section::Tuning,
        Section::Teleport,
        Section::Radio,
        Section::Vision,
        Section::Telemetry,
    ];
    fn title(self) -> &'static str {
        match self {
            Section::Control => "Control",
            Section::Skills => "Skills",
            Section::Inspector => "Inspector",
            Section::Tuning => "Tuning",
            Section::Teleport => "Teleport (sim)",
            Section::Radio => "Radio",
            Section::Vision => "Visión",
            Section::Telemetry => "Telemetría",
        }
    }
}

#[derive(Debug, Clone)]
pub enum Message {
    StatusUpdate(StatusUpdate),
    FieldSnapshot(FieldSnapshot),
    MotionUpdate(Vec<RobotMotionDebug>),
    ChangeIp(String),
    ChangePort(String),
    Connect,
    ToggleTracker(bool),
    Tick,
    ToggleSection(Section),
    // --- Control manual y paneles ---
    KeyPressed(char),
    KeyReleased(char),
    /// Tick rápido que recomputa y envía el comando manual mientras está activo.
    ManualTick,
    ToggleManual(bool),
    SelectRobot(u32),
    SelectTeam(u32),
    RadioPortChanged(String),
    RadioBaudChanged(String),
    ToggleFrame,
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
    EngageEstop,
    ToggleTrace(bool),
    TpRobotXChanged(String),
    TpRobotYChanged(String),
    TpRobotThetaChanged(String),
    TpBallXChanged(String),
    TpBallYChanged(String),
    TeleportRobot,
    TeleportBall,
    ToggleTelemetryFreeze(bool),
    SetTelemetryWindow(f64),
}

#[derive(Debug, Clone)]
pub struct Robot {
    pub id: u32,
    pub team: u32,
    pub position: Vec2,
    pub orientation: f32,
    /// Velocidad lineal medida por visión (m/s, marco mundo).
    pub velocity: Vec2,
    /// Velocidad angular medida por visión (rad/s).
    pub angular_velocity: f32,
}

#[derive(Debug, Clone)]
pub struct Ball {
    pub position: Vec2,
}

#[derive(Debug, Clone, Default)]
pub struct FieldSnapshot {
    ball_count: Option<usize>,
    robot_count: Option<usize>,
    ball: Option<Ball>,
    robots: Vec<Robot>,
}

#[derive(Debug, Default)]
struct FieldSnapshotBuffer {
    ball_count: Option<usize>,
    robot_count: Option<usize>,
    ball: Option<Ball>,
    robots: BTreeMap<(u32, u32), Robot>,
}

impl FieldSnapshotBuffer {
    fn push(&mut self, update: StatusUpdate) -> Option<StatusUpdate> {
        match update {
            StatusUpdate::Connected(_, _)
            | StatusUpdate::PacketReceived
            | StatusUpdate::TransportStatus(_)
            | StatusUpdate::SkillWarning(_) => Some(update),
            StatusUpdate::BallDetected(count) => {
                self.ball_count = Some(count);
                None
            }
            StatusUpdate::RobotsDetected(count) => {
                self.robot_count = Some(count);
                None
            }
            StatusUpdate::BallPosition(position) => {
                self.ball = Some(Ball { position });
                None
            }
            StatusUpdate::RobotPosition(id, team, position, orientation, velocity, angular_velocity) => {
                self.robots.insert(
                    (team, id),
                    Robot {
                        id,
                        team,
                        position,
                        orientation,
                        velocity,
                        angular_velocity,
                    },
                );
                None
            }
        }
    }

    fn drain_snapshot(&mut self) -> Option<FieldSnapshot> {
        let snapshot = FieldSnapshot {
            ball_count: self.ball_count.take(),
            robot_count: self.robot_count.take(),
            ball: self.ball.take(),
            robots: std::mem::take(&mut self.robots).into_values().collect(),
        };

        if snapshot.ball_count.is_none()
            && snapshot.robot_count.is_none()
            && snapshot.ball.is_none()
            && snapshot.robots.is_empty()
        {
            None
        } else {
            Some(snapshot)
        }
    }
}

pub struct VisionGui {
    vision_ip: String,
    vision_port: String,
    connected: bool,
    packet_count: u64,
    packet_frequency: f64,
    last_ball_count: usize,
    last_robot_count: usize,
    robots: HashMap<(u32, u32), Robot>,
    ball: Option<Ball>,
    motion_debug: HashMap<(u32, u32), RobotMotionDebug>,
    /// Instante del último dato de visión por robot (para antigüedad/actividad).
    last_seen: HashMap<(u32, u32), Instant>,
    /// Traza de posiciones recientes del robot seleccionado (mm, marco cancha).
    trace: VecDeque<Vec2>,
    /// Si se dibuja la traza del robot seleccionado.
    trace_enabled: bool,
    field_cache: Cache,
    chart_cache: Cache,
    config_tx: Option<mpsc::Sender<ConfigUpdate>>,
    status_rx: Arc<Mutex<Option<mpsc::Receiver<StatusUpdate>>>>,
    motion_rx: Arc<Mutex<Option<mpsc::Receiver<Vec<RobotMotionDebug>>>>>,
    expanded: HashSet<Section>,
    packet_history: VecDeque<(f64, u64)>,
    start_time: Instant,
    last_second: u64,
    current_second_count: u64,
    tracker_enabled: bool,
    // --- Control manual / radio / telemetría ---
    num_robots: usize,
    selected_robot: u32,
    selected_team: u32,
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
    /// Último comando (vx, vy, omega) enviado — base de la rampa de aceleración.
    applied_cmd: (f64, f64, f64),
    keys: HashSet<char>,
    manual_tx: Option<mpsc::Sender<ManualCommand>>,
    /// Skill de GUI activa sobre el robot seleccionado. `None` = coach.
    active_skill: Option<SkillId>,
    /// Target de la skill activa (metros, marco mundo), fijado con click.
    skill_target: Vec2,
    skill_tx: Option<mpsc::Sender<GuiSkillCommand>>,
    /// Centro del arco propio (m), desde `VSSL_SIDE` para el equipo propio. Se lee
    /// una vez al arrancar, igual que `main` y el `ZoneGuard`.
    own_goal: Vec2,
    /// Motion de dos caras (`MotionConfig::from_env`, misma lectura que el loop).
    bidirectional: bool,
    /// Único robot que el `ZoneGuard` deja entrar al área propia (`coach.keeper_id`).
    keeper_id: i32,
    /// Aviso del control loop si la skill de GUI no corre (robot fuera de `World`
    /// o de otro equipo). Se muestra solo mientras hay una skill activa.
    skill_warning: Option<String>,
    /// Mapeo visión → radio vigente (`describe_slot_map`), para el panel de radio.
    slot_map_lines: Vec<String>,
    /// Velocidad angular de Spin (rad/s), tuneable en vivo.
    spin_omega: f64,
    spin_omega_str: String,
    /// PID de heading tuneable en vivo.
    pid_kp: f64,
    pid_ki: f64,
    pid_kd: f64,
    pid_kp_str: String,
    pid_ki_str: String,
    pid_kd_str: String,
    pid_tx: Option<mpsc::Sender<HeadingPid>>,
    teleport_tx: Option<mpsc::Sender<Vec<TeleportItem>>>,
    /// Campos de teleport (metros / rad) como texto.
    tp_robot_x: String,
    tp_robot_y: String,
    tp_robot_theta: String,
    tp_ball_x: String,
    tp_ball_y: String,
    /// Nombre de archivo para presets de tuning.
    preset_name: String,
    /// Mensaje breve de estado de guardar/cargar preset.
    preset_status: String,
    estop: Arc<AtomicBool>,
    /// Serie `(t, v_mm_s, w_deg_s)` del frame que llega al robot seleccionado.
    vw_history: VecDeque<(f64, i16, i16)>,
    vw_chart_cache: Cache,
    /// Serie `(t, v_comandada, v_medida)` en m/s del robot seleccionado.
    vel_history: VecDeque<(f64, f32, f32)>,
    vel_chart_cache: Cache,
    /// Serie `(t, ω_comandada, ω_medida)` en rad/s del robot seleccionado.
    omega_history: VecDeque<(f64, f32, f32)>,
    omega_chart_cache: Cache,
    /// Serie `(t, error_heading_rad, distancia_m)` respecto al target de la skill.
    skill_err_history: VecDeque<(f64, f32, f32)>,
    skill_err_chart_cache: Cache,
    /// Freeze de la telemetría: si está activo, no se agregan muestras a las series.
    telemetry_frozen: bool,
    /// Ventana de tiempo mostrada en la telemetría (s).
    telemetry_window_s: f64,
    transport_connected: Option<bool>,
    radio_target_label: String,
    radio_port: String,
    radio_baud: String,
}

#[derive(Debug, Clone)]
pub enum ConfigUpdate {
    ChangeIpPort(String, u16),
    ToggleTracker(bool), // true = habilitado, false = deshabilitado
}

/// Parámetros de arranque de la GUI. Agrupa los canales y la config inicial para
/// no explotar la aridad de `run_gui`/`new`.
pub struct GuiSetup {
    pub ip: String,
    pub port: u16,
    pub config_tx: mpsc::Sender<ConfigUpdate>,
    pub status_rx: mpsc::Receiver<StatusUpdate>,
    pub motion_rx: mpsc::Receiver<Vec<RobotMotionDebug>>,
    /// Canal de comandos manuales GUI→control loop.
    pub manual_tx: Option<mpsc::Sender<ManualCommand>>,
    /// Canal de skills de GUI GUI→control loop.
    pub skill_tx: Option<mpsc::Sender<GuiSkillCommand>>,
    /// Canal de tuneo del PID de heading GUI→control loop.
    pub pid_tx: Option<mpsc::Sender<HeadingPid>>,
    /// Canal de teleport (sim) GUI→control loop.
    pub teleport_tx: Option<mpsc::Sender<Vec<TeleportItem>>>,
    /// Flag de parada de emergencia compartido con el control loop.
    pub estop: Arc<AtomicBool>,
    pub num_robots: usize,
    /// Equipo propio inicial (0 = azul, 1 = amarillo).
    pub own_team: u32,
    /// Etiqueta del transporte activo (p. ej. "BaseStation", "FiraSim").
    pub radio_target_label: String,
    pub radio_port: String,
    pub radio_baud: String,
}

impl VisionGui {
    fn new(setup: GuiSetup) -> (Self, Task<Message>) {
        (
            VisionGui {
                vision_ip: setup.ip,
                vision_port: setup.port.to_string(),
                connected: false,
                packet_count: 0,
                packet_frequency: 0.0,
                last_ball_count: 0,
                last_robot_count: 0,
                robots: HashMap::new(),
                ball: None,
                motion_debug: HashMap::new(),
                last_seen: HashMap::new(),
                trace: VecDeque::new(),
                trace_enabled: true,
                field_cache: Cache::default(),
                chart_cache: Cache::default(),
                config_tx: Some(setup.config_tx),
                status_rx: Arc::new(Mutex::new(Some(setup.status_rx))),
                motion_rx: Arc::new(Mutex::new(Some(setup.motion_rx))),
                // Por defecto Control y Skills abiertas (lo más usado en bring-up).
                expanded: HashSet::from([Section::Control, Section::Skills]),
                packet_history: VecDeque::new(),
                start_time: Instant::now(),
                last_second: 0,
                current_second_count: 0,
                tracker_enabled: true,
                num_robots: setup.num_robots.max(1),
                selected_robot: 0,
                selected_team: setup.own_team,
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
                own_goal: own_goal_center(crate::skills::zones::defend_left_from_env(
                    setup.own_team as i32,
                )),
                bidirectional: crate::motion::MotionConfig::from_env().bidirectional,
                keeper_id: crate::params::params().coach.keeper_id,
                skill_warning: None,
                slot_map_lines: crate::radio::base_station::describe_slot_map(
                    &crate::params::params().robot.radio_slot_by_vision_id,
                ),
                spin_omega: SPIN_OMEGA_DEFAULT,
                spin_omega_str: format!("{SPIN_OMEGA_DEFAULT}"),
                pid_kp: PID_KP_DEFAULT,
                pid_ki: PID_KI_DEFAULT,
                pid_kd: PID_KD_DEFAULT,
                pid_kp_str: format!("{PID_KP_DEFAULT}"),
                pid_ki_str: format!("{PID_KI_DEFAULT}"),
                pid_kd_str: format!("{PID_KD_DEFAULT}"),
                pid_tx: setup.pid_tx,
                teleport_tx: setup.teleport_tx,
                tp_robot_x: "0.0".to_string(),
                tp_robot_y: "0.0".to_string(),
                tp_robot_theta: "0.0".to_string(),
                tp_ball_x: "0.0".to_string(),
                tp_ball_y: "0.0".to_string(),
                preset_name: "tuning.json".to_string(),
                preset_status: String::new(),
                estop: setup.estop,
                vw_history: VecDeque::new(),
                vw_chart_cache: Cache::default(),
                vel_history: VecDeque::new(),
                vel_chart_cache: Cache::default(),
                omega_history: VecDeque::new(),
                omega_chart_cache: Cache::default(),
                skill_err_history: VecDeque::new(),
                skill_err_chart_cache: Cache::default(),
                telemetry_frozen: false,
                telemetry_window_s: TELEMETRY_WINDOW_DEFAULT_S,
                transport_connected: None,
                radio_target_label: setup.radio_target_label,
                radio_port: setup.radio_port,
                radio_baud: setup.radio_baud,
            },
            Task::none(),
        )
    }

    fn title(&self) -> String {
        String::from("RustEngine - Vision System")
    }

    fn update(&mut self, message: Message) -> Task<Message> {
        match message {
            Message::StatusUpdate(update) => {
                match update {
                    StatusUpdate::Connected(ip, port) => {
                        self.vision_ip = ip;
                        self.vision_port = port.to_string();
                        self.connected = true;
                    }
                    StatusUpdate::PacketReceived => {
                        self.packet_count += 1;
                        let elapsed = self.start_time.elapsed().as_secs_f64();
                        let current_second = elapsed.floor() as u64;

                        // If we've moved to a new second, record the previous second's count
                        if current_second > self.last_second {
                            if self.current_second_count > 0 {
                                self.packet_history.push_back((
                                    self.last_second as f64,
                                    self.current_second_count,
                                ));
                            }
                            self.last_second = current_second;
                            self.current_second_count = 1;

                            // Keep only last 60 seconds of data
                            while let Some(&(time, _)) = self.packet_history.front() {
                                if current_second as f64 - time > 60.0 {
                                    self.packet_history.pop_front();
                                } else {
                                    break;
                                }
                            }

                            // Clear chart cache to trigger redraw
                            self.chart_cache.clear();
                        } else {
                            // Same second, just increment the counter
                            self.current_second_count += 1;
                        }
                    }
                    StatusUpdate::BallDetected(count) => {
                        self.last_ball_count = count;
                    }
                    StatusUpdate::RobotsDetected(count) => {
                        self.last_robot_count = count;
                    }
                    StatusUpdate::RobotPosition(
                        id,
                        team,
                        position,
                        orientation,
                        velocity,
                        angular_velocity,
                    ) => {
                        self.robots.insert(
                            (team, id),
                            Robot {
                                id,
                                team,
                                position,
                                orientation,
                                velocity,
                                angular_velocity,
                            },
                        );
                        self.note_robot_seen(team, id, position);
                        self.field_cache.clear();
                    }
                    StatusUpdate::BallPosition(position) => {
                        self.ball = Some(Ball { position });
                        self.field_cache.clear();
                    }
                    StatusUpdate::TransportStatus(ok) => {
                        self.transport_connected = Some(ok);
                    }
                    StatusUpdate::SkillWarning(warning) => {
                        self.skill_warning = warning;
                    }
                }
            }
            Message::FieldSnapshot(snapshot) => {
                let mut field_changed = false;

                if let Some(count) = snapshot.ball_count {
                    self.last_ball_count = count;
                }
                if let Some(count) = snapshot.robot_count {
                    self.last_robot_count = count;
                }
                if let Some(ball) = snapshot.ball {
                    self.ball = Some(ball);
                    field_changed = true;
                }
                for robot in snapshot.robots {
                    let (t, i, p) = (robot.team, robot.id, robot.position);
                    self.robots.insert((t, i), robot);
                    self.note_robot_seen(t, i, p);
                    field_changed = true;
                }

                if field_changed {
                    self.field_cache.clear();
                }
            }
            Message::MotionUpdate(updates) => {
                let t = self.start_time.elapsed().as_secs_f64();
                for m in updates {
                    // Telemetría del robot seleccionado (si no está congelada).
                    if m.team == self.selected_team
                        && m.id == self.selected_robot
                        && !self.telemetry_frozen
                    {
                        // Consigna que llega al robot (v mm/s, w °/s).
                        push_capped(&mut self.vw_history, (t, m.v_mm_s, m.w_deg_s));

                        // Comandado vs medido: velocidad lineal (m/s) y ω (rad/s).
                        let cmd_v = (m.vx * m.vx + m.vy * m.vy).sqrt();
                        let cmd_w = m.omega;
                        let robot = self.robots.get(&(m.team, m.id));
                        let meas_v = robot
                            .map(|r| (r.velocity.x * r.velocity.x + r.velocity.y * r.velocity.y).sqrt())
                            .unwrap_or(0.0);
                        let meas_w = robot.map(|r| r.angular_velocity).unwrap_or(0.0);
                        push_capped(&mut self.vel_history, (t, cmd_v, meas_v));
                        push_capped(&mut self.omega_history, (t, cmd_w, meas_w));

                        // Error de heading y distancia al target de la skill activa.
                        if let (Some(target), Some(r)) = (m.target, robot) {
                            let pos_m = Vec2::new(r.position.x / 1000.0, r.position.y / 1000.0);
                            let to_target = target - pos_m;
                            let dist = to_target.length();
                            let heading_err =
                                wrap_angle(to_target.y.atan2(to_target.x) - r.orientation);
                            push_capped(&mut self.skill_err_history, (t, heading_err, dist));
                        }

                        self.vw_chart_cache.clear();
                        self.vel_chart_cache.clear();
                        self.omega_chart_cache.clear();
                        self.skill_err_chart_cache.clear();
                    }
                    self.motion_debug.insert((m.team, m.id), m);
                }
                self.field_cache.clear();
            }
            Message::ChangeIp(ip) => {
                self.vision_ip = ip;
            }
            Message::ChangePort(port) => {
                self.vision_port = port;
            }
            Message::Connect => {
                if let Ok(port) = self.vision_port.parse::<u16>()
                    && let Some(tx) = &self.config_tx
                {
                    let _ = tx.try_send(ConfigUpdate::ChangeIpPort(self.vision_ip.clone(), port));
                }
            }
            Message::Tick => {
                // Update chart even when no packets arrive
                let elapsed = self.start_time.elapsed().as_secs_f64();
                let current_second = elapsed.floor() as u64;

                // If we've moved to a new second, record the previous second's count
                if current_second > self.last_second {
                    // Record the count for the previous second (could be 0)
                    self.packet_history
                        .push_back((self.last_second as f64, self.current_second_count));

                    // Fill in any missing seconds with 0 packets
                    for sec in (self.last_second + 1)..current_second {
                        self.packet_history.push_back((sec as f64, 0));
                    }

                    self.last_second = current_second;
                    self.current_second_count = 0;

                    // Keep only last 60 seconds of data
                    while let Some(&(time, _)) = self.packet_history.front() {
                        if current_second as f64 - time > 60.0 {
                            self.packet_history.pop_front();
                        } else {
                            break;
                        }
                    }

                    // Clear chart cache to trigger redraw
                    self.chart_cache.clear();
                }
            }
            Message::ToggleSection(sec) => {
                if !self.expanded.remove(&sec) {
                    self.expanded.insert(sec);
                }
            }
            Message::ToggleTracker(enabled) => {
                self.tracker_enabled = enabled;
                // Enviar comando al módulo Vision
                if let Some(tx) = &self.config_tx {
                    let _ = tx.try_send(ConfigUpdate::ToggleTracker(enabled));
                }
            }
            Message::KeyPressed(c) => {
                if self.manual_enabled {
                    self.keys.insert(c);
                }
            }
            Message::KeyReleased(c) => {
                self.keys.remove(&c);
            }
            Message::ToggleManual(enabled) => {
                self.manual_enabled = enabled;
                if !enabled {
                    self.keys.clear();
                    // Resetear la rampa para no arrancar con inercia la próxima vez.
                    self.applied_cmd = (0.0, 0.0, 0.0);
                }
            }
            Message::SelectRobot(id) => {
                if id != self.selected_robot {
                    self.selected_robot = id;
                    // Reiniciar las series de telemetría para no mezclar robots.
                    self.clear_telemetry();
                    self.trace.clear();
                    self.field_cache.clear();
                }
            }
            Message::SelectTeam(team) => {
                if team != self.selected_team {
                    self.selected_team = team;
                    self.clear_telemetry();
                    self.trace.clear();
                    self.field_cache.clear();
                }
            }
            Message::ManualTick => {
                // Mientras el modo manual está activo, recomputar el objetivo,
                // aplicar la rampa y reenviar (incluso cero al soltar teclas) para
                // mantener el comando vigente en el loop.
                if self.manual_enabled
                    && let Some(tx) = &self.manual_tx
                {
                    // Orientación del robot seleccionado (para el marco robot).
                    let theta = self
                        .robots
                        .get(&(self.selected_team, self.selected_robot))
                        .map(|r| r.orientation as f64)
                        .unwrap_or(0.0);
                    let (tx_v, ty_v, tw_v) = manual_world_target(
                        &self.keys,
                        self.manual_frame,
                        self.manual_lin_ms,
                        self.manual_ang_rads,
                        theta,
                    );
                    // Rampa por componente (tasas ajustables desde la GUI).
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
                // Skill de GUI: streamear mientras haya una activa y el manual esté off.
                if !self.manual_enabled
                    && let (Some(skill), Some(tx)) = (self.active_skill, &self.skill_tx)
                {
                    // Target efectivo: Spin elige el sentido relativo al robot,
                    // BlockLine/GoalKeep usan el arco propio y el resto el click.
                    let robot_x_m = self
                        .robots
                        .get(&(self.selected_team, self.selected_robot))
                        .map(|r| r.position.x / 1000.0)
                        .unwrap_or(0.0);
                    let target =
                        effective_skill_target(skill, self.skill_target, robot_x_m, self.own_goal);
                    let _ = tx.try_send(GuiSkillCommand {
                        team: self.selected_team as i32,
                        id: self.selected_robot as i32,
                        skill_id: skill,
                        target,
                        spin_omega: self.spin_omega,
                    });
                }
            }
            Message::ToggleFrame => {
                self.manual_frame = match self.manual_frame {
                    FrameMode::World => FrameMode::Robot,
                    FrameMode::Robot => FrameMode::World,
                };
            }
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
            Message::SelectSkill(skill) => {
                self.active_skill = skill;
                self.field_cache.clear();
            }
            Message::FieldClicked(world_m) => {
                // El click fija el target de la skill activa (si hay).
                if self.active_skill.is_some() {
                    self.skill_target = world_m;
                    self.field_cache.clear();
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
            Message::PresetNameChanged(s) => {
                self.preset_name = s;
            }
            Message::SavePreset => {
                self.preset_status = match self.save_preset() {
                    Ok(()) => format!("guardado: {}", self.preset_name),
                    Err(e) => format!("error al guardar: {e}"),
                };
            }
            Message::LoadPreset => {
                self.preset_status = match self.load_preset() {
                    Ok(()) => {
                        self.send_pid();
                        format!("cargado: {}", self.preset_name)
                    }
                    Err(e) => format!("error al cargar: {e}"),
                };
            }
            Message::ToggleEstop => {
                // Enclava/libera.
                let now = self.estop.load(Ordering::Relaxed);
                self.estop.store(!now, Ordering::Relaxed);
            }
            Message::EngageEstop => {
                // Tecla de pánico: activar (idempotente).
                self.estop.store(true, Ordering::Relaxed);
            }
            Message::ToggleTrace(on) => {
                self.trace_enabled = on;
                if !on {
                    self.trace.clear();
                }
                self.field_cache.clear();
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
                if let (Ok(x), Ok(y)) =
                    (self.tp_ball_x.parse::<f64>(), self.tp_ball_y.parse::<f64>())
                    && let Some(tx) = &self.teleport_tx
                {
                    let _ = tx.try_send(vec![TeleportItem::Ball { x, y }]);
                }
            }
            Message::RadioPortChanged(port) => {
                self.radio_port = port;
            }
            Message::RadioBaudChanged(baud) => {
                // Validación mínima: aceptar solo dígitos (o vacío mientras se edita).
                if baud.is_empty() || baud.chars().all(|c| c.is_ascii_digit()) {
                    self.radio_baud = baud;
                }
            }
            Message::ToggleTelemetryFreeze(on) => {
                self.telemetry_frozen = on;
            }
            Message::SetTelemetryWindow(s) => {
                self.telemetry_window_s = s;
                self.vw_chart_cache.clear();
                self.vel_chart_cache.clear();
                self.omega_chart_cache.clear();
                self.skill_err_chart_cache.clear();
            }
        }
        Task::none()
    }

    /// Reinicia todas las series de telemetría y sus caches (al cambiar de robot/equipo).
    fn clear_telemetry(&mut self) {
        self.vw_history.clear();
        self.vel_history.clear();
        self.omega_history.clear();
        self.skill_err_history.clear();
        self.vw_chart_cache.clear();
        self.vel_chart_cache.clear();
        self.omega_chart_cache.clear();
        self.skill_err_chart_cache.clear();
    }

    fn subscription(&self) -> Subscription<Message> {
        let rx = self.status_rx.clone();
        let motion_rx = self.motion_rx.clone();

        let status_subscription = Subscription::run_with_id(
            "status_updates",
            stream::channel(100, move |mut output| async move {
                let receiver = {
                    let mut rx_lock = rx.lock().unwrap();
                    rx_lock.take()
                };

                if let Some(mut rx) = receiver {
                    let mut buffer = FieldSnapshotBuffer::default();
                    let mut field_tick = tokio::time::interval(tokio::time::Duration::from_millis(
                        GUI_FIELD_UPDATE_INTERVAL_MS,
                    ));
                    field_tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

                    loop {
                        tokio::select! {
                            maybe_update = rx.recv() => {
                                match maybe_update {
                                    Some(update) => {
                                        if let Some(immediate) = buffer.push(update) {
                                            let _ = output.send(Message::StatusUpdate(immediate)).await;
                                        }
                                    }
                                    None => {
                                        if let Some(snapshot) = buffer.drain_snapshot() {
                                            let _ = output.send(Message::FieldSnapshot(snapshot)).await;
                                        }
                                        break;
                                    }
                                }
                            }
                            _ = field_tick.tick() => {
                                if let Some(snapshot) = buffer.drain_snapshot() {
                                    let _ = output.send(Message::FieldSnapshot(snapshot)).await;
                                }
                            }
                        }
                    }
                }

                loop {
                    tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
                }
            }),
        );

        let motion_subscription = Subscription::run_with_id(
            "motion_updates",
            stream::channel(32, move |mut output| async move {
                let receiver = {
                    let mut rx_lock = motion_rx.lock().unwrap();
                    rx_lock.take()
                };

                if let Some(mut rx) = receiver {
                    while let Some(updates) = rx.recv().await {
                        let _ = output.send(Message::MotionUpdate(updates)).await;
                    }
                }

                loop {
                    tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
                }
            }),
        );

        let tick_subscription =
            iced::time::every(std::time::Duration::from_millis(500)).map(|_| Message::Tick);

        // Control manual: teclas presionadas/soltadas + tick rápido de reenvío.
        let key_press = iced::keyboard::on_key_press(|key, _mods| {
            // Espacio = parada de emergencia (tecla de pánico).
            if matches!(
                key,
                iced::keyboard::Key::Named(iced::keyboard::key::Named::Space)
            ) {
                return Some(Message::EngageEstop);
            }
            key_to_char(&key).map(Message::KeyPressed)
        });
        let key_release = iced::keyboard::on_key_release(|key, _mods| {
            key_to_char(&key).map(Message::KeyReleased)
        });
        let manual_tick =
            iced::time::every(std::time::Duration::from_millis(33)).map(|_| Message::ManualTick);

        Subscription::batch([
            status_subscription,
            motion_subscription,
            tick_subscription,
            key_press,
            key_release,
            manual_tick,
        ])
    }

    fn theme(&self) -> Theme {
        theme::theme()
    }

    /// Registra que se recibió dato de visión de un robot: actualiza `last_seen`
    /// y, si es el seleccionado, agrega su posición a la traza.
    fn note_robot_seen(&mut self, team: u32, id: u32, position: Vec2) {
        self.last_seen.insert((team, id), Instant::now());
        if team == self.selected_team && id == self.selected_robot {
            self.trace.push_back(position);
            while self.trace.len() > TRACE_MAX {
                self.trace.pop_front();
            }
        }
    }

    /// Envía las ganancias PID actuales al loop (si hay canal).
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

    /// Carga el tuning desde `preset_name` y actualiza el estado (no envía PID; el
    /// caller decide reenviar). Errores de archivo/JSON se devuelven como `Err`.
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

    /// Contenido de la sección **Control**: manual, robot, equipo, marco.
    fn control_section(&self) -> Element<'_, Message> {
        let manual_btn = button(
            text(if self.manual_enabled {
                "Manual: ON"
            } else {
                "Manual: OFF"
            })
            .size(13),
        )
        .padding([4, 10])
        .style(if self.manual_enabled {
            button::primary
        } else {
            button::secondary
        })
        .on_press(Message::ToggleManual(!self.manual_enabled));

        let max_id = self.num_robots.saturating_sub(1) as u32;
        let robot_dec = button(text("-").size(14)).padding([4, 10]).on_press(
            Message::SelectRobot(self.selected_robot.saturating_sub(1)),
        );
        let robot_inc = button(text("+").size(14)).padding([4, 10]).on_press(
            Message::SelectRobot((self.selected_robot + 1).min(max_id)),
        );

        let team_btn = button(
            text(if self.selected_team == 0 {
                "Equipo: Azul"
            } else {
                "Equipo: Amarillo"
            })
            .size(13),
        )
        .padding([4, 10])
        .on_press(Message::SelectTeam(1 - self.selected_team));

        let frame_btn = button(
            text(match self.manual_frame {
                FrameMode::World => "Marco: Mundo",
                FrameMode::Robot => "Marco: Robot",
            })
            .size(13),
        )
        .padding([4, 10])
        .on_press(Message::ToggleFrame);

        let help = match self.manual_frame {
            FrameMode::World => "W/S=±Y · A/D=±X · Q/E girar",
            FrameMode::Robot => "W/S adelante/atrás · A/D girar",
        };

        column![
            row![manual_btn, frame_btn]
                .spacing(6)
                .align_y(iced::Alignment::Center),
            row![
                text("Robot:").size(13),
                robot_dec,
                text(format!("{}", self.selected_robot)).size(14),
                robot_inc,
                team_btn,
            ]
            .spacing(6)
            .align_y(iced::Alignment::Center),
            text(help).size(10),
        ]
        .spacing(6)
        .into()
    }

    /// Contenido de la sección **Tuning**: escalas, rampa, Spin ω, PID y presets.
    fn tuning_section(&self) -> Element<'_, Message> {
        // Fila de parámetro: etiqueta + valor en vivo + slider + entrada de texto fina.
        // El slider siempre produce un valor dentro de rango; el text_input conserva la
        // ruta de validación existente (entradas inválidas no rompen la GUI).
        let param = |label: &'static str,
                     value: f64,
                     sval: &str,
                     min: f64,
                     max: f64,
                     step: f64,
                     msg: fn(String) -> Message| {
            column![
                row![
                    text(label).size(theme::FS_SM).width(Length::Fixed(70.0)),
                    text(format!("{value:.2}"))
                        .size(theme::FS_SM)
                        .width(Length::Fixed(46.0)),
                    text_input("", sval)
                        .on_input(msg)
                        .size(theme::FS_SM)
                        .width(Length::Fixed(64.0)),
                ]
                .spacing(theme::SP_SM)
                .align_y(iced::Alignment::Center),
                slider(min..=max, value, move |v| msg(format!("{v:.3}"))).step(step),
            ]
            .spacing(theme::SP_XS)
        };

        let save_btn = button(text("Guardar").size(theme::FS_SM))
            .padding([4, 8])
            .on_press(Message::SavePreset);
        let load_btn = button(text("Cargar").size(theme::FS_SM))
            .padding([4, 8])
            .on_press(Message::LoadPreset);

        column![
            text("Manual").size(theme::FS_SM),
            param("Vel lin", self.manual_lin_ms, &self.manual_lin_str, 0.0, 1.5, 0.05, Message::ManualLinChanged),
            param("Vel ang", self.manual_ang_rads, &self.manual_ang_str, 0.0, 12.0, 0.25, Message::ManualAngChanged),
            param("Acc lin", self.manual_lin_accel, &self.manual_lin_accel_str, 0.0, 6.0, 0.1, Message::ManualLinAccelChanged),
            param("Acc ang", self.manual_ang_accel, &self.manual_ang_accel_str, 0.0, 30.0, 0.5, Message::ManualAngAccelChanged),
            text("Spin").size(theme::FS_SM),
            param("Spin ω", self.spin_omega, &self.spin_omega_str, 0.0, 40.0, 0.5, Message::SpinOmegaChanged),
            text("PID heading (GoTo/FacePoint/ChaseBall)").size(theme::FS_SM),
            param("kp", self.pid_kp, &self.pid_kp_str, 0.0, 10.0, 0.1, Message::PidKpChanged),
            param("ki", self.pid_ki, &self.pid_ki_str, 0.0, 1.0, 0.01, Message::PidKiChanged),
            param("kd", self.pid_kd, &self.pid_kd_str, 0.0, 2.0, 0.05, Message::PidKdChanged),
            text("Preset").size(theme::FS_SM),
            row![
                text_input("tuning.json", &self.preset_name)
                    .on_input(Message::PresetNameChanged)
                    .size(12)
                    .width(Length::Fixed(150.0)),
                save_btn,
                load_btn,
            ]
            .spacing(6)
            .align_y(iced::Alignment::Center),
            text(&self.preset_status).size(10),
        ]
        .spacing(4)
        .into()
    }

    /// Contenido de la sección **Skills**: botones de todo el catálogo por grupo
    /// (`SKILL_GROUPS`), ayuda de la skill activa y estado de solo lectura
    /// (bidireccional, arco propio, arquero).
    fn skills_section(&self) -> Element<'_, Message> {
        let mut col = column![skill_button(
            "Ninguna (coach)".to_string(),
            None,
            self.active_skill
        )]
        .spacing(6);
        // Un título por grupo y filas de hasta 3 botones (caben en el sidebar).
        for (title, skills) in SKILL_GROUPS {
            col = col.push(
                text(title)
                    .size(theme::FS_XS)
                    .style(|_t: &Theme| text::Style {
                        color: Some(theme::TEXT_DIM),
                    }),
            );
            for chunk in skills.chunks(3) {
                col = col.push(
                    iced::widget::Row::with_children(chunk.iter().map(|&id| {
                        skill_button(format!("{id:?}"), Some(id), self.active_skill).into()
                    }))
                    .spacing(4),
                );
            }
        }
        col = col.push(text(skill_help(self.active_skill)).size(theme::FS_SM));
        // Aviso del loop (fuente de verdad: `World`) cuando la skill no corre.
        if self.active_skill.is_some()
            && let Some(warning) = &self.skill_warning
        {
            col = col.push(
                text(format!("⚠ {warning}"))
                    .size(theme::FS_SM)
                    .style(|_t: &Theme| text::Style {
                        color: Some(theme::WARN),
                    }),
            );
        }
        if let Some(warning) =
            keeper_warning(self.active_skill, self.selected_robot, self.keeper_id)
        {
            col = col.push(
                text(warning)
                    .size(theme::FS_SM)
                    .style(|_t: &Theme| text::Style {
                        color: Some(theme::WARN),
                    }),
            );
        }
        let motion = if self.bidirectional {
            "Motion: bidireccional ON (heading mod 180°)"
        } else {
            "Motion: una cara (bidireccional OFF)"
        };
        let side = if self.own_goal.x < 0.0 {
            "izquierdo"
        } else {
            "derecho"
        };
        col.push(text(motion).size(theme::FS_XS))
            .push(
                text(format!(
                    "Arco propio: {side} (VSSL_SIDE) · arquero: robot {} (coach.keeper_id)",
                    self.keeper_id
                ))
                .size(theme::FS_XS),
            )
            .push(
                text(
                    "Tuning: Spin ω solo afecta a Spin; el PID de heading, solo a \
                     GoTo/FacePoint/ChaseBall",
                )
                .size(theme::FS_XS),
            )
            .into()
    }

    /// Contenido de la sección **Teleport** (solo sim): reposicionar robot y pelota.
    fn teleport_section(&self) -> Element<'_, Message> {
        let field = |value: &str, msg: fn(String) -> Message| {
            text_input("", value)
                .on_input(msg)
                .size(12)
                .width(Length::Fixed(56.0))
        };
        column![
            text(format!("Robot {} (x, y, θ)", self.selected_robot)).size(12),
            row![
                field(&self.tp_robot_x, Message::TpRobotXChanged),
                field(&self.tp_robot_y, Message::TpRobotYChanged),
                field(&self.tp_robot_theta, Message::TpRobotThetaChanged),
                button(text("Teleport").size(12))
                    .padding([4, 8])
                    .on_press(Message::TeleportRobot),
            ]
            .spacing(6)
            .align_y(iced::Alignment::Center),
            text("Pelota (x, y)").size(12),
            row![
                field(&self.tp_ball_x, Message::TpBallXChanged),
                field(&self.tp_ball_y, Message::TpBallYChanged),
                button(text("Teleport").size(12))
                    .padding([4, 8])
                    .on_press(Message::TeleportBall),
            ]
            .spacing(6)
            .align_y(iced::Alignment::Center),
            text("metros, marco mundo · solo FIRASim/grSim").size(10),
        ]
        .spacing(6)
        .into()
    }

    /// Contenido de la sección **Inspector**: datos de visión del robot seleccionado.
    fn inspector_section(&self) -> Element<'_, Message> {
        let trace_btn = button(
            text(if self.trace_enabled {
                "Traza: ON"
            } else {
                "Traza: OFF"
            })
            .size(12),
        )
        .padding([4, 10])
        .style(if self.trace_enabled {
            button::primary
        } else {
            button::secondary
        })
        .on_press(Message::ToggleTrace(!self.trace_enabled));

        let key = (self.selected_team, self.selected_robot);
        let data: Element<'_, Message> = match self.robots.get(&key) {
            Some(r) => {
                let speed = (r.velocity.x * r.velocity.x + r.velocity.y * r.velocity.y).sqrt();
                let age = self
                    .last_seen
                    .get(&key)
                    .map(|t| t.elapsed().as_secs_f64())
                    .unwrap_or(f64::INFINITY);
                let active = age <= ACTIVE_THRESHOLD_S;
                column![
                    text(format!(
                        "pos: ({:.2}, {:.2}) m",
                        r.position.x / 1000.0,
                        r.position.y / 1000.0
                    ))
                    .size(12),
                    text(format!("θ: {:.3} rad", r.orientation)).size(12),
                    text(format!("rapidez: {speed:.2} m/s")).size(12),
                    text(format!("ω: {:.2} rad/s", r.angular_velocity)).size(12),
                    text(format!(
                        "estado: {}",
                        if active { "activo" } else { "inactivo" }
                    ))
                    .size(12)
                    .style(move |_t: &Theme| text::Style {
                        color: Some(if active { theme::OK } else { theme::WARN }),
                    }),
                    text(format!("antigüedad: {age:.2} s")).size(12),
                ]
                .spacing(4)
                .into()
            }
            None => text("sin datos del robot seleccionado").size(12).into(),
        };
        column![trace_btn, data].spacing(6).into()
    }

    /// Contenido de la sección **Telemetría**: lectura instantánea, controles
    /// (freeze + ventana) y los gráficos temporales del robot seleccionado.
    fn telemetry_section(&self) -> Element<'_, Message> {
        let key = (self.selected_team, self.selected_robot);
        let robot = self.robots.get(&key);
        let motion = self.motion_debug.get(&key);

        // Lectura instantánea de valores actuales (o guiones si no hay datos).
        let readout = {
            let (pos, v, w) = match robot {
                Some(r) => (
                    format!("({:.2}, {:.2}) m", r.position.x / 1000.0, r.position.y / 1000.0),
                    format!(
                        "{:.2} m/s",
                        (r.velocity.x * r.velocity.x + r.velocity.y * r.velocity.y).sqrt()
                    ),
                    format!("{:.2} rad/s", r.angular_velocity),
                ),
                None => ("—".to_string(), "—".to_string(), "—".to_string()),
            };
            let lr = match motion {
                Some(m) => format!("v:{} mm/s  w:{} °/s", m.v_mm_s, m.w_deg_s),
                None => "v:— w:—".to_string(),
            };
            column![
                text(format!("pos {pos}")).size(theme::FS_XS),
                text(format!("v {v}   ω {w}")).size(theme::FS_XS),
                text(format!("radio {lr}")).size(theme::FS_XS),
            ]
            .spacing(theme::SP_XS)
        };

        // Controles: freeze + presets de ventana de tiempo.
        let freeze_btn = button(
            text(if self.telemetry_frozen {
                "Freeze: ON"
            } else {
                "Freeze: OFF"
            })
            .size(theme::FS_SM),
        )
        .padding([4, 8])
        .style(if self.telemetry_frozen {
            button::primary
        } else {
            button::secondary
        })
        .on_press(Message::ToggleTelemetryFreeze(!self.telemetry_frozen));

        let mut window_row = row![freeze_btn, text("ventana:").size(theme::FS_XS)]
            .spacing(theme::SP_SM)
            .align_y(iced::Alignment::Center);
        for w in TELEMETRY_WINDOWS_S {
            let active = (self.telemetry_window_s - w).abs() < 1e-6;
            window_row = window_row.push(
                button(text(format!("{w:.0}s")).size(theme::FS_SM))
                    .padding([4, 8])
                    .style(if active {
                        button::primary
                    } else {
                        button::secondary
                    })
                    .on_press(Message::SetTelemetryWindow(w)),
            );
        }

        let vw_chart = Canvas::new(VwChart {
            history: &self.vw_history,
            window_s: self.telemetry_window_s,
            cache: &self.vw_chart_cache,
        })
        .width(Length::Fill)
        .height(Length::Fixed(120.0));

        let vel_chart = Canvas::new(LineChart {
            history: &self.vel_history,
            window_s: self.telemetry_window_s,
            title: "v: comandada vs medida (m/s)",
            label_a: "cmd",
            color_a: theme::ACCENT,
            label_b: "med",
            color_b: theme::DATA_R,
            symmetric: false,
            cache: &self.vel_chart_cache,
        })
        .width(Length::Fill)
        .height(Length::Fixed(110.0));

        let omega_chart = Canvas::new(LineChart {
            history: &self.omega_history,
            window_s: self.telemetry_window_s,
            title: "ω: comandada vs medida (rad/s)",
            label_a: "cmd",
            color_a: theme::ACCENT,
            label_b: "med",
            color_b: theme::DATA_R,
            symmetric: true,
            cache: &self.omega_chart_cache,
        })
        .width(Length::Fill)
        .height(Length::Fixed(110.0));

        let skill_chart = Canvas::new(LineChart {
            history: &self.skill_err_history,
            window_s: self.telemetry_window_s,
            title: "skill: error θ (rad) y distancia (m)",
            label_a: "errθ",
            color_a: theme::DATA_L,
            label_b: "dist",
            color_b: theme::WARN,
            symmetric: true,
            cache: &self.skill_err_chart_cache,
        })
        .width(Length::Fill)
        .height(Length::Fixed(110.0));

        column![
            readout,
            window_row,
            vw_chart,
            vel_chart,
            omega_chart,
            skill_chart,
        ]
        .spacing(theme::SP_SM)
        .into()
    }

    /// Envuelve un contenido con su encabezado colapsable de sección.
    fn section_view<'a>(
        &'a self,
        sec: Section,
        content: Element<'a, Message>,
    ) -> Element<'a, Message> {
        let open = self.expanded.contains(&sec);
        let header = button(
            row![
                text(if open { "▼" } else { "▶" }).size(theme::FS_MD),
                text(sec.title()).size(theme::FS_LG),
            ]
            .spacing(theme::SP_SM),
        )
        .width(Length::Fill)
        .padding([6, 8])
        .style(button::secondary)
        .on_press(Message::ToggleSection(sec));

        if open {
            column![header, container(content).padding([4, 8])]
                .spacing(2)
                .into()
        } else {
            header.into()
        }
    }

    /// Sidebar: columna scrollable de secciones colapsables.
    fn sidebar_view(&self) -> Element<'_, Message> {
        let mut col = column![].spacing(6).padding(6);
        for sec in Section::ALL {
            let content: Element<'_, Message> = match sec {
                Section::Control => self.control_section(),
                Section::Skills => self.skills_section(),
                Section::Inspector => self.inspector_section(),
                Section::Tuning => self.tuning_section(),
                Section::Teleport => self.teleport_section(),
                Section::Radio => radio_panel::view(
                    &self.radio_target_label,
                    &self.radio_port,
                    &self.radio_baud,
                    self.selected_team,
                    self.transport_connected,
                    self.packet_frequency,
                    &self.slot_map_lines,
                ),
                Section::Vision => vision_status::view(
                    self.connected,
                    &self.vision_ip,
                    &self.vision_port,
                    self.packet_count,
                    self.packet_frequency,
                    self.last_ball_count,
                    self.last_robot_count,
                    &self.packet_history,
                    &self.chart_cache,
                    self.tracker_enabled,
                ),
                Section::Telemetry => self.telemetry_section(),
            };
            col = col.push(self.section_view(sec, content));
        }
        scrollable(col)
            .width(Length::Fixed(360.0))
            .height(Length::Fill)
            .into()
    }

    /// Barra superior: STOP + título + estado de conexión.
    fn top_bar_view(&self) -> Element<'_, Message> {
        let estop_on = self.estop.load(Ordering::Relaxed);
        let estop_btn = button(
            text(if estop_on {
                "⚠ ESTOP — liberar"
            } else {
                "STOP (Espacio)"
            })
            .size(15),
        )
        .padding([6, 16])
        .style(if estop_on {
            button::secondary
        } else {
            button::danger
        })
        .on_press(Message::ToggleEstop);

        let (conn_txt, conn_col) = match self.transport_connected {
            Some(true) => ("radio ✓", theme::OK),
            Some(false) => ("radio ✗", theme::ERR),
            None => ("radio —", theme::NEUTRAL),
        };
        let vis_col = if self.connected {
            theme::OK
        } else {
            theme::ERR
        };

        row![
            estop_btn,
            text("VSSS — Debug").size(theme::FS_XL),
            horizontal_space(),
            text("visión")
                .size(12)
                .style(move |_t: &Theme| text::Style { color: Some(vis_col) }),
            text(conn_txt)
                .size(12)
                .style(move |_t: &Theme| text::Style {
                    color: Some(conn_col)
                }),
        ]
        .spacing(theme::SP_LG)
        .padding([theme::SP_SM, theme::SP_MD])
        .align_y(iced::Alignment::Center)
        .into()
    }

    /// Barra de estado inferior.
    fn status_bar_view(&self) -> Element<'_, Message> {
        let theta = self
            .robots
            .get(&(self.selected_team, self.selected_robot))
            .map(|r| r.orientation)
            .unwrap_or(0.0);
        let estop_on = self.estop.load(Ordering::Relaxed);
        row![
            text(format!(
                "robot {} · {}",
                self.selected_robot,
                if self.selected_team == 0 {
                    "azul"
                } else {
                    "amarillo"
                }
            ))
            .size(12),
            text(format!("θ {theta:.2}")).size(12),
            text(format!("PPS {:.0}", self.packet_frequency)).size(12),
            horizontal_space(),
            text("Espacio = STOP")
                .size(theme::FS_XS)
                .style(|_t: &Theme| text::Style {
                    color: Some(theme::TEXT_DIM),
                }),
            text(if estop_on { "ESTOP: ON" } else { "ESTOP: off" })
                .size(12)
                .style(move |_t: &Theme| text::Style {
                    color: Some(if estop_on { theme::ERR } else { theme::NEUTRAL }),
                }),
        ]
        .spacing(14)
        .padding([4, 10])
        .align_y(iced::Alignment::Center)
        .into()
    }

    fn view(&self) -> Element<'_, Message> {
        let selected = if self.manual_enabled {
            Some((self.selected_team, self.selected_robot))
        } else {
            None
        };
        let field = Canvas::new(FieldCanvas {
            robots: &self.robots,
            ball: &self.ball,
            motion: &self.motion_debug,
            cache: &self.field_cache,
            selected,
            skill_target: self
                .active_skill
                .and_then(|s| skill_marker(s, self.skill_target, self.own_goal)),
            trace: &self.trace,
            show_trace: self.trace_enabled,
        })
        .width(Length::Fill)
        .height(Length::Fill);

        let center = row![
            self.sidebar_view(),
            container(field)
                .width(Length::Fill)
                .height(Length::Fill)
                .padding(4),
        ]
        .height(Length::Fill);

        column![self.top_bar_view(), center, self.status_bar_view()]
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
    }
}

pub fn run_gui(setup: GuiSetup) -> iced::Result {
    iced::application(VisionGui::title, VisionGui::update, VisionGui::view)
        .subscription(VisionGui::subscription)
        .theme(VisionGui::theme)
        .run_with(move || VisionGui::new(setup))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn field_snapshot_buffer_keeps_latest_visual_state() {
        let mut buffer = FieldSnapshotBuffer::default();

        assert!(matches!(
            buffer.push(StatusUpdate::PacketReceived),
            Some(StatusUpdate::PacketReceived)
        ));

        assert!(buffer.push(StatusUpdate::BallDetected(1)).is_none());
        assert!(buffer.push(StatusUpdate::RobotsDetected(3)).is_none());
        assert!(
            buffer
                .push(StatusUpdate::BallPosition(Vec2::new(10.0, 20.0)))
                .is_none()
        );
        assert!(
            buffer
                .push(StatusUpdate::RobotPosition(
                    0,
                    0,
                    Vec2::new(1.0, 2.0),
                    0.1,
                    Vec2::ZERO,
                    0.0
                ))
                .is_none()
        );
        assert!(
            buffer
                .push(StatusUpdate::RobotPosition(
                    0,
                    0,
                    Vec2::new(3.0, 4.0),
                    0.2,
                    Vec2::ZERO,
                    0.0
                ))
                .is_none()
        );

        let snapshot = buffer.drain_snapshot().expect("snapshot should exist");
        assert_eq!(snapshot.ball_count, Some(1));
        assert_eq!(snapshot.robot_count, Some(3));
        assert_eq!(
            snapshot.ball.as_ref().map(|ball| ball.position),
            Some(Vec2::new(10.0, 20.0))
        );
        assert_eq!(snapshot.robots.len(), 1);
        assert_eq!(snapshot.robots[0].id, 0);
        assert_eq!(snapshot.robots[0].team, 0);
        assert_eq!(snapshot.robots[0].position, Vec2::new(3.0, 4.0));
        assert_eq!(snapshot.robots[0].orientation, 0.2);
    }

    #[test]
    fn field_snapshot_buffer_forwards_skill_warning_immediately() {
        let mut buffer = FieldSnapshotBuffer::default();
        let w = "la skill no corre: el robot azul #1 no está en la visión".to_string();
        assert!(matches!(
            buffer.push(StatusUpdate::SkillWarning(Some(w.clone()))),
            Some(StatusUpdate::SkillWarning(Some(ref got))) if *got == w
        ));
        assert!(matches!(
            buffer.push(StatusUpdate::SkillWarning(None)),
            Some(StatusUpdate::SkillWarning(None))
        ));
        assert!(buffer.drain_snapshot().is_none());
    }

    #[test]
    fn field_snapshot_buffer_returns_none_when_empty() {
        let mut buffer = FieldSnapshotBuffer::default();
        assert!(buffer.drain_snapshot().is_none());
    }

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
    /// que `Message::MotionUpdate`).
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
