mod field;
mod radio_panel;
mod vision_status;
mod wheel_chart;

use glam::Vec2;
use iced::futures::SinkExt;
use iced::stream;
use iced::widget::canvas::Cache;
use iced::{
    Element, Length, Subscription, Task, Theme,
    widget::{Canvas, button, column, container, row, text, text_input},
};
use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Instant;
use tokio::sync::mpsc;

use crate::control_loop::{GuiSkillCommand, ManualCommand};
use crate::skills::SkillId;
use field::FieldCanvas;
use wheel_chart::WheelChart;
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
/// Ventana de la telemetría de rueda (cantidad máxima de muestras retenidas).
const WHEEL_HISTORY_MAX: usize = 300;
/// Velocidad angular de Spin por defecto en la GUI (rad/s). Coincide con
/// `SkillConfig::default().spin_omega`.
const SPIN_OMEGA_DEFAULT: f64 = 20.0;

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

/// Botón de selección de skill; resaltado si es la skill activa.
fn skill_button<'a>(
    label: &'a str,
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
    /// Destino del skill activo (metros, frame mundial). None si no aplica.
    pub target: Option<Vec2>,
    /// Velocidades de rueda que LLEGAN al robot (mm/s), ya clampadas a ±1500.
    /// Mismo cálculo que el CSV de auditoría (`command_to_wheel_mm_s`).
    pub wheel_l_mm_s: i16,
    pub wheel_r_mm_s: i16,
}

/// `true` imprime cada actualización de robot en stderr (muy ruidoso). Dejar en `false` para auditar con `[FieldAudit]` en `main`.
const GUI_LOG_EVERY_ROBOT_UPDATE: bool = false;
/// Ritmo de refresco del campo en GUI. La visión/control siguen a tasa completa;
/// solo la pintura del mapa se limita para evitar trabajo visual redundante.
const GUI_FIELD_UPDATE_INTERVAL_MS: u64 = 50;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TabView {
    Vision,
    Robots,
    Radio,
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
    TabSelected(TabView),
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
    ToggleEstop,
    EngageEstop,
}

#[derive(Debug, Clone)]
pub struct Robot {
    #[allow(dead_code)] // ID may be used for labeling robots in the future
    pub id: u32,
    pub team: u32,
    pub position: Vec2,
    pub orientation: f32,
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
            | StatusUpdate::TransportStatus(_) => Some(update),
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
            StatusUpdate::RobotPosition(id, team, position, orientation) => {
                self.robots.insert(
                    (team, id),
                    Robot {
                        id,
                        team,
                        position,
                        orientation,
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
    field_cache: Cache,
    chart_cache: Cache,
    config_tx: Option<mpsc::Sender<ConfigUpdate>>,
    status_rx: Arc<Mutex<Option<mpsc::Receiver<StatusUpdate>>>>,
    motion_rx: Arc<Mutex<Option<mpsc::Receiver<Vec<RobotMotionDebug>>>>>,
    active_tab: TabView,
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
    /// Velocidad angular de Spin (rad/s), tuneable en vivo.
    spin_omega: f64,
    spin_omega_str: String,
    estop: Arc<AtomicBool>,
    wheel_history: VecDeque<(f64, i16, i16)>,
    wheel_chart_cache: Cache,
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
                field_cache: Cache::default(),
                chart_cache: Cache::default(),
                config_tx: Some(setup.config_tx),
                status_rx: Arc::new(Mutex::new(Some(setup.status_rx))),
                motion_rx: Arc::new(Mutex::new(Some(setup.motion_rx))),
                active_tab: TabView::Vision,
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
                spin_omega: SPIN_OMEGA_DEFAULT,
                spin_omega_str: format!("{SPIN_OMEGA_DEFAULT}"),
                estop: setup.estop,
                wheel_history: VecDeque::new(),
                wheel_chart_cache: Cache::default(),
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
                    StatusUpdate::RobotPosition(id, team, position, orientation) => {
                        if GUI_LOG_EVERY_ROBOT_UPDATE {
                            eprintln!(
                                "[GUI] Recibida posición de robot: ID={}, team={}, pos=({:.2}, {:.2}) mm, orientación={:.2} rad",
                                id, team, position.x, position.y, orientation
                            );
                        }
                        self.robots.insert(
                            (team, id),
                            Robot {
                                id,
                                team,
                                position,
                                orientation,
                            },
                        );
                        self.field_cache.clear();
                        if GUI_LOG_EVERY_ROBOT_UPDATE {
                            eprintln!("[GUI] Total robots en mapa: {}", self.robots.len());
                        }
                    }
                    StatusUpdate::BallPosition(position) => {
                        self.ball = Some(Ball { position });
                        self.field_cache.clear();
                    }
                    StatusUpdate::TransportStatus(ok) => {
                        self.transport_connected = Some(ok);
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
                    self.robots.insert((robot.team, robot.id), robot);
                    field_changed = true;
                }

                if field_changed {
                    self.field_cache.clear();
                }
            }
            Message::MotionUpdate(updates) => {
                let t = self.start_time.elapsed().as_secs_f64();
                for m in updates {
                    // Telemetría de rueda del robot seleccionado.
                    if m.team == self.selected_team && m.id == self.selected_robot {
                        self.wheel_history.push_back((t, m.wheel_l_mm_s, m.wheel_r_mm_s));
                        while self.wheel_history.len() > WHEEL_HISTORY_MAX {
                            self.wheel_history.pop_front();
                        }
                        self.wheel_chart_cache.clear();
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
            Message::TabSelected(tab) => {
                self.active_tab = tab;
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
                    // Reiniciar la serie de telemetría para no mezclar robots.
                    self.wheel_history.clear();
                    self.wheel_chart_cache.clear();
                    self.field_cache.clear();
                }
            }
            Message::SelectTeam(team) => {
                if team != self.selected_team {
                    self.selected_team = team;
                    self.wheel_history.clear();
                    self.wheel_chart_cache.clear();
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
                    // Spin: el sentido se elige relativo al robot (signo de
                    // click.x − robot.x). Las demás skills usan el target tal cual.
                    let target = if skill == SkillId::Spin {
                        let robot_x_m = self
                            .robots
                            .get(&(self.selected_team, self.selected_robot))
                            .map(|r| r.position.x / 1000.0)
                            .unwrap_or(0.0);
                        spin_relative_target(self.skill_target.x, robot_x_m)
                    } else {
                        self.skill_target
                    };
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
            Message::ToggleEstop => {
                // Enclava/libera.
                let now = self.estop.load(Ordering::Relaxed);
                self.estop.store(!now, Ordering::Relaxed);
            }
            Message::EngageEstop => {
                // Tecla de pánico: activar (idempotente).
                self.estop.store(true, Ordering::Relaxed);
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
        }
        Task::none()
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
        Theme::Dark
    }

    /// Fila de controles de control manual: toggle, selector de robot y de equipo.
    fn manual_controls_view(&self) -> Element<'_, Message> {
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
            FrameMode::World => "(W/S=±Y, A/D=±X, Q/E girar)",
            FrameMode::Robot => "(W/S adelante/atrás, A/D girar)",
        };

        let top = row![
            manual_btn,
            text("Robot:").size(13),
            robot_dec,
            text(format!("{}", self.selected_robot)).size(14),
            robot_inc,
            team_btn,
            frame_btn,
            text(help).size(11),
        ]
        .spacing(8)
        .align_y(iced::Alignment::Center);

        let scales = row![
            text("Lin máx (m/s):").size(12),
            text_input("0.5", &self.manual_lin_str)
                .on_input(Message::ManualLinChanged)
                .size(12)
                .width(Length::Fixed(70.0)),
            text("Ang máx (rad/s):").size(12),
            text_input("3.0", &self.manual_ang_str)
                .on_input(Message::ManualAngChanged)
                .size(12)
                .width(Length::Fixed(70.0)),
        ]
        .spacing(8)
        .align_y(iced::Alignment::Center);

        let accels = row![
            text("Accel lin (m/s²):").size(12),
            text_input("2.0", &self.manual_lin_accel_str)
                .on_input(Message::ManualLinAccelChanged)
                .size(12)
                .width(Length::Fixed(70.0)),
            text("Accel ang (rad/s²):").size(12),
            text_input("12.0", &self.manual_ang_accel_str)
                .on_input(Message::ManualAngAccelChanged)
                .size(12)
                .width(Length::Fixed(70.0)),
            text("(↑ para giro más snappy)").size(11),
        ]
        .spacing(8)
        .align_y(iced::Alignment::Center);

        let estop_on = self.estop.load(Ordering::Relaxed);
        let estop_btn = button(
            text(if estop_on {
                "⚠ ESTOP ACTIVO — liberar"
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

        let estop_row = row![estop_btn].spacing(8).align_y(iced::Alignment::Center);

        let skills = row![
            text("Skill:").size(12),
            skill_button("Ninguna", None, self.active_skill),
            skill_button("GoTo", Some(SkillId::GoTo), self.active_skill),
            skill_button("FacePoint", Some(SkillId::FacePoint), self.active_skill),
            skill_button("ChaseBall", Some(SkillId::ChaseBall), self.active_skill),
            skill_button("Spin", Some(SkillId::Spin), self.active_skill),
            text("Spin ω (rad/s):").size(12),
            text_input("20", &self.spin_omega_str)
                .on_input(Message::SpinOmegaChanged)
                .size(12)
                .width(Length::Fixed(60.0)),
            text("(click = target/lado)").size(11),
        ]
        .spacing(6)
        .align_y(iced::Alignment::Center);

        column![estop_row, top, scales, accels, skills]
            .spacing(6)
            .into()
    }

    fn view(&self) -> Element<'_, Message> {
        // Tab buttons
        let vision_button = button(text("Vision").size(14))
            .padding([8, 16])
            .style(if self.active_tab == TabView::Vision {
                button::primary
            } else {
                button::secondary
            })
            .on_press(Message::TabSelected(TabView::Vision));

        let robots_button = button(text("Robots").size(14))
            .padding([8, 16])
            .style(if self.active_tab == TabView::Robots {
                button::primary
            } else {
                button::secondary
            })
            .on_press(Message::TabSelected(TabView::Robots));

        let radio_button = button(text("Radio").size(14))
            .padding([8, 16])
            .style(if self.active_tab == TabView::Radio {
                button::primary
            } else {
                button::secondary
            })
            .on_press(Message::TabSelected(TabView::Radio));

        let tabs = row![vision_button, robots_button, radio_button]
            .spacing(5)
            .padding([8, 16]);

        // Content based on active tab
        let content = match self.active_tab {
            TabView::Vision => vision_status::view(
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
            TabView::Robots => {
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
                    skill_target: self.active_skill.map(|_| self.skill_target),
                })
                .width(Length::Fill)
                .height(Length::Fill);

                let controls = self.manual_controls_view();

                let chart = Canvas::new(WheelChart {
                    history: &self.wheel_history,
                    cache: &self.wheel_chart_cache,
                })
                .width(Length::Fill)
                .height(Length::Fixed(160.0));

                column![
                    container(field).width(Length::Fill).height(Length::Fill),
                    controls,
                    chart,
                ]
                .spacing(8)
                .padding(8)
                .into()
            }
            TabView::Radio => radio_panel::view(
                &self.radio_target_label,
                &self.radio_port,
                &self.radio_baud,
                self.selected_team,
                self.transport_connected,
                self.packet_frequency,
            ),
        };

        let main_content = column![tabs, content]
            .width(Length::Fill)
            .height(Length::Fill);

        container(main_content)
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
                .push(StatusUpdate::RobotPosition(0, 0, Vec2::new(1.0, 2.0), 0.1))
                .is_none()
        );
        assert!(
            buffer
                .push(StatusUpdate::RobotPosition(0, 0, Vec2::new(3.0, 4.0), 0.2))
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

    /// 4.5 — La ventana de telemetría descarta muestras viejas (misma lógica
    /// que `Message::MotionUpdate`).
    #[test]
    fn wheel_history_respects_window() {
        let mut hist: VecDeque<(f64, i16, i16)> = VecDeque::new();
        for i in 0..(WHEEL_HISTORY_MAX + 50) {
            hist.push_back((i as f64, i as i16, -(i as i16)));
            while hist.len() > WHEEL_HISTORY_MAX {
                hist.pop_front();
            }
        }
        assert_eq!(hist.len(), WHEEL_HISTORY_MAX);
        // Las 50 muestras más viejas fueron descartadas.
        assert!(hist.front().unwrap().0 >= 50.0);
    }
}
