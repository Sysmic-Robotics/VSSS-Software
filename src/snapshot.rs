//! Foto inmutable de un tick del control loop, para la GUI.
//!
//! El lazo la arma **solo si hay GUI**, con el mismo `World` con el que decidió el
//! tick, y la publica por un `tokio::sync::watch`: el canal guarda siempre la última
//! foto, publicar nunca espera a la GUI y una GUI atrasada ve la más reciente, no una
//! cola de fotos viejas. Es la única entrada de datos de la vista.
//!
//! Puntos de extensión:
//! - Fase 2: la estructura de depuración por tick (la misma que va al CSV, cambio de
//!   contrato) se cuelga de `LoopSnapshot` como `debug: Option<Arc<TickDebug>>`.
//! - Fase 3: el búfer de 30 s se alimenta desde [`publish`], el único punto de
//!   publicación.

use crate::coach::{Foul, Role, SharedReferee, SkillChoice};
use crate::control_loop::TickMode;
use crate::motion::MotionCommand;
use crate::radio::base_station::command_to_vw;
use crate::skills::{SkillId, SkillStatus};
use crate::world::World;
use glam::Vec2;
use std::collections::VecDeque;
use std::sync::Arc;
use std::time::Instant;
use tokio::sync::watch;

/// Extremo del lazo del canal de fotos.
pub type SnapshotSender = watch::Sender<Arc<LoopSnapshot>>;
/// Extremo de la GUI del canal de fotos.
pub type SnapshotReceiver = watch::Receiver<Arc<LoopSnapshot>>;

/// Canal de fotos, con una foto vacía de arranque.
pub fn snapshot_channel() -> (SnapshotSender, SnapshotReceiver) {
    watch::channel(Arc::new(LoopSnapshot::default()))
}

/// Único punto de publicación. `send_replace` no espera a nadie: reemplaza la foto
/// del canal (aunque no haya receptores) y despierta al receptor.
pub fn publish(tx: &SnapshotSender, snapshot: LoopSnapshot) {
    tx.send_replace(Arc::new(snapshot));
}

/// Foto de un tick.
#[derive(Debug, Clone)]
pub struct LoopSnapshot {
    /// Número de tick (el mismo que recibe el hook `on_tick`).
    pub tick: u32,
    /// Milisegundos desde que arrancó el lazo.
    pub t_ms: u64,
    /// Modo del tick: juego, STOP o HALT (la parada toma el camino de HALT).
    pub mode: TickMode,
    /// Parada de emergencia activa.
    pub estop: bool,
    /// Último comando del árbitro. `None` = el lazo corre sin árbitro.
    pub referee: Option<RefereeView>,
    /// Todos los robots del `World` (los dos equipos), ordenados por equipo e id.
    pub robots: Vec<RobotView>,
    /// Pelota del `World`.
    pub ball: Option<BallView>,
    /// Un elemento por id propio en `0..num_robots`, en orden.
    pub own: Vec<OwnRobotView>,
    /// Aviso de skill de la GUI que no corre (`control_loop::gui_skill_warning`).
    pub skill_warning: Option<String>,
    /// Período del lazo en los últimos 2 s.
    pub timing: TimingStats,
    /// Contadores monótonos de la visión.
    pub vision: VisionCounters,
    /// Contadores monótonos de la radio.
    pub radio: RadioCounters,
    // Fase 2: pub debug: Option<Arc<TickDebug>>,
}

impl Default for LoopSnapshot {
    fn default() -> Self {
        Self {
            tick: 0,
            t_ms: 0,
            mode: TickMode::Play,
            estop: false,
            referee: None,
            robots: Vec::new(),
            ball: None,
            own: Vec::new(),
            skill_warning: None,
            timing: TimingStats::default(),
            vision: VisionCounters::default(),
            radio: RadioCounters::default(),
        }
    }
}

impl LoopSnapshot {
    /// Robot del `World` por equipo e id.
    pub fn robot(&self, team: i32, id: i32) -> Option<&RobotView> {
        self.robots.iter().find(|r| r.team == team && r.id == id)
    }

    /// Vista del robot propio `id`.
    pub fn own_robot(&self, id: i32) -> Option<&OwnRobotView> {
        self.own.iter().find(|r| r.id == id)
    }
}

/// Último comando del árbitro.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RefereeView {
    pub foul: Foul,
    /// Comandos recibidos desde el arranque (0 = todavía no llegó ninguno).
    pub seq: u64,
}

/// Estado del árbitro compartido, en la forma de la foto.
pub fn referee_view(referee: &Option<SharedReferee>) -> Option<RefereeView> {
    referee.as_ref().and_then(|r| {
        r.lock().ok().map(|s| RefereeView {
            foul: s.command.foul,
            seq: s.seq,
        })
    })
}

/// Un robot del `World`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RobotView {
    pub team: i32,
    pub id: i32,
    /// Metros, marco mundo.
    pub position: Vec2,
    /// Radianes.
    pub orientation: f64,
    /// m/s, marco mundo.
    pub velocity: Vec2,
    /// rad/s.
    pub angular_velocity: f64,
    /// `active` del `World` (sin datos hace más de ~2 s → inactivo).
    pub active: bool,
    /// Segundos desde el último dato de visión.
    pub age_s: f32,
}

impl RobotView {
    /// Velocidad medida proyectada sobre el heading, con signo (m/s): la misma
    /// proyección que hace `command_to_vw` con el comando.
    pub fn forward_speed(&self) -> f32 {
        forward_speed(self.velocity, self.orientation)
    }
}

/// `vx·cos θ + vy·sin θ`: velocidad sobre el heading, con signo.
pub fn forward_speed(velocity: Vec2, orientation: f64) -> f32 {
    (velocity.x as f64 * orientation.cos() + velocity.y as f64 * orientation.sin()) as f32
}

/// La pelota del `World`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BallView {
    pub position: Vec2,
    pub velocity: Vec2,
}

/// Robots del `World` (los dos equipos), ordenados por equipo e id.
pub fn robot_views(world: &World) -> Vec<RobotView> {
    let mut out: Vec<RobotView> = world
        .get_blue_team_state()
        .into_iter()
        .chain(world.get_yellow_team_state())
        .map(|r| RobotView {
            team: r.team,
            id: r.id,
            position: r.position,
            orientation: r.orientation,
            velocity: r.velocity,
            angular_velocity: r.angular_velocity,
            active: r.active,
            age_s: r.last_update.elapsed().map_or(0.0, |d| d.as_secs_f32()),
        })
        .collect();
    out.sort_by_key(|r| (r.team, r.id));
    out
}

/// La pelota del `World`.
pub fn ball_view(world: &World) -> BallView {
    let b = world.get_ball_state();
    BallView {
        position: b.position,
        velocity: b.velocity,
    }
}

/// Quién decidió el comando de un robot propio en este tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandSource {
    /// La skill que eligió el coach (o el decisor).
    Coach,
    /// La skill elegida en la GUI.
    GuiSkill,
    /// Control manual por teclado.
    Manual,
    /// Parada de emergencia o HALT del árbitro: cero.
    Halt,
    /// STOP del árbitro: sin traslación.
    Stop,
}

/// Un robot propio en el tick.
#[derive(Debug, Clone, PartialEq)]
pub struct OwnRobotView {
    pub id: i32,
    /// `None` = no tuvo comando en este tick (no visto, sin skill).
    pub source: Option<CommandSource>,
    /// Rol que informa el decisor (`TickDecider::role`).
    pub role: Option<Role>,
    /// Skill aplicada (no la hay con manual ni con HALT).
    pub skill: Option<SkillId>,
    /// `SkillCatalog::status` de esa skill, con el mismo target que recibió `tick`.
    pub status: Option<SkillStatus>,
    /// Target visible del comando (metros).
    pub target: Option<Vec2>,
    pub command: Option<CommandView>,
    /// La recuperación de atasco reemplazó el comando por el escape.
    pub escaping: bool,
}

/// El comando enviado y lo que viaja en el frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CommandView {
    pub vx: f32,
    pub vy: f32,
    /// ω comandada (rad/s).
    pub omega: f32,
    /// `command_to_vw` sobre el mismo comando: lo que llega al robot.
    pub v_mm_s: i16,
    pub w_deg_s: i16,
    /// El frame recortó v al tope (`robot.max_v_mm_s`).
    pub v_clamped: bool,
    /// El frame recortó ω al tope (`robot.max_w_deg_s`).
    pub w_clamped: bool,
}

impl CommandView {
    /// Vista de un comando, con los topes de `params().robot`.
    pub fn of(cmd: &MotionCommand) -> Self {
        let robot = &crate::params::params().robot;
        let (v_mm_s, w_deg_s) = command_to_vw(cmd);
        let (v_clamped, w_clamped) = frame_clamped(cmd, robot.max_v_mm_s, robot.max_w_deg_s);
        Self {
            vx: cmd.vx as f32,
            vy: cmd.vy as f32,
            omega: cmd.omega as f32,
            v_mm_s,
            w_deg_s,
            v_clamped,
            w_clamped,
        }
    }
}

/// Si el frame recorta v u ω de `cmd` al tope: misma proyección y redondeo que
/// `command_to_vw`, antes del clamp. Un comando no finito no se recorta (el frame
/// manda cero).
pub fn frame_clamped(cmd: &MotionCommand, max_v_mm_s: i32, max_w_deg_s: i32) -> (bool, bool) {
    if !cmd.vx.is_finite() || !cmd.vy.is_finite() || !cmd.omega.is_finite() || !cmd.orientation.is_finite() {
        return (false, false);
    }
    let v = (cmd.vx * cmd.orientation.cos() + cmd.vy * cmd.orientation.sin()) * 1000.0;
    let w = cmd.omega.to_degrees();
    (
        v.round().abs() > f64::from(max_v_mm_s.max(0)),
        w.round().abs() > f64::from(max_w_deg_s.max(0)),
    )
}

/// Lo que hizo el tick con los robots propios, para armar las `OwnRobotView`.
pub struct OwnTick<'a> {
    pub mode: TickMode,
    pub own_team: i32,
    pub num_robots: usize,
    /// Salidas de `tick_commands`.
    pub commands: &'a [MotionCommand],
    pub targets: &'a [Option<Vec2>],
    pub applied: &'a [SkillChoice],
    pub escaping: &'a [bool],
    /// Ids propios con comando manual vigente.
    pub manual_ids: &'a [i32],
    /// Ids propios con skill de la GUI vigente.
    pub gui_skill_ids: &'a [i32],
    /// `SkillStatus` de cada choice aplicada.
    pub statuses: &'a [(i32, SkillStatus)],
}

impl OwnTick<'_> {
    /// Una vista por id propio en `0..num_robots`. Prioridad de la fuente: HALT,
    /// STOP (si tuvo comando), manual, skill de la GUI, coach.
    pub fn views(&self, role: impl Fn(i32) -> Option<Role>) -> Vec<OwnRobotView> {
        (0..self.num_robots as i32)
            .map(|id| {
                let idx = self
                    .commands
                    .iter()
                    .position(|c| c.team == self.own_team && c.id == id);
                let choice = self.applied.iter().find(|c| c.robot_id == id);
                let source = match self.mode {
                    TickMode::Halt => Some(CommandSource::Halt),
                    TickMode::Stop if idx.is_some() => Some(CommandSource::Stop),
                    _ if self.manual_ids.contains(&id) && idx.is_some() => Some(CommandSource::Manual),
                    _ if choice.is_some() && self.gui_skill_ids.contains(&id) => Some(CommandSource::GuiSkill),
                    _ if choice.is_some() => Some(CommandSource::Coach),
                    _ => None,
                };
                let runs_skill = matches!(
                    source,
                    Some(CommandSource::Coach | CommandSource::GuiSkill | CommandSource::Stop)
                );
                let skill = choice.filter(|_| runs_skill).map(|c| c.skill_id);
                let status = skill.and_then(|_| {
                    self.statuses.iter().find(|(i, _)| *i == id).map(|(_, s)| *s)
                });
                let target = if runs_skill {
                    idx.and_then(|i| self.targets.get(i).copied().flatten())
                } else {
                    None
                };
                OwnRobotView {
                    id,
                    source,
                    role: role(id),
                    skill,
                    status,
                    target,
                    command: idx.map(|i| CommandView::of(&self.commands[i])),
                    escaping: idx.and_then(|i| self.escaping.get(i).copied()).unwrap_or(false),
                }
            })
            .collect()
    }
}

/// Ventana de la temporización del lazo: 125 intervalos de 16 ms = 2 s.
pub const TIMING_WINDOW: usize = 125;

/// Período del lazo en la ventana.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct TimingStats {
    /// Frecuencia media (Hz).
    pub hz: f32,
    /// Desviación estándar del intervalo (ms).
    pub jitter_ms: f32,
    /// Peor intervalo (ms).
    pub worst_ms: f32,
    /// Intervalos en la ventana (0 = todavía sin dato).
    pub samples: usize,
}

/// Mide el intervalo entre ticks consecutivos (en el lazo, porque la GUI pierde
/// fotos y no vería todos los intervalos).
#[derive(Debug, Default)]
pub struct LoopTiming {
    last: Option<Instant>,
    dts_ms: VecDeque<f32>,
}

impl LoopTiming {
    /// Registra un tick. Se llama justo después de esperar el `interval`.
    pub fn tick(&mut self, now: Instant) {
        if let Some(prev) = self.last {
            self.push_ms(now.duration_since(prev).as_secs_f32() * 1000.0);
        }
        self.last = Some(now);
    }

    /// Agrega un intervalo (ms), descartando el más viejo al llenar la ventana.
    pub fn push_ms(&mut self, dt_ms: f32) {
        self.dts_ms.push_back(dt_ms);
        while self.dts_ms.len() > TIMING_WINDOW {
            self.dts_ms.pop_front();
        }
    }

    pub fn stats(&self) -> TimingStats {
        let n = self.dts_ms.len();
        if n == 0 {
            return TimingStats::default();
        }
        let sum: f32 = self.dts_ms.iter().sum();
        let mean = sum / n as f32;
        let var = self.dts_ms.iter().map(|d| (d - mean) * (d - mean)).sum::<f32>() / n as f32;
        TimingStats {
            hz: if sum > 0.0 { n as f32 * 1000.0 / sum } else { 0.0 },
            jitter_ms: var.sqrt(),
            worst_ms: self.dts_ms.iter().copied().fold(0.0, f32::max),
            samples: n,
        }
    }
}

/// Contadores monótonos de la visión (`vision::VisionStats`).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct VisionCounters {
    /// Cuadros con detección procesados.
    pub frames: u64,
    /// Cuadros perdidos: saltos del `step` de FIRASim o del `frame_number` SSL
    /// (incluye los que descartó el proxy de ruido).
    pub lost: u64,
    /// Descartados por el proxy de ruido (informativo; ya están en `lost`).
    pub proxy_dropped: u64,
    /// Último `step` de FIRASim: su tiempo simulado en ms. `None` sin FIRASim.
    pub sim_time_ms: Option<u64>,
    /// Latencia del último cuadro (ms): `t_sent − t_capture` con SSL, más la del
    /// proxy si está activo. `None` sin dato (FIRASim sin proxy).
    pub latency_ms: Option<f32>,
}

/// Contadores monótonos de la radio.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct RadioCounters {
    /// Envíos al transporte desde el arranque.
    pub sends: u64,
    /// Resultado del último envío. `None` = todavía no se envió nada.
    pub last_ok: Option<bool>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::skills::Face;

    fn cmd(id: i32, vx: f64, vy: f64, omega: f64, orientation: f64) -> MotionCommand {
        MotionCommand {
            id,
            team: 0,
            vx,
            vy,
            omega,
            orientation,
        }
    }

    #[test]
    fn timing_regular_loop() {
        let mut t = LoopTiming::default();
        for _ in 0..TIMING_WINDOW {
            t.push_ms(16.0);
        }
        let s = t.stats();
        assert!((s.hz - 62.5).abs() < 1e-3, "{}", s.hz);
        assert!(s.jitter_ms.abs() < 1e-4);
        assert_eq!(s.worst_ms, 16.0);
        assert_eq!(s.samples, TIMING_WINDOW);
    }

    #[test]
    fn timing_one_late_tick() {
        let mut t = LoopTiming::default();
        for _ in 0..TIMING_WINDOW - 1 {
            t.push_ms(16.0);
        }
        t.push_ms(40.0);
        let s = t.stats();
        assert_eq!(s.worst_ms, 40.0);
        assert!(s.hz < 62.5 && s.hz > 61.0, "{}", s.hz);
        assert!(s.jitter_ms > 1.0);
    }

    #[test]
    fn timing_window_drops_old_intervals() {
        let mut t = LoopTiming::default();
        t.push_ms(100.0);
        for _ in 0..TIMING_WINDOW {
            t.push_ms(16.0);
        }
        assert_eq!(t.stats().worst_ms, 16.0);
    }

    #[test]
    fn timing_from_instants() {
        let mut t = LoopTiming::default();
        let t0 = Instant::now();
        t.tick(t0);
        assert_eq!(t.stats().samples, 0, "el primer tick no tiene intervalo");
        t.tick(t0 + std::time::Duration::from_millis(16));
        let s = t.stats();
        assert_eq!(s.samples, 1);
        assert!((s.worst_ms - 16.0).abs() < 1e-3);
    }

    #[test]
    fn frame_clamp_at_and_above_the_top() {
        // 1.5 m/s hacia adelante: justo en el tope, no recorta.
        assert_eq!(frame_clamped(&cmd(0, 1.5, 0.0, 0.0, 0.0), 1500, 720), (false, false));
        // 1.8 m/s: recorta v.
        assert_eq!(frame_clamped(&cmd(0, 1.8, 0.0, 0.0, 0.0), 1500, 720), (true, false));
        // Marcha atrás también.
        assert_eq!(frame_clamped(&cmd(0, -1.8, 0.0, 0.0, 0.0), 1500, 720), (true, false));
        // 20 rad/s = 1146 °/s: recorta ω.
        assert_eq!(frame_clamped(&cmd(0, 0.0, 0.0, 20.0, 0.0), 1500, 720), (false, true));
        // Velocidad lateral pura: la proyección sobre el heading es cero.
        assert_eq!(frame_clamped(&cmd(0, 0.0, 3.0, 0.0, 0.0), 1500, 720), (false, false));
        // No finito: el frame manda cero, no hay recorte.
        assert_eq!(frame_clamped(&cmd(0, f64::NAN, 0.0, 0.0, 0.0), 1500, 720), (false, false));
    }

    #[test]
    fn command_view_uses_command_to_vw() {
        let c = cmd(1, 0.3, 0.0, 0.0, 0.0);
        let v = CommandView::of(&c);
        assert_eq!((v.v_mm_s, v.w_deg_s), command_to_vw(&c));
        assert_eq!(v.v_mm_s, 300);
        assert!(!v.v_clamped && !v.w_clamped);
    }

    #[test]
    fn forward_speed_has_sign() {
        assert!((forward_speed(Vec2::new(0.5, 0.0), 0.0) - 0.5).abs() < 1e-6);
        assert!((forward_speed(Vec2::new(0.5, 0.0), std::f64::consts::PI) + 0.5).abs() < 1e-6);
        assert!(forward_speed(Vec2::new(0.0, 0.5), 0.0).abs() < 1e-6);
    }

    fn status(progress: f32) -> SkillStatus {
        SkillStatus {
            progress,
            done: false,
            feasible: true,
            face: Face::Back,
        }
    }

    #[test]
    fn own_views_mark_the_source_of_each_command() {
        let commands = [cmd(0, 0.1, 0.0, 0.0, 0.0), cmd(1, 0.2, 0.0, 0.0, 0.0), cmd(2, 0.3, 0.0, 0.0, 0.0)];
        let targets = [Some(Vec2::new(0.1, 0.1)), Some(Vec2::new(0.2, 0.2)), Some(Vec2::new(0.3, 0.3))];
        let applied = [
            SkillChoice::new(0, SkillId::GoTo, Vec2::ZERO),
            SkillChoice::new(1, SkillId::ShootPush, Vec2::ZERO),
            SkillChoice::new(2, SkillId::GoalKeep, Vec2::ZERO),
        ];
        let statuses = [(1, status(0.72)), (2, status(0.1))];
        let tick = OwnTick {
            mode: TickMode::Play,
            own_team: 0,
            num_robots: 3,
            commands: &commands,
            targets: &targets,
            applied: &applied,
            escaping: &[false, true, false],
            manual_ids: &[0],
            gui_skill_ids: &[1],
            statuses: &statuses,
        };
        let v = tick.views(|id| (id == 2).then_some(Role::Keeper));
        assert_eq!(v.len(), 3);
        assert_eq!(v[0].source, Some(CommandSource::Manual));
        assert_eq!(v[0].skill, None, "el manual no corre skill");
        assert_eq!(v[0].target, None);
        assert_eq!(v[1].source, Some(CommandSource::GuiSkill));
        assert_eq!(v[1].skill, Some(SkillId::ShootPush));
        assert_eq!(v[1].status.map(|s| s.progress), Some(0.72));
        assert!(v[1].escaping);
        assert_eq!(v[2].source, Some(CommandSource::Coach));
        assert_eq!(v[2].role, Some(Role::Keeper));
        assert_eq!(v[2].target, Some(Vec2::new(0.3, 0.3)));
        assert_eq!(v[2].command.map(|c| c.v_mm_s), Some(300));
    }

    #[test]
    fn own_views_cover_every_id_even_without_command() {
        let tick = OwnTick {
            mode: TickMode::Play,
            own_team: 0,
            num_robots: 3,
            commands: &[],
            targets: &[],
            applied: &[],
            escaping: &[],
            manual_ids: &[],
            gui_skill_ids: &[],
            statuses: &[],
        };
        let v = tick.views(|_| None);
        assert_eq!(v.iter().map(|r| r.id).collect::<Vec<_>>(), vec![0, 1, 2]);
        assert!(v.iter().all(|r| r.source.is_none() && r.command.is_none()));
    }

    #[test]
    fn own_views_in_halt_and_stop() {
        let commands = [cmd(0, 0.0, 0.0, 0.0, 0.0)];
        let applied = [SkillChoice::new(0, SkillId::GoTo, Vec2::ZERO)];
        let mut tick = OwnTick {
            mode: TickMode::Halt,
            own_team: 0,
            num_robots: 1,
            commands: &commands,
            targets: &[None],
            applied: &[],
            escaping: &[false],
            manual_ids: &[],
            gui_skill_ids: &[],
            statuses: &[],
        };
        let v = tick.views(|_| None);
        assert_eq!(v[0].source, Some(CommandSource::Halt));
        assert_eq!(v[0].skill, None);
        tick.mode = TickMode::Stop;
        tick.applied = &applied;
        let v = tick.views(|_| None);
        assert_eq!(v[0].source, Some(CommandSource::Stop));
        assert_eq!(v[0].skill, Some(SkillId::GoTo), "en STOP la skill sigue corriendo (solo ω)");
    }

    #[test]
    fn publishing_never_waits_and_the_receiver_sees_the_last() {
        let (tx, rx) = snapshot_channel();
        let t0 = Instant::now();
        for tick in 1..=1000u32 {
            publish(
                &tx,
                LoopSnapshot {
                    tick,
                    ..LoopSnapshot::default()
                },
            );
        }
        assert!(t0.elapsed() < std::time::Duration::from_secs(1));
        assert!(rx.has_changed().unwrap());
        assert_eq!(rx.borrow().tick, 1000);
    }

    #[test]
    fn publishing_without_receivers_does_not_fail() {
        let (tx, rx) = snapshot_channel();
        drop(rx);
        publish(&tx, LoopSnapshot::default());
        assert_eq!(tx.borrow().tick, 0);
    }

    #[test]
    fn robot_views_from_world_are_sorted() {
        let mut w = World::new(3, 3);
        w.update_robot(2, 1, Vec2::new(0.1, 0.0), 0.0, Vec2::ZERO, 0.0);
        w.update_robot(1, 0, Vec2::new(0.2, 0.0), 0.0, Vec2::new(0.3, 0.0), 0.5);
        w.update_robot(0, 0, Vec2::new(0.3, 0.0), 0.0, Vec2::ZERO, 0.0);
        let v = robot_views(&w);
        assert_eq!(v.iter().map(|r| (r.team, r.id)).collect::<Vec<_>>(), vec![(0, 0), (0, 1), (1, 2)]);
        assert!(v.iter().all(|r| r.active && r.age_s < 1.0));
        assert!((v[1].forward_speed() - 0.3).abs() < 1e-6);
    }

    #[test]
    fn referee_view_follows_the_shared_state() {
        assert_eq!(referee_view(&None), None);
        let shared = crate::coach::new_shared_referee();
        let r = referee_view(&Some(shared.clone())).unwrap();
        assert_eq!((r.foul, r.seq), (Foul::GameOn, 0));
        crate::coach::referee::apply_command(
            &shared,
            crate::coach::RefereeCommand {
                foul: Foul::Stop,
                ..crate::coach::RefereeCommand::GAME_ON
            },
        );
        let r = referee_view(&Some(shared)).unwrap();
        assert_eq!((r.foul, r.seq), (Foul::Stop, 1));
    }
}
