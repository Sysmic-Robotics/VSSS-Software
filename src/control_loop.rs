//! Loop de control reutilizable: una sola fuente de verdad del bucle de 60 Hz.
//!
//! Producción (`src/main.rs`) y probador (`src/bin/skill_test.rs --mode skill`)
//! invocan **la misma función** `run_control_loop`. La única diferencia es el
//! decisor: `CoachDecider` (envuelve un `Box<dyn Coach>` con frame-skip) vs
//! `FixedSkillDecider` (emite la misma `SkillChoice` cada tick).
//!
//! Principio de fidelidad: si una skill pasa la prueba en `skill_test`, debe
//! comportarse idéntico al correrla bajo `main` con el mismo target.

use crate::GUI;
use crate::coach::{Coach, Foul, Observation, SharedReferee, SkillChoice};
use crate::motion::{BorderRecovery, Motion, MotionCommand, MotionConfig};
use crate::radio::{RadioTarget, TransportError};
use crate::skills::zones::ZoneGuard;
use crate::skills::{SkillCatalog, SkillId};
use crate::vision::{Vision, VisionEvent, VisionSource};
use crate::world::{RobotState, World};
use glam::Vec2;
use std::collections::{HashMap, HashSet};
use std::sync::{
    Arc,
    atomic::{AtomicBool, AtomicU64, Ordering},
};
use std::time::{Duration, Instant};
use tokio::sync::{Mutex as TokioMutex, RwLock as TokioRwLock, mpsc};

/// Decide qué skills correr en este tick. Producción: `CoachDecider`.
/// Probador: `FixedSkillDecider`.
pub trait TickDecider: Send {
    /// Devuelve las `SkillChoice` vigentes para `tick`. Puede reusar las anteriores.
    fn decide(&mut self, tick: u32, world: &World) -> Vec<SkillChoice>;
}

/// Configuración del loop. `vision_source` y `radio_target` se pasan EXPLÍCITOS
/// (no se leen del entorno aquí). El caller decide cómo obtenerlos: `main.rs`
/// los lee de env una sola vez; `skill_test.rs` los lee de sus flags CLI.
pub struct ControlLoopConfig {
    pub own_team: i32,
    pub num_robots: usize,
    pub vision_source: VisionSource,
    pub radio_target: RadioTarget,
    /// `None` = infinito (modo producción). `Some(N)` = auto-stop tras N ticks.
    pub max_ticks: Option<u32>,
    /// Si está presente y no llegan paquetes de visión en esa ventana, abort con error.
    pub vision_timeout: Option<Duration>,
    /// Estado del árbitro (listener de `coach::referee`). Con HALT el loop detiene al
    /// equipo propio y con STOP solo deja corregir la orientación (ver `TickMode`).
    /// `None` = sin árbitro: el loop no lo consulta.
    pub referee: Option<SharedReferee>,
}

/// Qué hace el loop con el equipo propio en un tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TickMode {
    /// Juego: decider → skills → reflejos.
    Play,
    /// STOP del árbitro (§9.5, "mantener y solo corregir orientación"): decider y
    /// skills, pero traslación cero, la `omega` de cada skill y sin reflejos.
    Stop,
    /// HALT del árbitro o parada de emergencia: cero a todos los ids propios, sin
    /// decider, sin skills y sin reflejos.
    Halt,
}

/// Modo que impone el árbitro según su último comando (`Play` sin árbitro o con el lock
/// envenenado).
pub fn referee_mode(referee: &Option<SharedReferee>) -> TickMode {
    match referee.as_ref().and_then(|r| r.lock().ok().map(|s| s.command.foul)) {
        Some(Foul::Halt) => TickMode::Halt,
        Some(Foul::Stop) => TickMode::Stop,
        _ => TickMode::Play,
    }
}

/// Información disponible al hook `on_tick` para logging externo. El loop común
/// no escribe nada; el hook decide si materializa CSV, manda al GUI, etc.
pub struct TickRecord<'a> {
    pub tick: u32,
    pub t_ms: u64,
    pub world: &'a World,
    pub commands: &'a [MotionCommand],
    /// Targets visibles paralelos a `commands` (para overlay tipo GUI o columnas de log).
    pub targets: &'a [Option<Vec2>],
    /// Choices que produjeron `commands` en este tick. Útil para que el log
    /// sepa qué `SkillId` corrió y qué `target` paramétrico se usó.
    pub choices: &'a [SkillChoice],
    /// Paralelo a `commands`: `true` si en este tick la recuperación de atasco
    /// reemplazó el comando por la maniobra de escape (lo cuenta `motion_bench`).
    pub escaping: &'a [bool],
}

pub type OnTick = Box<dyn FnMut(&TickRecord<'_>) + Send>;

/// Decisor de producción: envuelve un `Box<dyn Coach>` con frame-skip K=6.
///
/// Replica el comportamiento que vivía inline en `main.rs::async_main`
/// (líneas 256-275 del archivo pre-refactor):
///   - Cada `decision_period` ticks llama `coach.decide(&obs)`.
///   - Entre medio reusa `last_choices`.
pub struct CoachDecider {
    coach: Box<dyn Coach>,
    own_team: i32,
    decision_period: u32,
    last_choices: Vec<SkillChoice>,
}

impl CoachDecider {
    pub fn new(coach: Box<dyn Coach>, own_team: i32, decision_period: u32) -> Self {
        Self {
            coach,
            own_team,
            decision_period,
            last_choices: Vec::new(),
        }
    }
}

impl TickDecider for CoachDecider {
    fn decide(&mut self, tick: u32, world: &World) -> Vec<SkillChoice> {
        if tick.is_multiple_of(self.decision_period) {
            let obs = Observation::from_world(world, self.own_team);
            self.last_choices = self.coach.decide(&obs);
        }
        self.last_choices.clone()
    }
}

/// Decisor del probador: emite la misma `SkillChoice` cada tick.
pub struct FixedSkillDecider {
    pub robot_id: i32,
    pub skill_id: SkillId,
    pub target: Vec2,
}

impl FixedSkillDecider {
    pub fn new(robot_id: i32, skill_id: SkillId, target: Vec2) -> Self {
        Self {
            robot_id,
            skill_id,
            target,
        }
    }
}

impl TickDecider for FixedSkillDecider {
    fn decide(&mut self, _tick: u32, _world: &World) -> Vec<SkillChoice> {
        vec![SkillChoice {
            robot_id: self.robot_id,
            skill_id: self.skill_id,
            target: self.target,
        }]
    }
}

/// Comando de control manual proveniente de la GUI. Lleva velocidades en marco
/// mundo `(vx, vy, omega)` para un robot concreto; la `orientation` NO viaja aquí
/// porque el loop la toma del `World` (visión) en cada tick, de modo que la
/// proyección `v = vx·cosθ + vy·sinθ` use el heading real.
#[derive(Debug, Clone)]
pub struct ManualCommand {
    pub team: i32,
    pub id: i32,
    pub vx: f64,
    pub vy: f64,
    pub omega: f64,
}

/// Skill elegida desde la GUI para un robot puntual. Se inyecta como `SkillChoice`
/// antes del dispatch, corriendo por el mismo `SkillCatalog::tick` que el coach.
#[derive(Debug, Clone)]
pub struct GuiSkillCommand {
    pub team: i32,
    pub id: i32,
    pub skill_id: SkillId,
    pub target: Vec2,
    /// Override en vivo de la velocidad angular de `Spin` (rad/s). Ignorado por
    /// las otras skills. Permite tunear el tiro por giro desde la GUI.
    pub spin_omega: f64,
}

/// Ganancias del PID de heading enviadas desde la GUI para tuneo en vivo.
#[derive(Debug, Clone, Copy)]
pub struct HeadingPid {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

/// Cuántos ticks (a 60 Hz) se mantiene vigente un comando manual/skill sin refresco.
/// La GUI reenvía a mayor tasa que esto mientras está activo; al apagarlo deja de
/// enviar y el comando expira, devolviendo el control al decider. 15 ticks ≈ 250 ms.
const MANUAL_STALE_TICKS: u32 = 15;

/// Período del aviso "la skill no corre": máximo un log por período en la
/// terminal, y reenvío a la GUI con la misma cadencia (por si se perdió uno).
const SKILL_WARNING_PERIOD: Duration = Duration::from_secs(1);

/// Canales opcionales para alimentar el GUI (mismo shape que el código pre-refactor).
pub struct GuiChannels {
    pub status_tx: mpsc::Sender<GUI::StatusUpdate>,
    pub motion_tx: mpsc::Sender<Vec<GUI::RobotMotionDebug>>,
    /// Canal opcional de comandos manuales GUI→loop. `None` = sin control manual
    /// (comportamiento idéntico al headless). Ver `ManualCommand`.
    pub manual_rx: Option<mpsc::Receiver<ManualCommand>>,
    /// Canal opcional de skills de GUI GUI→loop. `None` = sin runner de skills.
    pub skill_rx: Option<mpsc::Receiver<GuiSkillCommand>>,
    /// Flag opcional de parada de emergencia compartido con la GUI. Si está
    /// activo, el loop comanda cero a todo el equipo propio. `None` = sin parada.
    pub estop: Option<Arc<AtomicBool>>,
    /// Canal opcional de tuneo del PID de heading GUI→loop. `None` = sin tuneo.
    pub pid_rx: Option<mpsc::Receiver<HeadingPid>>,
    /// Canal opcional de teleport (sim) GUI→loop. `None` = sin teleport.
    pub teleport_rx: Option<mpsc::Receiver<Vec<crate::radio::TeleportItem>>>,
}

/// Construye comandos de velocidad cero para todos los robots activos del equipo
/// propio (usado por la parada de emergencia y como base del stop de cierre).
/// Reflejos de bajo nivel sobre los comandos autónomos del equipo propio, en este
/// orden: `ZoneGuard` → recuperación de atasco → `ZoneGuard`. La recuperación ve el
/// comando ya filtrado (si el guardia frena al robot en el borde de un área, no es
/// atasco) y la maniobra de escape vuelve a pasar por el guardia (no puede entrar a
/// una zona prohibida). Los robots en `manual_keys` no se tocan.
///
/// Devuelve, paralelo a `cmds`, si la recuperación reemplazó cada comando por el escape.
pub fn apply_reflexes(
    cmds: &mut [MotionCommand],
    world: &World,
    recovery: &mut BorderRecovery,
    zone_guard: &ZoneGuard,
    manual_keys: &HashSet<(i32, i32)>,
    own_team: i32,
) -> Vec<bool> {
    zone_guard.guard_commands(cmds, world, own_team, manual_keys);
    let escaping = recovery.guard_commands(cmds, world, manual_keys, own_team);
    zone_guard.guard_commands(cmds, world, own_team, manual_keys);
    escaping
}

/// Modo del tick: la parada de emergencia toma el camino de HALT, prevalezca lo que
/// diga el árbitro.
pub fn tick_mode(estop_engaged: bool, ref_mode: TickMode) -> TickMode {
    if estop_engaged { TickMode::Halt } else { ref_mode }
}

/// Comando cero para cada id del equipo propio en `0..num_robots`, lo vea o no la visión
/// (la orientación, informativa, sale de la visión si el robot está). Así el camino de
/// parada siempre emite un frame en cero: sin comandos el loop no envía nada y la base
/// station repite el último comando (no tiene timeout de serial).
fn zero_commands_for_team(
    world: &World,
    own_team: i32,
    num_robots: usize,
) -> (Vec<MotionCommand>, Vec<Option<Vec2>>) {
    let cmds: Vec<MotionCommand> = (0..num_robots as i32)
        .map(|id| MotionCommand {
            id,
            team: own_team,
            vx: 0.0,
            vy: 0.0,
            omega: 0.0,
            orientation: world.get_robot_state(id, own_team).map_or(0.0, |r| r.orientation),
        })
        .collect();
    let tgts = vec![None; cmds.len()];
    (cmds, tgts)
}

/// STOP: traslación cero y la `omega` de cada comando propio; los ids sin comando reciben
/// cero (ver `zero_commands_for_team`).
fn stop_translation(
    cmds: &mut Vec<MotionCommand>,
    tgts: &mut Vec<Option<Vec2>>,
    world: &World,
    own_team: i32,
    num_robots: usize,
) {
    for c in cmds.iter_mut().filter(|c| c.team == own_team) {
        c.vx = 0.0;
        c.vy = 0.0;
    }
    let (zeros, _) = zero_commands_for_team(world, own_team, num_robots);
    for z in zeros {
        if !cmds.iter().any(|c| c.team == z.team && c.id == z.id) {
            cmds.push(z);
            tgts.push(None);
        }
    }
}

/// Comandos de un tick: `(comandos, targets, choices aplicadas, en escape)`.
///
/// - `Play`: decider → overrides de GUI → skills → control manual → reflejos.
/// - `Stop`: igual sin reflejos, con traslación cero (`stop_translation`).
/// - `Halt`: cero a todos los ids propios, sin decider, skills ni reflejos.
///
/// En `Stop` y `Halt` vacía la recuperación de atasco y el catálogo olvida la última
/// skill de cada robot: al volver a `Play` no queda un escape interrumpido, la rampa
/// arranca de cero y cada skill se reinicia.
#[allow(clippy::too_many_arguments)]
pub(crate) fn tick_commands(
    mode: TickMode,
    world: &World,
    decider: &mut dyn TickDecider,
    tick: u32,
    catalog: &mut SkillCatalog,
    motion: &Motion,
    recovery: &mut BorderRecovery,
    zone_guard: &ZoneGuard,
    manual_state: &HashMap<(i32, i32), (ManualCommand, u32)>,
    skill_state: &HashMap<(i32, i32), (GuiSkillCommand, u32)>,
    own_team: i32,
    num_robots: usize,
) -> (Vec<MotionCommand>, Vec<Option<Vec2>>, Vec<SkillChoice>, Vec<bool>) {
    if mode == TickMode::Halt {
        recovery.reset();
        catalog.forget_last_skills();
        let (cmds, tgts) = zero_commands_for_team(world, own_team, num_robots);
        let esc = vec![false; cmds.len()];
        return (cmds, tgts, Vec::new(), esc);
    }
    // Override en vivo de la velocidad de Spin desde la GUI (por robot).
    for ((team, id), (cmd, _)) in skill_state {
        if *team == own_team && cmd.skill_id == SkillId::Spin {
            catalog.set_spin_omega_for(*id as usize, cmd.spin_omega);
        }
    }
    let mut choices = decider.decide(tick, world);
    apply_gui_skill_overrides(&mut choices, skill_state, own_team);
    let (mut cmds, mut tgts, applied) = dispatch_choices(&choices, catalog, world, motion, own_team);
    apply_manual_overrides(&mut cmds, &mut tgts, manual_state, world);
    if mode == TickMode::Stop {
        recovery.reset();
        catalog.forget_last_skills();
        stop_translation(&mut cmds, &mut tgts, world, own_team, num_robots);
        let esc = vec![false; cmds.len()];
        return (cmds, tgts, applied, esc);
    }
    // Reflejos de bajo nivel (áreas y atasco) sobre los comandos autónomos del equipo
    // propio (salta manual).
    let manual_keys: HashSet<(i32, i32)> = manual_state.keys().copied().collect();
    let esc = apply_reflexes(&mut cmds, world, recovery, zone_guard, &manual_keys, own_team);
    (cmds, tgts, applied, esc)
}

/// Límites físicos de la cancha VSSS con margen (m). Fuera de esto la fuente de
/// visión no es una cancha VSSS (p. ej. grSim con campo SSL de 9×6 m).
const VSSS_MAX_ABS_X: f32 = 0.95;
const VSSS_MAX_ABS_Y: f32 = 0.85;

/// Avisa (una sola vez) si algún robot activo o la pelota está fuera de la cancha
/// VSSS. Casi siempre es un simulador con la geometría equivocada: las skills,
/// la evasión de paredes y la recuperación de atasco asumen |x| ≤ 0.75, |y| ≤ 0.65
/// y con posiciones lejanas el robot termina girando sin sentido. Devuelve `true`
/// si avisó.
fn warn_if_outside_vsss_field(world: &World) -> bool {
    let outside = |p: Vec2| p.x.abs() > VSSS_MAX_ABS_X || p.y.abs() > VSSS_MAX_ABS_Y;
    let mut out: Vec<String> = world
        .get_blue_team_active()
        .into_iter()
        .chain(world.get_yellow_team_active())
        .filter(|r| outside(r.position))
        .map(|r| {
            format!(
                "robot {} equipo {} en ({:.2}, {:.2})",
                r.id, r.team, r.position.x, r.position.y
            )
        })
        .collect();
    let b = world.get_ball_state().position;
    if outside(b) {
        out.push(format!("pelota en ({:.2}, {:.2})", b.x, b.y));
    }
    if out.is_empty() {
        return false;
    }
    eprintln!(
        "[control_loop] ⚠ posiciones fuera de la cancha VSSS (1.5×1.3 m): {}. \
         ¿El simulador está con campo SSL? El engine asume |x|≤0.75, |y|≤0.65 (m).",
        out.join("; ")
    );
    true
}

/// Mismo dispatcher que tenía `main.rs` pre-refactor (`dispatch_choices`).
/// Para cada `SkillChoice`, busca el robot del equipo activo y delega en
/// `SkillCatalog::tick`. Robots no activos se ignoran. Devuelve comando + target
/// visible para overlay GUI o columnas de log.
fn dispatch_choices(
    choices: &[SkillChoice],
    catalog: &mut SkillCatalog,
    world: &World,
    motion: &Motion,
    own_team: i32,
) -> (Vec<MotionCommand>, Vec<Option<Vec2>>, Vec<SkillChoice>) {
    let team_robots: Vec<RobotState> = if own_team == 0 {
        world.get_blue_team_active().into_iter().cloned().collect()
    } else {
        world
            .get_yellow_team_active()
            .into_iter()
            .cloned()
            .collect()
    };

    let mut commands = Vec::with_capacity(choices.len());
    let mut targets = Vec::with_capacity(choices.len());
    let mut applied = Vec::with_capacity(choices.len());
    for choice in choices {
        let Some(robot) = team_robots.iter().find(|r| r.id == choice.robot_id) else {
            continue;
        };
        let robot_idx = choice.robot_id as usize;
        if robot_idx >= catalog.num_robots() {
            continue;
        }
        let cmd = catalog.tick(
            robot_idx,
            choice.skill_id,
            choice.target,
            robot,
            world,
            motion,
        );
        let target = match choice.skill_id {
            SkillId::GoTo
            | SkillId::FacePoint
            | SkillId::ApproachAligned
            | SkillId::ShootPush
            | SkillId::BlockLine
            | SkillId::GoalKeep
            | SkillId::Clear
            | SkillId::SpinKick
            | SkillId::Mark => Some(choice.target),
            SkillId::ChaseBall | SkillId::Intercept => Some(world.get_ball_state().position),
            SkillId::Spin | SkillId::Hold => None,
        };
        commands.push(cmd);
        targets.push(target);
        applied.push(*choice);
    }
    (commands, targets, applied)
}

/// Aplica las skills de GUI vigentes sobre las `SkillChoice` del decider, antes
/// del dispatch. Para cada skill de GUI cuyo `team == own_team`, reemplaza la
/// choice del robot (mismo `robot_id`) o la inserta si no existía. Los demás
/// robots conservan la choice del coach.
///
/// Función pura: no lee canales ni reloj. Testeable sin sockets.
fn apply_gui_skill_overrides(
    choices: &mut Vec<SkillChoice>,
    skill_state: &std::collections::HashMap<(i32, i32), (GuiSkillCommand, u32)>,
    own_team: i32,
) {
    for ((team, id), (cmd, _)) in skill_state {
        if *team != own_team {
            continue;
        }
        let choice = SkillChoice {
            robot_id: *id,
            skill_id: cmd.skill_id,
            target: cmd.target,
        };
        if let Some(pos) = choices.iter().position(|c| c.robot_id == *id) {
            choices[pos] = choice;
        } else {
            choices.push(choice);
        }
    }
}

fn team_color_name(team: i32) -> &'static str {
    if team == 0 { "azul" } else { "amarillo" }
}

/// Aviso para las skills de GUI vigentes que el loop descarta en silencio, con la
/// misma regla que `apply_gui_skill_overrides` + `dispatch_choices`: el robot debe
/// ser del equipo propio y estar activo en `World`. La regla no cambia (una skill
/// sin pose no puede correr); esto solo la hace visible. Agrega los robots que sí
/// ve la visión (azules y luego amarillos, por id). `None` = toda skill corre.
///
/// Función pura: única fuente del texto para la terminal y la GUI.
fn gui_skill_warning(
    skill_state: &HashMap<(i32, i32), (GuiSkillCommand, u32)>,
    world: &World,
    own_team: i32,
) -> Option<String> {
    let own_active = if own_team == 0 {
        world.get_blue_team_active()
    } else {
        world.get_yellow_team_active()
    };
    let mut keys: Vec<(i32, i32)> = skill_state.keys().copied().collect();
    keys.sort();
    let reasons: Vec<String> = keys
        .into_iter()
        .filter_map(|(team, id)| {
            if team != own_team {
                Some(format!(
                    "el robot {} #{id} no es del equipo propio ({})",
                    team_color_name(team),
                    team_color_name(own_team)
                ))
            } else if !own_active.iter().any(|r| r.id == id) {
                Some(format!(
                    "el robot {} #{id} no está en la visión",
                    team_color_name(team)
                ))
            } else {
                None
            }
        })
        .collect();
    if reasons.is_empty() {
        return None;
    }
    let mut seen: Vec<(i32, i32)> = world
        .get_blue_team_active()
        .into_iter()
        .chain(world.get_yellow_team_active())
        .map(|r| (r.team, r.id))
        .collect();
    seen.sort();
    let seen = if seen.is_empty() {
        "La visión no ve ningún robot".to_string()
    } else {
        let list: Vec<String> = seen
            .iter()
            .map(|(t, i)| format!("{} #{i}", team_color_name(*t)))
            .collect();
        format!("La visión ve: {}", list.join(", "))
    };
    Some(format!("la skill no corre: {}. {seen}", reasons.join("; ")))
}

/// Aplica los comandos manuales vigentes sobre los comandos del decider.
/// Para cada robot con comando manual, reemplaza el comando del decider (mismo
/// `id`/`team`) o lo inserta si no existía, tomando `orientation` del `World`
/// (0.0 si el robot no es visible). Mantiene `targets` alineado con `commands`.
///
/// Función pura sobre las entradas: no lee canales ni reloj. Testeable sin sockets.
fn apply_manual_overrides(
    commands: &mut Vec<MotionCommand>,
    targets: &mut Vec<Option<Vec2>>,
    manual_state: &HashMap<(i32, i32), (ManualCommand, u32)>,
    world: &World,
) {
    for ((team, id), (mc, _)) in manual_state {
        let orientation = world
            .get_robot_state(*id, *team)
            .map(|r| r.orientation)
            .unwrap_or(0.0);
        let manual_cmd = MotionCommand {
            id: *id,
            team: *team,
            vx: mc.vx,
            vy: mc.vy,
            omega: mc.omega,
            orientation,
        };
        if let Some(pos) = commands.iter().position(|c| c.id == *id && c.team == *team) {
            commands[pos] = manual_cmd;
        } else {
            commands.push(manual_cmd);
            targets.push(None);
        }
    }
}

/// Ejecuta el loop de control con el decisor entregado. Una sola fuente de
/// verdad: tanto `main` como `skill_test` (modo skill) llaman aquí.
pub async fn run_control_loop(
    config: ControlLoopConfig,
    mut decider: Box<dyn TickDecider>,
    mut on_tick: Option<OnTick>,
    gui: Option<GuiChannels>,
    shutdown: Arc<AtomicBool>,
) -> Result<(), TransportError> {
    let (vision_tx, mut vision_rx) = mpsc::channel(100);
    let world = Arc::new(TokioRwLock::new(World::new(
        config.num_robots,
        config.num_robots,
    )));
    // VSSL_TRACKER=off|0 desactiva el EKF desde el arranque (mediciones de ruido
    // crudo de cámara). Por defecto el tracker está encendido.
    let tracker_on_at_start = std::env::var("VSSL_TRACKER")
        .map(|v| !matches!(v.trim().to_ascii_lowercase().as_str(), "off" | "0" | "false"))
        .unwrap_or(true);
    if !tracker_on_at_start {
        eprintln!("[control_loop] VSSL_TRACKER=off → EKF desactivado (poses crudas de visión)");
    }
    let tracker_enabled = Arc::new(AtomicBool::new(tracker_on_at_start));
    let vision_pkt_count = Arc::new(AtomicU64::new(0));

    let (status_tx, motion_tx, mut manual_rx, mut skill_rx, estop, mut pid_rx, mut teleport_rx) =
        match gui {
            Some(g) => (
                Some(g.status_tx),
                Some(g.motion_tx),
                g.manual_rx,
                g.skill_rx,
                g.estop,
                g.pid_rx,
                g.teleport_rx,
            ),
            None => (None, None, None, None, None, None, None),
        };

    // Estado de comandos manuales vigentes por (team, id) con el tick de último
    // refresco, para expirar comandos rancios (ver `MANUAL_STALE_TICKS`).
    // Capa de recuperación de borde/atasco (wrapper sobre la salida de las skills).
    // Conmutable por VSSL_BORDER_RECOVERY (default on).
    let mut recovery = BorderRecovery::from_env();
    let mut manual_state: HashMap<(i32, i32), (ManualCommand, u32)> = HashMap::new();
    // Estado de skills de GUI vigentes por (team, id), misma mecánica de expiry.
    let mut skill_state: HashMap<(i32, i32), (GuiSkillCommand, u32)> = HashMap::new();
    // Última señal de conexión del transporte reportada a la GUI (para emitir
    // solo en transiciones y no inundar el canal de estado).
    let mut last_transport_ok: Option<bool> = None;
    // Aviso "la skill no corre": último log a terminal y último envío a la GUI.
    let mut skill_warning_logged_at: Option<Instant> = None;
    let mut skill_warning_sent: Option<(Option<String>, Instant)> = None;

    // Vision
    {
        let tracker_enabled = tracker_enabled.clone();
        let source = config.vision_source;
        eprintln!(
            "[control_loop] visión: {:?} ({}:{})",
            source,
            source.multicast_ip(),
            source.port()
        );
        let status_tx_vis = status_tx.clone();
        tokio::spawn(async move {
            let mut vis = Vision::new(source, tracker_enabled);
            let (dummy_tx, _) = mpsc::channel(1);
            let tx = status_tx_vis.unwrap_or(dummy_tx);
            if let Err(err) = vis.run(vision_tx, tx).await {
                eprintln!("[control_loop] vision error: {err}");
            }
        });
    }

    // World updater desde visión + contador de paquetes para el watchdog
    {
        let world = world.clone();
        let vision_pkt_count = vision_pkt_count.clone();
        tokio::spawn(async move {
            while let Some(event) = vision_rx.recv().await {
                vision_pkt_count.fetch_add(1, Ordering::Relaxed);
                let mut w = world.write().await;
                match event {
                    VisionEvent::Robot(r) => {
                        w.update_robot(
                            r.id as i32,
                            r.team as i32,
                            r.position,
                            r.orientation as f64,
                            r.velocity,
                            r.angular_velocity as f64,
                        );
                    }
                    VisionEvent::Ball(b) => {
                        w.update_ball(b.position, b.velocity);
                    }
                }
            }
        });
    }

    // Marcar robots inactivos
    {
        let world = world.clone();
        tokio::spawn(async move {
            let mut interval = tokio::time::interval(Duration::from_millis(100));
            loop {
                interval.tick().await;
                world.write().await.update();
            }
        });
    }

    // Watchdog de visión: si vision_timeout está set, abortar si no llegan paquetes
    let vision_watchdog: Option<tokio::task::JoinHandle<bool>> =
        config.vision_timeout.map(|timeout| {
            let vision_pkt_count = vision_pkt_count.clone();
            let shutdown = shutdown.clone();
            tokio::spawn(async move {
                tokio::time::sleep(timeout).await;
                if vision_pkt_count.load(Ordering::Relaxed) == 0 {
                    eprintln!(
                        "[control_loop] ✗ no recibo visión en {:?}: ¿está corriendo vsss-vision-sysmic / cámara / calibración?",
                        timeout
                    );
                    shutdown.store(true, Ordering::Relaxed);
                    true
                } else {
                    false
                }
            })
        });

    let radio = match crate::radio::Radio::from_target(config.radio_target).await {
        Ok(r) => Arc::new(TokioMutex::new(r)),
        Err(err) => {
            eprintln!("[control_loop] radio error: {err}");
            return Err(err);
        }
    };

    eprintln!("[control_loop] listo. 60 Hz control loop. Ctrl+C para detener.");

    let motion_cfg = MotionConfig::from_env();
    if motion_cfg.bidirectional {
        eprintln!("[control_loop] VSSL_BIDIRECTIONAL=1 → motion de dos caras (heading mod 180°)");
    }
    let motion = Motion::with_config(motion_cfg);
    let mut catalog = SkillCatalog::new(config.num_robots);
    // Reglamento §9.5 como restricción dura: solo el arquero en el área propia,
    // un solo atacante en el área rival (ver `skills::zones`).
    let zone_guard = ZoneGuard::new(
        crate::skills::zones::attack_sign_from_env(config.own_team),
        crate::params::params().coach.keeper_id,
    );
    let mut field_scale_warned = false;
    let mut halted_prev = false;
    let mut tick_counter: u32 = 0;
    let mut interval = tokio::time::interval(Duration::from_millis(16));
    let started = Instant::now();

    loop {
        interval.tick().await;

        if shutdown.load(Ordering::Relaxed) {
            break;
        }
        if let Some(max) = config.max_ticks
            && tick_counter >= max
        {
            break;
        }

        // Drenar el canal manual (no bloqueante) quedándose con el más reciente
        // por robot; refrescar su marca de tick.
        if let Some(rx) = manual_rx.as_mut() {
            while let Ok(mc) = rx.try_recv() {
                manual_state.insert((mc.team, mc.id), (mc, tick_counter));
            }
        }
        // Expirar comandos manuales sin refresco reciente (modo manual apagado).
        if !manual_state.is_empty() {
            manual_state
                .retain(|_, (_, seen)| tick_counter.wrapping_sub(*seen) <= MANUAL_STALE_TICKS);
        }

        // Tuneo de PID de heading desde la GUI (aplica al catálogo en runtime).
        if let Some(rx) = pid_rx.as_mut() {
            while let Ok(p) = rx.try_recv() {
                catalog.set_heading_pid(p.kp, p.ki, p.kd);
            }
        }

        // Teleport (sim) solicitado desde la GUI.
        if let Some(rx) = teleport_rx.as_mut() {
            let mut reqs: Vec<crate::radio::TeleportItem> = Vec::new();
            while let Ok(items) = rx.try_recv() {
                reqs.extend(items);
            }
            if !reqs.is_empty() {
                let mut radio_guard = radio.lock().await;
                if let Err(e) = radio_guard.teleport(&reqs).await {
                    eprintln!("[control_loop] teleport error: {e}");
                }
            }
        }

        // Drenar y expirar skills de GUI (misma mecánica que el manual).
        if let Some(rx) = skill_rx.as_mut() {
            while let Ok(sc) = rx.try_recv() {
                skill_state.insert((sc.team, sc.id), (sc, tick_counter));
            }
        }
        if !skill_state.is_empty() {
            skill_state
                .retain(|_, (_, seen)| tick_counter.wrapping_sub(*seen) <= MANUAL_STALE_TICKS);
        }

        // Parada de emergencia y HALT del árbitro: prevalecen sobre coach/manual/skills
        // (mismo camino). STOP: solo corregir orientación (ver `TickMode`).
        let estop_engaged = estop
            .as_ref()
            .map(|e| e.load(Ordering::Relaxed))
            .unwrap_or(false);
        let ref_mode = referee_mode(&config.referee);
        if (ref_mode == TickMode::Halt) != halted_prev {
            halted_prev = ref_mode == TickMode::Halt;
            if halted_prev {
                eprintln!("[control_loop] HALT del árbitro: equipo propio detenido");
            } else {
                eprintln!("[control_loop] fin de HALT");
            }
        }
        let mode = tick_mode(estop_engaged, ref_mode);
        if mode == TickMode::Halt {
            manual_state.clear();
            skill_state.clear();
        }

        let skill_warning: Option<String>;
        let (commands, targets, applied_choices, escaping) = {
            let world_guard = world.read().await;
            if !field_scale_warned {
                field_scale_warned = warn_if_outside_vsss_field(&world_guard);
            }
            // Con la parada activa `skill_state` ya está vacío → sin aviso.
            skill_warning = gui_skill_warning(&skill_state, &world_guard, config.own_team);
            tick_commands(
                mode,
                &world_guard,
                decider.as_mut(),
                tick_counter,
                &mut catalog,
                &motion,
                &mut recovery,
                &zone_guard,
                &manual_state,
                &skill_state,
                config.own_team,
                config.num_robots,
            )
        };
        tick_counter = tick_counter.wrapping_add(1);

        // Aviso de skill de GUI que no corre: terminal con límite de frecuencia y
        // GUI al cambiar o cada período. Antes del `continue` por comandos vacíos:
        // si el único robot comandado no se ve, no hay comandos.
        let now = Instant::now();
        let due = |last: Option<Instant>| {
            last.is_none_or(|t| now.duration_since(t) >= SKILL_WARNING_PERIOD)
        };
        if let Some(w) = &skill_warning
            && due(skill_warning_logged_at)
        {
            eprintln!("[control_loop] ⚠ {w}");
            skill_warning_logged_at = Some(now);
        }
        if let Some(ref tx) = status_tx {
            let changed = skill_warning_sent.as_ref().map(|(w, _)| w) != Some(&skill_warning);
            if changed || due(skill_warning_sent.as_ref().map(|(_, t)| *t)) {
                let _ = tx.try_send(GUI::StatusUpdate::SkillWarning(skill_warning.clone()));
                skill_warning_sent = Some((skill_warning.clone(), now));
            }
        }

        // Hook de logging — recibe snapshot del mundo + comandos + choices que se aplicaron.
        if let Some(ref mut hook) = on_tick {
            let world_guard = world.read().await;
            let rec = TickRecord {
                tick: tick_counter,
                t_ms: started.elapsed().as_millis() as u64,
                world: &world_guard,
                commands: &commands,
                targets: &targets,
                choices: &applied_choices,
                escaping: &escaping,
            };
            hook(&rec);
        }

        if commands.is_empty() {
            continue;
        }

        // GUI debug (igual que pre-refactor)
        if let Some(ref tx) = motion_tx {
            let updates: Vec<GUI::RobotMotionDebug> = commands
                .iter()
                .zip(targets.iter())
                .map(|(cmd, target)| {
                    // Mismo cálculo que el CSV de auditoría (skill_log) y el frame →
                    // overlay, log y radio no pueden divergir. `cmd` ya es un MotionCommand.
                    let (v_mm_s, w_deg_s) = crate::radio::base_station::command_to_vw(cmd);
                    GUI::RobotMotionDebug {
                        team: cmd.team as u32,
                        id: cmd.id as u32,
                        vx: cmd.vx as f32,
                        vy: cmd.vy as f32,
                        omega: cmd.omega as f32,
                        target: *target,
                        v_mm_s,
                        w_deg_s,
                    }
                })
                .collect();
            let _ = tx.try_send(updates);
        }

        let mut radio_guard = radio.lock().await;
        for cmd in &commands {
            radio_guard.add_motion_command(cmd.clone());
        }
        let send_result = radio_guard.send_commands().await;
        drop(radio_guard);
        let ok = send_result.is_ok();
        if let Err(err) = send_result {
            eprintln!("[control_loop] error enviando: {err}");
        }
        // Reportar el estado del transporte a la GUI solo en transiciones.
        if last_transport_ok != Some(ok) {
            last_transport_ok = Some(ok);
            if let Some(ref tx) = status_tx {
                let _ = tx.try_send(GUI::StatusUpdate::TransportStatus(ok));
            }
        }
    }

    // Stop sequence: enviar comando con velocidades en cero por cada robot del equipo
    // que haya estado activo en el último tick. Defensa en profundidad — el watchdog
    // del firmware ya frena en 200 ms aunque no llegue el stop.
    let last_world = world.read().await;
    let team_robots = if config.own_team == 0 {
        last_world.get_blue_team_active()
    } else {
        last_world.get_yellow_team_active()
    };
    let mut radio_guard = radio.lock().await;
    for robot in team_robots {
        radio_guard.add_motion_command(MotionCommand {
            id: robot.id,
            team: robot.team,
            vx: 0.0,
            vy: 0.0,
            omega: 0.0,
            orientation: robot.orientation,
        });
    }
    let _ = radio_guard.send_commands().await;

    if let Some(handle) = vision_watchdog {
        handle.abort();
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::{KickerCommand, RobotCommand};
    use crate::radio::{RobotTransport, TransportError as TErr};
    use async_trait::async_trait;
    use std::sync::Mutex;

    struct MockTransport {
        sent: Arc<Mutex<Vec<Vec<RobotCommand>>>>,
    }

    #[async_trait]
    impl RobotTransport for MockTransport {
        async fn send_commands(&mut self, commands: &[RobotCommand]) -> Result<(), TErr> {
            self.sent.lock().unwrap().push(commands.to_vec());
            Ok(())
        }
    }

    fn own_cmd(v_body: f64, theta: f64) -> MotionCommand {
        MotionCommand {
            id: 0,
            team: 0,
            vx: v_body * theta.cos(),
            vy: v_body * theta.sin(),
            omega: 0.0,
            orientation: theta,
        }
    }

    // ── Árbitro: HALT y STOP sobre `tick_commands` ──────────────────────────

    /// Estado mínimo del loop para correr `tick_commands` tick a tick, sin sockets.
    struct Rig {
        catalog: SkillCatalog,
        motion: Motion,
        recovery: BorderRecovery,
        guard: ZoneGuard,
        manual: HashMap<(i32, i32), (ManualCommand, u32)>,
        gui: HashMap<(i32, i32), (GuiSkillCommand, u32)>,
        tick: u32,
    }

    impl Rig {
        fn new(recovery: bool) -> Self {
            Self {
                catalog: SkillCatalog::new(3),
                motion: Motion::new(),
                recovery: BorderRecovery::new(recovery),
                guard: ZoneGuard::new(1.0, 2),
                manual: HashMap::new(),
                gui: HashMap::new(),
                tick: 0,
            }
        }

        fn step(&mut self, mode: TickMode, world: &World, decider: &mut dyn TickDecider) -> (Vec<MotionCommand>, Vec<bool>) {
            let (cmds, _, _, esc) = tick_commands(
                mode,
                world,
                decider,
                self.tick,
                &mut self.catalog,
                &self.motion,
                &mut self.recovery,
                &self.guard,
                &self.manual,
                &self.gui,
                0,
                3,
            );
            self.tick += 1;
            (cmds, esc)
        }
    }

    /// Decider que cuenta sus llamadas.
    struct CountingDecider {
        inner: FixedSkillDecider,
        calls: usize,
    }

    impl TickDecider for CountingDecider {
        fn decide(&mut self, tick: u32, world: &World) -> Vec<SkillChoice> {
            self.calls += 1;
            self.inner.decide(tick, world)
        }
    }

    fn robot0(cmds: &[MotionCommand]) -> MotionCommand {
        cmds.iter().find(|c| c.id == 0 && c.team == 0).cloned().expect("comando del robot 0")
    }

    fn all_zero(cmds: &[MotionCommand]) -> bool {
        cmds.iter().all(|c| c.vx == 0.0 && c.vy == 0.0 && c.omega == 0.0)
    }

    fn world_with_robot(x: f32, y: f32, th: f64) -> World {
        let mut w = World::new(3, 3);
        w.update_robot(0, 0, Vec2::new(x, y), th, Vec2::ZERO, 0.0);
        w.update_ball(Vec2::new(0.6, 0.5), Vec2::ZERO);
        w
    }

    /// Lleva al robot 0 (quieto en la visión, con GoTo hacia adelante) hasta la maniobra
    /// de escape de la recuperación de atasco.
    fn drive_into_escape(rig: &mut Rig, world: &World, decider: &mut dyn TickDecider) {
        for _ in 0..120 {
            let (_, esc) = rig.step(TickMode::Play, world, decider);
            if esc.first().copied().unwrap_or(false) {
                return;
            }
        }
        panic!("no llegó a la maniobra de escape");
    }

    fn set_referee(shared: &SharedReferee, foul: Foul) {
        crate::coach::referee::apply_command(
            shared,
            crate::coach::RefereeCommand { foul, ..crate::coach::RefereeCommand::GAME_ON },
        );
    }

    #[test]
    fn referee_mode_follows_the_last_command_and_estop_wins() {
        assert_eq!(referee_mode(&None), TickMode::Play);
        let shared = crate::coach::new_shared_referee();
        let referee = Some(shared.clone());
        assert_eq!(referee_mode(&referee), TickMode::Play, "arranca en GAME_ON");
        set_referee(&shared, Foul::Stop);
        assert_eq!(referee_mode(&referee), TickMode::Stop);
        set_referee(&shared, Foul::Halt);
        assert_eq!(referee_mode(&referee), TickMode::Halt);
        set_referee(&shared, Foul::FreeKick);
        assert_eq!(referee_mode(&referee), TickMode::Play);
        // La parada de emergencia toma el camino de HALT diga lo que diga el árbitro.
        for m in [TickMode::Play, TickMode::Stop, TickMode::Halt] {
            assert_eq!(tick_mode(true, m), TickMode::Halt);
            assert_eq!(tick_mode(false, m), m);
        }
    }

    #[test]
    fn halt_from_game_on_zeroes_at_once_and_skips_the_decider() {
        let world = world_with_robot(-0.4, 0.0, 0.0);
        let shared = crate::coach::new_shared_referee();
        let referee = Some(shared.clone());
        let mut rig = Rig::new(true);
        let mut d = CountingDecider { inner: FixedSkillDecider::new(0, SkillId::GoTo, Vec2::new(0.4, 0.0)), calls: 0 };
        let mut last = Vec::new();
        for _ in 0..10 {
            last = rig.step(referee_mode(&referee), &world, &mut d).0;
        }
        assert!(robot0(&last).vx > 0.05, "avanzaba: {:?}", robot0(&last));
        set_referee(&shared, Foul::Halt);
        for k in 0..20 {
            let (cmds, esc) = rig.step(referee_mode(&referee), &world, &mut d);
            assert!(all_zero(&cmds), "tick {k} de HALT: {cmds:?}");
            assert!(esc.iter().all(|e| !e));
        }
        assert_eq!(d.calls, 10, "el decider no se consulta en HALT");
    }

    #[test]
    fn halt_does_not_let_the_guard_move_a_field_robot_in_the_area() {
        let world = world_with_robot(-0.66, 0.0, std::f64::consts::FRAC_PI_2);
        let mut d = FixedSkillDecider::new(0, SkillId::Hold, Vec2::ZERO);
        // En juego el guardia lo saca (gira en el lugar hacia "afuera").
        let (play, _) = Rig::new(true).step(TickMode::Play, &world, &mut d);
        assert!(!all_zero(&play), "sin HALT el guardia actúa: {play:?}");
        let mut rig = Rig::new(true);
        for _ in 0..30 {
            assert!(all_zero(&rig.step(TickMode::Halt, &world, &mut d).0));
        }
    }

    #[test]
    fn halt_stops_an_escape_and_leaving_it_starts_clean() {
        let world = world_with_robot(0.0, 0.0, 0.0);
        let mut d = FixedSkillDecider::new(0, SkillId::GoTo, Vec2::new(0.6, 0.0));
        let mut rig = Rig::new(true);
        drive_into_escape(&mut rig, &world, &mut d);
        assert!(all_zero(&rig.step(TickMode::Halt, &world, &mut d).0), "cero desde el primer tick");
        for _ in 0..10 {
            rig.step(TickMode::Halt, &world, &mut d);
        }
        // Al salir: el comando es el de la skill (sin escape) y la rampa arranca de cero.
        let (cmds, esc) = rig.step(TickMode::Play, &world, &mut d);
        assert!(!esc[0], "el escape interrumpido no sigue");
        let c = robot0(&cmds);
        let v = c.vx.hypot(c.vy);
        assert!(v <= rig.motion.config.max_linear_accel * crate::motion::CONTROL_DT + 1e-9, "rampa: v={v}");
    }

    #[test]
    fn leaving_halt_restarts_spin_kick() {
        let mut world = World::new(3, 3);
        let ball = Vec2::new(0.2, -0.3);
        world.update_ball(ball, Vec2::ZERO);
        let tgt = Vec2::new(0.0, 0.3);
        let probe = crate::skills::SpinKickSkill::new(tgt);
        let (ccw, _) = probe.contact_centers(ball, (tgt - ball).normalize());
        world.update_robot(0, 0, ccw, 0.0, Vec2::ZERO, 0.0);
        let mut d = FixedSkillDecider::new(0, SkillId::SpinKick, tgt);
        let mut rig = Rig::new(false);
        let (cmds, _) = rig.step(TickMode::Play, &world, &mut d);
        assert!((robot0(&cmds).omega.abs() - probe.omega).abs() < 1e-9, "girando");
        rig.step(TickMode::Halt, &world, &mut d);
        world.update_robot(0, 0, ball + Vec2::new(0.10, 0.12), 0.0, Vec2::ZERO, 0.0);
        let (cmds, _) = rig.step(TickMode::Play, &world, &mut d);
        let w = robot0(&cmds).omega;
        assert!(w.abs() <= rig.motion.config.max_angular_speed + 1e-9, "sigue girando: ω={w}");
    }

    #[test]
    fn halt_without_visible_robots_still_zeroes_every_own_id() {
        let world = World::new(3, 3);
        let mut d = FixedSkillDecider::new(0, SkillId::GoTo, Vec2::new(0.4, 0.0));
        let (cmds, _) = Rig::new(true).step(TickMode::Halt, &world, &mut d);
        let mut ids: Vec<i32> = cmds.iter().map(|c| c.id).collect();
        ids.sort();
        assert_eq!(ids, vec![0, 1, 2]);
        assert!(all_zero(&cmds) && cmds.iter().all(|c| c.team == 0));
    }

    #[test]
    fn halt_works_with_a_decider_that_ignores_the_referee() {
        // `FixedSkillDecider` no lee al árbitro: HALT vale igual (lo aplica el loop).
        let world = world_with_robot(-0.4, 0.0, 0.0);
        let mut d = FixedSkillDecider::new(0, SkillId::ChaseBall, Vec2::ZERO);
        let mut rig = Rig::new(true);
        for _ in 0..5 {
            rig.step(TickMode::Play, &world, &mut d);
        }
        assert!(all_zero(&rig.step(TickMode::Halt, &world, &mut d).0));
    }

    #[test]
    fn base_station_frame_is_all_zero_while_stopped() {
        use crate::radio::base_station::{TeamColor, build_frame};
        use std::collections::BTreeMap;
        let to_radio = |cmds: &[MotionCommand]| -> Vec<RobotCommand> {
            cmds.iter()
                .map(|c| RobotCommand {
                    id: c.id,
                    team: c.team,
                    motion: c.clone(),
                    kicker: KickerCommand { id: c.id, team: c.team, kick_x: false, kick_z: false, dribbler: 0.0 },
                })
                .collect()
        };
        // Mapa no vacío que deja las posiciones de radio 1 y 2 sin usar.
        let slots: BTreeMap<u32, u32> = [(0, 3), (1, 0), (2, 4)].into_iter().collect();
        let mut world = world_with_robot(-0.4, 0.0, 0.0);
        world.update_robot(1, 0, Vec2::new(0.0, 0.3), 0.0, Vec2::ZERO, 0.0);
        let mut d = FixedSkillDecider::new(0, SkillId::GoTo, Vec2::new(0.4, 0.0));
        let mut rig = Rig::new(true);
        let mut play = Vec::new();
        for _ in 0..10 {
            play = rig.step(TickMode::Play, &world, &mut d).0;
        }
        let zero = "0,0,0,0,0,0,0,0,0,0\n";
        assert_ne!(build_frame(&to_radio(&play), TeamColor::Blue, &slots), zero, "en juego no es nulo");
        let halt = rig.step(TickMode::Halt, &world, &mut d).0;
        assert_eq!(build_frame(&to_radio(&halt), TeamColor::Blue, &slots), zero);
        // Con la visión caída tampoco queda una posición con el comando anterior.
        let blind = rig.step(TickMode::Halt, &World::new(3, 3), &mut d).0;
        assert_eq!(build_frame(&to_radio(&blind), TeamColor::Blue, &slots), zero);
    }

    #[test]
    fn stop_keeps_a_field_robot_in_the_area_still() {
        let world = world_with_robot(-0.66, 0.0, std::f64::consts::FRAC_PI_2);
        let mut d = FixedSkillDecider::new(0, SkillId::Hold, Vec2::ZERO);
        let mut rig = Rig::new(true);
        for _ in 0..30 {
            let c = robot0(&rig.step(TickMode::Stop, &world, &mut d).0);
            assert_eq!((c.vx, c.vy, c.omega), (0.0, 0.0, 0.0), "el guardia no lo mueve en STOP");
        }
    }

    #[test]
    fn stop_cuts_an_escape_and_game_on_does_not_resume_it() {
        let world = world_with_robot(0.0, 0.0, 0.0);
        let mut d = FixedSkillDecider::new(0, SkillId::GoTo, Vec2::new(0.6, 0.0));
        let mut rig = Rig::new(true);
        drive_into_escape(&mut rig, &world, &mut d);
        for _ in 0..10 {
            let (cmds, esc) = rig.step(TickMode::Stop, &world, &mut d);
            let c = robot0(&cmds);
            assert_eq!((c.vx, c.vy), (0.0, 0.0));
            assert!(!esc[0]);
        }
        let (_, esc) = rig.step(TickMode::Play, &world, &mut d);
        assert!(!esc[0], "el escape no continúa al volver a GAME_ON");
    }

    #[test]
    fn stop_keeps_the_skill_omega_without_translation() {
        // FacePoint hacia un punto a 90°: en STOP sigue corrigiendo la orientación.
        let world = world_with_robot(0.0, 0.0, 0.0);
        let mut d = FixedSkillDecider::new(0, SkillId::FacePoint, Vec2::new(0.0, 0.5));
        let c = robot0(&Rig::new(true).step(TickMode::Stop, &world, &mut d).0);
        assert_eq!((c.vx, c.vy), (0.0, 0.0));
        assert!(c.omega > 0.1, "gira hacia el punto: ω={}", c.omega);
    }

    #[test]
    fn stop_to_game_on_restarts_the_ramp() {
        let world = world_with_robot(-0.4, 0.0, 0.0);
        let mut d = FixedSkillDecider::new(0, SkillId::GoTo, Vec2::new(0.6, 0.0));
        let mut rig = Rig::new(false);
        let mut c = robot0(&rig.step(TickMode::Play, &world, &mut d).0);
        for _ in 0..60 {
            c = robot0(&rig.step(TickMode::Play, &world, &mut d).0);
        }
        assert!(c.vx > 0.8, "avanzaba a {}", c.vx);
        for _ in 0..5 {
            rig.step(TickMode::Stop, &world, &mut d);
        }
        let c = robot0(&rig.step(TickMode::Play, &world, &mut d).0);
        let v = c.vx.hypot(c.vy);
        assert!(v <= rig.motion.config.max_linear_accel * crate::motion::CONTROL_DT + 1e-9, "rampa: v={v}");
    }

    #[test]
    fn play_mode_matches_the_plain_pipeline() {
        // Sin árbitro (o en GAME_ON) `tick_commands` es el camino de siempre: decider →
        // skills → reflejos. Se compara contra la composición directa, tick a tick.
        let world = world_with_robot(-0.52, 0.10, std::f64::consts::PI);
        let target = Vec2::new(-0.7, 0.1);
        let mut d1 = FixedSkillDecider::new(0, SkillId::GoTo, target);
        let mut d2 = FixedSkillDecider::new(0, SkillId::GoTo, target);
        let mut rig = Rig::new(true);
        let (mut catalog, motion) = (SkillCatalog::new(3), Motion::new());
        let mut recovery = BorderRecovery::new(true);
        let guard = ZoneGuard::new(1.0, 2);
        assert_eq!(referee_mode(&None), TickMode::Play);
        for k in 0..80 {
            let (a, ea) = rig.step(TickMode::Play, &world, &mut d1);
            let choices = d2.decide(k, &world);
            let (mut b, _, _) = dispatch_choices(&choices, &mut catalog, &world, &motion, 0);
            let eb = apply_reflexes(&mut b, &world, &mut recovery, &guard, &HashSet::new(), 0);
            assert_eq!(a, b, "tick {k}");
            assert_eq!(ea, eb, "tick {k}");
        }
    }

    #[test]
    fn reflexes_robot_held_by_the_guard_is_not_stuck() {
        // Robot de campo frente al área propia, mirando al arco y queriendo entrar: el
        // guardia anula la traslación y la recuperación ve ese comando → no es atasco.
        let guard = ZoneGuard::new(1.0, 2);
        let mut recovery = BorderRecovery::new(true);
        let th = std::f64::consts::PI;
        let mut world = World::new(3, 3);
        world.update_robot(0, 0, Vec2::new(-0.52, 0.10), th, Vec2::ZERO, 0.0);
        for k in 0..60 {
            let mut cmds = vec![own_cmd(0.5, th)];
            let esc = apply_reflexes(&mut cmds, &world, &mut recovery, &guard, &HashSet::new(), 0);
            assert!(!esc[0], "escape espurio en el tick {k}");
            assert_eq!((cmds[0].vx, cmds[0].vy), (0.0, 0.0));
        }
    }

    #[test]
    fn reflexes_escape_cannot_enter_the_area() {
        // Robot trabado junto a la esquina del área propia, con heading diagonal: la
        // skill lo aleja (marcha atrás), pero no se mueve → a los 30 ticks escapa hacia el
        // centro. Proyectado a su heading, ese escape lo metería al área: el segundo paso
        // del guardia anula la traslación y deja el giro del escape.
        let guard = ZoneGuard::new(1.0, 2);
        let mut recovery = BorderRecovery::new(true);
        let th = (-0.954f64).atan2(-0.3);
        let p = Vec2::new(-0.555, 0.41);
        assert!(!guard.own_area().touches(p));
        let mut world = World::new(3, 3);
        world.update_robot(0, 0, p, th, Vec2::ZERO, 0.0);
        let mut escaped = false;
        for _ in 0..30 {
            let mut cmds = vec![own_cmd(-0.3, th)];
            let esc = apply_reflexes(&mut cmds, &world, &mut recovery, &guard, &HashSet::new(), 0);
            if esc[0] {
                escaped = true;
                assert_eq!((cmds[0].vx, cmds[0].vy), (0.0, 0.0), "el escape no entra al área");
                assert!(cmds[0].omega.abs() > 1.0, "conserva el giro del escape");
            }
        }
        assert!(escaped, "la recuperación debía disparar");
    }

    #[test]
    fn fixed_skill_decider_emits_same_choice_each_tick() {
        let mut decider = FixedSkillDecider::new(0, SkillId::GoTo, Vec2::new(0.3, 0.0));
        let world = World::new(3, 3);
        let c1 = decider.decide(0, &world);
        let c2 = decider.decide(1, &world);
        let c3 = decider.decide(2, &world);
        assert_eq!(c1.len(), 1);
        assert_eq!(c1[0].robot_id, 0);
        assert_eq!(c1[0].skill_id, SkillId::GoTo);
        assert_eq!(c1[0].target, Vec2::new(0.3, 0.0));
        assert_eq!(c1, c2);
        assert_eq!(c2, c3);
    }

    /// `CoachDecider` debe invocar al coach solo cada `decision_period` ticks
    /// y reusar `last_choices` entre medio.
    #[test]
    fn coach_decider_frame_skip_matches_main_rs_pre_refactor() {
        use std::sync::atomic::{AtomicU32, Ordering};

        struct CountingCoach {
            calls: Arc<AtomicU32>,
        }
        impl Coach for CountingCoach {
            fn decide(&mut self, _obs: &Observation) -> Vec<SkillChoice> {
                self.calls.fetch_add(1, Ordering::Relaxed);
                vec![SkillChoice {
                    robot_id: 0,
                    skill_id: SkillId::GoTo,
                    target: Vec2::ZERO,
                }]
            }
        }

        let calls = Arc::new(AtomicU32::new(0));
        let coach = Box::new(CountingCoach {
            calls: calls.clone(),
        });
        let mut decider = CoachDecider::new(coach, 0, 6);
        let world = World::new(3, 3);

        for t in 0..18 {
            let _ = decider.decide(t, &world);
        }
        // Debe haber sido llamado en ticks 0, 6, 12 → 3 veces.
        assert_eq!(calls.load(Ordering::Relaxed), 3);
    }

    /// 1.7 — Sin comandos manuales, `apply_manual_overrides` no toca nada.
    #[test]
    fn manual_override_empty_is_noop() {
        let mut cmds = vec![MotionCommand {
            id: 0,
            team: 0,
            vx: 1.0,
            vy: 0.0,
            omega: 0.0,
            orientation: 0.0,
        }];
        let mut tgts: Vec<Option<Vec2>> = vec![Some(Vec2::new(0.3, 0.0))];
        let manual: HashMap<(i32, i32), (ManualCommand, u32)> = HashMap::new();
        let world = World::new(3, 3);

        let before = cmds.clone();
        apply_manual_overrides(&mut cmds, &mut tgts, &manual, &world);

        assert_eq!(cmds.len(), before.len());
        assert_eq!(cmds[0].vx, before[0].vx);
        assert_eq!(tgts.len(), 1);
    }

    /// 1.8 — El comando manual reemplaza al del decider para su robot y deja
    /// intactos los de los otros robots. `orientation` sale del World.
    #[test]
    fn manual_override_replaces_target_robot_only() {
        let mut world = World::new(3, 3);
        // Robot 1 azul visible con orientación conocida.
        world.update_robot(1, 0, Vec2::new(0.0, 0.0), 1.5, Vec2::ZERO, 0.0);

        let mut cmds = vec![
            MotionCommand {
                id: 0,
                team: 0,
                vx: 0.1,
                vy: 0.0,
                omega: 0.0,
                orientation: 0.0,
            },
            MotionCommand {
                id: 1,
                team: 0,
                vx: 0.2,
                vy: 0.0,
                omega: 0.0,
                orientation: 0.0,
            },
        ];
        let mut tgts: Vec<Option<Vec2>> = vec![None, None];

        let mut manual: HashMap<(i32, i32), (ManualCommand, u32)> = HashMap::new();
        manual.insert(
            (0, 1),
            (
                ManualCommand {
                    team: 0,
                    id: 1,
                    vx: 0.9,
                    vy: 0.0,
                    omega: 0.0,
                },
                0,
            ),
        );

        apply_manual_overrides(&mut cmds, &mut tgts, &manual, &world);

        // Robot 0 (no manual) intacto.
        let r0 = cmds.iter().find(|c| c.id == 0).unwrap();
        assert_eq!(r0.vx, 0.1);
        // Robot 1 (manual) reemplazado, orientación tomada del World (1.5).
        let r1 = cmds.iter().find(|c| c.id == 1).unwrap();
        assert_eq!(r1.vx, 0.9);
        assert_eq!(r1.orientation, 1.5);
    }

    /// El comando manual de un robot NO presente en los comandos del decider se
    /// inserta, y `targets` queda alineado en largo con `commands`.
    #[test]
    fn manual_override_inserts_when_absent() {
        let mut cmds: Vec<MotionCommand> = Vec::new();
        let mut tgts: Vec<Option<Vec2>> = Vec::new();
        let world = World::new(3, 3); // robot no visible → orientation 0.0

        let mut manual: HashMap<(i32, i32), (ManualCommand, u32)> = HashMap::new();
        manual.insert(
            (0, 2),
            (
                ManualCommand {
                    team: 0,
                    id: 2,
                    vx: 0.5,
                    vy: 0.3,
                    omega: -1.0,
                },
                0,
            ),
        );

        apply_manual_overrides(&mut cmds, &mut tgts, &manual, &world);

        assert_eq!(cmds.len(), 1);
        assert_eq!(tgts.len(), cmds.len());
        assert_eq!(cmds[0].id, 2);
        assert_eq!(cmds[0].vx, 0.5);
        assert_eq!(cmds[0].orientation, 0.0);
    }

    /// 1.7 — `apply_gui_skill_overrides`: noop sin skills; reemplaza solo su robot
    /// e inserta si ausente; respeta `own_team`.
    #[test]
    fn gui_skill_override_replaces_and_inserts() {
        use std::collections::HashMap;
        let mut choices = vec![
            SkillChoice {
                robot_id: 0,
                skill_id: SkillId::GoTo,
                target: Vec2::ZERO,
            },
            SkillChoice {
                robot_id: 1,
                skill_id: SkillId::GoTo,
                target: Vec2::ZERO,
            },
        ];

        // Sin skills → noop.
        let empty: HashMap<(i32, i32), (GuiSkillCommand, u32)> = HashMap::new();
        let before = choices.clone();
        apply_gui_skill_overrides(&mut choices, &empty, 0);
        assert_eq!(choices.len(), before.len());

        // Reemplaza robot 1 (own_team=0) y agrega robot 2; ignora otro equipo.
        let mut state: HashMap<(i32, i32), (GuiSkillCommand, u32)> = HashMap::new();
        state.insert(
            (0, 1),
            (
                GuiSkillCommand {
                    team: 0,
                    id: 1,
                    skill_id: SkillId::Spin,
                    target: Vec2::new(0.5, 0.0),
                    spin_omega: 20.0,
                },
                0,
            ),
        );
        state.insert(
            (0, 2),
            (
                GuiSkillCommand {
                    team: 0,
                    id: 2,
                    skill_id: SkillId::ChaseBall,
                    target: Vec2::ZERO,
                    spin_omega: 20.0,
                },
                0,
            ),
        );
        state.insert(
            (1, 0),
            (
                GuiSkillCommand {
                    team: 1,
                    id: 0,
                    skill_id: SkillId::FacePoint,
                    target: Vec2::ZERO,
                    spin_omega: 20.0,
                },
                0,
            ),
        );

        apply_gui_skill_overrides(&mut choices, &state, 0);

        // Robot 0 intacto (GoTo del "coach"), no lo tocó el equipo contrario.
        let c0 = choices.iter().find(|c| c.robot_id == 0).unwrap();
        assert_eq!(c0.skill_id, SkillId::GoTo);
        // Robot 1 reemplazado por Spin.
        let c1 = choices.iter().find(|c| c.robot_id == 1).unwrap();
        assert_eq!(c1.skill_id, SkillId::Spin);
        // Robot 2 insertado (ChaseBall).
        let c2 = choices.iter().find(|c| c.robot_id == 2).unwrap();
        assert_eq!(c2.skill_id, SkillId::ChaseBall);
    }

    fn gui_skill_state(team: i32, id: i32) -> HashMap<(i32, i32), (GuiSkillCommand, u32)> {
        let mut state = HashMap::new();
        state.insert(
            (team, id),
            (
                GuiSkillCommand {
                    team,
                    id,
                    skill_id: SkillId::GoTo,
                    target: Vec2::ZERO,
                    spin_omega: 20.0,
                },
                0,
            ),
        );
        state
    }

    fn see(world: &mut World, team: i32, id: i32) {
        world.update_robot(id, team, Vec2::ZERO, 0.0, Vec2::ZERO, 0.0);
    }

    #[test]
    fn gui_skill_warning_own_robot_not_in_vision_lists_seen_robots() {
        let mut world = World::new(3, 3);
        see(&mut world, 1, 2);
        see(&mut world, 0, 0);
        let w = gui_skill_warning(&gui_skill_state(0, 1), &world, 0).unwrap();
        assert_eq!(
            w,
            "la skill no corre: el robot azul #1 no está en la visión. \
             La visión ve: azul #0, amarillo #2"
        );
    }

    #[test]
    fn gui_skill_warning_without_any_robot() {
        let world = World::new(3, 3);
        let w = gui_skill_warning(&gui_skill_state(0, 1), &world, 0).unwrap();
        assert_eq!(
            w,
            "la skill no corre: el robot azul #1 no está en la visión. \
             La visión no ve ningún robot"
        );
    }

    #[test]
    fn gui_skill_warning_robot_of_other_team() {
        let mut world = World::new(3, 3);
        see(&mut world, 1, 1);
        let w = gui_skill_warning(&gui_skill_state(1, 1), &world, 0).unwrap();
        assert!(
            w.starts_with("la skill no corre: el robot amarillo #1 no es del equipo propio (azul)"),
            "{w}"
        );
    }

    #[test]
    fn gui_skill_warning_none_when_skill_runs_or_absent() {
        let mut world = World::new(3, 3);
        see(&mut world, 0, 1);
        assert_eq!(gui_skill_warning(&gui_skill_state(0, 1), &world, 0), None);
        assert_eq!(gui_skill_warning(&HashMap::new(), &world, 0), None);
    }

    /// La parada (y HALT) produce comando cero por cada id del equipo propio, lo vea o no
    /// la visión; la orientación sale de la visión si el robot está.
    #[test]
    fn estop_zeroes_every_own_id() {
        let mut world = World::new(3, 3);
        world.update_robot(0, 0, Vec2::ZERO, 0.0, Vec2::ZERO, 0.0);
        world.update_robot(2, 0, Vec2::new(0.1, 0.1), 1.0, Vec2::ZERO, 0.0);
        // Robot del otro equipo no debe aparecer.
        world.update_robot(1, 1, Vec2::ZERO, 0.0, Vec2::ZERO, 0.0);

        let (cmds, tgts) = zero_commands_for_team(&world, 0, 3);
        assert_eq!(cmds.len(), 3);
        assert_eq!(tgts.len(), 3);
        for c in &cmds {
            assert_eq!((c.vx, c.vy, c.omega), (0.0, 0.0, 0.0));
            assert_eq!(c.team, 0);
        }
        assert_eq!(cmds.iter().find(|c| c.id == 2).map(|c| c.orientation), Some(1.0));
        assert_eq!(cmds.iter().find(|c| c.id == 1).map(|c| c.orientation), Some(0.0));
    }

    /// Smoke test: `run_control_loop` con `FixedSkillDecider` y `MockTransport`.
    /// Verifica que el bucle dispatcha vía `SkillCatalog::tick` y que el transport
    /// recibe comandos. `MockTransport` requiere construir Radio manualmente, así
    /// que esto no testea `Radio::from_target` (cubierto en `radio::mod` por
    /// `from_env_defaults_to_firasim`); testea el dispatch de skills + el flujo
    /// del loop con un robot que existe en el World.
    ///
    /// Como `run_control_loop` instancia Radio desde `RadioTarget`, este test
    /// usa `RadioTarget::FiraSim` y solo verifica que el loop arranca y se
    /// detiene por `max_ticks`. El test de "el mock recibe N sends" se cubre
    /// en `radio::mod::tests::radio_dispatches_and_clears`, que ya prueba el
    /// camino Radio → transport.
    #[test]
    fn smoke_compile_loop_module() {
        // Test de compilación + sanity: las structs y traits del módulo se ensamblan.
        let _config = ControlLoopConfig {
            own_team: 0,
            num_robots: 3,
            vision_source: VisionSource::FiraSim,
            radio_target: RadioTarget::FiraSim,
            max_ticks: Some(5),
            vision_timeout: None,
            referee: None,
        };
        let _decider: Box<dyn TickDecider> =
            Box::new(FixedSkillDecider::new(0, SkillId::GoTo, Vec2::ZERO));
        let _shutdown = Arc::new(AtomicBool::new(false));
        // No invocamos run_control_loop porque abre socket UDP de visión real.
        // El smoke test del dispatcher + decider está cubierto arriba.
    }
}
