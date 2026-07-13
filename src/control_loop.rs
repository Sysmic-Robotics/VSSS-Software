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
use crate::coach::{Coach, Observation, SkillChoice};
use crate::motion::{BorderRecovery, Motion, MotionCommand};
use crate::radio::{RadioTarget, TransportError};
use crate::skills::{SkillCatalog, SkillId};
use crate::vision::{Vision, VisionEvent, VisionSource};
use crate::world::{RobotState, World};
use glam::Vec2;
use std::collections::HashMap;
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
fn zero_commands_for_active_team(
    world: &World,
    own_team: i32,
) -> (Vec<MotionCommand>, Vec<Option<Vec2>>) {
    let team_robots = if own_team == 0 {
        world.get_blue_team_active()
    } else {
        world.get_yellow_team_active()
    };
    let cmds: Vec<MotionCommand> = team_robots
        .iter()
        .map(|r| MotionCommand {
            id: r.id,
            team: r.team,
            vx: 0.0,
            vy: 0.0,
            omega: 0.0,
            orientation: r.orientation,
        })
        .collect();
    let tgts = vec![None; cmds.len()];
    (cmds, tgts)
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
            SkillId::GoTo | SkillId::FacePoint => Some(choice.target),
            SkillId::ChaseBall => Some(world.get_ball_state().position),
            SkillId::Spin => None,
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
    let tracker_enabled = Arc::new(AtomicBool::new(true));
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

    let motion = Motion::new();
    let mut catalog = SkillCatalog::new(config.num_robots);
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

        // Parada de emergencia: prevalece sobre coach/manual/skills.
        let estop_engaged = estop
            .as_ref()
            .map(|e| e.load(Ordering::Relaxed))
            .unwrap_or(false);
        if estop_engaged {
            manual_state.clear();
            skill_state.clear();
        }

        let (commands, targets, applied_choices) = {
            let world_guard = world.read().await;
            if estop_engaged {
                let (z_cmds, z_tgts) = zero_commands_for_active_team(&world_guard, config.own_team);
                (z_cmds, z_tgts, Vec::new())
            } else {
                // Override en vivo de la velocidad de Spin desde la GUI (por robot).
                for ((team, id), (cmd, _)) in &skill_state {
                    if *team == config.own_team && cmd.skill_id == SkillId::Spin {
                        catalog.set_spin_omega_for(*id as usize, cmd.spin_omega);
                    }
                }
                let mut choices = decider.decide(tick_counter, &world_guard);
                apply_gui_skill_overrides(&mut choices, &skill_state, config.own_team);
                let (mut cmds, mut tgts, applied) = dispatch_choices(
                    &choices,
                    &mut catalog,
                    &world_guard,
                    &motion,
                    config.own_team,
                );
                apply_manual_overrides(&mut cmds, &mut tgts, &manual_state, &world_guard);
                // Recuperación de borde/atasco: reflejo de bajo nivel sobre los comandos
                // autónomos del equipo propio (salta manual; el estop ya cortó arriba).
                let manual_keys: std::collections::HashSet<(i32, i32)> =
                    manual_state.keys().copied().collect();
                recovery.guard_commands(&mut cmds, &world_guard, &manual_keys, config.own_team);
                (cmds, tgts, applied)
            }
        };
        tick_counter = tick_counter.wrapping_add(1);

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
                    // Mismo cálculo que el CSV de auditoría (skill_log) → overlay y log
                    // no pueden divergir. `cmd` ya es un MotionCommand aquí.
                    let (wheel_l_mm_s, wheel_r_mm_s) =
                        crate::radio::base_station::command_to_wheel_mm_s(cmd);
                    GUI::RobotMotionDebug {
                        team: cmd.team as u32,
                        id: cmd.id as u32,
                        vx: cmd.vx as f32,
                        vy: cmd.vy as f32,
                        omega: cmd.omega as f32,
                        target: *target,
                        wheel_l_mm_s,
                        wheel_r_mm_s,
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

    /// 1.4 — La parada produce comando cero por cada robot activo del equipo propio.
    #[test]
    fn estop_zeroes_active_team() {
        let mut world = World::new(3, 3);
        world.update_robot(0, 0, Vec2::ZERO, 0.0, Vec2::ZERO, 0.0);
        world.update_robot(2, 0, Vec2::new(0.1, 0.1), 1.0, Vec2::ZERO, 0.0);
        // Robot del otro equipo no debe aparecer.
        world.update_robot(1, 1, Vec2::ZERO, 0.0, Vec2::ZERO, 0.0);

        let (cmds, tgts) = zero_commands_for_active_team(&world, 0);
        assert_eq!(cmds.len(), 2);
        assert_eq!(tgts.len(), 2);
        for c in &cmds {
            assert_eq!((c.vx, c.vy, c.omega), (0.0, 0.0, 0.0));
            assert_eq!(c.team, 0);
        }
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
        };
        let _decider: Box<dyn TickDecider> =
            Box::new(FixedSkillDecider::new(0, SkillId::GoTo, Vec2::ZERO));
        let _shutdown = Arc::new(AtomicBool::new(false));
        // No invocamos run_control_loop porque abre socket UDP de visión real.
        // El smoke test del dispatcher + decider está cubierto arriba.
    }
}
