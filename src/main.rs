// Modo base: headless por defecto.
// Con VSSL_DEBUG_GUI=1 lanza la interfaz gráfica con vectores de velocidad y targets.

use rustengine::GUI;

// ====== Parámetros ======
const NUM_ROBOTS: usize = 3;

// Frame-skip del coach (Fase 3 — opción E del horizonte de decisión).
// El coach se consulta una vez cada COACH_DECISION_PERIOD ticks; entre medio
// el dispatcher reusa la última decisión y solo refresca las skills (que sí
// corren a 60 Hz para suavidad de control). Con período=6 a 60 Hz, la policy
// decide a 10 Hz (≈ 100 ms por decisión), valor estándar en VSSS-RL.
const COACH_DECISION_PERIOD: u32 = 6;
// ========================

use glam::Vec2;
use rustengine::coach::{
    Coach, HeuristicCoach, RuleBasedCoach, SharedReferee, SkillChoice, new_shared_referee,
    run_referee_listener,
};
use rustengine::control_loop::{
    CoachDecider, ControlLoopConfig, GuiChannels, GuiSkillCommand, HeadingPid, ManualCommand,
    TickDecider, run_control_loop,
};
use rustengine::radio::{RadioTarget, TeamColor};
use rustengine::skill_log::MatchLogger;
use rustengine::vision::VisionSource;
use rustengine::world::World;

/// Equipo propio: `VSSL_TEAM_COLOR=blue|yellow` (default azul). Permite correr
/// dos engines en la misma máquina, uno por equipo, para partidos en simulador.
fn own_team_from_env() -> i32 {
    TeamColor::from_env().as_team_id()
}

/// Decider que nunca emite skills. Preserva el modo `VSSL_COACH=none` del
/// pre-refactor: visión y radio siguen vivos, pero no se mandan comandos.
struct NoOpDecider;
impl TickDecider for NoOpDecider {
    fn decide(&mut self, _tick: u32, _world: &World) -> Vec<SkillChoice> {
        Vec::new()
    }
}

/// Arcos (ataque, propio) según el lado que defendemos. `VSSL_SIDE=left|right`
/// dice en qué arco está nuestro portero (cambia en el segundo tiempo y según
/// cómo esté configurado el simulador); por defecto azul defiende el izquierdo
/// (ataca a +X) y amarillo el derecho. Con el lado equivocado el equipo ataca su
/// propio arco y en FIRASim los robots terminan cruzando el arco rival a toda
/// velocidad y rompiendo la física.
fn goals_for_team(own_team: i32) -> (Vec2, Vec2) {
    let defend_left = rustengine::skills::zones::defend_left_from_env(own_team);
    eprintln!(
        "[main] lado propio: {} (VSSL_SIDE) → atacamos hacia {}",
        if defend_left { "izquierdo" } else { "derecho" },
        if defend_left { "+X" } else { "-X" }
    );
    if defend_left {
        (Vec2::new(0.75, 0.0), Vec2::new(-0.75, 0.0))
    } else {
        (Vec2::new(-0.75, 0.0), Vec2::new(0.75, 0.0))
    }
}

/// Construye el coach inicial. Soporta selección por env var:
/// - `VSSL_COACH=heuristic` (default): equipo heurístico STP (roles dinámicos
///   + tácticas sobre el catálogo completo, robot de dos caras).
/// - `VSSL_COACH=rule_based`: baseline clásico de roles fijos (regresión).
/// - `VSSL_COACH=none`: no emite decisiones (útil para test de visión/radio).
fn make_coach(own_team: i32, referee: &SharedReferee) -> Option<Box<dyn Coach>> {
    let kind = std::env::var("VSSL_COACH").unwrap_or_else(|_| "heuristic".to_string());
    let (attack_goal, own_goal) = goals_for_team(own_team);
    let heuristic = || {
        let mut c = HeuristicCoach::new(attack_goal, own_goal);
        c.set_referee(referee.clone());
        Box::new(c) as Box<dyn Coach>
    };
    match kind.as_str() {
        "heuristic" => Some(heuristic()),
        "rule_based" => Some(Box::new(RuleBasedCoach::new(attack_goal, own_goal))),
        "none" => None,
        other => {
            eprintln!("[main] VSSL_COACH='{other}' inválido, usando 'heuristic'");
            Some(heuristic())
        }
    }
}

use std::sync::{Arc, atomic::AtomicBool};
use tokio::sync::mpsc;

fn main() {
    // Parámetros calibrables (config/team_params.json o VSSL_PARAMS) antes de todo.
    rustengine::params::TeamParams::install_or_exit("main");
    if std::env::var("VSSL_DEBUG_GUI").unwrap_or_default() == "1" {
        run_with_gui();
    } else {
        let rt = tokio::runtime::Runtime::new().unwrap();
        rt.block_on(async_main(None));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Modo debug: GUI en el hilo principal, tokio en background
// ─────────────────────────────────────────────────────────────────────────────
fn run_with_gui() {
    // Foto de cada tick, lazo → GUI (canal de "último valor": nunca frena el lazo).
    let (snapshot_tx, snapshot_rx) = rustengine::snapshot::snapshot_channel();
    let (manual_tx, manual_rx) = mpsc::channel::<ManualCommand>(32);
    let (skill_tx, skill_rx) = mpsc::channel::<GuiSkillCommand>(32);
    let (pid_tx, pid_rx) = mpsc::channel::<HeadingPid>(16);
    let (teleport_tx, teleport_rx) =
        mpsc::channel::<Vec<rustengine::radio::TeleportItem>>(16);
    let estop = Arc::new(AtomicBool::new(false));

    // Configuración de la corrida para la barra de estado y el Inspector (solo lectura).
    let run = GUI::RunInfo::from_env(
        RadioTarget::from_env(),
        VisionSource::from_env(),
        own_team_from_env(),
        NUM_ROBOTS,
        GUI::CoachKind::from_env(),
    );

    let estop_loop = estop.clone();
    std::thread::spawn(move || {
        let rt = tokio::runtime::Runtime::new().unwrap();
        rt.block_on(async_main(Some((
            snapshot_tx, manual_rx, skill_rx, estop_loop, pid_rx, teleport_rx,
        ))));
    });

    let setup = GUI::GuiSetup {
        snapshot_rx,
        manual_tx: Some(manual_tx),
        skill_tx: Some(skill_tx),
        pid_tx: Some(pid_tx),
        teleport_tx: Some(teleport_tx),
        estop,
        run,
    };
    GUI::run_gui(setup).expect("GUI terminó con error");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Pipeline principal: wrapper sobre `run_control_loop` del módulo común.
//
//  Comportamiento observable IDÉNTICO al pre-refactor:
//    - Lee env (VSSL_COACH, VSSL_VISION_SOURCE, VSSL_RADIO_TARGET) una sola vez.
//    - Arma un CoachDecider con frame-skip COACH_DECISION_PERIOD=6.
//    - Delega en run_control_loop (60 Hz, dispatcher idéntico al pre-refactor).
//    - Con GUI, el lazo publica una foto por tick (`rustengine::snapshot`).
//
//  Si necesitas cambiar el lazo de control, edita src/control_loop.rs — esta
//  función solo arma la configuración.
// ─────────────────────────────────────────────────────────────────────────────
/// Canales GUI→loop / loop→GUI que `run_with_gui` pasa a `async_main`.
type GuiChannelBundle = (
    rustengine::snapshot::SnapshotSender,
    mpsc::Receiver<ManualCommand>,
    mpsc::Receiver<GuiSkillCommand>,
    Arc<AtomicBool>,
    mpsc::Receiver<HeadingPid>,
    mpsc::Receiver<Vec<rustengine::radio::TeleportItem>>,
);

async fn async_main(gui_channels: Option<GuiChannelBundle>) {
    let vision_source = VisionSource::from_env();
    let radio_target = RadioTarget::from_env();
    eprintln!(
        "[main] visión: {:?} ({}:{})",
        vision_source,
        vision_source.multicast_ip(),
        vision_source.port()
    );

    let own_team = own_team_from_env();
    eprintln!(
        "[main] equipo propio: {} (VSSL_TEAM_COLOR)",
        if own_team == 0 { "azul" } else { "amarillo" }
    );
    // Árbitro (VSSReferee o texto del operador, VSSL_REFEREE_ADDR): estado
    // compartido que el coach heurístico lee en cada decisión.
    let referee = new_shared_referee();
    tokio::spawn(run_referee_listener(referee.clone()));

    let coach = make_coach(own_team, &referee);
    eprintln!(
        "[main] coach: {}",
        if coach.is_some() {
            std::env::var("VSSL_COACH").unwrap_or_else(|_| "heuristic".to_string())
        } else {
            "none".to_string()
        }
    );
    eprintln!(
        "[main] decisión cada {} ticks ({:.0} Hz)",
        COACH_DECISION_PERIOD,
        60.0 / COACH_DECISION_PERIOD as f64
    );

    let decider: Box<dyn TickDecider> = match coach {
        Some(c) => Box::new(CoachDecider::new(c, own_team, COACH_DECISION_PERIOD)),
        None => Box::new(NoOpDecider),
    };

    // Registro de partido (CSV por tick, propios y rivales) con VSSL_MATCH_LOG=ruta.
    let on_tick = match std::env::var("VSSL_MATCH_LOG") {
        Ok(path) if !path.trim().is_empty() => match MatchLogger::new(&path, own_team) {
            Ok(mut logger) => {
                eprintln!("[main] registro de partido → {path}");
                Some(Box::new(move |rec: &rustengine::control_loop::TickRecord<'_>| {
                    let _ = logger.write_tick(rec);
                }) as rustengine::control_loop::OnTick)
            }
            Err(e) => {
                eprintln!("[main] no se pudo abrir VSSL_MATCH_LOG={path}: {e}");
                None
            }
        },
        _ => None,
    };

    let config = ControlLoopConfig {
        own_team,
        num_robots: NUM_ROBOTS,
        vision_source,
        radio_target,
        max_ticks: None,
        vision_timeout: None,
        referee: Some(referee.clone()),
    };

    let gui = gui_channels.map(
        |(snapshot_tx, manual_rx, skill_rx, estop, pid_rx, teleport_rx)| GuiChannels {
            snapshot_tx,
            manual_rx: Some(manual_rx),
            skill_rx: Some(skill_rx),
            estop: Some(estop),
            pid_rx: Some(pid_rx),
            teleport_rx: Some(teleport_rx),
        },
    );

    let shutdown = Arc::new(AtomicBool::new(false));

    if let Err(err) = run_control_loop(config, decider, on_tick, gui, shutdown).await {
        eprintln!("[main] control loop error: {err}");
    }
}
