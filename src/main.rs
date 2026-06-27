// Modo base: headless por defecto.
// Con VSSL_DEBUG_GUI=1 lanza la interfaz gráfica con vectores de velocidad y targets.

use rustengine::GUI;

// ====== Parámetros ======
// Equipo propio configurable por VSSL_TEAM_COLOR (ver own_team_from_env). Default azul.
const NUM_ROBOTS: usize = 3;

// Frame-skip del coach (Fase 3 — opción E del horizonte de decisión).
// El coach se consulta una vez cada COACH_DECISION_PERIOD ticks; entre medio
// el dispatcher reusa la última decisión y solo refresca las skills (que sí
// corren a 60 Hz para suavidad de control). Con período=6 a 60 Hz, la policy
// decide a 10 Hz (≈ 100 ms por decisión), valor estándar en VSSS-RL.
const COACH_DECISION_PERIOD: u32 = 6;
// ========================

use glam::Vec2;
use rustengine::coach::{Coach, RuleBasedCoach, SkillChoice};
use rustengine::control_loop::{
    CoachDecider, ControlLoopConfig, GuiChannels, TickDecider, run_control_loop,
};
use rustengine::radio::RadioTarget;
use rustengine::vision::VisionSource;
use rustengine::world::World;

/// Decider que nunca emite skills. Preserva el modo `VSSL_COACH=none` del
/// pre-refactor: visión y radio siguen vivos, pero no se mandan comandos.
struct NoOpDecider;
impl TickDecider for NoOpDecider {
    fn decide(&mut self, _tick: u32, _world: &World) -> Vec<SkillChoice> {
        Vec::new()
    }
}

/// Direcciones de ataque convencionales según el equipo propio.
/// Azul ataca a +X, amarillo ataca a -X.
fn goals_for_team(own_team: i32) -> (Vec2, Vec2) {
    if own_team == 0 {
        (Vec2::new(0.75, 0.0), Vec2::new(-0.75, 0.0))
    } else {
        (Vec2::new(-0.75, 0.0), Vec2::new(0.75, 0.0))
    }
}

/// Equipo propio desde `VSSL_TEAM_COLOR` (blue|yellow). Default azul (0).
/// Permite correr DOS instancias (una por color) para un partido completo:
/// p.ej. RL en azul vs rule-based en amarillo.
fn own_team_from_env() -> i32 {
    match std::env::var("VSSL_TEAM_COLOR").as_deref() {
        Ok("yellow") | Ok("amarillo") | Ok("1") => 1,
        _ => 0,
    }
}

/// Construye el coach inicial. Soporta selección por env var:
/// - `VSSL_COACH=rule_based` (default): baseline clásico.
/// - `VSSL_COACH=none`: no emite decisiones (útil para test de visión/radio).
///
/// El día que llegue Fase 7, este factory va a aceptar también `VSSL_COACH=rl`
/// para cargar un modelo ONNX.
fn make_coach(own_team: i32) -> Option<Box<dyn Coach>> {
    let kind = std::env::var("VSSL_COACH").unwrap_or_else(|_| "rule_based".to_string());
    let (attack_goal, own_goal) = goals_for_team(own_team);
    match kind.as_str() {
        "rule_based" => Some(Box::new(RuleBasedCoach::new(attack_goal, own_goal))),
        "none" => None,
        "rl" => make_rl_coach(attack_goal, own_goal),
        other => {
            eprintln!("[main] VSSL_COACH='{other}' inválido, usando 'rule_based'");
            Some(Box::new(RuleBasedCoach::new(attack_goal, own_goal)))
        }
    }
}

/// Carga el RlCoach (modelo ONNX) cuando el binario se compiló con `--features rl`.
/// Path del modelo vía `VSSL_RL_MODEL` (default: training/models/policy.onnx).
/// Si no se compiló con la feature o falla la carga, cae a rule_based.
#[cfg(feature = "rl")]
fn make_rl_coach(attack_goal: Vec2, own_goal: Vec2) -> Option<Box<dyn Coach>> {
    let path =
        std::env::var("VSSL_RL_MODEL").unwrap_or_else(|_| "training/models/policy.onnx".to_string());
    // Arquero entrenado: por defecto training/models/policy_gk.onnx (si carga).
    // Poné VSSL_RL_GK_MODEL=none para forzar el arquero rule-based.
    let gk_env = std::env::var("VSSL_RL_GK_MODEL")
        .unwrap_or_else(|_| "training/models/policy_gk.onnx".to_string());
    let gk_path: Option<&str> = if gk_env == "none" { None } else { Some(gk_env.as_str()) };
    match rustengine::coach::RlCoach::load(&path, gk_path, own_goal, true) {
        Ok(c) => {
            eprintln!("[main] RlCoach cargado desde {path}");
            Some(Box::new(c))
        }
        Err(e) => {
            eprintln!("[main] error cargando RlCoach ({e}); usando rule_based");
            Some(Box::new(RuleBasedCoach::new(attack_goal, own_goal)))
        }
    }
}

#[cfg(not(feature = "rl"))]
fn make_rl_coach(attack_goal: Vec2, own_goal: Vec2) -> Option<Box<dyn Coach>> {
    eprintln!("[main] VSSL_COACH=rl pero el binario no se compiló con --features rl; usando rule_based");
    Some(Box::new(RuleBasedCoach::new(attack_goal, own_goal)))
}

use std::sync::{Arc, atomic::AtomicBool};
use tokio::sync::mpsc;

fn main() {
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
    let (status_tx, status_rx) = mpsc::channel(100);
    let (motion_tx, motion_rx) = mpsc::channel::<Vec<GUI::RobotMotionDebug>>(32);
    let (config_tx, _config_rx) = mpsc::channel::<GUI::ConfigUpdate>(8);

    let source = VisionSource::from_env();
    let ip = source.multicast_ip().to_string();
    let port = source.port();

    std::thread::spawn(move || {
        let rt = tokio::runtime::Runtime::new().unwrap();
        rt.block_on(async_main(Some((status_tx, motion_tx))));
    });

    GUI::run_gui(ip, port, config_tx, status_rx, motion_rx).expect("GUI terminó con error");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Pipeline principal: wrapper sobre `run_control_loop` del módulo común.
//
//  Comportamiento observable IDÉNTICO al pre-refactor:
//    - Lee env (VSSL_COACH, VSSL_VISION_SOURCE, VSSL_RADIO_TARGET) una sola vez.
//    - Arma un CoachDecider con frame-skip COACH_DECISION_PERIOD=6.
//    - Delega en run_control_loop (60 Hz, dispatcher idéntico al pre-refactor).
//    - GUI recibe los mismos RobotMotionDebug por canal.
//
//  Si necesitas cambiar el lazo de control, edita src/control_loop.rs — esta
//  función solo arma la configuración.
// ─────────────────────────────────────────────────────────────────────────────
async fn async_main(
    gui_channels: Option<(
        mpsc::Sender<GUI::StatusUpdate>,
        mpsc::Sender<Vec<GUI::RobotMotionDebug>>,
    )>,
) {
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
        "[main] equipo propio: {}",
        if own_team == 0 { "azul (blue)" } else { "amarillo (yellow)" }
    );
    let coach = make_coach(own_team);
    eprintln!(
        "[main] coach: {}",
        if coach.is_some() {
            std::env::var("VSSL_COACH").unwrap_or_else(|_| "rule_based".to_string())
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
        Some(c) => Box::new(CoachDecider::new(c, OWN_TEAM, COACH_DECISION_PERIOD)),
        None => Box::new(NoOpDecider),
    };

    let config = ControlLoopConfig {
        own_team,
        num_robots: NUM_ROBOTS,
        vision_source,
        radio_target,
        max_ticks: None,
        vision_timeout: None,
    };

    let gui = gui_channels.map(|(status_tx, motion_tx)| GuiChannels {
        status_tx,
        motion_tx,
    });

    let shutdown = Arc::new(AtomicBool::new(false));

    if let Err(err) = run_control_loop(config, decider, None, gui, shutdown).await {
        eprintln!("[main] control loop error: {err}");
    }
}
