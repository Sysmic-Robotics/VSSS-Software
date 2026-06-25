//! Banco de pruebas "editar y correr" para las 4 skills congeladas.
//!
//! Comparte el lazo de control con `main.rs` (vía `rustengine::control_loop::run_control_loop`),
//! la cinemática inversa, el dispatcher de skills y el formato CSV con `skill_test`. La única
//! diferencia con producción es el decisor: este banco usa `FixedSkillDecider` parametrizado
//! por la línea descomentada en la zona de edición.
//!
//! Para auditar visualmente, la GUI se lanza SIEMPRE (sin `VSSL_DEBUG_GUI`). El CSV se escribe
//! SIEMPRE a `logs/scenario_<skill>_<YYYYmmdd_HHMMSS>.csv`.
//!
//! Referencias: `VSSS-Software/docs/probador_skills_diseno.md`, `CLAUDE.md §5`.

use glam::Vec2;
use rustengine::GUI;
use rustengine::control_loop::{
    ControlLoopConfig, FixedSkillDecider, GuiChannels, TickRecord, run_control_loop,
};
use rustengine::radio::{RadioTarget, TeamColor, base_station::SLOT_COUNT};
use rustengine::skill_log::{CsvLogger, SkillLogCtx, format_human_summary};
use rustengine::skills::SkillId;
use rustengine::vision::VisionSource;
use std::fs;
use std::path::PathBuf;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use tokio::sync::mpsc;

// ─────────────────────────────────────────────────────────────────────────────
//  Scenario: wrapper privado al bin sobre (SkillId, Vec2).
//  Codifica la convención de cada skill (CLAUDE.md §5).
// ─────────────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy)]
struct Scenario {
    skill_id: SkillId,
    target: Vec2,
    name: &'static str,
}

// Los 5 constructores son la "zona de edición": uno está activo en main()
// y los otros están comentados. Rust no ve los comentados → suprimimos dead_code.
#[allow(dead_code)]
impl Scenario {
    fn go_to(target: Vec2) -> Self {
        Self {
            skill_id: SkillId::GoTo,
            target,
            name: "go_to",
        }
    }
    fn face_point(target: Vec2) -> Self {
        Self {
            skill_id: SkillId::FacePoint,
            target,
            name: "face_point",
        }
    }
    fn chase_ball() -> Self {
        Self {
            skill_id: SkillId::ChaseBall,
            target: Vec2::ZERO,
            name: "chase_ball",
        }
    }
    fn spin_ccw() -> Self {
        // Convención: target.x > 0 → CCW (omega > 0).
        Self {
            skill_id: SkillId::Spin,
            target: Vec2::new(1.0, 0.0),
            name: "spin_ccw",
        }
    }
    fn spin_cw() -> Self {
        // Convención: target.x < 0 → CW (omega < 0).
        Self {
            skill_id: SkillId::Spin,
            target: Vec2::new(-1.0, 0.0),
            name: "spin_cw",
        }
    }
}

/// Default del log_path: `logs/scenario_<skill>_<epoch_secs>.csv`.
/// Crea el directorio `logs/` si no existe.
///
/// `<epoch_secs>` es Unix epoch en segundos (numérico). Sort por filename = sort
/// por tiempo. Evita la dep `chrono` por una formatting humana del timestamp.
fn scenario_log_path(scenario: &Scenario) -> PathBuf {
    let _ = fs::create_dir_all("logs");
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    PathBuf::from(format!("logs/scenario_{}_{}.csv", scenario.name, secs))
}

// ─────────────────────────────────────────────────────────────────────────────
//  Entry point
// ─────────────────────────────────────────────────────────────────────────────

fn main() {
    // ╔══════════════════════════════════════════════════════════════════════╗
    // ║  ZONA DE EDICIÓN — descomenta UNA línea para elegir la skill         ║
    // ╚══════════════════════════════════════════════════════════════════════╝

    let scenario = Scenario::go_to(Vec2::new(0.3, 0.0));
    //                  navega a (0.3, 0); audita err_dist → 0 y pose_x → 0.3
    // let scenario = Scenario::face_point(Vec2::new(0.5, 0.0));
    //                  rota a mirar (0.5, 0); audita err_heading → 0
    // let scenario = Scenario::chase_ball();
    //                  persigue la pelota; ponla en FIRASim antes de correr
    // let scenario = Scenario::spin_ccw();
    //                  gira CCW; audita cmd_omega > 0
    // let scenario = Scenario::spin_cw();
    //                  gira CW; audita cmd_omega < 0

    let robot_id: i32 = 0;
    let team: TeamColor = TeamColor::Blue;
    let vision: VisionSource = VisionSource::FiraSim;
    let transport: RadioTarget = RadioTarget::FiraSim;
    let log: Option<PathBuf> = Some(scenario_log_path(&scenario));
    //                  None = no escribe CSV (la GUI y el resumen humano 1 Hz a stderr siguen).
    let dur_s: Option<f64> = None; // None = corre hasta cerrar la ventana o Ctrl-C

    // ╔══════════════════════════════════════════════════════════════════════╗
    // ║  FIN ZONA DE EDICIÓN — abajo viene el wiring estándar                ║
    // ╚══════════════════════════════════════════════════════════════════════╝
    run_bench(scenario, robot_id, team, vision, transport, log, dur_s);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Wiring: GUI en hilo principal, control loop en background (mismo patrón
//  que `src/main.rs::run_with_gui`).
// ─────────────────────────────────────────────────────────────────────────────

fn run_bench(
    scenario: Scenario,
    robot_id: i32,
    team: TeamColor,
    vision: VisionSource,
    transport: RadioTarget,
    log: Option<PathBuf>,
    dur_s: Option<f64>,
) {
    let (status_tx, status_rx) = mpsc::channel(100);
    let (motion_tx, motion_rx) = mpsc::channel::<Vec<GUI::RobotMotionDebug>>(32);
    let (config_tx, _config_rx) = mpsc::channel::<GUI::ConfigUpdate>(8);

    let ip = vision.multicast_ip().to_string();
    let port = vision.port();

    let log_desc = log
        .as_ref()
        .map(|p| p.display().to_string())
        .unwrap_or_else(|| "(off)".to_string());
    eprintln!(
        "[scenario] skill={} robot={} team={:?} transport={:?} vision={:?} log={}",
        scenario.name, robot_id, team, transport, vision, log_desc
    );

    std::thread::spawn(move || {
        let rt = tokio::runtime::Runtime::new().expect("tokio runtime");
        rt.block_on(async_run(
            scenario, robot_id, team, vision, transport, log, dur_s, status_tx, motion_tx,
        ));
    });

    GUI::run_gui(ip, port, config_tx, status_rx, motion_rx).expect("GUI terminó con error");
}

#[allow(clippy::too_many_arguments)]
async fn async_run(
    scenario: Scenario,
    robot_id: i32,
    team: TeamColor,
    vision: VisionSource,
    transport: RadioTarget,
    log: Option<PathBuf>,
    dur_s: Option<f64>,
    status_tx: mpsc::Sender<GUI::StatusUpdate>,
    motion_tx: mpsc::Sender<Vec<GUI::RobotMotionDebug>>,
) {
    let shutdown = Arc::new(AtomicBool::new(false));
    {
        let shutdown = shutdown.clone();
        tokio::spawn(async move {
            if tokio::signal::ctrl_c().await.is_ok() {
                eprintln!("[scenario] Ctrl-C recibido, deteniendo...");
                shutdown.store(true, Ordering::Relaxed);
            }
        });
    }

    let decider = Box::new(FixedSkillDecider::new(
        robot_id,
        scenario.skill_id,
        scenario.target,
    ));

    let max_ticks = dur_s.map(|s| (s * 60.0).ceil() as u32);

    let config = ControlLoopConfig {
        own_team: team.as_team_id(),
        num_robots: SLOT_COUNT,
        vision_source: vision,
        radio_target: transport,
        max_ticks,
        vision_timeout: None,
    };

    let mut csv: Option<CsvLogger> = match &log {
        Some(p) => match CsvLogger::new(p) {
            Ok(l) => Some(l),
            Err(e) => {
                eprintln!("[scenario] error abriendo CSV {}: {e}", p.display());
                return;
            }
        },
        None => {
            eprintln!("[scenario] CSV desactivado (log=None)");
            None
        }
    };
    let mut last_print = Instant::now() - Duration::from_secs(2);

    let ctx = SkillLogCtx {
        transport,
        vision: Some(vision),
        robot: robot_id as usize,
        team,
        skill: scenario.skill_id,
    };

    let on_tick: Box<dyn FnMut(&TickRecord<'_>) + Send> = Box::new(move |rec: &TickRecord<'_>| {
        let row = ctx.build_skill_row(rec);
        if let Some(ref mut log) = csv {
            let _ = log.write_row(&row);
        }
        if last_print.elapsed() >= Duration::from_secs(1) {
            eprintln!("{}", format_human_summary(&row));
            last_print = Instant::now();
        }
    });

    let gui = Some(GuiChannels {
        status_tx,
        motion_tx,
    });

    if let Err(err) = run_control_loop(config, decider, Some(on_tick), gui, shutdown).await {
        eprintln!("[scenario] control loop error: {err}");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn go_to_sets_target_and_skill() {
        let s = Scenario::go_to(Vec2::new(0.3, 0.5));
        assert_eq!(s.skill_id, SkillId::GoTo);
        assert_eq!(s.target, Vec2::new(0.3, 0.5));
        assert_eq!(s.name, "go_to");
    }

    #[test]
    fn face_point_sets_target_and_skill() {
        let s = Scenario::face_point(Vec2::new(0.7, -0.2));
        assert_eq!(s.skill_id, SkillId::FacePoint);
        assert_eq!(s.target, Vec2::new(0.7, -0.2));
        assert_eq!(s.name, "face_point");
    }

    #[test]
    fn chase_ball_ignores_target() {
        let s = Scenario::chase_ball();
        assert_eq!(s.skill_id, SkillId::ChaseBall);
        assert_eq!(s.target, Vec2::ZERO);
        assert_eq!(s.name, "chase_ball");
    }

    #[test]
    fn spin_ccw_target_x_positive() {
        let s = Scenario::spin_ccw();
        assert_eq!(s.skill_id, SkillId::Spin);
        assert!(s.target.x > 0.0, "spin_ccw target.x debe ser > 0");
        assert_eq!(s.name, "spin_ccw");
    }

    #[test]
    fn spin_cw_target_x_negative() {
        let s = Scenario::spin_cw();
        assert_eq!(s.skill_id, SkillId::Spin);
        assert!(s.target.x < 0.0, "spin_cw target.x debe ser < 0");
        assert_eq!(s.name, "spin_cw");
    }

    #[test]
    fn scenario_log_path_creates_dir_and_uses_name() {
        let s = Scenario::go_to(Vec2::ZERO);
        let path = scenario_log_path(&s);
        // El path comienza con "logs/scenario_go_to_"
        let path_str = path.to_string_lossy();
        assert!(
            path_str.starts_with("logs/scenario_go_to_"),
            "got: {path_str}"
        );
        assert!(path_str.ends_with(".csv"), "got: {path_str}");
        // El directorio logs/ debe existir tras la llamada
        assert!(std::path::Path::new("logs").exists());
    }
}
