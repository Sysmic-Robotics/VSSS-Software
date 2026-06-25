//! Probador de skills + bring-up de hardware.
//!
//! Ejecuta una skill (lazo cerrado, reusando exactamente el mismo loop que `main`)
//! o velocidades de rueda directas en mm/s (lazo abierto, solo `--transport base-station`)
//! con la misma CLI en simulador y en robot real.
//!
//! Ver `cargo run --bin skill_test -- --help` para uso.

use glam::Vec2;
use rustengine::control_loop::{
    ControlLoopConfig, FixedSkillDecider, GuiChannels, TickRecord, run_control_loop,
};
use rustengine::radio::{
    BaseStationTransport, RadioTarget, TeamColor, base_station::SLOT_COUNT,
    base_station::build_frame_from_wheels,
};
use rustengine::skill_log::{
    CsvLogger, CsvRow, SkillLogCtx, format_human_summary, team_label, transport_label,
};
use rustengine::skills::SkillId;
use rustengine::vision::VisionSource;
use std::path::PathBuf;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::time::{Duration, Instant};

const MAX_WHEEL_MM_S: i32 = 1500;

// ─────────────────────────────────────────────────────────────────────────────
//  Args
// ─────────────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    Wheels,
    Skill,
}

#[derive(Debug, Clone)]
struct Args {
    transport: RadioTarget,
    vision: Option<VisionSource>,
    robot: usize,
    team: TeamColor,
    mode: Mode,
    dur: f64,
    log: Option<PathBuf>,
    dry_run: bool,
    vision_timeout_s: f64,
    skill: Option<SkillId>,
    target: Option<Vec2>,
    left: Option<i32>,
    right: Option<i32>,
}

const HELP_TEXT: &str = r#"skill_test — probador de skills + bring-up de hardware

USO:
    cargo run --bin skill_test -- [FLAGS]

FLAGS COMUNES (obligatorios):
    --transport <firasim|grsim|base-station>
    --robot N          índice 0-based del robot (0..5)
                       Convención: el robot físico que ejecuta este comando es
                       el que tiene MI_ROBOT_ID = N + 1 compilado en su
                       VSSL-firmware/include/config.h
    --team <blue|yellow>
    --mode <wheels|skill>
    --dur S            segundos antes del auto-stop (float > 0)

FLAGS COMUNES (opcionales):
    --log RUTA           CSV estructurado al archivo. SIN --log no se escribe
                         CSV; solo print humano a stderr (1 Hz).
    --dry-run            arma el frame y lo imprime, NO envía nada
    --vision-timeout S   default 5.0 s. Solo aplica con --vision real.

FLAGS MODO skill:
    --vision <sim|real>
    --skill <goto|facepoint|chaseball|spin>
    --target x,y         obligatorio para goto, facepoint, spin
                         (spin solo usa el signo de x: + = CCW, − = CW)
                         opcional para chaseball

FLAGS MODO wheels (solo --transport base-station):
    --left L             mm/s rueda izquierda. Se clampa a ±1500.
    --right R            mm/s rueda derecha.   Se clampa a ±1500.

SECUENCIA DE BRING-UP (modo wheels) para diagnosticar el giro descontrolado:
    1) --left 500  --right 0    → solo rueda izquierda (pivota a la derecha)
    2) --left 0    --right 500  → solo rueda derecha   (pivota a la izquierda)
    3) --left 500  --right 500  → debe AVANZAR RECTO. Si gira, hay un signo
                                  invertido en una rueda (LEFT/RIGHT_WHEEL_SIGN
                                  en VSSL-firmware/include/config.h).
    4) --left -500 --right 500  → giro CCW sobre el eje (confirma convención Spin).

EJEMPLOS:
    # Sim, skill cerrada:
    cargo run --bin skill_test -- --transport firasim --vision sim \
        --mode skill --skill goto --target 0.3,0.0 --robot 0 --team blue --dur 5

    # Robot real, bring-up de wheels:
    cargo run --bin skill_test -- --transport base-station \
        --mode wheels --team blue --robot 0 --left 500 --right 500 --dur 2

    # Robot real, skill cerrada (requiere visión real corriendo):
    cargo run --bin skill_test -- --transport base-station --vision real \
        --mode skill --skill spin --target 1,0 --robot 0 --team blue --dur 3 \
        --log /tmp/spin.csv
"#;

impl Args {
    fn parse<I: IntoIterator<Item = String>>(argv: I) -> Result<Self, String> {
        let mut iter = argv.into_iter().peekable();

        let mut transport: Option<RadioTarget> = None;
        let mut vision: Option<VisionSource> = None;
        let mut robot: Option<usize> = None;
        let mut team: Option<TeamColor> = None;
        let mut mode: Option<Mode> = None;
        let mut dur: Option<f64> = None;
        let mut log: Option<PathBuf> = None;
        let mut dry_run = false;
        let mut vision_timeout_s: f64 = 5.0;
        let mut skill: Option<SkillId> = None;
        let mut target: Option<Vec2> = None;
        let mut left: Option<i32> = None;
        let mut right: Option<i32> = None;

        while let Some(arg) = iter.next() {
            match arg.as_str() {
                "--help" | "-h" => return Err("__HELP__".to_string()),
                "--transport" => {
                    let v = iter.next().ok_or("--transport requiere un valor")?;
                    transport = Some(match v.as_str() {
                        "firasim" => RadioTarget::FiraSim,
                        "grsim" => RadioTarget::GrSim,
                        "base-station" => RadioTarget::BaseStation,
                        other => {
                            return Err(format!(
                                "--transport: valor inválido '{other}' (esperaba firasim|grsim|base-station)"
                            ));
                        }
                    });
                }
                "--vision" => {
                    let v = iter.next().ok_or("--vision requiere un valor")?;
                    vision = Some(match v.as_str() {
                        "sim" => VisionSource::FiraSim,
                        "real" => VisionSource::SslVision,
                        other => {
                            return Err(format!(
                                "--vision: valor inválido '{other}' (esperaba sim|real)"
                            ));
                        }
                    });
                }
                "--robot" => {
                    let v = iter.next().ok_or("--robot requiere un valor")?;
                    let n: usize = v
                        .parse()
                        .map_err(|e| format!("--robot: '{v}' no es entero ({e})"))?;
                    if n >= SLOT_COUNT {
                        return Err(format!("--robot: {n} fuera de rango [0, {SLOT_COUNT})"));
                    }
                    robot = Some(n);
                }
                "--team" => {
                    let v = iter.next().ok_or("--team requiere un valor")?;
                    team = Some(match v.as_str() {
                        "blue" => TeamColor::Blue,
                        "yellow" => TeamColor::Yellow,
                        other => {
                            return Err(format!(
                                "--team: valor inválido '{other}' (esperaba blue|yellow)"
                            ));
                        }
                    });
                }
                "--mode" => {
                    let v = iter.next().ok_or("--mode requiere un valor")?;
                    mode = Some(match v.as_str() {
                        "wheels" => Mode::Wheels,
                        "skill" => Mode::Skill,
                        other => {
                            return Err(format!(
                                "--mode: valor inválido '{other}' (esperaba wheels|skill)"
                            ));
                        }
                    });
                }
                "--dur" => {
                    let v = iter.next().ok_or("--dur requiere un valor")?;
                    let s: f64 = v
                        .parse()
                        .map_err(|e| format!("--dur: '{v}' no es flotante ({e})"))?;
                    if s <= 0.0 {
                        return Err(format!("--dur debe ser > 0 (recibido {s})"));
                    }
                    dur = Some(s);
                }
                "--log" => {
                    let v = iter.next().ok_or("--log requiere una ruta")?;
                    log = Some(PathBuf::from(v));
                }
                "--dry-run" => {
                    dry_run = true;
                }
                "--vision-timeout" => {
                    let v = iter.next().ok_or("--vision-timeout requiere un valor")?;
                    let s: f64 = v
                        .parse()
                        .map_err(|e| format!("--vision-timeout: '{v}' no es flotante ({e})"))?;
                    if s <= 0.0 {
                        return Err(format!("--vision-timeout debe ser > 0 (recibido {s})"));
                    }
                    vision_timeout_s = s;
                }
                "--skill" => {
                    let v = iter.next().ok_or("--skill requiere un valor")?;
                    skill = Some(match v.as_str() {
                        "goto" => SkillId::GoTo,
                        "facepoint" => SkillId::FacePoint,
                        "chaseball" => SkillId::ChaseBall,
                        "spin" => SkillId::Spin,
                        other => {
                            return Err(format!(
                                "--skill: valor inválido '{other}' (esperaba goto|facepoint|chaseball|spin)"
                            ));
                        }
                    });
                }
                "--target" => {
                    let v = iter.next().ok_or("--target requiere x,y")?;
                    let parts: Vec<&str> = v.split(',').collect();
                    if parts.len() != 2 {
                        return Err(format!(
                            "--target: '{v}' debe ser x,y (dos flotantes separados por coma)"
                        ));
                    }
                    let x: f32 = parts[0]
                        .trim()
                        .parse()
                        .map_err(|e| format!("--target x: {e}"))?;
                    let y: f32 = parts[1]
                        .trim()
                        .parse()
                        .map_err(|e| format!("--target y: {e}"))?;
                    target = Some(Vec2::new(x, y));
                }
                "--left" => {
                    let v = iter.next().ok_or("--left requiere un valor")?;
                    left = Some(
                        v.parse()
                            .map_err(|e| format!("--left: '{v}' no es entero ({e})"))?,
                    );
                }
                "--right" => {
                    let v = iter.next().ok_or("--right requiere un valor")?;
                    right = Some(
                        v.parse()
                            .map_err(|e| format!("--right: '{v}' no es entero ({e})"))?,
                    );
                }
                other => return Err(format!("argumento desconocido: '{other}'")),
            }
        }

        // Obligatorios comunes
        let transport = transport.ok_or("--transport es obligatorio")?;
        let robot = robot.ok_or("--robot es obligatorio")?;
        let team = team.ok_or("--team es obligatorio")?;
        let mode = mode.ok_or("--mode es obligatorio")?;
        let dur = dur.ok_or("--dur es obligatorio")?;

        // Validaciones por modo
        match mode {
            Mode::Wheels => {
                if transport != RadioTarget::BaseStation {
                    return Err("modo wheels solo aplica a --transport base-station".to_string());
                }
                if vision.is_some() {
                    return Err("modo wheels NO usa --vision".to_string());
                }
                if skill.is_some() || target.is_some() {
                    return Err("modo wheels NO acepta --skill ni --target".to_string());
                }
                if left.is_none() || right.is_none() {
                    return Err("modo wheels requiere --left y --right".to_string());
                }
            }
            Mode::Skill => {
                let vision = vision.ok_or("modo skill requiere --vision")?;
                let _ = vision;
                let skill_id = skill.ok_or("modo skill requiere --skill")?;
                if left.is_some() || right.is_some() {
                    return Err("modo skill NO acepta --left ni --right".to_string());
                }
                if matches!(skill_id, SkillId::GoTo | SkillId::FacePoint | SkillId::Spin)
                    && target.is_none()
                {
                    return Err(format!(
                        "modo skill --skill {:?} requiere --target x,y",
                        skill_id
                    ));
                }
            }
        }

        Ok(Self {
            transport,
            vision,
            robot,
            team,
            mode,
            dur,
            log,
            dry_run,
            vision_timeout_s,
            skill,
            target,
            left,
            right,
        })
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Modo wheels
// ─────────────────────────────────────────────────────────────────────────────

async fn run_wheels_mode(args: &Args, shutdown: Arc<AtomicBool>) -> Result<(), String> {
    let max = MAX_WHEEL_MM_S;
    let raw_l = args.left.expect("validated");
    let raw_r = args.right.expect("validated");
    let l = raw_l.clamp(-max, max) as i16;
    let r = raw_r.clamp(-max, max) as i16;
    if raw_l != l as i32 {
        eprintln!("[skill_test] --left saturado de {raw_l} a {l} mm/s");
    }
    if raw_r != r as i32 {
        eprintln!("[skill_test] --right saturado de {raw_r} a {r} mm/s");
    }

    let mut slots: [(i16, i16); SLOT_COUNT] = [(0, 0); SLOT_COUNT];
    slots[args.robot] = (l, r);
    let zero_slots: [(i16, i16); SLOT_COUNT] = [(0, 0); SLOT_COUNT];

    let mut csv_logger = match &args.log {
        Some(p) => Some(CsvLogger::new(p).map_err(|e| format!("--log: {e}"))?),
        None => None,
    };

    if args.dry_run {
        let frame = build_frame_from_wheels(slots);
        print!("{frame}");
        return Ok(());
    }

    let mut transport =
        BaseStationTransport::from_env(args.team).map_err(|e| format!("base-station: {e}"))?;

    eprintln!(
        "[skill_test] modo wheels: slots[{}] = ({l},{r}) mm/s, dur={}s",
        args.robot, args.dur
    );

    let started = Instant::now();
    let deadline = started + Duration::from_secs_f64(args.dur);
    let mut interval = tokio::time::interval(Duration::from_millis(50)); // 20 Hz, matchea SEND_INTERVAL_MS
    let mut tick: u32 = 0;
    let mut last_print = Instant::now() - Duration::from_secs(2);

    while !shutdown.load(Ordering::Relaxed) && Instant::now() < deadline {
        interval.tick().await;
        transport
            .send_raw_wheels_frame(slots)
            .await
            .map_err(|e| format!("send error: {e}"))?;

        let t_ms = started.elapsed().as_millis() as u64;
        let frame = build_frame_from_wheels(slots);
        let frame_stripped = frame.trim_end().to_string();

        if let Some(ref mut log) = csv_logger {
            let _ = log.write_row(&CsvRow {
                t_ms,
                tick,
                mode: "wheels",
                transport: transport_label(args.transport),
                vision: "",
                robot: args.robot,
                team: team_label(args.team),
                skill: "",
                pose_x: None,
                pose_y: None,
                pose_theta: None,
                target_x: None,
                target_y: None,
                cmd_vx: None,
                cmd_vy: None,
                cmd_omega: None,
                wheel_l: Some(l),
                wheel_r: Some(r),
                frame_str: frame_stripped.clone(),
                err_dist: None,
                err_heading: None,
            });
        }

        if last_print.elapsed() >= Duration::from_secs(1) {
            eprintln!(
                "[skill_test t={:.1}s tick={tick} wheels=({l},{r})]",
                started.elapsed().as_secs_f64()
            );
            last_print = Instant::now();
        }

        tick = tick.wrapping_add(1);
    }

    eprintln!("[skill_test] stop secuence (5 frames a cero)");
    for _ in 0..5 {
        let _ = transport.send_raw_wheels_frame(zero_slots).await;
    }
    Ok(())
}

// ─────────────────────────────────────────────────────────────────────────────
//  Modo skill
// ─────────────────────────────────────────────────────────────────────────────

async fn run_skill_mode(args: &Args, shutdown: Arc<AtomicBool>) -> Result<(), String> {
    if args.dry_run {
        eprintln!(
            "[skill_test] --dry-run en modo skill no abre transporte; no se ejecuta el control loop. \
             Para imprimir el frame que produciría una skill, usar --mode wheels --dry-run."
        );
        return Ok(());
    }

    let vision_source = args.vision.expect("validated");
    let skill_id = args.skill.expect("validated");
    let target = args.target.unwrap_or(Vec2::ZERO);

    let decider = Box::new(FixedSkillDecider::new(args.robot as i32, skill_id, target));

    let vision_timeout = if matches!(vision_source, VisionSource::SslVision) {
        Some(Duration::from_secs_f64(args.vision_timeout_s))
    } else {
        None
    };

    let max_ticks = Some((args.dur * 60.0).ceil() as u32);

    let config = ControlLoopConfig {
        own_team: args.team.as_team_id(),
        num_robots: SLOT_COUNT,
        vision_source,
        radio_target: args.transport,
        max_ticks,
        vision_timeout,
    };

    // Logger compartido (si --log presente).
    let mut csv: Option<CsvLogger> = match &args.log {
        Some(p) => Some(CsvLogger::new(p).map_err(|e| format!("--log: {e}"))?),
        None => None,
    };
    let mut last_print = Instant::now() - Duration::from_secs(2);

    // Contexto fijo del run; el row-builder vive en `rustengine::skill_log`.
    let ctx = SkillLogCtx {
        transport: args.transport,
        vision: args.vision,
        robot: args.robot,
        team: args.team,
        skill: skill_id,
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

    run_control_loop(
        config,
        decider,
        Some(on_tick),
        None as Option<GuiChannels>,
        shutdown,
    )
    .await
    .map_err(|e| format!("control loop: {e}"))?;
    Ok(())
}

// ─────────────────────────────────────────────────────────────────────────────
//  Entry point
// ─────────────────────────────────────────────────────────────────────────────

fn main() {
    let argv: Vec<String> = std::env::args().skip(1).collect();

    let args = match Args::parse(argv) {
        Ok(a) => a,
        Err(msg) if msg == "__HELP__" => {
            print!("{HELP_TEXT}");
            std::process::exit(0);
        }
        Err(msg) => {
            eprintln!("error: {msg}\n");
            eprintln!("usa --help para ver opciones");
            std::process::exit(2);
        }
    };

    let rt = tokio::runtime::Runtime::new().unwrap();
    let shutdown = Arc::new(AtomicBool::new(false));
    {
        let shutdown = shutdown.clone();
        rt.spawn(async move {
            if tokio::signal::ctrl_c().await.is_ok() {
                eprintln!("[skill_test] Ctrl-C recibido, deteniendo...");
                shutdown.store(true, Ordering::Relaxed);
            }
        });
    }

    let result = rt.block_on(async {
        match args.mode {
            Mode::Wheels => run_wheels_mode(&args, shutdown.clone()).await,
            Mode::Skill => run_skill_mode(&args, shutdown.clone()).await,
        }
    });

    if let Err(msg) = result {
        eprintln!("error: {msg}");
        std::process::exit(1);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(args: &[&str]) -> Result<Args, String> {
        Args::parse(args.iter().map(|s| s.to_string()))
    }

    // ---------- Casos válidos ----------

    #[test]
    fn wheels_full_valid() {
        let a = parse(&[
            "--transport",
            "base-station",
            "--mode",
            "wheels",
            "--team",
            "blue",
            "--robot",
            "0",
            "--left",
            "500",
            "--right",
            "500",
            "--dur",
            "2",
        ])
        .unwrap();
        assert_eq!(a.mode, Mode::Wheels);
        assert_eq!(a.transport, RadioTarget::BaseStation);
        assert_eq!(a.robot, 0);
        assert_eq!(a.left, Some(500));
        assert_eq!(a.right, Some(500));
        assert_eq!(a.dur, 2.0);
    }

    #[test]
    fn skill_goto_valid() {
        let a = parse(&[
            "--transport",
            "firasim",
            "--vision",
            "sim",
            "--mode",
            "skill",
            "--skill",
            "goto",
            "--target",
            "0.3,0.0",
            "--robot",
            "0",
            "--team",
            "blue",
            "--dur",
            "5",
        ])
        .unwrap();
        assert_eq!(a.mode, Mode::Skill);
        assert_eq!(a.skill, Some(SkillId::GoTo));
        assert_eq!(a.target, Some(Vec2::new(0.3, 0.0)));
        assert_eq!(a.vision, Some(VisionSource::FiraSim));
    }

    #[test]
    fn skill_chaseball_target_optional() {
        let a = parse(&[
            "--transport",
            "firasim",
            "--vision",
            "sim",
            "--mode",
            "skill",
            "--skill",
            "chaseball",
            "--robot",
            "1",
            "--team",
            "yellow",
            "--dur",
            "3",
        ])
        .unwrap();
        assert_eq!(a.skill, Some(SkillId::ChaseBall));
        assert!(a.target.is_none());
    }

    #[test]
    fn vision_timeout_override() {
        let a = parse(&[
            "--transport",
            "base-station",
            "--vision",
            "real",
            "--mode",
            "skill",
            "--skill",
            "spin",
            "--target",
            "1,0",
            "--robot",
            "2",
            "--team",
            "blue",
            "--dur",
            "1",
            "--vision-timeout",
            "2.5",
        ])
        .unwrap();
        assert_eq!(a.vision_timeout_s, 2.5);
    }

    #[test]
    fn log_path_recorded() {
        let a = parse(&[
            "--transport",
            "base-station",
            "--mode",
            "wheels",
            "--team",
            "blue",
            "--robot",
            "0",
            "--left",
            "0",
            "--right",
            "0",
            "--dur",
            "1",
            "--log",
            "/tmp/out.csv",
        ])
        .unwrap();
        assert_eq!(a.log, Some(PathBuf::from("/tmp/out.csv")));
    }

    // ---------- Casos inválidos ----------

    #[test]
    fn wheels_without_left_right_fails() {
        let err = parse(&[
            "--transport",
            "base-station",
            "--mode",
            "wheels",
            "--team",
            "blue",
            "--robot",
            "0",
            "--dur",
            "1",
        ])
        .unwrap_err();
        assert!(
            err.contains("--left") && err.contains("--right"),
            "got: {err}"
        );
    }

    #[test]
    fn wheels_with_firasim_fails() {
        let err = parse(&[
            "--transport",
            "firasim",
            "--mode",
            "wheels",
            "--team",
            "blue",
            "--robot",
            "0",
            "--left",
            "0",
            "--right",
            "0",
            "--dur",
            "1",
        ])
        .unwrap_err();
        assert!(err.contains("base-station"), "got: {err}");
    }

    #[test]
    fn skill_goto_without_target_fails() {
        let err = parse(&[
            "--transport",
            "firasim",
            "--vision",
            "sim",
            "--mode",
            "skill",
            "--skill",
            "goto",
            "--robot",
            "0",
            "--team",
            "blue",
            "--dur",
            "1",
        ])
        .unwrap_err();
        assert!(err.contains("--target"), "got: {err}");
    }

    #[test]
    fn skill_without_vision_fails() {
        let err = parse(&[
            "--transport",
            "firasim",
            "--mode",
            "skill",
            "--skill",
            "goto",
            "--target",
            "0,0",
            "--robot",
            "0",
            "--team",
            "blue",
            "--dur",
            "1",
        ])
        .unwrap_err();
        assert!(err.contains("--vision"), "got: {err}");
    }

    #[test]
    fn robot_out_of_range_fails() {
        let err = parse(&[
            "--transport",
            "firasim",
            "--vision",
            "sim",
            "--mode",
            "skill",
            "--skill",
            "chaseball",
            "--robot",
            "9",
            "--team",
            "blue",
            "--dur",
            "1",
        ])
        .unwrap_err();
        assert!(err.contains("fuera de rango"), "got: {err}");
    }

    #[test]
    fn dur_zero_fails() {
        let err = parse(&[
            "--transport",
            "firasim",
            "--vision",
            "sim",
            "--mode",
            "skill",
            "--skill",
            "chaseball",
            "--robot",
            "0",
            "--team",
            "blue",
            "--dur",
            "0",
        ])
        .unwrap_err();
        assert!(err.contains("> 0"), "got: {err}");
    }

    #[test]
    fn dur_negative_fails() {
        let err = parse(&[
            "--transport",
            "firasim",
            "--vision",
            "sim",
            "--mode",
            "skill",
            "--skill",
            "chaseball",
            "--robot",
            "0",
            "--team",
            "blue",
            "--dur",
            "-1",
        ])
        .unwrap_err();
        assert!(err.contains("> 0"), "got: {err}");
    }

    #[test]
    fn unknown_flag_fails() {
        let err = parse(&[
            "--transport",
            "firasim",
            "--vision",
            "sim",
            "--mode",
            "skill",
            "--skill",
            "chaseball",
            "--robot",
            "0",
            "--team",
            "blue",
            "--dur",
            "1",
            "--bogus",
        ])
        .unwrap_err();
        assert!(err.contains("--bogus"), "got: {err}");
    }

    #[test]
    fn help_returns_help_sentinel() {
        let err = parse(&["--help"]).unwrap_err();
        assert_eq!(err, "__HELP__");
    }

    #[test]
    fn help_text_documents_bring_up_sequence() {
        // Verifica que el --help menciona la secuencia de bring-up y MI_ROBOT_ID.
        assert!(HELP_TEXT.contains("MI_ROBOT_ID = N + 1"));
        assert!(HELP_TEXT.contains("SECUENCIA DE BRING-UP"));
        assert!(HELP_TEXT.contains("500"));
    }
}
