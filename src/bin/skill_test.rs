//! Probador de skills + bring-up de hardware.
//!
//! Ejecuta una skill (lazo cerrado, reusando exactamente el mismo loop que `main`)
//! o una consigna (v mm/s, ω °/s) directa al robot real (lazo abierto, modo `vw`,
//! solo `--transport base-station`), con la misma CLI en simulador y en robot real.
//!
//! Ver `cargo run --bin skill_test -- --help` para uso.

use glam::Vec2;
use rustengine::control_loop::{
    ControlLoopConfig, FixedSkillDecider, GuiChannels, TickRecord, run_control_loop,
};
use rustengine::radio::{
    BaseStationTransport, RadioTarget, TeamColor,
    base_station::SLOT_COUNT,
    base_station::{build_frame_from_vw, clamp_vw},
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

// ─────────────────────────────────────────────────────────────────────────────
//  Args
// ─────────────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    Vw,
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
    v_mm_s: Option<i32>,
    w_deg_s: Option<i32>,
}

const HELP_TEXT: &str = r#"skill_test — probador de skills + bring-up de hardware

USO:
    cargo run --bin skill_test -- [FLAGS]

FLAGS COMUNES (obligatorios):
    --transport <firasim|grsim|base-station>
    --robot N          índice del robot, desde 0 (0..5)
                       Convención: el robot físico que ejecuta este comando es
                       el que tiene MI_ROBOT_ID = N + 1 compilado en su
                       VSSL-firmware/include/config.h. El robot del checkout
                       actual del firmware (rama Peluche) es MI_ROBOT_ID 2
                       → --robot 1.
    --team <blue|yellow>
    --mode <vw|skill>
    --dur S            segundos antes del auto-stop (float > 0)

FLAGS COMUNES (opcionales):
    --log RUTA           CSV estructurado al archivo. SIN --log no se escribe
                         CSV; solo print humano a stderr (1 Hz).
    --dry-run            arma el frame y lo imprime, NO envía nada
    --vision-timeout S   default 5.0 s. Solo aplica con --vision real.

FLAGS MODO skill:
    --vision <sim|real>
    --skill <goto|facepoint|chaseball|spin|approach|shoot|intercept|blockline|
             goalkeep|clear|spinkick|mark>
    --target x,y         obligatorio para goto, facepoint, spin, approach,
                         shoot/clear/spinkick (punto al que lanzar la pelota),
                         blockline y goalkeep (centro del arco propio, p.ej.
                         -0.75,0) y mark (punto donde pararse mirando la pelota)
                         (spin solo usa el signo de x: + = CCW, − = CW)
                         opcional para chaseball; ignorado por intercept

VARIABLES DE ENTORNO:
    VSSL_BIDIRECTIONAL=1   motion de dos caras: el heading se controla mod 180°
                           (usa la cara frontal o trasera, la que requiera
                           menos giro). Por defecto apagado (solo frente).
    VSSL_TRACKER=off       arranca con el EKF apagado → el CSV registra poses
                           CRUDAS de visión (para medir ruido de cámara).

FLAGS MODO vw (solo --transport base-station, lazo abierto):
    --v MM_S             velocidad lineal en mm/s. Se clampa a ±max_v_mm_s
                         (config/team_params.json; 1500 por defecto).
    --w DEG_S            velocidad angular en grados/s (+ = antihorario). Se
                         clampa a ±max_w_deg_s (720 por defecto).
                         Va a la base como frame "V1,W1,...,V5,W5". El firmware
                         reparte a las ruedas, corrige ω con el giroscopio y topa
                         cada rueda en 450 mm/s (escala ambas, conserva la curva).

SECUENCIA DE BRING-UP (modo vw). Encender el robot QUIETO (calibra el gyro);
--dur 2 por paso. Si algo falla, parar y anotar qué pasó.
    1) --v 300 --w 0     → avanza recto.
         Se desvía y termina girando cada vez más rápido → GYRO_Z_SIGN.
         Va hacia atrás o gira en el lugar → LEFT/RIGHT_WHEEL_SIGN
         (VSSL-firmware/include/config.h).
    2) --v -300 --w 0    → retrocede recto.
         Si (1) anduvo y (2) no: marcha atrás de una rueda (driver o
         feedforward), no signos.
    3) --v 0 --w 90      → gira antihorario visto desde arriba (~1 vuelta/4 s).
         Gira horario con (1) bien → canales de motor izq./der. intercambiados
         (cableado o pines). Gira mucho más rápido → GYRO_Z_SIGN.
         No arranca → zona muerta (pide solo ±59 mm/s por rueda); probar
         --w 180 para distinguir zona muerta de fricción.
    4) --v 0 --w -90     → gira horario.
         Si (3) anduvo y (4) no: asimetría de una rueda.
    5) --v 300 --w 45    → arco hacia la izquierda (radio ≈ 0.38 m).
         Va a la derecha → mismo diagnóstico que el giro horario de (3).
         Radio muy distinto → WHEEL_TRACK_MM o gyro.

EJEMPLOS:
    # Sim, skill cerrada:
    cargo run --bin skill_test -- --transport firasim --vision sim \
        --mode skill --skill goto --target 0.3,0.0 --robot 0 --team blue --dur 5

    # Robot real (MI_ROBOT_ID 2), paso 1 del bring-up:
    cargo run --bin skill_test -- --transport base-station \
        --mode vw --team blue --robot 1 --v 300 --w 0 --dur 2

    # Robot real, skill cerrada (requiere visión real corriendo):
    cargo run --bin skill_test -- --transport base-station --vision real \
        --mode skill --skill spin --target 1,0 --robot 1 --team blue --dur 3 \
        --log /tmp/spin.csv

    # Sim, empuje de dos caras hacia el arco rival:
    VSSL_BIDIRECTIONAL=1 cargo run --bin skill_test -- --transport firasim \
        --vision sim --mode skill --skill shoot --target 0.75,0 \
        --robot 0 --team blue --dur 6 --log /tmp/shoot.csv
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
        let mut v_mm_s: Option<i32> = None;
        let mut w_deg_s: Option<i32> = None;

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
                        "vw" => Mode::Vw,
                        "skill" => Mode::Skill,
                        "wheels" => {
                            return Err("--mode wheels ya no existe (era del protocolo L,R): \
                                        usar --mode vw --v MM_S --w DEG_S"
                                .to_string());
                        }
                        other => {
                            return Err(format!(
                                "--mode: valor inválido '{other}' (esperaba vw|skill)"
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
                        "approach" => SkillId::ApproachAligned,
                        "shoot" => SkillId::ShootPush,
                        "intercept" => SkillId::Intercept,
                        "blockline" => SkillId::BlockLine,
                        "goalkeep" => SkillId::GoalKeep,
                        "clear" => SkillId::Clear,
                        "spinkick" => SkillId::SpinKick,
                        "mark" => SkillId::Mark,
                        "hold" => SkillId::Hold,
                        other => {
                            return Err(format!(
                                "--skill: valor inválido '{other}' (esperaba goto|facepoint|chaseball|spin|approach|shoot|intercept|blockline|goalkeep|clear|spinkick|mark|hold)"
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
                "--v" => {
                    let v = iter.next().ok_or("--v requiere un valor")?;
                    v_mm_s = Some(
                        v.parse()
                            .map_err(|e| format!("--v: '{v}' no es entero ({e})"))?,
                    );
                }
                "--w" => {
                    let v = iter.next().ok_or("--w requiere un valor")?;
                    w_deg_s = Some(
                        v.parse()
                            .map_err(|e| format!("--w: '{v}' no es entero ({e})"))?,
                    );
                }
                "--left" | "--right" => {
                    return Err(format!(
                        "{arg} ya no existe (era del protocolo L,R): usar --mode vw --v MM_S --w DEG_S"
                    ));
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
            Mode::Vw => {
                if transport != RadioTarget::BaseStation {
                    return Err("modo vw solo aplica a --transport base-station".to_string());
                }
                if vision.is_some() {
                    return Err("modo vw NO usa --vision".to_string());
                }
                if skill.is_some() || target.is_some() {
                    return Err("modo vw NO acepta --skill ni --target".to_string());
                }
                if v_mm_s.is_none() || w_deg_s.is_none() {
                    return Err("modo vw requiere --v y --w".to_string());
                }
            }
            Mode::Skill => {
                let vision = vision.ok_or("modo skill requiere --vision")?;
                let _ = vision;
                let skill_id = skill.ok_or("modo skill requiere --skill")?;
                if v_mm_s.is_some() || w_deg_s.is_some() {
                    return Err("modo skill NO acepta --v ni --w".to_string());
                }
                if matches!(
                    skill_id,
                    SkillId::GoTo
                        | SkillId::FacePoint
                        | SkillId::Spin
                        | SkillId::ApproachAligned
                        | SkillId::ShootPush
                        | SkillId::BlockLine
                        | SkillId::GoalKeep
                        | SkillId::Clear
                        | SkillId::SpinKick
                        | SkillId::Mark
                ) && target.is_none()
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
            v_mm_s,
            w_deg_s,
        })
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Modo vw
// ─────────────────────────────────────────────────────────────────────────────

async fn run_vw_mode(args: &Args, shutdown: Arc<AtomicBool>) -> Result<(), String> {
    let raw_v = args.v_mm_s.expect("validated");
    let raw_w = args.w_deg_s.expect("validated");
    let (v, w) = clamp_vw(f64::from(raw_v), f64::from(raw_w));
    if i32::from(v) != raw_v {
        eprintln!("[skill_test] --v saturado de {raw_v} a {v} mm/s");
    }
    if i32::from(w) != raw_w {
        eprintln!("[skill_test] --w saturado de {raw_w} a {w} °/s");
    }

    let mut slots: [(i16, i16); SLOT_COUNT] = [(0, 0); SLOT_COUNT];
    slots[args.robot] = (v, w);
    let zero_slots: [(i16, i16); SLOT_COUNT] = [(0, 0); SLOT_COUNT];

    let mut csv_logger = match &args.log {
        Some(p) => Some(CsvLogger::new(p).map_err(|e| format!("--log: {e}"))?),
        None => None,
    };

    if args.dry_run {
        let frame = build_frame_from_vw(slots);
        print!("{frame}");
        return Ok(());
    }

    let mut transport =
        BaseStationTransport::from_env(args.team).map_err(|e| format!("base-station: {e}"))?;

    eprintln!(
        "[skill_test] modo vw: slots[{}] = (v={v} mm/s, w={w} °/s), dur={}s",
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
            .send_raw_vw_frame(slots)
            .await
            .map_err(|e| format!("send error: {e}"))?;

        let t_ms = started.elapsed().as_millis() as u64;
        let frame = build_frame_from_vw(slots);
        let frame_stripped = frame.trim_end().to_string();

        if let Some(ref mut log) = csv_logger {
            let _ = log.write_row(&CsvRow {
                t_ms,
                tick,
                mode: "vw",
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
                v_mm_s: Some(v),
                w_deg_s: Some(w),
                frame_str: frame_stripped.clone(),
                err_dist: None,
                err_heading: None,
                ball_x: None,
                ball_y: None,
                ball_vx: None,
                ball_vy: None,
            });
        }

        if last_print.elapsed() >= Duration::from_secs(1) {
            eprintln!(
                "[skill_test t={:.1}s tick={tick} vw=({v},{w})]",
                started.elapsed().as_secs_f64()
            );
            last_print = Instant::now();
        }

        tick = tick.wrapping_add(1);
    }

    eprintln!("[skill_test] stop secuence (5 frames a cero)");
    for _ in 0..5 {
        let _ = transport.send_raw_vw_frame(zero_slots).await;
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
             Para imprimir un frame sin hardware, usar --mode vw --dry-run."
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
    rustengine::params::TeamParams::install_or_exit("skill_test");

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
            Mode::Vw => run_vw_mode(&args, shutdown.clone()).await,
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
    fn vw_full_valid() {
        let a = parse(&[
            "--transport",
            "base-station",
            "--mode",
            "vw",
            "--team",
            "blue",
            "--robot",
            "0",
            "--v",
            "500",
            "--w",
            "500",
            "--dur",
            "2",
        ])
        .unwrap();
        assert_eq!(a.mode, Mode::Vw);
        assert_eq!(a.transport, RadioTarget::BaseStation);
        assert_eq!(a.robot, 0);
        assert_eq!(a.v_mm_s, Some(500));
        assert_eq!(a.w_deg_s, Some(500));
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
            "vw",
            "--team",
            "blue",
            "--robot",
            "0",
            "--v",
            "0",
            "--w",
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
    fn vw_without_v_w_fails() {
        let err = parse(&[
            "--transport",
            "base-station",
            "--mode",
            "vw",
            "--team",
            "blue",
            "--robot",
            "0",
            "--dur",
            "1",
        ])
        .unwrap_err();
        assert!(err.contains("--v") && err.contains("--w"), "got: {err}");
    }

    #[test]
    fn vw_with_firasim_fails() {
        let err = parse(&[
            "--transport",
            "firasim",
            "--mode",
            "vw",
            "--team",
            "blue",
            "--robot",
            "0",
            "--v",
            "0",
            "--w",
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
    fn vw_with_vision_fails() {
        let err = parse(&[
            "--transport",
            "base-station",
            "--vision",
            "real",
            "--mode",
            "vw",
            "--team",
            "blue",
            "--robot",
            "1",
            "--v",
            "300",
            "--w",
            "0",
            "--dur",
            "1",
        ])
        .unwrap_err();
        assert!(err.contains("--vision"), "got: {err}");
    }

    #[test]
    fn old_wheels_mode_points_to_vw() {
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
        assert!(err.contains("--mode vw"), "got: {err}");
    }

    #[test]
    fn old_left_right_flags_point_to_vw() {
        for flag in ["--left", "--right"] {
            let err = parse(&["--transport", "base-station", flag, "500"]).unwrap_err();
            assert!(
                err.contains(flag) && err.contains("--v") && err.contains("--w"),
                "got: {err}"
            );
        }
    }

    #[test]
    fn skill_with_v_fails() {
        let err = parse(&[
            "--transport",
            "firasim",
            "--vision",
            "sim",
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
            "--v",
            "300",
            "--dur",
            "1",
        ])
        .unwrap_err();
        assert!(err.contains("--v"), "got: {err}");
    }

    #[test]
    fn help_text_documents_bring_up_sequence() {
        // El --help documenta la convención de slot, el robot del checkout y la
        // secuencia de bring-up del modo vw con sus diagnósticos.
        for needle in [
            "MI_ROBOT_ID = N + 1",
            "--robot 1",
            "SECUENCIA DE BRING-UP",
            "GYRO_Z_SIGN",
            "LEFT/RIGHT_WHEEL_SIGN",
            "--v 300 --w 0",
            "--v -300 --w 0",
            "--v 0 --w 90",
            "--v 0 --w -90",
            "--v 300 --w 45",
            "--w 180",
        ] {
            assert!(HELP_TEXT.contains(needle), "falta '{needle}' en el --help");
        }
    }
}
