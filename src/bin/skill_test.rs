//! Probador de skills + bring-up de hardware.
//!
//! Ejecuta una skill (lazo cerrado, reusando exactamente el mismo loop que `main`)
//! o una consigna (v mm/s, ω °/s) directa al robot real (lazo abierto, modo `vw`,
//! solo `--transport base-station`), con la misma CLI en simulador y en robot real.
//! El modo `vw` también corre los perfiles de sysid (`--profile`): en el robot, en
//! FIRASim o en la planta offline, registrando la pose cruda de visión.
//!
//! Ver `cargo run --bin skill_test -- --help` para uso.

use glam::Vec2;
use rustengine::control_loop::{
    ContextoTick, ControlLoopConfig, FixedSkillDecider, GuiChannels, PerfilVelocidad, TickDecider, TickRecord,
    run_control_loop,
};
use rustengine::motion::{KickerCommand, MotionCommand, RobotCommand};
use rustengine::radio::{
    BaseStationTransport, FiraSimTransport, RadioTarget, RobotTransport, TeamColor, TeleportItem,
    base_station::SLOT_COUNT,
    base_station::{build_frame_from_vw, clamp_vw, radio_slot},
};
use rustengine::rutinas::resumen::{Resultado, ResumenCorrida, corrida_json, escribir_resumen, junto_al_log};
use rustengine::rutinas::{
    ContextoValidacion, Eleccion, OpcionesCorrida, OrdenCorrida, Repeticiones, RoutineDecider, RutinaVista,
    velocidad_valida,
};
use rustengine::skill_log::{
    CsvLogger, CsvRow, MatchLogger, SkillLogCtx, format_human_summary, team_label, transport_label, vision_label,
};
use rustengine::skills::SkillId;
use rustengine::sysid;
use rustengine::vision::{Vision, VisionEvent, VisionSource};
use std::fs::File;
use std::io::{LineWriter, Write};
use std::path::{Path, PathBuf};
use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, Ordering},
};
use tokio::sync::mpsc;
use std::time::{Duration, Instant};

// ─────────────────────────────────────────────────────────────────────────────
//  Args
// ─────────────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    Vw,
    Skill,
    Rutina,
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
    /// Perfil de sysid (modo vw): reemplaza a `--v/--w/--dur`.
    profile: Option<String>,
    /// `--transport plant`: planta offline, sin red ni tiempo real (modo vw con perfil).
    /// `transport` queda en FiraSim: la planta usa su geometría.
    plant: bool,
    /// Id de visión del robot que se registra (modo vw con perfil).
    track: Option<u32>,
    battery: Option<String>,
    battery_v: Option<f64>,
    /// Caja segura (|x|, |y| máximos, m): fuera de ella, el perfil se corta.
    safe_box: (f32, f32),
    /// Modo rutina: id de `config/rutinas/` o ruta a un `.json`.
    rutina: Option<String>,
    /// Modo rutina: un robot por puesto; `None` = automático.
    robots: Option<Vec<i32>>,
    velocidad: Option<u32>,
    repeticiones: Option<Repeticiones>,
    teletransportar: bool,
}

/// Caja segura por defecto del modo vw con perfil (m).
const DEFAULT_SAFE_BOX: (f32, f32) = (0.60, 0.50);

const HELP_TEXT: &str = r#"skill_test — probador de skills + bring-up de hardware

USO:
    cargo run --bin skill_test -- [FLAGS]

FLAGS COMUNES (obligatorios):
    --transport <firasim|grsim|base-station|plant>
    --robot N          índice del robot, desde 0 (0..5)
                       --mode vw: N es la POSICIÓN de radio en el frame. El
                       robot físico que ejecuta el comando es el que tiene
                       MI_ROBOT_ID = N + 1 compilado en su
                       VSSL-firmware/include/config.h. El robot del checkout
                       actual del firmware (rama Peluche) es MI_ROBOT_ID 2
                       → --robot 1.
                       --mode skill: N es el id de VISIÓN (parche de colores,
                       el mismo número que muestra la GUI). Con base-station,
                       la posición de radio sale de robot.radio_slot_by_vision_id
                       (config/team_params.json); sin entrada, posición = N.
    --team <blue|yellow>
    --mode <vw|skill|rutina>
    --dur S            segundos antes del auto-stop (float > 0). No con --profile.
                       En --mode rutina es opcional: un tope que aborta la rutina.
    (--robot no va en --mode rutina: los robots salen de la asignación.)

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

FLAGS MODO rutina (circuitos de config/rutinas/, ver README):
    --rutina NOMBRE      id de config/rutinas/ (o de VSSL_RUTINAS_DIR) o ruta a un
                         .json. Una rutina inválida se rechaza con sus razones.
    --vision <sim|real>  obligatorio.
    --robots auto|N,M    robots por puesto, en orden (default auto: los más
                         cercanos a los inicios). Ids de visión.
    --velocidad PCT      de 20 a 100 en pasos de 10 (default 60 con base-station,
                         100 en simulación). Escala los topes de motion y el giro.
    --repeticiones N|bucle   default: las del archivo.
    --teletransportar    solo firasim/grsim: pone los robots en su inicio y saca
                         de la cancha a los demás y a la pelota.
    --log RUTA           CSV de partido (con las columnas de rutina) y el resumen
                         en <RUTA sin .csv>.resumen.json.
    --listar-rutinas     lista las rutinas con su validez y sale.
    La rutina se prepara, arranca sola al quedar lista y el proceso termina al
    cerrarla. Ctrl+C la aborta (en bucle: "detenida") y escribe el resumen.
    Código de salida: 0 completada (o detenida en bucle), 2 abortada, 1 error.

VARIABLES DE ENTORNO:
    VSSL_BIDIRECTIONAL=1   motion de dos caras: el heading se controla mod 180°
                           (usa la cara frontal o trasera, la que requiera
                           menos giro). Por defecto apagado (solo frente).
    VSSL_TRACKER=off       arranca con el EKF apagado → el CSV registra poses
                           CRUDAS de visión (para medir ruido de cámara).

FLAGS MODO vw (lazo abierto). Con --v/--w (bring-up), solo --transport base-station:
    --v MM_S             velocidad lineal en mm/s. Se clampa a ±max_v_mm_s
                         (config/team_params.json; 1500 por defecto).
    --w DEG_S            velocidad angular en grados/s (+ = antihorario). Se
                         clampa a ±max_w_deg_s (720 por defecto).
                         Va a la base como frame "V1,W1,...,V5,W5". El firmware
                         reparte a las ruedas, corrige ω con el giroscopio y topa
                         cada rueda en 450 mm/s (escala ambas, conserva la curva).

MODO vw CON PERFIL (sysid, ver tools/sysid_procedimiento.txt):
    --profile NOMBRE     perfil fijo de (v, ω) a 20 Hz: reemplaza a --v, --w y
                         --dur. --list-profiles los lista con duración y recorrido.
                         Requiere --log RUTA: escribe RUTA (comandos), <RUTA sin
                         .csv>.pose.csv (pose CRUDA de visión, cada frame) y
                         <RUTA sin .csv>.meta.json (robot, batería, perfil, estado).
                         Con VSSL_VISION_RECORD=ruta graba además los paquetes crudos.
    --transport          base-station (con --vision real), firasim (con --vision
                         sim; pone el robot en el centro mirando a +x) o plant
                         (planta offline, sin red ni tiempo real; con
                         VSSL_REAL_ACTUATOR usa el robot medido).
    --track N            id de VISIÓN del robot a registrar. Obligatorio con
                         base-station (--robot es la posición de radio); en
                         firasim, por defecto = --robot.
    --battery full|half  obligatorio con base-station. --battery-v V (tester),
                         opcional, va al .meta.json.
    --safe-box X,Y       caja segura en m (default 0.60,0.50): si la pose sale de
                         |x| ≤ X, |y| ≤ Y, o la visión se pierde 0.5 s, el perfil se
                         corta y manda ceros. Antes de mover, exige ver al robot
                         dentro de la caja.
    SEGURIDAD: la base NO tiene timeout de serial; si skill_test muere sin mandar
    ceros, la base repite el último comando para siempre. Siempre debe haber
    alguien listo para levantar el robot o desenchufar la base. Ctrl+C, SIGTERM
    (kill) y cerrar la terminal mandan ceros; kill -9 NO.

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

    # Robot real (MI_ROBOT_ID 2, parche 1), perfil de sysid con batería cargada:
    cargo run --bin skill_test -- --transport base-station --vision real \
        --mode vw --team blue --robot 1 --track 1 --battery full \
        --profile v_steps --log ~/sysid/1/full/v_steps.csv

    # Robot real, skill cerrada (requiere visión real corriendo):
    cargo run --bin skill_test -- --transport base-station --vision real \
        --mode skill --skill spin --target 1,0 --robot 1 --team blue --dur 3 \
        --log /tmp/spin.csv

    # Sim, el triángulo 10 veces, con ruido de cámara y teleport:
    VSSL_VISION_NOISE=1 cargo run --bin skill_test -- --transport firasim \
        --vision sim --team blue --mode rutina --rutina triangulo \
        --repeticiones 10 --teletransportar --log /tmp/triangulo.csv

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
        let mut profile: Option<String> = None;
        let mut plant = false;
        let mut track: Option<u32> = None;
        let mut battery: Option<String> = None;
        let mut battery_v: Option<f64> = None;
        let mut safe_box: Option<(f32, f32)> = None;
        let mut rutina: Option<String> = None;
        let mut robots: Option<Vec<i32>> = None;
        let mut velocidad: Option<u32> = None;
        let mut repeticiones: Option<Repeticiones> = None;
        let mut teletransportar = false;

        while let Some(arg) = iter.next() {
            match arg.as_str() {
                "--help" | "-h" => return Err("__HELP__".to_string()),
                "--list-profiles" => return Err("__PROFILES__".to_string()),
                "--listar-rutinas" => return Err("__RUTINAS__".to_string()),
                "--rutina" => {
                    rutina = Some(iter.next().ok_or("--rutina requiere un nombre")?);
                }
                "--robots" => {
                    let v = iter.next().ok_or("--robots requiere auto o N,M")?;
                    robots = if v == "auto" {
                        None
                    } else {
                        let ids: Result<Vec<i32>, String> = v
                            .split(',')
                            .map(|x| {
                                let n: i32 = x.trim().parse().map_err(|e| format!("--robots: '{x}' no es entero ({e})"))?;
                                if n < 0 || n as usize >= SLOT_COUNT {
                                    return Err(format!("--robots: {n} fuera de rango [0, {SLOT_COUNT})"));
                                }
                                Ok(n)
                            })
                            .collect();
                        Some(ids?)
                    };
                }
                "--velocidad" => {
                    let v = iter.next().ok_or("--velocidad requiere un porcentaje")?;
                    let p: u32 = v.parse().map_err(|e| format!("--velocidad: '{v}' no es entero ({e})"))?;
                    if !velocidad_valida(p) {
                        return Err(format!("--velocidad {p}: de 20 a 100 en pasos de 10"));
                    }
                    velocidad = Some(p);
                }
                "--repeticiones" => {
                    let v = iter.next().ok_or("--repeticiones requiere N o bucle")?;
                    repeticiones = Some(Repeticiones::de_texto(&v).map_err(|e| format!("--{e}"))?);
                }
                "--teletransportar" => teletransportar = true,
                "--transport" => {
                    let v = iter.next().ok_or("--transport requiere un valor")?;
                    plant = v == "plant";
                    transport = Some(match v.as_str() {
                        "firasim" | "plant" => RadioTarget::FiraSim,
                        "grsim" => RadioTarget::GrSim,
                        "base-station" => RadioTarget::BaseStation,
                        other => {
                            return Err(format!(
                                "--transport: valor inválido '{other}' (esperaba firasim|grsim|base-station|plant)"
                            ));
                        }
                    });
                }
                "--profile" => {
                    let v = iter.next().ok_or("--profile requiere un nombre")?;
                    if sysid::profile(&v).is_none() {
                        return Err(format!(
                            "--profile: '{v}' no existe (perfiles: {})",
                            sysid::PROFILE_NAMES.join(", ")
                        ));
                    }
                    profile = Some(v);
                }
                "--track" => {
                    let v = iter.next().ok_or("--track requiere un id de visión")?;
                    track = Some(v.parse().map_err(|e| format!("--track: '{v}' no es entero ({e})"))?);
                }
                "--battery" => {
                    let v = iter.next().ok_or("--battery requiere full o half")?;
                    if v != "full" && v != "half" {
                        return Err(format!("--battery: '{v}' inválido (esperaba full|half)"));
                    }
                    battery = Some(v);
                }
                "--battery-v" => {
                    let v = iter.next().ok_or("--battery-v requiere un voltaje")?;
                    let volts: f64 = v
                        .parse()
                        .map_err(|e| format!("--battery-v: '{v}' no es flotante ({e})"))?;
                    if !(volts > 0.0 && volts < 30.0) {
                        return Err(format!("--battery-v fuera de rango: {volts}"));
                    }
                    battery_v = Some(volts);
                }
                "--safe-box" => {
                    let v = iter.next().ok_or("--safe-box requiere X,Y")?;
                    let (x, y) = v.split_once(',').ok_or("--safe-box: formato X,Y")?;
                    let x: f32 = x.trim().parse().map_err(|e| format!("--safe-box x: {e}"))?;
                    let y: f32 = y.trim().parse().map_err(|e| format!("--safe-box y: {e}"))?;
                    if !(x > 0.0 && y > 0.0 && x <= 0.75 && y <= 0.65) {
                        return Err(format!("--safe-box fuera de la cancha: {x},{y}"));
                    }
                    safe_box = Some((x, y));
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
                        "rutina" => Mode::Rutina,
                        "wheels" => {
                            return Err("--mode wheels ya no existe (era del protocolo L,R): \
                                        usar --mode vw --v MM_S --w DEG_S"
                                .to_string());
                        }
                        other => {
                            return Err(format!(
                                "--mode: valor inválido '{other}' (esperaba vw|skill|rutina)"
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
        let team = team.ok_or("--team es obligatorio")?;
        let mode = mode.ok_or("--mode es obligatorio")?;
        let profile_only = [
            ("--track", track.is_some()),
            ("--battery", battery.is_some()),
            ("--battery-v", battery_v.is_some()),
            ("--safe-box", safe_box.is_some()),
        ];
        let rutina_only = [
            ("--rutina", rutina.is_some()),
            ("--robots", robots.is_some()),
            ("--velocidad", velocidad.is_some()),
            ("--repeticiones", repeticiones.is_some()),
            ("--teletransportar", teletransportar),
        ];

        // Modo rutina: sus propias reglas.
        if mode == Mode::Rutina {
            if robot.is_some() {
                return Err("modo rutina no usa --robot: los robots salen de la asignación (--robots auto|N,M)".to_string());
            }
            if vision.is_none() {
                return Err("modo rutina requiere --vision".to_string());
            }
            if skill.is_some() || target.is_some() || v_mm_s.is_some() || w_deg_s.is_some() || profile.is_some() {
                return Err("modo rutina NO acepta --skill, --target, --v, --w ni --profile".to_string());
            }
            if plant || dry_run {
                return Err("modo rutina no admite --transport plant ni --dry-run".to_string());
            }
            if let Some((flag, _)) = profile_only.iter().find(|(_, given)| *given) {
                return Err(format!("{flag} solo aplica a --mode vw con --profile"));
            }
            if rutina.is_none() {
                return Err("modo rutina requiere --rutina NOMBRE (--listar-rutinas muestra las que hay)".to_string());
            }
            if teletransportar && transport == RadioTarget::BaseStation {
                return Err("--teletransportar es solo de simulación (firasim o grsim)".to_string());
            }
            return Ok(Self {
                transport,
                vision,
                robot: 0,
                team,
                mode,
                dur: dur.unwrap_or(f64::INFINITY),
                log,
                dry_run,
                vision_timeout_s,
                skill: None,
                target: None,
                v_mm_s: None,
                w_deg_s: None,
                profile: None,
                plant: false,
                track: None,
                battery: None,
                battery_v: None,
                safe_box: DEFAULT_SAFE_BOX,
                rutina,
                robots,
                velocidad,
                repeticiones,
                teletransportar,
            });
        }
        if let Some((flag, _)) = rutina_only.iter().find(|(_, given)| *given) {
            return Err(format!("{flag} solo aplica a --mode rutina"));
        }
        let robot = robot.ok_or("--robot es obligatorio")?;

        // Modo vw con perfil (sysid): sus propias reglas.
        if let Some(name) = &profile {
            if mode != Mode::Vw {
                return Err("--profile solo aplica a --mode vw".to_string());
            }
            if v_mm_s.is_some() || w_deg_s.is_some() || dur.is_some() {
                return Err("--profile excluye --v, --w y --dur (el perfil fija la consigna y la duración)".to_string());
            }
            if skill.is_some() || target.is_some() {
                return Err("modo vw NO acepta --skill ni --target".to_string());
            }
            if log.is_none() {
                return Err("--profile requiere --log (sin registro no sirve para el ajuste)".to_string());
            }
            if dry_run {
                return Err("--profile no admite --dry-run: para probar sin robot, --transport plant".to_string());
            }
            let wanted = match (transport, plant) {
                (_, true) => None,
                (RadioTarget::FiraSim, false) => Some(VisionSource::FiraSim),
                (RadioTarget::BaseStation, false) => Some(VisionSource::SslVision),
                (RadioTarget::GrSim, false) => {
                    return Err("--profile corre en base-station, firasim o plant".to_string());
                }
            };
            match wanted {
                None if vision.is_some() || track.is_some() => {
                    return Err("--transport plant no usa --vision ni --track (la pose es la de la planta)".to_string());
                }
                Some(want) if vision != Some(want) => {
                    return Err(format!(
                        "--profile con --transport {} requiere --vision {}",
                        if plant { "plant" } else { transport_label(transport) },
                        if want == VisionSource::FiraSim { "sim" } else { "real" }
                    ));
                }
                _ => {}
            }
            if transport == RadioTarget::BaseStation && !plant {
                if battery.is_none() {
                    return Err("--profile con base-station requiere --battery full|half".to_string());
                }
                if track.is_none() {
                    return Err("--profile con base-station requiere --track <id de visión> (--robot es la posición de radio)".to_string());
                }
            }
            let dur = sysid::profile(name).expect("validado").duration_s();
            let track = if wanted.is_some() { Some(track.unwrap_or(robot as u32)) } else { None };
            return Ok(Self {
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
                profile,
                plant,
                track,
                battery,
                battery_v,
                safe_box: safe_box.unwrap_or(DEFAULT_SAFE_BOX),
                rutina: None,
                robots: None,
                velocidad: None,
                repeticiones: None,
                teletransportar: false,
            });
        }
        if plant {
            return Err("--transport plant solo aplica a --mode vw con --profile".to_string());
        }
        if let Some((flag, _)) = profile_only.iter().find(|(_, given)| *given) {
            return Err(format!("{flag} solo aplica a --mode vw con --profile"));
        }
        let dur = dur.ok_or("--dur es obligatorio")?;

        // Validaciones por modo
        match mode {
            Mode::Vw => {
                if transport != RadioTarget::BaseStation {
                    return Err("modo vw con --v/--w solo aplica a --transport base-station (los perfiles corren también en firasim y plant)".to_string());
                }
                if vision.is_some() {
                    return Err("modo vw con --v/--w NO usa --vision (solo los perfiles registran la pose)".to_string());
                }
                if skill.is_some() || target.is_some() {
                    return Err("modo vw NO acepta --skill ni --target".to_string());
                }
                if v_mm_s.is_none() || w_deg_s.is_none() {
                    return Err("modo vw requiere --v y --w, o --profile".to_string());
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
            Mode::Rutina => unreachable!("el modo rutina se valida arriba"),
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
            profile,
            plant,
            track,
            battery,
            battery_v,
            safe_box: DEFAULT_SAFE_BOX,
            rutina: None,
            robots: None,
            velocidad: None,
            repeticiones: None,
            teletransportar: false,
        })
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Modo vw
// ─────────────────────────────────────────────────────────────────────────────

async fn run_vw_mode(args: &Args, shutdown: Arc<AtomicBool>) -> Result<(), String> {
    if let Some(name) = &args.profile {
        let profile = sysid::profile(name).expect("validado");
        return run_vw_profile(args, &profile, shutdown).await;
    }
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

    // Un error de envío corta el bucle, pero también pasa por la parada antes de salir.
    let mut error_de_envio: Option<String> = None;
    while !shutdown.load(Ordering::Relaxed) && Instant::now() < deadline {
        interval.tick().await;
        if let Err(e) = transport.send_raw_vw_frame(slots).await {
            error_de_envio = Some(format!("send error: {e}"));
            break;
        }

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

    eprintln!("[skill_test] parada: frames en cero");
    rustengine::parada::mandar_ceros(async || transport.send_raw_vw_frame(zero_slots).await)
        .await;
    match error_de_envio {
        Some(e) => Err(e),
        None => Ok(()),
    }
}


// ─────────────────────────────────────────────────────────────────────────────
//  Modo vw con perfil (sysid)
// ─────────────────────────────────────────────────────────────────────────────

/// Pose cruda del robot registrado: (ms desde el arranque, x, y, θ).
type PoseSample = (f64, f32, f32, f64);

/// Visión perdida por más de este tiempo (ms) → corte.
const VISION_LOST_MS: f64 = 500.0;

/// Archivo hermano del log: `dir/v_steps.csv` → `dir/v_steps.<ext>`.
fn sidecar(log: &Path, ext: &str) -> PathBuf {
    let stem = log.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
    log.with_file_name(format!("{stem}.{ext}"))
}

/// Corte de seguridad: `Some(motivo)` si la última pose está fuera de la caja segura o
/// es más vieja que `VISION_LOST_MS`.
fn safety_cut(latest: Option<PoseSample>, now_ms: f64, safe_box: (f32, f32)) -> Option<String> {
    let Some((t, x, y, _)) = latest else {
        return Some("sin visión del robot".to_string());
    };
    if now_ms - t > VISION_LOST_MS {
        return Some(format!("visión perdida por {:.1} s", (now_ms - t) / 1000.0));
    }
    if x.abs() > safe_box.0 || y.abs() > safe_box.1 {
        return Some(format!(
            "robot en ({x:.2}, {y:.2}), fuera de la caja segura ±{:.2} × ±{:.2} m",
            safe_box.0, safe_box.1
        ));
    }
    None
}

fn team_id(team: TeamColor) -> u32 {
    match team {
        TeamColor::Blue => 0,
        TeamColor::Yellow => 1,
    }
}

/// Metadatos de la corrida (`<log>.meta.json`).
fn profile_meta(args: &Args, profile: &sysid::Profile, status: &str) -> serde_json::Value {
    let env = |k: &str| std::env::var(k).ok().filter(|v| !v.trim().is_empty());
    let unix_s = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    serde_json::json!({
        "profile": profile.name,
        "profile_dt_s": sysid::PROFILE_DT,
        "profile_duration_s": round3(profile.duration_s()),
        "transport": if args.plant { "plant" } else { transport_label(args.transport) },
        "vision": args.vision.map(|v| if v == VisionSource::FiraSim { "sim" } else { "real" }),
        "robot": args.robot,
        "track": args.track,
        "team": team_label(args.team),
        "battery": args.battery,
        "battery_v": args.battery_v,
        "safe_box_m": [round3(args.safe_box.0.into()), round3(args.safe_box.1.into())],
        "unix_time_s": unix_s,
        "engine_version": env!("CARGO_PKG_VERSION"),
        "real_actuator": env(rustengine::radio::actuator::ACTUATOR_ENV),
        "vision_noise": env(rustengine::vision_tools::VISION_NOISE_ENV),
        "vision_record": env(rustengine::vision_tools::VISION_RECORD_ENV),
        "status": status,
    })
}

fn round3(x: f64) -> f64 {
    (x * 1000.0).round() / 1000.0
}

fn write_meta(path: &Path, meta: &serde_json::Value) -> Result<(), String> {
    let text = serde_json::to_string_pretty(meta).expect("json");
    std::fs::write(path, text + "\n").map_err(|e| format!("{}: {e}", path.display()))
}

/// Fila del CSV de comandos del modo vw.
fn vw_row<'a>(args: &'a Args, t_ms: u64, tick: u32, (v, w): (i16, i16), pose: Option<PoseSample>) -> CsvRow<'a> {
    let mut slots = [(0i16, 0i16); SLOT_COUNT];
    slots[args.robot] = (v, w);
    CsvRow {
        t_ms,
        tick,
        mode: "vw",
        transport: if args.plant { "plant" } else { transport_label(args.transport) },
        vision: if args.plant { "plant" } else { vision_label(args.vision) },
        robot: args.robot,
        team: team_label(args.team),
        skill: args.profile.as_deref().unwrap_or(""),
        pose_x: pose.map(|p| p.1),
        pose_y: pose.map(|p| p.2),
        pose_theta: pose.map(|p| p.3),
        target_x: None,
        target_y: None,
        cmd_vx: None,
        cmd_vy: None,
        cmd_omega: None,
        v_mm_s: Some(v),
        w_deg_s: Some(w),
        // Como en el modo skill: el frame solo con la base.
        frame_str: if args.transport == RadioTarget::BaseStation {
            build_frame_from_vw(slots).trim_end().to_string()
        } else {
            String::new()
        },
        err_dist: None,
        err_heading: None,
        ball_x: None,
        ball_y: None,
        ball_vx: None,
        ball_vy: None,
    }
}

fn pose_writer(path: &Path) -> Result<LineWriter<File>, String> {
    let mut w = LineWriter::new(File::create(path).map_err(|e| format!("{}: {e}", path.display()))?);
    writeln!(w, "t_ms,x,y,theta").map_err(|e| e.to_string())?;
    Ok(w)
}

/// Levanta la visión con el tracker apagado (pose cruda) y registra cada frame del
/// robot `track` en el CSV de poses. Devuelve la última pose vista.
fn spawn_pose_recorder(
    source: VisionSource,
    team: u32,
    track: u32,
    mut out: LineWriter<File>,
    started: Instant,
) -> Arc<Mutex<Option<PoseSample>>> {
    let latest = Arc::new(Mutex::new(None));
    let (vision_tx, mut vision_rx) = mpsc::channel(256);
    tokio::spawn(async move {
        let mut vis = Vision::new(source, Arc::new(AtomicBool::new(false)));
        let (status_tx, _) = mpsc::channel(1);
        if let Err(err) = vis.run(vision_tx, status_tx).await {
            eprintln!("[skill_test] error de visión: {err}");
        }
    });
    let sink = latest.clone();
    tokio::spawn(async move {
        while let Some(ev) = vision_rx.recv().await {
            let VisionEvent::Robot(r) = ev else { continue };
            if r.team != team || r.id != track {
                continue;
            }
            let t_ms = started.elapsed().as_secs_f64() * 1000.0;
            let theta = f64::from(r.orientation);
            let _ = writeln!(out, "{t_ms:.1},{:.5},{:.5},{theta:.5}", r.position.x, r.position.y);
            *sink.lock().unwrap() = Some((t_ms, r.position.x, r.position.y, theta));
        }
    });
    latest
}

/// Destino en tiempo real de un perfil.
enum ProfileOut {
    Base(BaseStationTransport),
    Fira(FiraSimTransport),
}

impl ProfileOut {
    async fn send(&mut self, args: &Args, (v, w): (i16, i16)) -> Result<(), String> {
        match self {
            ProfileOut::Base(t) => {
                let mut slots = [(0i16, 0i16); SLOT_COUNT];
                slots[args.robot] = (v, w);
                t.send_raw_vw_frame(slots).await.map_err(|e| format!("send error: {e}"))
            }
            ProfileOut::Fira(t) => {
                let team = team_id(args.team) as i32;
                let id = args.robot as i32;
                let motion = MotionCommand {
                    id,
                    team,
                    vx: f64::from(v) / 1000.0,
                    vy: 0.0,
                    omega: f64::from(w).to_radians(),
                    orientation: 0.0,
                };
                let kicker = KickerCommand { id, team, kick_x: false, kick_z: false, dribbler: 0.0 };
                t.send_commands(&[RobotCommand { id, team, motion, kicker }])
                    .await
                    .map_err(|e| format!("send error: {e}"))
            }
        }
    }

    /// Período del loop: 20 Hz con la base (su cadencia); 60 Hz con FIRASim, para que
    /// la capa de actuador corra al ritmo del loop del engine.
    fn period(&self) -> Duration {
        match self {
            ProfileOut::Base(_) => Duration::from_millis(50),
            ProfileOut::Fira(_) => Duration::from_micros(16_667),
        }
    }
}

async fn run_vw_profile(args: &Args, profile: &sysid::Profile, shutdown: Arc<AtomicBool>) -> Result<(), String> {
    let log = args.log.as_ref().expect("validado");
    let meta_path = sidecar(log, "meta.json");
    let pose_path = sidecar(log, "pose.csv");
    let mut csv = CsvLogger::new(log).map_err(|e| format!("--log: {e}"))?;
    write_meta(&meta_path, &profile_meta(args, profile, "en curso"))?;
    eprintln!(
        "[skill_test] perfil {} ({:.1} s): {} — CSV {}, poses {}",
        profile.name,
        profile.duration_s(),
        profile.purpose,
        log.display(),
        pose_path.display()
    );
    if args.plant {
        let result = run_profile_on_plant(args, profile, &mut csv, &pose_path);
        let status = result.as_ref().map_or_else(|e| format!("error: {e}"), |_| "completo".to_string());
        write_meta(&meta_path, &profile_meta(args, profile, &status))?;
        return result;
    }

    let mut out = if args.transport == RadioTarget::BaseStation {
        ProfileOut::Base(BaseStationTransport::from_env(args.team).map_err(|e| format!("base-station: {e}"))?)
    } else {
        let mut t = FiraSimTransport::new("127.0.0.1", 20011).await.map_err(|e| format!("firasim: {e}"))?;
        place_on_firasim(&mut t, args).await?;
        ProfileOut::Fira(t)
    };

    let started = Instant::now();
    let source = args.vision.expect("validado");
    let track = args.track.expect("validado");
    let latest = spawn_pose_recorder(source, team_id(args.team), track, pose_writer(&pose_path)?, started);
    let now_ms = || started.elapsed().as_secs_f64() * 1000.0;

    // Antes de mover: la visión tiene que ver al robot dentro de la caja.
    let wait_until = Instant::now() + Duration::from_secs_f64(args.vision_timeout_s);
    while latest.lock().unwrap().is_none() {
        if Instant::now() >= wait_until || shutdown.load(Ordering::Relaxed) {
            let msg = format!("la visión no ve al robot {track} ({}) en {:.1} s", team_label(args.team), args.vision_timeout_s);
            write_meta(&meta_path, &profile_meta(args, profile, &format!("no empezó: {msg}")))?;
            return Err(msg);
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    if let Some(reason) = safety_cut(*latest.lock().unwrap(), now_ms(), args.safe_box) {
        write_meta(&meta_path, &profile_meta(args, profile, &format!("no empezó: {reason}")))?;
        return Err(format!("no empiezo: {reason}"));
    }

    let t0 = started.elapsed().as_secs_f64();
    let mut interval = tokio::time::interval(out.period());
    let mut tick: u32 = 0;
    let mut last_print = Instant::now();
    let status = loop {
        interval.tick().await;
        let t = started.elapsed().as_secs_f64() - t0;
        if shutdown.load(Ordering::Relaxed) {
            break Some("interrumpido (señal de terminación)".to_string());
        }
        if t >= profile.duration_s() {
            break None;
        }
        let pose = *latest.lock().unwrap();
        if let Some(reason) = safety_cut(pose, now_ms(), args.safe_box) {
            break Some(format!("cortado: {reason}"));
        }
        let vw = profile.at(t);
        // Un error de envío también pasa por la parada antes de salir.
        if let Err(e) = out.send(args, vw).await {
            break Some(format!("cortado: {e}"));
        }
        let _ = csv.write_row(&vw_row(args, now_ms() as u64, tick, vw, pose));
        if last_print.elapsed() >= Duration::from_secs(1) {
            eprintln!("[skill_test t={t:.1}/{:.1}s] vw=({},{})", profile.duration_s(), vw.0, vw.1);
            last_print = Instant::now();
        }
        tick = tick.wrapping_add(1);
    };

    eprintln!("[skill_test] fin del perfil: frames en cero");
    rustengine::parada::mandar_ceros(async || {
        out.send(args, (0, 0))
            .await
            .map_err(rustengine::radio::TransportError::from)
    })
    .await;
    // Medio segundo más de poses para ver el frenado.
    tokio::time::sleep(Duration::from_millis(500)).await;
    let label = status.clone().unwrap_or_else(|| "completo".to_string());
    write_meta(&meta_path, &profile_meta(args, profile, &label))?;
    match status {
        None => Ok(()),
        Some(reason) => Err(reason),
    }
}

/// FIRASim: el robot al centro mirando a +x, el resto y la pelota fuera del camino.
async fn place_on_firasim(t: &mut FiraSimTransport, args: &Args) -> Result<(), String> {
    let me = (team_id(args.team), args.robot as u32);
    let mut items: Vec<TeleportItem> = (0u32..2)
        .flat_map(|team| (0u32..3).map(move |id| (team, id)))
        .filter(|&r| r != me)
        .enumerate()
        .map(|(k, (team, id))| TeleportItem::Robot { team, id, x: -0.5 + 0.25 * k as f64, y: 1.0, theta: 0.0 })
        .collect();
    items.push(TeleportItem::Robot { team: me.0, id: me.1, x: 0.0, y: 0.0, theta: 0.0 });
    items.push(TeleportItem::Ball { x: -0.6, y: 0.5, vx: 0.0, vy: 0.0 });
    for _ in 0..5 {
        t.teleport(&items).await.map_err(|e| format!("firasim teleport: {e}"))?;
        tokio::time::sleep(Duration::from_millis(80)).await;
    }
    tokio::time::sleep(Duration::from_millis(300)).await;
    Ok(())
}

/// `--transport plant`: corre el perfil sin red ni tiempo real, a 60 Hz, con la capa de
/// actuador real si está `VSSL_REAL_ACTUATOR` (si no, el robot ejecuta la consigna
/// exacta). Escribe los mismos archivos que una corrida con visión.
fn run_profile_on_plant(args: &Args, profile: &sysid::Profile, csv: &mut CsvLogger, pose_path: &Path) -> Result<(), String> {
    use rustengine::radio::actuator::{ActuatorModel, ActuatorState, selected_measurement};
    const DT: f64 = 1.0 / 60.0;
    let model = selected_measurement()?.map(|(m, _)| ActuatorModel::for_plant(&m));
    let mut state = ActuatorState::default();
    let mut poses = pose_writer(pose_path)?;
    let (mut x, mut y, mut th) = (0.0f64, 0.0f64, 0.0f64);
    let ticks = (profile.duration_s() / DT).ceil() as u32 + 30;
    for tick in 0..ticks {
        let t = f64::from(tick) * DT;
        let t_ms = t * 1000.0;
        writeln!(poses, "{t_ms:.1},{x:.5},{y:.5},{th:.5}").map_err(|e| e.to_string())?;
        let vw = profile.at(t);
        let pose = Some((t_ms, x as f32, y as f32, th));
        csv.write_row(&vw_row(args, t_ms.round() as u64, tick, vw, pose)).map_err(|e| e.to_string())?;
        let (v, w) = match &model {
            Some(m) => state.step(m, vw.0, vw.1, DT),
            None => (f64::from(vw.0) / 1000.0, f64::from(vw.1).to_radians()),
        };
        x += v * th.cos() * DT;
        y += v * th.sin() * DT;
        th = rustengine::motion::Motion::normalize_angle(th + w * DT);
    }
    Ok(())
}

fn print_profiles() {
    println!("Perfiles de sysid (20 Hz, ida y vuelta, pausas de 1 s en cero):");
    for p in sysid::all_profiles() {
        let (max, end) = p.ideal_excursion_m();
        println!(
            "  {:<8} {:>5.1} s  recorrido máx. {:.2} m (vuelve a {:.3} m)  {}",
            p.name,
            p.duration_s(),
            max,
            end,
            p.purpose
        );
    }
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
        referee: None,
        exportar_depuracion: false,
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
        // --robot en modo skill es el id de visión; el frame va a su posición de radio.
        radio_slot: radio_slot(
            args.robot as i32,
            &rustengine::params::params().robot.radio_slot_by_vision_id,
        ),
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
//  Modo rutina
// ─────────────────────────────────────────────────────────────────────────────

/// La rutina por consola: aborta la rutina con Ctrl+C (`detener`) o con el tope de
/// `--dur`, para que el resumen se escriba antes de parar el lazo.
struct DecisorDeConsola {
    rutina: RoutineDecider,
    detener: Arc<AtomicBool>,
    /// Tick del tope de `--dur`.
    tope: Option<u32>,
}

impl TickDecider for DecisorDeConsola {
    fn decide(&mut self, tick: u32, world: &rustengine::world::World) -> Vec<rustengine::coach::SkillChoice> {
        self.rutina.decide(tick, world)
    }

    fn observar(&mut self, ctx: &ContextoTick<'_>) {
        self.rutina.observar(ctx);
        if self.detener.load(Ordering::Relaxed) {
            self.rutina.ordenar(OrdenCorrida::Abortar);
        }
        if self.tope.is_some_and(|t| ctx.tick >= t) {
            self.rutina.abortar("tiempo máximo de la corrida");
        }
    }

    fn perfil_velocidad(&self) -> Option<PerfilVelocidad> {
        self.rutina.perfil_velocidad()
    }

    fn tomar_teleport(&mut self) -> Vec<TeleportItem> {
        self.rutina.tomar_teleport()
    }

    fn rutina(&self) -> Option<Arc<RutinaVista>> {
        TickDecider::rutina(&self.rutina)
    }
}

/// `--listar-rutinas`: cada rutina de la carpeta con su validez (lado de `VSSL_SIDE` para
/// el equipo azul).
fn listar_rutinas() {
    let dir = rustengine::rutinas::carpeta();
    match rustengine::rutinas::cargar_carpeta(&dir, &ContextoValidacion::vigente(0)) {
        Err(e) => eprintln!("error: {e}"),
        Ok(entradas) => {
            println!("Rutinas en {}:", dir.display());
            for e in entradas {
                match (e.valida(), &e.rutina) {
                    (Some(r), _) => println!("  ✓ {:<16} {} ({}): {}", e.id, r.nombre, r.texto_robots(), r.descripcion),
                    (None, rutina) => {
                        let nombre = rutina.as_ref().map_or(String::new(), |r| format!(" {}", r.nombre));
                        println!("  ✗ {:<16}{nombre}", e.id);
                        for razon in &e.razones {
                            println!("      - {razon}");
                        }
                    }
                }
            }
        }
    }
}

/// Corre la rutina y devuelve el código de salida: 0 completada (o detenida en bucle), 2
/// abortada.
async fn run_rutina_mode(args: &Args, shutdown: Arc<AtomicBool>, detener: Arc<AtomicBool>) -> Result<i32, String> {
    let own_team = args.team.as_team_id();
    let nombre = args.rutina.as_deref().expect("validado");
    let entrada = rustengine::rutinas::buscar(nombre, &ContextoValidacion::vigente(own_team))?;
    let Some(rutina) = entrada.valida().cloned() else {
        return Err(format!(
            "la rutina `{}` no se puede correr:\n  - {}",
            entrada.id,
            entrada.razones.join("\n  - ")
        ));
    };
    let vision_source = args.vision.expect("validado");
    let simulacion = args.transport != RadioTarget::BaseStation;
    let mut op = OpcionesCorrida::por_defecto(&rutina, simulacion);
    if let Some(ids) = &args.robots {
        op.eleccion = Eleccion::Manual(ids.clone());
    }
    if let Some(p) = args.velocidad {
        op.velocidad_pct = p;
    }
    if let Some(r) = args.repeticiones {
        op.repeticiones = r;
    }
    op.teletransportar = args.teletransportar;
    op.inicio_automatico = true;
    eprintln!(
        "[rutina] {} ({}) al {} %, {} {}",
        rutina.nombre,
        rutina.id,
        op.velocidad_pct,
        op.repeticiones.texto(),
        if op.repeticiones == Repeticiones::Veces(1) { "vez" } else { "veces" }
    );
    let tope = args
        .dur
        .is_finite()
        .then(|| (args.dur / rustengine::motion::CONTROL_DT).ceil() as u32);
    let decider = Box::new(DecisorDeConsola {
        rutina: RoutineDecider::vigente(rutina, op, own_team, SLOT_COUNT),
        detener: detener.clone(),
        tope,
    });

    let config = ControlLoopConfig {
        own_team,
        num_robots: SLOT_COUNT,
        vision_source,
        radio_target: args.transport,
        max_ticks: None,
        vision_timeout: matches!(vision_source, VisionSource::SslVision)
            .then(|| Duration::from_secs_f64(args.vision_timeout_s)),
        referee: None,
        // El CSV de partido lleva la depuración, como en `main`.
        exportar_depuracion: true,
    };

    let mut logger = match &args.log {
        Some(p) => Some(MatchLogger::new(p, own_team).map_err(|e| format!("--log: {e}"))?),
        None => None,
    };
    let ruta_resumen = args.log.as_ref().map(|p| junto_al_log(p, ".resumen.json"));
    let corrida = corrida_json(transport_label(args.transport), vision_label(args.vision), own_team);
    let resultado: Arc<Mutex<Option<Arc<ResumenCorrida>>>> = Arc::new(Mutex::new(None));
    let (resultado_hook, shutdown_hook) = (resultado.clone(), shutdown.clone());
    let mut ultima_frase = String::new();
    let mut ticks_deteniendo = 0u32;
    let on_tick: Box<dyn FnMut(&TickRecord<'_>) + Send> = Box::new(move |rec: &TickRecord<'_>| {
        if let Some(l) = logger.as_mut() {
            let _ = l.write_tick(rec);
        }
        if let Some(v) = rec.rutina {
            if v.frase != ultima_frase {
                eprintln!("[rutina] {}", v.frase);
                ultima_frase = v.frase.clone();
            }
            let mut guardado = resultado_hook.lock().unwrap();
            if guardado.is_none()
                && let Some(r) = &v.resumen
            {
                for linea in r.lineas() {
                    eprintln!("[rutina] {linea}");
                }
                if let Some(ruta) = &ruta_resumen {
                    match escribir_resumen(ruta, r, corrida.clone()) {
                        Ok(()) => eprintln!("[rutina] resumen → {}", ruta.display()),
                        Err(e) => eprintln!("[rutina] no se pudo escribir {}: {e}", ruta.display()),
                    }
                }
                *guardado = Some(r.clone());
                shutdown_hook.store(true, Ordering::Relaxed);
            }
        }
        // Por si la rutina no cierra tras la señal: el lazo para igual a los ~2 s.
        if detener.load(Ordering::Relaxed) {
            ticks_deteniendo += 1;
            if ticks_deteniendo > 120 {
                shutdown_hook.store(true, Ordering::Relaxed);
            }
        }
    });

    run_control_loop(config, decider, Some(on_tick), None as Option<GuiChannels>, shutdown)
        .await
        .map_err(|e| format!("control loop: {e}"))?;
    let resumen = resultado.lock().unwrap().clone();
    match resumen.map(|r| r.resultado) {
        Some(Resultado::Completada | Resultado::Detenida) => Ok(0),
        Some(Resultado::Abortada) => Ok(2),
        None => Err("el lazo terminó antes de cerrar la rutina".to_string()),
    }
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
        Err(msg) if msg == "__PROFILES__" => {
            print_profiles();
            std::process::exit(0);
        }
        Err(msg) if msg == "__RUTINAS__" => {
            rustengine::params::TeamParams::install_or_exit("skill_test");
            listar_rutinas();
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
    // SIGINT, SIGTERM o SIGHUP: el modo en curso para con ceros antes de salir. En el modo
    // rutina, la señal primero aborta la rutina (para escribir el resumen) y el lazo para
    // al cerrarla.
    let shutdown = Arc::new(AtomicBool::new(false));
    let detener = Arc::new(AtomicBool::new(false));
    {
        let _runtime = rt.enter();
        let flag = if args.mode == Mode::Rutina { detener.clone() } else { shutdown.clone() };
        if let Err(e) = rustengine::parada::al_recibir_senal(flag) {
            eprintln!("[skill_test] no pude registrar las señales de terminación: {e}");
        }
    }

    let result = rt.block_on(async {
        match args.mode {
            Mode::Vw => run_vw_mode(&args, shutdown.clone()).await.map(|_| 0),
            Mode::Skill => run_skill_mode(&args, shutdown.clone()).await.map(|_| 0),
            Mode::Rutina => run_rutina_mode(&args, shutdown.clone(), detener.clone()).await,
        }
    });

    match result {
        Ok(0) => {}
        Ok(codigo) => std::process::exit(codigo),
        Err(msg) => {
            eprintln!("error: {msg}");
            std::process::exit(1);
        }
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
            "id de VISIÓN",
            "robot.radio_slot_by_vision_id",
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

    // ---------- Modo vw con perfil (sysid) ----------

    fn profile_args(extra: &[&str]) -> Vec<String> {
        let mut v: Vec<String> = ["--mode", "vw", "--team", "blue", "--robot", "1", "--profile", "v_steps", "--log", "/tmp/x.csv"]
            .iter()
            .map(|s| s.to_string())
            .collect();
        v.extend(extra.iter().map(|s| s.to_string()));
        v
    }

    fn parse_profile(extra: &[&str]) -> Result<Args, String> {
        Args::parse(profile_args(extra))
    }

    const BASE: [&str; 8] = ["--transport", "base-station", "--vision", "real", "--track", "3", "--battery", "full"];

    #[test]
    fn profile_on_base_station_valid() {
        let a = parse_profile(&[&BASE[..], &["--battery-v", "8.1", "--safe-box", "0.5,0.4"]].concat()).unwrap();
        assert_eq!(a.profile.as_deref(), Some("v_steps"));
        assert_eq!((a.track, a.battery.as_deref(), a.battery_v), (Some(3), Some("full"), Some(8.1)));
        assert_eq!(a.safe_box, (0.5, 0.4));
        assert!(!a.plant);
        assert!((a.dur - sysid::profile("v_steps").unwrap().duration_s()).abs() < 1e-9);
    }

    #[test]
    fn profile_on_base_station_requires_battery_track_and_real_vision() {
        let without = |skip: &str| {
            let mut v: Vec<&str> = Vec::new();
            for pair in BASE.chunks(2) {
                if pair[0] != skip {
                    v.extend_from_slice(pair);
                }
            }
            parse_profile(&v).unwrap_err()
        };
        assert!(without("--battery").contains("--battery"));
        assert!(without("--track").contains("--track"));
        assert!(without("--vision").contains("--vision real"));
        let sim = parse_profile(&["--transport", "base-station", "--vision", "sim", "--track", "3", "--battery", "full"]);
        assert!(sim.unwrap_err().contains("--vision real"));
        assert!(parse_profile(&[&BASE[..6], &["--battery", "media"]].concat()).unwrap_err().contains("full|half"));
    }

    #[test]
    fn profile_excludes_v_w_dur_and_needs_a_log() {
        for extra in [["--v", "100"], ["--w", "90"], ["--dur", "2"]] {
            let err = parse_profile(&[&BASE[..], &extra[..]].concat()).unwrap_err();
            assert!(err.contains("excluye"), "{extra:?}: {err}");
        }
        let mut no_log = profile_args(&BASE);
        no_log.retain(|a| a != "--log" && a != "/tmp/x.csv");
        assert!(Args::parse(no_log).unwrap_err().contains("--log"));
        assert!(parse_profile(&[&BASE[..], &["--dry-run"]].concat()).unwrap_err().contains("plant"));
        assert!(parse_profile(&["--transport", "plant", "--profile", "nope"]).unwrap_err().contains("v_steps"));
    }

    #[test]
    fn profile_transports() {
        // FIRASim: visión sim, y el robot registrado es el comandado.
        let a = parse_profile(&["--transport", "firasim", "--vision", "sim"]).unwrap();
        assert_eq!((a.transport, a.track, a.battery), (RadioTarget::FiraSim, Some(1), None));
        assert!(parse_profile(&["--transport", "firasim", "--vision", "real"]).unwrap_err().contains("--vision sim"));
        // Planta offline: sin visión ni --track.
        let a = parse_profile(&["--transport", "plant"]).unwrap();
        assert!(a.plant && a.vision.is_none() && a.track.is_none());
        assert!(parse_profile(&["--transport", "plant", "--vision", "sim"]).is_err());
        assert!(parse_profile(&["--transport", "plant", "--track", "1"]).is_err());
        assert!(parse_profile(&["--transport", "grsim", "--vision", "sim"]).is_err());
    }

    #[test]
    fn profile_only_flags_are_rejected_elsewhere() {
        let bring_up = ["--transport", "base-station", "--mode", "vw", "--team", "blue", "--robot", "1", "--v", "300", "--w", "0", "--dur", "2"];
        for extra in [&["--battery", "full"][..], &["--track", "1"], &["--safe-box", "0.5,0.5"], &["--battery-v", "8"]] {
            let err = parse(&[&bring_up[..], extra].concat()).unwrap_err();
            assert!(err.contains("--profile"), "{extra:?}: {err}");
        }
        let plant = ["--transport", "plant", "--mode", "vw", "--team", "blue", "--robot", "1", "--v", "300", "--w", "0", "--dur", "2"];
        assert!(parse(&plant).unwrap_err().contains("--profile"));
        // El bring-up no cambia.
        assert!(parse(&bring_up).is_ok());
        assert!(parse(&["--list-profiles"]).unwrap_err() == "__PROFILES__");
    }

    #[test]
    fn safety_cut_on_box_and_lost_vision() {
        let b = DEFAULT_SAFE_BOX;
        assert_eq!(safety_cut(Some((1000.0, 0.2, -0.3, 0.0)), 1100.0, b), None);
        assert!(safety_cut(Some((1000.0, 0.65, 0.0, 0.0)), 1010.0, b).unwrap().contains("caja segura"));
        assert!(safety_cut(Some((1000.0, 0.0, -0.55, 0.0)), 1010.0, b).unwrap().contains("caja segura"));
        assert!(safety_cut(Some((1000.0, 0.0, 0.0, 0.0)), 1501.0, b).unwrap().contains("visión perdida"));
        assert_eq!(safety_cut(Some((1000.0, 0.0, 0.0, 0.0)), 1499.0, b), None);
        assert!(safety_cut(None, 0.0, b).is_some());
    }

    #[test]
    fn sidecar_files_sit_next_to_the_log() {
        assert_eq!(sidecar(Path::new("/s/v_steps.csv"), "meta.json"), PathBuf::from("/s/v_steps.meta.json"));
        assert_eq!(sidecar(Path::new("v_steps"), "pose.csv"), PathBuf::from("v_steps.pose.csv"));
    }

    #[test]
    fn help_documents_profiles_and_safety() {
        for needle in ["--profile", "--list-profiles", "--battery", "--track", "--safe-box", "plant", "Ctrl+C", "levantar el robot"] {
            assert!(HELP_TEXT.contains(needle), "falta '{needle}' en el --help");
        }
    }

    // ---------- Modo rutina ----------

    const RUTINA: [&str; 10] = [
        "--transport", "firasim", "--vision", "sim", "--team", "blue", "--mode", "rutina", "--rutina", "triangulo",
    ];

    fn parse_rutina(extra: &[&str]) -> Result<Args, String> {
        let mut v: Vec<&str> = RUTINA.to_vec();
        v.extend_from_slice(extra);
        parse(&v)
    }

    #[test]
    fn routine_mode_valid() {
        let a = parse_rutina(&["--repeticiones", "10", "--teletransportar"]).unwrap();
        assert_eq!(a.mode, Mode::Rutina);
        assert_eq!(a.rutina.as_deref(), Some("triangulo"));
        assert_eq!(a.repeticiones, Some(Repeticiones::Veces(10)));
        assert!(a.teletransportar);
        assert_eq!(a.robots, None, "automático");
        assert!(a.dur.is_infinite(), "sin --dur no hay tope");
        let b = parse_rutina(&["--robots", "1,0", "--velocidad", "60", "--repeticiones", "bucle", "--dur", "90"]).unwrap();
        assert_eq!(b.robots, Some(vec![1, 0]));
        assert_eq!(b.velocidad, Some(60));
        assert_eq!(b.repeticiones, Some(Repeticiones::Bucle));
        assert_eq!(b.dur, 90.0);
        assert_eq!(parse_rutina(&["--robots", "auto"]).unwrap().robots, None);
    }

    #[test]
    fn routine_mode_rejects_robot_and_skill_flags() {
        let err = parse_rutina(&["--robot", "1"]).unwrap_err();
        assert!(err.contains("no usa --robot"), "{err}");
        let err = parse_rutina(&["--skill", "goto"]).unwrap_err();
        assert!(err.contains("NO acepta --skill"), "{err}");
        let err = parse(&["--transport", "firasim", "--team", "blue", "--mode", "rutina", "--rutina", "x"]).unwrap_err();
        assert!(err.contains("requiere --vision"), "{err}");
        let err = parse(&["--transport", "firasim", "--vision", "sim", "--team", "blue", "--mode", "rutina"]).unwrap_err();
        assert!(err.contains("requiere --rutina"), "{err}");
    }

    #[test]
    fn routine_teleport_is_only_for_simulation() {
        let err = parse(&[
            "--transport", "base-station", "--vision", "real", "--team", "blue", "--mode", "rutina", "--rutina",
            "triangulo", "--teletransportar",
        ])
        .unwrap_err();
        assert!(err.contains("solo de simulación"), "{err}");
    }

    #[test]
    fn routine_speed_and_repetitions_are_checked() {
        for malo in ["10", "65", "110"] {
            let err = parse_rutina(&["--velocidad", malo]).unwrap_err();
            assert!(err.contains("de 20 a 100 en pasos de 10"), "{err}");
        }
        assert!(parse_rutina(&["--repeticiones", "0"]).unwrap_err().contains("repeticiones"));
        assert!(parse_rutina(&["--robots", "1,9"]).unwrap_err().contains("fuera de rango"));
    }

    #[test]
    fn routine_flags_only_in_routine_mode() {
        let err = parse(&[
            "--transport", "firasim", "--vision", "sim", "--mode", "skill", "--skill", "goto", "--target", "0.3,0",
            "--robot", "0", "--team", "blue", "--dur", "5", "--velocidad", "60",
        ])
        .unwrap_err();
        assert!(err.contains("--velocidad solo aplica a --mode rutina"), "{err}");
        assert_eq!(parse(&["--listar-rutinas"]).unwrap_err(), "__RUTINAS__");
    }

    #[test]
    fn help_documents_the_routine_mode() {
        for needle in ["--mode <vw|skill|rutina>", "--rutina NOMBRE", "--teletransportar", "--listar-rutinas", ".resumen.json"] {
            assert!(HELP_TEXT.contains(needle), "falta '{needle}' en el --help");
        }
    }
}
