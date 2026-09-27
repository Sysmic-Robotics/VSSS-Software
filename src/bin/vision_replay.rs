//! Replay de grabaciones de visión (`VSSL_VISION_RECORD`).
//!
//! Dos modos:
//!
//! 1. `--publish`: re-publica los paquetes crudos en el multicast de su fuente con
//!    el timing original. El engine los consume como si fueran en vivo, así que
//!    una grabación de 15 minutos de cámara real sirve para probar coach/skills
//!    en el escritorio sin robots (los comandos van a FIRASim o a nadie).
//! 2. `--ekf-csv salida.csv`: pasa la grabación por el tracker OFFLINE y escribe
//!    por detección la medición cruda y la salida filtrada. Con `--params otro.json`
//!    se prueban Q/R distintos sobre la MISMA grabación: así se valida el EKF con
//!    datos reales antes de jugar (protocolo de torneo, plan §3).
//!
//! Salida del modo 2 (stderr): por entidad, n, RMS del residuo crudo−filtrado en
//! posición (mm) y orientación (°) y |v| media. Con el robot QUIETO el RMS del
//! residuo ≈ σ del ruido de la cámara (medición M1).

use rustengine::params::TeamParams;
use rustengine::tracker::Tracker;
use rustengine::vision_tools::{Detection, VisionRecording, parse_detections};
use std::collections::HashMap;
use std::io::Write;

const HELP: &str = "vision_replay — reproduce o analiza grabaciones de visión (VSSL_VISION_RECORD)

USO:
    vision_replay --file grab.bin --publish [--loop] [--speed 1.0]
    vision_replay --file grab.bin --ekf-csv salida.csv [--params otro.json] [--no-tracker]

    --publish        re-publica los paquetes en el multicast de la fuente grabada
                     (FIRASim 224.0.0.1:10002, SSL-Vision 224.5.23.2:10015) con el
                     timing original. --loop repite; --speed 2.0 va al doble.
    --ekf-csv RUTA   corre el tracker offline y escribe CSV: t_ms,team,id,raw_x,raw_y,
                     raw_theta,f_x,f_y,f_theta,vx,vy,omega. Resumen por entidad a stderr.
    --params RUTA    JSON de parámetros alternativo para el modo --ekf-csv
                     (por defecto, el vigente: VSSL_PARAMS o config/team_params.json).
    --no-tracker     en --ekf-csv, copia la medición cruda como filtrada (referencia).
";

struct Args {
    file: String,
    publish: bool,
    loop_forever: bool,
    speed: f64,
    ekf_csv: Option<String>,
    params: Option<String>,
    no_tracker: bool,
}

fn parse_args() -> Result<Args, String> {
    let mut it = std::env::args().skip(1);
    let mut a = Args {
        file: String::new(),
        publish: false,
        loop_forever: false,
        speed: 1.0,
        ekf_csv: None,
        params: None,
        no_tracker: false,
    };
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--file" => a.file = it.next().ok_or("--file requiere una ruta")?,
            "--publish" => a.publish = true,
            "--loop" => a.loop_forever = true,
            "--speed" => {
                a.speed = it
                    .next()
                    .ok_or("--speed requiere un número")?
                    .parse()
                    .map_err(|_| "--speed inválido")?;
            }
            "--ekf-csv" => a.ekf_csv = Some(it.next().ok_or("--ekf-csv requiere una ruta")?),
            "--params" => a.params = Some(it.next().ok_or("--params requiere una ruta")?),
            "--no-tracker" => a.no_tracker = true,
            "-h" | "--help" => return Err("__HELP__".into()),
            other => return Err(format!("argumento desconocido: {other}")),
        }
    }
    if a.file.is_empty() {
        return Err("falta --file".into());
    }
    if !a.publish && a.ekf_csv.is_none() {
        return Err("indica --publish o --ekf-csv".into());
    }
    if a.speed <= 0.0 {
        return Err("--speed debe ser > 0".into());
    }
    Ok(a)
}

async fn publish(rec: &VisionRecording, loop_forever: bool, speed: f64) -> Result<(), String> {
    use tokio::net::UdpSocket;
    let socket = UdpSocket::bind("0.0.0.0:0").await.map_err(|e| e.to_string())?;
    let _ = socket.set_multicast_ttl_v4(1);
    let _ = socket.set_multicast_loop_v4(true);
    let dest = format!("{}:{}", rec.source.multicast_ip(), rec.source.port());
    eprintln!(
        "[vision_replay] publicando {} paquetes ({:.1} s) en {dest}, velocidad ×{speed}{}",
        rec.packets.len(),
        rec.duration_s(),
        if loop_forever { ", en bucle" } else { "" }
    );
    loop {
        let t0 = tokio::time::Instant::now();
        for p in &rec.packets {
            let at = t0 + std::time::Duration::from_secs_f64(p.t_us as f64 / 1e6 / speed);
            tokio::time::sleep_until(at).await;
            socket.send_to(&p.data, &dest).await.map_err(|e| e.to_string())?;
        }
        if !loop_forever {
            break;
        }
    }
    Ok(())
}

#[derive(Default)]
struct EntityStats {
    n: u64,
    sum_res_pos2: f64,
    sum_res_theta2: f64,
    sum_speed: f64,
}

fn norm_angle(a: f64) -> f64 {
    let mut a = a % (2.0 * std::f64::consts::PI);
    if a > std::f64::consts::PI {
        a -= 2.0 * std::f64::consts::PI;
    } else if a < -std::f64::consts::PI {
        a += 2.0 * std::f64::consts::PI;
    }
    a
}

fn ekf_csv(rec: &VisionRecording, out_path: &str, params_path: Option<&str>, no_tracker: bool) -> Result<(), String> {
    let vision_params = match params_path {
        Some(p) => TeamParams::from_file(p)?.vision,
        None => rustengine::params::params().vision.clone(),
    };
    let mut tracker = Tracker::with_params(vision_params);
    let mut out = std::fs::File::create(out_path).map_err(|e| format!("{out_path}: {e}"))?;
    writeln!(out, "t_ms,team,id,raw_x,raw_y,raw_theta,f_x,f_y,f_theta,vx,vy,omega").map_err(|e| e.to_string())?;
    let mut last_t: HashMap<(i32, i32), u64> = HashMap::new();
    let mut stats: HashMap<(i32, i32), EntityStats> = HashMap::new();
    let mut detections = 0u64;
    for p in &rec.packets {
        for d in parse_detections(rec.source, &p.data) {
            let key = (d.team, d.id);
            // Mismo dt que el receptor en vivo: wall-clock acotado a 5–100 ms.
            let dt = last_t
                .get(&key)
                .map(|t| (p.t_us.saturating_sub(*t)) as f64 / 1e6)
                .unwrap_or(0.016)
                .clamp(0.005, 0.1);
            last_t.insert(key, p.t_us);
            let (fx, fy, ft, vx, vy, om) = if no_tracker {
                (d.x, d.y, d.theta, 0.0, 0.0, 0.0)
            } else {
                tracker.track(d.team, d.id, d.x, d.y, d.theta, dt)
            };
            writeln!(
                out,
                "{},{},{},{},{},{},{},{},{},{},{},{}",
                p.t_us / 1000, d.team, d.id, d.x, d.y, d.theta, fx, fy, ft, vx, vy, om
            )
            .map_err(|e| e.to_string())?;
            let s = stats.entry(key).or_default();
            s.n += 1;
            s.sum_res_pos2 += (d.x - fx).powi(2) + (d.y - fy).powi(2);
            if !d.is_ball() {
                s.sum_res_theta2 += norm_angle(d.theta - ft).powi(2);
            }
            s.sum_speed += (vx * vx + vy * vy).sqrt();
            detections += 1;
        }
    }
    eprintln!(
        "[vision_replay] {} paquetes, {} detecciones, {:.1} s → {out_path}",
        rec.packets.len(),
        detections,
        rec.duration_s()
    );
    eprintln!("{:<10} {:>6} {:>14} {:>14} {:>10}", "entidad", "n", "rms_pos_mm", "rms_theta_deg", "|v|_medio");
    let mut keys: Vec<_> = stats.keys().copied().collect();
    keys.sort();
    for key in keys {
        let s = &stats[&key];
        let label = if key.0 < 0 { "pelota".to_string() } else { format!("{}-{}", if key.0 == 0 { "azul" } else { "amar" }, key.1) };
        let rms_pos = (s.sum_res_pos2 / s.n as f64).sqrt() * 1000.0;
        let rms_theta = if key.0 < 0 { 0.0 } else { (s.sum_res_theta2 / s.n as f64).sqrt().to_degrees() };
        eprintln!("{:<10} {:>6} {:>14.2} {:>14.2} {:>10.3}", label, s.n, rms_pos, rms_theta, s.sum_speed / s.n as f64);
    }
    Ok(())
}

fn main() {
    let args = match parse_args() {
        Ok(a) => a,
        Err(e) if e == "__HELP__" => {
            print!("{HELP}");
            return;
        }
        Err(e) => {
            eprintln!("error: {e}\n\n{HELP}");
            std::process::exit(2);
        }
    };
    TeamParams::install_or_exit("vision_replay");
    let rec = match VisionRecording::read(&args.file) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("error leyendo {}: {e}", args.file);
            std::process::exit(1);
        }
    };
    eprintln!(
        "[vision_replay] {}: fuente {:?}, {} paquetes, {:.1} s",
        args.file,
        rec.source,
        rec.packets.len(),
        rec.duration_s()
    );
    if let Some(csv) = args.ekf_csv.as_deref()
        && let Err(e) = ekf_csv(&rec, csv, args.params.as_deref(), args.no_tracker)
    {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
    if args.publish {
        let rt = tokio::runtime::Runtime::new().expect("tokio runtime");
        if let Err(e) = rt.block_on(publish(&rec, args.loop_forever, args.speed)) {
            eprintln!("error publicando: {e}");
            std::process::exit(1);
        }
    }
}

#[allow(dead_code)]
fn _assert_detection_is_pub(d: Detection) -> f64 {
    d.x
}
