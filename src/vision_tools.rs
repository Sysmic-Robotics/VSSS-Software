//! Herramientas de percepción (Plan Equipo Heurístico §3, "F0"):
//!
//! - [`NoiseProxy`]: inyecta en el simulador el ruido de una cámara real
//!   (σ de posición/orientación, latencia, pérdida de frames) con los valores de
//!   `config/team_params.json` → `vision.proxy_*`. Regla del equipo: **nada se
//!   acepta en sim limpio**; se activa con `VSSL_VISION_NOISE=1`.
//! - [`VisionRecorder`] / [`VisionRecording`]: graban y leen los paquetes UDP
//!   crudos de visión (`VSSL_VISION_RECORD=ruta`) para reproducirlos después
//!   (`vision_replay --publish`) o para pasar la grabación por el EKF offline con
//!   otros parámetros (`vision_replay --ekf-csv`).
//! - [`parse_detections`]: parser puro FIRA / SSL-Vision → detecciones en metros,
//!   compartido por el replay offline y los tests.

use crate::params::VisionParams;
use crate::protos::fira_packet::Environment as FiraEnvironment;
use crate::protos::ssl_vision_wrapper::SSL_WrapperPacket;
use crate::vision::VisionSource;
use protobuf::Message;
use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::Path;
use std::time::{Duration, Instant};

// ─────────────────────────────────────────────────────────────────────────────
//  RNG determinista (sin dependencias): xorshift64* + Box-Muller
// ─────────────────────────────────────────────────────────────────────────────

pub struct XorShift64 {
    state: u64,
    spare_gauss: Option<f64>,
}

impl XorShift64 {
    pub fn new(seed: u64) -> Self {
        Self {
            state: if seed == 0 { 0x9E37_79B9_7F4A_7C15 } else { seed },
            spare_gauss: None,
        }
    }

    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Uniforme en [0, 1).
    pub fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Normal estándar (Box-Muller).
    pub fn gauss(&mut self) -> f64 {
        if let Some(g) = self.spare_gauss.take() {
            return g;
        }
        let u1 = (1.0 - self.next_f64()).max(f64::MIN_POSITIVE);
        let u2 = self.next_f64();
        let r = (-2.0 * u1.ln()).sqrt();
        let (s, c) = (2.0 * std::f64::consts::PI * u2).sin_cos();
        self.spare_gauss = Some(r * s);
        r * c
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Proxy de ruido de cámara para el simulador
// ─────────────────────────────────────────────────────────────────────────────

pub const VISION_NOISE_ENV: &str = "VSSL_VISION_NOISE";

/// Ruido de percepción inyectado ANTES del tracker, con el mismo pipeline que en
/// cancha: σ gaussiana en posición y orientación, latencia (retención de paquetes)
/// y pérdida de frames. Los valores vienen de `VisionParams::proxy_*` (defaults de
/// literatura; reemplazar por la medición M1).
pub struct NoiseProxy {
    pub sigma_pos_m: f64,
    pub sigma_theta_rad: f64,
    pub latency: Duration,
    pub drop_prob: f64,
    rng: XorShift64,
}

impl NoiseProxy {
    pub fn from_params(p: &VisionParams, seed: u64) -> Self {
        Self {
            sigma_pos_m: p.proxy_sigma_pos_m.max(0.0),
            sigma_theta_rad: p.proxy_sigma_theta_rad.max(0.0),
            latency: Duration::from_secs_f64(p.proxy_latency_ms.max(0.0) / 1000.0),
            drop_prob: p.proxy_drop_prob.clamp(0.0, 1.0),
            rng: XorShift64::new(seed),
        }
    }

    /// `VSSL_VISION_NOISE=1|true|on` → proxy activo con los parámetros vigentes.
    pub fn from_env(p: &VisionParams) -> Option<Self> {
        let on = std::env::var(VISION_NOISE_ENV)
            .map(|v| matches!(v.trim().to_ascii_lowercase().as_str(), "1" | "true" | "on"))
            .unwrap_or(false);
        on.then(|| {
            let seed = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_nanos() as u64)
                .unwrap_or(1);
            Self::from_params(p, seed)
        })
    }

    pub fn describe(&self) -> String {
        format!(
            "σ_pos={:.1} mm, σ_θ={:.2}°, latencia={} ms, pérdida={:.1} %",
            self.sigma_pos_m * 1000.0,
            self.sigma_theta_rad.to_degrees(),
            self.latency.as_millis(),
            self.drop_prob * 100.0
        )
    }

    /// `true` si este paquete se pierde.
    pub fn drop_packet(&mut self) -> bool {
        self.drop_prob > 0.0 && self.rng.next_f64() < self.drop_prob
    }

    pub fn perturb_pos(&mut self, x: f64, y: f64) -> (f64, f64) {
        if self.sigma_pos_m <= 0.0 {
            return (x, y);
        }
        (
            x + self.rng.gauss() * self.sigma_pos_m,
            y + self.rng.gauss() * self.sigma_pos_m,
        )
    }

    pub fn perturb_theta(&mut self, theta: f64) -> f64 {
        if self.sigma_theta_rad <= 0.0 {
            return theta;
        }
        theta + self.rng.gauss() * self.sigma_theta_rad
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Grabación de paquetes crudos
// ─────────────────────────────────────────────────────────────────────────────

pub const VISION_RECORD_ENV: &str = "VSSL_VISION_RECORD";
/// Cabecera: magic (8 B) + fuente (1 B: 0 = FIRASim, 1 = SSL-Vision).
/// Luego, por paquete: `t_us` (u64 LE, desde el inicio) + `len` (u32 LE) + bytes.
pub const RECORD_MAGIC: &[u8; 8] = b"VSSLREC1";

fn source_code(source: VisionSource) -> u8 {
    match source {
        VisionSource::FiraSim => 0,
        VisionSource::SslVision => 1,
    }
}

fn source_from_code(code: u8) -> Option<VisionSource> {
    match code {
        0 => Some(VisionSource::FiraSim),
        1 => Some(VisionSource::SslVision),
        _ => None,
    }
}

pub struct VisionRecorder {
    out: BufWriter<File>,
    t0: Instant,
    count: u64,
}

impl VisionRecorder {
    pub fn create<P: AsRef<Path>>(path: P, source: VisionSource) -> std::io::Result<Self> {
        if let Some(parent) = path.as_ref().parent()
            && !parent.as_os_str().is_empty()
        {
            std::fs::create_dir_all(parent)?;
        }
        let mut out = BufWriter::new(File::create(path)?);
        out.write_all(RECORD_MAGIC)?;
        out.write_all(&[source_code(source)])?;
        Ok(Self {
            out,
            t0: Instant::now(),
            count: 0,
        })
    }

    /// `VSSL_VISION_RECORD=<ruta>` → grabador abierto (o `None`).
    pub fn from_env(source: VisionSource) -> Option<Self> {
        let path = std::env::var(VISION_RECORD_ENV).ok()?;
        if path.trim().is_empty() {
            return None;
        }
        match Self::create(path.trim(), source) {
            Ok(r) => Some(r),
            Err(e) => {
                eprintln!("[Vision] ✗ no se pudo abrir {VISION_RECORD_ENV}={path}: {e}");
                None
            }
        }
    }

    /// Escribe un paquete crudo (tal como llegó, antes de cualquier proxy).
    pub fn write(&mut self, data: &[u8]) -> std::io::Result<()> {
        let t_us = self.t0.elapsed().as_micros() as u64;
        self.out.write_all(&t_us.to_le_bytes())?;
        self.out.write_all(&(data.len() as u32).to_le_bytes())?;
        self.out.write_all(data)?;
        self.count += 1;
        Ok(())
    }

    pub fn count(&self) -> u64 {
        self.count
    }

    pub fn flush(&mut self) -> std::io::Result<()> {
        self.out.flush()
    }
}

impl Drop for VisionRecorder {
    fn drop(&mut self) {
        let _ = self.out.flush();
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RecordedPacket {
    pub t_us: u64,
    pub data: Vec<u8>,
}

pub struct VisionRecording {
    pub source: VisionSource,
    pub packets: Vec<RecordedPacket>,
}

impl VisionRecording {
    pub fn read<P: AsRef<Path>>(path: P) -> Result<Self, String> {
        let mut bytes = Vec::new();
        File::open(&path)
            .and_then(|mut f| f.read_to_end(&mut bytes))
            .map_err(|e| format!("{}: {e}", path.as_ref().display()))?;
        Self::from_bytes(&bytes)
    }

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, String> {
        if bytes.len() < 9 || &bytes[..8] != RECORD_MAGIC {
            return Err("no es una grabación VSSLREC1".to_string());
        }
        let source = source_from_code(bytes[8]).ok_or("fuente desconocida en la cabecera")?;
        let mut packets = Vec::new();
        let mut i = 9;
        while i + 12 <= bytes.len() {
            let t_us = u64::from_le_bytes(bytes[i..i + 8].try_into().unwrap());
            let len = u32::from_le_bytes(bytes[i + 8..i + 12].try_into().unwrap()) as usize;
            i += 12;
            if i + len > bytes.len() {
                break; // paquete truncado al final (grabación cortada): se ignora
            }
            packets.push(RecordedPacket {
                t_us,
                data: bytes[i..i + len].to_vec(),
            });
            i += len;
        }
        Ok(Self { source, packets })
    }

    pub fn duration_s(&self) -> f64 {
        self.packets.last().map(|p| p.t_us as f64 / 1e6).unwrap_or(0.0)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Parser puro de detecciones
// ─────────────────────────────────────────────────────────────────────────────

/// Una detección en metros/radianes. `team == -1` es la pelota (`id == -1`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Detection {
    pub team: i32,
    pub id: i32,
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

impl Detection {
    pub fn is_ball(&self) -> bool {
        self.team < 0
    }
}

/// Parsea un paquete de visión según la fuente. Misma lógica que el receptor en
/// vivo (`vision.rs`): FIRA en metros; SSL-Vision en mm → m, descartando los
/// placeholders del fork vsss-vision-sysmic (confidence 0 y pixel (0,0)).
pub fn parse_detections(source: VisionSource, data: &[u8]) -> Vec<Detection> {
    let mut out = Vec::new();
    if matches!(source, VisionSource::FiraSim)
        && let Ok(env) = FiraEnvironment::parse_from_bytes(data)
        && let Some(frame) = env.frame.as_ref()
    {
        if let Some(b) = frame.ball.as_ref() {
            out.push(Detection {
                team: -1,
                id: -1,
                x: b.x,
                y: b.y,
                theta: 0.0,
            });
        }
        for (team, robots) in [(0, &frame.robots_blue), (1, &frame.robots_yellow)] {
            for r in robots {
                out.push(Detection {
                    team,
                    id: r.robot_id as i32,
                    x: r.x,
                    y: r.y,
                    theta: r.orientation,
                });
            }
        }
        return out;
    }
    if let Ok(pkt) = SSL_WrapperPacket::parse_from_bytes(data)
        && let Some(det) = pkt.detection.as_ref()
    {
        for b in det.balls.iter() {
            if b.confidence() <= 0.0 || (b.pixel_x() == 0.0 && b.pixel_y() == 0.0) {
                continue;
            }
            out.push(Detection {
                team: -1,
                id: -1,
                x: b.x() as f64 / 1000.0,
                y: b.y() as f64 / 1000.0,
                theta: 0.0,
            });
        }
        for (team, robots) in [(0, &det.robots_blue), (1, &det.robots_yellow)] {
            for r in robots {
                if !r.has_x() || !r.has_y() {
                    continue;
                }
                if r.confidence() <= 0.0 || (r.pixel_x() == 0.0 && r.pixel_y() == 0.0) {
                    continue;
                }
                out.push(Detection {
                    team,
                    id: r.robot_id() as i32,
                    x: r.x() as f64 / 1000.0,
                    y: r.y() as f64 / 1000.0,
                    theta: r.orientation() as f64,
                });
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protos::fira_common::{Ball, Frame, Robot};
    use crate::protos::ssl_vision_detection::{SSL_DetectionBall, SSL_DetectionFrame, SSL_DetectionRobot};

    #[test]
    fn gauss_has_unit_variance() {
        let mut rng = XorShift64::new(42);
        let n = 20_000;
        let (mut sum, mut sum2) = (0.0, 0.0);
        for _ in 0..n {
            let g = rng.gauss();
            sum += g;
            sum2 += g * g;
        }
        let mean = sum / n as f64;
        let var = sum2 / n as f64 - mean * mean;
        assert!(mean.abs() < 0.03, "media {mean}");
        assert!((var - 1.0).abs() < 0.05, "varianza {var}");
    }

    #[test]
    fn noise_proxy_uses_params_and_drops_frames() {
        let p = VisionParams {
            proxy_sigma_pos_m: 0.002,
            proxy_sigma_theta_rad: 0.03,
            proxy_latency_ms: 90.0,
            proxy_drop_prob: 0.5,
            ..VisionParams::default()
        };
        let mut n = NoiseProxy::from_params(&p, 7);
        assert_eq!(n.latency, Duration::from_millis(90));
        let dropped = (0..2000).filter(|_| n.drop_packet()).count();
        assert!((800..1200).contains(&dropped), "dropped={dropped}");
        let mut acc = 0.0;
        for _ in 0..2000 {
            let (x, _) = n.perturb_pos(1.0, 0.0);
            acc += (x - 1.0) * (x - 1.0);
        }
        let sigma = (acc / 2000.0).sqrt();
        assert!((sigma - 0.002).abs() < 0.0004, "σ medida {sigma}");
        let mut zero = NoiseProxy::from_params(&VisionParams { proxy_sigma_pos_m: 0.0, proxy_drop_prob: 0.0, ..VisionParams::default() }, 1);
        assert_eq!(zero.perturb_pos(0.3, -0.2), (0.3, -0.2));
        assert!(!zero.drop_packet());
    }

    #[test]
    fn recorder_round_trips_packets() {
        let path = std::env::temp_dir().join(format!("vsss_rec_{}.bin", std::process::id()));
        {
            let mut rec = VisionRecorder::create(&path, VisionSource::SslVision).unwrap();
            rec.write(b"hola").unwrap();
            rec.write(&[1, 2, 3]).unwrap();
            assert_eq!(rec.count(), 2);
        }
        let r = VisionRecording::read(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert_eq!(r.source, VisionSource::SslVision);
        assert_eq!(r.packets.len(), 2);
        assert_eq!(r.packets[0].data, b"hola");
        assert_eq!(r.packets[1].data, vec![1, 2, 3]);
        assert!(r.packets[1].t_us >= r.packets[0].t_us);
        assert!(VisionRecording::from_bytes(b"basura").is_err());
    }

    #[test]
    fn parses_fira_environment() {
        let mut env = FiraEnvironment::new();
        let mut frame = Frame::new();
        let mut ball = Ball::new();
        ball.x = 0.1;
        ball.y = -0.2;
        frame.ball = protobuf::MessageField::some(ball);
        let mut r = Robot::new();
        r.robot_id = 2;
        r.x = -0.5;
        r.y = 0.3;
        r.orientation = 1.0;
        frame.robots_yellow.push(r);
        env.frame = protobuf::MessageField::some(frame);
        let bytes = env.write_to_bytes().unwrap();
        let det = parse_detections(VisionSource::FiraSim, &bytes);
        assert_eq!(det.len(), 2);
        assert!(det[0].is_ball());
        assert_eq!(det[1], Detection { team: 1, id: 2, x: -0.5, y: 0.3, theta: 1.0 });
    }

    #[test]
    fn parses_ssl_wrapper_in_meters_and_skips_placeholders() {
        let mut pkt = SSL_WrapperPacket::new();
        let mut det = SSL_DetectionFrame::new();
        // Campos `required` del proto2 de SSL-Vision.
        det.set_frame_number(1);
        det.set_t_capture(0.0);
        det.set_t_sent(0.0);
        det.set_camera_id(0);
        let mut b = SSL_DetectionBall::new();
        b.set_confidence(0.9);
        b.set_pixel_x(10.0);
        b.set_pixel_y(10.0);
        b.set_x(250.0);
        b.set_y(-100.0);
        det.balls.push(b);
        let mut r = SSL_DetectionRobot::new();
        r.set_confidence(0.8);
        r.set_pixel_x(5.0);
        r.set_pixel_y(5.0);
        r.set_robot_id(1);
        r.set_x(-600.0);
        r.set_y(120.0);
        r.set_orientation(0.5);
        det.robots_blue.push(r);
        let mut placeholder = SSL_DetectionRobot::new();
        placeholder.set_confidence(0.0);
        placeholder.set_pixel_x(0.0);
        placeholder.set_pixel_y(0.0);
        placeholder.set_robot_id(2);
        placeholder.set_x(0.0);
        placeholder.set_y(0.0);
        det.robots_blue.push(placeholder);
        pkt.detection = protobuf::MessageField::some(det);
        let bytes = pkt.write_to_bytes().unwrap();
        let d = parse_detections(VisionSource::SslVision, &bytes);
        assert_eq!(d.len(), 2, "{d:?}");
        assert!((d[0].x - 0.25).abs() < 1e-9 && (d[0].y + 0.1).abs() < 1e-9);
        assert_eq!(d[1].team, 0);
        assert_eq!(d[1].id, 1);
        assert!((d[1].x + 0.6).abs() < 1e-9);
    }
}
