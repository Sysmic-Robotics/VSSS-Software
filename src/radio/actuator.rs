//! Capa de actuador real: el modelo medido del robot (sysid sobre (v, ω)) entre el frame
//! y el simulador.
//!
//! Recibe el mismo `(v mm/s, ω °/s)` entero que va a la base (`command_to_vw`) y devuelve
//! el `(v, ω)` que ejecutaría el robot real:
//! 1. retardo de actuación;
//! 2. lo que hace el firmware con la consigna: cinemática con el track, tope por rueda
//!    que escala las dos ruedas y zona muerta por rueda;
//! 3. ganancia en régimen de v y de ω (1 si el firmware está bien calibrado);
//! 4. primer orden en v y en ω con aceleración máxima (el PID de rueda, la inercia y el
//!    PI de guiñada juntos).
//!
//! El retardo conmuta con los pasos estáticos 2 y 3, y todos van antes
//! de la dinámica, como en el robot: el firmware topa y recorta la consigna, y las
//! ruedas la siguen con su retraso.
//!
//! Los parámetros salen de `config/real_calibration.json` (`tools/sysid_ajuste.py`), por
//! robot y por batería. Se activa con `VSSL_REAL_ACTUATOR=<id de visión>:<full|half>`:
//! en el transporte de FIRASim (descontando la dinámica propia de FIRASim, sección
//! `firasim` del archivo), en la planta de test y en `skill_test --transport plant`.
//! Sin la variable, nada cambia.

use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::OnceLock;

use serde::{Deserialize, Serialize};

pub const CALIBRATION_PATH: &str = "config/real_calibration.json";
/// Otra ruta para el archivo de calibración (ensayos y tests).
pub const CALIBRATION_ENV: &str = "VSSL_REAL_CALIBRATION";
/// `<id de visión>:<full|half>`: activa la capa con esa medición.
pub const ACTUATOR_ENV: &str = "VSSL_REAL_ACTUATOR";

/// Una medición de un robot con una batería (lo que escribe `tools/sysid_ajuste.py`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Measurement {
    /// Id de visión (parche de colores): la clave de `VSSL_REAL_ACTUATOR`.
    pub vision_id: u32,
    /// `MI_ROBOT_ID` compilado en el firmware del robot.
    pub mi_robot_id: u32,
    /// `full` (cargada) o `half` (media carga).
    pub battery: String,
    /// Voltaje medido con tester al empezar la sesión (V).
    pub battery_v: f64,
    /// Fecha de la sesión (AAAA-MM-DD).
    pub date: String,
    #[serde(default)]
    pub notes: String,
    /// Latencia de punta a punta: comando enviado → movimiento visto por el engine (ms).
    pub latency_ms: f64,
    /// Parte de `latency_ms` que es de la visión (ms), si se midió (F0). Sin medir, 0.
    #[serde(default)]
    pub vision_latency_ms: f64,
    pub tau_v_s: f64,
    pub tau_w_s: f64,
    pub max_accel_m_s2: f64,
    pub max_alpha_rad_s2: f64,
    pub deadzone_wheel_mm_s: f64,
    pub max_wheel_mm_s: f64,
    /// Track efectivo: el que explica el tope de ω y la zona muerta al girar (mm).
    pub wheel_track_mm: f64,
    /// Ganancia en régimen de v y de ω (medida / pedida, sin saturar). Lejos de 1 en v
    /// apunta al radio de rueda del firmware; en ω, a la escala del giroscopio.
    #[serde(default = "one")]
    pub gain_v: f64,
    #[serde(default = "one")]
    pub gain_w: f64,
    /// Error del ajuste (RMS, R², intervalos): informativo, no se valida.
    #[serde(default)]
    pub fit: serde_json::Value,
}

/// Dinámica propia de FIRASim, medida con los mismos perfiles sin la capa (design, D4).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SimDynamics {
    pub latency_ms: f64,
    pub tau_v_s: f64,
    pub tau_w_s: f64,
    pub date: String,
    #[serde(default)]
    pub notes: String,
    #[serde(default)]
    pub fit: serde_json::Value,
}

#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RealCalibration {
    #[serde(default)]
    pub firasim: Option<SimDynamics>,
    #[serde(default)]
    pub measurements: Vec<Measurement>,
}

fn one() -> f64 {
    1.0
}

fn check(who: &str, key: &str, value: f64, min: f64, max: f64) -> Result<(), String> {
    if value.is_finite() && value >= min && value <= max {
        Ok(())
    } else {
        Err(format!("{who}: {key} = {value} fuera de rango [{min}, {max}]"))
    }
}

impl Measurement {
    fn who(&self) -> String {
        format!("robot {} ({})", self.vision_id, self.battery)
    }

    fn validate(&self) -> Result<(), String> {
        let who = self.who();
        if self.battery != "full" && self.battery != "half" {
            return Err(format!("{who}: battery debe ser full o half"));
        }
        check(&who, "battery_v", self.battery_v, 1.0, 30.0)?;
        check(&who, "latency_ms", self.latency_ms, 0.0, 1000.0)?;
        check(&who, "vision_latency_ms", self.vision_latency_ms, 0.0, self.latency_ms)?;
        check(&who, "tau_v_s", self.tau_v_s, 0.0, 5.0)?;
        check(&who, "tau_w_s", self.tau_w_s, 0.0, 5.0)?;
        check(&who, "max_accel_m_s2", self.max_accel_m_s2, 0.01, 100.0)?;
        check(&who, "max_alpha_rad_s2", self.max_alpha_rad_s2, 0.01, 2000.0)?;
        check(&who, "deadzone_wheel_mm_s", self.deadzone_wheel_mm_s, 0.0, 500.0)?;
        check(&who, "max_wheel_mm_s", self.max_wheel_mm_s, 1.0, 5000.0)?;
        check(&who, "wheel_track_mm", self.wheel_track_mm, 10.0, 300.0)?;
        check(&who, "gain_v", self.gain_v, 0.5, 1.5)?;
        check(&who, "gain_w", self.gain_w, 0.5, 1.5)
    }
}

impl RealCalibration {
    pub fn from_json(text: &str) -> Result<Self, String> {
        let text = text.strip_prefix('\u{feff}').unwrap_or(text);
        let c: Self = serde_json::from_str(text).map_err(|e| e.to_string())?;
        c.validate()?;
        Ok(c)
    }

    fn validate(&self) -> Result<(), String> {
        if let Some(s) = &self.firasim {
            check("firasim", "latency_ms", s.latency_ms, 0.0, 1000.0)?;
            check("firasim", "tau_v_s", s.tau_v_s, 0.0, 5.0)?;
            check("firasim", "tau_w_s", s.tau_w_s, 0.0, 5.0)?;
        }
        for (k, m) in self.measurements.iter().enumerate() {
            m.validate()?;
            if self.measurements[..k]
                .iter()
                .any(|o| o.vision_id == m.vision_id && o.battery == m.battery)
            {
                return Err(format!("{}: medición repetida", m.who()));
            }
        }
        Ok(())
    }

    pub fn find(&self, vision_id: u32, battery: &str) -> Option<&Measurement> {
        self.measurements
            .iter()
            .find(|m| m.vision_id == vision_id && m.battery == battery)
    }
}

/// `<id de visión>:<full|half>`.
pub fn parse_selector(raw: &str) -> Result<(u32, String), String> {
    let (id, bat) = raw
        .trim()
        .split_once(':')
        .ok_or_else(|| format!("{ACTUATOR_ENV}='{raw}': se esperaba <id de visión>:<full|half>"))?;
    let id = id
        .trim()
        .parse()
        .map_err(|_| format!("{ACTUATOR_ENV}='{raw}': '{id}' no es un id de visión"))?;
    let bat = bat.trim();
    if bat != "full" && bat != "half" {
        return Err(format!("{ACTUATOR_ENV}='{raw}': la batería es full o half"));
    }
    Ok((id, bat.to_string()))
}

/// El modelo que aplica la capa, en unidades SI.
#[derive(Debug, Clone, PartialEq)]
pub struct ActuatorModel {
    pub delay_s: f64,
    pub tau_v_s: f64,
    pub tau_w_s: f64,
    pub max_accel: f64,
    pub max_alpha: f64,
    /// Zona muerta por rueda (m/s).
    pub deadzone: f64,
    /// Tope por rueda (m/s).
    pub max_wheel: f64,
    pub half_track: f64,
    pub gain_v: f64,
    pub gain_w: f64,
}

impl ActuatorModel {
    /// El robot medido completo: para una planta sin dinámica propia (la de test y la
    /// de `skill_test --transport plant`), que tampoco tiene latencia de visión.
    pub fn for_plant(m: &Measurement) -> Self {
        Self {
            delay_s: m.latency_ms / 1000.0,
            tau_v_s: m.tau_v_s,
            tau_w_s: m.tau_w_s,
            max_accel: m.max_accel_m_s2,
            max_alpha: m.max_alpha_rad_s2,
            deadzone: m.deadzone_wheel_mm_s / 1000.0,
            max_wheel: m.max_wheel_mm_s / 1000.0,
            half_track: m.wheel_track_mm / 2000.0,
            gain_v: m.gain_v,
            gain_w: m.gain_w,
        }
    }

    /// Para FIRASim: descuenta su dinámica propia (design, D4). Retardo de la capa =
    /// latencia medida − la de la visión − la de FIRASim; τ de la capa = τ medido − τ de
    /// FIRASim, sin bajar de 0. Sin la sección `firasim`, no descuenta nada.
    pub fn for_firasim(m: &Measurement, sim: Option<&SimDynamics>) -> Self {
        let mut model = Self::for_plant(m);
        let own = m.latency_ms - m.vision_latency_ms;
        let (sim_lat, sim_tv, sim_tw) =
            sim.map_or((0.0, 0.0, 0.0), |s| (s.latency_ms, s.tau_v_s, s.tau_w_s));
        model.delay_s = (own - sim_lat).max(0.0) / 1000.0;
        model.tau_v_s = (m.tau_v_s - sim_tv).max(0.0);
        model.tau_w_s = (m.tau_w_s - sim_tw).max(0.0);
        model
    }

    /// Lo que hace el firmware con la consigna: (v, ω) → ruedas con el track, tope por
    /// rueda que escala las dos y zona muerta por rueda; de vuelta a (v m/s, ω rad/s).
    pub fn firmware_setpoint(&self, v_mm_s: i16, w_deg_s: i16) -> (f64, f64) {
        let v = f64::from(v_mm_s) / 1000.0;
        let w = f64::from(w_deg_s).to_radians();
        let (mut l, mut r) = (v - w * self.half_track, v + w * self.half_track);
        let peak = l.abs().max(r.abs());
        if peak > self.max_wheel {
            l *= self.max_wheel / peak;
            r *= self.max_wheel / peak;
        }
        let dead = |x: f64| if x.abs() < self.deadzone { 0.0 } else { x };
        let (l, r) = (dead(l), dead(r));
        ((l + r) / 2.0, (r - l) / (2.0 * self.half_track))
    }
}

/// Estado de la capa para un robot.
#[derive(Debug, Clone, Default)]
pub struct ActuatorState {
    t: f64,
    /// Consignas del firmware pendientes por el retardo: (instante, v, ω).
    queue: VecDeque<(f64, f64, f64)>,
    target: (f64, f64),
    v: f64,
    w: f64,
}

/// Un paso de primer orden hacia `target` con la variación acotada a `max_rate · dt`.
fn first_order(x: f64, target: f64, tau: f64, max_rate: f64, dt: f64) -> f64 {
    let gain = if tau > 0.0 { 1.0 - (-dt / tau).exp() } else { 1.0 };
    let lim = max_rate * dt;
    x + ((target - x) * gain).clamp(-lim, lim)
}

impl ActuatorState {
    /// Avanza `dt` segundos con el frame `(v mm/s, ω °/s)` vigente y devuelve el
    /// `(v m/s, ω rad/s)` que ejecuta el robot.
    pub fn step(&mut self, m: &ActuatorModel, v_mm_s: i16, w_deg_s: i16, dt: f64) -> (f64, f64) {
        let dt = if dt.is_finite() { dt.max(0.0) } else { 0.0 };
        self.t += dt;
        let (sv, sw) = m.firmware_setpoint(v_mm_s, w_deg_s);
        self.queue.push_back((self.t, m.gain_v * sv, m.gain_w * sw));
        while let Some(&(t0, v, w)) = self.queue.front() {
            if t0 > self.t - m.delay_s + 1e-9 {
                break;
            }
            self.target = (v, w);
            self.queue.pop_front();
        }
        self.v = first_order(self.v, self.target.0, m.tau_v_s, m.max_accel, dt);
        self.w = first_order(self.w, self.target.1, m.tau_w_s, m.max_alpha, dt);
        (self.v, self.w)
    }
}

/// Ruta del archivo de calibración: `VSSL_REAL_CALIBRATION` o el de `config/`.
pub fn calibration_path() -> PathBuf {
    match std::env::var(CALIBRATION_ENV) {
        Ok(p) if !p.trim().is_empty() => PathBuf::from(p.trim()),
        _ => PathBuf::from(CALIBRATION_PATH),
    }
}

/// Lee `VSSL_REAL_ACTUATOR` y su medición. `Ok(None)` sin la variable; un selector o
/// un archivo inválido, o una medición que no está, es error (no se simula a medias
/// sin saberlo).
pub fn selected_measurement() -> Result<Option<(Measurement, Option<SimDynamics>)>, String> {
    let raw = match std::env::var(ACTUATOR_ENV) {
        Ok(r) if !r.trim().is_empty() => r,
        _ => return Ok(None),
    };
    let (id, bat) = parse_selector(&raw)?;
    let path = calibration_path();
    let text = std::fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    let cal = RealCalibration::from_json(&text).map_err(|e| format!("{}: {e}", path.display()))?;
    let m = cal
        .find(id, &bat)
        .ok_or_else(|| format!("{}: no hay medición del robot {id} con batería {bat}", path.display()))?
        .clone();
    Ok(Some((m, cal.firasim)))
}

/// El modelo para una planta sin dinámica propia, leído una vez por proceso.
/// Entra en pánico con el error si la variable está pero no se puede usar.
pub fn plant_model() -> Option<&'static ActuatorModel> {
    static MODEL: OnceLock<Option<ActuatorModel>> = OnceLock::new();
    MODEL
        .get_or_init(|| match selected_measurement() {
            Ok(sel) => sel.map(|(m, _)| ActuatorModel::for_plant(&m)),
            Err(e) => panic!("[actuator] {e}"),
        })
        .as_ref()
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 1.0 / 60.0;

    fn measurement() -> Measurement {
        Measurement {
            vision_id: 1,
            mi_robot_id: 2,
            battery: "full".into(),
            battery_v: 8.2,
            date: "2026-10-08".into(),
            notes: String::new(),
            latency_ms: 100.0,
            vision_latency_ms: 0.0,
            tau_v_s: 0.2,
            tau_w_s: 0.1,
            max_accel_m_s2: 100.0,
            max_alpha_rad_s2: 1000.0,
            deadzone_wheel_mm_s: 20.0,
            max_wheel_mm_s: 450.0,
            wheel_track_mm: 75.0,
            gain_v: 1.0,
            gain_w: 1.0,
            fit: serde_json::Value::Null,
        }
    }

    fn run(m: &ActuatorModel, v: i16, w: i16, ticks: usize) -> Vec<(f64, f64)> {
        let mut st = ActuatorState::default();
        (0..ticks).map(|_| st.step(m, v, w, DT)).collect()
    }

    #[test]
    fn wheel_cap_scales_both_wheels() {
        let m = ActuatorModel::for_plant(&measurement());
        let (v, w) = *run(&m, 600, 0, 300).last().unwrap();
        assert!((v - 0.45).abs() < 1e-6 && w.abs() < 1e-9, "v={v} w={w}");
        // Un arco que satura la rueda derecha conserva la curva (ω/v).
        let (sv, sw) = m.firmware_setpoint(400, 360);
        let r = sv + sw * m.half_track;
        assert!((r - 0.45).abs() < 1e-9, "rueda derecha al tope: {r}");
        assert!((sw / sv - 360f64.to_radians() / 0.4).abs() < 1e-9);
    }

    #[test]
    fn deadzone_zeroes_small_wheel_setpoints() {
        let m = ActuatorModel::for_plant(&measurement());
        assert!(run(&m, 15, 0, 120).iter().all(|&(v, w)| v == 0.0 && w == 0.0));
        // 90 °/s pide ±59 mm/s por rueda (track 75): gira.
        assert!(run(&m, 0, 90, 120).last().unwrap().1 > 1.5);
        // 25 °/s pide ±16 mm/s por rueda: no gira.
        assert!(run(&m, 0, 25, 120).iter().all(|&(_, w)| w == 0.0));
    }

    #[test]
    fn delay_then_first_order_reaches_63_percent_at_tau() {
        let m = ActuatorModel::for_plant(&measurement());
        let out = run(&m, 300, 0, 60);
        // Los primeros 100 ms (6 ticks), en cero.
        assert!(out[..6].iter().all(|&(v, _)| v == 0.0), "{:?}", &out[..7]);
        assert!(out[6].0 > 0.0);
        // A los 300 ms (tick 18) va por el 63 %, ± 1 tick.
        let frac = |k: usize| out[k - 1].0 / 0.3;
        assert!(frac(17) < 0.632 && frac(19) > 0.632, "{} {} {}", frac(17), frac(18), frac(19));
        assert!((frac(18) - (1.0 - (-1f64).exp())).abs() < 1e-6);
    }

    #[test]
    fn acceleration_limit_bounds_the_slope() {
        let mut meas = measurement();
        meas.max_accel_m_s2 = 1.0;
        meas.latency_ms = 0.0;
        let m = ActuatorModel::for_plant(&meas);
        let out = run(&m, 400, 0, 30);
        for w in out.windows(2) {
            assert!(w[1].0 - w[0].0 <= 1.0 * DT + 1e-12);
        }
        assert!((out[0].0 - DT).abs() < 1e-12);
    }

    #[test]
    fn gains_scale_the_regime_and_default_to_one() {
        let mut meas = measurement();
        meas.gain_v = 0.9;
        meas.gain_w = 1.1;
        let m = ActuatorModel::for_plant(&meas);
        let (v, w) = *run(&m, 300, 90, 300).last().unwrap();
        assert!((v - 0.27).abs() < 1e-6 && (w - 1.1 * 90f64.to_radians()).abs() < 1e-6, "{v} {w}");
        let text = file_with(&measurement()).replace(",\"gain_v\":1.0,\"gain_w\":1.0", "");
        assert!(!text.contains("gain_v"));
        assert_eq!(RealCalibration::from_json(&text).unwrap().measurements[0].gain_v, 1.0);
    }

    #[test]
    fn firasim_model_discounts_its_own_dynamics() {
        let mut meas = measurement();
        meas.vision_latency_ms = 20.0;
        let sim = SimDynamics {
            latency_ms: 30.0,
            tau_v_s: 0.05,
            tau_w_s: 0.2,
            date: "2026-10-08".into(),
            notes: String::new(),
            fit: serde_json::Value::Null,
        };
        let m = ActuatorModel::for_firasim(&meas, Some(&sim));
        assert!((m.delay_s - 0.05).abs() < 1e-12);
        assert!((m.tau_v_s - 0.15).abs() < 1e-12);
        assert_eq!(m.tau_w_s, 0.0);
        assert_eq!(ActuatorModel::for_firasim(&meas, None).delay_s, 0.08);
    }

    fn file_with(m: &Measurement) -> String {
        serde_json::to_string(&RealCalibration { firasim: None, measurements: vec![m.clone()] }).unwrap()
    }

    #[test]
    fn invalid_file_names_the_key_and_the_robot() {
        let mut m = measurement();
        m.tau_v_s = -0.1;
        let err = RealCalibration::from_json(&file_with(&m)).unwrap_err();
        assert!(err.contains("tau_v_s") && err.contains("robot 1"), "{err}");
        let mut m = measurement();
        m.battery = "empty".into();
        assert!(RealCalibration::from_json(&file_with(&m)).unwrap_err().contains("battery"));
        let mut m = measurement();
        m.max_wheel_mm_s = f64::NAN;
        assert!(serde_json::to_string(&m).is_ok_and(|s| RealCalibration::from_json(&s).is_err()));
        // Clave desconocida y medición repetida.
        let text = file_with(&measurement()).replace("\"tau_w_s\"", "\"tau_x_s\"");
        assert!(RealCalibration::from_json(&text).unwrap_err().contains("tau_x_s"));
        let two = RealCalibration { firasim: None, measurements: vec![measurement(), measurement()] };
        let err = RealCalibration::from_json(&serde_json::to_string(&two).unwrap()).unwrap_err();
        assert!(err.contains("repetida"), "{err}");
    }

    #[test]
    fn file_round_trips_and_finds_by_robot_and_battery() {
        let mut half = measurement();
        half.battery = "half".into();
        half.tau_v_s = 0.3;
        let cal = RealCalibration { firasim: None, measurements: vec![measurement(), half] };
        let back = RealCalibration::from_json(&serde_json::to_string_pretty(&cal).unwrap()).unwrap();
        assert_eq!(back, cal);
        assert_eq!(back.find(1, "half").unwrap().tau_v_s, 0.3);
        assert!(back.find(2, "full").is_none());
        assert_eq!(RealCalibration::from_json("{}").unwrap(), RealCalibration::default());
    }

    #[test]
    fn without_the_variable_the_layer_is_off() {
        if std::env::var(ACTUATOR_ENV).is_err() {
            assert!(selected_measurement().unwrap().is_none());
            assert!(plant_model().is_none());
        }
    }

    #[test]
    fn selector_parses_robot_and_battery() {
        assert_eq!(parse_selector("1:full").unwrap(), (1, "full".to_string()));
        assert_eq!(parse_selector(" 0 : half ").unwrap(), (0, "half".to_string()));
        assert!(parse_selector("1").is_err());
        assert!(parse_selector("x:full").is_err());
        assert!(parse_selector("1:media").is_err());
    }
}
