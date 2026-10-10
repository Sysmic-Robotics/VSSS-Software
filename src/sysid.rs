//! Perfiles de identificación del actuador sobre el contrato (v, ω).
//!
//! Secuencias deterministas de consignas `(v mm/s, ω °/s)` a 20 Hz, la cadencia con la
//! que la base reenvía el frame al robot. Las corre `skill_test --mode vw --profile`.
//! Todo perfil empieza y termina con 1 s en cero, separa los tramos con pausas de 1 s en
//! cero y es de ida y vuelta: cada tramo tiene su espejo, así que el robot vuelve cerca
//! de la marca y no sale de la cancha.

use std::f64::consts::PI;

/// Período de muestreo de los perfiles (s): 20 Hz, como la base.
pub const PROFILE_DT: f64 = 0.05;

/// Pausa entre tramos (s).
const PAUSE_S: f64 = 1.0;

/// Un perfil con nombre: una muestra `(v mm/s, ω °/s)` cada `PROFILE_DT`.
#[derive(Debug, Clone)]
pub struct Profile {
    pub name: &'static str,
    /// Qué mide el perfil (para `--list-profiles`).
    pub purpose: &'static str,
    pub samples: Vec<(i16, i16)>,
}

impl Profile {
    pub fn duration_s(&self) -> f64 {
        self.samples.len() as f64 * PROFILE_DT
    }

    /// Consigna vigente en el instante `t` (s) desde el inicio; cero fuera del perfil.
    pub fn at(&self, t: f64) -> (i16, i16) {
        if t.is_nan() || t < 0.0 {
            return (0, 0);
        }
        let k = (t / PROFILE_DT + 1e-9).floor() as usize;
        self.samples.get(k).copied().unwrap_or((0, 0))
    }

    /// Integra la cinemática ideal (el robot ejecuta la consigna exacta) desde
    /// (0, 0, 0°) a 1 kHz. Devuelve (distancia máxima al origen, distancia final) en m.
    pub fn ideal_excursion_m(&self) -> (f64, f64) {
        let sub = 50;
        let dt = PROFILE_DT / sub as f64;
        let (mut x, mut y, mut th, mut max) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
        for &(v, w) in &self.samples {
            let v = f64::from(v) / 1000.0;
            let w = f64::from(w).to_radians();
            for _ in 0..sub {
                x += v * th.cos() * dt;
                y += v * th.sin() * dt;
                th += w * dt;
                max = max.max(x.hypot(y));
            }
        }
        (max, x.hypot(y))
    }
}

/// Arma un perfil por tramos a `PROFILE_DT`.
struct Builder(Vec<(i16, i16)>);

impl Builder {
    fn new() -> Self {
        let mut b = Self(Vec::new());
        b.pause();
        b
    }

    fn samples(secs: f64) -> usize {
        (secs / PROFILE_DT).round() as usize
    }

    fn pause(&mut self) {
        self.hold(0.0, 0.0, PAUSE_S);
    }

    fn hold(&mut self, v: f64, w: f64, secs: f64) {
        for _ in 0..Self::samples(secs) {
            self.0.push((v.round() as i16, w.round() as i16));
        }
    }

    /// Tramo generado por `f(t)` → (v, ω), muestreado en el centro de cada período.
    fn shape(&mut self, secs: f64, f: impl Fn(f64) -> (f64, f64)) {
        for k in 0..Self::samples(secs) {
            let (v, w) = f((k as f64 + 0.5) * PROFILE_DT);
            self.0.push((v.round() as i16, w.round() as i16));
        }
    }

    /// El tramo `f`, una pausa, su espejo (v y ω con el signo cambiado) y otra pausa:
    /// con la cinemática ideal, el espejo deshace el tramo.
    fn there_and_back(&mut self, secs: f64, f: impl Fn(f64) -> (f64, f64)) {
        self.shape(secs, &f);
        self.pause();
        self.shape(secs, |t| {
            let (v, w) = f(t);
            (-v, -w)
        });
        self.pause();
    }

    fn build(self, name: &'static str, purpose: &'static str) -> Profile {
        Profile { name, purpose, samples: self.0 }
    }
}

/// Barrido sinusoidal lineal de `f0` a `f1` Hz en `secs`, de amplitud `amp`.
fn chirp(amp: f64, f0: f64, f1: f64, secs: f64, t: f64) -> f64 {
    let phase = 2.0 * PI * (f0 * t + (f1 - f0) * t * t / (2.0 * secs));
    amp * phase.sin()
}

/// Nombres de los perfiles, en el orden en que los corre la sesión de laboratorio.
pub const PROFILE_NAMES: [&str; 6] = ["v_steps", "v_ramp", "v_chirp", "w_steps", "w_chirp", "sat_arc"];

/// El perfil de nombre `name`.
pub fn profile(name: &str) -> Option<Profile> {
    let mut b = Builder::new();
    let p = match name {
        "v_steps" => {
            // Cada escalón recorre a lo sumo 0.36 m (2 s como máximo).
            for v in [100.0f64, 200.0, 300.0, 450.0, 600.0] {
                let secs = (0.36 / (v / 1000.0)).min(2.0);
                let secs = (secs / PROFILE_DT).floor() * PROFILE_DT;
                b.there_and_back(secs, |_| (v, 0.0));
            }
            b.build("v_steps", "escalones de v: τ_v, ganancia, aceleración y tope por rueda")
        }
        "v_ramp" => {
            // Lenta (0 → 60 → 0 mm/s en 8 s) para la zona muerta: pasa 2.7 s por debajo
            // de 20 mm/s, que con el ruido de la cámara hace falta. Rápida (0 → 600 en
            // 1.2 s) para la curva de saturación.
            b.there_and_back(8.0, |t| (60.0 * (1.0 - (t - 4.0).abs() / 4.0), 0.0));
            b.there_and_back(1.2, |t| (600.0 * t / 1.2, 0.0));
            b.build("v_ramp", "rampas de v: zona muerta y curva de saturación")
        }
        "v_chirp" => {
            b.there_and_back(10.0, |t| (chirp(250.0, 0.3, 3.0, 10.0, t), 0.0));
            b.build("v_chirp", "barrido de v de 0.3 a 3 Hz: latencia y respuesta en frecuencia")
        }
        "w_steps" => {
            for w in [90.0, 180.0, 360.0, 720.0] {
                b.there_and_back(1.0, |_| (0.0, w));
            }
            b.build("w_steps", "giro puro con escalones de ω: τ_ω y track efectivo")
        }
        "w_chirp" => {
            b.there_and_back(10.0, |t| (0.0, chirp(360.0, 0.3, 3.0, 10.0, t)));
            b.build("w_chirp", "barrido de ω de 0.3 a 3 Hz: latencia y respuesta en frecuencia")
        }
        "sat_arc" => {
            // Arcos en los que una rueda pasa del tope de 450 mm/s (con el track de 75 mm
            // del firmware): las dos ruedas se escalan por el mismo factor.
            for (v, w) in [(400.0, 360.0), (400.0, -360.0), (300.0, 540.0), (300.0, -540.0)] {
                b.there_and_back(1.0, |_| (v, w));
            }
            b.build("sat_arc", "arcos que saturan una rueda: escalado de las dos y track efectivo")
        }
        _ => return None,
    };
    Some(p)
}

/// Todos los perfiles, en el orden de `PROFILE_NAMES`.
pub fn all_profiles() -> Vec<Profile> {
    PROFILE_NAMES.iter().map(|n| profile(n).expect("perfil listado")).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const PAUSE_SAMPLES: usize = 20;

    #[test]
    fn every_listed_profile_exists_and_unknown_is_none() {
        assert_eq!(all_profiles().len(), PROFILE_NAMES.len());
        assert!(profile("nope").is_none());
        for p in all_profiles() {
            assert_eq!(profile(p.name).unwrap().samples, p.samples, "determinista: {}", p.name);
        }
    }

    #[test]
    fn profiles_stay_inside_the_field_and_come_back() {
        for p in all_profiles() {
            let (max, end) = p.ideal_excursion_m();
            assert!(max <= 0.5, "{}: se aleja {max:.3} m del origen", p.name);
            assert!(end < 0.05, "{}: termina a {end:.3} m", p.name);
        }
    }

    #[test]
    fn profiles_start_and_end_with_a_second_at_zero() {
        for p in all_profiles() {
            let n = p.samples.len();
            assert!(p.samples[..PAUSE_SAMPLES].iter().all(|&s| s == (0, 0)), "{}: inicio", p.name);
            assert!(p.samples[n - PAUSE_SAMPLES..].iter().all(|&s| s == (0, 0)), "{}: final", p.name);
        }
    }

    #[test]
    fn segments_are_separated_by_a_second_at_zero() {
        // Una racha de ceros más corta que la pausa solo puede ser un cruce por cero de
        // un barrido: a lo sumo 2 muestras, entre muestras de signo opuesto.
        let sign = |s: (i16, i16)| (s.0 + s.1).signum();
        for p in all_profiles() {
            let s = &p.samples;
            let mut k = 0;
            let mut segments = 0;
            while k < s.len() {
                if s[k] != (0, 0) {
                    k += 1;
                    continue;
                }
                let start = k;
                while k < s.len() && s[k] == (0, 0) {
                    k += 1;
                }
                let len = k - start;
                if len >= PAUSE_SAMPLES {
                    segments += 1;
                    continue;
                }
                assert!(len <= 2 && start > 0 && k < s.len(), "{}: racha corta en {start}", p.name);
                assert_eq!(sign(s[start - 1]), -sign(s[k]), "{}: racha corta en {start} sin cruce", p.name);
            }
            assert!(segments >= 3, "{}: {segments} pausas", p.name);
        }
    }

    #[test]
    fn at_holds_each_sample_for_a_period_and_is_zero_outside() {
        let p = profile("v_steps").unwrap();
        let k = p.samples.iter().position(|&s| s != (0, 0)).unwrap();
        let t = k as f64 * PROFILE_DT;
        assert_eq!(p.at(t), p.samples[k]);
        assert_eq!(p.at(t + 0.049), p.samples[k]);
        assert_eq!(p.at(t - 0.001), (0, 0));
        assert_eq!(p.at(-1.0), (0, 0));
        assert_eq!(p.at(p.duration_s() + 1.0), (0, 0));
    }

    #[test]
    fn step_and_saturation_profiles_ask_what_they_promise() {
        let max_v = |n: &str| profile(n).unwrap().samples.iter().map(|s| s.0).max().unwrap();
        let max_w = |n: &str| profile(n).unwrap().samples.iter().map(|s| s.1).max().unwrap();
        assert_eq!(max_v("v_steps"), 600);
        assert_eq!(max_w("w_steps"), 720);
        // sat_arc: con el track de 75 mm del firmware, una rueda pasa de 450 mm/s.
        let arc = profile("sat_arc").unwrap();
        assert!(arc.samples.iter().all(|&(v, w)| {
            let wheel = f64::from(v).abs() + f64::from(w).abs().to_radians() * 37.5;
            (v, w) == (0, 0) || wheel > 450.0
        }));
    }
}
