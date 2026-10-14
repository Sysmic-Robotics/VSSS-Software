//! Resumen de una corrida (D11 del design): lo arma el decisor al terminar o abortar, y el
//! binario le agrega la configuración de la corrida al escribirlo en JSON.

use super::formato::Repeticiones;
use super::vista::Fase;
use serde_json::{Map, Value, json};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Resultado {
    Completada,
    Abortada,
    /// El usuario la detuvo en bucle: no es una falla.
    Detenida,
}

impl Resultado {
    pub fn etiqueta(self) -> &'static str {
        match self {
            Resultado::Completada => "completada",
            Resultado::Abortada => "abortada",
            Resultado::Detenida => "detenida",
        }
    }
}

/// Un puesto en una vuelta completa.
#[derive(Debug, Clone, PartialEq)]
pub struct VueltaPuesto {
    pub puesto: String,
    pub robot: i32,
    /// Distancia media y máxima de la pose al tramo ideal mientras corrían los `ir` (m).
    /// `None` si el puesto no tiene `ir`.
    pub error_medio_m: Option<f64>,
    pub error_max_m: Option<f64>,
    /// Episodios de escape de la recuperación de atasco.
    pub escapes: u32,
    /// Episodios en que el guardia frenó o sacó al robot.
    pub guardia: u32,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Vuelta {
    /// Número de la vuelta (desde 1).
    pub n: u32,
    /// Duración sin las pausas (s).
    pub duracion_s: f64,
    pub puestos: Vec<VueltaPuesto>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ResumenCorrida {
    pub rutina: String,
    pub nombre: String,
    pub velocidad_pct: u32,
    pub repeticiones: Repeticiones,
    /// (puesto, robot), en el orden de los puestos.
    pub asignacion: Vec<(String, i32)>,
    pub resultado: Resultado,
    pub causa: Option<String>,
    /// Fase en la que se abortó.
    pub fase_aborto: Option<Fase>,
    /// De Preparar a Iniciar (s), si llegó a empezar.
    pub preparacion_s: Option<f64>,
    /// Solo las vueltas completas.
    pub vueltas: Vec<Vuelta>,
}

fn r4(x: f64) -> f64 {
    (x * 1e4).round() / 1e4
}

impl ResumenCorrida {
    /// Media, mínimo y máximo de la duración de las vueltas.
    pub fn duraciones(&self) -> (Option<f64>, Option<f64>, Option<f64>) {
        let d: Vec<f64> = self.vueltas.iter().map(|v| v.duracion_s).collect();
        if d.is_empty() {
            return (None, None, None);
        }
        let media = d.iter().sum::<f64>() / d.len() as f64;
        let min = d.iter().copied().fold(f64::INFINITY, f64::min);
        let max = d.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        (Some(media), Some(min), Some(max))
    }

    /// Escapes y guardia sumados por puesto, en el orden de la asignación.
    pub fn totales(&self) -> Vec<(String, u32, u32)> {
        self.asignacion
            .iter()
            .map(|(p, _)| {
                let de = |f: fn(&VueltaPuesto) -> u32| {
                    self.vueltas
                        .iter()
                        .flat_map(|v| v.puestos.iter())
                        .filter(|vp| vp.puesto == *p)
                        .map(f)
                        .sum::<u32>()
                };
                (p.clone(), de(|v| v.escapes), de(|v| v.guardia))
            })
            .collect()
    }

    /// Error medio de todas las vueltas, por puesto (m).
    fn error_medio(&self, puesto: &str) -> Option<f64> {
        let e: Vec<f64> = self
            .vueltas
            .iter()
            .flat_map(|v| v.puestos.iter())
            .filter(|vp| vp.puesto == puesto)
            .filter_map(|vp| vp.error_medio_m)
            .collect();
        (!e.is_empty()).then(|| e.iter().sum::<f64>() / e.len() as f64)
    }

    /// JSON del resumen; `corrida` es la configuración que agrega el binario.
    pub fn a_json(&self, corrida: Value) -> Value {
        let asignacion: Map<String, Value> = self.asignacion.iter().map(|(p, r)| (p.clone(), json!(r))).collect();
        let vueltas: Vec<Value> = self
            .vueltas
            .iter()
            .map(|v| {
                let puestos: Map<String, Value> = v
                    .puestos
                    .iter()
                    .map(|vp| {
                        (
                            vp.puesto.clone(),
                            json!({
                                "robot": vp.robot,
                                "error_medio_m": vp.error_medio_m.map(r4),
                                "error_max_m": vp.error_max_m.map(r4),
                                "escapes": vp.escapes,
                                "guardia": vp.guardia,
                            }),
                        )
                    })
                    .collect();
                json!({ "n": v.n, "duracion_s": r4(v.duracion_s), "puestos": puestos })
            })
            .collect();
        let (media, min, max) = self.duraciones();
        let totales = self.totales();
        let escapes: Map<String, Value> = totales.iter().map(|(p, e, _)| (p.clone(), json!(e))).collect();
        let guardia: Map<String, Value> = totales.iter().map(|(p, _, g)| (p.clone(), json!(g))).collect();
        json!({
            "rutina": self.rutina,
            "nombre": self.nombre,
            "velocidad_pct": self.velocidad_pct,
            "repeticiones": match self.repeticiones {
                Repeticiones::Veces(n) => json!(n),
                Repeticiones::Bucle => json!("bucle"),
            },
            "asignacion": asignacion,
            "resultado": self.resultado.etiqueta(),
            "causa": self.causa,
            "fase_aborto": self.fase_aborto.map(Fase::etiqueta),
            "preparacion_s": self.preparacion_s.map(r4),
            "vueltas_completas": self.vueltas.len(),
            "duracion_media_s": media.map(r4),
            "duracion_min_s": min.map(r4),
            "duracion_max_s": max.map(r4),
            "vueltas": vueltas,
            "escapes": escapes,
            "guardia": guardia,
            "pases": Value::Null,
            "corrida": corrida,
        })
    }

    /// Líneas para la terminal y la GUI.
    pub fn lineas(&self) -> Vec<String> {
        let mut l = vec![format!(
            "{} ({}): {}{}",
            self.nombre,
            self.rutina,
            self.resultado.etiqueta(),
            self.causa.as_ref().map(|c| format!(", {c}")).unwrap_or_default()
        )];
        let asign: Vec<String> = self.asignacion.iter().map(|(p, r)| format!("{p} ← {r}")).collect();
        l.push(format!(
            "{} al {} %, {} {}",
            asign.join(", "),
            self.velocidad_pct,
            self.vueltas.len(),
            if self.vueltas.len() == 1 { "vuelta completa" } else { "vueltas completas" }
        ));
        if let (Some(media), Some(min), Some(max)) = self.duraciones() {
            l.push(format!("vuelta: {media:.2} s (de {min:.2} a {max:.2})"));
        }
        for (p, e, g) in self.totales() {
            let err = self
                .error_medio(&p)
                .map(|x| format!("error medio {:.1} cm, ", x * 100.0))
                .unwrap_or_default();
            l.push(format!("{p}: {err}{e} escapes, guardia {g}"));
        }
        l
    }
}

/// Configuración de la corrida que el binario agrega al resumen: transporte, visión,
/// equipo, proxy de ruido, modo bidireccional, actuador real, tracker y hora.
pub fn corrida_json(transporte: &str, vision: &str, own_team: i32) -> Value {
    let hora = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    json!({
        "transporte": transporte,
        "vision": vision,
        "equipo": if own_team == 0 { "azul" } else { "amarillo" },
        "ruido_proxy": crate::vision_tools::vision_noise_enabled(),
        "bidireccional": crate::motion::MotionConfig::from_env().bidirectional,
        "actuador_real": std::env::var(crate::radio::actuator::ACTUATOR_ENV).ok().filter(|v| !v.trim().is_empty()),
        "tracker": crate::control_loop::tracker_on_from_env(),
        "hora_unix": hora,
    })
}

/// `<log sin .csv><sufijo>`, junto al CSV.
pub fn junto_al_log(log: &std::path::Path, sufijo: &str) -> std::path::PathBuf {
    let base = log.with_extension("");
    std::path::PathBuf::from(format!("{}{sufijo}", base.display()))
}

/// Escribe el resumen en JSON (con sangría).
pub fn escribir_resumen(ruta: &std::path::Path, resumen: &ResumenCorrida, corrida: Value) -> std::io::Result<()> {
    let texto = serde_json::to_string_pretty(&resumen.a_json(corrida)).map_err(std::io::Error::other)?;
    std::fs::write(ruta, texto + "\n")
}

/// Avisa una sola vez por corrida cuando la vista trae un resumen nuevo.
#[derive(Debug, Default)]
pub struct VigiaResumen {
    ultimo: Option<std::sync::Arc<ResumenCorrida>>,
}

impl VigiaResumen {
    /// El resumen de la vista, si es uno que todavía no se vio.
    pub fn nuevo(&mut self, vista: Option<&super::vista::RutinaVista>) -> Option<std::sync::Arc<ResumenCorrida>> {
        let r = vista?.resumen.clone()?;
        if self.ultimo.as_ref().is_some_and(|u| std::sync::Arc::ptr_eq(u, &r)) {
            return None;
        }
        self.ultimo = Some(r.clone());
        Some(r)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn resumen() -> ResumenCorrida {
        let vp = |escapes, err| VueltaPuesto {
            puesto: "A".to_string(),
            robot: 1,
            error_medio_m: Some(err),
            error_max_m: Some(err * 2.0),
            escapes,
            guardia: 0,
        };
        ResumenCorrida {
            rutina: "triangulo".to_string(),
            nombre: "Triángulo".to_string(),
            velocidad_pct: 60,
            repeticiones: Repeticiones::Veces(3),
            asignacion: vec![("A".to_string(), 1)],
            resultado: Resultado::Abortada,
            causa: Some("el robot 1 dejó de verse".to_string()),
            fase_aborto: Some(Fase::Ejecutando),
            preparacion_s: Some(3.2),
            vueltas: vec![
                Vuelta { n: 1, duracion_s: 8.0, puestos: vec![vp(1, 0.01)] },
                Vuelta { n: 2, duracion_s: 10.0, puestos: vec![vp(0, 0.03)] },
            ],
        }
    }

    #[test]
    fn durations_and_totals() {
        let r = resumen();
        assert_eq!(r.duraciones(), (Some(9.0), Some(8.0), Some(10.0)));
        assert_eq!(r.totales(), vec![("A".to_string(), 1, 0)]);
    }

    #[test]
    fn json_has_the_fields() {
        let j = resumen().a_json(json!({ "transporte": "firasim" }));
        assert_eq!(j["resultado"], "abortada");
        assert_eq!(j["fase_aborto"], "ejecutando");
        assert_eq!(j["vueltas_completas"], 2);
        assert_eq!(j["asignacion"]["A"], 1);
        assert_eq!(j["vueltas"][1]["puestos"]["A"]["error_max_m"], 0.06);
        assert_eq!(j["escapes"]["A"], 1);
        assert!(j["pases"].is_null());
        assert_eq!(j["corrida"]["transporte"], "firasim");
        assert_eq!(j["repeticiones"], 3);
    }

    #[test]
    fn summary_file_next_to_the_log() {
        let log = std::path::Path::new("/tmp/x/corrida.csv");
        assert_eq!(junto_al_log(log, ".resumen.json"), std::path::PathBuf::from("/tmp/x/corrida.resumen.json"));
        let dir = std::env::temp_dir().join(format!("vsss_resumen_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let ruta = dir.join("r.json");
        escribir_resumen(&ruta, &resumen(), corrida_json("firasim", "sim", 0)).unwrap();
        let j: Value = serde_json::from_str(&std::fs::read_to_string(&ruta).unwrap()).unwrap();
        let _ = std::fs::remove_dir_all(&dir);
        assert_eq!(j["rutina"], "triangulo");
        assert_eq!(j["corrida"]["transporte"], "firasim");
        assert_eq!(j["corrida"]["equipo"], "azul");
    }

    #[test]
    fn the_watch_reports_each_summary_once() {
        use crate::rutinas::vista::RutinaVista;
        let rutina = crate::rutinas::prueba::rutina(&crate::rutinas::prueba::un_puesto(r#"{ "paso": "esperar", "tiempo_s": 1 }"#));
        let mut vista = RutinaVista {
            rutina,
            fase: Fase::Ejecutando,
            frase: String::new(),
            velocidad_pct: 100,
            repeticion: 1,
            repeticiones: Repeticiones::Veces(1),
            robots: Vec::new(),
            resumen: None,
        };
        let mut v = VigiaResumen::default();
        assert!(v.nuevo(None).is_none());
        assert!(v.nuevo(Some(&vista)).is_none());
        vista.resumen = Some(std::sync::Arc::new(resumen()));
        assert!(v.nuevo(Some(&vista)).is_some());
        assert!(v.nuevo(Some(&vista)).is_none(), "una sola vez");
        vista.resumen = Some(std::sync::Arc::new(resumen()));
        assert!(v.nuevo(Some(&vista)).is_some(), "otra corrida");
    }

    #[test]
    fn human_lines() {
        let l = resumen().lineas();
        assert_eq!(l[0], "Triángulo (triangulo): abortada, el robot 1 dejó de verse");
        assert_eq!(l[1], "A ← 1 al 60 %, 2 vueltas completas");
        assert_eq!(l[2], "vuelta: 9.00 s (de 8.00 a 10.00)");
        assert_eq!(l[3], "A: error medio 2.0 cm, 1 escapes, guardia 0");
    }
}
