//! Parámetros de calibración y táctica FUERA del código (`config/team_params.json`).
//!
//! Todo lo que se ajusta con mediciones o con partidos de prueba (topes del robot
//! real, velocidades, distancias de las skills, umbrales del coach) vive aquí, para
//! que calibrar sea editar un archivo y no recompilar.
//!
//! Carga (`TeamParams::load`):
//! 1. `VSSL_PARAMS=<ruta>` si está definida (error si el archivo no existe o no parsea);
//! 2. si no, `config/team_params.json` relativo al directorio de trabajo, si existe;
//! 3. si no, los defaults de código (los mismos valores que trae el JSON del repo).
//!
//! El JSON puede ser PARCIAL: cualquier campo ausente toma su default. Un campo con
//! nombre desconocido es error (`deny_unknown_fields`), para que un typo en una
//! calibración no pase en silencio.
//!
//! Acceso global: `params()` (se fija una vez al arranque con `load_and_install`;
//! si nadie lo instaló, p. ej. en tests, devuelve los defaults).

use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::OnceLock;

/// Umbrales del coach heurístico (`HeuristicCoach`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct CoachParams {
    /// Robot que juega de arquero (id 0..2). Cambia solo si queda inactivo.
    pub keeper_id: i32,
    /// Velocidad lineal de referencia para estimar tiempo de llegada (m/s).
    pub v_ref: f32,
    /// Velocidad angular de referencia para el costo de giro (rad/s).
    pub omega_ref: f32,
    /// El striker cambia solo si el otro llega en menos de este factor del tiempo.
    pub role_switch_gain: f32,
    /// Decisiones mínimas entre cambios de rol (a 10 Hz: 5 = 0.5 s).
    pub role_min_hold: u32,
    /// Decisiones mínimas manteniendo una skill (a 10 Hz: 3 = 0.3 s).
    pub skill_min_hold: u32,
    /// Pelota más rápida que esto y viniendo hacia el robot → `Intercept` (m/s).
    pub intercept_min_ball_speed: f32,
    /// Pelota más lenta que esto dentro del área propia → el arquero despeja (m/s).
    pub gk_clear_max_ball_speed: f32,
    /// Rival a menos de esto del centro del arco rival cuenta como arquero (m).
    pub opp_keeper_radius: f32,
    /// |y| de la pelota desde el que se considera "pegada a la banda" (m).
    pub wall_band_y: f32,
    /// |x| de la pelota desde el que se considera "en el fondo" (m).
    pub wall_band_x: f32,
    /// Frecuencia de decisión del coach (Hz); convierte conteos en segundos.
    pub decision_hz: f32,
    /// Distancia robot–pelota desde la que el striker puede optar por `SpinKick` (m).
    pub spin_engage_radius: f32,
    /// Rival a menos de esto de la pelota cuenta como "encima de la pelota" (m).
    pub spin_when_opponent_within: f32,
    /// Segundos con la pelota dentro del área propia tras los que el arquero
    /// despeja sí o sí (el reglamento castiga retenerla más de 10 s).
    pub gk_force_clear_s: f32,
    /// Distancia del punto de marca al rival marcado, hacia nuestro arco (m).
    pub mark_distance: f32,
    /// Bono de score a la opción vigente del striker (histéresis: otra opción
    /// debe superarla por más que esto para reemplazarla).
    pub score_hysteresis: f32,
    /// Score base de `ApproachAligned` (fallback): las demás deben superarlo.
    pub score_approach_base: f32,
    /// Factor al score de `ShootPush` cuando un rival tapa la línea de tiro.
    pub score_shot_blocked_factor: f32,
    /// Distancia a nuestro arco (|x| desde el centro) desde la que empieza a
    /// puntuar `Clear` (m)…
    pub clear_zone_start_x: f32,
    /// …y desde la que puntúa al máximo (m).
    pub clear_zone_full_x: f32,
    /// Adaptación por contadores (éxito de tiro por carril, disputas) encendida.
    pub adapt_enabled: bool,
    /// Peso de cada observación nueva en las tasas adaptativas (0..1).
    pub adapt_alpha: f32,
}

impl Default for CoachParams {
    fn default() -> Self {
        Self {
            keeper_id: 2,
            v_ref: 1.0,
            omega_ref: 3.0,
            role_switch_gain: 0.75,
            role_min_hold: 5,
            skill_min_hold: 3,
            intercept_min_ball_speed: 0.30,
            gk_clear_max_ball_speed: 0.15,
            opp_keeper_radius: 0.30,
            wall_band_y: 0.52,
            wall_band_x: 0.62,
            decision_hz: 10.0,
            spin_engage_radius: 0.25,
            spin_when_opponent_within: 0.10,
            gk_force_clear_s: 6.0,
            mark_distance: 0.15,
            score_hysteresis: 0.15,
            score_approach_base: 0.30,
            score_shot_blocked_factor: 0.6,
            clear_zone_start_x: 0.20,
            clear_zone_full_x: 0.60,
            adapt_enabled: true,
            adapt_alpha: 0.3,
        }
    }
}

/// Distancias y tolerancias de las skills tácticas (`skills/tactical.rs`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct SkillParams {
    /// ApproachAligned: distancia del staging detrás de la pelota (m).
    pub approach_staging_offset: f32,
    /// ApproachAligned: tolerancia posicional para "en staging" (m).
    pub approach_pos_tol: f32,
    /// ApproachAligned: tolerancia angular de alineación (rad).
    pub approach_angle_tol: f64,
    /// ApproachAligned: radio desde el que ya mira a la pelota (m).
    pub approach_pre_align_radius: f32,
    /// ShootPush: cuánto más allá de la pelota apunta el movimiento (m).
    pub shoot_push_overshoot: f32,
    /// ShootPush: distancia robot–pelota desde la que deja de ser factible (m).
    pub shoot_lose_radius: f32,
    /// ShootPush: margen para "detrás de la pelota" sobre la línea de empuje (m).
    pub shoot_behind_tol: f32,
    /// ShootPush: velocidad de la pelota hacia el objetivo que cuenta como soltada (m/s).
    pub shoot_release_ball_speed: f32,
    /// Intercept: horizonte de predicción (s).
    pub intercept_horizon: f32,
    /// Intercept: bajo esta velocidad la pelota se considera quieta (m/s).
    pub intercept_min_ball_speed: f32,
    /// Intercept: constante de frenado exponencial de la pelota (1/s). MEDIR (M3).
    pub intercept_ball_decay_per_s: f32,
    /// Intercept: retardo de reacción sumado al tiempo de viaje (s).
    pub intercept_reaction_delay: f32,
    /// Intercept: distancia robot–pelota que cuenta como alcanzada (m).
    pub intercept_reach_radius: f32,
    /// BlockLine: distancia del punto de bloqueo al arco propio (m).
    pub block_distance: f32,
    /// BlockLine: |x| máximo del bloqueo (no entrar al área propia).
    pub block_max_abs_x: f32,
    /// Clear: staging corto detrás de la pelota (m).
    pub clear_staging_offset: f32,
    /// Clear: margen "detrás de la pelota" (más amplio que ShootPush) (m).
    pub clear_behind_tol: f32,
    /// Clear: radio de trabajo del empuje (m).
    pub clear_lose_radius: f32,
    /// Clear: velocidad de la pelota hacia el objetivo que cuenta como despejada (m/s).
    pub clear_release_ball_speed: f32,
    /// SpinKick: distancia centro del robot–pelota en el contacto (m). MEDIR (M3).
    pub spin_contact_radius: f32,
    /// SpinKick: velocidad angular del giro (rad/s).
    pub spin_omega: f64,
    /// SpinKick: tolerancia para "en el punto de contacto" (m).
    pub spin_pos_tol: f32,
    /// SpinKick: velocidad de la pelota hacia el objetivo que cuenta como lanzada (m/s).
    pub spin_release_ball_speed: f32,
    /// SpinKick: tiempo máximo girando antes de darse por terminada (s).
    pub spin_max_time_s: f32,
}

impl Default for SkillParams {
    fn default() -> Self {
        Self {
            approach_staging_offset: 0.14,
            approach_pos_tol: 0.05,
            approach_angle_tol: 0.15,
            approach_pre_align_radius: 0.25,
            shoot_push_overshoot: 0.25,
            shoot_lose_radius: 0.30,
            shoot_behind_tol: 0.03,
            shoot_release_ball_speed: 0.6,
            intercept_horizon: 1.5,
            intercept_min_ball_speed: 0.08,
            intercept_ball_decay_per_s: 0.3,
            intercept_reaction_delay: 0.10,
            intercept_reach_radius: 0.09,
            block_distance: 0.30,
            block_max_abs_x: 0.58,
            clear_staging_offset: 0.10,
            clear_behind_tol: 0.06,
            clear_lose_radius: 0.35,
            clear_release_ball_speed: 0.5,
            spin_contact_radius: 0.065,
            spin_omega: 20.0,
            spin_pos_tol: 0.03,
            spin_release_ball_speed: 0.4,
            spin_max_time_s: 1.0,
        }
    }
}

/// Navegación (`MotionConfig`). Las velocidades máximas se calibran con M2/M4.
///
/// `heading_gain`, `max_linear_accel` y `max_angular_speed` se calibraron en FIRASim con
/// el proxy de ruido de cámara (90 ms de latencia, ~1.2 m/s² de aceleración física): hay
/// que recalibrarlos con la sysid del robot real (latencia de la cámara y a_max con el
/// firmware (v, ω)).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct MotionParams {
    pub max_linear_speed: f64,
    pub min_linear_speed: f64,
    pub max_angular_speed: f64,
    /// Distancia al destino bajo la cual el robot se considera llegado (m). Debe ser
    /// menor que `skills.approach_pos_tol` (se valida al cargar).
    pub arrival_threshold: f32,
    pub brake_distance: f32,
    pub uvf_influence_radius: f32,
    pub uvf_k_rep: f32,
    /// Ganancia del seguimiento de heading hacia la dirección del UVF (rad/s por rad).
    pub heading_gain: f64,
    /// Límite de aceleración del avance comandado, sobre el comando anterior (m/s²).
    pub max_linear_accel: f64,
    /// Robot de dos caras (heading módulo 180°). `VSSL_BIDIRECTIONAL` lo sobreescribe.
    pub bidirectional: bool,
}

impl Default for MotionParams {
    fn default() -> Self {
        Self {
            max_linear_speed: 1.2,
            min_linear_speed: 0.06,
            max_angular_speed: 3.0,
            arrival_threshold: 0.04,
            brake_distance: 0.50,
            uvf_influence_radius: 0.20,
            uvf_k_rep: 1.5,
            heading_gain: 3.0,
            max_linear_accel: 1.0,
            bidirectional: false,
        }
    }
}

/// Robot REAL: topes del frame `V,W` que va a la base station.
///
/// La cinemática diferencial la hace el firmware (`WHEEL_TRACK_MM`), así que el PC
/// no guarda la geometría del robot real; la del simulador está en `SimRobotParams`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RobotParams {
    /// Tope de velocidad lineal (mm/s). Mismo clamp que la base
    /// (`MAX_V_MM_S`, `base_station_lineal_angulo.ino`).
    pub max_v_mm_s: i32,
    /// Tope de velocidad angular (grados/s). Mismo clamp que la base (`MAX_W_DEG_S`).
    pub max_w_deg_s: i32,
    /// Id de visión (parche de colores) → posición en el frame de la base
    /// (`MI_ROBOT_ID − 1` del firmware). Un id sin entrada usa su propio número.
    /// Solo lo usa la base station (`radio::base_station::radio_slot`); validado en
    /// `validate`.
    pub radio_slot_by_vision_id: BTreeMap<u32, u32>,
}

impl Default for RobotParams {
    fn default() -> Self {
        Self {
            max_v_mm_s: 1500,
            max_w_deg_s: 720,
            radio_slot_by_vision_id: BTreeMap::new(),
        }
    }
}

impl RobotParams {
    /// Valida `radio_slot_by_vision_id`: toda posición en `0..SLOT_COUNT` y dos ids
    /// de visión nunca en la misma posición, contando también los ids sin entrada
    /// (usan su propio número). Si dos comandos cayeran en la misma posición, el
    /// frame se quedaría con el último y el robot recibiría consignas alternadas.
    pub fn validate(&self) -> Result<(), String> {
        use crate::radio::base_station::SLOT_COUNT;
        let map = &self.radio_slot_by_vision_id;
        for (id, slot) in map {
            if *slot as usize >= SLOT_COUNT {
                return Err(format!(
                    "robot.radio_slot_by_vision_id: visión #{id} → pos {slot} fuera de rango (0..{})",
                    SLOT_COUNT - 1
                ));
            }
        }
        // Primero las entradas explícitas (el error más directo), luego los ids sin
        // entrada, que usan su propio número.
        let unmapped = (0..SLOT_COUNT as u32)
            .filter(|i| !map.contains_key(i))
            .map(|i| (i, i));
        let mut taken: BTreeMap<u32, u32> = BTreeMap::new(); // posición → id de visión
        for (id, slot) in map.iter().map(|(i, s)| (*i, *s)).chain(unmapped) {
            if let Some(prev) = taken.insert(slot, id) {
                let describe = |i: u32| match map.get(&i) {
                    Some(s) => format!("visión #{i} (→ pos {s})"),
                    None => format!("visión #{i} (sin entrada, usa pos {i})"),
                };
                return Err(format!(
                    "robot.radio_slot_by_vision_id: {} y {} caen en la misma posición de radio {slot}; \
                     agrega o corrige la entrada de visión #{id} hacia una posición libre",
                    describe(prev),
                    describe(id)
                ));
            }
        }
        Ok(())
    }
}

/// Robot de FIRASim (conversión (v, ω) → rad/s por rueda). Medido con
/// `measure_wheelbase.py`: r = 0.02000 m, L = 0.08499 m (constante en 3 velocidades).
/// Con el valor viejo L = 0.05 el robot giraba al 59 % de lo comandado.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct SimRobotParams {
    pub wheel_base_m: f64,
    pub wheel_radius_m: f64,
    /// Tope por rueda (rad/s): 60 · 0.02 = 1.2 m/s por rueda.
    pub max_wheel_rad_s: f64,
}

impl Default for SimRobotParams {
    fn default() -> Self {
        Self {
            wheel_base_m: 0.085,
            wheel_radius_m: 0.02,
            max_wheel_rad_s: 60.0,
        }
    }
}

/// Percepción: EKF del tracker (`tracker/ekf.rs`) y proxy de ruido para el
/// simulador (`vision_tools.rs`). Calibrar con la medición M1 (cámara real):
/// `r_*` = varianza medida del ruido de la cámara; `proxy_*` = los mismos números
/// para reproducirlo en FIRASim. Defaults: literatura (σ_pos ≈ 1.85 mm, σ_θ ≈
/// 0.031 rad, latencia 90 ms) para el proxy; para el EKF, valores conservadores.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct VisionParams {
    /// Varianza de medición de posición (m²). 1e-4 → σ ≈ 1 cm.
    pub r_pos: f64,
    /// Varianza de medición de orientación (rad²). 2.5e-3 → σ ≈ 2.9°.
    pub r_theta: f64,
    /// Ruido de proceso: posición (m² por paso).
    pub q_pos: f64,
    /// Ruido de proceso: sin/cos de la orientación.
    pub q_angle: f64,
    /// Ruido de proceso: velocidad lineal ((m/s)² por paso).
    pub q_vel: f64,
    /// Ruido de proceso: velocidad angular ((rad/s)² por paso).
    pub q_omega: f64,
    /// Gating de innovación (χ² con 3 g.l.; 16.27 ≈ p 0.999).
    pub gating_chi2: f64,
    /// Velocidad lineal máxima plausible para el gate físico (m/s).
    pub max_lin_speed: f64,
    /// Velocidad angular máxima plausible (rad/s); también acota ω del estado.
    pub max_omega: f64,
    /// Margen sobre el máximo físico en el gate.
    pub gate_margin: f64,
    /// Proxy (sim): σ de posición inyectada (m).
    pub proxy_sigma_pos_m: f64,
    /// Proxy (sim): σ de orientación inyectada (rad).
    pub proxy_sigma_theta_rad: f64,
    /// Proxy (sim): latencia de visión (ms).
    pub proxy_latency_ms: f64,
    /// Proxy (sim): probabilidad de perder un frame [0, 1].
    pub proxy_drop_prob: f64,
    /// Offset (grados) que se suma a la orientación de cada robot SOLO con la
    /// visión real (`VSSL_VISION_SOURCE=sslvision`), antes del EKF y de la GUI.
    /// vsss-vision-sysmic entrega el heading girado 180°. FIRASim no se toca.
    pub real_theta_offset_deg: f64,
}

impl Default for VisionParams {
    fn default() -> Self {
        Self {
            r_pos: 1e-4,
            r_theta: 2.5e-3,
            q_pos: 1e-7,
            q_angle: 1e-4,
            q_vel: 1e-4,
            q_omega: 1e-2,
            gating_chi2: 16.27,
            max_lin_speed: 4.0,
            max_omega: 45.0,
            gate_margin: 2.0,
            proxy_sigma_pos_m: 0.00185,
            proxy_sigma_theta_rad: 0.031,
            proxy_latency_ms: 90.0,
            proxy_drop_prob: 0.02,
            real_theta_offset_deg: 0.0,
        }
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct TeamParams {
    pub coach: CoachParams,
    pub skills: SkillParams,
    pub motion: MotionParams,
    pub robot: RobotParams,
    pub sim: SimRobotParams,
    pub vision: VisionParams,
}

static PARAMS: OnceLock<TeamParams> = OnceLock::new();

/// Parámetros vigentes del proceso. Defaults si nadie llamó a `load_and_install`.
pub fn params() -> &'static TeamParams {
    PARAMS.get_or_init(TeamParams::default)
}

impl TeamParams {
    pub const DEFAULT_PATH: &'static str = "config/team_params.json";
    pub const ENV_VAR: &'static str = "VSSL_PARAMS";

    pub fn from_json(text: &str) -> Result<Self, String> {
        let p: Self = serde_json::from_str(text).map_err(|e| e.to_string())?;
        p.robot.validate()?;
        p.validate_arrival()?;
        Ok(p)
    }

    /// Motion se detiene a `arrival_threshold` del destino: si eso queda por encima de
    /// la tolerancia de `ApproachAligned`, la skill nunca alcanza su staging.
    fn validate_arrival(&self) -> Result<(), String> {
        if self.motion.arrival_threshold < self.skills.approach_pos_tol {
            Ok(())
        } else {
            Err(format!(
                "motion.arrival_threshold ({}) debe ser menor que skills.approach_pos_tol ({}): \
                 si no, ApproachAligned nunca alcanza su staging",
                self.motion.arrival_threshold, self.skills.approach_pos_tol
            ))
        }
    }

    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Self, String> {
        let text = std::fs::read_to_string(&path)
            .map_err(|e| format!("{}: {e}", path.as_ref().display()))?;
        // Editores de Windows (Bloc de notas, PowerShell) suelen guardar con BOM;
        // serde_json no lo acepta y el error resultante es críptico.
        let text = text.strip_prefix('\u{feff}').unwrap_or(&text);
        Self::from_json(text).map_err(|e| format!("{}: {e}", path.as_ref().display()))
    }

    pub fn to_json_pretty(&self) -> String {
        serde_json::to_string_pretty(self).expect("TeamParams serializable")
    }

    /// Ruta a cargar: `VSSL_PARAMS` si está definida; si no, el archivo por defecto
    /// solo si existe.
    pub fn resolve_path() -> Option<PathBuf> {
        if let Ok(p) = std::env::var(Self::ENV_VAR)
            && !p.trim().is_empty()
        {
            return Some(PathBuf::from(p.trim()));
        }
        let default = PathBuf::from(Self::DEFAULT_PATH);
        default.exists().then_some(default)
    }

    /// Carga según `resolve_path`. Devuelve los parámetros y una descripción de la
    /// fuente para el log. Un archivo indicado pero inválido es error duro (no se
    /// juega con parámetros a medias sin saberlo).
    pub fn load() -> Result<(Self, String), String> {
        match Self::resolve_path() {
            Some(path) => {
                let p = Self::from_file(&path)?;
                Ok((p, format!("archivo {}", path.display())))
            }
            None => Ok((Self::default(), "defaults de código (sin JSON)".to_string())),
        }
    }

    /// Carga e instala como parámetros globales del proceso. Si ya había unos
    /// instalados, no los reemplaza (devuelve error descriptivo).
    pub fn load_and_install() -> Result<String, String> {
        let (p, source) = Self::load()?;
        PARAMS
            .set(p)
            .map_err(|_| "los parámetros ya estaban instalados".to_string())?;
        Ok(source)
    }

    /// Para los binarios: carga e instala, imprime la fuente y, si el archivo
    /// indicado es inválido, termina el proceso (no se juega con parámetros a
    /// medias sin saberlo).
    pub fn install_or_exit(tag: &str) {
        match Self::load_and_install() {
            Ok(src) => eprintln!("[{tag}] parámetros: {src}"),
            Err(e) => {
                eprintln!("[{tag}] ✗ parámetros inválidos: {e}");
                std::process::exit(2);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_json_gives_defaults() {
        let p = TeamParams::from_json("{}").unwrap();
        assert_eq!(p, TeamParams::default());
    }

    #[test]
    fn partial_json_overrides_only_given_fields() {
        let p = TeamParams::from_json(
            r#"{ "coach": { "keeper_id": 0 }, "robot": { "max_v_mm_s": 400 } }"#,
        )
        .unwrap();
        assert_eq!(p.coach.keeper_id, 0);
        assert_eq!(p.coach.role_min_hold, CoachParams::default().role_min_hold);
        assert_eq!(p.robot.max_v_mm_s, 400);
        assert_eq!(p.robot.max_w_deg_s, RobotParams::default().max_w_deg_s);
        assert_eq!(p.sim, SimRobotParams::default());
    }

    #[test]
    fn robot_defaults_match_base_station_clamps() {
        let r = RobotParams::default();
        assert_eq!(r.max_v_mm_s, 1500);
        assert_eq!(r.max_w_deg_s, 720);
    }

    #[test]
    fn robot_wheel_base_is_rejected() {
        // La geometría del robot real vive en el firmware: un JSON viejo con
        // `robot.wheel_base_m` debe fallar nombrando el campo, no ignorarse.
        let err = TeamParams::from_json(r#"{ "robot": { "wheel_base_m": 0.07 } }"#).unwrap_err();
        assert!(err.contains("wheel_base_m"), "{err}");
    }

    #[test]
    fn unknown_field_is_an_error() {
        let err = TeamParams::from_json(r#"{ "robot": { "wheelbase_m": 0.07 } }"#).unwrap_err();
        assert!(err.contains("wheelbase_m"), "{err}");
    }

    #[test]
    fn default_json_round_trips() {
        let text = TeamParams::default().to_json_pretty();
        let back = TeamParams::from_json(&text).unwrap();
        assert_eq!(back, TeamParams::default());
    }

    #[test]
    fn file_with_utf8_bom_is_accepted_and_unknown_field_is_named() {
        let dir = std::env::temp_dir();
        let ok_path = dir.join(format!("vsss_params_bom_{}.json", std::process::id()));
        std::fs::write(&ok_path, "\u{feff}{ \"coach\": { \"keeper_id\": 1 } }").unwrap();
        let p = TeamParams::from_file(&ok_path).unwrap();
        let _ = std::fs::remove_file(&ok_path);
        assert_eq!(p.coach.keeper_id, 1);

        let bad_path = dir.join(format!("vsss_params_bad_{}.json", std::process::id()));
        std::fs::write(&bad_path, "\u{feff}{ \"robot\": { \"wheelbase_m\": 0.07 } }").unwrap();
        let err = TeamParams::from_file(&bad_path).unwrap_err();
        let _ = std::fs::remove_file(&bad_path);
        assert!(err.contains("wheelbase_m"), "{err}");
    }

    #[test]
    fn repo_config_file_parses() {
        // El archivo del repo debe ser siempre cargable (sin campos desconocidos).
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join(TeamParams::DEFAULT_PATH);
        let p = TeamParams::from_file(&path).unwrap_or_else(|e| panic!("{e}"));
        assert!(p.robot.max_v_mm_s > 0);
        assert!(p.robot.max_w_deg_s > 0);
        assert_eq!(p.sim.wheel_base_m, 0.085);
        // La visión real entrega el heading girado 180° (vsss-vision-sysmic).
        assert_eq!(p.vision.real_theta_offset_deg, 180.0);
    }

    #[test]
    fn arrival_threshold_must_be_below_approach_tolerance() {
        // Con 0.06 ≥ 0.05, motion se detiene antes de que ApproachAligned dé por
        // alcanzado su staging: la carga lo rechaza nombrando ambas claves.
        let err = TeamParams::from_json(
            r#"{ "motion": { "arrival_threshold": 0.06 }, "skills": { "approach_pos_tol": 0.05 } }"#,
        )
        .unwrap_err();
        assert!(err.contains("arrival_threshold") && err.contains("approach_pos_tol"), "{err}");
        // Los defaults y el JSON del repo cumplen la relación.
        let d = TeamParams::default();
        assert!(d.motion.arrival_threshold < d.skills.approach_pos_tol);
    }

    #[test]
    fn coupling_floor_is_no_longer_accepted() {
        // Se eliminó: un JSON local que todavía la tenga falla al cargar.
        let err = TeamParams::from_json(r#"{ "motion": { "coupling_floor": 0.22 } }"#).unwrap_err();
        assert!(err.contains("coupling_floor"), "{err}");
    }

    #[test]
    fn vision_without_offset_defaults_to_zero() {
        let p = TeamParams::from_json(r#"{ "vision": { "r_pos": 0.0002 } }"#).unwrap();
        assert_eq!(p.vision.real_theta_offset_deg, 0.0);
        assert!(TeamParams::default().robot.radio_slot_by_vision_id.is_empty());
    }

    fn slot_map_err(map: &str) -> String {
        TeamParams::from_json(&format!(
            r#"{{ "robot": {{ "radio_slot_by_vision_id": {map} }} }}"#
        ))
        .unwrap_err()
    }

    #[test]
    fn slot_map_swap_is_valid() {
        let p = TeamParams::from_json(
            r#"{ "robot": { "radio_slot_by_vision_id": { "0": 1, "1": 0 } } }"#,
        )
        .unwrap();
        assert_eq!(p.robot.radio_slot_by_vision_id.get(&1), Some(&0));
        assert_eq!(p.robot.radio_slot_by_vision_id.get(&0), Some(&1));
    }

    #[test]
    fn slot_map_explicit_duplicate_is_rejected() {
        let err = slot_map_err(r#"{ "1": 0, "2": 0 }"#);
        assert!(err.contains("radio_slot_by_vision_id"), "{err}");
        assert!(err.contains("visión #1") && err.contains("visión #2"), "{err}");
        assert!(err.contains("posición de radio 0"), "{err}");
    }

    #[test]
    fn slot_map_clash_with_unmapped_id_is_rejected() {
        // {"1": 0} sin entrada para #0: el #0 usa su propia posición (0) y choca.
        let err = slot_map_err(r#"{ "1": 0 }"#);
        assert!(err.contains("visión #0 (sin entrada, usa pos 0)"), "{err}");
        assert!(err.contains("visión #1 (→ pos 0)"), "{err}");
        assert!(err.contains("entrada de visión #0"), "{err}");
    }

    #[test]
    fn slot_map_out_of_range_is_rejected() {
        let err = slot_map_err(r#"{ "1": 5 }"#);
        assert!(err.contains("radio_slot_by_vision_id"), "{err}");
        assert!(err.contains("pos 5 fuera de rango (0..4)"), "{err}");
    }

    #[test]
    fn slot_map_negative_or_non_numeric_is_rejected() {
        assert!(TeamParams::from_json(r#"{ "robot": { "radio_slot_by_vision_id": { "1": -1 } } }"#).is_err());
        assert!(TeamParams::from_json(r#"{ "robot": { "radio_slot_by_vision_id": { "a": 0 } } }"#).is_err());
    }
}
