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
    /// Fracción de `score_approach_base` que conserva `ApproachAligned` cuando el
    /// empuje ya es factible: casi nada (un tiro tapado vale más que no tirar), pero
    /// no cero, para que el compromiso mínimo siga evitando el ping-pong.
    pub score_approach_when_feasible: f32,
    /// Segundos de empuje con la pelota pegada y quieta antes de sacarla con el giro.
    pub push_stall_s: f32,
    /// Tope del giro (forzado o elegido) si la pelota tampoco se suelta así (s).
    pub stall_kick_max_s: f32,
    /// Segundos sin giro tras un giro que no soltó la pelota: que pruebe otra opción.
    pub spin_ban_s: f32,
    /// Distancia (m) a la que se aparta un jugador de campo de una pelota que juega
    /// el arquero, y mínima del punto de apoyo a la pelota.
    pub standoff_from_ball: f32,
    /// Desplazamiento lateral del apoyo cuando el equipo ataca: se ofrece en el carril
    /// contrario al striker, 0.25 m adelante de la pelota, como receptor de segunda pelota (m).
    pub pass_receiver_offset: f32,
    /// Presión: con un rival encima de la pelota, el striker va a disputarla solo si
    /// está a menos de esto de ella (m); más lejos espera sobre la línea pelota–arco
    /// propio a esta distancia de la pelota. Grande = presiona siempre.
    pub press_distance: f32,
    /// Arquero activo: sale a despejar una pelota conducida por un rival dentro del
    /// área propia ampliada en este margen (m). Más lejos deja el arco vacío.
    pub gk_engage_margin: f32,
    /// Arquero activo: rival a menos de esto de la pelota cuenta como conduciéndola (m).
    pub gk_engage_opp_dist: f32,
    /// Arquero activo: solo sale a una pelota más lenta que esto (m/s).
    pub gk_engage_max_ball_speed: f32,
    /// Arquero: una pelota que avanza hacia la boca de nuestro arco a más de esto (m/s) se
    /// bloquea desde la línea, no se despeja. Grande (p. ej. 9.9) = desactivado.
    pub gk_block_incoming_speed: f32,
    /// Arquero: cuida el arco con `RailKeeper` (riel cerca del arco, orientado a lo largo de
    /// él, con anticipación) en vez de `GoalKeep`.
    pub gk_rail: bool,
    /// Roles: el atacante es el que llega antes a donde ESTARÁ la pelota (modelo de
    /// `InterceptSkill`), no a donde está.
    pub role_by_intercept: bool,
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
            mark_distance: 0.30,
            score_hysteresis: 0.15,
            score_approach_base: 0.30,
            score_shot_blocked_factor: 0.6,
            clear_zone_start_x: 0.20,
            clear_zone_full_x: 0.60,
            adapt_enabled: true,
            adapt_alpha: 0.3,
            score_approach_when_feasible: 0.2,
            push_stall_s: 1.5,
            stall_kick_max_s: 3.0,
            spin_ban_s: 2.0,
            standoff_from_ball: 0.30,
            pass_receiver_offset: 0.35,
            press_distance: 0.60,
            gk_engage_margin: 0.05,
            gk_engage_opp_dist: 0.20,
            gk_engage_max_ball_speed: 0.40,
            gk_block_incoming_speed: 0.10,
            gk_rail: true,
            role_by_intercept: false,
        }
    }
}

/// Rangos admisibles de los parámetros de ESTILO de juego (`config/style_ranges.json`):
/// los que mueve el optimizador (`tools/annealing.py`). Solo claves de `coach`; los
/// parámetros físicos y las tolerancias de las skills se calibran con el robot y no
/// entran. Cada rango es `[mín, máx]`; el default del código debe caer adentro.
pub type StyleRanges = BTreeMap<String, [f64; 2]>;

impl CoachParams {
    pub const STYLE_RANGES_PATH: &'static str = "config/style_ranges.json";

    /// Valores vigentes como mapa `clave → número`, para cruzarlos con los rangos.
    pub fn as_numbers(&self) -> BTreeMap<String, f64> {
        let value = serde_json::to_value(self).expect("CoachParams serializa");
        value
            .as_object()
            .expect("CoachParams es un objeto")
            .iter()
            .filter_map(|(k, v)| v.as_f64().map(|x| (k.clone(), x)))
            .collect()
    }

    /// Lee los rangos y verifica que cada clave exista en `CoachParams` y que el valor
    /// vigente esté dentro del rango. Devuelve el primer problema como texto.
    pub fn check_style_ranges(&self, text: &str) -> Result<StyleRanges, String> {
        let ranges: StyleRanges = serde_json::from_str(text).map_err(|e| e.to_string())?;
        let values = self.as_numbers();
        for (key, [lo, hi]) in &ranges {
            let Some(v) = values.get(key) else {
                return Err(format!("style_ranges: `{key}` no es un parámetro numérico de coach"));
            };
            if lo > hi {
                return Err(format!("style_ranges: `{key}` tiene mín {lo} > máx {hi}"));
            }
            // Tolerancia por la conversión f32 → f64 (0.6_f32 = 0.6000000238…).
            let eps = 1e-6 * hi.abs().max(1.0);
            if *v < lo - eps || *v > hi + eps {
                return Err(format!("style_ranges: `{key}` = {v} fuera de [{lo}, {hi}]"));
            }
        }
        Ok(ranges)
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
    /// ShootPush y Clear: distancia lateral máxima del robot a la recta de empuje para
    /// empujar (m). Menor que medio frente + radio de la pelota (0.059: más allá la pelota
    /// ya no está frente a la cara) y ≥ `approach_pos_tol` (lo que ApproachAligned da por
    /// terminado es factible para ShootPush).
    pub shoot_lateral_tol: f32,
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
    /// BlockLine: distancia del punto de bloqueo al arco propio (m). Si el punto cae en el
    /// área propia, la skill lo corre hacia afuera por la línea de bloqueo.
    pub block_distance: f32,
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
    /// SpinKick: a esta distancia del punto de contacto deja de navegar a él y cierra
    /// sobre la pelota hasta tenerla al alcance (gira solo tocándola) (m). Mayor que
    /// `motion.arrival_threshold`: si no, motion lo detiene antes y queda quieto.
    pub spin_pos_tol: f32,
    /// SpinKick: velocidad de la pelota hacia el objetivo que cuenta como lanzada (m/s).
    pub spin_release_ball_speed: f32,
    /// SpinKick: tiempo máximo girando antes de darse por terminada (s).
    pub spin_max_time_s: f32,
    /// RailKeeper: distancia del riel (centro del robot) a la línea de gol (m).
    pub keeper_depth: f32,
    /// RailKeeper: velocidad lateral pedida por metro de error en y (1/s).
    pub keeper_gain: f64,
    /// RailKeeper: horizonte para tratar como tiro una pelota que viene al arco (s).
    pub keeper_horizon_s: f32,
    /// RailKeeper: adelanto con que extrapola la pelota (latencia de visión y reacción, s).
    pub keeper_lead_s: f32,
    /// RailKeeper: desvío del riel desde el que vuelve con la navegación normal (m).
    pub keeper_rail_tol: f32,
    /// RailKeeper: distancia rival–pelota desde la que anticipa el empuje del rival (m).
    pub keeper_contact_dist: f32,
    /// RailKeeper: cuánto más allá de la pelota apunta al barrerla de la boca del arco (m).
    pub keeper_sweep: f32,
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
            shoot_lateral_tol: 0.05,
            shoot_release_ball_speed: 0.6,
            intercept_horizon: 1.5,
            intercept_min_ball_speed: 0.08,
            intercept_ball_decay_per_s: 0.3,
            intercept_reaction_delay: 0.10,
            intercept_reach_radius: 0.09,
            block_distance: 0.30,
            clear_staging_offset: 0.10,
            clear_behind_tol: 0.06,
            clear_lose_radius: 0.35,
            clear_release_ball_speed: 0.5,
            spin_contact_radius: 0.065,
            spin_omega: 20.0,
            spin_pos_tol: 0.05,
            spin_release_ball_speed: 0.4,
            spin_max_time_s: 1.0,
            keeper_depth: 0.05,
            keeper_gain: 8.0,
            keeper_horizon_s: 1.5,
            keeper_lead_s: 0.10,
            keeper_rail_tol: 0.06,
            keeper_contact_dist: 0.12,
            keeper_sweep: 0.10,
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
    /// Corte por pérdida de visión (s): si ningún robot propio tuvo un dato en este
    /// tiempo, el lazo detiene al equipo (`TickMode::SinVision`). 0.4 s son ~25 cuadros:
    /// una pérdida breve no corta. Pendiente de revisar con la medición de la cámara
    /// real (frames perdidos y duración de los huecos).
    pub blind_stop_s: f64,
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
            blind_stop_s: 0.4,
        }
    }
}

/// Rutinas (`crate::rutinas`): validación de los archivos, tolerancias y tiempos por
/// defecto de los pasos, quietud y velocidad de la corrida. Los tiempos están medidos al
/// 100 % de velocidad: el decisor divide los de movimiento por la escala de la corrida.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RutinasParams {
    /// Distancia mínima de todo destino a las paredes y a los triángulos de las esquinas (m).
    pub zona_segura_m: f32,
    /// Solo validación del archivo: distancia mínima entre los inicios de los puestos (y,
    /// en la fase 2, en cada barrera) (m). No aborta nada en vivo.
    pub distancia_minima_m: f32,
    /// Aborto en vivo por casi choque: dos robots propios a menos de esto entre centros (m).
    pub distancia_choque_m: f32,
    /// Tolerancia de posición por defecto de los pasos (m). ≥ `motion.arrival_threshold`.
    pub tolerancia_m: f32,
    /// Tolerancia de orientación por defecto de los pasos (grados).
    pub tolerancia_deg: f64,
    /// Tiempo máximo por defecto de un paso de movimiento, al 100 % (s).
    pub tiempo_max_s: f64,
    /// Tiempo máximo de la preparación, al 100 % (s).
    pub preparacion_max_s: f64,
    /// Ventana de quietud (s).
    pub quieto_s: f64,
    /// Diferencia máxima entre las posiciones medias de las dos mitades de la ventana (m).
    pub quieto_m: f32,
    /// Diferencia máxima entre las orientaciones medias de las dos mitades (grados).
    pub quieto_deg: f64,
    /// Porcentaje de velocidad por defecto con la base station.
    pub velocidad_real_pct: u32,
    /// Porcentaje de velocidad por defecto en simulación.
    pub velocidad_sim_pct: u32,
    /// Tope de giro de `girar` (rad/s): el del firmware, 450 mm/s por rueda sobre una vía
    /// de 75 mm. Se recalibra con la sysid.
    pub giro_max_rad_s: f64,
    /// `empujar`: distancia máxima que el pasador avanza empujando antes de soltar (m).
    /// Sin el corte, si ShootPush no suelta la pelota, la conduce hasta el receptor.
    pub pase_empuje_max_m: f32,
    /// `recibir`: el receptor solo va por la pelota cuando se detuvo a esta distancia o
    /// menos de su punto (m); mientras rueda, la espera de frente.
    pub recepcion_radio_m: f32,
    /// Con pelota: sin progreso (pase logrado, paso sin pelota terminado o vuelta cerrada)
    /// durante este tiempo, la rutina se aborta (s). Las pausas reales no cuentan.
    pub sin_progreso_s: f64,
    /// Distancia a la pelota de los robots que no van por ella (rescate, despegue,
    /// reposición) y la del perseguidor (m).
    pub resguardo_m: f32,
    /// Despegue: el mayor ángulo hacia adentro, desde la dirección a lo largo de la pared,
    /// que se prueba para empujar la pelota pegada (grados).
    pub despegue_max_deg: f32,
    /// Rescate: el rescatador parte cuando la pelota suelta va más lenta que esto (m/s)...
    pub rescate_vel_m_s: f32,
    /// ... o cuando lleva este tiempo suelta, aunque siga rodando (s).
    pub rescate_espera_s: f64,
}

impl Default for RutinasParams {
    fn default() -> Self {
        Self {
            zona_segura_m: 0.10,
            distancia_minima_m: 0.20,
            distancia_choque_m: 0.10,
            tolerancia_m: 0.05,
            tolerancia_deg: 10.0,
            tiempo_max_s: 8.0,
            preparacion_max_s: 20.0,
            quieto_s: 0.5,
            quieto_m: 0.005,
            quieto_deg: 3.0,
            velocidad_real_pct: 60,
            velocidad_sim_pct: 100,
            giro_max_rad_s: 12.0,
            pase_empuje_max_m: 0.15,
            recepcion_radio_m: 0.25,
            sin_progreso_s: 30.0,
            resguardo_m: 0.30,
            despegue_max_deg: 45.0,
            rescate_vel_m_s: 0.15,
            rescate_espera_s: 2.0,
        }
    }
}

/// Control manual de la GUI (`gui-manual-control`): topes de velocidad por transporte y
/// el giro corto de los jugadores. Las velocidades son a la vez el valor de arranque y el
/// tope de la GUI; las aceleraciones de la rampa son constantes de la GUI.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ManualParams {
    /// Velocidad lineal máxima con la base station (m/s): 0.30 + 3·0.0375 = 0.41 m/s por
    /// rueda, bajo el tope de 450 mm/s del firmware.
    pub velocidad_real_m_s: f64,
    /// Velocidad lineal máxima en simulación (m/s).
    pub velocidad_sim_m_s: f64,
    /// Velocidad angular máxima del giro con A/D o las flechas (rad/s).
    pub giro_rad_s: f64,
    /// Giro corto (E o Enter): velocidad angular (rad/s), la del firmware en giro puro.
    pub spin_rad_s: f64,
    /// Giro corto: duración del pulso (s).
    pub spin_s: f64,
}

impl Default for ManualParams {
    fn default() -> Self {
        Self {
            velocidad_real_m_s: 0.30,
            velocidad_sim_m_s: 0.50,
            giro_rad_s: 3.0,
            spin_rad_s: 12.0,
            spin_s: 0.3,
        }
    }
}

impl ManualParams {
    /// Rangos: velocidades en (0, 1.5] m/s; giros en (0, 12.5] rad/s, bajo el tope del
    /// frame de 720 °/s; el pulso en (0, 1] s.
    pub fn validate(&self) -> Result<(), String> {
        let rangos = [
            ("velocidad_real_m_s", self.velocidad_real_m_s, 1.5, "m/s"),
            ("velocidad_sim_m_s", self.velocidad_sim_m_s, 1.5, "m/s"),
            ("giro_rad_s", self.giro_rad_s, 12.5, "rad/s"),
            ("spin_rad_s", self.spin_rad_s, 12.5, "rad/s"),
            ("spin_s", self.spin_s, 1.0, "s"),
        ];
        for (nombre, v, max, unidad) in rangos {
            if !(v > 0.0 && v <= max) {
                return Err(format!(
                    "manual.{nombre} ({v}) debe ser mayor que cero y a lo sumo {max} {unidad}"
                ));
            }
        }
        Ok(())
    }

    /// Tope (y arranque) de la velocidad lineal según el transporte.
    pub fn velocidad_m_s(&self, base_station: bool) -> f64 {
        if base_station { self.velocidad_real_m_s } else { self.velocidad_sim_m_s }
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
    pub rutinas: RutinasParams,
    pub manual: ManualParams,
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
        p.validate_tolerances()?;
        p.validate_rutinas()?;
        p.manual.validate()?;
        if p.vision.blind_stop_s <= 0.0 {
            return Err(format!(
                "vision.blind_stop_s ({}) debe ser mayor que cero: es el tiempo sin ver robots \
                 propios tras el cual el lazo detiene al equipo",
                p.vision.blind_stop_s
            ));
        }
        Ok(p)
    }

    /// Tolerancias cruzadas entre motion y skills:
    /// - motion se detiene a `arrival_threshold` del destino: si eso queda por encima de
    ///   la tolerancia de `ApproachAligned` o de `SpinKick`, la skill nunca llega;
    /// - lo que `ApproachAligned` da por terminado debe ser factible para `ShootPush`.
    fn validate_tolerances(&self) -> Result<(), String> {
        let (m, s) = (&self.motion, &self.skills);
        if m.arrival_threshold >= s.approach_pos_tol {
            return Err(format!(
                "motion.arrival_threshold ({}) debe ser menor que skills.approach_pos_tol ({}): \
                 si no, ApproachAligned nunca alcanza su staging",
                m.arrival_threshold, s.approach_pos_tol
            ));
        }
        if m.arrival_threshold >= s.spin_pos_tol {
            return Err(format!(
                "motion.arrival_threshold ({}) debe ser menor que skills.spin_pos_tol ({}): \
                 si no, SpinKick puede quedar detenido sin empezar a girar",
                m.arrival_threshold, s.spin_pos_tol
            ));
        }
        if s.shoot_lateral_tol < s.approach_pos_tol {
            return Err(format!(
                "skills.shoot_lateral_tol ({}) debe ser mayor o igual que skills.approach_pos_tol ({}): \
                 si no, un staging que ApproachAligned da por terminado no es factible para ShootPush",
                s.shoot_lateral_tol, s.approach_pos_tol
            ));
        }
        Ok(())
    }

    /// Rutinas: la tolerancia por defecto no pide más precisión que motion, el casi choque
    /// queda por debajo de la distancia del archivo, y los porcentajes en 20–100.
    fn validate_rutinas(&self) -> Result<(), String> {
        let r = &self.rutinas;
        if r.tolerancia_m < self.motion.arrival_threshold {
            return Err(format!(
                "rutinas.tolerancia_m ({}) debe ser mayor o igual que motion.arrival_threshold ({}): \
                 motion se detiene a esa distancia y el paso nunca terminaría",
                r.tolerancia_m, self.motion.arrival_threshold
            ));
        }
        let choque_ok = r.distancia_choque_m > 0.0 && r.distancia_choque_m <= r.distancia_minima_m;
        if !choque_ok {
            return Err(format!(
                "rutinas.distancia_choque_m ({}) debe ser mayor que cero y menor o igual que \
                 rutinas.distancia_minima_m ({})",
                r.distancia_choque_m, r.distancia_minima_m
            ));
        }
        for (nombre, pct) in [
            ("velocidad_real_pct", r.velocidad_real_pct),
            ("velocidad_sim_pct", r.velocidad_sim_pct),
        ] {
            if !(20..=100).contains(&pct) || !pct.is_multiple_of(10) {
                return Err(format!(
                    "rutinas.{nombre} ({pct}) debe ir de 20 a 100 en pasos de 10"
                ));
            }
        }
        let positivos = [
            ("zona_segura_m", r.zona_segura_m as f64),
            ("tolerancia_deg", r.tolerancia_deg),
            ("tiempo_max_s", r.tiempo_max_s),
            ("preparacion_max_s", r.preparacion_max_s),
            ("quieto_s", r.quieto_s),
            ("quieto_m", r.quieto_m as f64),
            ("quieto_deg", r.quieto_deg),
            ("giro_max_rad_s", r.giro_max_rad_s),
        ];
        for (nombre, v) in positivos {
            if v.is_nan() || v <= 0.0 {
                return Err(format!("rutinas.{nombre} ({v}) debe ser mayor que cero"));
            }
        }
        if !(r.pase_empuje_max_m > 0.0 && r.pase_empuje_max_m <= 0.60) {
            return Err(format!(
                "rutinas.pase_empuje_max_m ({}) debe ser mayor que cero y a lo sumo 0.60 m",
                r.pase_empuje_max_m
            ));
        }
        if !(r.recepcion_radio_m > 0.0 && r.recepcion_radio_m <= 0.50) {
            return Err(format!(
                "rutinas.recepcion_radio_m ({}) debe ser mayor que cero y a lo sumo 0.50 m",
                r.recepcion_radio_m
            ));
        }
        if !(5.0..=600.0).contains(&r.sin_progreso_s) {
            return Err(format!("rutinas.sin_progreso_s ({}) debe estar entre 5 y 600 s", r.sin_progreso_s));
        }
        if !(r.resguardo_m >= r.distancia_minima_m && r.resguardo_m <= 0.60) {
            return Err(format!(
                "rutinas.resguardo_m ({}) debe estar entre distancia_minima_m ({}) y 0.60 m",
                r.resguardo_m, r.distancia_minima_m
            ));
        }
        if !(r.despegue_max_deg > 0.0 && r.despegue_max_deg <= 80.0) {
            return Err(format!(
                "rutinas.despegue_max_deg ({}) debe ser mayor que cero y a lo sumo 80°",
                r.despegue_max_deg
            ));
        }
        if !(r.rescate_vel_m_s > 0.0 && r.rescate_vel_m_s <= 0.60) {
            return Err(format!(
                "rutinas.rescate_vel_m_s ({}) debe ser mayor que cero y a lo sumo 0.60 m/s",
                r.rescate_vel_m_s
            ));
        }
        if !(0.0..=10.0).contains(&r.rescate_espera_s) {
            return Err(format!("rutinas.rescate_espera_s ({}) debe estar entre 0 y 10 s", r.rescate_espera_s));
        }
        Ok(())
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
    fn style_ranges_file_matches_coach_defaults() {
        // Cada clave de config/style_ranges.json es un parámetro numérico del coach y
        // el default del código cae dentro de su rango (los 20 de estilo del TDP, con el
        // receptor de pase en lugar de la profundidad del apoyo, que dejó de usarse).
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join(CoachParams::STYLE_RANGES_PATH);
        let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{e}"));
        let ranges = CoachParams::default().check_style_ranges(&text).unwrap_or_else(|e| panic!("{e}"));
        assert_eq!(ranges.len(), 20, "la lista blanca de estilo tiene 20 parámetros");
        assert!(ranges.contains_key("press_distance") && ranges.contains_key("pass_receiver_offset"));
        // Y también lo cumple el JSON del repo.
        let repo = TeamParams::from_file(&Path::new(env!("CARGO_MANIFEST_DIR")).join(TeamParams::DEFAULT_PATH)).unwrap();
        repo.coach.check_style_ranges(&text).unwrap_or_else(|e| panic!("{e}"));
    }

    #[test]
    fn style_ranges_reject_unknown_keys_and_values_outside() {
        let p = CoachParams::default();
        let err = p.check_style_ranges(r#"{ "no_existe": [0.0, 1.0] }"#).unwrap_err();
        assert!(err.contains("no_existe"), "{err}");
        let err = p.check_style_ranges(r#"{ "score_hysteresis": [0.5, 0.9] }"#).unwrap_err();
        assert!(err.contains("score_hysteresis") && err.contains("fuera"), "{err}");
        assert!(p.check_style_ranges(r#"{ "role_min_hold": [0, 10] }"#).is_ok());
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
    fn blind_stop_defaults_to_0_4_s_and_must_be_positive() {
        assert_eq!(TeamParams::default().vision.blind_stop_s, 0.4);
        let p = TeamParams::from_json(r#"{ "vision": { "blind_stop_s": 0.3 } }"#).unwrap();
        assert_eq!(p.vision.blind_stop_s, 0.3);
        for malo in ["0", "-0.4"] {
            let err = TeamParams::from_json(&format!(r#"{{ "vision": {{ "blind_stop_s": {malo} }} }}"#))
                .unwrap_err();
            assert!(err.contains("vision.blind_stop_s"), "{err}");
        }
    }

    #[test]
    fn coupling_floor_is_no_longer_accepted() {
        // Se eliminó: un JSON local que todavía la tenga falla al cargar.
        let err = TeamParams::from_json(r#"{ "motion": { "coupling_floor": 0.22 } }"#).unwrap_err();
        assert!(err.contains("coupling_floor"), "{err}");
    }

    #[test]
    fn block_max_abs_x_is_no_longer_accepted() {
        // BlockLine deriva el límite del área del `ZoneGuard`: la clave vieja no carga.
        let err = TeamParams::from_json(r#"{ "skills": { "block_max_abs_x": 0.58 } }"#).unwrap_err();
        assert!(err.contains("block_max_abs_x"), "{err}");
    }

    #[test]
    fn spin_tolerance_must_exceed_arrival_threshold() {
        // Con spin_pos_tol ≤ arrival_threshold, motion detiene al robot antes del contacto.
        let err = TeamParams::from_json(
            r#"{ "motion": { "arrival_threshold": 0.04 }, "skills": { "spin_pos_tol": 0.04 } }"#,
        )
        .unwrap_err();
        assert!(err.contains("arrival_threshold") && err.contains("spin_pos_tol"), "{err}");
        let d = TeamParams::default();
        assert!(d.motion.arrival_threshold < d.skills.spin_pos_tol);
    }

    #[test]
    fn lateral_push_tolerance_covers_the_approach_tolerance() {
        let err = TeamParams::from_json(
            r#"{ "skills": { "approach_pos_tol": 0.05, "shoot_lateral_tol": 0.04 } }"#,
        )
        .unwrap_err();
        assert!(err.contains("shoot_lateral_tol") && err.contains("approach_pos_tol"), "{err}");
        let d = TeamParams::default();
        assert!(d.skills.shoot_lateral_tol >= d.skills.approach_pos_tol);
    }

    #[test]
    fn rutinas_defaults_and_repo_file() {
        let d = RutinasParams::default();
        assert_eq!(d.distancia_minima_m, 0.20);
        assert_eq!(d.distancia_choque_m, 0.10);
        assert_eq!((d.velocidad_real_pct, d.velocidad_sim_pct), (60, 100));
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join(TeamParams::DEFAULT_PATH);
        let p = TeamParams::from_file(&path).unwrap_or_else(|e| panic!("{e}"));
        assert_eq!(p.rutinas, d, "el JSON del repo trae los valores por defecto");
    }

    #[test]
    fn manual_defaults_repo_file_and_missing_section() {
        let d = ManualParams::default();
        assert_eq!((d.velocidad_real_m_s, d.velocidad_sim_m_s), (0.30, 0.50));
        assert_eq!((d.giro_rad_s, d.spin_rad_s, d.spin_s), (3.0, 12.0, 0.3));
        assert_eq!((d.velocidad_m_s(true), d.velocidad_m_s(false)), (0.30, 0.50));
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join(TeamParams::DEFAULT_PATH);
        let p = TeamParams::from_file(&path).unwrap_or_else(|e| panic!("{e}"));
        assert_eq!(p.manual, d, "el JSON del repo trae los valores por defecto");
        let sin = TeamParams::from_json(r#"{ "rutinas": { "tolerancia_m": 0.05 } }"#).unwrap();
        assert_eq!(sin.manual, d, "sin la sección, los valores por defecto");
    }

    #[test]
    fn manual_ranges_name_their_key() {
        for (json, clave) in [
            (r#"{ "manual": { "velocidad_real_m_s": 0.0 } }"#, "manual.velocidad_real_m_s"),
            (r#"{ "manual": { "velocidad_sim_m_s": 1.6 } }"#, "manual.velocidad_sim_m_s"),
            (r#"{ "manual": { "giro_rad_s": -1.0 } }"#, "manual.giro_rad_s"),
            (r#"{ "manual": { "spin_rad_s": 20.0 } }"#, "manual.spin_rad_s"),
            (r#"{ "manual": { "spin_s": 0.0 } }"#, "manual.spin_s"),
            (r#"{ "manual": { "spin_s": 1.5 } }"#, "manual.spin_s"),
        ] {
            let err = TeamParams::from_json(json).unwrap_err();
            assert!(err.contains(clave) && err.contains("a lo sumo"), "{json}: {err}");
        }
        let bordes = r#"{ "manual": { "velocidad_real_m_s": 1.5, "spin_rad_s": 12.5, "spin_s": 1.0 } }"#;
        assert!(TeamParams::from_json(bordes).is_ok());
        assert!(TeamParams::from_json(r#"{ "manual": { "marco": "mundo" } }"#).is_err());
    }

    #[test]
    fn rutinas_tolerance_must_cover_arrival_threshold() {
        let err = TeamParams::from_json(
            r#"{ "motion": { "arrival_threshold": 0.04 }, "rutinas": { "tolerancia_m": 0.03 } }"#,
        )
        .unwrap_err();
        assert!(err.contains("rutinas.tolerancia_m") && err.contains("arrival_threshold"), "{err}");
        // Igual a arrival_threshold vale.
        assert!(TeamParams::from_json(r#"{ "rutinas": { "tolerancia_m": 0.04 } }"#).is_ok());
    }

    #[test]
    fn rutinas_pass_params_have_defaults_and_bounds() {
        let p = TeamParams::from_json(r#"{ "rutinas": { "tolerancia_m": 0.05 } }"#).unwrap();
        assert_eq!((p.rutinas.pase_empuje_max_m, p.rutinas.recepcion_radio_m), (0.15, 0.25));
        let r = &p.rutinas;
        assert_eq!((r.sin_progreso_s, r.resguardo_m, r.despegue_max_deg), (30.0, 0.30, 45.0));
        assert_eq!((r.rescate_vel_m_s, r.rescate_espera_s), (0.15, 2.0));
        for (malo, campo) in [
            (r#"{ "rutinas": { "recepcion_radio_m": 0.0 } }"#, "recepcion_radio_m"),
            (r#"{ "rutinas": { "recepcion_radio_m": 0.6 } }"#, "recepcion_radio_m"),
            (r#"{ "rutinas": { "pase_empuje_max_m": 0.0 } }"#, "pase_empuje_max_m"),
            (r#"{ "rutinas": { "pase_empuje_max_m": 0.7 } }"#, "pase_empuje_max_m"),
            (r#"{ "rutinas": { "sin_progreso_s": 4.0 } }"#, "sin_progreso_s"),
            (r#"{ "rutinas": { "resguardo_m": 0.15 } }"#, "resguardo_m"),
            (r#"{ "rutinas": { "resguardo_m": 0.7 } }"#, "resguardo_m"),
            (r#"{ "rutinas": { "despegue_max_deg": 0.0 } }"#, "despegue_max_deg"),
            (r#"{ "rutinas": { "despegue_max_deg": 85.0 } }"#, "despegue_max_deg"),
            (r#"{ "rutinas": { "rescate_vel_m_s": 0.0 } }"#, "rescate_vel_m_s"),
            (r#"{ "rutinas": { "rescate_espera_s": 11.0 } }"#, "rescate_espera_s"),
            // Retirado: con deny_unknown_fields, el mensaje lo nombra.
            (r#"{ "rutinas": { "reintentos_max": 3 } }"#, "reintentos_max"),
        ] {
            let err = TeamParams::from_json(malo).unwrap_err();
            assert!(err.contains(campo), "{err}");
        }
        assert!(TeamParams::from_json(r#"{ "rutinas": { "pase_empuje_max_m": 0.6 } }"#).is_ok());
    }

    #[test]
    fn rutinas_crash_distance_below_file_distance() {
        for malo in [
            r#"{ "rutinas": { "distancia_choque_m": 0.0 } }"#,
            r#"{ "rutinas": { "distancia_choque_m": 0.25 } }"#,
        ] {
            let err = TeamParams::from_json(malo).unwrap_err();
            assert!(err.contains("distancia_choque_m"), "{err}");
        }
    }

    #[test]
    fn rutinas_speed_percent_in_tens_from_20_to_100() {
        for malo in ["10", "105", "65"] {
            let err = TeamParams::from_json(&format!(
                r#"{{ "rutinas": {{ "velocidad_real_pct": {malo} }} }}"#
            ))
            .unwrap_err();
            assert!(err.contains("velocidad_real_pct"), "{err}");
        }
        let err = TeamParams::from_json(r#"{ "rutinas": { "quieto_s": 0 } }"#).unwrap_err();
        assert!(err.contains("quieto_s"), "{err}");
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
