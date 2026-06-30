//! Coach de RL: carga una política entrenada (ONNX) y la ejecuta bajo el
//! contrato `Coach<SkillChoice>`. Es el deploy del modelo entrenado en Python.
//!
//! Gateado tras el feature `rl` para no forzar la dependencia `tract-onnx`
//! (motor de inferencia ONNX en Rust puro) en el build por defecto del equipo.
//! Compilar con: `cargo build --release --features rl`.
//!
//! # Contrato de la política (espejo del trainer Python — Intento 3, control (v,ω))
//! - Input: 52 floats = `Observation::to_flat_vec()`.
//! - Output: `2·N` floats para N robots de campo controlados. Cada par `[v, ω]`
//!   en `[-1, 1]` se denormaliza con los MISMOS topes que el env de entrenamiento
//!   (`V_MAX`, `OMEGA_MAX`) y se emite como `SkillId::DirectVel` → `MotionCommand`
//!   directo a ruedas, SIN UVF. Así el sim y el runtime ejecutan la misma dinámica
//!   (cierra el gap de executor que hacía "torpe" al modelo de skills).
//! - El portero (robot id 2) lo controla una red de arquero entrenada aparte
//!   (2º ONNX, `VSSL_RL_GK_MODEL`, también `(v,ω)`); si no carga, cae a una regla
//!   fija. La política de campo NO controla al arquero (1 red de campo + 1 de arquero).

use crate::coach::coach_trait::Coach;
use crate::coach::observation::{FIELD_HALF_X, FIELD_HALF_Y, Observation};
use crate::coach::skill_choice::SkillChoice;
use crate::skills::SkillId;
use glam::Vec2;
use tract_onnx::prelude::*;

type OnnxModel = TypedRunnableModel<TypedModel>;

/// Topes de la acción `(v, ω)` — espejo EXACTO de `SIM_V_MAX` / `SIM_OMEGA_MAX`
/// del env de entrenamiento (`soccer_env.py`). La red emite `[-1, 1]`; acá se
/// denormaliza a m/s y rad/s. Si cambian en Python, actualizar acá.
const V_MAX: f32 = 1.2;
const OMEGA_MAX: f32 = 3.0;

pub struct RlCoach {
    model: OnnxModel,
    /// Arquero entrenado (opcional). Si está, controla el robot id 2 con su
    /// propia red; si es `None`, se usa el arquero rule-based (`goalkeeper_choice`).
    gk_model: Option<OnnxModel>,
    own_goal: Vec2,
    /// Si true, agrega un portero (RL si `gk_model` está, si no rule-based) en el
    /// robot id 2 cuando la política de campo controla menos de 3 robots.
    with_goalkeeper: bool,
    /// Diagnóstico (VSSL_RL_DEBUG=1): loguea obs + acciones cada ~20 decisiones.
    dbg: bool,
    dbg_count: u32,
}

impl RlCoach {
    /// Carga el modelo ONNX de campo desde `path`. Si `gk_path` es `Some` y carga
    /// bien, el arquero (robot id 2) lo controla esa red entrenada; si es `None` o
    /// falla la carga, se usa el arquero rule-based. `own_goal` se usa para el
    /// rule-based. Error de carga del modelo de campo = error de setup (no runtime).
    pub fn load(
        path: &str,
        gk_path: Option<&str>,
        own_goal: Vec2,
        with_goalkeeper: bool,
    ) -> TractResult<Self> {
        let model = Self::load_model(path)?;
        let gk_model = match gk_path {
            Some(p) => match Self::load_model(p) {
                Ok(m) => {
                    eprintln!("[RlCoach] arquero RL cargado desde {p}");
                    Some(m)
                }
                Err(e) => {
                    eprintln!("[RlCoach] no se pudo cargar arquero RL ({e}); uso rule-based");
                    None
                }
            },
            None => None,
        };
        let dbg = std::env::var("VSSL_RL_DEBUG").map(|v| v == "1").unwrap_or(false);
        Ok(Self {
            model,
            gk_model,
            own_goal,
            with_goalkeeper,
            dbg,
            dbg_count: 0,
        })
    }

    /// Carga un modelo ONNX (campo o arquero) fijando el input a la observación
    /// de 52 floats. Ambas redes comparten el mismo contrato de entrada.
    fn load_model(path: &str) -> TractResult<OnnxModel> {
        tract_onnx::onnx()
            .model_for_path(path)?
            .with_input_fact(0, f32::fact([1, Observation::FLAT_SIZE]).into())?
            .into_optimized()?
            .into_runnable()
    }

    /// Portero rule-based (idéntico a RuleBasedCoach::goalkeeper_choice): se
    /// para 12 cm delante del arco propio y sigue la `y` de la pelota clampeada.
    fn goalkeeper_choice(&self, obs: &Observation) -> SkillChoice {
        let ball = Vec2::new(obs.ball.x * FIELD_HALF_X, obs.ball.y * FIELD_HALF_Y);
        let sign = if self.own_goal.x < 0.0 { 1.0_f32 } else { -1.0 };
        let defend_x = self.own_goal.x + sign * 0.12;
        let target_y = ball.y.clamp(-0.20, 0.20);
        SkillChoice::goto(2, Vec2::new(defend_x, target_y))
    }

    /// Corre la inferencia ONNX de `model` sobre la observación. Devuelve el
    /// vector de acción crudo, o vacío si la inferencia falla (estado seguro).
    fn run_model(model: &OnnxModel, obs: &Observation) -> Vec<f32> {
        let flat = obs.to_flat_vec();
        let input = match tract_ndarray::Array2::from_shape_vec((1, flat.len()), flat) {
            Ok(arr) => arr.into_tensor(),
            Err(_) => return Vec::new(),
        };
        match model.run(tvec!(input.into())) {
            Ok(result) => match result[0].to_array_view::<f32>() {
                Ok(view) => view.iter().copied().collect(),
                Err(_) => Vec::new(),
            },
            Err(_) => Vec::new(),
        }
    }
}

impl Coach for RlCoach {
    fn decide(&mut self, obs: &Observation) -> Vec<SkillChoice> {
        let action = Self::run_model(&self.model, obs);
        let num_field = action.len() / 2;
        let mut choices = Vec::with_capacity(num_field + 1);

        for i in 0..num_field {
            // Intento 3: la política emite (v, ω) en [-1,1]. Clamp + denormalización
            // con los mismos topes que el env, y se emite como DirectVel (sin UVF).
            let v = action[2 * i].clamp(-1.0, 1.0) * V_MAX;
            let omega = action[2 * i + 1].clamp(-1.0, 1.0) * OMEGA_MAX;
            choices.push(SkillChoice::new(i as i32, SkillId::DirectVel, Vec2::new(v, omega)));
        }

        // Arquero (robot id 2), si la política de campo controla <3 robots.
        if num_field < 3 {
            if let Some(gk) = self.gk_model.as_ref() {
                // Arquero entrenado: misma observación de 52 floats → (v, ω).
                let gk_action = Self::run_model(gk, obs);
                if gk_action.len() >= 2 {
                    let v = gk_action[0].clamp(-1.0, 1.0) * V_MAX;
                    let omega = gk_action[1].clamp(-1.0, 1.0) * OMEGA_MAX;
                    choices.push(SkillChoice::new(2, SkillId::DirectVel, Vec2::new(v, omega)));
                } else if self.with_goalkeeper {
                    choices.push(self.goalkeeper_choice(obs)); // fallback si falla la inferencia
                }
            } else if self.with_goalkeeper {
                choices.push(self.goalkeeper_choice(obs)); // rule-based
            }
        }

        // Diagnóstico throttleado: ¿qué percibe y qué decide la red? (VSSL_RL_DEBUG=1)
        if self.dbg {
            self.dbg_count = self.dbg_count.wrapping_add(1);
            if self.dbg_count % 20 == 0 {
                let f = obs.to_flat_vec();
                eprintln!(
                    "[rl-dbg] nf={} ball=({:.2},{:.2}) own0=({:.2},{:.2} act{:.0}) own1=({:.2},{:.2} act{:.0}) own2=({:.2},{:.2} act{:.0})",
                    num_field, f[0], f[1], f[4], f[5], f[11], f[12], f[13], f[19], f[20], f[21], f[27]
                );
                for c in &choices {
                    eprintln!(
                        "[rl-dbg]   robot {} -> {:?} tgt=({:.2},{:.2})",
                        c.robot_id, c.skill_id, c.target.x, c.target.y
                    );
                }
            }
        }

        choices
    }
}
