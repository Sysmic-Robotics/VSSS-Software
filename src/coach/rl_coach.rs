//! Coach de RL: carga una política entrenada (ONNX) y la ejecuta bajo el
//! contrato `Coach<SkillChoice>`. Es el deploy del modelo entrenado en Python.
//!
//! Gateado tras el feature `rl` para no forzar la dependencia `tract-onnx`
//! (motor de inferencia ONNX en Rust puro) en el build por defecto del equipo.
//! Compilar con: `cargo build --release --features rl`.
//!
//! # Contrato de la política (espejo del trainer Python)
//! - Input: 52 floats = `Observation::to_flat_vec()`.
//! - Output: `3·N` floats para N robots de campo controlados. Cada tripleta
//!   `[skill_sel, tx, ty]`:
//!     - `skill_sel` → `SkillId` por binning en 5 (idéntico al env Python).
//!     - `(tx, ty)`  → target en coords de campo: `tx·FIELD_HALF_X`, `ty·FIELD_HALF_Y`.
//! - El portero (robot id 2) lo resuelve una regla fija (no lo controla la
//!   política de campo), igual que en el entrenamiento (1 política de campo
//!   compartida + arquero aparte).

use crate::coach::coach_trait::Coach;
use crate::coach::observation::{FIELD_HALF_X, FIELD_HALF_Y, Observation};
use crate::coach::skill_choice::SkillChoice;
use crate::skills::SkillId;
use glam::Vec2;
use tract_onnx::prelude::*;

type OnnxModel = TypedRunnableModel<TypedModel>;

pub struct RlCoach {
    model: OnnxModel,
    own_goal: Vec2,
    /// Si true, agrega un portero rule-based en el robot id 2 cuando la política
    /// controla menos de 3 robots de campo.
    with_goalkeeper: bool,
}

impl RlCoach {
    /// Carga el modelo ONNX desde `path`. `own_goal` se usa para el portero
    /// rule-based. Pánico si el modelo no carga (error de setup, no de runtime).
    pub fn load(path: &str, own_goal: Vec2, with_goalkeeper: bool) -> TractResult<Self> {
        let model = tract_onnx::onnx()
            .model_for_path(path)?
            .with_input_fact(0, f32::fact([1, Observation::FLAT_SIZE]).into())?
            .into_optimized()?
            .into_runnable()?;
        Ok(Self {
            model,
            own_goal,
            with_goalkeeper,
        })
    }

    /// Discretiza el valor continuo `skill_sel ∈ [-1, 1]` a un `SkillId`
    /// (binning en `SkillId::COUNT`=5, idéntico a `VsssSoccerEnv._bin_skill`).
    fn bin_skill(v: f32) -> SkillId {
        let v = v.clamp(-1.0, 1.0);
        let idx = (((v + 1.0) / 2.0) * SkillId::COUNT as f32).floor() as i64;
        let idx = idx.clamp(0, SkillId::COUNT as i64 - 1) as u8;
        SkillId::from_u8(idx).unwrap_or(SkillId::ChaseBall)
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

    /// Corre la inferencia ONNX sobre la observación. Devuelve el vector de
    /// acción crudo, o un vector vacío si la inferencia falla (estado seguro).
    fn infer(&self, obs: &Observation) -> Vec<f32> {
        let flat = obs.to_flat_vec();
        let input = match tract_ndarray::Array2::from_shape_vec((1, flat.len()), flat) {
            Ok(arr) => arr.into_tensor(),
            Err(_) => return Vec::new(),
        };
        match self.model.run(tvec!(input.into())) {
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
        let action = self.infer(obs);
        let num_field = action.len() / 3;
        let mut choices = Vec::with_capacity(num_field + 1);

        for i in 0..num_field {
            let skill_id = Self::bin_skill(action[3 * i]);
            let target = Vec2::new(
                action[3 * i + 1] * FIELD_HALF_X,
                action[3 * i + 2] * FIELD_HALF_Y,
            );
            choices.push(SkillChoice::new(i as i32, skill_id, target));
        }

        // Portero aparte (no lo controla la política de campo).
        if self.with_goalkeeper && num_field < 3 {
            choices.push(self.goalkeeper_choice(obs));
        }

        choices
    }
}
