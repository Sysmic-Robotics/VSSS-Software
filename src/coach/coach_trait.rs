use crate::coach::observation::Observation;
use crate::coach::skill_choice::SkillChoice;

/// Trait que implementa cualquier estrategia de alto nivel: reglas clásicas
/// o modelo RL.
///
/// **Cambio en Fase 3 del plan RL**: la firma cambió de
/// `decide -> Vec<RobotTarget>` (target-espacial) a `decide -> Vec<SkillChoice>`
/// (selección discreta de skill + parámetro). Esto alinea la interfaz con
/// el patrón ItAndroids / Bassani 2020 / LARC 2019, donde la policy elige
/// **qué skill correr** sobre el catálogo cerrado y el dispatcher de bajo
/// nivel ejecuta esa skill con UVF + PID.
///
/// # Contrato
/// - `decide` se llama cada N ticks según el horizonte configurado en el
///   dispatcher (Fase 3 usa **opción E** con frame-skip K=6 ≈ 100ms).
/// - Retorna **una `SkillChoice` por robot activo del equipo propio**.
///   Robots inactivos pueden omitirse: el dispatcher ignora choices cuyos
///   `robot_id` no aparecen en el equipo activo.
/// - Debe completar en < 2ms para no perder el deadline de 16ms del
///   control loop, aunque el frame-skip lo invoca cada 6 ticks.
///
/// # Por qué recibe `&Observation` y no `&World`
/// - El modelo RL necesita `obs.to_flat_vec()` — la conversión ocurre una
///   sola vez en el dispatcher.
/// - `RuleBasedCoach` también usa `Observation`, lo que valida paridad
///   numérica con el modelo RL bajo el mismo contrato de entrada.
/// - Los tests unitarios construyen `Observation` directamente, sin World.
/// - El contrato es explícito: el Coach solo ve estado normalizado, nunca
///   los internals del engine (motion, radio, etc.).
pub trait Coach: Send + Sync {
    fn decide(&mut self, obs: &Observation) -> Vec<SkillChoice>;
}
