use crate::coach::observation::Observation;
use crate::coach::robot_target::RobotTarget;

/// Trait que implementa cualquier estrategia de alto nivel: reglas clásicas o modelo RL.
///
/// # Contrato
/// - `decide` es llamado exactamente 1× por tick a ~60Hz.
/// - Debe retornar un `RobotTarget` por cada robot activo del equipo.
/// - Si falta un robot en el Vec retornado, `CoachPlay` lo mantiene en posición.
/// - Debe completar en < 2ms para no perder el deadline de 16ms del loop de control.
///
/// # Por qué recibe `&Observation` y no `&World`
/// - El modelo RL necesita `obs.to_flat_vec()` — la conversión ocurre una sola vez en `CoachPlay`.
/// - `RuleBasedCoach` también usa `Observation`, validando paridad con el modelo RL.
/// - Los tests unitarios del Coach construyen `Observation` directamente sin World.
/// - El contrato es explícito: el Coach solo ve estado normalizado, no los internals del engine.
pub trait Coach: Send + Sync {
    fn decide(&mut self, obs: &Observation) -> Vec<RobotTarget>;
}
