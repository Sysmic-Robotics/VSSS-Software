//! Configuración tunable centralizada del catálogo de skills.
//!
//! En Fase 2 del plan RL, los parámetros que antes vivían como constantes
//! privadas (`CONTROL_KP`, `CONTROL_KI`, `CONTROL_KD`) se promueven a una
//! struct pública. La motivación:
//!
//! 1. **Tuning sin recompilar**: cuando lleguemos a entrenar y luego a
//!    deployar, vamos a querer ajustar PID gains y velocidad de spin sin
//!    tocar código. Una `SkillConfig` cargable desde TOML/JSON/env hace eso
//!    posible. Por ahora hay solo `Default::default()`, pero el seam queda.
//!
//! 2. **Paridad Rust ↔ Python**: el entorno Python (Fase 4) va a replicar
//!    estas mismas skills. Si los gains divergen, el modelo RL aprende
//!    contra una dinámica que no existe en deploy. Tener un único struct
//!    centralizando defaults facilita la verificación de paridad.
//!
//! 3. **A/B testing de tuning**: poder construir varios `SkillConfig`s en
//!    runtime permite comparar policies entrenadas contra distintos
//!    setpoints sin recompilar.

/// Defaults razonables del catálogo. Calibrados para FIRASim a 60 Hz.
///
/// Si cambian estos valores, hay que actualizar la versión del contrato
/// del catálogo y reentrenar cualquier modelo RL que dependa de ellos.
#[derive(Debug, Clone)]
pub struct SkillConfig {
    /// Ganancia proporcional del PID de heading usado por GoTo, FacePoint
    /// y ChaseBall.
    pub control_kp: f64,
    /// Ganancia integral del PID de heading.
    pub control_ki: f64,
    /// Ganancia derivativa del PID de heading.
    pub control_kd: f64,
    /// Velocidad angular máxima cuando SpinSkill está activa (rad/s).
    /// Se aplica con signo según `SpinSkill::direction`.
    pub spin_omega: f64,
}

impl Default for SkillConfig {
    fn default() -> Self {
        Self {
            control_kp: 3.0,
            control_ki: 0.08,
            control_kd: 0.20,
            // Tiro por giro: magnitud alta para que el cuerpo impulse la pelota
            // (el diff-drive no tiene pateador). Queda holgada bajo el clamp de
            // rueda (±1500 mm/s ≈ 42 rad/s en giro puro). CONTRATO RL: si cambia,
            // subir la versión del catálogo y reentrenar. (era 2.0)
            spin_omega: 20.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spin_omega_within_wheel_clamp() {
        // Spin no pasa por el clamp angular de Motion (lo ignora), así que el
        // límite real es el clamp de rueda de la base: en giro puro cada rueda
        // va a `omega · WHEEL_BASE/2 · 1000` mm/s y no debe superar ±1500.
        const WHEEL_BASE_M: f64 = 0.07; // igual que radio::base_station::WHEEL_BASE_M
        const MAX_WHEEL_MM_S: f64 = 1500.0;
        let cfg = SkillConfig::default();
        let wheel_mm_s = cfg.spin_omega.abs() * WHEEL_BASE_M / 2.0 * 1000.0;
        assert!(
            wheel_mm_s <= MAX_WHEEL_MM_S,
            "spin_omega={} → {} mm/s excede el clamp de rueda",
            cfg.spin_omega,
            wheel_mm_s
        );
    }

    #[test]
    fn pid_gains_are_positive() {
        let cfg = SkillConfig::default();
        assert!(cfg.control_kp > 0.0);
        assert!(cfg.control_ki >= 0.0);
        assert!(cfg.control_kd >= 0.0);
    }
}
