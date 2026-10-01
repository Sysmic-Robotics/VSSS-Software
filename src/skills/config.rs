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
/// Ya no son un contrato de RL congelado (el paradigma es STP; el RL es una capa
/// opcional): se recalibran con mediciones del robot real.
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
            // (el diff-drive no tiene pateador). Parámetro a calibrar tras medir en
            // banco (era 2.0). En FIRASim es ejecutable (0.85 m/s por rueda < 1.2).
            // En el robot real NO: el frame recorta ω a 720 °/s (≈ 12.6 rad/s) y el
            // firmware satura en ~12 rad/s (450 mm/s por rueda sobre una vía de
            // 75 mm). OJO: Spin toma este default de código; el `skills.spin_omega`
            // de config/team_params.json es el de SpinKick.
            spin_omega: 20.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spin_omega_within_sim_wheel_clamp() {
        // Spin no pasa por el clamp angular de Motion (lo ignora). En el sim, el
        // límite es el clamp de rueda de FIRASim: en giro puro cada rueda va a
        // `omega · L/2` m/s y no debe superar `max_wheel_rad_s · r`. En el robot
        // real el frame recorta ω a `robot.max_w_deg_s` (test en radio::base_station).
        let sim = &crate::params::params().sim;
        let cfg = SkillConfig::default();
        let wheel_m_s = cfg.spin_omega.abs() * sim.wheel_base_m / 2.0;
        let max_wheel_m_s = sim.max_wheel_rad_s * sim.wheel_radius_m;
        assert!(
            wheel_m_s <= max_wheel_m_s,
            "spin_omega={} → {wheel_m_s} m/s excede el clamp de rueda del sim ({max_wheel_m_s} m/s)",
            cfg.spin_omega
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
