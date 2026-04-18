use crate::motion::MotionCommand;

/// Escenarios simples inspirados en benchmarks de equipos SSL para tuning rápido.
#[derive(Debug, Clone, Copy)]
pub enum MotionBenchmarkScenario {
    LineStepTarget,
    RotateInPlace,
    ChaseTarget,
}

/// KPIs mínimos para comparar iteraciones de control.
#[derive(Debug, Clone, Copy, Default)]
pub struct MotionKpi {
    pub mean_speed: f64,
    pub peak_speed: f64,
    pub mean_abs_omega: f64,
}

/// Evalúa una secuencia de comandos y extrae indicadores básicos.
pub fn summarize_commands(commands: &[MotionCommand]) -> MotionKpi {
    if commands.is_empty() {
        return MotionKpi::default();
    }

    let mut speed_sum = 0.0f64;
    let mut peak_speed = 0.0f64;
    let mut omega_sum = 0.0f64;

    for cmd in commands {
        let speed = (cmd.vx * cmd.vx + cmd.vy * cmd.vy).sqrt();
        speed_sum += speed;
        peak_speed = peak_speed.max(speed);
        omega_sum += cmd.omega.abs();
    }

    MotionKpi {
        mean_speed: speed_sum / commands.len() as f64,
        peak_speed,
        mean_abs_omega: omega_sum / commands.len() as f64,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn summarize_commands_reports_expected_values() {
        let data = vec![
            MotionCommand {
                id: 0,
                team: 0,
                vx: 1.0,
                vy: 0.0,
                omega: 0.5,
                orientation: 0.0,
            },
            MotionCommand {
                id: 0,
                team: 0,
                vx: 0.0,
                vy: 1.0,
                omega: -0.5,
                orientation: 0.0,
            },
            MotionCommand {
                id: 0,
                team: 0,
                vx: 0.0,
                vy: 0.0,
                omega: 0.0,
                orientation: 0.0,
            },
        ];

        let kpi = summarize_commands(&data);
        assert!(kpi.mean_speed > 0.0);
        assert!(kpi.peak_speed >= 1.0);
        assert!(kpi.mean_abs_omega > 0.0);
    }
}
