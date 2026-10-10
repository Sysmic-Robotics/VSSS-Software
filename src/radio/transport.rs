use super::actuator::{ActuatorModel, ActuatorState};
use super::base_station::command_to_vw;
use crate::motion::RobotCommand;
use async_trait::async_trait;
use std::collections::HashMap;
use std::time::{Duration, Instant};

pub type TransportError = Box<dyn std::error::Error + Send + Sync>;

/// Solicitud de reposicionamiento (teleport) en simulador. Coordenadas en metros,
/// marco mundo; `theta` en **grados**, la unidad del replacement de FIRASim y de grSim
/// (se pasa sin convertir). Sin efecto en base station (robots reales).
#[derive(Debug, Clone, Copy)]
pub enum TeleportItem {
    Robot {
        team: u32,
        id: u32,
        x: f64,
        y: f64,
        theta: f64,
    },
    /// Pelota en `(x, y)` con velocidad inicial `(vx, vy)` (m/s, marco mundo; cero para
    /// dejarla quieta).
    Ball {
        x: f64,
        y: f64,
        vx: f64,
        vy: f64,
    },
}

#[async_trait]
pub trait RobotTransport: Send + Sync {
    async fn send_commands(&mut self, commands: &[RobotCommand]) -> Result<(), TransportError>;

    async fn create_robots(&mut self, _robot_ids: &[(u32, u32)]) -> Result<(), TransportError> {
        Ok(())
    }

    /// Reposiciona robots/pelota en el simulador. Default no-op (base station real).
    async fn teleport(&mut self, _items: &[TeleportItem]) -> Result<(), TransportError> {
        Ok(())
    }
}

pub struct FiraSimTransport {
    client: super::FIRASimClient,
    /// Capa de actuador real (`VSSL_REAL_ACTUATOR`), o `None`.
    actuator: Option<RealActuatorLayer>,
}

impl FiraSimTransport {
    pub async fn new(address: &str, port: u16) -> Result<Self, TransportError> {
        let actuator = match super::actuator::selected_measurement()? {
            Some((m, sim)) => {
                let model = ActuatorModel::for_firasim(&m, sim.as_ref());
                eprintln!(
                    "[Radio] capa de actuador real ACTIVA: robot {} ({}, {}): retardo {:.0} ms, \
                     τ_v {:.3} s, τ_ω {:.3} s, tope {:.0} mm/s por rueda{}",
                    m.vision_id,
                    m.battery,
                    m.date,
                    model.delay_s * 1000.0,
                    model.tau_v_s,
                    model.tau_w_s,
                    m.max_wheel_mm_s,
                    if sim.is_some() { "" } else { " (sin descontar FIRASim: falta la sección firasim)" }
                );
                Some(RealActuatorLayer { model, states: HashMap::new() })
            }
            None => None,
        };
        Ok(Self {
            client: super::FIRASimClient::new(address, port).await?,
            actuator,
        })
    }
}

/// La capa aplicada a cada robot que manda este engine, con su propio estado y su
/// propio reloj.
struct RealActuatorLayer {
    model: ActuatorModel,
    states: HashMap<(i32, i32), (ActuatorState, Instant)>,
}

impl RealActuatorLayer {
    /// Reemplaza cada comando por el (v, ω) que ejecutaría el robot real con ese frame,
    /// expresado en el heading (`orientation` = 0, `vx` = v), para que el serializador
    /// lo lleve a ruedas con la geometría de FIRASim.
    fn apply(&mut self, commands: &[RobotCommand], now: Instant) -> Vec<RobotCommand> {
        commands
            .iter()
            .map(|c| {
                let (v_mm_s, w_deg_s) = command_to_vw(&c.motion);
                let (state, last) = self
                    .states
                    .entry((c.team, c.id))
                    .or_insert_with(|| (ActuatorState::default(), now - FIRST_DT));
                let dt = now.duration_since(*last).as_secs_f64().min(0.1);
                *last = now;
                let (v, w) = state.step(&self.model, v_mm_s, w_deg_s, dt);
                let mut out = c.clone();
                out.motion.vx = v;
                out.motion.vy = 0.0;
                out.motion.omega = w;
                out.motion.orientation = 0.0;
                out
            })
            .collect()
    }
}

/// `dt` del primer comando de un robot: un tick del loop.
const FIRST_DT: Duration = Duration::from_micros(16_667);

#[async_trait]
impl RobotTransport for FiraSimTransport {
    async fn send_commands(&mut self, commands: &[RobotCommand]) -> Result<(), TransportError> {
        match self.actuator.as_mut() {
            Some(layer) => {
                let commands = layer.apply(commands, Instant::now());
                self.client.send_commands(&commands).await
            }
            None => self.client.send_commands(commands).await,
        }
    }

    async fn create_robots(&mut self, robot_ids: &[(u32, u32)]) -> Result<(), TransportError> {
        self.client.create_robots(robot_ids).await
    }

    async fn teleport(&mut self, items: &[TeleportItem]) -> Result<(), TransportError> {
        self.client.teleport(items).await
    }
}

pub struct GrSimTransport {
    client: super::GrSimClient,
}

impl GrSimTransport {
    pub async fn new(address: &str, port: u16) -> Result<Self, TransportError> {
        Ok(Self {
            client: super::GrSimClient::new(address, port).await?,
        })
    }
}

#[async_trait]
impl RobotTransport for GrSimTransport {
    async fn send_commands(&mut self, commands: &[RobotCommand]) -> Result<(), TransportError> {
        let (blue, yellow): (Vec<_>, Vec<_>) = commands.iter().cloned().partition(|c| c.team == 0);
        if !blue.is_empty() {
            self.client.send_commands(&blue).await?;
        }
        if !yellow.is_empty() {
            self.client.send_commands(&yellow).await?;
        }
        Ok(())
    }

    async fn teleport(&mut self, items: &[TeleportItem]) -> Result<(), TransportError> {
        self.client.teleport(items).await
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::{KickerCommand, MotionCommand};
    use crate::radio::actuator::Measurement;

    fn command(id: i32, v: f64, w: f64, th: f64) -> RobotCommand {
        let motion = MotionCommand {
            id,
            team: 0,
            vx: v * th.cos(),
            vy: v * th.sin(),
            omega: w,
            orientation: th,
        };
        RobotCommand {
            id,
            team: 0,
            motion,
            kicker: KickerCommand { id, team: 0, kick_x: false, kick_z: false, dribbler: 0.0 },
        }
    }

    fn layer() -> RealActuatorLayer {
        let m = Measurement {
            vision_id: 1,
            mi_robot_id: 2,
            battery: "full".into(),
            battery_v: 8.2,
            date: "2026-10-08".into(),
            notes: String::new(),
            latency_ms: 50.0,
            vision_latency_ms: 0.0,
            tau_v_s: 0.1,
            tau_w_s: 0.1,
            max_accel_m_s2: 100.0,
            max_alpha_rad_s2: 1000.0,
            deadzone_wheel_mm_s: 20.0,
            max_wheel_mm_s: 450.0,
            wheel_track_mm: 75.0,
            gain_v: 1.0,
            gain_w: 1.0,
            fit: serde_json::Value::Null,
        };
        RealActuatorLayer { model: ActuatorModel::for_plant(&m), states: HashMap::new() }
    }

    #[test]
    fn layer_caps_each_wheel_in_firasim_rad_s() {
        // 600 mm/s en un heading cualquiera: en régimen, las dos ruedas a 450 mm/s
        // = 22.5 rad/s con la rueda de 0.02 m de FIRASim.
        let mut layer = layer();
        let t0 = Instant::now();
        let mut out = Vec::new();
        for k in 0..240u32 {
            out = layer.apply(&[command(0, 0.6, 0.0, 1.0)], t0 + FIRST_DT * k);
        }
        let m = &out[0].motion;
        let (l, r, _) = super::super::commands::fira_wheel_speeds(m.vx, m.vy, m.omega, m.orientation);
        let r_wheel = crate::params::params().sim.wheel_radius_m;
        assert!((l * r_wheel - 0.45).abs() < 1e-3 && (r * r_wheel - 0.45).abs() < 1e-3, "{l} {r}");
    }

    #[test]
    fn layer_keeps_a_state_per_robot() {
        let mut layer = layer();
        let t0 = Instant::now();
        for k in 0..60u32 {
            layer.apply(&[command(0, 0.3, 0.0, 0.0)], t0 + FIRST_DT * k);
        }
        // El robot 1 recién empieza: su retardo de 50 ms lo deja en cero.
        let out = layer.apply(&[command(0, 0.3, 0.0, 0.0), command(1, 0.3, 0.0, 0.0)], t0 + FIRST_DT * 60);
        assert!(out[0].motion.vx > 0.29, "robot 0 en régimen: {}", out[0].motion.vx);
        assert_eq!(out[1].motion.vx, 0.0);
    }
}
