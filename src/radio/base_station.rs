use crate::motion::RobotCommand;
use crate::radio::transport::{RobotTransport, TransportError};
use async_trait::async_trait;
use std::time::Duration;
use tokio::io::AsyncWriteExt;
use tokio_serial::{SerialPortBuilderExt, SerialStream};

/// Equipo propio que la base station controla.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TeamColor {
    Blue,
    Yellow,
}

impl TeamColor {
    pub fn as_team_id(self) -> i32 {
        match self {
            TeamColor::Blue => 0,
            TeamColor::Yellow => 1,
        }
    }

    pub fn from_env() -> Self {
        match std::env::var("VSSL_TEAM_COLOR")
            .unwrap_or_default()
            .to_ascii_lowercase()
            .as_str()
        {
            "yellow" | "amarillo" | "1" => TeamColor::Yellow,
            "" | "blue" | "azul" | "0" => TeamColor::Blue,
            other => {
                eprintln!(
                    "[BaseStation] VSSL_TEAM_COLOR='{other}' inválido, usando 'blue' por defecto"
                );
                TeamColor::Blue
            }
        }
    }
}

/// Cantidad de slots en el frame ASCII (firmware espera 5 pares L,R).
pub const SLOT_COUNT: usize = 5;

/// Límite de velocidad de rueda en mm/s — el mismo clamp que aplica la base
/// (`base_station2.ino:14`, `MAX_WHEEL_MM_S = 1500`). Defensa en profundidad:
/// la base también recortaría, pero saturar en Rust mantiene el frame
/// explícito y deja preparado el lugar para futuro logging de saturación.
pub const MAX_WHEEL_MM_S: i32 = 1500;

/// Separación física entre ruedas izquierda y derecha del robot real, en metros.
/// Usada en la cinemática inversa diferencial `(v, ω) → (v_izq, v_der)`.
// medido 2026-06-13; calibración fina del giro pendiente de validar en banco
pub const WHEEL_BASE_M: f64 = 0.07;

// Contrato del frame ASCII PC → base ESP32 (verificado contra base_station2.ino:142-172):
//   - Formato exacto: "L1,R1,L2,R2,L3,R3,L4,R4,L5,R5\n"
//   - 10 enteros decimales separados por coma, terminador '\n' (sin '\r').
//   - Unidad: mm/s, clamp ±MAX_WHEEL_MM_S.
//   - Slots 0-based: el par en posición i corresponde a cmd.id = i (i ∈ 0..SLOT_COUNT).
//   - Mapeo a firmware: el robot físico con MI_ROBOT_ID = N (firmware, 1-based)
//     lee robots[N - 1] del binario que arma la base (communication.cpp:91-94).
//     Por tanto cmd.id en Rust (0-based) = MI_ROBOT_ID - 1 en firmware.
//   - La cinemática inversa diferencial se hace en PC (este archivo); el firmware
//     en modo ESP-NOW solo aplica velocidades de rueda directas.
//
// Cinemática inversa diferencial:
//   v     = vx·cos(orientation) + vy·sin(orientation)   // m/s (proyección al heading)
//   v_izq = v − (omega · WHEEL_BASE_M) / 2              // m/s
//   v_der = v + (omega · WHEEL_BASE_M) / 2              // m/s
// Convención: omega > 0 → CCW visto desde arriba → rueda derecha más rápida.
// Coincide con la convención Spin del catálogo de skills (CLAUDE.md §5).
pub fn command_to_wheel_mm_s(motion: &crate::motion::MotionCommand) -> (i16, i16) {
    if !motion.vx.is_finite()
        || !motion.vy.is_finite()
        || !motion.omega.is_finite()
        || !motion.orientation.is_finite()
    {
        return (0, 0);
    }

    let cos_t = motion.orientation.cos();
    let sin_t = motion.orientation.sin();
    let v = motion.vx * cos_t + motion.vy * sin_t;
    let half_wheel_term = motion.omega * WHEEL_BASE_M / 2.0;

    let v_left_mm_s = ((v - half_wheel_term) * 1000.0).round() as i32;
    let v_right_mm_s = ((v + half_wheel_term) * 1000.0).round() as i32;

    let left = v_left_mm_s.clamp(-MAX_WHEEL_MM_S, MAX_WHEEL_MM_S) as i16;
    let right = v_right_mm_s.clamp(-MAX_WHEEL_MM_S, MAX_WHEEL_MM_S) as i16;
    (left, right)
}

/// Construye el frame ASCII "L1,R1,...,L5,R5\n" a partir de comandos del equipo propio.
/// Slot index = cmd.id (0-based); ids fuera de rango se ignoran; slots sin comando salen 0,0.
pub fn build_frame(commands: &[RobotCommand], own_team: TeamColor) -> String {
    let mut slots = [(0i16, 0i16); SLOT_COUNT];
    for cmd in commands {
        if cmd.team != own_team.as_team_id() {
            continue;
        }
        let idx = cmd.id as usize;
        if idx < SLOT_COUNT {
            slots[idx] = command_to_wheel_mm_s(&cmd.motion);
        }
    }

    let mut out = String::with_capacity(64);
    for (i, (l, r)) in slots.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        out.push_str(&format!("{l},{r}"));
    }
    out.push('\n');
    out
}

pub struct BaseStationTransport {
    port: SerialStream,
    own_team: TeamColor,
    device: String,
}

impl BaseStationTransport {
    pub fn new(device: &str, baud: u32, own_team: TeamColor) -> Result<Self, TransportError> {
        let port = tokio_serial::new(device, baud)
            .timeout(Duration::from_millis(50))
            .open_native_async()?;
        eprintln!(
            "[BaseStation] Abierto {device} @ {baud} baud, equipo propio = {:?}",
            own_team
        );
        Ok(Self {
            port,
            own_team,
            device: device.to_string(),
        })
    }

    pub fn from_env(own_team: TeamColor) -> Result<Self, TransportError> {
        let device =
            std::env::var("VSSL_BASESTATION_DEVICE").unwrap_or_else(|_| "/dev/ttyUSB0".to_string());
        let baud = std::env::var("VSSL_BASESTATION_BAUD")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(115200u32);
        Self::new(&device, baud, own_team)
    }
}

#[async_trait]
impl RobotTransport for BaseStationTransport {
    async fn send_commands(&mut self, commands: &[RobotCommand]) -> Result<(), TransportError> {
        let frame = build_frame(commands, self.own_team);
        match self.port.write_all(frame.as_bytes()).await {
            Ok(()) => Ok(()),
            Err(e) => {
                eprintln!("[BaseStation] error escribiendo en {}: {e}", self.device);
                Err(e.into())
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::{KickerCommand, MotionCommand};

    fn make_cmd(
        team: i32,
        id: i32,
        vx: f64,
        vy: f64,
        omega: f64,
        orientation: f64,
    ) -> RobotCommand {
        RobotCommand {
            id,
            team,
            motion: MotionCommand {
                id,
                team,
                vx,
                vy,
                omega,
                orientation,
            },
            kicker: KickerCommand {
                id,
                team,
                kick_x: false,
                kick_z: false,
                dribbler: 0.0,
            },
        }
    }

    fn motion(vx: f64, vy: f64, omega: f64, orientation: f64) -> MotionCommand {
        MotionCommand {
            id: 0,
            team: 0,
            vx,
            vy,
            omega,
            orientation,
        }
    }

    /// Half-wheel contribution of omega to a single wheel, en mm/s, computado desde
    /// la constante para que un cambio del const no rompa el test silenciosamente.
    fn spin_half_mm_s(omega: f64) -> i16 {
        (omega * WHEEL_BASE_M / 2.0 * 1000.0).round() as i16
    }

    // ---------- Cinemática ----------

    #[test]
    fn kinematics_pure_forward_zero_heading() {
        // Avance puro, sin omega: ambas ruedas iguales, en mm/s = vx · 1000.
        assert_eq!(
            command_to_wheel_mm_s(&motion(1.0, 0.0, 0.0, 0.0)),
            (1000, 1000)
        );
    }

    #[test]
    fn kinematics_pure_forward_heading_pi_over_2() {
        // Robot mirando a +Y: la proyección al heading recupera v = vy.
        assert_eq!(
            command_to_wheel_mm_s(&motion(0.0, 0.5, 0.0, std::f64::consts::FRAC_PI_2)),
            (500, 500)
        );
    }

    #[test]
    fn kinematics_pure_ccw_spin() {
        // Giro puro CCW (omega > 0): rueda derecha avanza, izquierda retrocede.
        let half = spin_half_mm_s(2.0);
        assert_eq!(
            command_to_wheel_mm_s(&motion(0.0, 0.0, 2.0, 0.0)),
            (-half, half)
        );
    }

    #[test]
    fn kinematics_forward_plus_spin() {
        // Combinación: avance + giro. Calculado desde la constante.
        let half = spin_half_mm_s(2.0);
        assert_eq!(
            command_to_wheel_mm_s(&motion(0.5, 0.0, 2.0, 0.0)),
            (500 - half, 500 + half)
        );
    }

    // ---------- Clamp ----------

    #[test]
    fn clamp_saturates_both_wheels() {
        // v enorme → ambos saturan al mismo signo.
        assert_eq!(
            command_to_wheel_mm_s(&motion(5.0, 0.0, 0.0, 0.0)),
            (1500, 1500)
        );
    }

    #[test]
    fn clamp_saturates_opposite_signs() {
        // omega enorme → un lado +1500, otro −1500.
        assert_eq!(
            command_to_wheel_mm_s(&motion(0.0, 0.0, 100.0, 0.0)),
            (-1500, 1500)
        );
    }

    #[test]
    fn clamp_borderline_does_not_overflow() {
        // ±1.5 m/s exactos saturan a ±1500 sin overflow ni panic.
        assert_eq!(
            command_to_wheel_mm_s(&motion(1.5, 0.0, 0.0, 0.0)),
            (1500, 1500)
        );
        assert_eq!(
            command_to_wheel_mm_s(&motion(-1.5, 0.0, 0.0, 0.0)),
            (-1500, -1500)
        );
    }

    // ---------- Robustez ante no-finitos ----------

    #[test]
    fn robustness_nan_returns_zero() {
        assert_eq!(
            command_to_wheel_mm_s(&motion(f64::NAN, 0.0, 0.0, 0.0)),
            (0, 0)
        );
    }

    #[test]
    fn robustness_inf_omega_returns_zero() {
        assert_eq!(
            command_to_wheel_mm_s(&motion(0.0, 0.0, f64::INFINITY, 0.0)),
            (0, 0)
        );
    }

    #[test]
    fn robustness_neg_inf_orientation_returns_zero() {
        assert_eq!(
            command_to_wheel_mm_s(&motion(1.0, 0.0, 0.0, f64::NEG_INFINITY)),
            (0, 0)
        );
    }

    // ---------- Frame golden ----------

    #[test]
    fn frame_golden_forward_blue_slot_0() {
        let cmds = vec![make_cmd(0, 0, 0.5, 0.0, 0.0, 0.0)];
        let frame = build_frame(&cmds, TeamColor::Blue);
        assert_eq!(frame, "500,500,0,0,0,0,0,0,0,0\n");
    }

    #[test]
    fn frame_golden_spin_blue_slot_2() {
        // El golden se construye desde la constante para no romper silenciosamente
        // si WHEEL_BASE_M se recalibra.
        let half = spin_half_mm_s(2.0);
        let cmds = vec![make_cmd(0, 2, 0.0, 0.0, 2.0, 0.0)];
        let frame = build_frame(&cmds, TeamColor::Blue);
        let expected = format!("0,0,0,0,{},{},0,0,0,0\n", -half, half);
        assert_eq!(frame, expected);
    }

    // ---------- Filtro de team y descarte ----------

    #[test]
    fn frame_filters_opposing_team() {
        // own_team = Blue, comando con team = Yellow → ignorado.
        let cmds = vec![make_cmd(1, 0, 1.0, 0.0, 0.0, 0.0)];
        let frame = build_frame(&cmds, TeamColor::Blue);
        assert_eq!(frame, "0,0,0,0,0,0,0,0,0,0\n");
    }

    #[test]
    fn frame_drops_id_out_of_range() {
        let cmds = vec![make_cmd(0, 9, 1.0, 0.0, 0.0, 0.0)];
        let frame = build_frame(&cmds, TeamColor::Blue);
        assert_eq!(frame, "0,0,0,0,0,0,0,0,0,0\n");
    }

    #[test]
    fn frame_yellow_team_filters_blue_out() {
        // own_team = Yellow → comando team=0 (Blue) ignorado; team=1 (Yellow) entra.
        let cmds = vec![
            make_cmd(0, 0, 1.0, 0.0, 0.0, 0.0),
            make_cmd(1, 1, 0.5, 0.0, 0.0, 0.0),
        ];
        let frame = build_frame(&cmds, TeamColor::Yellow);
        assert_eq!(frame, "0,0,500,500,0,0,0,0,0,0\n");
    }
}
