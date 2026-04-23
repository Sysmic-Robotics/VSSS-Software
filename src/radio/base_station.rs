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

/// Cantidad de slots en el frame ASCII (firmware espera 5 pares x,y).
pub const SLOT_COUNT: usize = 5;

/// Conversión vx/vy/omega → joystick X,Y firmware-side.
pub const SCALE_LIN: f64 = 200.0;
pub const SCALE_ANG: f64 = 15.0;
pub const JOYSTICK_MAX: i32 = 100;

/// Convierte un MotionCommand (frame mundial) a (X, Y) firmware.
/// Tras PR#4 del firmware: X = velocidad lineal, Y = velocidad angular.
pub fn command_to_xy(motion: &crate::motion::MotionCommand) -> (i32, i32) {
    let cos_t = motion.orientation.cos();
    let sin_t = motion.orientation.sin();
    let v_forward = motion.vx * cos_t + motion.vy * sin_t;

    let x_raw = (v_forward * SCALE_LIN).round() as i32; // X = lineal
    let y_raw = (motion.omega * SCALE_ANG).round() as i32; // Y = angular

    (
        x_raw.clamp(-JOYSTICK_MAX, JOYSTICK_MAX),
        y_raw.clamp(-JOYSTICK_MAX, JOYSTICK_MAX),
    )
}

/// Construye el frame ASCII "x1,y1,...,x5,y5\n" a partir de comandos del equipo propio.
/// Slot index = robot id; ids fuera de rango se ignoran; slots sin comando salen 0,0.
pub fn build_frame(commands: &[RobotCommand], own_team: TeamColor) -> String {
    let mut slots = [(0i32, 0i32); SLOT_COUNT];
    for cmd in commands {
        if cmd.team != own_team.as_team_id() {
            continue;
        }
        let idx = cmd.id as usize;
        if idx < SLOT_COUNT {
            slots[idx] = command_to_xy(&cmd.motion);
        }
    }

    let mut out = String::with_capacity(64);
    for (i, (x, y)) in slots.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        out.push_str(&format!("{x},{y}"));
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

    fn make_cmd(team: i32, id: i32, vx: f64, vy: f64, omega: f64, orientation: f64) -> RobotCommand {
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

    #[test]
    fn xy_forward_along_heading() {
        // Robot mirando hacia +X (theta=0), velocidad global +X.
        // v_forward = 1.0 m/s → X = 200 → clamp 100. Sin omega → Y = 0.
        let cmd = MotionCommand {
            id: 0,
            team: 0,
            vx: 1.0,
            vy: 0.0,
            omega: 0.0,
            orientation: 0.0,
        };
        assert_eq!(command_to_xy(&cmd), (100, 0));
    }

    #[test]
    fn xy_forward_with_heading_pi_over_2() {
        // Robot mirando hacia +Y (theta = π/2). Velocidad global +Y debe ser forward.
        let cmd = MotionCommand {
            id: 0,
            team: 0,
            vx: 0.0,
            vy: 0.4,
            omega: 0.0,
            orientation: std::f64::consts::FRAC_PI_2,
        };
        let (x, y) = command_to_xy(&cmd);
        assert_eq!(x, 80); // 0.4 * 200 → X = lineal
        assert_eq!(y, 0);
    }

    #[test]
    fn xy_omega_drives_x() {
        let cmd = MotionCommand {
            id: 0,
            team: 0,
            vx: 0.0,
            vy: 0.0,
            omega: 2.0,
            orientation: 0.0,
        };
        let (x, y) = command_to_xy(&cmd);
        assert_eq!(x, 30); // 2.0 * 15
        assert_eq!(y, 0);
    }

    #[test]
    fn xy_clamps_to_joystick_max() {
        let cmd = MotionCommand {
            id: 0,
            team: 0,
            vx: 5.0,
            vy: 0.0,
            omega: 100.0,
            orientation: 0.0,
        };
        assert_eq!(command_to_xy(&cmd), (100, 100));
    }

    #[test]
    fn frame_slots_by_id_and_filters_team() {
        let cmds = vec![
            make_cmd(0, 0, 0.5, 0.0, 0.0, 0.0), // own slot 0 → Y=100
            make_cmd(0, 2, 0.0, 0.0, 1.0, 0.0), // own slot 2 → X=15
            make_cmd(1, 1, 1.0, 0.0, 0.0, 0.0), // opp, ignorar
        ];
        let frame = build_frame(&cmds, TeamColor::Blue);
        assert_eq!(frame, "0,100,0,0,15,0,0,0,0,0\n");
    }

    #[test]
    fn frame_drops_id_out_of_range() {
        let cmds = vec![make_cmd(0, 9, 1.0, 0.0, 0.0, 0.0)];
        let frame = build_frame(&cmds, TeamColor::Blue);
        assert_eq!(frame, "0,0,0,0,0,0,0,0,0,0\n");
    }

    #[test]
    fn frame_yellow_team_filters_blue_out() {
        let cmds = vec![
            make_cmd(0, 0, 1.0, 0.0, 0.0, 0.0), // azul → ignorar
            make_cmd(1, 1, 0.5, 0.0, 0.0, 0.0), // amarillo → Y=100
        ];
        let frame = build_frame(&cmds, TeamColor::Yellow);
        assert_eq!(frame, "0,0,0,100,0,0,0,0,0,0\n");
    }
}
