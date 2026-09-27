//! Estado de juego del árbitro (LARC VSSS §10) para la capa de plays.
//!
//! Fuentes, por el mismo socket UDP (`VSSL_REFEREE_ADDR`, default
//! `224.5.23.2:10003` = VSSReferee de RoboCIn):
//! - **VSSReferee** (simulador): datagramas protobuf `VSSRef_Command`
//!   (`foul=1, teamcolor=2, foulQuadrant=3, timestamp=4, gameHalf=5`). Se parsea a
//!   mano (proto3, 5 campos) para no agregar archivos `.proto` al engine.
//! - **Operador** (cancha real, árbitro humano): datagramas de TEXTO, p. ej.
//!   `KICKOFF BLUE`, `FREE_BALL Q1`, `PENALTY YELLOW`, `GAME_ON`, `STOP`, `HALT`
//!   (`tools/referee_cli.py`).
//!
//! El listener corre en una tarea aparte y deja el último comando en un
//! `SharedReferee`; el coach lo lee en cada decisión (sin bloquear el loop).

use std::sync::{Arc, Mutex};
use std::time::Instant;

/// Tipo de jugada señalada (misma numeración que el enum `Foul` de VSSReferee).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Foul {
    FreeKick = 0,
    PenaltyKick = 1,
    GoalKick = 2,
    FreeBall = 3,
    Kickoff = 4,
    Stop = 5,
    GameOn = 6,
    Halt = 7,
}

impl Foul {
    pub fn from_i32(v: i32) -> Option<Self> {
        Some(match v {
            0 => Self::FreeKick,
            1 => Self::PenaltyKick,
            2 => Self::GoalKick,
            3 => Self::FreeBall,
            4 => Self::Kickoff,
            5 => Self::Stop,
            6 => Self::GameOn,
            7 => Self::Halt,
            _ => return None,
        })
    }

    pub fn is_set_piece(self) -> bool {
        matches!(
            self,
            Self::FreeKick | Self::PenaltyKick | Self::GoalKick | Self::FreeBall | Self::Kickoff
        )
    }
}

/// Equipo al que se le concede la jugada (enum `Color` de VSSReferee).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RefTeam {
    Blue = 0,
    Yellow = 1,
    None = 2,
}

impl RefTeam {
    pub fn from_i32(v: i32) -> Option<Self> {
        Some(match v {
            0 => Self::Blue,
            1 => Self::Yellow,
            2 => Self::None,
            _ => return None,
        })
    }

    /// `team_id` del engine (0 azul, 1 amarillo).
    pub fn team_id(self) -> Option<i32> {
        match self {
            Self::Blue => Some(0),
            Self::Yellow => Some(1),
            Self::None => None,
        }
    }
}

/// Cuadrante de un Free Ball (enum `Quadrant` de VSSReferee; convención
/// matemática: Q1 = +x +y, Q2 = −x +y, Q3 = −x −y, Q4 = +x −y).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Quadrant {
    None = 0,
    Q1 = 1,
    Q2 = 2,
    Q3 = 3,
    Q4 = 4,
}

impl Quadrant {
    pub fn from_i32(v: i32) -> Option<Self> {
        Some(match v {
            0 => Self::None,
            1 => Self::Q1,
            2 => Self::Q2,
            3 => Self::Q3,
            4 => Self::Q4,
            _ => return None,
        })
    }

    /// Signos (x, y) del cuadrante; `None` → (0, 0) (pelota al centro).
    pub fn signs(self) -> (f32, f32) {
        match self {
            Self::None => (0.0, 0.0),
            Self::Q1 => (1.0, 1.0),
            Self::Q2 => (-1.0, 1.0),
            Self::Q3 => (-1.0, -1.0),
            Self::Q4 => (1.0, -1.0),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RefereeCommand {
    pub foul: Foul,
    pub team: RefTeam,
    pub quadrant: Quadrant,
    pub timestamp: f64,
    pub half: i32,
}

impl RefereeCommand {
    pub const GAME_ON: Self = Self {
        foul: Foul::GameOn,
        team: RefTeam::None,
        quadrant: Quadrant::None,
        timestamp: 0.0,
        half: 0,
    };
}

// ─────────────────────────────────────────────────────────────────────────────
//  Parsers
// ─────────────────────────────────────────────────────────────────────────────

fn read_varint(b: &[u8], i: &mut usize) -> Option<u64> {
    let mut r = 0u64;
    let mut shift = 0;
    loop {
        let c = *b.get(*i)?;
        *i += 1;
        r |= ((c & 0x7f) as u64) << shift;
        if c < 0x80 {
            return Some(r);
        }
        shift += 7;
        if shift > 63 {
            return None;
        }
    }
}

/// Parsea un `VSSRef_Command` (proto3). Campos desconocidos se saltan.
pub fn parse_vssref_command(bytes: &[u8]) -> Option<RefereeCommand> {
    let mut cmd = RefereeCommand::GAME_ON;
    let mut seen_foul = false;
    let mut i = 0;
    while i < bytes.len() {
        let tag = read_varint(bytes, &mut i)?;
        let (field, wire) = (tag >> 3, tag & 7);
        match wire {
            0 => {
                let v = read_varint(bytes, &mut i)? as i32;
                match field {
                    1 => {
                        cmd.foul = Foul::from_i32(v)?;
                        seen_foul = true;
                    }
                    2 => cmd.team = RefTeam::from_i32(v)?,
                    3 => cmd.quadrant = Quadrant::from_i32(v)?,
                    5 => cmd.half = v,
                    _ => {}
                }
            }
            1 => {
                let raw = bytes.get(i..i + 8)?;
                i += 8;
                if field == 4 {
                    cmd.timestamp = f64::from_le_bytes(raw.try_into().ok()?);
                }
            }
            2 => {
                let len = read_varint(bytes, &mut i)? as usize;
                i = i.checked_add(len)?;
                if i > bytes.len() {
                    return None;
                }
            }
            5 => i += 4,
            _ => return None,
        }
    }
    // proto3 omite el campo 1 cuando vale 0 (FREE_KICK): un mensaje sin foul
    // explícito con otros campos presentes sigue siendo válido.
    if !seen_foul && bytes.is_empty() {
        return None;
    }
    if !seen_foul {
        cmd.foul = Foul::FreeKick;
    }
    Some(cmd)
}

/// Comando de texto del operador: `<JUGADA> [BLUE|YELLOW] [Q1..Q4]`, sin importar
/// mayúsculas ni orden de los argumentos. Jugadas: KICKOFF, FREE_KICK (FREEKICK),
/// PENALTY, GOAL_KICK (GOALKICK), FREE_BALL (FREEBALL), GAME_ON (GO, PLAY),
/// STOP, HALT.
pub fn parse_text_command(text: &str) -> Option<RefereeCommand> {
    let mut words = text.split_whitespace().map(|w| w.to_ascii_uppercase());
    let foul = match words.next()?.as_str() {
        "KICKOFF" | "KICK_OFF" => Foul::Kickoff,
        "FREE_KICK" | "FREEKICK" => Foul::FreeKick,
        "PENALTY" | "PENALTY_KICK" => Foul::PenaltyKick,
        "GOAL_KICK" | "GOALKICK" => Foul::GoalKick,
        "FREE_BALL" | "FREEBALL" => Foul::FreeBall,
        "GAME_ON" | "GAMEON" | "GO" | "PLAY" => Foul::GameOn,
        "STOP" => Foul::Stop,
        "HALT" => Foul::Halt,
        _ => return None,
    };
    let mut cmd = RefereeCommand {
        foul,
        ..RefereeCommand::GAME_ON
    };
    for w in words {
        match w.as_str() {
            "BLUE" | "AZUL" => cmd.team = RefTeam::Blue,
            "YELLOW" | "AMARILLO" => cmd.team = RefTeam::Yellow,
            "Q1" => cmd.quadrant = Quadrant::Q1,
            "Q2" => cmd.quadrant = Quadrant::Q2,
            "Q3" => cmd.quadrant = Quadrant::Q3,
            "Q4" => cmd.quadrant = Quadrant::Q4,
            _ => {}
        }
    }
    Some(cmd)
}

/// Texto si todos los bytes son ASCII imprimibles/espacios; si no, protobuf.
pub fn parse_datagram(bytes: &[u8]) -> Option<RefereeCommand> {
    let looks_text = !bytes.is_empty()
        && bytes
            .iter()
            .all(|b| b.is_ascii_alphanumeric() || b.is_ascii_whitespace() || *b == b'_');
    if looks_text {
        return parse_text_command(std::str::from_utf8(bytes).ok()?);
    }
    parse_vssref_command(bytes)
}

// ─────────────────────────────────────────────────────────────────────────────
//  Estado compartido + listener
// ─────────────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub struct RefereeState {
    pub command: RefereeCommand,
    pub received_at: Instant,
    /// Contador de comandos recibidos (cambia aunque se repita el mismo comando).
    pub seq: u64,
}

impl Default for RefereeState {
    fn default() -> Self {
        Self {
            command: RefereeCommand::GAME_ON,
            received_at: Instant::now(),
            seq: 0,
        }
    }
}

pub type SharedReferee = Arc<Mutex<RefereeState>>;

pub fn new_shared_referee() -> SharedReferee {
    Arc::new(Mutex::new(RefereeState::default()))
}

/// Aplica un comando al estado compartido (lo usan el listener y los tests).
pub fn apply_command(shared: &SharedReferee, command: RefereeCommand) {
    if let Ok(mut s) = shared.lock() {
        s.command = command;
        s.received_at = Instant::now();
        s.seq += 1;
    }
}

pub const REFEREE_ADDR_ENV: &str = "VSSL_REFEREE_ADDR";
pub const REFEREE_ADDR_DEFAULT: &str = "224.5.23.2:10003";

/// Escucha comandos de árbitro (VSSReferee o texto del operador) y los deja en
/// `shared`. Corre hasta que el runtime termina.
pub async fn run_referee_listener(shared: SharedReferee) {
    use socket2::{Domain, Protocol, Socket, Type};
    let addr_text = std::env::var(REFEREE_ADDR_ENV)
        .ok()
        .filter(|s| !s.trim().is_empty())
        .unwrap_or_else(|| REFEREE_ADDR_DEFAULT.to_string());
    let addr: std::net::SocketAddrV4 = match addr_text.trim().parse() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("[referee] {REFEREE_ADDR_ENV}='{addr_text}' inválido ({e}); árbitro desactivado");
            return;
        }
    };
    let socket = (|| -> std::io::Result<tokio::net::UdpSocket> {
        let s = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
        s.set_reuse_address(true)?;
        #[cfg(unix)]
        s.set_reuse_port(true)?;
        s.set_nonblocking(true)?;
        let bind: std::net::SocketAddr =
            std::net::SocketAddrV4::new(std::net::Ipv4Addr::UNSPECIFIED, addr.port()).into();
        s.bind(&bind.into())?;
        let std_socket: std::net::UdpSocket = s.into();
        if addr.ip().is_multicast() {
            std_socket.join_multicast_v4(addr.ip(), &std::net::Ipv4Addr::UNSPECIFIED)?;
        }
        tokio::net::UdpSocket::from_std(std_socket)
    })();
    let socket = match socket {
        Ok(s) => s,
        Err(e) => {
            eprintln!("[referee] no se pudo abrir {addr}: {e}; árbitro desactivado");
            return;
        }
    };
    eprintln!("[referee] escuchando en {addr} (VSSReferee o texto del operador)");
    let mut buf = [0u8; 2048];
    loop {
        let Ok((len, from)) = socket.recv_from(&mut buf).await else {
            continue;
        };
        match parse_datagram(&buf[..len]) {
            Some(cmd) => {
                let changed = shared
                    .lock()
                    .map(|s| s.command != cmd)
                    .unwrap_or(true);
                apply_command(&shared, cmd);
                if changed {
                    eprintln!(
                        "[referee] {:?} equipo {:?} cuadrante {:?} (de {from})",
                        cmd.foul, cmd.team, cmd.quadrant
                    );
                }
            }
            None => eprintln!("[referee] datagrama no reconocido de {from} ({len} B)"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn varint(mut v: u64) -> Vec<u8> {
        let mut out = Vec::new();
        loop {
            let b = (v & 0x7f) as u8;
            v >>= 7;
            if v == 0 {
                out.push(b);
                return out;
            }
            out.push(b | 0x80);
        }
    }

    fn encode(foul: i32, team: i32, quadrant: i32, ts: f64, half: i32) -> Vec<u8> {
        let mut b = Vec::new();
        // proto3: los campos con valor 0 se omiten (como haría protobuf real).
        if foul != 0 {
            b.push(0x08);
            b.extend(varint(foul as u64));
        }
        if team != 0 {
            b.push(0x10);
            b.extend(varint(team as u64));
        }
        if quadrant != 0 {
            b.push(0x18);
            b.extend(varint(quadrant as u64));
        }
        if ts != 0.0 {
            b.push(0x21);
            b.extend(ts.to_le_bytes());
        }
        if half != 0 {
            b.push(0x28);
            b.extend(varint(half as u64));
        }
        b
    }

    #[test]
    fn parses_vssref_commands() {
        let c = parse_vssref_command(&encode(4, 1, 0, 12.5, 1)).unwrap();
        assert_eq!(c.foul, Foul::Kickoff);
        assert_eq!(c.team, RefTeam::Yellow);
        assert_eq!(c.quadrant, Quadrant::None);
        assert!((c.timestamp - 12.5).abs() < 1e-12);
        assert_eq!(c.half, 1);

        let c = parse_vssref_command(&encode(3, 2, 4, 0.0, 0)).unwrap();
        assert_eq!(c.foul, Foul::FreeBall);
        assert_eq!(c.team, RefTeam::None);
        assert_eq!(c.quadrant, Quadrant::Q4);

        // FREE_KICK = 0 se omite en proto3: solo viene el equipo.
        let c = parse_vssref_command(&encode(0, 1, 0, 0.0, 0)).unwrap();
        assert_eq!(c.foul, Foul::FreeKick);
        assert_eq!(c.team, RefTeam::Yellow);

        assert!(parse_vssref_command(&[0x08, 0x2a]).is_none()); // foul 42 inválido
    }

    #[test]
    fn parses_operator_text() {
        let c = parse_text_command("kickoff blue").unwrap();
        assert_eq!((c.foul, c.team), (Foul::Kickoff, RefTeam::Blue));
        let c = parse_text_command("FREE_BALL Q3").unwrap();
        assert_eq!((c.foul, c.quadrant), (Foul::FreeBall, Quadrant::Q3));
        let c = parse_text_command("penalty amarillo").unwrap();
        assert_eq!((c.foul, c.team), (Foul::PenaltyKick, RefTeam::Yellow));
        assert_eq!(parse_text_command("go").unwrap().foul, Foul::GameOn);
        assert_eq!(parse_text_command("HALT").unwrap().foul, Foul::Halt);
        assert!(parse_text_command("bailar").is_none());
        assert_eq!(parse_datagram(b"STOP").unwrap().foul, Foul::Stop);
        assert_eq!(parse_datagram(&encode(7, 0, 0, 0.0, 0)).unwrap().foul, Foul::Halt);
    }

    #[test]
    fn shared_state_counts_commands() {
        let shared = new_shared_referee();
        assert_eq!(shared.lock().unwrap().seq, 0);
        apply_command(&shared, parse_text_command("STOP").unwrap());
        apply_command(&shared, parse_text_command("STOP").unwrap());
        let s = shared.lock().unwrap();
        assert_eq!(s.seq, 2);
        assert_eq!(s.command.foul, Foul::Stop);
    }
}
