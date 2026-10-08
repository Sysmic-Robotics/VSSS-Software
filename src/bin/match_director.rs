//! Director de partido para FIRASim: lo que el simulador no hace solo.
//!
//! FIRASim no repone la pelota tras un gol ni lleva el marcador. Este binario escucha la
//! visión del simulador, detecta los goles, repone pelota y robots con el replacement
//! FIRA y pita a los dos engines por el canal del árbitro (texto a `VSSL_REFEREE_ADDR`):
//! `STOP` → reposición → `KICKOFF <equipo>` → `GAME_ON`. Termina por tiempo de juego o
//! por diferencia de goles y deja el resultado en la salida (y en `--out`, una línea JSON).
//!
//!   match_director [--minutes 3] [--goals 3] [--kickoff blue|yellow] [--out partidos.jsonl]
//!                  [--vision-port 10002] [--cmd-port 20011]
//!
//! El tiempo de juego se cuenta en frames de visión (60 por segundo simulado), así vale
//! igual con FIRASim acelerado. Los lados se leen de la primera detección: cada equipo
//! defiende la mitad en la que está respecto del otro.

use std::net::{Ipv4Addr, SocketAddr, UdpSocket as StdUdpSocket};
use std::time::Duration;

use protobuf::Message;
use rustengine::coach::referee::{REFEREE_ADDR_DEFAULT, REFEREE_ADDR_ENV};
use rustengine::protos::fira_packet::Environment;
use rustengine::radio::{FiraSimTransport, RobotTransport, TeleportItem};
use tokio::net::UdpSocket;

/// Línea de gol (|x|) y media boca del arco (|y|) del campo VSSS (m).
const GOAL_LINE_X: f64 = 0.75;
const GOAL_HALF_Y: f64 = 0.20;
/// Radio de la pelota: el gol es con la pelota entera pasada la línea.
const BALL_RADIUS: f64 = 0.021;
/// Frames de visión por segundo de simulación.
const FRAMES_PER_S: f64 = 60.0;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Team {
    Blue,
    Yellow,
}

impl Team {
    fn name(self) -> &'static str {
        match self {
            Team::Blue => "BLUE",
            Team::Yellow => "YELLOW",
        }
    }
    fn spanish(self) -> &'static str {
        match self {
            Team::Blue => "azul",
            Team::Yellow => "amarillo",
        }
    }
    fn other(self) -> Team {
        match self {
            Team::Blue => Team::Yellow,
            Team::Yellow => Team::Blue,
        }
    }
    fn fira_id(self) -> u32 {
        match self {
            Team::Blue => 0,
            Team::Yellow => 1,
        }
    }
}

struct Options {
    minutes: f64,
    goals: u32,
    kickoff: Team,
    out: Option<String>,
    vision_port: u16,
    cmd_port: u16,
}

fn parse_args() -> Options {
    let mut o = Options {
        minutes: 3.0,
        goals: 3,
        kickoff: Team::Blue,
        out: None,
        vision_port: 10002,
        cmd_port: 20011,
    };
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let value = |args: &mut std::iter::Skip<std::env::Args>| {
            args.next().unwrap_or_else(|| {
                eprintln!("[director] falta el valor de {a}");
                std::process::exit(2);
            })
        };
        match a.as_str() {
            "--minutes" => o.minutes = value(&mut args).parse().expect("--minutes: número"),
            "--goals" => o.goals = value(&mut args).parse().expect("--goals: entero"),
            "--kickoff" => {
                o.kickoff = match value(&mut args).to_ascii_lowercase().as_str() {
                    "blue" | "azul" => Team::Blue,
                    "yellow" | "amarillo" => Team::Yellow,
                    other => {
                        eprintln!("[director] --kickoff blue|yellow (no '{other}')");
                        std::process::exit(2);
                    }
                }
            }
            "--out" => o.out = Some(value(&mut args)),
            "--vision-port" => o.vision_port = value(&mut args).parse().expect("--vision-port: entero"),
            "--cmd-port" => o.cmd_port = value(&mut args).parse().expect("--cmd-port: entero"),
            other => {
                eprintln!("[director] opción desconocida: {other}");
                std::process::exit(2);
            }
        }
    }
    o
}

/// Socket de visión compartido con los engines (mismo puerto multicast, `SO_REUSEADDR`).
fn bind_vision(port: u16) -> Result<UdpSocket, Box<dyn std::error::Error>> {
    use socket2::{Domain, Protocol, Socket, Type};
    let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
    socket.set_reuse_address(true)?;
    #[cfg(unix)]
    socket.set_reuse_port(true)?;
    socket.set_nonblocking(true)?;
    let addr: SocketAddr = format!("0.0.0.0:{port}").parse()?;
    socket.bind(&addr.into())?;
    let std_socket: StdUdpSocket = socket.into();
    std_socket.join_multicast_v4(&Ipv4Addr::new(224, 0, 0, 1), &Ipv4Addr::UNSPECIFIED)?;
    Ok(UdpSocket::from_std(std_socket)?)
}

/// Silbato: texto al mismo grupo/puerto que escuchan los engines.
struct Whistle {
    sock: StdUdpSocket,
    addr: SocketAddr,
}

impl Whistle {
    fn new() -> Result<Self, Box<dyn std::error::Error>> {
        let text = std::env::var(REFEREE_ADDR_ENV).ok().filter(|s| !s.trim().is_empty());
        let addr: SocketAddr = text.as_deref().unwrap_or(REFEREE_ADDR_DEFAULT).trim().parse()?;
        let sock = StdUdpSocket::bind("0.0.0.0:0")?;
        sock.set_multicast_loop_v4(true)?;
        Ok(Self { sock, addr })
    }

    fn send(&self, text: &str) {
        if let Err(e) = self.sock.send_to(text.as_bytes(), self.addr) {
            eprintln!("[director] no pude enviar {text} al árbitro ({}): {e}", self.addr);
        }
    }
}

/// Instantánea de un frame de visión: pelota y x medio de cada equipo.
struct Frame {
    ball: Option<(f64, f64)>,
    blue_mean_x: Option<f64>,
    yellow_mean_x: Option<f64>,
}

fn mean_x(robots: &[rustengine::protos::fira_common::Robot]) -> Option<f64> {
    (!robots.is_empty()).then(|| robots.iter().map(|r| r.x).sum::<f64>() / robots.len() as f64)
}

fn parse_frame(data: &[u8]) -> Option<Frame> {
    let env = Environment::parse_from_bytes(data).ok()?;
    let frame = env.frame.as_ref()?;
    Some(Frame {
        ball: frame.ball.as_ref().map(|b| (b.x, b.y)),
        blue_mean_x: mean_x(&frame.robots_blue),
        yellow_mean_x: mean_x(&frame.robots_yellow),
    })
}

/// Formación de saque inicial de un equipo que ataca hacia `s` (+1 → +x): arquero en
/// su línea, pateador dentro del círculo central detrás de la pelota si saca él (si no,
/// fuera del círculo), apoyo en su mitad.
fn kickoff_positions(team: Team, s: f64, kicks: bool) -> Vec<TeleportItem> {
    let theta = if s > 0.0 { 0.0 } else { 180.0 };
    let t = team.fira_id();
    let striker_x = if kicks { 0.10 } else { 0.28 };
    let support_y = if kicks { 0.30 } else { -0.30 };
    vec![
        TeleportItem::Robot { team: t, id: 0, x: -s * striker_x, y: 0.0, theta },
        TeleportItem::Robot { team: t, id: 1, x: -s * 0.38, y: support_y, theta },
        TeleportItem::Robot { team: t, id: 2, x: -s * 0.68, y: 0.0, theta: 90.0 },
    ]
}

struct Match {
    blue_attack_sign: f64,
    score: [u32; 2],
    frames: u64,
    goals: Vec<(f64, Team)>,
}

impl Match {
    fn seconds(&self) -> f64 {
        self.frames as f64 / FRAMES_PER_S
    }

    fn clock(&self) -> String {
        let s = self.seconds() as u64;
        format!("{}:{:02}", s / 60, s % 60)
    }

    fn scoreline(&self) -> String {
        format!("azul {} - {} amarillo", self.score[0], self.score[1])
    }

    /// Equipo que convirtió si la pelota entera cruzó una línea de gol por la boca.
    fn goal_scored(&self, ball: (f64, f64)) -> Option<Team> {
        let (x, y) = ball;
        if y.abs() > GOAL_HALF_Y || x.abs() < GOAL_LINE_X + BALL_RADIUS {
            return None;
        }
        // Gol en el arco de +x: lo convierte el que ataca hacia +x.
        let scorer_attacks_plus = x > 0.0;
        Some(if (self.blue_attack_sign > 0.0) == scorer_attacks_plus { Team::Blue } else { Team::Yellow })
    }

    fn attack_sign(&self, team: Team) -> f64 {
        match team {
            Team::Blue => self.blue_attack_sign,
            Team::Yellow => -self.blue_attack_sign,
        }
    }
}

/// Descarta los frames que llegaron durante una pausa: son anteriores a la reposición
/// (la pelota todavía en el arco) y no son tiempo de juego.
fn drain(vision: &UdpSocket, buf: &mut [u8]) {
    while vision.try_recv(buf).is_ok() {}
}

async fn kickoff(m: &Match, kicker: Team, whistle: &Whistle, sim: &mut FiraSimTransport, vision: &UdpSocket) {
    whistle.send("STOP");
    tokio::time::sleep(Duration::from_millis(800)).await;
    let mut items = vec![TeleportItem::Ball { x: 0.0, y: 0.0, vx: 0.0, vy: 0.0 }];
    items.extend(kickoff_positions(kicker, m.attack_sign(kicker), true));
    items.extend(kickoff_positions(kicker.other(), m.attack_sign(kicker.other()), false));
    if let Err(e) = sim.teleport(&items).await {
        eprintln!("[director] replacement falló: {e}");
    }
    tokio::time::sleep(Duration::from_millis(1000)).await;
    whistle.send(&format!("KICKOFF {}", kicker.name()));
    tokio::time::sleep(Duration::from_millis(1200)).await;
    let mut buf = [0u8; 65536];
    drain(vision, &mut buf);
    whistle.send("GAME_ON");
    eprintln!("[director] {} saque {} — {}", m.clock(), kicker.spanish(), m.scoreline());
}

#[tokio::main]
async fn main() {
    let opt = parse_args();
    let vision = bind_vision(opt.vision_port).unwrap_or_else(|e| {
        eprintln!("[director] visión {}: {e}", opt.vision_port);
        std::process::exit(1);
    });
    let whistle = Whistle::new().unwrap_or_else(|e| {
        eprintln!("[director] árbitro: {e}");
        std::process::exit(1);
    });
    let mut sim = FiraSimTransport::new("127.0.0.1", opt.cmd_port).await.unwrap_or_else(|e| {
        eprintln!("[director] FIRASim {}: {e}", opt.cmd_port);
        std::process::exit(1);
    });

    // Lados: cada equipo defiende la mitad en la que está respecto del otro.
    let mut buf = [0u8; 65536];
    let blue_attack_sign = loop {
        let n = match tokio::time::timeout(Duration::from_secs(10), vision.recv(&mut buf)).await {
            Ok(Ok(n)) => n,
            Ok(Err(e)) => {
                eprintln!("[director] visión: {e}");
                std::process::exit(1);
            }
            Err(_) => {
                eprintln!("[director] sin visión de FIRASim en 10 s (puerto {})", opt.vision_port);
                std::process::exit(1);
            }
        };
        if let Some(f) = parse_frame(&buf[..n])
            && let (Some(blue), Some(yellow)) = (f.blue_mean_x, f.yellow_mean_x)
            && (blue - yellow).abs() > 0.10
        {
            break if blue > yellow { -1.0 } else { 1.0 };
        }
    };
    let mut m = Match { blue_attack_sign, score: [0, 0], frames: 0, goals: Vec::new() };
    eprintln!(
        "[director] azul ataca hacia {:+}X · {} min · corte a {} goles · árbitro {}",
        blue_attack_sign as i32, opt.minutes, opt.goals, whistle.addr
    );

    kickoff(&m, opt.kickoff, &whistle, &mut sim, &vision).await;
    let limit_frames = (opt.minutes * 60.0 * FRAMES_PER_S) as u64;
    let mut next_report = 30.0;
    let ended = loop {
        let n = match tokio::time::timeout(Duration::from_secs(5), vision.recv(&mut buf)).await {
            Ok(Ok(n)) => n,
            Ok(Err(e)) => {
                eprintln!("[director] visión: {e}");
                break "vision";
            }
            Err(_) => {
                eprintln!("[director] FIRASim dejó de emitir visión");
                break "vision";
            }
        };
        let Some(f) = parse_frame(&buf[..n]) else { continue };
        m.frames += 1;
        if m.seconds() >= next_report {
            eprintln!("[director] {} {}", m.clock(), m.scoreline());
            next_report += 30.0;
        }
        if m.frames >= limit_frames {
            break "time";
        }
        let Some(ball) = f.ball else { continue };
        if let Some(scorer) = m.goal_scored(ball) {
            m.score[scorer.fira_id() as usize] += 1;
            m.goals.push((m.seconds(), scorer));
            eprintln!("[director] {} ¡GOL {}! {}", m.clock(), scorer.spanish(), m.scoreline());
            if m.score[scorer.fira_id() as usize] >= opt.goals {
                break "goals";
            }
            kickoff(&m, scorer.other(), &whistle, &mut sim, &vision).await;
        }
    };

    whistle.send("HALT");
    println!("RESULTADO {} · {} de juego · fin por {}", m.scoreline(), m.clock(), ended);
    if let Some(path) = opt.out {
        let goals: Vec<String> = m
            .goals
            .iter()
            .map(|(t, team)| format!("{{\"t\":{t:.1},\"team\":\"{}\"}}", team.spanish()))
            .collect();
        let line = format!(
            "{{\"blue\":{},\"yellow\":{},\"seconds\":{:.1},\"ended\":\"{ended}\",\"goals\":[{}]}}",
            m.score[0],
            m.score[1],
            m.seconds(),
            goals.join(",")
        );
        use std::io::Write;
        match std::fs::OpenOptions::new().create(true).append(true).open(&path) {
            Ok(mut f) => {
                let _ = writeln!(f, "{line}");
            }
            Err(e) => eprintln!("[director] no pude escribir {path}: {e}"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn m(blue_attack_sign: f64) -> Match {
        Match { blue_attack_sign, score: [0, 0], frames: 0, goals: Vec::new() }
    }

    #[test]
    fn goal_needs_whole_ball_past_the_line_inside_the_mouth() {
        let m = m(1.0);
        assert_eq!(m.goal_scored((0.76, 0.0)), None);
        assert_eq!(m.goal_scored((0.78, 0.25)), None);
        assert_eq!(m.goal_scored((0.78, 0.1)), Some(Team::Blue));
        assert_eq!(m.goal_scored((-0.78, -0.1)), Some(Team::Yellow));
    }

    #[test]
    fn sides_follow_where_blue_starts() {
        let m = m(-1.0);
        assert_eq!(m.goal_scored((0.78, 0.0)), Some(Team::Yellow));
        assert_eq!(m.goal_scored((-0.78, 0.0)), Some(Team::Blue));
    }

    #[test]
    fn kickoff_formation_is_legal() {
        for s in [1.0, -1.0] {
            for kicks in [true, false] {
                for item in kickoff_positions(Team::Blue, s, kicks) {
                    let TeleportItem::Robot { id, x, y, .. } = item else { panic!() };
                    assert!(x * s < 0.0, "en su mitad: id {id} x {x}");
                    let in_circle = (x * x + y * y).sqrt() < 0.20;
                    assert_eq!(in_circle, kicks && id == 0, "círculo central: id {id}");
                }
            }
        }
    }
}
