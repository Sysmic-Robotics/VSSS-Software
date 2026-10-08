//! Director de partido para FIRASim: lo que el simulador no hace solo.
//!
//! FIRASim no repone la pelota tras un gol ni lleva el marcador. Este binario escucha la
//! visión del simulador, detecta los goles, repone pelota y robots con el replacement
//! FIRA y pita a los dos engines por el canal del árbitro (texto a `VSSL_REFEREE_ADDR`):
//! `STOP` → reposición → `KICKOFF <equipo>` → `GAME_ON`. Pelota trabada 10 s → `FREE_BALL`
//! en la cruz del cuadrante (un robot por equipo a 0.20 m del lado propio). Termina por
//! tiempo de juego o por goles y deja el resultado en la salida (y en `--out`, JSON).
//!
//!   match_director [--minutes 3] [--goals 3] [--kickoff blue|yellow] [--out partidos.jsonl]
//!                  [--vision-port 10002] [--cmd-port 20011]
//!
//! El tiempo de juego se cuenta en frames de visión (60 por segundo simulado), así vale
//! igual con FIRASim acelerado. Los lados se leen de la primera detección: cada equipo
//! defiende la mitad en la que está respecto del otro.

use std::collections::VecDeque;
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
/// Pelota trabada: no se movió más de `STUCK_DIST` en `STUCK_S` segundos de juego.
const STUCK_S: f64 = 10.0;
const STUCK_DIST: f64 = 0.05;
/// Cruces de free ball (|x|, |y|) y distancia del robot colocado a la cruz.
const MARK_X: f64 = 0.375;
const MARK_Y: f64 = 0.40;
const MARK_OFFSET: f64 = 0.20;

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

/// Robot visto: equipo, id y posición (m).
#[derive(Clone, Copy)]
struct Seen {
    team: Team,
    id: u32,
    x: f64,
    y: f64,
}

/// Instantánea de un frame de visión: pelota, robots y x medio de cada equipo.
struct Frame {
    ball: Option<(f64, f64)>,
    robots: Vec<Seen>,
    blue_mean_x: Option<f64>,
    yellow_mean_x: Option<f64>,
}

fn mean_x(robots: &[rustengine::protos::fira_common::Robot]) -> Option<f64> {
    (!robots.is_empty()).then(|| robots.iter().map(|r| r.x).sum::<f64>() / robots.len() as f64)
}

fn parse_frame(data: &[u8]) -> Option<Frame> {
    let env = Environment::parse_from_bytes(data).ok()?;
    let frame = env.frame.as_ref()?;
    let mut robots = Vec::with_capacity(6);
    robots.extend(frame.robots_blue.iter().map(|r| Seen { team: Team::Blue, id: r.robot_id, x: r.x, y: r.y }));
    robots.extend(frame.robots_yellow.iter().map(|r| Seen { team: Team::Yellow, id: r.robot_id, x: r.x, y: r.y }));
    Some(Frame {
        ball: frame.ball.as_ref().map(|b| (b.x, b.y)),
        robots,
        blue_mean_x: mean_x(&frame.robots_blue),
        yellow_mean_x: mean_x(&frame.robots_yellow),
    })
}

/// Free ball por pelota trabada en `ball`: cruz del cuadrante y colocación. El robot de
/// campo de cada equipo más cercano a la cruz va al punto a `MARK_OFFSET` del lado propio;
/// cualquier otro jugador de campo a menos de 0.35 m de la cruz se corre hacia su arco
/// (el arquero, id 2, se queda en su área).
fn free_ball_placement(ball: (f64, f64), robots: &[Seen], attack_sign: impl Fn(Team) -> f64) -> (u8, Vec<TeleportItem>) {
    let (qx, qy) = (ball.0.signum(), ball.1.signum());
    let quadrant = match (qx > 0.0, qy > 0.0) {
        (true, true) => 1,
        (false, true) => 2,
        (false, false) => 3,
        (true, false) => 4,
    };
    let mark = (qx * MARK_X, qy * MARK_Y);
    let mut items = vec![TeleportItem::Ball { x: mark.0, y: mark.1, vx: 0.0, vy: 0.0 }];
    for team in [Team::Blue, Team::Yellow] {
        let s = attack_sign(team);
        let theta = if s > 0.0 { 0.0 } else { 180.0 };
        let dist = |r: &Seen| ((r.x - mark.0).powi(2) + (r.y - mark.1).powi(2)).sqrt();
        let mut own: Vec<Seen> = robots.iter().copied().filter(|r| r.team == team).collect();
        // Arquero (id 2) al final: que coloque a un jugador de campo si lo hay.
        own.sort_by(|a, b| (a.id == 2, dist(a)).partial_cmp(&(b.id == 2, dist(b))).unwrap());
        for (i, r) in own.iter().enumerate() {
            if i == 0 {
                items.push(TeleportItem::Robot { team: team.fira_id(), id: r.id, x: mark.0 - s * MARK_OFFSET, y: mark.1, theta });
            } else if r.id != 2 && dist(r) < 0.35 {
                let x = (r.x - s * 0.35).clamp(-0.70, 0.70);
                items.push(TeleportItem::Robot { team: team.fira_id(), id: r.id, x, y: r.y, theta });
            }
        }
    }
    (quadrant, items)
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
    free_balls: u32,
    /// Posiciones de la pelota de los últimos `STUCK_S` segundos de juego.
    ball_trail: VecDeque<(f64, f64)>,
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

    /// Registra la pelota de este frame; `true` si lleva `STUCK_S` s sin moverse.
    fn ball_stuck(&mut self, ball: (f64, f64)) -> bool {
        let window = (STUCK_S * FRAMES_PER_S) as usize;
        self.ball_trail.push_back(ball);
        if self.ball_trail.len() <= window {
            return false;
        }
        self.ball_trail.pop_front();
        let oldest = self.ball_trail[0];
        ((ball.0 - oldest.0).powi(2) + (ball.1 - oldest.1).powi(2)).sqrt() < STUCK_DIST
    }
}

/// Descarta los frames que llegaron durante una pausa: son anteriores a la reposición
/// (la pelota todavía en el arco) y no son tiempo de juego.
fn drain(vision: &UdpSocket, buf: &mut [u8]) {
    while vision.try_recv(buf).is_ok() {}
}

/// Parada con reposición: `STOP` → replacement → comando de la play → `GAME_ON`.
async fn set_piece(m: &mut Match, command: &str, items: &[TeleportItem], whistle: &Whistle, sim: &mut FiraSimTransport, vision: &UdpSocket) {
    whistle.send("STOP");
    tokio::time::sleep(Duration::from_millis(800)).await;
    if let Err(e) = sim.teleport(items).await {
        eprintln!("[director] replacement falló: {e}");
    }
    tokio::time::sleep(Duration::from_millis(1000)).await;
    whistle.send(command);
    tokio::time::sleep(Duration::from_millis(1200)).await;
    let mut buf = [0u8; 65536];
    drain(vision, &mut buf);
    m.ball_trail.clear();
    whistle.send("GAME_ON");
}

async fn kickoff(m: &mut Match, kicker: Team, whistle: &Whistle, sim: &mut FiraSimTransport, vision: &UdpSocket) {
    let mut items = vec![TeleportItem::Ball { x: 0.0, y: 0.0, vx: 0.0, vy: 0.0 }];
    items.extend(kickoff_positions(kicker, m.attack_sign(kicker), true));
    items.extend(kickoff_positions(kicker.other(), m.attack_sign(kicker.other()), false));
    set_piece(m, &format!("KICKOFF {}", kicker.name()), &items, whistle, sim, vision).await;
    eprintln!("[director] {} saque {} — {}", m.clock(), kicker.spanish(), m.scoreline());
}

async fn free_ball(m: &mut Match, ball: (f64, f64), robots: &[Seen], whistle: &Whistle, sim: &mut FiraSimTransport, vision: &UdpSocket) {
    let (quadrant, items) = free_ball_placement(ball, robots, |t| m.attack_sign(t));
    m.free_balls += 1;
    eprintln!("[director] {} pelota trabada en ({:.2}, {:.2}) → free ball Q{quadrant}", m.clock(), ball.0, ball.1);
    set_piece(m, &format!("FREE_BALL Q{quadrant}"), &items, whistle, sim, vision).await;
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
    let mut m = Match {
        blue_attack_sign,
        score: [0, 0],
        frames: 0,
        goals: Vec::new(),
        free_balls: 0,
        ball_trail: VecDeque::new(),
    };
    eprintln!(
        "[director] azul ataca hacia {:+}X · {} min · corte a {} goles · árbitro {}",
        blue_attack_sign as i32, opt.minutes, opt.goals, whistle.addr
    );

    kickoff(&mut m, opt.kickoff, &whistle, &mut sim, &vision).await;
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
            kickoff(&mut m, scorer.other(), &whistle, &mut sim, &vision).await;
        } else if m.ball_stuck(ball) {
            free_ball(&mut m, ball, &f.robots, &whistle, &mut sim, &vision).await;
        }
    };

    whistle.send("HALT");
    println!(
        "RESULTADO {} · {} de juego · {} free balls · fin por {}",
        m.scoreline(),
        m.clock(),
        m.free_balls,
        ended
    );
    if let Some(path) = opt.out {
        let goals: Vec<String> = m
            .goals
            .iter()
            .map(|(t, team)| format!("{{\"t\":{t:.1},\"team\":\"{}\"}}", team.spanish()))
            .collect();
        let line = format!(
            "{{\"blue\":{},\"yellow\":{},\"seconds\":{:.1},\"ended\":\"{ended}\",\"free_balls\":{},\"goals\":[{}]}}",
            m.score[0],
            m.score[1],
            m.seconds(),
            m.free_balls,
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
        Match {
            blue_attack_sign,
            score: [0, 0],
            frames: 0,
            goals: Vec::new(),
            free_balls: 0,
            ball_trail: VecDeque::new(),
        }
    }

    #[test]
    fn stuck_ball_needs_ten_still_seconds() {
        let mut game = m(1.0);
        let still = (0.61, 0.63);
        for _ in 0..600 {
            assert!(!game.ball_stuck(still));
        }
        assert!(game.ball_stuck((0.62, 0.64)));
        // Se movió más de 5 cm en la ventana: no está trabada.
        let mut rolling = m(1.0);
        for i in 0..601 {
            let moved = rolling.ball_stuck((0.61 + 0.0002 * i as f64, 0.63));
            assert!(!moved, "frame {i}");
        }
    }

    #[test]
    fn free_ball_puts_one_field_robot_per_team_on_its_side() {
        // Pelota trabada en la esquina (+,+): cruz Q1 en (0.375, 0.40).
        let robots = [
            Seen { team: Team::Blue, id: 1, x: 0.66, y: 0.59 },
            Seen { team: Team::Blue, id: 0, x: 0.65, y: 0.43 },
            Seen { team: Team::Blue, id: 2, x: 0.60, y: 0.18 },
            Seen { team: Team::Yellow, id: 0, x: 0.54, y: 0.58 },
            Seen { team: Team::Yellow, id: 1, x: 0.47, y: 0.33 },
            Seen { team: Team::Yellow, id: 2, x: -0.65, y: 0.19 },
        ];
        // Azul ataca hacia -x (defiende +x), amarillo hacia +x.
        let (q, items) = free_ball_placement((0.61, 0.63), &robots, |t| if t == Team::Blue { -1.0 } else { 1.0 });
        assert_eq!(q, 1);
        let placed: Vec<(u32, u32, f64, f64)> = items
            .iter()
            .filter_map(|i| match *i {
                TeleportItem::Robot { team, id, x, y, .. } => Some((team, id, x, y)),
                _ => None,
            })
            .collect();
        let at = |team: u32, id: u32, x: f64, y: f64| {
            placed.iter().any(|&(t, i, px, py)| t == team && i == id && (px - x).abs() < 1e-6 && (py - y).abs() < 1e-6)
        };
        // El de campo más cercano a la cruz: azul 0 a 0.20 m hacia +x, amarillo 1 hacia -x.
        assert!(at(0, 0, 0.575, 0.40), "{placed:?}");
        assert!(at(1, 1, 0.175, 0.40), "{placed:?}");
        // Los otros de campo estaban a < 0.35 m: se corren hacia su arco (azul topa con el borde).
        assert!(at(0, 1, 0.70, 0.59), "{placed:?}");
        assert!(at(1, 0, 0.19, 0.58), "{placed:?}");
        // Los arqueros no se tocan aunque estén cerca de la cruz.
        assert!(!placed.iter().any(|&(_, id, _, _)| id == 2), "{placed:?}");
        assert!(matches!(items[0], TeleportItem::Ball { x, y, .. } if x == 0.375 && y == 0.40));
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
