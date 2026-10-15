//! Comprueba la visión antes de abrir el engine (lo usa `tools/iniciar.sh`).
//!
//! Escucha unos segundos el multicast de la fuente con la misma configuración que el
//! engine (`VSSL_VISION_SOURCE`, `VSSL_VISION_PORT`, `VSSL_MULTICAST_IFACE`) y el mismo
//! parser (`parse_detections`), y dice en frases si llegan paquetes y qué robots ve.
//! El equipo propio es el de `VSSL_TEAM_COLOR`. No manda nada a los robots.
//!
//!   comprobar_vision [--segundos 2]
//!
//! Código de salida: 0 llegan paquetes y se ve al menos un robot propio; 3 no llega
//! ningún paquete; 4 llegan, pero no se ve ningún robot propio; 1 error.

use std::collections::{BTreeMap, BTreeSet};
use std::net::{Ipv4Addr, SocketAddr, UdpSocket};
use std::time::{Duration, Instant};

use rustengine::radio::TeamColor;
use rustengine::vision::VisionSource;
use rustengine::vision_tools::{Detection, parse_detections};

const HELP: &str = "comprobar_vision — ¿llega la visión y ve a nuestros robots?

USO:
    comprobar_vision [--segundos S]

    --segundos S   cuánto escuchar (default 2).

Usa la fuente, el puerto y la interfaz del engine (VSSL_VISION_SOURCE,
VSSL_VISION_PORT, VSSL_MULTICAST_IFACE) y el equipo de VSSL_TEAM_COLOR.
Código de salida: 0 bien; 3 no llegan paquetes; 4 no ve robots propios; 1 error.
";

/// Lo que se vio en la ventana de escucha.
#[derive(Default)]
struct Escucha {
    paquetes: u64,
    /// Paquetes en que apareció cada robot, por (equipo, id).
    robots: BTreeMap<(i32, i32), u64>,
    /// Paquetes en que apareció la pelota.
    pelota: u64,
}

impl Escucha {
    /// Anota un paquete. Un robot repetido en el mismo paquete cuenta una vez.
    fn anotar(&mut self, detecciones: &[Detection]) {
        self.paquetes += 1;
        let robots: BTreeSet<(i32, i32)> = detecciones
            .iter()
            .filter(|d| !d.is_ball())
            .map(|d| (d.team, d.id))
            .collect();
        for r in robots {
            *self.robots.entry(r).or_default() += 1;
        }
        if detecciones.iter().any(Detection::is_ball) {
            self.pelota += 1;
        }
    }
}

fn nombre_equipo(team: i32) -> &'static str {
    if team == 0 { "azules" } else { "amarillos" }
}

/// Frases del resultado y código de salida.
fn informe(e: &Escucha, segundos: f64, direccion: &str, propio: i32) -> (Vec<String>, i32) {
    if e.paquetes == 0 {
        return (
            vec![format!(
                "✗ No llega ningún paquete de visión a {direccion} en {segundos} s."
            )],
            3,
        );
    }
    let pct = |n: u64| 100 * n / e.paquetes;
    let lista = |team: i32| -> Vec<String> {
        e.robots
            .iter()
            .filter(|((t, _), _)| *t == team)
            .map(|((_, id), n)| format!("#{id} ({} %)", pct(*n)))
            .collect()
    };
    let (propios, rivales) = (lista(propio), lista(1 - propio));
    let mut lineas = vec![format!(
        "✓ Llegan paquetes de visión a {direccion} ({:.0} por segundo).",
        e.paquetes as f64 / segundos
    )];
    let codigo = if propios.is_empty() {
        lineas.push(format!(
            "✗ La visión no ve robots {}.",
            nombre_equipo(propio)
        ));
        4
    } else {
        lineas.push(format!(
            "✓ La visión ve robots {}: {}.",
            nombre_equipo(propio),
            propios.join(", ")
        ));
        0
    };
    if !rivales.is_empty() {
        lineas.push(format!(
            "  También ve robots {}: {}.",
            nombre_equipo(1 - propio),
            rivales.join(", ")
        ));
    }
    lineas.push(if e.pelota > 0 {
        format!("  Ve la pelota ({} %).", pct(e.pelota))
    } else {
        "  No ve la pelota.".to_string()
    });
    if !e.robots.is_empty() {
        lineas.push("  El % es la parte de los cuadros en que se vio.".to_string());
    }
    (lineas, codigo)
}

/// Socket en el puerto de la fuente, compartido con un engine que ya esté escuchando.
fn escuchar(fuente: VisionSource) -> Result<UdpSocket, String> {
    use socket2::{Domain, Protocol, Socket, Type};
    let interfaz = match std::env::var("VSSL_MULTICAST_IFACE") {
        Ok(v) => v
            .trim()
            .parse::<Ipv4Addr>()
            .map_err(|_| format!("VSSL_MULTICAST_IFACE='{v}' no es una IPv4."))?,
        Err(_) => Ipv4Addr::UNSPECIFIED,
    };
    let grupo: Ipv4Addr = fuente.multicast_ip().parse().map_err(|e| format!("{e}"))?;
    let socket =
        Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP)).map_err(|e| e.to_string())?;
    socket.set_reuse_address(true).map_err(|e| e.to_string())?;
    #[cfg(unix)]
    socket.set_reuse_port(true).map_err(|e| e.to_string())?;
    let addr: SocketAddr = SocketAddr::from((Ipv4Addr::UNSPECIFIED, fuente.port()));
    socket.bind(&addr.into()).map_err(|e| e.to_string())?;
    let socket: UdpSocket = socket.into();
    socket
        .join_multicast_v4(&grupo, &interfaz)
        .map_err(|e| e.to_string())?;
    socket
        .set_read_timeout(Some(Duration::from_millis(100)))
        .map_err(|e| e.to_string())?;
    Ok(socket)
}

fn segundos_de_args() -> Result<f64, String> {
    let mut segundos = 2.0;
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "-h" | "--help" => {
                print!("{HELP}");
                std::process::exit(0);
            }
            "--segundos" => {
                let v = it.next().ok_or("--segundos requiere un número")?;
                segundos = v
                    .parse::<f64>()
                    .ok()
                    .filter(|s| *s > 0.0)
                    .ok_or(format!("--segundos: '{v}' no es un número mayor que 0"))?;
            }
            other => return Err(format!("opción desconocida '{other}' (ver --help)")),
        }
    }
    Ok(segundos)
}

fn main() {
    let segundos = segundos_de_args().unwrap_or_else(|e| {
        eprintln!("comprobar_vision: {e}");
        std::process::exit(1);
    });
    let fuente = VisionSource::from_env();
    let direccion = format!("{}:{}", fuente.multicast_ip(), fuente.port());
    let socket = escuchar(fuente).unwrap_or_else(|e| {
        println!("✗ No pude escuchar la visión en {direccion}: {e}");
        std::process::exit(1);
    });
    let mut escucha = Escucha::default();
    let mut buf = [0u8; 65536];
    let fin = Instant::now() + Duration::from_secs_f64(segundos);
    while Instant::now() < fin {
        if let Ok(n) = socket.recv(&mut buf) {
            escucha.anotar(&parse_detections(fuente, &buf[..n]));
        }
    }
    let (lineas, codigo) = informe(
        &escucha,
        segundos,
        &direccion,
        TeamColor::from_env().as_team_id(),
    );
    for l in lineas {
        println!("{l}");
    }
    std::process::exit(codigo);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn robot(team: i32, id: i32) -> Detection {
        Detection {
            team,
            id,
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        }
    }
    const PELOTA: Detection = Detection {
        team: -1,
        id: -1,
        x: 0.0,
        y: 0.0,
        theta: 0.0,
    };
    const DIR: &str = "224.5.23.2:10015";

    #[test]
    fn sin_paquetes_da_3() {
        let (lineas, codigo) = informe(&Escucha::default(), 2.0, DIR, 0);
        assert_eq!(codigo, 3);
        assert_eq!(
            lineas,
            ["✗ No llega ningún paquete de visión a 224.5.23.2:10015 en 2 s."]
        );
    }

    #[test]
    fn paquetes_sin_robots_propios_da_4_y_dice_que_ve() {
        let mut e = Escucha::default();
        e.anotar(&[robot(1, 2)]);
        e.anotar(&[]);
        let (lineas, codigo) = informe(&e, 2.0, DIR, 0);
        assert_eq!(codigo, 4);
        assert_eq!(
            lineas[0],
            "✓ Llegan paquetes de visión a 224.5.23.2:10015 (1 por segundo)."
        );
        assert_eq!(lineas[1], "✗ La visión no ve robots azules.");
        assert_eq!(lineas[2], "  También ve robots amarillos: #2 (50 %).");
        assert_eq!(lineas[3], "  No ve la pelota.");
    }

    #[test]
    fn robots_propios_da_0_con_el_porcentaje_de_cuadros() {
        let mut e = Escucha::default();
        e.anotar(&[robot(0, 0), robot(0, 1), PELOTA]);
        e.anotar(&[robot(0, 0), robot(0, 0)]);
        e.anotar(&[robot(0, 0)]);
        e.anotar(&[robot(0, 0), robot(1, 0)]);
        let (lineas, codigo) = informe(&e, 2.0, DIR, 0);
        assert_eq!(codigo, 0);
        assert_eq!(
            lineas[1],
            "✓ La visión ve robots azules: #0 (100 %), #1 (25 %)."
        );
        assert_eq!(lineas[2], "  También ve robots amarillos: #0 (25 %).");
        assert_eq!(lineas[3], "  Ve la pelota (25 %).");
    }

    #[test]
    fn el_equipo_propio_puede_ser_el_amarillo() {
        let mut e = Escucha::default();
        e.anotar(&[robot(0, 0)]);
        assert_eq!(informe(&e, 1.0, DIR, 1).1, 4);
        e.anotar(&[robot(1, 3)]);
        let (lineas, codigo) = informe(&e, 1.0, DIR, 1);
        assert_eq!(codigo, 0);
        assert_eq!(lineas[1], "✓ La visión ve robots amarillos: #3 (50 %).");
        assert_eq!(lineas[2], "  También ve robots azules: #0 (50 %).");
    }
}
