//! Dibujo de un robot: cuerpo negro de 75 × 75 mm con las dos
//! caras cóncavas, marcas de identificación y marca frontal. La misma función dibuja
//! en la cancha y, en miniatura, en las filas de robots.
//!
//! Todo se define en el marco del robot (metros; x hacia adelante, y hacia la
//! izquierda), se pasa a mundo con la pose y a pantalla con la transformación que
//! recibe. Como las transformaciones son afines, las curvas se transforman con sus
//! puntos de control y cualquier orientación de la vista (también espejada) dibuja
//! bien sin casos especiales.

use super::theme::{self, hex};
use glam::Vec2;
use iced::widget::canvas::{self, Frame, Path, Stroke};
use iced::{Color, Point, Rectangle, Renderer, Theme, mouse};

/// Medio lado del cuerpo (75 mm, reglamento).
pub const ROBOT_HALF_M: f32 = 0.0375;

/// **Provisional, pendiente de verificar con una foto cenital del robot.**
/// Profundidad de las caras cóncavas (la de adelante y la de atrás). 5 mm, como el
/// mockup.
pub const FACE_DEPTH_M: f32 = 0.005;

/// **Provisional, pendiente de verificar con una foto cenital del robot.**
/// Giro del tag respecto de la cara frontal (rad, antihorario visto desde arriba).
/// 0 = parche de equipo atrás e identificación adelante, como el mockup y la figura 8
/// del Anexo 1 con el frente hacia abajo.
pub const TAG_ROTATION: f32 = 0.0;

/// Medio lado del área de parches (65 mm, figura 7 del Anexo 1).
const TAG_HALF_M: f32 = 0.0325;
/// Media separación entre parches (5 mm).
const TAG_GAP_HALF_M: f32 = 0.0025;

/// Colores de identificación del Anexo 1 (figura 8, muestreados de
/// `LARC-VSSS-2025-PR.pdf`, pág. 18).
pub const ROJO: Color = hex(0xcc0001);
pub const VERDE: Color = hex(0x00cc08);
pub const CIAN: Color = hex(0x00aace);
pub const MAGENTA: Color = hex(0xcd17dc);

/// Pares de identificación por id de visión (figura 8): `(izquierda, derecha)` de la
/// figura, con el parche de equipo arriba.
pub const ANEXO1: [(Color, Color); 10] = [
    (ROJO, VERDE),
    (ROJO, CIAN),
    (VERDE, ROJO),
    (VERDE, CIAN),
    (VERDE, MAGENTA),
    (CIAN, ROJO),
    (CIAN, VERDE),
    (CIAN, MAGENTA),
    (MAGENTA, VERDE),
    (MAGENTA, CIAN),
];

/// Par de identificación de un id (`None` si el Anexo no lo define).
pub fn colores_id(id: i32) -> Option<(Color, Color)> {
    usize::try_from(id).ok().and_then(|i| ANEXO1.get(i).copied())
}

/// Color del parche de equipo (los tokens de la GUI: es el color del equipo en toda
/// la interfaz).
pub fn color_equipo(team: i32) -> Color {
    if team == 0 { theme::AZUL } else { theme::AMARILLO }
}

/// Marcas de identificación que se dibujan (opción de configuración hasta confirmar
/// en la salita qué llevan los robots).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MarcasRobot {
    /// Parches del Anexo 1 del reglamento IEEE VSSS.
    #[default]
    Anexo1,
    /// Solo un parche de equipo de 65 × 65 mm, sin colores de identificación.
    SoloEquipo,
}

/// Variable de entorno con las marcas de arranque.
pub const MARCAS_ENV: &str = "VSSL_GUI_MARCAS";

impl MarcasRobot {
    pub const ALL: [MarcasRobot; 2] = [MarcasRobot::Anexo1, MarcasRobot::SoloEquipo];

    pub fn etiqueta(self) -> &'static str {
        match self {
            MarcasRobot::Anexo1 => "Anexo 1",
            MarcasRobot::SoloEquipo => "solo equipo",
        }
    }

    /// `anexo1` o `equipo` (sin distinguir mayúsculas).
    pub fn parse(raw: &str) -> Option<Self> {
        match raw.trim().to_ascii_lowercase().as_str() {
            "anexo1" => Some(MarcasRobot::Anexo1),
            "equipo" => Some(MarcasRobot::SoloEquipo),
            _ => None,
        }
    }

    /// Valor de `VSSL_GUI_MARCAS`; inválido o ausente, Anexo 1 (con aviso si es inválido).
    pub fn from_env() -> Self {
        match std::env::var(MARCAS_ENV) {
            Ok(raw) => Self::parse(&raw).unwrap_or_else(|| {
                eprintln!("[GUI] {MARCAS_ENV}='{raw}' no es válido (anexo1 o equipo): uso anexo1");
                MarcasRobot::Anexo1
            }),
            Err(_) => MarcasRobot::Anexo1,
        }
    }
}

/// Un parche: rectángulo en el marco del tag (m) `(x0, y0, x1, y1)` y su color.
pub type Parche = ([f32; 4], Color);

/// Parches de un robot, en el marco del tag (antes de `TAG_ROTATION`), según la
/// figura 7: equipo de 30 × 65 mm atrás, identificación de 30 × 30 mm adelante, 5 mm
/// entre parches. Con la figura 8 vista con el frente hacia abajo, el color de la
/// izquierda de la figura queda a la derecha del robot (−y).
pub fn parches(marcas: MarcasRobot, team: i32, id: i32) -> Vec<Parche> {
    let (a, g) = (TAG_HALF_M, TAG_GAP_HALF_M);
    let equipo = color_equipo(team);
    match (marcas, colores_id(id)) {
        (MarcasRobot::SoloEquipo, _) => vec![([-a, -a, a, a], equipo)],
        (MarcasRobot::Anexo1, Some((izq_fig, der_fig))) => vec![
            ([-a, -a, -g, a], equipo),
            ([g, -a, a, -g], izq_fig),
            ([g, g, a, a], der_fig),
        ],
        (MarcasRobot::Anexo1, None) => vec![([-a, -a, -g, a], equipo)],
    }
}

/// Pose de un robot en el mundo (m, rad).
#[derive(Debug, Clone, Copy)]
pub struct Pose {
    pub pos: Vec2,
    pub theta: f32,
}

impl Pose {
    /// Punto del marco del robot → mundo.
    pub fn a_mundo(&self, local: Vec2) -> Vec2 {
        let (s, c) = self.theta.sin_cos();
        self.pos + Vec2::new(c * local.x - s * local.y, s * local.x + c * local.y)
    }
}

/// Dibuja un robot. `a_pantalla` pasa un punto del mundo a la pantalla.
pub fn dibujar_robot(
    frame: &mut Frame,
    a_pantalla: &impl Fn(Vec2) -> Point,
    pose: Pose,
    team: i32,
    id: i32,
    marcas: MarcasRobot,
) {
    let p = |x: f32, y: f32| a_pantalla(pose.a_mundo(Vec2::new(x, y)));
    let h = ROBOT_HALF_M;
    // Con una cuadrática, la flecha es la mitad del corrimiento del punto de control.
    let d2 = 2.0 * FACE_DEPTH_M;

    let cuerpo = Path::new(|b| {
        b.move_to(p(-h, -h));
        b.line_to(p(h, -h));
        b.quadratic_curve_to(p(h - d2, 0.0), p(h, h));
        b.line_to(p(-h, h));
        b.quadratic_curve_to(p(-h + d2, 0.0), p(-h, -h));
        b.close();
    });
    frame.fill(&cuerpo, theme::PARED);

    let (s, c) = TAG_ROTATION.sin_cos();
    let t = |x: f32, y: f32| p(c * x - s * y, s * x + c * y);
    for ([x0, y0, x1, y1], color) in parches(marcas, team, id) {
        let rect = Path::new(|b| {
            b.move_to(t(x0, y0));
            b.line_to(t(x1, y0));
            b.line_to(t(x1, y1));
            b.line_to(t(x0, y1));
            b.close();
        });
        frame.fill(&rect, color);
    }

    // Marca frontal: la cara delantera en cal.
    let frente = Path::new(|b| {
        b.move_to(p(h, -h * 0.8));
        b.quadratic_curve_to(p(h - d2 * 0.9, 0.0), p(h, h * 0.8));
    });
    frame.stroke(&frente, Stroke::default().with_width(1.6).with_color(theme::CAL));
}

/// Miniatura del robot para las filas (frente hacia la derecha).
pub struct ParcheCanvas {
    pub team: i32,
    pub id: i32,
    pub marcas: MarcasRobot,
}

impl<Message> canvas::Program<Message> for ParcheCanvas {
    type State = ();

    fn draw(
        &self,
        _state: &Self::State,
        renderer: &Renderer,
        _theme: &Theme,
        bounds: Rectangle,
        _cursor: mouse::Cursor,
    ) -> Vec<canvas::Geometry> {
        let mut frame = Frame::new(renderer, bounds.size());
        // 75 mm de robot con 2 mm de aire por lado.
        let k = bounds.width.min(bounds.height) / (2.0 * ROBOT_HALF_M + 0.004);
        let centro = Point::new(bounds.width / 2.0, bounds.height / 2.0);
        let a_pantalla = |p: Vec2| Point::new(centro.x + p.x * k, centro.y - p.y * k);
        let pose = Pose {
            pos: Vec2::ZERO,
            theta: 0.0,
        };
        dibujar_robot(&mut frame, &a_pantalla, pose, self.team, self.id, self.marcas);
        vec![frame.into_geometry()]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Figura 8 del Anexo 1, de izquierda a derecha con el equipo arriba.
    #[test]
    fn table_matches_figure_8() {
        let nombre = |c: Color| match c {
            c if c == ROJO => "rojo",
            c if c == VERDE => "verde",
            c if c == CIAN => "cian",
            c if c == MAGENTA => "magenta",
            _ => "?",
        };
        let figura = [
            ("rojo", "verde"),
            ("rojo", "cian"),
            ("verde", "rojo"),
            ("verde", "cian"),
            ("verde", "magenta"),
            ("cian", "rojo"),
            ("cian", "verde"),
            ("cian", "magenta"),
            ("magenta", "verde"),
            ("magenta", "cian"),
        ];
        for (id, esperado) in figura.iter().enumerate() {
            let (a, b) = colores_id(id as i32).expect("id del Anexo");
            assert_eq!((nombre(a), nombre(b)), *esperado, "id {id}");
        }
        assert_eq!(colores_id(10), None);
        assert_eq!(colores_id(-1), None);
    }

    #[test]
    fn every_pair_is_distinct() {
        for (i, a) in ANEXO1.iter().enumerate() {
            for (j, b) in ANEXO1.iter().enumerate().skip(i + 1) {
                assert_ne!(a, b, "ids {i} y {j}");
            }
        }
    }

    #[test]
    fn annex_1_patches_of_robot_4() {
        let p = parches(MarcasRobot::Anexo1, 0, 4);
        assert_eq!(p.len(), 3);
        assert_eq!(p[0].1, theme::AZUL);
        // Equipo de 30 × 65 mm atrás.
        let [x0, y0, x1, y1] = p[0].0;
        assert!(((x1 - x0) - 0.030).abs() < 1e-6 && ((y1 - y0) - 0.065).abs() < 1e-6);
        assert!(x1 < 0.0);
        // Identificación de 30 × 30 mm adelante, verde a la derecha y magenta a la izquierda.
        assert_eq!(p[1].1, VERDE);
        assert_eq!(p[2].1, MAGENTA);
        for ([x0, y0, x1, y1], _) in &p[1..] {
            assert!(((x1 - x0) - 0.030).abs() < 1e-6 && ((y1 - y0) - 0.030).abs() < 1e-6);
            assert!(*x0 > 0.0);
        }
        assert!(p[1].0[3] < 0.0 && p[2].0[1] > 0.0);
    }

    #[test]
    fn team_only_has_no_id_colors() {
        let p = parches(MarcasRobot::SoloEquipo, 1, 4);
        assert_eq!(p.len(), 1);
        assert_eq!(p[0].1, theme::AMARILLO);
        let [x0, y0, x1, y1] = p[0].0;
        assert!(((x1 - x0) - 0.065).abs() < 1e-6 && ((y1 - y0) - 0.065).abs() < 1e-6);
    }

    #[test]
    fn id_without_annex_entry_keeps_the_team_patch() {
        let p = parches(MarcasRobot::Anexo1, 0, 12);
        assert_eq!(p.len(), 1);
        assert_eq!(p[0].1, theme::AZUL);
    }

    #[test]
    fn patches_fit_inside_the_body() {
        for (rect, _) in parches(MarcasRobot::Anexo1, 0, 0) {
            assert!(rect.iter().all(|v| v.abs() <= ROBOT_HALF_M));
        }
    }

    #[test]
    fn marks_option_parsing() {
        assert_eq!(MarcasRobot::parse("anexo1"), Some(MarcasRobot::Anexo1));
        assert_eq!(MarcasRobot::parse(" Equipo "), Some(MarcasRobot::SoloEquipo));
        assert_eq!(MarcasRobot::parse("qr"), None);
    }

    #[test]
    fn pose_to_world() {
        let pose = Pose {
            pos: Vec2::new(0.3, 0.2),
            theta: std::f32::consts::FRAC_PI_2,
        };
        let frente = pose.a_mundo(Vec2::new(ROBOT_HALF_M, 0.0));
        assert!((frente - Vec2::new(0.3, 0.2 + ROBOT_HALF_M)).length() < 1e-6);
    }
}
