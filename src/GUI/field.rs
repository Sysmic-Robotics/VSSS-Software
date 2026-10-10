//! La cancha: medidas del reglamento, fieltro configurable, regla en cm,
//! robots y pelota a escala.
//!
//! Un solo canvas en pilas:
//! 1. **Base** en un `canvas::Cache`: fieltro, líneas, área con su arco, centro,
//!    marcas, paredes, triángulos, arcos y regla. Se regenera sola al cambiar el
//!    tamaño y se limpia al cambiar el fieltro o la orientación.
//! 2. **Capas** (fase 2, ver `keys::Layer`): aquí se apilarán; en esta fase no hay.
//! 3. **Objetos**, en un `Frame` nuevo por foto: robots, pelota, rótulos, robot
//!    seleccionado y target de la skill de la GUI.
//!
//! El dibujo y los clicks usan la misma transformación ([`Vista`]). Las medidas salen
//! de las constantes que ya usa el software (el guardia de zonas, los plays): lo que
//! se ve es lo mismo que respeta el guardia.
//!
//! Un robot, la pelota o un target con un valor no finito (NaN o infinito, de la
//! visión o del filtro) no se dibujan en ese cuadro: lyon rechaza los puntos no
//! finitos al armar un `Path`.

use super::tag::{self, MarcasRobot, Pose};
use super::{Message, fonts, theme};
use crate::coach::heuristic_coach::GOAL_HALF_Y;
use crate::coach::plays::{CENTER_CIRCLE_R, MARK_X, MARK_Y};
use crate::coach::{FIELD_HALF_X, FIELD_HALF_Y};
use crate::skills::zones::{ARC_CENTER_X, ARC_RADIUS, AREA_HALF_Y, AREA_X};
use crate::snapshot::{LoopSnapshot, RobotView};
use glam::Vec2;
use iced::widget::canvas::{self, Cache, Frame, Path, Stroke};
use iced::{Color, Point, Rectangle, Renderer, Size, Theme, mouse};
use std::collections::BTreeMap;

/// Ancho de las paredes vistas desde arriba (2.5 cm, reglamento).
pub const WALL_M: f32 = 0.025;
/// Profundidad interior de los arcos (10 cm).
pub const GOAL_DEPTH_M: f32 = 0.10;
/// Cateto de los triángulos de las esquinas (7 cm).
pub const TRIANGLE_M: f32 = 0.07;
/// Radio de la pelota (42.7 mm de diámetro).
pub const BALL_RADIUS_M: f32 = 0.02135;
/// Ancho de las líneas de cal (3 mm).
pub const LINE_M: f32 = 0.003;
/// Brazo de las cruces de las marcas.
const MARK_ARM_M: f32 = 0.012;
/// Margen lateral (px) para los números de la regla de la izquierda, a cada lado para
/// que la cancha quede centrada. En píxeles porque el texto tiene tamaño fijo.
const MARGIN_X_PX: f32 = 30.0;
/// Margen vertical (px) para los números de la regla de arriba.
const MARGIN_Y_PX: f32 = 18.0;
/// Debajo de esta separación (px) se omiten las marcas de 1 cm de la regla.
const RULER_MIN_PX_PER_CM: f32 = 3.0;
/// Radio de la marca del robot seleccionado.
const SELECTION_RADIUS_M: f32 = 0.06;
/// Un robot con el último dato más viejo que esto no se dibuja.
const MAX_DRAW_AGE_S: f32 = 1.0;

/// Color del fieltro.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Fieltro {
    /// El de la salita (por defecto).
    #[default]
    Verde,
    /// Negro mate del reglamento.
    Negro,
}

impl Fieltro {
    pub const ALL: [Fieltro; 2] = [Fieltro::Verde, Fieltro::Negro];

    pub fn color(self) -> Color {
        match self {
            Fieltro::Verde => theme::FIELTRO,
            Fieltro::Negro => theme::FIELTRO_NEGRO,
        }
    }

    pub fn etiqueta(self) -> &'static str {
        match self {
            Fieltro::Verde => "verde",
            Fieltro::Negro => "negro reglamento",
        }
    }
}

/// Orientación de la vista, para que coincida con la mesa y la cámara.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum OrientacionVista {
    /// x hacia la derecha, y hacia arriba (el mundo del software).
    #[default]
    Normal,
    Girada180,
    /// Izquierda y derecha intercambiadas.
    EspejoHorizontal,
    /// Arriba y abajo intercambiados.
    EspejoVertical,
}

/// Variable de entorno con la orientación de arranque.
pub const VISTA_ENV: &str = "VSSL_GUI_VISTA";

impl OrientacionVista {
    pub const ALL: [OrientacionVista; 4] = [
        OrientacionVista::Normal,
        OrientacionVista::Girada180,
        OrientacionVista::EspejoHorizontal,
        OrientacionVista::EspejoVertical,
    ];

    /// Signo de cada eje del mundo en pantalla (x hacia la derecha, y hacia arriba).
    fn signos(self) -> (f32, f32) {
        match self {
            OrientacionVista::Normal => (1.0, 1.0),
            OrientacionVista::Girada180 => (-1.0, -1.0),
            OrientacionVista::EspejoHorizontal => (-1.0, 1.0),
            OrientacionVista::EspejoVertical => (1.0, -1.0),
        }
    }

    pub fn etiqueta(self) -> &'static str {
        match self {
            OrientacionVista::Normal => "normal",
            OrientacionVista::Girada180 => "girada 180°",
            OrientacionVista::EspejoHorizontal => "espejada horizontal",
            OrientacionVista::EspejoVertical => "espejada vertical",
        }
    }

    /// `normal`, `girada180`, `espejo-h` o `espejo-v` (sin distinguir mayúsculas).
    pub fn parse(raw: &str) -> Option<Self> {
        match raw.trim().to_ascii_lowercase().as_str() {
            "normal" => Some(OrientacionVista::Normal),
            "girada180" => Some(OrientacionVista::Girada180),
            "espejo-h" => Some(OrientacionVista::EspejoHorizontal),
            "espejo-v" => Some(OrientacionVista::EspejoVertical),
            _ => None,
        }
    }

    /// Valor de `VSSL_GUI_VISTA`; inválido o ausente, normal (con aviso si es inválido).
    pub fn from_env() -> Self {
        match std::env::var(VISTA_ENV) {
            Ok(raw) => Self::parse(&raw).unwrap_or_else(|| {
                eprintln!(
                    "[GUI] {VISTA_ENV}='{raw}' no es válido (normal, girada180, espejo-h o espejo-v): uso normal"
                );
                OrientacionVista::Normal
            }),
            Err(_) => OrientacionVista::Normal,
        }
    }
}

/// Transformación mundo (m) ↔ pantalla (px) del canvas. Escala uniforme, cancha
/// centrada, con lugar para paredes, arcos y regla. La usan el dibujo y el click.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vista {
    centro: Point,
    k: f32,
    sx: f32,
    sy: f32,
}

impl Vista {
    pub fn new(size: Size, orientacion: OrientacionVista) -> Self {
        let half_w = FIELD_HALF_X + GOAL_DEPTH_M + WALL_M;
        let half_h = FIELD_HALF_Y + WALL_M;
        let k = ((size.width - 2.0 * MARGIN_X_PX) / (2.0 * half_w))
            .min((size.height - 2.0 * MARGIN_Y_PX) / (2.0 * half_h))
            .max(1e-3);
        let (sx, sy) = orientacion.signos();
        Self {
            centro: Point::new(size.width / 2.0, size.height / 2.0),
            k,
            sx,
            sy,
        }
    }

    pub fn a_pantalla(&self, p: Vec2) -> Point {
        Point::new(
            self.centro.x + self.sx * p.x * self.k,
            self.centro.y - self.sy * p.y * self.k,
        )
    }

    pub fn a_mundo(&self, q: Point) -> Vec2 {
        Vec2::new(
            (q.x - self.centro.x) / (self.sx * self.k),
            (self.centro.y - q.y) / (self.sy * self.k),
        )
    }

    /// Metros → píxeles.
    pub fn px(&self, m: f32) -> f32 {
        m * self.k
    }

    /// Rectángulo de la cancha en pantalla: igual en toda orientación (la cancha es
    /// simétrica).
    pub fn rect_cancha(&self) -> Rectangle {
        Rectangle {
            x: self.centro.x - FIELD_HALF_X * self.k,
            y: self.centro.y - FIELD_HALF_Y * self.k,
            width: 2.0 * FIELD_HALF_X * self.k,
            height: 2.0 * FIELD_HALF_Y * self.k,
        }
    }
}

/// Ancho aproximado (px) de un texto en Barlow, para alinear a mano: el texto del
/// canvas va alineado arriba a la izquierda (ver [`escribir`]).
pub fn ancho_texto(texto: &str, tamano: f32) -> f32 {
    texto
        .chars()
        .map(|c| match c {
            ' ' => 0.2,
            '0'..='9' => 0.53,
            'm' => 0.85,
            _ => 0.5,
        })
        .sum::<f32>()
        * tamano
}

/// Si el texto del canvas se escribe como trazos: con tiny-skia, el renderer por CPU
/// de iced (el respaldo sin GPU).
///
/// iced 0.13 le da al texto del canvas límites infinitos, y el compositor de
/// tiny-skia, al calcular qué cambió en cada cuadro, los multiplica por una matriz:
/// 0·∞ da NaN, y un rectángulo NaN hace entrar en pánico al `sort` de
/// `iced_graphics::damage::group` (Rust 1.81+). Pasa apenas un texto cambia entre
/// cuadros, por ejemplo el rótulo de un robot que se mueve. Como trazos de sus
/// glifos, el texto tiene límites finitos. Con wgpu no hay cálculo de daño, y el
/// texto va como texto.
fn texto_como_trazos(renderer: &Renderer) -> bool {
    matches!(renderer, Renderer::Secondary(_))
}

/// Escribe `texto` en el canvas, como texto o como trazos (ver
/// [`texto_como_trazos`]).
fn escribir(frame: &mut Frame, texto: canvas::Text, trazos: bool) {
    if trazos {
        texto.draw_with(|path, color| frame.fill(&path, color));
    } else {
        frame.fill_text(texto);
    }
}

/// Si la pose de un robot se puede dibujar (sin NaN ni infinitos).
pub fn pose_finita(r: &RobotView) -> bool {
    r.position.is_finite() && r.orientation.is_finite()
}

/// Si un punto del mundo está dentro de la cancha.
pub fn en_cancha(p: Vec2) -> bool {
    p.x.abs() <= FIELD_HALF_X && p.y.abs() <= FIELD_HALF_Y
}

/// Largo de una marca de la regla.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MarcaRegla {
    Uno,
    Cinco,
    Diez,
}

/// Marcas de una regla de `largo_cm`: cada 1 cm, salvo que a `px_por_cm` queden a
/// menos de 3 px; entonces solo las de 5 y 10 cm.
pub fn marcas_regla(largo_cm: u32, px_por_cm: f32) -> Vec<(u32, MarcaRegla)> {
    let todas = px_por_cm >= RULER_MIN_PX_PER_CM;
    (0..=largo_cm)
        .filter_map(|c| {
            let m = if c % 10 == 0 {
                MarcaRegla::Diez
            } else if c % 5 == 0 {
                MarcaRegla::Cinco
            } else {
                MarcaRegla::Uno
            };
            (todas || m != MarcaRegla::Uno).then_some((c, m))
        })
        .collect()
}

/// Arco del área sobre su frente, del lado `signo` (+1 derecho, −1 izquierdo): cuerda
/// de 20 cm en |x| = `AREA_X` y flecha de 5 cm hacia la cancha.
pub fn arco_area(signo: f32) -> Vec<Vec2> {
    let a0 = ((ARC_CENTER_X - AREA_X) / ARC_RADIUS).acos();
    const N: usize = 24;
    (0..=N)
        .map(|i| {
            let a = -a0 + 2.0 * a0 * i as f32 / N as f32;
            Vec2::new(
                signo * (ARC_CENTER_X - ARC_RADIUS * a.cos()),
                ARC_RADIUS * a.sin(),
            )
        })
        .collect()
}

/// La cancha y lo que hay sobre ella.
pub struct FieldCanvas<'a> {
    pub snap: &'a LoopSnapshot,
    pub base: &'a Cache,
    pub fieltro: Fieltro,
    pub orientacion: OrientacionVista,
    pub marcas: MarcasRobot,
    pub own_team: i32,
    /// Robot seleccionado `(equipo, id)`.
    pub selected: Option<(i32, i32)>,
    /// Target de la skill de la GUI (`skill_marker`).
    pub skill_target: Option<Vec2>,
    /// Mapa visión → radio, solo con la base station (rótulo "radio P").
    pub radio_slots: Option<&'a BTreeMap<u32, u32>>,
}

impl canvas::Program<Message> for FieldCanvas<'_> {
    type State = ();

    fn update(
        &self,
        _state: &mut Self::State,
        event: canvas::Event,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> (canvas::event::Status, Option<Message>) {
        if let canvas::Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) = event
            && let Some(p) = cursor.position_in(bounds)
        {
            let mundo = Vista::new(bounds.size(), self.orientacion).a_mundo(p);
            if en_cancha(mundo) {
                return (canvas::event::Status::Captured, Some(Message::FieldClicked(mundo)));
            }
        }
        (canvas::event::Status::Ignored, None)
    }

    fn draw(
        &self,
        _state: &Self::State,
        renderer: &Renderer,
        _theme: &Theme,
        bounds: Rectangle,
        _cursor: mouse::Cursor,
    ) -> Vec<canvas::Geometry> {
        let vista = Vista::new(bounds.size(), self.orientacion);
        let trazos = texto_como_trazos(renderer);
        let base = self.base.draw(renderer, bounds.size(), |frame| {
            dibujar_base(frame, &vista, self.fieltro, trazos)
        });
        // Fase 2: aquí van las capas, entre la base y los objetos.
        let mut objetos = Frame::new(renderer, bounds.size());
        self.dibujar_objetos(&mut objetos, &vista, trazos);
        vec![base, objetos.into_geometry()]
    }
}

/// Rectángulo del mundo como polígono (sirve con la vista espejada).
fn rect_mundo(vista: &Vista, x0: f32, y0: f32, x1: f32, y1: f32) -> Path {
    Path::new(|b| {
        b.move_to(vista.a_pantalla(Vec2::new(x0, y0)));
        b.line_to(vista.a_pantalla(Vec2::new(x1, y0)));
        b.line_to(vista.a_pantalla(Vec2::new(x1, y1)));
        b.line_to(vista.a_pantalla(Vec2::new(x0, y1)));
        b.close();
    })
}

fn polilinea(vista: &Vista, puntos: &[Vec2]) -> Path {
    Path::new(|b| {
        for (i, p) in puntos.iter().enumerate() {
            let q = vista.a_pantalla(*p);
            if i == 0 {
                b.move_to(q);
            } else {
                b.line_to(q);
            }
        }
    })
}

/// Parte fija de la cancha.
fn dibujar_base(frame: &mut Frame, vista: &Vista, fieltro: Fieltro, trazos: bool) {
    let (hx, hy) = (FIELD_HALF_X, FIELD_HALF_Y);
    let pared = theme::PARED;

    // Paredes alrededor de la cancha y de los arcos.
    frame.fill(&rect_mundo(vista, -hx - WALL_M, -hy - WALL_M, hx + WALL_M, hy + WALL_M), pared);
    for s in [-1.0_f32, 1.0] {
        let fondo = s * (hx + GOAL_DEPTH_M + WALL_M);
        frame.fill(
            &rect_mundo(vista, s * hx, -GOAL_HALF_Y - WALL_M, fondo, GOAL_HALF_Y + WALL_M),
            pared,
        );
    }
    // Fieltro y fondo de los arcos.
    frame.fill(&rect_mundo(vista, -hx, -hy, hx, hy), fieltro.color());
    for s in [-1.0_f32, 1.0] {
        frame.fill(
            &rect_mundo(vista, s * hx, -GOAL_HALF_Y, s * (hx + GOAL_DEPTH_M), GOAL_HALF_Y),
            theme::fondo_arco(fieltro.color()),
        );
    }
    // Triángulos de las esquinas.
    for (sx, sy) in [(-1.0_f32, -1.0_f32), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)] {
        let tri = polilinea(
            vista,
            &[
                Vec2::new(sx * hx, sy * hy),
                Vec2::new(sx * (hx - TRIANGLE_M), sy * hy),
                Vec2::new(sx * hx, sy * (hy - TRIANGLE_M)),
                Vec2::new(sx * hx, sy * hy),
            ],
        );
        frame.fill(&tri, pared);
    }

    // Líneas de cal de 3 mm.
    let ancho = vista.px(LINE_M).max(1.0);
    let cal = Stroke::default()
        .with_width(ancho)
        .with_color(theme::alfa(theme::CAL, 0.92));
    frame.stroke(&polilinea(vista, &[Vec2::new(0.0, -hy), Vec2::new(0.0, hy)]), cal);
    frame.stroke(
        &Path::circle(vista.a_pantalla(Vec2::ZERO), vista.px(CENTER_CIRCLE_R)),
        cal,
    );
    for s in [-1.0_f32, 1.0] {
        // Rectángulo del área (el cuarto lado es la línea de fondo, la pared).
        frame.stroke(
            &polilinea(
                vista,
                &[
                    Vec2::new(s * hx, AREA_HALF_Y),
                    Vec2::new(s * AREA_X, AREA_HALF_Y),
                    Vec2::new(s * AREA_X, -AREA_HALF_Y),
                    Vec2::new(s * hx, -AREA_HALF_Y),
                ],
            ),
            cal,
        );
        frame.stroke(&polilinea(vista, &arco_area(s)), cal);
        // Línea de gol, algo más marcada.
        frame.stroke(
            &polilinea(vista, &[Vec2::new(s * hx, -GOAL_HALF_Y), Vec2::new(s * hx, GOAL_HALF_Y)]),
            Stroke {
                width: ancho * 1.6,
                ..cal
            },
        );
    }
    // Marcas (penal, tiro libre y bola libre).
    let marca = Stroke::default()
        .with_width((ancho * 0.8).max(1.0))
        .with_color(theme::alfa(theme::CAL, 0.7));
    for (x, y) in [
        (MARK_X, 0.0),
        (MARK_X, MARK_Y),
        (MARK_X, -MARK_Y),
        (-MARK_X, 0.0),
        (-MARK_X, MARK_Y),
        (-MARK_X, -MARK_Y),
    ] {
        let c = Vec2::new(x, y);
        frame.stroke(
            &polilinea(vista, &[c - Vec2::X * MARK_ARM_M, c + Vec2::X * MARK_ARM_M]),
            marca,
        );
        frame.stroke(
            &polilinea(vista, &[c - Vec2::Y * MARK_ARM_M, c + Vec2::Y * MARK_ARM_M]),
            marca,
        );
    }

    dibujar_regla(frame, vista, trazos);
}

/// Regla grabada en las paredes que se ven arriba y a la izquierda, contando desde la
/// esquina de arriba a la izquierda (en coordenadas de pantalla: no depende de la
/// orientación de la vista).
fn dibujar_regla(frame: &mut Frame, vista: &Vista, trazos: bool) {
    let r = vista.rect_cancha();
    let banda = vista.px(WALL_M);
    let px_cm = vista.px(0.01);
    let largo = |m: MarcaRegla| match m {
        MarcaRegla::Diez => banda * 0.85,
        MarcaRegla::Cinco => banda * 0.55,
        MarcaRegla::Uno => banda * 0.3,
    };
    let trazo = |m: MarcaRegla| {
        Stroke::default()
            .with_width(if m == MarcaRegla::Diez { 1.3 } else { 0.8 })
            .with_color(theme::REGLA)
    };
    // Texto alineado arriba a la izquierda (ver `ancho_texto`): `position` es su
    // esquina de arriba a la izquierda.
    let alto_numero = theme::TXT_MINI * 1.3;
    let numero = |content: String, position: Point| canvas::Text {
        content,
        position,
        color: theme::GRIS,
        size: theme::TXT_MINI.into(),
        font: fonts::TEXTO,
        ..canvas::Text::default()
    };

    let largo_x = (2.0 * FIELD_HALF_X * 100.0).round() as u32;
    for (c, m) in marcas_regla(largo_x, px_cm) {
        let x = r.x + c as f32 * px_cm;
        frame.stroke(&Path::line(Point::new(x, r.y), Point::new(x, r.y - largo(m))), trazo(m));
        if c % 25 == 0 {
            let texto = format!("{c} cm");
            let ancho = ancho_texto(&texto, theme::TXT_MINI);
            escribir(
                frame,
                numero(texto, Point::new(x - ancho / 2.0, r.y - banda - 2.0 - alto_numero)),
                trazos,
            );
        }
    }
    let largo_y = (2.0 * FIELD_HALF_Y * 100.0).round() as u32;
    // Los números de la izquierda van por fuera de los arcos, alineados en columna.
    let x_numeros = r.x - banda - vista.px(GOAL_DEPTH_M) - 4.0;
    for (c, m) in marcas_regla(largo_y, px_cm) {
        let y = r.y + c as f32 * px_cm;
        frame.stroke(&Path::line(Point::new(r.x, y), Point::new(r.x - largo(m), y)), trazo(m));
        if c % 25 == 0 {
            let texto = c.to_string();
            let ancho = ancho_texto(&texto, theme::TXT_MINI);
            escribir(
                frame,
                numero(texto, Point::new(x_numeros - ancho, y - alto_numero / 2.0)),
                trazos,
            );
        }
    }
}

impl FieldCanvas<'_> {
    fn dibujar_objetos(&self, frame: &mut Frame, vista: &Vista, trazos: bool) {
        let a_pantalla = |p: Vec2| vista.a_pantalla(p);
        let visibles = self
            .snap
            .robots
            .iter()
            .filter(|r| r.active && r.age_s <= MAX_DRAW_AGE_S && pose_finita(r));

        for r in visibles.clone() {
            let pose = Pose {
                pos: r.position,
                theta: r.orientation as f32,
            };
            tag::dibujar_robot(frame, &a_pantalla, pose, r.team, r.id, self.marcas);
        }

        if let Some(ball) = self.snap.ball.filter(|b| b.position.is_finite()) {
            let c = vista.a_pantalla(ball.position);
            let disco = Path::circle(c, vista.px(BALL_RADIUS_M).max(2.0));
            frame.fill(&disco, theme::PELOTA);
            frame.stroke(&disco, Stroke::default().with_width(1.0).with_color(theme::PELOTA_BORDE));
        }

        for r in visibles {
            let c = vista.a_pantalla(r.position);
            if self.selected == Some((r.team, r.id)) {
                frame.stroke(
                    &Path::circle(c, vista.px(SELECTION_RADIUS_M)),
                    Stroke::default()
                        .with_width(1.6)
                        .with_color(theme::alfa(theme::CAL, 0.8)),
                );
            }
            let texto = canvas::Text {
                content: rotulo(r.team == self.own_team, r.id, self.radio_slots),
                // Arriba a la izquierda (ver `ancho_texto`): el rótulo queda arriba a la
                // derecha del robot.
                position: Point::new(
                    c.x + vista.px(0.045),
                    c.y - vista.px(0.045) - theme::TXT * 1.3,
                ),
                color: theme::CAL,
                size: theme::TXT.into(),
                font: fonts::TITULO,
                ..canvas::Text::default()
            };
            escribir(frame, texto, trazos);
        }

        if let Some(t) = self.skill_target.filter(|t| t.is_finite()) {
            let c = vista.a_pantalla(t);
            let trazo = Stroke::default().with_width(2.0).with_color(theme::CAL);
            let r = vista.px(0.025).max(8.0);
            frame.stroke(&Path::circle(c, r), trazo);
            let brazo = r * 0.7;
            frame.stroke(&Path::line(Point::new(c.x - brazo, c.y), Point::new(c.x + brazo, c.y)), trazo);
            frame.stroke(&Path::line(Point::new(c.x, c.y - brazo), Point::new(c.x, c.y + brazo)), trazo);
        }
    }
}

/// Rótulo de un robot: el número de visión, y "radio P" para los propios con la base
/// station.
pub fn rotulo(propio: bool, id: i32, radio_slots: Option<&BTreeMap<u32, u32>>) -> String {
    match radio_slots.filter(|_| propio) {
        Some(mapa) => match crate::radio::base_station::radio_slot(id, mapa) {
            Some(slot) => format!("{id} radio {slot}"),
            None => id.to_string(),
        },
        None => id.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn size() -> Size {
        Size::new(1000.0, 800.0)
    }

    #[test]
    fn screen_center_is_world_origin() {
        for o in OrientacionVista::ALL {
            let v = Vista::new(size(), o);
            let w = v.a_mundo(Point::new(500.0, 400.0));
            assert!(w.length() < 1e-6, "{o:?}");
        }
    }

    #[test]
    fn round_trip_in_every_orientation() {
        for o in OrientacionVista::ALL {
            let v = Vista::new(size(), o);
            for p in [Vec2::new(0.3, 0.2), Vec2::new(-0.7, 0.6), Vec2::new(0.0, -0.65)] {
                let q = v.a_mundo(v.a_pantalla(p));
                assert!((q - p).length() < 1e-4, "{o:?}: {p} → {q}");
            }
        }
    }

    #[test]
    fn known_point_in_the_normal_view() {
        let v = Vista::new(size(), OrientacionVista::Normal);
        let q = v.a_pantalla(Vec2::new(0.1, 0.05));
        assert!((q.x - (500.0 + v.px(0.1))).abs() < 1e-3);
        assert!((q.y - (400.0 - v.px(0.05))).abs() < 1e-3, "y crece hacia abajo en pantalla");
    }

    #[test]
    fn rotated_view_puts_a_robot_where_normal_draws_its_opposite() {
        let n = Vista::new(size(), OrientacionVista::Normal);
        let g = Vista::new(size(), OrientacionVista::Girada180);
        let a = g.a_pantalla(Vec2::new(0.3, 0.2));
        let b = n.a_pantalla(Vec2::new(-0.3, -0.2));
        assert!((a.x - b.x).abs() < 1e-3 && (a.y - b.y).abs() < 1e-3);
    }

    #[test]
    fn mirrored_click_returns_the_world_point() {
        let v = Vista::new(size(), OrientacionVista::EspejoHorizontal);
        let p = Vec2::new(0.4, 0.1);
        let q = v.a_mundo(v.a_pantalla(p));
        assert!((q - p).length() < 1e-4);
        // Y en pantalla queda a la izquierda del centro.
        assert!(v.a_pantalla(p).x < 500.0);
    }

    #[test]
    fn mirrored_robot_front_points_up_left() {
        let v = Vista::new(size(), OrientacionVista::EspejoHorizontal);
        let pose = Pose {
            pos: Vec2::new(0.1, 0.1),
            theta: 30f32.to_radians(),
        };
        let centro = v.a_pantalla(pose.pos);
        let frente = v.a_pantalla(pose.a_mundo(Vec2::new(tag::ROBOT_HALF_M, 0.0)));
        assert!(frente.x < centro.x, "espejada: el frente va a la izquierda");
        assert!(frente.y < centro.y, "y hacia arriba");
    }

    #[test]
    fn field_rect_is_the_same_in_every_orientation() {
        let r0 = Vista::new(size(), OrientacionVista::Normal).rect_cancha();
        for o in OrientacionVista::ALL {
            assert_eq!(Vista::new(size(), o).rect_cancha(), r0, "{o:?}");
        }
    }

    #[test]
    fn everything_fits_in_the_canvas() {
        let v = Vista::new(size(), OrientacionVista::Normal);
        let afuera = v.a_pantalla(Vec2::new(
            -(FIELD_HALF_X + GOAL_DEPTH_M + WALL_M),
            FIELD_HALF_Y + WALL_M,
        ));
        // Queda lugar para los números de la regla.
        assert!(afuera.x >= MARGIN_X_PX - 1e-3 && afuera.y >= MARGIN_Y_PX - 1e-3);
    }

    #[test]
    fn click_outside_the_field_is_ignored() {
        assert!(en_cancha(Vec2::new(0.74, -0.64)));
        assert!(!en_cancha(Vec2::new(0.8, 0.0)), "dentro del arco no es cancha");
        assert!(!en_cancha(Vec2::new(0.0, 0.7)));
    }

    #[test]
    fn ruler_marks_by_scale() {
        // 420 px/m = 4.2 px/cm: todas las marcas.
        let m = marcas_regla(150, 4.2);
        assert_eq!(m.len(), 151);
        assert_eq!(m.iter().filter(|(c, _)| c % 25 == 0).count(), 7);
        // 250 px/m = 2.5 px/cm: solo las de 5 y 10.
        let m = marcas_regla(150, 2.5);
        assert_eq!(m.len(), 31);
        assert!(m.iter().all(|(_, k)| *k != MarcaRegla::Uno));
    }

    #[test]
    fn area_matches_the_guard() {
        let arco = arco_area(1.0);
        let primero = arco.first().unwrap();
        let ultimo = arco.last().unwrap();
        assert!((primero.x - AREA_X).abs() < 1e-5 && (primero.y + 0.10).abs() < 1e-5);
        assert!((ultimo.x - AREA_X).abs() < 1e-5 && (ultimo.y - 0.10).abs() < 1e-5);
        let vertice = arco.iter().map(|p| p.x).fold(f32::MAX, f32::min);
        assert!((vertice - 0.55).abs() < 1e-5, "{vertice}");
        assert!((2.0 * AREA_HALF_Y - 0.70).abs() < 1e-6);
        assert!(arco_area(-1.0).iter().all(|p| p.x < 0.0));
    }

    #[test]
    fn ball_scale() {
        let v = Vista::new(Size::new(2000.0, 2000.0), OrientacionVista::Normal);
        let d = 2.0 * v.px(BALL_RADIUS_M) / v.px(1.0) * 400.0;
        assert!((d - 17.08).abs() < 0.01, "a 400 px/m la pelota mide {d} px");
    }

    #[test]
    fn text_width_estimate() {
        assert!((ancho_texto("25 cm", 12.0) - (2.0 * 0.53 + 0.2 + 0.5 + 0.85) * 12.0).abs() < 1e-4);
        assert!(ancho_texto("125", 12.0) > ancho_texto("25", 12.0));
    }

    #[test]
    fn labels() {
        let mut mapa = BTreeMap::new();
        mapa.insert(1, 0);
        mapa.insert(0, 1);
        assert_eq!(rotulo(true, 1, None), "1");
        assert_eq!(rotulo(true, 1, Some(&mapa)), "1 radio 0");
        assert_eq!(rotulo(false, 1, Some(&mapa)), "1", "los rivales no llevan radio");
    }

    #[test]
    fn orientation_parsing() {
        assert_eq!(OrientacionVista::parse("girada180"), Some(OrientacionVista::Girada180));
        assert_eq!(OrientacionVista::parse("ESPEJO-H"), Some(OrientacionVista::EspejoHorizontal));
        assert_eq!(OrientacionVista::parse("diagonal"), None);
    }
}
