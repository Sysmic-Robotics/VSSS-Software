use iced::widget::canvas::{self, Cache, Geometry, Path, Stroke};
use iced::{Point, Rectangle, Theme};
use std::collections::VecDeque;

use super::Message;
use super::theme;

/// Gráfico temporal de la consigna que LLEGA al robot seleccionado: v (mm/s) y
/// ω (grados/s) del frame `V,W`. Como las unidades difieren, cada serie se escala
/// a su propio tope (`params().robot`): una serie saturada toca el borde.
/// Solo lectura; no afecta comandos ni headless.
pub struct VwChart<'a> {
    /// Muestras `(t_segundos, v_mm_s, w_deg_s)`, más viejas al frente.
    pub history: &'a VecDeque<(f64, i16, i16)>,
    /// Ventana de tiempo mostrada (s): solo se dibujan las muestras recientes.
    pub window_s: f64,
    pub cache: &'a Cache,
}

impl<'a> canvas::Program<Message> for VwChart<'a> {
    type State = ();

    fn draw(
        &self,
        _state: &Self::State,
        renderer: &iced::Renderer,
        _theme: &Theme,
        bounds: Rectangle,
        _cursor: iced::mouse::Cursor,
    ) -> Vec<Geometry> {
        let robot = &crate::params::params().robot;
        let max_v = robot.max_v_mm_s.max(1) as f32;
        let max_w = robot.max_w_deg_s.max(1) as f32;

        let geometry = self.cache.draw(renderer, bounds.size(), |frame| {
            let width = bounds.width;
            let height = bounds.height;
            let mid_y = height / 2.0;

            // Muestras dentro de la ventana de tiempo seleccionada.
            let samples: Vec<(f64, i16, i16)> = match self.history.back() {
                Some(&(t_last, _, _)) => self
                    .history
                    .iter()
                    .filter(|(t, _, _)| *t >= t_last - self.window_s)
                    .copied()
                    .collect(),
                None => Vec::new(),
            };

            // Mapea una fracción del tope (-1..1) a la coordenada Y del gráfico.
            let map_y = |frac: f32| -> f32 { mid_y - frac.clamp(-1.0, 1.0) * (height / 2.0 - 4.0) };

            // Fondo + borde
            let background = Path::rectangle(Point::ORIGIN, bounds.size());
            frame.fill(&background, theme::BG_ELEVATED);
            frame.stroke(
                &background,
                Stroke::default().with_width(2.0).with_color(theme::BORDER),
            );

            // Gridlines de referencia: 0 (fuerte), ±50 % y ±100 % del tope (tenues).
            for (frac, strong) in [
                (0.0_f32, true),
                (0.5, false),
                (-0.5, false),
                (1.0, false),
                (-1.0, false),
            ] {
                let y = map_y(frac);
                frame.stroke(
                    &Path::line(Point::new(0.0, y), Point::new(width, y)),
                    Stroke::default()
                        .with_width(if strong { 1.2 } else { 1.0 })
                        .with_color(theme::GRID),
                );
            }

            // Etiqueta de escalas (cada serie, su tope)
            frame.fill_text(canvas::Text {
                content: format!("v ±{max_v} mm/s · w ±{max_w} °/s"),
                position: Point::new(4.0, 2.0),
                color: theme::AXIS_TEXT,
                size: (theme::FS_XS as f32).into(),
                ..Default::default()
            });

            if samples.len() < 2 {
                frame.fill_text(canvas::Text {
                    content: "Sin datos del robot seleccionado...".to_string(),
                    position: Point::new(width / 2.0 - 110.0, mid_y),
                    color: theme::TEXT_DIM,
                    size: (theme::FS_MD as f32).into(),
                    ..Default::default()
                });
                return;
            }

            let n = samples.len();
            let dx = width / (n - 1) as f32;

            // Curva v (cyan) y ω (magenta), cada una normalizada a su tope.
            for (pick, max, color) in [
                (
                    (|s: &(f64, i16, i16)| s.1) as fn(&(f64, i16, i16)) -> i16,
                    max_v,
                    theme::DATA_L,
                ),
                (
                    (|s: &(f64, i16, i16)| s.2) as fn(&(f64, i16, i16)) -> i16,
                    max_w,
                    theme::DATA_R,
                ),
            ] {
                let path = Path::new(|b| {
                    for (i, sample) in samples.iter().enumerate() {
                        let x = i as f32 * dx;
                        let y = map_y(pick(sample) as f32 / max);
                        if i == 0 {
                            b.move_to(Point::new(x, y));
                        } else {
                            b.line_to(Point::new(x, y));
                        }
                    }
                });
                frame.stroke(&path, Stroke::default().with_width(2.0).with_color(color));
            }

            // Lectura numérica del valor actual (última muestra), en unidades nativas.
            let (_, last_v, last_w) = *samples.last().unwrap();
            frame.fill_text(canvas::Text {
                content: format!("v {last_v}"),
                position: Point::new(width - 104.0, 2.0),
                color: theme::DATA_L,
                size: (theme::FS_SM as f32).into(),
                ..Default::default()
            });
            frame.fill_text(canvas::Text {
                content: format!("w {last_w}"),
                position: Point::new(width - 48.0, 2.0),
                color: theme::DATA_R,
                size: (theme::FS_SM as f32).into(),
                ..Default::default()
            });
        });

        vec![geometry]
    }
}
