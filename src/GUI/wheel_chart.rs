use iced::widget::canvas::{self, Cache, Geometry, Path, Stroke};
use iced::{Color, Point, Rectangle, Theme};
use std::collections::VecDeque;

use super::Message;

/// Escala vertical del gráfico: ±1500 mm/s (mismo clamp que la base station).
const WHEEL_MAX_MM_S: f32 = 1500.0;

/// Gráfico temporal de la velocidad de rueda comandada L/R (mm/s) del robot
/// seleccionado. Solo lectura; no afecta comandos ni headless.
pub struct WheelChart<'a> {
    /// Muestras `(t_segundos, wheel_l_mm_s, wheel_r_mm_s)`, más viejas al frente.
    pub history: &'a VecDeque<(f64, i16, i16)>,
    pub cache: &'a Cache,
}

impl<'a> canvas::Program<Message> for WheelChart<'a> {
    type State = ();

    fn draw(
        &self,
        _state: &Self::State,
        renderer: &iced::Renderer,
        _theme: &Theme,
        bounds: Rectangle,
        _cursor: iced::mouse::Cursor,
    ) -> Vec<Geometry> {
        let geometry = self.cache.draw(renderer, bounds.size(), |frame| {
            let width = bounds.width;
            let height = bounds.height;

            // Fondo + borde
            let background = Path::rectangle(Point::ORIGIN, bounds.size());
            frame.fill(&background, Color::from_rgb(0.1, 0.1, 0.1));
            frame.stroke(
                &background,
                Stroke::default()
                    .with_width(2.0)
                    .with_color(Color::from_rgb(0.3, 0.3, 0.3)),
            );

            // Línea de cero (centro vertical)
            let mid_y = height / 2.0;
            frame.stroke(
                &Path::line(Point::new(0.0, mid_y), Point::new(width, mid_y)),
                Stroke::default()
                    .with_width(1.0)
                    .with_color(Color::from_rgba(0.5, 0.5, 0.5, 0.4)),
            );

            // Etiquetas de eje
            let text_color = Color::from_rgb(0.8, 0.8, 0.8);
            frame.fill_text(canvas::Text {
                content: "+1500".to_string(),
                position: Point::new(4.0, 2.0),
                color: text_color,
                size: 11.0.into(),
                ..Default::default()
            });
            frame.fill_text(canvas::Text {
                content: "-1500".to_string(),
                position: Point::new(4.0, height - 16.0),
                color: text_color,
                size: 11.0.into(),
                ..Default::default()
            });
            frame.fill_text(canvas::Text {
                content: "Ruedas L/R (mm/s)".to_string(),
                position: Point::new(width / 2.0 - 60.0, 2.0),
                color: text_color,
                size: 11.0.into(),
                ..Default::default()
            });

            if self.history.len() < 2 {
                frame.fill_text(canvas::Text {
                    content: "Sin datos del robot seleccionado...".to_string(),
                    position: Point::new(width / 2.0 - 110.0, mid_y),
                    color: Color::from_rgb(0.5, 0.5, 0.5),
                    size: 13.0.into(),
                    ..Default::default()
                });
                return;
            }

            let n = self.history.len();
            let dx = width / (n - 1) as f32;
            let map_y = |v: i16| -> f32 {
                let clamped = (v as f32).clamp(-WHEEL_MAX_MM_S, WHEEL_MAX_MM_S);
                mid_y - (clamped / WHEEL_MAX_MM_S) * (height / 2.0 - 4.0)
            };

            // Curva rueda izquierda (cyan) y derecha (magenta).
            for (pick, color) in [
                ((|s: &(f64, i16, i16)| s.1) as fn(&(f64, i16, i16)) -> i16, Color::from_rgb(0.0, 0.8, 1.0)),
                ((|s: &(f64, i16, i16)| s.2) as fn(&(f64, i16, i16)) -> i16, Color::from_rgb(1.0, 0.3, 0.8)),
            ] {
                let path = Path::new(|b| {
                    for (i, sample) in self.history.iter().enumerate() {
                        let x = i as f32 * dx;
                        let y = map_y(pick(sample));
                        if i == 0 {
                            b.move_to(Point::new(x, y));
                        } else {
                            b.line_to(Point::new(x, y));
                        }
                    }
                });
                frame.stroke(&path, Stroke::default().with_width(2.0).with_color(color));
            }

            // Leyenda
            frame.fill_text(canvas::Text {
                content: "L".to_string(),
                position: Point::new(width - 40.0, 2.0),
                color: Color::from_rgb(0.0, 0.8, 1.0),
                size: 12.0.into(),
                ..Default::default()
            });
            frame.fill_text(canvas::Text {
                content: "R".to_string(),
                position: Point::new(width - 20.0, 2.0),
                color: Color::from_rgb(1.0, 0.3, 0.8),
                size: 12.0.into(),
                ..Default::default()
            });
        });

        vec![geometry]
    }
}
