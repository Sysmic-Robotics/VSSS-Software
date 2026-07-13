use iced::widget::canvas::{self, Cache, Geometry, Path, Stroke};
use iced::{Color, Point, Rectangle, Theme};
use std::collections::VecDeque;

use super::Message;
use super::theme;

/// Gráfico de líneas de dos series `(a, b)` en el tiempo, para el robot seleccionado.
/// Reutilizable para: velocidad comandada vs medida (lineal), ω comandada vs medida
/// (con eje simétrico), y error de heading vs distancia al target. Solo lectura.
pub struct LineChart<'a> {
    /// Muestras `(t_segundos, a, b)`, más viejas al frente.
    pub history: &'a VecDeque<(f64, f32, f32)>,
    /// Ventana de tiempo mostrada (s): solo se dibujan las muestras recientes.
    pub window_s: f64,
    pub title: &'a str,
    pub label_a: &'a str,
    pub color_a: Color,
    pub label_b: &'a str,
    pub color_b: Color,
    /// Si el eje Y es simétrico en torno a 0 (para valores con signo, p. ej. ω o error).
    pub symmetric: bool,
    pub cache: &'a Cache,
}

impl<'a> LineChart<'a> {
    fn empty(&self, frame: &mut canvas::Frame, bounds: Rectangle, msg: &str) {
        let width = bounds.width;
        let height = bounds.height;
        let background = Path::rectangle(Point::ORIGIN, bounds.size());
        frame.fill(&background, theme::BG_ELEVATED);
        frame.stroke(
            &background,
            Stroke::default().with_width(2.0).with_color(theme::BORDER),
        );
        frame.fill_text(canvas::Text {
            content: self.title.to_string(),
            position: Point::new(4.0, 2.0),
            color: theme::AXIS_TEXT,
            size: (theme::FS_XS as f32).into(),
            ..Default::default()
        });
        frame.fill_text(canvas::Text {
            content: msg.to_string(),
            position: Point::new(width / 2.0 - 70.0, height / 2.0),
            color: theme::TEXT_DIM,
            size: (theme::FS_SM as f32).into(),
            ..Default::default()
        });
    }
}

impl<'a> canvas::Program<Message> for LineChart<'a> {
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
            // Muestras dentro de la ventana de tiempo.
            let samples: Vec<(f64, f32, f32)> = match self.history.back() {
                Some(&(t_last, _, _)) => self
                    .history
                    .iter()
                    .filter(|(t, _, _)| *t >= t_last - self.window_s)
                    .copied()
                    .collect(),
                None => Vec::new(),
            };

            if samples.len() < 2 {
                self.empty(frame, bounds, "Sin datos...");
                return;
            }

            let width = bounds.width;
            let height = bounds.height;
            let pad = 4.0;

            // Fondo + borde
            let background = Path::rectangle(Point::ORIGIN, bounds.size());
            frame.fill(&background, theme::BG_ELEVATED);
            frame.stroke(
                &background,
                Stroke::default().with_width(2.0).with_color(theme::BORDER),
            );

            // Rango vertical automático.
            let mut vmax = f32::MIN;
            let mut vmin = f32::MAX;
            for (_, a, b) in &samples {
                vmax = vmax.max(*a).max(*b);
                vmin = vmin.min(*a).min(*b);
            }
            let map_y = |v: f32| -> f32 {
                if self.symmetric {
                    let m = vmax.abs().max(vmin.abs()).max(0.1);
                    let mid = height / 2.0;
                    mid - (v / m) * (height / 2.0 - pad)
                } else {
                    let hi = vmax.max(0.1);
                    let lo = vmin.min(0.0);
                    let span = (hi - lo).max(0.1);
                    (height - pad) - ((v - lo) / span) * (height - 2.0 * pad)
                }
            };

            // Línea de cero (útil cuando hay valores con signo).
            let y0 = map_y(0.0);
            frame.stroke(
                &Path::line(Point::new(0.0, y0), Point::new(width, y0)),
                Stroke::default().with_width(1.0).with_color(theme::GRID),
            );

            let n = samples.len();
            let dx = width / (n - 1) as f32;
            for (pick, color) in [
                ((|s: &(f64, f32, f32)| s.1) as fn(&(f64, f32, f32)) -> f32, self.color_a),
                ((|s: &(f64, f32, f32)| s.2) as fn(&(f64, f32, f32)) -> f32, self.color_b),
            ] {
                let path = Path::new(|b| {
                    for (i, s) in samples.iter().enumerate() {
                        let x = i as f32 * dx;
                        let y = map_y(pick(s));
                        if i == 0 {
                            b.move_to(Point::new(x, y));
                        } else {
                            b.line_to(Point::new(x, y));
                        }
                    }
                });
                frame.stroke(&path, Stroke::default().with_width(2.0).with_color(color));
            }

            // Título
            frame.fill_text(canvas::Text {
                content: self.title.to_string(),
                position: Point::new(4.0, 2.0),
                color: theme::AXIS_TEXT,
                size: (theme::FS_XS as f32).into(),
                ..Default::default()
            });

            // Leyenda + valor actual de cada serie (última muestra).
            let (_, last_a, last_b) = samples[n - 1];
            frame.fill_text(canvas::Text {
                content: format!("{} {:.2}", self.label_a, last_a),
                position: Point::new(width - 150.0, 2.0),
                color: self.color_a,
                size: (theme::FS_XS as f32).into(),
                ..Default::default()
            });
            frame.fill_text(canvas::Text {
                content: format!("{} {:.2}", self.label_b, last_b),
                position: Point::new(width - 74.0, 2.0),
                color: self.color_b,
                size: (theme::FS_XS as f32).into(),
                ..Default::default()
            });
        });

        vec![geometry]
    }
}
