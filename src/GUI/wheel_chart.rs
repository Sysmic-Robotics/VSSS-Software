use iced::widget::canvas::{self, Cache, Geometry, Path, Stroke};
use iced::{Point, Rectangle, Theme};
use std::collections::VecDeque;

use super::Message;
use super::theme;

/// Escala vertical del gráfico: ±1500 mm/s (mismo clamp que la base station).
const WHEEL_MAX_MM_S: f32 = 1500.0;

/// Gráfico temporal de la velocidad de rueda comandada L/R (mm/s) del robot
/// seleccionado. Solo lectura; no afecta comandos ni headless.
pub struct WheelChart<'a> {
    /// Muestras `(t_segundos, wheel_l_mm_s, wheel_r_mm_s)`, más viejas al frente.
    pub history: &'a VecDeque<(f64, i16, i16)>,
    /// Ventana de tiempo mostrada (s): solo se dibujan las muestras recientes.
    pub window_s: f64,
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

            // Mapea un valor mm/s a la coordenada Y del gráfico.
            let map_y = |v: f32| -> f32 {
                let clamped = v.clamp(-WHEEL_MAX_MM_S, WHEEL_MAX_MM_S);
                mid_y - (clamped / WHEEL_MAX_MM_S) * (height / 2.0 - 4.0)
            };

            // Fondo + borde
            let background = Path::rectangle(Point::ORIGIN, bounds.size());
            frame.fill(&background, theme::BG_ELEVATED);
            frame.stroke(
                &background,
                Stroke::default().with_width(2.0).with_color(theme::BORDER),
            );

            // Gridlines de referencia: 0 (fuerte), ±750 y ±1500 (tenues).
            for (val, strong) in [
                (0.0_f32, true),
                (750.0, false),
                (-750.0, false),
                (1500.0, false),
                (-1500.0, false),
            ] {
                let y = map_y(val);
                frame.stroke(
                    &Path::line(Point::new(0.0, y), Point::new(width, y)),
                    Stroke::default()
                        .with_width(if strong { 1.2 } else { 1.0 })
                        .with_color(theme::GRID),
                );
            }

            // Etiquetas de eje
            frame.fill_text(canvas::Text {
                content: "+1500".to_string(),
                position: Point::new(4.0, 2.0),
                color: theme::AXIS_TEXT,
                size: (theme::FS_XS as f32).into(),
                ..Default::default()
            });
            frame.fill_text(canvas::Text {
                content: "-1500".to_string(),
                position: Point::new(4.0, height - 14.0),
                color: theme::AXIS_TEXT,
                size: (theme::FS_XS as f32).into(),
                ..Default::default()
            });
            frame.fill_text(canvas::Text {
                content: "Ruedas L/R (mm/s)".to_string(),
                position: Point::new(width / 2.0 - 56.0, 2.0),
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

            // Curva rueda izquierda (cyan) y derecha (magenta).
            for (pick, color) in [
                ((|s: &(f64, i16, i16)| s.1) as fn(&(f64, i16, i16)) -> i16, theme::DATA_L),
                ((|s: &(f64, i16, i16)| s.2) as fn(&(f64, i16, i16)) -> i16, theme::DATA_R),
            ] {
                let path = Path::new(|b| {
                    for (i, sample) in samples.iter().enumerate() {
                        let x = i as f32 * dx;
                        let y = map_y(pick(sample) as f32);
                        if i == 0 {
                            b.move_to(Point::new(x, y));
                        } else {
                            b.line_to(Point::new(x, y));
                        }
                    }
                });
                frame.stroke(&path, Stroke::default().with_width(2.0).with_color(color));
            }

            // Lectura numérica del valor actual (última muestra) L/R.
            let (_, last_l, last_r) = *samples.last().unwrap();
            frame.fill_text(canvas::Text {
                content: format!("L {last_l}"),
                position: Point::new(width - 96.0, 2.0),
                color: theme::DATA_L,
                size: (theme::FS_SM as f32).into(),
                ..Default::default()
            });
            frame.fill_text(canvas::Text {
                content: format!("R {last_r}"),
                position: Point::new(width - 44.0, 2.0),
                color: theme::DATA_R,
                size: (theme::FS_SM as f32).into(),
                ..Default::default()
            });
        });

        vec![geometry]
    }
}
