//! Gráficos de la GUI.
//!
//! - **Abajo:** "Velocidad del robot N"
//!   y "Giro del robot N", con lo pedido (la consigna del frame, de `command_to_vw`) en
//!   azul y lo medido por la visión en tinta. Cada uno va escalado a su tope
//!   (`robot.max_v_mm_s`, `robot.max_w_deg_s`): una consigna saturada toca el borde.
//! - **Inspector:** error de heading y distancia al target de la skill, y cuadros por
//!   segundo de la visión.
//!
//! Solo lectura: no afectan comandos ni el modo headless. Una muestra no finita (NaN o
//! infinito) corta la línea: lyon rechaza los puntos no finitos al armar un `Path`.

use super::format::con_unidad;
use super::{DebugGui, Message, TELEMETRY_WINDOWS_S, fonts, theme};
use iced::widget::canvas::{self, Cache, Path, Stroke};
use iced::widget::{Canvas, button, column, container, horizontal_space, row, text};
use iced::{Alignment, Color, Element, Length, Point, Rectangle, Renderer, Theme, mouse};
use std::collections::VecDeque;

/// Una muestra de una señal pedida/medida. `None` = no había dato en esa foto.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MuestraSenal {
    /// Segundos (reloj del lazo).
    pub t: f64,
    pub pedida: Option<f32>,
    pub medida: Option<f32>,
}

/// Muestras dentro de la ventana que termina en la última.
fn en_ventana<T>(serie: &VecDeque<T>, ventana_s: f64, t: impl Fn(&T) -> f64) -> (f64, Vec<&T>) {
    let fin = serie.back().map(&t).unwrap_or(0.0);
    let inicio = fin - ventana_s;
    (inicio, serie.iter().filter(|m| t(*m) >= inicio).collect())
}

/// Filetes en 0 (más marcado), ±50 % y ±100 %.
fn filetes(frame: &mut canvas::Frame, w: f32, map_y: impl Fn(f32) -> f32) {
    for (frac, fuerte) in [(0.0, true), (0.5, false), (-0.5, false), (1.0, false), (-1.0, false)] {
        let y = map_y(frac);
        frame.stroke(
            &Path::line(Point::new(0.0, y), Point::new(w, y)),
            Stroke::default()
                .with_width(if fuerte { 1.2 } else { 0.8 })
                .with_color(theme::RAYA),
        );
    }
}

/// Señal pedida y medida, escalada a `tope`.
pub struct SenalChart<'a> {
    pub serie: &'a VecDeque<MuestraSenal>,
    pub tope: f32,
    pub ventana_s: f64,
    pub cache: &'a Cache,
}

impl canvas::Program<Message> for SenalChart<'_> {
    type State = ();

    fn draw(
        &self,
        _state: &Self::State,
        renderer: &Renderer,
        _theme: &Theme,
        bounds: Rectangle,
        _cursor: mouse::Cursor,
    ) -> Vec<canvas::Geometry> {
        let g = self.cache.draw(renderer, bounds.size(), |frame| {
            let (w, h) = (bounds.width, bounds.height);
            let map_y = |frac: f32| h / 2.0 - frac.clamp(-1.0, 1.0) * (h / 2.0 - 3.0);
            filetes(frame, w, map_y);
            let (inicio, muestras) = en_ventana(self.serie, self.ventana_s, |m| m.t);
            let map_x = |t: f64| ((t - inicio) / self.ventana_s) as f32 * w;
            let tope = self.tope.max(1e-6);
            for (elegir, color, ancho) in [
                ((|m: &MuestraSenal| m.pedida) as fn(&MuestraSenal) -> Option<f32>, theme::AZUL, 2.0),
                ((|m: &MuestraSenal| m.medida) as fn(&MuestraSenal) -> Option<f32>, theme::TINTA, 1.4),
            ] {
                let camino = Path::new(|b| {
                    let mut abierto = false;
                    for m in &muestras {
                        match elegir(m).filter(|v| v.is_finite()) {
                            Some(v) => {
                                let p = Point::new(map_x(m.t), map_y(v / tope));
                                if abierto {
                                    b.line_to(p);
                                } else {
                                    b.move_to(p);
                                    abierto = true;
                                }
                            }
                            None => abierto = false,
                        }
                    }
                });
                frame.stroke(&camino, Stroke::default().with_width(ancho).with_color(color));
            }
        });
        vec![g]
    }
}

/// Dos series en el tiempo con eje simétrico automático (error de la skill: error de
/// heading en rad y distancia al target en m).
pub struct LineChart<'a> {
    /// Muestras `(t_s, a, b)`.
    pub history: &'a VecDeque<(f64, f32, f32)>,
    pub ventana_s: f64,
    pub color_a: Color,
    pub color_b: Color,
    pub cache: &'a Cache,
}

impl canvas::Program<Message> for LineChart<'_> {
    type State = ();

    fn draw(
        &self,
        _state: &Self::State,
        renderer: &Renderer,
        _theme: &Theme,
        bounds: Rectangle,
        _cursor: mouse::Cursor,
    ) -> Vec<canvas::Geometry> {
        let g = self.cache.draw(renderer, bounds.size(), |frame| {
            let (w, h) = (bounds.width, bounds.height);
            let (inicio, muestras) = en_ventana(self.history, self.ventana_s, |m| m.0);
            let escala = muestras
                .iter()
                .flat_map(|(_, a, b)| [*a, *b])
                .filter(|v| v.is_finite())
                .fold(0.1_f32, |e, v| e.max(v.abs()));
            let map_y = |v: f32| h / 2.0 - (v / escala).clamp(-1.0, 1.0) * (h / 2.0 - 3.0);
            frame.stroke(
                &Path::line(Point::new(0.0, h / 2.0), Point::new(w, h / 2.0)),
                Stroke::default().with_width(1.0).with_color(theme::RAYA),
            );
            let map_x = |t: f64| ((t - inicio) / self.ventana_s) as f32 * w;
            for (elegir, color) in [
                ((|m: &&(f64, f32, f32)| m.1) as fn(&&(f64, f32, f32)) -> f32, self.color_a),
                ((|m: &&(f64, f32, f32)| m.2) as fn(&&(f64, f32, f32)) -> f32, self.color_b),
            ] {
                let camino = Path::new(|b| {
                    let mut abierto = false;
                    for m in &muestras {
                        let v = elegir(m);
                        if !v.is_finite() {
                            abierto = false;
                            continue;
                        }
                        let p = Point::new(map_x(m.0), map_y(v));
                        if abierto {
                            b.line_to(p);
                        } else {
                            b.move_to(p);
                            abierto = true;
                        }
                    }
                });
                frame.stroke(&camino, Stroke::default().with_width(1.6).with_color(color));
            }
        });
        vec![g]
    }
}

/// Cuadros por segundo de la visión, una barra por segundo (últimos 60 s).
pub struct CuadrosChart<'a> {
    /// `(segundo, cuadros en ese segundo)`.
    pub history: &'a VecDeque<(u64, u64)>,
    pub cache: &'a Cache,
}

impl canvas::Program<Message> for CuadrosChart<'_> {
    type State = ();

    fn draw(
        &self,
        _state: &Self::State,
        renderer: &Renderer,
        _theme: &Theme,
        bounds: Rectangle,
        _cursor: mouse::Cursor,
    ) -> Vec<canvas::Geometry> {
        let g = self.cache.draw(renderer, bounds.size(), |frame| {
            let (w, h) = (bounds.width, bounds.height);
            let tope = self.history.iter().map(|(_, n)| *n).max().unwrap_or(0).max(70) as f32;
            let y = |n: f32| h - n / tope * (h - 2.0);
            // Referencia de 60 cuadros/s.
            frame.stroke(
                &Path::line(Point::new(0.0, y(60.0)), Point::new(w, y(60.0))),
                Stroke::default().with_width(0.8).with_color(theme::RAYA),
            );
            let ancho = w / 60.0;
            let n = self.history.len();
            for (i, (_, cuadros)) in self.history.iter().enumerate() {
                let x = w - (n - i) as f32 * ancho;
                let barra = Path::rectangle(
                    Point::new(x, y(*cuadros as f32)),
                    iced::Size::new((ancho - 1.0).max(1.0), h - y(*cuadros as f32)),
                );
                frame.fill(&barra, theme::alfa(theme::TINTA, 0.45));
            }
        });
        vec![g]
    }
}

impl DebugGui {
    /// Gráficos de velocidad y giro del robot seleccionado.
    pub(super) fn vista_graficos(&self) -> Element<'_, Message> {
        let mut controles = row![].spacing(theme::SP_XS).align_y(Alignment::Center);
        controles = controles.push(
            button(text(if self.telemetry_frozen { "seguir" } else { "congelar" }).size(theme::TXT_CHICO))
                .padding([2, 8])
                .style(theme::boton_opcion(self.telemetry_frozen))
                .on_press(Message::ToggleTelemetryFreeze(!self.telemetry_frozen)),
        );
        for s in TELEMETRY_WINDOWS_S {
            let activa = (self.telemetry_window_s - s).abs() < 1e-6;
            controles = controles.push(
                button(text(format!("{s:.0} s")).size(theme::TXT_CHICO))
                    .padding([2, 8])
                    .style(theme::boton_opcion(activa))
                    .on_press(Message::SetTelemetryWindow(s)),
            );
        }
        row![
            self.panel_senal(false, Some(controles.into())),
            self.panel_senal(true, None),
        ]
        .spacing(theme::SP_MD)
        .height(Length::Fill)
        .into()
    }

    /// Un gráfico de abajo: velocidad (`giro = false`) o giro.
    fn panel_senal<'a>(&'a self, giro: bool, extra: Option<Element<'a, Message>>) -> Element<'a, Message> {
        let robot = &crate::params::params().robot;
        let id = self.selected_robot;
        let (titulo, serie, cache, tope, unidad) = if giro {
            (
                format!("Giro del robot {id}"),
                &self.serie_w,
                &self.cache_w,
                (robot.max_w_deg_s.max(1) as f32).to_radians(),
                "rad/s",
            )
        } else {
            (
                format!("Velocidad del robot {id}"),
                &self.serie_v,
                &self.cache_v,
                robot.max_v_mm_s.max(1) as f32 / 1000.0,
                "m/s",
            )
        };
        let leyenda = |valor: Option<f32>, color: Color, nombre: &'static str| {
            let v = valor
                .map(|x| format!("{nombre} {}", con_unidad(x, 2, unidad)))
                .unwrap_or_else(|| nombre.to_string());
            text(v)
                .size(theme::TXT_CHICO)
                .style(move |_t: &Theme| text::Style { color: Some(color) })
        };
        let u = serie.back().copied();
        let mut cabeza = row![
            text(titulo).font(fonts::TITULO).size(theme::TXT_TITULO),
            horizontal_space(),
        ]
        .spacing(theme::SP_LG)
        .align_y(Alignment::Center);
        if let Some(e) = extra {
            cabeza = cabeza.push(e);
        }
        let leyendas = row![
            leyenda(u.and_then(|m| m.pedida), theme::AZUL, "pedida"),
            leyenda(u.and_then(|m| m.medida), theme::TINTA, "medida"),
        ]
        .spacing(theme::SP_LG);
        let grafico = Canvas::new(SenalChart {
            serie,
            tope,
            ventana_s: self.telemetry_window_s,
            cache,
        })
        .width(Length::Fill)
        .height(Length::Fill);
        container(column![cabeza, leyendas, grafico].spacing(2).padding([6, 12]))
            .width(Length::Fill)
            .height(Length::Fill)
            .style(theme::panel)
            .into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn window_keeps_the_last_seconds() {
        let mut serie = VecDeque::new();
        for i in 0..100 {
            serie.push_back(MuestraSenal {
                t: i as f64 * 0.1,
                pedida: Some(1.0),
                medida: None,
            });
        }
        let (inicio, m) = en_ventana(&serie, 5.0, |m| m.t);
        assert!((inicio - 4.9).abs() < 1e-9);
        assert_eq!(m.len(), 51);
    }

    #[test]
    fn empty_series_has_an_empty_window() {
        let serie: VecDeque<MuestraSenal> = VecDeque::new();
        let (_, m) = en_ventana(&serie, 5.0, |m| m.t);
        assert!(m.is_empty());
    }
}
