//! Pestaña **Inspector**: el robot
//! seleccionado, la salud del sistema en detalle, la visión y la radio (de solo
//! lectura: se configuran al arrancar) y el error de la skill.

use super::charts::{CuadrosChart, LineChart};
use super::format::{con_unidad, num, pct};
use super::robot_rows::NO_VISTO_S;
use super::{DebugGui, Message, fonts, theme};
use crate::radio::RadioTarget;
use crate::vision::VisionSource;
use iced::widget::{Canvas, Column, column, horizontal_rule, row, text};
use iced::{Element, Length};

fn titulo(t: impl ToString) -> Element<'static, Message> {
    text(t.to_string())
        .font(fonts::TITULO)
        .size(theme::TXT_TITULO)
        .into()
}

/// Una línea "etiqueta valor", con el valor en alerta si corresponde.
fn dato(etiqueta: &'static str, valor: impl ToString, alerta: bool) -> Element<'static, Message> {
    row![
        text(etiqueta)
            .size(theme::TXT_CHICO)
            .width(Length::Fixed(92.0))
            .style(theme::gris),
        text(valor.to_string())
            .size(theme::TXT_CHICO)
            .style(theme::tinta_o_alerta(alerta)),
    ]
    .spacing(theme::SP_SM)
    .into()
}

fn nota(t: impl ToString) -> Element<'static, Message> {
    text(t.to_string())
        .size(theme::TXT_CHICO)
        .style(theme::gris)
        .into()
}

fn separador<'a>() -> Element<'a, Message> {
    horizontal_rule(1).style(theme::filete).into()
}

impl DebugGui {
    pub(super) fn pestana_inspector(&self) -> Element<'_, Message> {
        column![
            self.seccion_robot(),
            separador(),
            self.seccion_salud(),
            separador(),
            self.seccion_vision(),
            separador(),
            self.seccion_radio(),
            separador(),
            self.seccion_error_skill(),
        ]
        .spacing(theme::SP_MD)
        .into()
    }

    fn seccion_robot(&self) -> Element<'_, Message> {
        let equipo = if self.selected_team == 0 { "azul" } else { "amarillo" };
        let mut col = Column::new()
            .spacing(theme::SP_XS)
            .push(titulo(format!("Robot {} {equipo}", self.selected_robot)));
        match self.snap.robot(self.selected_team as i32, self.selected_robot as i32) {
            Some(r) => {
                let activo = r.active && r.age_s <= NO_VISTO_S;
                col = col
                    .push(dato(
                        "posición",
                        format!("({}, {}) m", num(r.position.x, 2), num(r.position.y, 2)),
                        false,
                    ))
                    .push(dato(
                        "orientación",
                        format!("{}°", num((r.orientation as f32).to_degrees(), 0)),
                        false,
                    ))
                    .push(dato("rapidez", con_unidad(r.velocity.length(), 2, "m/s"), false))
                    .push(dato("giro", con_unidad(r.angular_velocity as f32, 2, "rad/s"), false))
                    .push(dato("estado", if activo { "activo" } else { "inactivo" }, !activo))
                    .push(dato("antigüedad", con_unidad(r.age_s, 2, "s"), !activo));
            }
            None => col = col.push(nota("No hay datos de este robot.")),
        }
        col.into()
    }

    fn seccion_salud(&self) -> Element<'_, Message> {
        let t = self.salud.tasas();
        let s = &self.snap;
        let lazo = if s.timing.samples > 0 {
            format!(
                "{} Hz, jitter {} ms, peor {} ms",
                num(s.timing.hz, 1),
                num(s.timing.jitter_ms, 1),
                num(s.timing.worst_ms, 1)
            )
        } else {
            "sin datos todavía".to_string()
        };
        let vision = match (t.cuadros_s, s.vision.latency_ms) {
            (Some(f), Some(l)) => format!("{} cuadros/s, latencia {} ms", num(f, 0), num(l, 0)),
            (Some(f), None) => format!("{} cuadros/s, sin dato de latencia", num(f, 0)),
            (None, _) => "sin datos todavía".to_string(),
        };
        let perdidas = match t.perdidas {
            Some(p) => format!(
                "{} % (descartados por el proxy desde el arranque: {})",
                num(p * 100.0, 1),
                s.vision.proxy_dropped
            ),
            None => "sin datos todavía".to_string(),
        };
        let radio = match (s.radio.last_ok, t.envios_s) {
            (Some(false), _) => "el último envío falló".to_string(),
            (Some(true), Some(e)) => format!("{} envíos/s, el último envío salió bien", num(e, 0)),
            _ => "sin envíos todavía".to_string(),
        };
        let mut col = column![
            titulo("Salud"),
            dato(
                "lazo",
                lazo,
                s.timing.samples > 0
                    && (s.timing.hz < super::status_line::LAZO_MIN_HZ
                        || s.timing.worst_ms > super::status_line::LAZO_MAX_INTERVALO_MS)
            ),
            dato("GUI", format!("{} cuadros/s", num(self.fps_gui.valor(), 0)), false),
            dato("visión", vision, false),
            dato("pérdidas", perdidas, t.perdidas.is_some_and(|p| p > super::status_line::PERDIDAS_ALERTA)),
            dato("radio", radio, s.radio.last_ok == Some(false)),
        ]
        .spacing(theme::SP_XS);
        if let Some(v) = t.velocidad_sim {
            col = col.push(dato(
                "simulador",
                format!("FIRASim al {} del tiempo real", pct(v)),
                !(super::status_line::SIM_MIN..=super::status_line::SIM_MAX).contains(&v),
            ));
        }
        col.into()
    }

    fn seccion_vision(&self) -> Element<'_, Message> {
        let src = self.run.vision_source;
        let fuente = match src {
            VisionSource::FiraSim => "FIRASim",
            VisionSource::SslVision => "cámara real (vsss-vision-sysmic)",
        };
        let vistos = self
            .snap
            .robots
            .iter()
            .filter(|r| r.active && r.age_s <= NO_VISTO_S)
            .count();
        column![
            titulo("Visión"),
            dato(
                "fuente",
                format!("{fuente} en {}:{}", src.multicast_ip(), src.port()),
                false
            ),
            dato(
                "filtro de Kalman",
                if self.run.tracker_on { "encendido" } else { "apagado (VSSL_TRACKER)" },
                !self.run.tracker_on
            ),
            dato(
                "ruido simulado",
                if self.run.noise_proxy { "activo (VSSL_VISION_NOISE)" } else { "apagado" },
                self.run.noise_proxy && src == VisionSource::SslVision
            ),
            dato("robots vistos", vistos, false),
            nota("Cuadros por segundo, último minuto:"),
            Canvas::new(CuadrosChart {
                history: &self.cuadros_hist,
                cache: &self.cache_cuadros,
            })
            .width(Length::Fill)
            .height(Length::Fixed(70.0)),
            nota("La fuente y el filtro se eligen al arrancar (VSSL_VISION_SOURCE, VSSL_TRACKER)."),
        ]
        .spacing(theme::SP_XS)
        .into()
    }

    fn seccion_radio(&self) -> Element<'_, Message> {
        let transporte = match self.run.transport {
            RadioTarget::FiraSim => "FIRASim",
            RadioTarget::GrSim => "grSim",
            RadioTarget::BaseStation => "base station",
        };
        let estado = match self.snap.radio.last_ok {
            Some(true) => "el último envío salió bien",
            Some(false) => "el último envío falló",
            None => "sin envíos todavía",
        };
        let mut col = Column::new()
            .spacing(theme::SP_XS)
            .push(titulo("Radio"))
            .push(dato("transporte", transporte, false))
            .push(dato("estado", estado, self.snap.radio.last_ok == Some(false)))
            .push(dato(
                "equipo propio",
                if self.run.own_team == 0 { "azul (VSSL_TEAM_COLOR)" } else { "amarillo (VSSL_TEAM_COLOR)" },
                false,
            ));
        if self.run.is_base_station() {
            col = col
                .push(dato("puerto", &self.run.radio_port, false))
                .push(dato("baud", &self.run.radio_baud, false))
                .push(nota(
                    "Se fijan al arrancar con VSSL_BASESTATION_DEVICE y VSSL_BASESTATION_BAUD.",
                ))
                .push(nota("Mapeo de visión a radio (el número de robot de la GUI es el de visión):"));
            // Las líneas son las del log de la base; la flecha no está en Barlow.
            for linea in crate::radio::base_station::describe_slot_map(&self.run.slot_map) {
                col = col.push(text(linea.replace(" \u{2192} ", " va a la ")).size(theme::TXT_CHICO));
            }
        }
        col.into()
    }

    fn seccion_error_skill(&self) -> Element<'_, Message> {
        column![
            titulo("Error de la skill"),
            row![
                text("error de heading (rad)")
                    .size(theme::TXT_CHICO)
                    .style(|_t: &iced::Theme| text::Style { color: Some(theme::AZUL) }),
                text("distancia al target (m)").size(theme::TXT_CHICO),
            ]
            .spacing(theme::SP_LG),
            Canvas::new(LineChart {
                history: &self.skill_err_history,
                ventana_s: self.telemetry_window_s,
                color_a: theme::AZUL,
                color_b: theme::TINTA,
                cache: &self.cache_err,
            })
            .width(Length::Fill)
            .height(Length::Fixed(90.0)),
            if self.skill_err_history.is_empty() {
                nota("Sin target para el robot seleccionado.")
            } else {
                column![].into()
            },
        ]
        .spacing(theme::SP_XS)
        .into()
    }
}
