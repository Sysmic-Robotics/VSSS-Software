//! Planilla de robots: una fila por robot propio con su parche,
//! rol, skill y progreso, cara, v y ω pedidos y medidos, y avisos; los rivales en una
//! línea. Un click en la fila selecciona el robot.
//!
//! Lo pedido es el `(v_mm_s, w_deg_s)` de `command_to_vw` (lo mismo que viaja en el
//! frame); lo medido, la velocidad de visión proyectada sobre el heading, con signo.

use super::format::{con_unidad, lista};
use super::run_info::RunInfo;
use super::tag::ParcheCanvas;
use super::{DebugGui, Message, fonts, theme};
use crate::coach::Role;
use crate::skills::Face;
use crate::snapshot::{CommandSource, LoopSnapshot, OwnRobotView, RobotView};
use iced::widget::{
    Canvas, Column, button, container, horizontal_rule, horizontal_space, row, scrollable, text,
};
use iced::{Alignment, Element, Length, Padding};

/// Un robot sin dato de visión por más que esto, "no lo vemos".
pub const NO_VISTO_S: f32 = 0.5;

/// Si el robot se ve ahora (activo y con dato reciente).
pub fn visible(r: Option<&RobotView>) -> bool {
    r.is_some_and(|r| r.active && r.age_s <= NO_VISTO_S)
}

pub fn nombre_rol(rol: Role) -> &'static str {
    match rol {
        Role::Keeper => "arquero",
        Role::Striker => "atacante",
        Role::Support => "apoyo",
    }
}

/// Texto al lado del nombre: el rol (o la fuente del comando si no es el coach) y la
/// posición de radio.
pub fn al_lado(vista: Option<&OwnRobotView>, radio: Option<usize>) -> String {
    let quien = vista.and_then(|v| match v.source {
        Some(CommandSource::Coach) => v.role.map(nombre_rol),
        Some(CommandSource::GuiSkill) => Some("skill de la GUI"),
        Some(CommandSource::Manual) => Some("control manual"),
        Some(CommandSource::Halt) => Some("detenido"),
        Some(CommandSource::Stop) => Some("STOP del árbitro"),
        None => v.role.map(nombre_rol),
    });
    let radio = radio.map(|p| format!("radio {p}"));
    [quien.map(str::to_string), radio]
        .into_iter()
        .flatten()
        .collect::<Vec<_>>()
        .join(", ")
}

/// Datos de una fila; un dato que no existe en el tick se omite.
#[derive(Debug, Clone, PartialEq)]
pub struct DatosFila {
    pub titulo: String,
    pub al_lado: String,
    pub skill: Option<String>,
    pub cara: Option<&'static str>,
    pub pedido: Option<String>,
    pub medido: Option<String>,
    pub avisos: Vec<String>,
}

/// v en m/s y ω en rad/s: "−0.62 m/s, 1.40 rad/s".
fn par(v_m_s: f32, w_rad_s: f32) -> String {
    format!("{}, {}", con_unidad(v_m_s, 2, "m/s"), con_unidad(w_rad_s, 2, "rad/s"))
}

/// Avisos (en alerta) de un robot propio. `skill_warning` es el aviso del lazo, que
/// se pasa solo para el robot con la skill de la GUI activa.
pub fn avisos_robot(
    vista: Option<&OwnRobotView>,
    robot: Option<&RobotView>,
    skill_warning: Option<&str>,
) -> Vec<String> {
    let mut out = Vec::new();
    if let Some(c) = vista.and_then(|v| v.command) {
        if c.v_clamped {
            out.push("v recortada al tope".to_string());
        }
        if c.w_clamped {
            out.push("giro recortado al tope".to_string());
        }
    }
    if let Some(w) = skill_warning {
        out.push(w.to_string());
    }
    if !visible(robot) {
        out.push(match robot.filter(|r| r.age_s.is_finite()) {
            Some(r) => format!("no lo vemos hace {} s", super::format::num(r.age_s, 1)),
            None => "no lo vemos".to_string(),
        });
    }
    if vista.is_some_and(|v| v.escaping) {
        out.push("escapando de un atasco".to_string());
    }
    out
}

/// Datos de la fila del robot propio `id`.
pub fn datos_fila(id: i32, run: &RunInfo, snap: &LoopSnapshot, skill_warning: Option<&str>) -> DatosFila {
    let vista = snap.own_robot(id);
    let robot = snap.robot(run.own_team, id);
    let skill = vista.and_then(|v| {
        v.skill.map(|s| match v.status {
            Some(st) => format!("{s:?}, {:.0} %", (st.progress.clamp(0.0, 1.0) * 100.0).round()),
            None => format!("{s:?}"),
        })
    });
    let cara = vista
        .filter(|v| v.skill.is_some())
        .and_then(|v| v.status)
        .map(|st| match st.face {
            Face::Front => "de frente",
            Face::Back => "de espaldas",
        });
    let pedido = vista
        .and_then(|v| v.command)
        .map(|c| par(f32::from(c.v_mm_s) / 1000.0, f32::from(c.w_deg_s).to_radians()));
    let medido = robot
        .filter(|r| visible(Some(*r)))
        .map(|r| (r.forward_speed(), r.angular_velocity as f32))
        .filter(|(v, w)| v.is_finite() && w.is_finite())
        .map(|(v, w)| par(v, w));
    DatosFila {
        titulo: format!("Robot {id}"),
        al_lado: al_lado(vista, run.radio_slot(id)),
        skill,
        cara,
        pedido,
        medido,
        avisos: avisos_robot(vista, robot, skill_warning),
    }
}

/// Resumen de los rivales que ve la visión.
pub fn linea_rivales(snap: &LoopSnapshot, own_team: i32, team_size: usize) -> String {
    let mut ids: Vec<i32> = snap
        .robots
        .iter()
        .filter(|r| r.team != own_team && visible(Some(*r)))
        .map(|r| r.id)
        .collect();
    ids.sort();
    match ids.len() {
        0 => "No vemos rivales".to_string(),
        1 => format!("Vemos 1 rival: el {}", ids[0]),
        n if n == team_size => format!("Vemos a los {n} rivales: {}", lista(&ids)),
        n => format!("Vemos {n} rivales: {}", lista(&ids)),
    }
}

impl DebugGui {
    /// Planilla de la izquierda.
    pub(super) fn vista_robots(&self) -> Element<'_, Message> {
        let mut filas = Column::new().push(
            container(
                text("Nuestros robots")
                    .font(fonts::TITULO)
                    .size(theme::TXT_TITULO),
            )
            .padding([0, 12]),
        );
        for id in 0..self.run.num_robots as i32 {
            let seleccionado =
                self.selected_team as i32 == self.run.own_team && self.selected_robot as i32 == id;
            let aviso_skill = (seleccionado && self.active_skill.is_some())
                .then_some(self.snap.skill_warning.as_deref())
                .flatten();
            let d = datos_fila(id, &self.run, &self.snap, aviso_skill);
            filas = filas
                .push(horizontal_rule(1).style(theme::filete))
                .push(self.fila_robot(id, d, seleccionado));
        }
        filas = filas
            .push(horizontal_rule(1).style(theme::filete))
            .push(
                container(text("Rivales").font(fonts::TITULO).size(theme::TXT_TITULO)).padding(
                    Padding {
                        top: theme::SP_MD,
                        left: 12.0,
                        ..Padding::ZERO
                    },
                ),
            )
            .push(
                container(
                    text(linea_rivales(&self.snap, self.run.own_team, self.run.num_robots))
                        .size(theme::TXT_CHICO)
                        .style(theme::gris),
                )
                .padding([0, 12]),
            );
        let barra = scrollable::Scrollbar::new().width(4).scroller_width(4).margin(1);
        container(
            scrollable(filas.spacing(theme::SP_SM).padding([12, 0]))
                .direction(scrollable::Direction::Vertical(barra)),
        )
            .width(Length::Fill)
            .height(Length::Fill)
            .style(theme::panel)
            .into()
    }

    fn fila_robot(&self, id: i32, d: DatosFila, seleccionado: bool) -> Element<'_, Message> {
        let dato = |etiqueta: &'static str, valor: String| {
            row![
                text(etiqueta)
                    .size(theme::TXT_CHICO)
                    .width(Length::Fixed(46.0))
                    .style(theme::gris),
                text(valor).size(theme::TXT_CHICO),
            ]
            .spacing(theme::SP_SM)
        };
        let mut datos = Column::new().spacing(1.0).push(
            row![
                text(d.titulo).font(fonts::TITULO).size(theme::TXT_NOMBRE),
                horizontal_space(),
                text(d.al_lado).size(theme::TXT_CHICO).style(theme::gris),
            ]
            .align_y(Alignment::Center),
        );
        let campos = [
            ("skill", d.skill),
            ("cara", d.cara.map(str::to_string)),
            ("pedido", d.pedido),
            ("medido", d.medido),
        ];
        for (etiqueta, valor) in campos {
            if let Some(v) = valor {
                datos = datos.push(dato(etiqueta, v));
            }
        }
        for aviso in d.avisos {
            datos = datos.push(text(aviso).size(theme::TXT_CHICO).style(theme::alerta));
        }

        let parche = Canvas::new(ParcheCanvas {
            team: self.run.own_team,
            id,
            marcas: self.marcas,
        })
        .width(Length::Fixed(34.0))
        .height(Length::Fixed(34.0));

        let fila = button(row![parche, datos.width(Length::Fill)].spacing(theme::SP_MD))
            .padding([8, 12])
            .width(Length::Fill)
            .style(theme::fila(seleccionado))
            .on_press(Message::SelectRow(id as u32));
        // La barra de 3 px de la seleccionada es el fondo tinta que asoma a la izquierda.
        let borde = if seleccionado { 3.0 } else { 0.0 };
        container(fila)
            .padding(Padding {
                left: borde,
                ..Padding::ZERO
            })
            .style(theme::barra_seleccion(seleccionado))
            .into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::GUI::run_info::run_info_de_prueba;
    use crate::radio::RadioTarget;
    use crate::skills::{SkillId, SkillStatus};
    use crate::snapshot::CommandView;
    use glam::Vec2;

    fn robot(team: i32, id: i32, age_s: f32) -> RobotView {
        RobotView {
            team,
            id,
            position: Vec2::ZERO,
            orientation: 0.0,
            velocity: Vec2::new(0.58, 0.0),
            angular_velocity: 1.2,
            active: true,
            age_s,
        }
    }

    fn comando(v_mm_s: i16, w_deg_s: i16) -> CommandView {
        CommandView {
            vx: 0.0,
            vy: 0.0,
            omega: 0.0,
            v_mm_s,
            w_deg_s,
            v_clamped: false,
            w_clamped: false,
        }
    }

    fn vista(id: i32, source: CommandSource) -> OwnRobotView {
        OwnRobotView {
            id,
            source: Some(source),
            role: None,
            skill: None,
            status: None,
            target: None,
            command: Some(comando(0, 0)),
            escaping: false,
        }
    }

    #[test]
    fn striker_with_a_tactical_skill() {
        let mut v = vista(0, CommandSource::Coach);
        v.role = Some(Role::Striker);
        v.skill = Some(SkillId::ApproachAligned);
        v.status = Some(SkillStatus {
            progress: 0.72,
            done: false,
            feasible: true,
            face: Face::Back,
        });
        v.command = Some(comando(-620, 80));
        let snap = LoopSnapshot {
            own: vec![v],
            robots: vec![robot(0, 0, 0.0)],
            ..LoopSnapshot::default()
        };
        let d = datos_fila(0, &run_info_de_prueba(), &snap, None);
        assert_eq!(d.titulo, "Robot 0");
        assert_eq!(d.al_lado, "atacante");
        assert_eq!(d.skill.as_deref(), Some("ApproachAligned, 72 %"));
        assert_eq!(d.cara, Some("de espaldas"));
        assert_eq!(d.pedido.as_deref(), Some("\u{2212}0.62 m/s, 1.40 rad/s"));
        assert_eq!(d.medido.as_deref(), Some("0.58 m/s, 1.20 rad/s"));
        assert!(d.avisos.is_empty(), "{:?}", d.avisos);
    }

    #[test]
    fn manual_control_replaces_the_role_and_shows_no_skill() {
        let snap = LoopSnapshot {
            own: vec![vista(0, CommandSource::Coach), vista(1, CommandSource::Manual)],
            robots: vec![robot(0, 1, 0.0)],
            ..LoopSnapshot::default()
        };
        let d = datos_fila(1, &run_info_de_prueba(), &snap, None);
        assert_eq!(d.al_lado, "control manual");
        assert_eq!(d.skill, None);
    }

    #[test]
    fn base_station_shows_the_radio_position() {
        let mut run = run_info_de_prueba();
        run.transport = RadioTarget::BaseStation;
        run.slot_map.insert(1, 0);
        run.slot_map.insert(0, 1);
        let mut v = vista(1, CommandSource::Coach);
        v.role = Some(Role::Support);
        let snap = LoopSnapshot {
            own: vec![v],
            ..LoopSnapshot::default()
        };
        assert_eq!(datos_fila(1, &run, &snap, None).al_lado, "apoyo, radio 0");
    }

    #[test]
    fn robot_not_seen() {
        let snap = LoopSnapshot {
            robots: vec![robot(0, 2, 2.3)],
            ..LoopSnapshot::default()
        };
        let d = datos_fila(2, &run_info_de_prueba(), &snap, None);
        assert!(d.avisos.contains(&"no lo vemos hace 2.3 s".to_string()), "{:?}", d.avisos);
        assert_eq!(d.medido, None);
        let d = datos_fila(1, &run_info_de_prueba(), &snap, None);
        assert!(d.avisos.contains(&"no lo vemos".to_string()));
    }

    #[test]
    fn clamped_command() {
        let mut v = vista(0, CommandSource::Coach);
        v.command = Some(CommandView {
            v_clamped: true,
            ..comando(1500, 0)
        });
        let snap = LoopSnapshot {
            own: vec![v],
            robots: vec![robot(0, 0, 0.0)],
            ..LoopSnapshot::default()
        };
        let d = datos_fila(0, &run_info_de_prueba(), &snap, None);
        assert!(d.avisos.contains(&"v recortada al tope".to_string()));
        assert_eq!(d.pedido.as_deref(), Some("1.50 m/s, 0.00 rad/s"));
    }

    #[test]
    fn escape_and_skill_warning() {
        let mut v = vista(1, CommandSource::GuiSkill);
        v.escaping = true;
        let snap = LoopSnapshot {
            own: vec![v],
            robots: vec![robot(0, 1, 0.0)],
            ..LoopSnapshot::default()
        };
        let d = datos_fila(1, &run_info_de_prueba(), &snap, Some("la skill no corre: x"));
        assert!(d.avisos.contains(&"escapando de un atasco".to_string()));
        assert!(d.avisos.contains(&"la skill no corre: x".to_string()));
        assert_eq!(d.al_lado, "skill de la GUI");
    }

    #[test]
    fn rivals_line() {
        let mut snap = LoopSnapshot::default();
        assert_eq!(linea_rivales(&snap, 0, 3), "No vemos rivales");
        snap.robots = vec![robot(1, 2, 0.0)];
        assert_eq!(linea_rivales(&snap, 0, 3), "Vemos 1 rival: el 2");
        snap.robots = vec![robot(1, 0, 0.0), robot(1, 2, 0.0), robot(0, 1, 0.0)];
        assert_eq!(linea_rivales(&snap, 0, 3), "Vemos 2 rivales: 0 y 2");
        snap.robots.push(robot(1, 1, 0.0));
        assert_eq!(linea_rivales(&snap, 0, 3), "Vemos a los 3 rivales: 0, 1 y 2");
        snap.robots.push(robot(1, 4, 3.0));
        assert_eq!(linea_rivales(&snap, 0, 3), "Vemos a los 3 rivales: 0, 1 y 2", "el 4 no se ve hace 3 s");
    }
}
