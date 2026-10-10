//! Pestañas **Herramientas** (provisional hasta la fase 4: control manual, skills de
//! la GUI y teleport) y **Parámetros** (vista y tuning). La lógica y los mensajes son
//! los de antes del rediseño; solo cambia dónde se dibujan.

use super::field::{Fieltro, OrientacionVista};
use super::tag::MarcasRobot;
use super::{
    DebugGui, FrameMode, Message, SKILL_GROUPS, fonts, keeper_warning, skill_help, theme,
};
use iced::widget::{
    Column, Row, button, column, horizontal_rule, row, slider, text, text_input,
};
use iced::{Alignment, Element, Length};

/// Título de una sección dentro de una pestaña.
fn titulo(t: impl ToString) -> Element<'static, Message> {
    text(t.to_string())
        .font(fonts::TITULO)
        .size(theme::TXT_TITULO)
        .into()
}

/// Nota en gris.
fn nota(t: impl ToString) -> Element<'static, Message> {
    text(t.to_string())
        .size(theme::TXT_CHICO)
        .style(theme::gris)
        .into()
}

/// Botón chico de opción.
fn opcion<'a>(etiqueta: impl ToString, activa: bool, msg: Message) -> Element<'a, Message> {
    button(text(etiqueta.to_string()).size(theme::TXT_CHICO))
        .padding([3, 8])
        .style(theme::boton_opcion(activa))
        .on_press(msg)
        .into()
}

fn separador<'a>() -> Element<'a, Message> {
    horizontal_rule(1).style(theme::filete).into()
}

impl DebugGui {
    /// Pestaña Herramientas.
    pub(super) fn pestana_herramientas(&self) -> Element<'_, Message> {
        column![
            self.seccion_manual(),
            separador(),
            self.seccion_skills(),
            separador(),
            self.seccion_teleport(),
        ]
        .spacing(theme::SP_MD)
        .into()
    }

    fn seccion_manual(&self) -> Element<'_, Message> {
        let entrar = button(
            text(if self.manual_enabled {
                "Salir del modo manual"
            } else {
                "Entrar al modo manual"
            })
            .size(theme::TXT),
        )
        .padding([4, 10])
        .style(theme::boton_opcion(self.manual_enabled))
        .on_press(Message::ToggleManual(!self.manual_enabled));

        let max_id = self.run.num_robots.saturating_sub(1) as u32;
        let robot = row![
            text("robot").size(theme::TXT_CHICO).style(theme::gris),
            opcion("−", false, Message::SelectRobot(self.selected_robot.saturating_sub(1))),
            text(self.selected_robot.to_string()).size(theme::TXT),
            opcion("+", false, Message::SelectRobot((self.selected_robot + 1).min(max_id))),
        ]
        .spacing(theme::SP_SM)
        .align_y(Alignment::Center);
        let equipo = row![
            text("equipo del robot elegido").size(theme::TXT_CHICO).style(theme::gris),
            opcion(
                if self.selected_team == 0 { "azul" } else { "amarillo" },
                false,
                Message::SelectTeam(1 - self.selected_team),
            ),
        ]
        .spacing(theme::SP_SM)
        .align_y(Alignment::Center);

        let marco = row![
            text("marco").size(theme::TXT_CHICO).style(theme::gris),
            opcion("robot", self.manual_frame == FrameMode::Robot, Message::SetFrame(FrameMode::Robot)),
            opcion("mundo", self.manual_frame == FrameMode::World, Message::SetFrame(FrameMode::World)),
        ]
        .spacing(theme::SP_SM)
        .align_y(Alignment::Center);

        let ayuda = match self.manual_frame {
            FrameMode::World => {
                "W y S mueven en y, A y D en x, Q y E giran. Esc sale del modo manual."
            }
            FrameMode::Robot => "W y S avanzan y retroceden, A y D giran. Esc sale del modo manual.",
        };

        column![
            titulo("Control manual"),
            entrar,
            robot,
            equipo,
            marco,
            nota(ayuda),
            nota("Espacio detiene todo, en cualquier modo."),
        ]
        .spacing(theme::SP_SM)
        .into()
    }

    fn seccion_skills(&self) -> Element<'_, Message> {
        let mut col = Column::new().spacing(theme::SP_SM).push(titulo(format!(
            "Skill para el robot {}",
            self.selected_robot
        )));
        col = col.push(opcion(
            "ninguna (coach)",
            self.active_skill.is_none(),
            Message::SelectSkill(None),
        ));
        for (grupo, skills) in SKILL_GROUPS {
            col = col.push(nota(grupo));
            let mut fila = Row::new().spacing(theme::SP_XS);
            for &id in skills {
                fila = fila.push(opcion(
                    format!("{id:?}"),
                    self.active_skill == Some(id),
                    Message::SelectSkill(Some(id)),
                ));
            }
            col = col.push(fila.wrap());
        }
        col = col.push(text(skill_help(self.active_skill)).size(theme::TXT_CHICO));
        // Aviso del lazo (fuente de verdad: `World`) cuando la skill no corre.
        if self.active_skill.is_some()
            && let Some(aviso) = &self.snap.skill_warning
        {
            col = col.push(text(aviso.clone()).size(theme::TXT_CHICO).style(theme::alerta));
        }
        if let Some(aviso) = keeper_warning(self.active_skill, self.selected_robot, self.keeper_id) {
            col = col.push(text(aviso).size(theme::TXT_CHICO).style(theme::alerta));
        }
        let motion = if self.run.bidirectional {
            "Motion bidireccional (heading módulo 180°)."
        } else {
            "Motion de una cara (bidireccional apagado)."
        };
        let lado = if self.own_goal.x < 0.0 { "izquierdo" } else { "derecho" };
        col.push(nota(motion))
            .push(nota(format!(
                "Arco propio: {lado} (VSSL_SIDE). Arquero: robot {} (coach.keeper_id).",
                self.keeper_id
            )))
            .push(nota(
                "En Parámetros, el giro de Spin solo afecta a Spin, y el PID de heading solo a GoTo, \
                 FacePoint y ChaseBall.",
            ))
            .into()
    }

    fn seccion_teleport(&self) -> Element<'_, Message> {
        let campo = |valor: &str, msg: fn(String) -> Message| {
            text_input("", valor)
                .on_input(msg)
                .size(theme::TXT_CHICO)
                .width(Length::Fixed(58.0))
                .style(theme::entrada)
        };
        column![
            titulo("Teleport (simulación)"),
            text(format!("Robot {} (x, y en m; orientación en grados)", self.selected_robot)).size(theme::TXT_CHICO),
            row![
                campo(&self.tp_robot_x, Message::TpRobotXChanged),
                campo(&self.tp_robot_y, Message::TpRobotYChanged),
                campo(&self.tp_robot_theta, Message::TpRobotThetaChanged),
                opcion("mover", false, Message::TeleportRobot),
            ]
            .spacing(theme::SP_SM)
            .align_y(Alignment::Center),
            text("Pelota (x, y en m)").size(theme::TXT_CHICO),
            row![
                campo(&self.tp_ball_x, Message::TpBallXChanged),
                campo(&self.tp_ball_y, Message::TpBallYChanged),
                opcion("mover", false, Message::TeleportBall),
            ]
            .spacing(theme::SP_SM)
            .align_y(Alignment::Center),
            nota("Marco mundo. Solo con FIRASim o grSim."),
        ]
        .spacing(theme::SP_SM)
        .into()
    }

    /// Pestaña Parámetros: vista y tuning.
    pub(super) fn pestana_parametros(&self) -> Element<'_, Message> {
        let grupo = |etiqueta: &'static str, opciones: Vec<Element<'static, Message>>| {
            column![
                nota(etiqueta),
                Row::with_children(opciones).spacing(theme::SP_XS).wrap(),
            ]
            .spacing(theme::SP_XS)
        };
        let vista = column![
            titulo("Vista"),
            grupo(
                "fieltro",
                Fieltro::ALL
                    .into_iter()
                    .map(|f| opcion(f.etiqueta(), self.fieltro == f, Message::SetFieltro(f)))
                    .collect(),
            ),
            grupo(
                "orientación (VSSL_GUI_VISTA)",
                OrientacionVista::ALL
                    .into_iter()
                    .map(|o| opcion(o.etiqueta(), self.orientacion == o, Message::SetOrientacion(o)))
                    .collect(),
            ),
            grupo(
                "marcas de los robots (VSSL_GUI_MARCAS)",
                MarcasRobot::ALL
                    .into_iter()
                    .map(|m| opcion(m.etiqueta(), self.marcas == m, Message::SetMarcas(m)))
                    .collect(),
            ),
        ]
        .spacing(theme::SP_SM);

        // Parámetro: etiqueta, valor en vivo, entrada de texto y slider. El slider
        // siempre da un valor dentro de rango; la entrada conserva la validación.
        let param = |etiqueta: &'static str,
                     valor: f64,
                     texto: &str,
                     min: f64,
                     max: f64,
                     paso: f64,
                     msg: fn(String) -> Message| {
            column![
                row![
                    text(etiqueta).size(theme::TXT_CHICO).width(Length::Fill),
                    text(format!("{valor:.2}")).size(theme::TXT_CHICO),
                    text_input("", texto)
                        .on_input(msg)
                        .size(theme::TXT_CHICO)
                        .width(Length::Fixed(64.0))
                        .style(theme::entrada),
                ]
                .spacing(theme::SP_SM)
                .align_y(Alignment::Center),
                slider(min..=max, valor, move |v| msg(format!("{v:.3}"))).step(paso),
            ]
            .spacing(theme::SP_XS)
        };

        column![
            vista,
            separador(),
            titulo("Control manual"),
            param("velocidad (m/s)", self.manual_lin_ms, &self.manual_lin_str, 0.0, 1.5, 0.05, Message::ManualLinChanged),
            param("giro (rad/s)", self.manual_ang_rads, &self.manual_ang_str, 0.0, 12.0, 0.25, Message::ManualAngChanged),
            param("aceleración (m/s²)", self.manual_lin_accel, &self.manual_lin_accel_str, 0.0, 6.0, 0.1, Message::ManualLinAccelChanged),
            param("aceleración de giro (rad/s²)", self.manual_ang_accel, &self.manual_ang_accel_str, 0.0, 30.0, 0.5, Message::ManualAngAccelChanged),
            separador(),
            titulo("Spin"),
            param("giro de Spin (rad/s)", self.spin_omega, &self.spin_omega_str, 0.0, 40.0, 0.5, Message::SpinOmegaChanged),
            separador(),
            titulo("PID de heading"),
            nota("Solo GoTo, FacePoint y ChaseBall."),
            param("kp", self.pid_kp, &self.pid_kp_str, 0.0, 10.0, 0.1, Message::PidKpChanged),
            param("ki", self.pid_ki, &self.pid_ki_str, 0.0, 1.0, 0.01, Message::PidKiChanged),
            param("kd", self.pid_kd, &self.pid_kd_str, 0.0, 2.0, 0.05, Message::PidKdChanged),
            separador(),
            titulo("Preset"),
            row![
                text_input("tuning.json", &self.preset_name)
                    .on_input(Message::PresetNameChanged)
                    .size(theme::TXT_CHICO)
                    .width(Length::Fill)
                    .style(theme::entrada),
                opcion("guardar", false, Message::SavePreset),
                opcion("cargar", false, Message::LoadPreset),
            ]
            .spacing(theme::SP_SM)
            .align_y(Alignment::Center),
            nota(&self.preset_status),
        ]
        .spacing(theme::SP_SM)
        .into()
    }
}
