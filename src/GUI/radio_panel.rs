use iced::{
    Border, Color, Element, Length, Theme, font,
    widget::{button, column, container, row, text, text_input},
};

use super::Message;

/// Panel de radio/transporte: muestra el transporte activo, puerto/baud, equipo
/// propio, estado de conexión y PPS de visión. La edición de puerto/baud queda
/// registrada en la GUI; aplicarla en vivo requiere reinicio (ver design.md D4),
/// por eso se indica explícitamente. En headless la config viene del entorno.
pub fn view<'a>(
    target_label: &str,
    port: &str,
    baud: &str,
    own_team: u32,
    transport_connected: Option<bool>,
    packet_frequency: f64,
) -> Element<'a, Message> {
    let (conn_text, conn_color) = match transport_connected {
        Some(true) => ("CONECTADO", Color::from_rgb(0.0, 0.8, 0.0)),
        Some(false) => ("ERROR / DESCONECTADO", Color::from_rgb(0.8, 0.0, 0.0)),
        None => ("SIN DATOS", Color::from_rgb(0.6, 0.6, 0.6)),
    };

    let is_base_station = target_label.eq_ignore_ascii_case("basestation")
        || target_label.eq_ignore_ascii_case("base-station")
        || target_label.eq_ignore_ascii_case("BaseStation");

    let port_string = port.to_string();
    let baud_string = baud.to_string();

    let mut body = column![
        text("Radio / Transporte")
            .size(16)
            .font(font::Font::MONOSPACE),
        row![
            text("Transporte: ").font(font::Font::MONOSPACE).size(12),
            text(target_label.to_string())
                .font(font::Font::MONOSPACE)
                .size(12),
        ]
        .spacing(5),
        row![
            text("Estado: ").font(font::Font::MONOSPACE).size(12),
            text(conn_text)
                .font(font::Font::MONOSPACE)
                .size(12)
                .style(move |_t: &Theme| text::Style {
                    color: Some(conn_color)
                }),
        ]
        .spacing(5),
        row![
            text(format!("Visión PPS: {:.1} Hz", packet_frequency))
                .font(font::Font::MONOSPACE)
                .size(12),
        ],
        row![
            text("Equipo propio: ").font(font::Font::MONOSPACE).size(12),
            button(
                text(if own_team == 0 { "Azul" } else { "Amarillo" })
                    .font(font::Font::MONOSPACE)
                    .size(12)
            )
            .padding([3, 10])
            .on_press(Message::SelectTeam(1 - own_team)),
        ]
        .spacing(5)
        .align_y(iced::Alignment::Center),
    ]
    .spacing(8)
    .padding(12);

    if is_base_station {
        body = body.push(
            row![
                text("Puerto: ")
                    .font(font::Font::MONOSPACE)
                    .size(12)
                    .width(Length::Fixed(70.0)),
                text_input("/dev/ttyUSB0", &port_string)
                    .on_input(Message::RadioPortChanged)
                    .font(font::Font::MONOSPACE)
                    .size(12)
                    .width(Length::Fixed(160.0)),
                text("Baud: ")
                    .font(font::Font::MONOSPACE)
                    .size(12)
                    .width(Length::Fixed(50.0)),
                text_input("115200", &baud_string)
                    .on_input(Message::RadioBaudChanged)
                    .font(font::Font::MONOSPACE)
                    .size(12)
                    .width(Length::Fixed(90.0)),
            ]
            .spacing(5)
            .align_y(iced::Alignment::Center),
        );
        body = body.push(
            text("Nota: aplicar puerto/baud requiere reiniciar el proceso (env VSSL_BASESTATION_DEVICE/BAUD).")
                .font(font::Font::MONOSPACE)
                .size(11)
                .style(|_t: &Theme| text::Style {
                    color: Some(Color::from_rgb(0.7, 0.7, 0.3)),
                }),
        );
    }

    container(body)
        .padding(8)
        .style(|_theme: &Theme| container::Style {
            border: Border {
                color: Color::from_rgb(0.3, 0.3, 0.3),
                width: 2.0,
                radius: 8.0.into(),
            },
            background: Some(Color::from_rgba(0.1, 0.1, 0.1, 0.5).into()),
            ..Default::default()
        })
        .into()
}
