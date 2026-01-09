use iced::{
    widget::{canvas, column, container, row, text, text_input, button, Canvas},
    Color, Element, Length, Point, Rectangle, Theme,
    Border, font,
};
use iced::widget::canvas::{Cache, Geometry, Path, Stroke};
use std::collections::VecDeque;

use super::Message;

struct PacketChart<'a> {
    history: &'a VecDeque<(f64, u64)>,
    cache: &'a Cache,
}

impl<'a> canvas::Program<Message> for PacketChart<'a> {
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
            let padding = 40.0;

            // Draw background
            let background = Path::rectangle(Point::ORIGIN, bounds.size());
            frame.fill(&background, Color::from_rgb(0.1, 0.1, 0.1));

            // Draw border
            frame.stroke(
                &background,
                Stroke::default()
                    .with_width(2.0)
                    .with_color(Color::from_rgb(0.3, 0.3, 0.3)),
            );

            if self.history.is_empty() {
                // Draw "No data" message
                frame.fill_text(iced::widget::canvas::Text {
                    content: "Waiting for packets...".to_string(),
                    position: Point::new(width / 2.0 - 80.0, height / 2.0),
                    color: Color::from_rgb(0.5, 0.5, 0.5),
                    size: 16.0.into(),
                    ..Default::default()
                });
                return;
            }

            // Calculate data ranges
            let time_min = self.history.front().map(|(t, _)| *t).unwrap_or(0.0);
            let time_max = self.history.back().map(|(t, _)| *t).unwrap_or(60.0);
            let time_range = (time_max - time_min).max(1.0);

            let packet_min = 0; // Always start from 0
            let packet_max = self.history.iter().map(|(_, p)| *p).max().unwrap_or(100);
            let packet_range = (packet_max - packet_min).max(1) as f64;

            // Calculate bar width based on time range to make bars adjacent
            let bar_width = ((width - 2.0 * padding) / time_range as f32).max(2.0);

            // Draw grid lines
            for i in 0..6 {
                let y = padding + (height - 2.0 * padding) * i as f32 / 5.0;
                let line = Path::line(
                    Point::new(padding, y),
                    Point::new(width - padding, y),
                );
                frame.stroke(
                    &line,
                    Stroke::default()
                        .with_width(1.0)
                        .with_color(Color::from_rgba(0.5, 0.5, 0.5, 0.3)),
                );
            }

            // Draw data as bars (like Wireshark)
            if !self.history.is_empty() {
                for &(time, packets) in self.history.iter() {
                    let x = padding + ((time - time_min) / time_range * (width - 2.0 * padding) as f64) as f32;
                    let bar_height = ((packets - packet_min) as f64 / packet_range) * (height - 2.0 * padding) as f64;
                    let y = height - padding - bar_height as f32;
                    
                    // Draw wide adjacent bars
                    let bar = Path::rectangle(
                        Point::new(x, y),
                        iced::Size::new(bar_width, bar_height as f32),
                    );
                    frame.fill(&bar, Color::from_rgb(0.0, 0.8, 1.0));
                }
            }

            // Draw axes labels
            let text_color = Color::from_rgb(0.8, 0.8, 0.8);
            
            // Y-axis label (packets per second)
            frame.fill_text(iced::widget::canvas::Text {
                content: format!("{}", packet_max),
                position: Point::new(5.0, padding),
                color: text_color,
                size: 12.0.into(),
                ..Default::default()
            });
            
            frame.fill_text(iced::widget::canvas::Text {
                content: format!("{}", packet_min),
                position: Point::new(5.0, height - padding),
                color: text_color,
                size: 12.0.into(),
                ..Default::default()
            });

            // X-axis label (time)
            frame.fill_text(iced::widget::canvas::Text {
                content: "Time (s)".to_string(),
                position: Point::new(width / 2.0 - 30.0, height - 10.0),
                color: text_color,
                size: 12.0.into(),
                ..Default::default()
            });
            
            frame.fill_text(iced::widget::canvas::Text {
                content: "Packets/1 seconds".to_string(),
                position: Point::new(5.0, 10.0),
                color: text_color,
                size: 12.0.into(),
                font: font::Font::MONOSPACE,
                ..Default::default()
            });
        });

        vec![geometry]
    }
}

pub fn view<'a>(
    connected: bool,
    vision_ip: &str,
    vision_port: &str,
    _packet_count: u64,
    packet_frequency: f64,
    last_ball_count: usize,
    last_robot_count: usize,
    packet_history: &'a VecDeque<(f64, u64)>,
    chart_cache: &'a Cache,
    tracker_enabled: bool,
) -> Element<'a, Message> {
    let status_color = if connected {
        Color::from_rgb(0.0, 0.8, 0.0)
    } else {
        Color::from_rgb(0.8, 0.0, 0.0)
    };

    let status_text = if connected {
        "CONNECTED"
    } else {
        "DISCONNECTED"
    };

    let ip_string = vision_ip.to_string();
    let port_string = vision_port.to_string();

    // Status information panel
    let status_info = container(
        column![
            text("RustEngine - Vision System Monitor")
                .size(16)
                .font(font::Font::MONOSPACE),
            row![
                text("Status: ").font(font::Font::MONOSPACE).size(12),
                text(status_text).style(move |_theme: &Theme| {
                    text::Style {
                        color: Some(status_color),
                    }
                }).font(font::Font::MONOSPACE).size(12),
            ]
            .spacing(5),
            row![
                text("IP: ").font(font::Font::MONOSPACE).size(12).width(Length::Fixed(50.0)),
                text_input("", &ip_string)
                    .on_input(Message::ChangeIp)
                    .font(font::Font::MONOSPACE)
                    .size(12)
                    .width(Length::Fixed(120.0)),
                text("Port: ").font(font::Font::MONOSPACE).size(12).width(Length::Fixed(50.0)),
                text_input("", &port_string)
                    .on_input(Message::ChangePort)
                    .font(font::Font::MONOSPACE)
                    .size(12)
                    .width(Length::Fixed(60.0)),
                button(text("Connect").font(font::Font::MONOSPACE).size(12))
                    .on_press(Message::Connect)
                    .padding([3, 10]),
            ]
            .spacing(5)
            .align_y(iced::Alignment::Center),
            row![
                text(format!("Frequency: {:.1} Hz", packet_frequency)).font(font::Font::MONOSPACE).size(12),
            ]
            .spacing(10),
            row![
                text(format!("Balls: {}", last_ball_count)).font(font::Font::MONOSPACE).size(12),
                text(format!("Robots: {}", last_robot_count)).font(font::Font::MONOSPACE).size(12),
            ]
            .spacing(10),
            row![
                text("Filtro Kalman: ").font(font::Font::MONOSPACE).size(12),
                button(text(if tracker_enabled { "Desactivar" } else { "Activar" })
                    .font(font::Font::MONOSPACE)
                    .size(12))
                    .on_press(Message::ToggleTracker(!tracker_enabled))
                    .padding([3, 10]),
                text(if tracker_enabled { "Habilitado" } else { "Deshabilitado" })
                    .font(font::Font::MONOSPACE)
                    .size(12)
                    .style(move |_theme: &Theme| {
                        text::Style {
                            color: Some(if tracker_enabled {
                                Color::from_rgb(0.0, 0.8, 0.0)
                            } else {
                                Color::from_rgb(0.8, 0.0, 0.0)
                            }),
                        }
                    }),
            ]
            .spacing(5)
            .align_y(iced::Alignment::Center),
        ]
        .spacing(8)
        .padding(12)
    )
    .padding(8)
    .style(|_theme: &Theme| {
        container::Style {
            border: Border {
                color: Color::from_rgb(0.3, 0.3, 0.3),
                width: 2.0,
                radius: 8.0.into(),
            },
            background: Some(Color::from_rgba(0.1, 0.1, 0.1, 0.5).into()),
            ..Default::default()
        }
    });

    let chart = Canvas::new(PacketChart {
        history: packet_history,
        cache: chart_cache,
    })
    .width(Length::Fill)
    .height(Length::Fixed(200.0));

    column![status_info, chart]
        .spacing(10)
        .padding(10)
        .into()
}
