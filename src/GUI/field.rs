use iced::mouse;
use iced::widget::canvas::{self, Cache, Geometry, Path, Stroke};
use iced::{Color, Point, Rectangle, Size, Theme};
use std::collections::HashMap;

use super::{Ball, Message, Robot};

// Dimensiones del campo VSS (Very Small Size League) — alineadas con FIRASim
const FIELD_LENGTH: f32 = 1500.0; // mm - Campo VSS 1.5m x 1.3m
const FIELD_WIDTH: f32 = 1300.0; // mm
const FIELD_MARGIN: f32 = 100.0; // mm - espacio verde fuera de los límites
// Zona de arquero (goalkeeper area): rectángulo delante del arco, como en FIRASim
const GOALKEEPER_AREA_WIDTH: f32 = 400.0; // mm - mismo ancho que el arco
const GOALKEEPER_AREA_DEPTH: f32 = 130.0; // mm - profundidad típica VSS/FIRASim
const GOAL_WIDTH: f32 = 400.0; // mm - ancho del arco
const GOAL_DEPTH: f32 = 50.0; // mm - profundidad del arco
const CENTER_CIRCLE_RADIUS: f32 = 200.0; // mm
// Tamaños visuales proporcionales a FIRASim (robots ~75mm diámetro, pelota ~43mm)
const ROBOT_RADIUS_MM: f32 = 38.0; // mm - radio para dibujo (~76mm diámetro)
const BALL_RADIUS_MM: f32 = 22.0; // mm - radio para dibujo (~44mm diámetro)
const ORIENTATION_LINE_MM: f32 = 55.0; // mm - longitud de la línea de orientación

pub struct FieldCanvas<'a> {
    pub robots: &'a HashMap<(u32, u32), Robot>,
    pub ball: &'a Option<Ball>,
    pub cache: &'a Cache,
}

impl<'a> canvas::Program<Message> for FieldCanvas<'a> {
    type State = ();

    fn draw(
        &self,
        _state: &Self::State,
        renderer: &iced::Renderer,
        _theme: &Theme,
        bounds: Rectangle,
        _cursor: mouse::Cursor,
    ) -> Vec<Geometry> {
        let geometry = self.cache.draw(renderer, bounds.size(), |frame| {
            let center = frame.center();
            let scale_x = bounds.width / (FIELD_LENGTH + FIELD_MARGIN * 2.0);
            let scale_y = bounds.height / (FIELD_WIDTH + FIELD_MARGIN * 2.0);
            let scale = scale_x.min(scale_y) * 0.9;

            // Draw margin (green area outside field boundaries)
            let margin_rect = Path::rectangle(
                Point::new(
                    center.x - (FIELD_LENGTH + FIELD_MARGIN * 2.0) * scale / 2.0,
                    center.y - (FIELD_WIDTH + FIELD_MARGIN * 2.0) * scale / 2.0,
                ),
                Size::new(
                    (FIELD_LENGTH + FIELD_MARGIN * 2.0) * scale,
                    (FIELD_WIDTH + FIELD_MARGIN * 2.0) * scale,
                ),
            );
            frame.fill(&margin_rect, Color::from_rgb(0.0, 0.5, 0.0));

            // Draw field background (white boundary)
            let field_rect = Path::rectangle(
                Point::new(
                    center.x - FIELD_LENGTH * scale / 2.0,
                    center.y - FIELD_WIDTH * scale / 2.0,
                ),
                Size::new(FIELD_LENGTH * scale, FIELD_WIDTH * scale),
            );
            frame.fill(&field_rect, Color::from_rgb(0.0, 0.5, 0.0));
            frame.stroke(
                &field_rect,
                Stroke::default().with_width(2.0).with_color(Color::WHITE),
            );

            // Draw center line
            let center_line = Path::line(
                Point::new(center.x, center.y - FIELD_WIDTH * scale / 2.0),
                Point::new(center.x, center.y + FIELD_WIDTH * scale / 2.0),
            );
            frame.stroke(
                &center_line,
                Stroke::default().with_width(2.0).with_color(Color::WHITE),
            );

            // Draw center circle
            let center_circle = Path::circle(center, CENTER_CIRCLE_RADIUS * scale);
            frame.stroke(
                &center_circle,
                Stroke::default().with_width(2.0).with_color(Color::WHITE),
            );

            // Zonas de arquero (goalkeeper areas) — rectángulos delante de cada arco, estilo FIRASim
            // Izquierda (lado X negativo)
            let left_goalkeeper_rect = Path::rectangle(
                Point::new(
                    center.x - FIELD_LENGTH * scale / 2.0,
                    center.y - GOALKEEPER_AREA_WIDTH * scale / 2.0,
                ),
                Size::new(GOALKEEPER_AREA_DEPTH * scale, GOALKEEPER_AREA_WIDTH * scale),
            );
            frame.stroke(
                &left_goalkeeper_rect,
                Stroke::default().with_width(2.0).with_color(Color::WHITE),
            );

            // Derecha (lado X positivo)
            let right_goalkeeper_rect = Path::rectangle(
                Point::new(
                    center.x + FIELD_LENGTH * scale / 2.0 - GOALKEEPER_AREA_DEPTH * scale,
                    center.y - GOALKEEPER_AREA_WIDTH * scale / 2.0,
                ),
                Size::new(GOALKEEPER_AREA_DEPTH * scale, GOALKEEPER_AREA_WIDTH * scale),
            );
            frame.stroke(
                &right_goalkeeper_rect,
                Stroke::default().with_width(2.0).with_color(Color::WHITE),
            );

            // Draw goals with depth
            // Left goal (negative X side)
            let left_goal_back = Path::rectangle(
                Point::new(
                    center.x - FIELD_LENGTH * scale / 2.0 - GOAL_DEPTH * scale,
                    center.y - GOAL_WIDTH * scale / 2.0,
                ),
                Size::new(GOAL_DEPTH * scale, GOAL_WIDTH * scale),
            );
            frame.fill(&left_goal_back, Color::from_rgb(0.3, 0.3, 0.3));
            frame.stroke(
                &left_goal_back,
                Stroke::default().with_width(2.0).with_color(Color::WHITE),
            );
            // Left goal opening line
            let left_goal_line = Path::line(
                Point::new(
                    center.x - FIELD_LENGTH * scale / 2.0,
                    center.y - GOAL_WIDTH * scale / 2.0,
                ),
                Point::new(
                    center.x - FIELD_LENGTH * scale / 2.0,
                    center.y + GOAL_WIDTH * scale / 2.0,
                ),
            );
            frame.stroke(
                &left_goal_line,
                Stroke::default().with_width(2.0).with_color(Color::WHITE),
            );

            // Right goal (positive X side)
            let right_goal_back = Path::rectangle(
                Point::new(
                    center.x + FIELD_LENGTH * scale / 2.0,
                    center.y - GOAL_WIDTH * scale / 2.0,
                ),
                Size::new(GOAL_DEPTH * scale, GOAL_WIDTH * scale),
            );
            frame.fill(&right_goal_back, Color::from_rgb(0.3, 0.3, 0.3));
            frame.stroke(
                &right_goal_back,
                Stroke::default().with_width(2.0).with_color(Color::WHITE),
            );
            // Right goal opening line
            let right_goal_line = Path::line(
                Point::new(
                    center.x + FIELD_LENGTH * scale / 2.0,
                    center.y - GOAL_WIDTH * scale / 2.0,
                ),
                Point::new(
                    center.x + FIELD_LENGTH * scale / 2.0,
                    center.y + GOAL_WIDTH * scale / 2.0,
                ),
            );
            frame.stroke(
                &right_goal_line,
                Stroke::default().with_width(2.0).with_color(Color::WHITE),
            );

            // Draw ball (proporción real ~43mm diámetro)
            if let Some(ball) = self.ball {
                let ball_pos = Point::new(
                    center.x + ball.position.x * scale,
                    center.y - ball.position.y * scale,
                );
                let ball_circle = Path::circle(ball_pos, BALL_RADIUS_MM * scale);
                frame.fill(&ball_circle, Color::from_rgb(1.0, 0.0, 0.0));
            }

            // Draw robots (proporción real ~75mm diámetro, como en FIRASim)
            for robot in self.robots.values() {
                let color = if robot.team == 0 {
                    Color::from_rgb(0.0, 0.0, 1.0)
                } else {
                    Color::from_rgb(1.0, 1.0, 0.0)
                };

                let robot_pos = Point::new(
                    center.x + robot.position.x * scale,
                    center.y - robot.position.y * scale,
                );

                let robot_circle = Path::circle(robot_pos, ROBOT_RADIUS_MM * scale);
                frame.fill(&robot_circle, color);
                frame.stroke(
                    &robot_circle,
                    Stroke::default().with_width(1.0).with_color(Color::BLACK),
                );

                // Línea de orientación (proporcionada al tamaño del robot)
                let dx = robot.orientation.cos() * ORIENTATION_LINE_MM * scale;
                let dy = -robot.orientation.sin() * ORIENTATION_LINE_MM * scale;
                let orientation_line =
                    Path::line(robot_pos, Point::new(robot_pos.x + dx, robot_pos.y + dy));
                frame.stroke(
                    &orientation_line,
                    Stroke::default().with_width(2.0).with_color(Color::BLACK),
                );
            }
        });

        vec![geometry]
    }
}
