use iced::widget::canvas::{self, Cache, Geometry, Path, Stroke};
use iced::mouse;
use iced::{Point, Rectangle, Color, Size, Theme};
use std::collections::HashMap;

use super::{Ball, Message, Robot};

const FIELD_LENGTH: f32 = 9000.0; // mm
const FIELD_WIDTH: f32 = 6000.0;  // mm

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
            let scale_x = bounds.width / FIELD_LENGTH;
            let scale_y = bounds.height / FIELD_WIDTH;
            let scale = scale_x.min(scale_y) * 0.9;

            // Draw field background
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
            let center_circle = Path::circle(center, 500.0 * scale);
            frame.stroke(
                &center_circle,
                Stroke::default().with_width(2.0).with_color(Color::WHITE),
            );

            // Draw ball
            if let Some(ball) = self.ball {
                let ball_pos = Point::new(
                    center.x + ball.position.x * scale,
                    center.y - ball.position.y * scale,
                );
                let ball_circle = Path::circle(ball_pos, 150.0 * scale);
                frame.fill(&ball_circle, Color::from_rgb(1.0, 0.0, 0.0));
            }

            // Draw robots
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

                // Draw robot circle
                let robot_circle = Path::circle(robot_pos, 90.0 * scale);
                frame.fill(&robot_circle, color);
                frame.stroke(
                    &robot_circle,
                    Stroke::default().with_width(1.0).with_color(Color::BLACK),
                );

                // Draw orientation indicator
                let dx = robot.orientation.cos() * 120.0 * scale;
                let dy = -robot.orientation.sin() * 120.0 * scale;
                let orientation_line = Path::line(
                    robot_pos,
                    Point::new(robot_pos.x + dx, robot_pos.y + dy),
                );
                frame.stroke(
                    &orientation_line,
                    Stroke::default().with_width(2.0).with_color(Color::BLACK),
                );
            }
        });

        vec![geometry]
    }
}
