use ratatui::{
    layout::Rect,
    style::Color,
    widgets::{Block, Borders, canvas::Canvas},
    Frame,
};
use std::collections::HashMap;
use glam::Vec2;

const FIELD_LENGTH: f64 = 9000.0; // mm
const FIELD_WIDTH: f64 = 6000.0;  // mm

#[derive(Debug, Clone)]
pub struct Robot {
    pub id: u32,
    pub team: u32, // 0 = blue, 1 = yellow
    pub position: Vec2,
    pub orientation: f32,
}

#[derive(Debug, Clone)]
pub struct Ball {
    pub position: Vec2,
}

pub struct FieldView {
    pub robots: HashMap<(u32, u32), Robot>, // (team, id) -> Robot
    pub ball: Option<Ball>,
}

impl FieldView {
    pub fn new() -> Self {
        Self {
            robots: HashMap::new(),
            ball: None,
        }
    }

    pub fn update_robot(&mut self, id: u32, team: u32, position: Vec2, orientation: f32) {
        self.robots.insert(
            (team, id),
            Robot {
                id,
                team,
                position,
                orientation,
            },
        );
    }

    pub fn update_ball(&mut self, position: Vec2) {
        self.ball = Some(Ball { position });
    }

    pub fn render(&self, f: &mut Frame, area: Rect) {
        let canvas = Canvas::default()
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .title("Field View (Press Tab to switch views)"),
            )
            .x_bounds([-FIELD_LENGTH / 2.0, FIELD_LENGTH / 2.0])
            .y_bounds([-FIELD_WIDTH / 2.0, FIELD_WIDTH / 2.0])
            .paint(|ctx: &mut ratatui::widgets::canvas::Context| {
                // Draw field outline
                ctx.draw(&ratatui::widgets::canvas::Rectangle {
                    x: -FIELD_LENGTH / 2.0,
                    y: -FIELD_WIDTH / 2.0,
                    width: FIELD_LENGTH,
                    height: FIELD_WIDTH,
                    color: Color::White,
                });

                // Draw center line
                ctx.draw(&ratatui::widgets::canvas::Line {
                    x1: 0.0,
                    y1: -FIELD_WIDTH / 2.0,
                    x2: 0.0,
                    y2: FIELD_WIDTH / 2.0,
                    color: Color::White,
                });

                // Draw center circle
                ctx.draw(&ratatui::widgets::canvas::Circle {
                    x: 0.0,
                    y: 0.0,
                    radius: 500.0,
                    color: Color::White,
                });

                // Draw ball
                if let Some(ball) = &self.ball {
                    ctx.draw(&ratatui::widgets::canvas::Circle {
                        x: ball.position.x as f64,
                        y: ball.position.y as f64,
                        radius: 150.0,
                        color: Color::Red,
                    });
                }

                // Draw robots
                for robot in self.robots.values() {
                    let color = if robot.team == 0 {
                        Color::Blue
                    } else {
                        Color::Yellow
                    };

                    // Draw robot as circle
                    ctx.draw(&ratatui::widgets::canvas::Circle {
                        x: robot.position.x as f64,
                        y: robot.position.y as f64,
                        radius: 90.0,
                        color,
                    });

                    // Draw orientation indicator
                    let dx = robot.orientation.cos() * 120.0;
                    let dy = robot.orientation.sin() * 120.0;
                    ctx.draw(&ratatui::widgets::canvas::Line {
                        x1: robot.position.x as f64,
                        y1: robot.position.y as f64,
                        x2: (robot.position.x + dx) as f64,
                        y2: (robot.position.y + dy) as f64,
                        color,
                    });
                }
            });

        f.render_widget(canvas, area);
    }
}
