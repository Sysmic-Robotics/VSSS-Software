use iced::mouse;
use iced::widget::canvas::{self, Cache, Geometry, Path, Stroke};
use iced::{Point, Rectangle, Size, Theme};
use std::collections::{HashMap, VecDeque};

use super::theme;
use super::{Ball, Message, Robot, RobotMotionDebug};

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
const ROBOT_RADIUS_MM: f32 = 38.0; // mm - radio de referencia para overlays (halo/texto)
const ROBOT_HALF_MM: f32 = 37.5; // mm - medio lado del cuerpo cuadrado (~75mm, cubo VSSS real)
const BALL_RADIUS_MM: f32 = 22.0; // mm - radio para dibujo (~44mm diámetro)
const ORIENTATION_LINE_MM: f32 = 55.0; // mm - longitud de la línea de orientación

/// mm por cada m/s — a 1.2 m/s la flecha mide 360 mm (bien visible en el campo)
const VELOCITY_SCALE_MM: f32 = 300.0;

pub struct FieldCanvas<'a> {
    pub robots: &'a HashMap<(u32, u32), Robot>,
    pub ball: &'a Option<Ball>,
    pub motion: &'a HashMap<(u32, u32), RobotMotionDebug>,
    pub cache: &'a Cache,
    /// Robot resaltado para control manual, como `(team, id)`. `None` = ninguno.
    pub selected: Option<(u32, u32)>,
    /// Target de la skill de GUI activa (metros, marco mundo). `None` = ninguno.
    pub skill_target: Option<glam::Vec2>,
    /// Traza de posiciones recientes del robot seleccionado (mm, marco cancha).
    pub trace: &'a VecDeque<glam::Vec2>,
    /// Si se dibuja la traza.
    pub show_trace: bool,
}

/// mm por cada m/s para el vector de velocidad **medida** (visión).
const MEASURED_VELOCITY_SCALE_MM: f32 = 300.0;

/// Centro y escala del dibujo del campo para unas `bounds` dadas. Compartido por
/// `draw` y por la conversión de click, para que no puedan divergir.
pub fn field_center_scale(bounds: Rectangle) -> (Point, f32) {
    let center = Point::new(bounds.width / 2.0, bounds.height / 2.0);
    let scale_x = bounds.width / (FIELD_LENGTH + FIELD_MARGIN * 2.0);
    let scale_y = bounds.height / (FIELD_WIDTH + FIELD_MARGIN * 2.0);
    (center, scale_x.min(scale_y) * 0.9)
}

/// Convierte un punto de pantalla (relativo a `bounds`) a coordenadas de mundo en
/// **metros**, inversa consistente con el `·1000` del marcador de target del dibujo.
pub fn screen_to_world_m(bounds: Rectangle, p: Point) -> glam::Vec2 {
    let (center, scale) = field_center_scale(bounds);
    glam::Vec2::new(
        (p.x - center.x) / (scale * 1000.0),
        (center.y - p.y) / (scale * 1000.0),
    )
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
            let (center, scale) = field_center_scale(bounds);

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
            frame.fill(&margin_rect, theme::FIELD_MARGIN);

            // Draw field background (white boundary)
            let field_rect = Path::rectangle(
                Point::new(
                    center.x - FIELD_LENGTH * scale / 2.0,
                    center.y - FIELD_WIDTH * scale / 2.0,
                ),
                Size::new(FIELD_LENGTH * scale, FIELD_WIDTH * scale),
            );
            frame.fill(&field_rect, theme::FIELD_GREEN);
            frame.stroke(
                &field_rect,
                Stroke::default().with_width(2.0).with_color(theme::FIELD_LINE),
            );

            // Draw center line
            let center_line = Path::line(
                Point::new(center.x, center.y - FIELD_WIDTH * scale / 2.0),
                Point::new(center.x, center.y + FIELD_WIDTH * scale / 2.0),
            );
            frame.stroke(
                &center_line,
                Stroke::default().with_width(2.0).with_color(theme::FIELD_LINE),
            );

            // Draw center circle
            let center_circle = Path::circle(center, CENTER_CIRCLE_RADIUS * scale);
            frame.stroke(
                &center_circle,
                Stroke::default().with_width(2.0).with_color(theme::FIELD_LINE),
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
                Stroke::default().with_width(2.0).with_color(theme::FIELD_LINE),
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
                Stroke::default().with_width(2.0).with_color(theme::FIELD_LINE),
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
            frame.fill(&left_goal_back, theme::GOAL_FILL);
            frame.stroke(
                &left_goal_back,
                Stroke::default().with_width(2.0).with_color(theme::FIELD_LINE),
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
                Stroke::default().with_width(2.0).with_color(theme::FIELD_LINE),
            );

            // Right goal (positive X side)
            let right_goal_back = Path::rectangle(
                Point::new(
                    center.x + FIELD_LENGTH * scale / 2.0,
                    center.y - GOAL_WIDTH * scale / 2.0,
                ),
                Size::new(GOAL_DEPTH * scale, GOAL_WIDTH * scale),
            );
            frame.fill(&right_goal_back, theme::GOAL_FILL);
            frame.stroke(
                &right_goal_back,
                Stroke::default().with_width(2.0).with_color(theme::FIELD_LINE),
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
                Stroke::default().with_width(2.0).with_color(theme::FIELD_LINE),
            );

            // Draw ball (proporción real ~43mm diámetro)
            if let Some(ball) = self.ball {
                let ball_pos = Point::new(
                    center.x + ball.position.x * scale,
                    center.y - ball.position.y * scale,
                );
                let ball_circle = Path::circle(ball_pos, BALL_RADIUS_MM * scale);
                frame.fill(&ball_circle, theme::BALL);
            }

            // Traza del robot seleccionado (celeste, debajo de los robots).
            if self.show_trace && self.trace.len() >= 2 {
                let path = Path::new(|b| {
                    for (i, p) in self.trace.iter().enumerate() {
                        let pt = Point::new(center.x + p.x * scale, center.y - p.y * scale);
                        if i == 0 {
                            b.move_to(pt);
                        } else {
                            b.line_to(pt);
                        }
                    }
                });
                frame.stroke(
                    &path,
                    Stroke::default()
                        .with_width(1.5)
                        .with_color(theme::TRACE),
                );
            }

            // Draw robots (proporción real ~75mm diámetro, como en FIRASim)
            for robot in self.robots.values() {
                let color = if robot.team == 0 {
                    theme::TEAM_BLUE
                } else {
                    theme::TEAM_YELLOW
                };

                let robot_pos = Point::new(
                    center.x + robot.position.x * scale,
                    center.y - robot.position.y * scale,
                );

                // Cuerpo cuadrado orientado (footprint real ~75mm, cubo VSSS). Vectores
                // unitarios en pantalla: "adelante" (heading) y "lado" (perpendicular).
                // La Y de pantalla está invertida (por eso -sin en la componente y).
                let (sin_t, cos_t) = robot.orientation.sin_cos();
                let fwd = Point::new(cos_t, -sin_t); // dirección de avance
                let side = Point::new(sin_t, cos_t); // perpendicular (unitario)
                let h = ROBOT_HALF_MM * scale;
                let corner = |a: f32, b: f32| {
                    Point::new(
                        robot_pos.x + a * fwd.x * h + b * side.x * h,
                        robot_pos.y + a * fwd.y * h + b * side.y * h,
                    )
                };
                let c_fl = corner(1.0, 1.0); // frente-izq
                let c_fr = corner(1.0, -1.0); // frente-der
                let c_br = corner(-1.0, -1.0); // atrás-der
                let c_bl = corner(-1.0, 1.0); // atrás-izq
                let body = Path::new(|b| {
                    b.move_to(c_fl);
                    b.line_to(c_fr);
                    b.line_to(c_br);
                    b.line_to(c_bl);
                    b.close();
                });
                frame.fill(&body, color);
                frame.stroke(
                    &body,
                    Stroke::default().with_width(1.0).with_color(theme::ROBOT_OUTLINE),
                );
                // Indicador de frente: resalta la cara delantera (c_fl→c_fr) en claro.
                frame.stroke(
                    &Path::line(c_fl, c_fr),
                    Stroke::default().with_width(3.0).with_color(theme::ROBOT_LABEL),
                );

                // Resaltado del robot seleccionado para control manual (halo).
                if self.selected == Some((robot.team, robot.id)) {
                    let halo = Path::circle(robot_pos, ROBOT_RADIUS_MM * scale + 6.0);
                    frame.stroke(
                        &halo,
                        Stroke::default()
                            .with_width(3.0)
                            .with_color(theme::SELECT_HALO),
                    );
                }

                // Línea de orientación
                let dx = robot.orientation.cos() * ORIENTATION_LINE_MM * scale;
                let dy = -robot.orientation.sin() * ORIENTATION_LINE_MM * scale;
                let orientation_line =
                    Path::line(robot_pos, Point::new(robot_pos.x + dx, robot_pos.y + dy));
                frame.stroke(
                    &orientation_line,
                    Stroke::default().with_width(2.0).with_color(theme::ROBOT_OUTLINE),
                );

                // Vector de velocidad MEDIDA (visión), naranja — distinto de la
                // flecha blanca de velocidad comandada.
                let mspeed = (robot.velocity.x * robot.velocity.x
                    + robot.velocity.y * robot.velocity.y)
                    .sqrt();
                if mspeed > 0.02 {
                    let mx = robot.velocity.x * MEASURED_VELOCITY_SCALE_MM * scale;
                    let my = -robot.velocity.y * MEASURED_VELOCITY_SCALE_MM * scale;
                    frame.stroke(
                        &Path::line(robot_pos, Point::new(robot_pos.x + mx, robot_pos.y + my)),
                        Stroke::default()
                            .with_width(2.0)
                            .with_color(theme::MEASURED_VEL),
                    );
                }

                // Número de robot, arriba del cuerpo.
                frame.fill_text(canvas::Text {
                    content: format!("{}", robot.id),
                    position: Point::new(
                        robot_pos.x - 4.0,
                        robot_pos.y - ROBOT_RADIUS_MM * scale - 14.0,
                    ),
                    color: theme::ROBOT_LABEL,
                    size: 13.0.into(),
                    ..Default::default()
                });

                // Vector de velocidad comandada (flecha blanca) + punto target (círculo cyan)
                if let Some(m) = self.motion.get(&(robot.team, robot.id)) {
                    // Flecha de velocidad
                    let speed = (m.vx * m.vx + m.vy * m.vy).sqrt();
                    if speed > 0.01 {
                        let arrow_dx = m.vx * VELOCITY_SCALE_MM * scale;
                        let arrow_dy = -m.vy * VELOCITY_SCALE_MM * scale;
                        let tip = Point::new(robot_pos.x + arrow_dx, robot_pos.y + arrow_dy);

                        let shaft = Path::line(robot_pos, tip);
                        frame.stroke(
                            &shaft,
                            Stroke::default().with_width(2.5).with_color(theme::CMD_ARROW),
                        );

                        // Cabeza de flecha (dos líneas cortas)
                        let head_len = 12.0_f32;
                        let angle = m.vy.atan2(m.vx);
                        for side in [-0.5_f32, 0.5] {
                            let hx = tip.x - head_len * (angle + side).cos();
                            let hy = tip.y + head_len * (angle + side).sin();
                            frame.stroke(
                                &Path::line(tip, Point::new(hx, hy)),
                                Stroke::default().with_width(2.5).with_color(theme::CMD_ARROW),
                            );
                        }
                    }

                    // Consigna que LLEGA al robot (v mm/s, w °/s).
                    // Texto debajo del robot para no chocar con la flecha/heading.
                    frame.fill_text(canvas::Text {
                        content: format!("v:{} w:{}", m.v_mm_s, m.w_deg_s),
                        position: Point::new(
                            robot_pos.x - ROBOT_RADIUS_MM * scale,
                            robot_pos.y + ROBOT_RADIUS_MM * scale + 2.0,
                        ),
                        color: theme::ROBOT_LABEL,
                        size: 12.0.into(),
                        ..Default::default()
                    });

                    // Punto target (círculo cyan pequeño)
                    if let Some(target) = m.target {
                        let target_pos = Point::new(
                            center.x + target.x * 1000.0 * scale,
                            center.y - target.y * 1000.0 * scale,
                        );
                        let target_circle = Path::circle(target_pos, 8.0);
                        frame.stroke(
                            &target_circle,
                            Stroke::default()
                                .with_width(2.0)
                                .with_color(theme::TARGET),
                        );
                        // Cruz en el centro del target
                        for (dx, dy) in [(-5.0_f32, 0.0), (5.0, 0.0), (0.0, -5.0_f32), (0.0, 5.0)] {
                            frame.stroke(
                                &Path::line(
                                    Point::new(target_pos.x - dx, target_pos.y - dy),
                                    Point::new(target_pos.x + dx, target_pos.y + dy),
                                ),
                                Stroke::default()
                                    .with_width(1.5)
                                    .with_color(theme::TARGET),
                            );
                        }
                    }
                }
            }

            // Marcador del target de la skill de GUI activa (verde).
            if let Some(t) = self.skill_target {
                let p = Point::new(center.x + t.x * 1000.0 * scale, center.y - t.y * 1000.0 * scale);
                let ring = Path::circle(p, 10.0);
                frame.stroke(
                    &ring,
                    Stroke::default()
                        .with_width(2.5)
                        .with_color(theme::SKILL_TARGET),
                );
                for (dx, dy) in [(-7.0_f32, 0.0), (7.0, 0.0), (0.0, -7.0_f32), (0.0, 7.0)] {
                    frame.stroke(
                        &Path::line(p, Point::new(p.x + dx, p.y + dy)),
                        Stroke::default()
                            .with_width(2.0)
                            .with_color(theme::SKILL_TARGET),
                    );
                }
            }
        });

        vec![geometry]
    }

    fn update(
        &self,
        _state: &mut Self::State,
        event: canvas::Event,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> (canvas::event::Status, Option<Message>) {
        if let canvas::Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) = event
            && let Some(p) = cursor.position_in(bounds)
        {
            let world = screen_to_world_m(bounds, p);
            return (
                canvas::event::Status::Captured,
                Some(Message::FieldClicked(world)),
            );
        }
        (canvas::event::Status::Ignored, None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bounds() -> Rectangle {
        Rectangle {
            x: 0.0,
            y: 0.0,
            width: 1000.0,
            height: 800.0,
        }
    }

    /// 2.4 — El centro de la pantalla mapea al origen del mundo (0,0).
    #[test]
    fn screen_center_is_world_origin() {
        let b = bounds();
        let (center, _scale) = field_center_scale(b);
        let w = screen_to_world_m(b, center);
        assert!(w.x.abs() < 1e-6 && w.y.abs() < 1e-6);
    }

    /// 2.4 — Un punto conocido mapea a los metros esperados (con flip de Y).
    #[test]
    fn known_point_maps_to_meters() {
        let b = bounds();
        let (center, scale) = field_center_scale(b);
        let p = Point::new(center.x + 100.0, center.y - 50.0);
        let w = screen_to_world_m(b, p);
        assert!((w.x - 100.0 / (scale * 1000.0)).abs() < 1e-6);
        assert!((w.y - 50.0 / (scale * 1000.0)).abs() < 1e-6);
    }
}
