//! Arquero sobre un riel (`SkillId::RailKeeper`): alternativa a `GoalKeep`
//! (`DefendGoalLineSkill`), armada con la investigación del arquero
//! (`docs/investigacion/01_arquero.md`) y con lo que mostraron nuestros registros.
//!
//! - **Riel cerca del arco.** Vive en una recta paralela a la línea de gol, a
//!   `keeper_depth` de ella (5 cm: el cuerpo llega a 1 cm de la línea). Con la línea a
//!   12 cm, la pelota que rodaba por la línea de gol le pasaba por detrás: 22 de 41 goles
//!   en contra en 48 partidos.
//! - **Orientado a lo largo del riel (±90°).** Un diferencial solo avanza hacia donde
//!   mira; mirando la pelota, cada corrección lateral exigía girar hasta 90° antes de
//!   moverse. A lo largo del riel, todo movimiento es avanzar o retroceder (dos caras).
//! - **Sin el perfil de frenado de motion.** Pide `min(ganancia·dy, √(2·a·|dy|))`: frena
//!   solo lo que el robot necesita para detenerse en el punto, no desde 0.50 m. Respeta la
//!   rampa de `max_linear_accel` (pedir más satura las ruedas y se pierde el giro) y manda
//!   el comando paralelo a la orientación, como el resto de motion.
//! - **Dónde pararse**, en orden: pelota entre el riel y la línea de gol (en la boca la
//!   barre hacia el palo; abierta, cierra el palo), tiro en curso (cruce previsto con la
//!   velocidad del EKF), rival encima de la pelota (anticipa el empuje proyectando la recta
//!   rival → pelota, como el equipo VSSS de la UFV y Warthog en SSL) y, si no, la recta
//!   pelota → centro del arco. Siempre entre los palos.
//!
//! Lejos del riel (después de un despeje, por ejemplo) vuelve con la navegación normal.

use super::{Face, Skill, SkillStatus};
use crate::motion::{Motion, MotionCommand, CONTROL_DT};
use crate::params::params;
use crate::world::{RobotState, World};
use glam::Vec2;
use std::f64::consts::FRAC_PI_2;

/// Mitad del ancho de la boca del arco (m).
const GOAL_HALF_Y: f32 = 0.20;
/// Radio de la pelota (m).
const BALL_RADIUS: f32 = 0.021;
/// Medio lado del robot (m).
const ROBOT_HALF: f32 = 0.04;
/// Velocidad mínima hacia el arco para tratar la pelota como un tiro (m/s).
const SHOT_MIN_SPEED: f32 = 0.10;
/// Ganancia de la corrección hacia el riel mientras se desplaza (por m de desvío).
const RAIL_X_GAIN: f64 = 6.0;
/// Tope de la componente de corrección hacia el riel (tangente de la inclinación).
const RAIL_X_MAX: f64 = 0.5;
/// PID de orientación para la navegación normal de vuelta al riel.
const BACK_KP: f64 = 3.0;
const BACK_KI: f64 = 0.08;
const BACK_KD: f64 = 0.20;

/// Por qué eligió la y (depuración y tests).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KeeperMode {
    /// Pelota entre el riel y la línea de gol, frente a la boca: la barre hacia el palo.
    SweepLine,
    /// Pelota honda y abierta (rodando por la línea de fondo hacia la boca): cierra el palo.
    NearPost,
    /// Tiro en curso: cruce previsto de la pelota con el riel.
    Shot,
    /// Rival encima de la pelota: proyección de la recta rival → pelota.
    Rival,
    /// Sin amenaza directa: recta pelota → centro del arco.
    Angle,
}

pub struct RailKeeperSkill {
    own_goal: Vec2,
    /// Distancia del riel a la línea de gol (centro del robot, m).
    pub depth: f32,
    /// Ganancia lateral (1/s): velocidad pedida por metro de error en y.
    pub gain: f64,
    /// Horizonte para tratar una pelota que viene como tiro (s).
    pub horizon_s: f32,
    /// Adelanto con el que se extrapola la pelota (latencia de visión y de reacción, s).
    pub lead_s: f32,
    /// Desvío del riel desde el que vuelve con la navegación normal (m).
    pub rail_tol: f32,
    /// Distancia rival–pelota desde la que el rival cuenta como encima de ella (m).
    pub contact_dist: f32,
    /// Cuánto más allá de la pelota (hacia su palo) apunta al barrerla de la boca (m).
    pub sweep: f32,
    /// El tiro manda sobre la barrida: una pelota que viene rápido hacia la línea se tapa en
    /// su cruce aunque ya esté entre el riel y la línea.
    pub shot_first: bool,
    v_prev: f64,
    last_target: Option<Vec2>,
    last_mode: KeeperMode,
}

impl RailKeeperSkill {
    pub fn new(own_goal: Vec2) -> Self {
        let p = &params().skills;
        Self {
            own_goal,
            depth: p.keeper_depth,
            gain: p.keeper_gain,
            horizon_s: p.keeper_horizon_s,
            lead_s: p.keeper_lead_s,
            rail_tol: p.keeper_rail_tol,
            contact_dist: p.keeper_contact_dist,
            sweep: p.keeper_sweep,
            shot_first: p.keeper_shot_first,
            v_prev: 0.0,
            last_target: None,
            last_mode: KeeperMode::Angle,
        }
    }

    /// El `target` de la skill es el centro del arco propio (como `GoalKeep`).
    pub fn set_own_goal(&mut self, own_goal: Vec2) {
        self.own_goal = own_goal;
    }

    pub fn mode(&self) -> KeeperMode {
        self.last_mode
    }

    /// +1 si desde el arco propio la cancha está hacia +x, −1 si hacia −x.
    fn inward(&self) -> f32 {
        if self.own_goal.x < 0.0 { 1.0 } else { -1.0 }
    }

    /// x del riel.
    pub fn rail_x(&self) -> f32 {
        self.own_goal.x + self.inward() * self.depth
    }

    /// Distancia de `p` a la línea de gol propia, hacia la cancha (negativa detrás).
    fn depth_of(&self, p: Vec2) -> f32 {
        (p.x - self.own_goal.x) * self.inward()
    }

    /// La y del riel donde pararse, y por qué.
    pub fn target_y(&self, ball: Vec2, ball_vel: Vec2, rivals: &[Vec2]) -> (f32, KeeperMode) {
        let ball_ahead = ball + ball_vel * self.lead_s;
        let d_ball = self.depth_of(ball_ahead);

        // Tiro que entra: va antes que la barrida. Un tiro pasa por la franja entre el riel y la
        // línea en sus últimos ~0.2 s; con la barrida primero el arquero se corría `sweep` hacia
        // el palo y la pelota entraba por donde estaba (50 goles en contra: desfase final
        // mediano 0.10 m, casi siempre en `SweepLine`). Tapa el cruce con el riel o, si la
        // pelota ya lo pasó, con la línea de gol.
        let toward = -ball_vel.x * self.inward();
        if self.shot_first && toward > SHOT_MIN_SPEED {
            let d_now = self.depth_of(ball);
            let line = if d_now > self.depth { self.depth } else { 0.0 };
            let t = (d_now - line) / toward;
            if (0.0..=self.horizon_s).contains(&t) {
                let y = ball.y + ball_vel.y * t;
                if y.abs() <= GOAL_HALF_Y + 0.10 {
                    return (y.clamp(-GOAL_HALF_Y, GOAL_HALF_Y), KeeperMode::Shot);
                }
            }
        }

        // Pelota entre el riel y la línea de gol: ya está "detrás" del arquero.
        if d_ball < self.depth + BALL_RADIUS {
            if ball_ahead.y.abs() <= GOAL_HALF_Y + BALL_RADIUS {
                // Frente a la boca: barrerla a lo largo de la línea hacia su palo, desde el
                // lado del centro. Puede pasar el palo lo necesario para sacarla de la boca.
                let side = if ball_ahead.y >= 0.0 { 1.0 } else { -1.0 };
                let reach = GOAL_HALF_Y + ROBOT_HALF + BALL_RADIUS;
                let y = (ball_ahead.y + side * self.sweep).clamp(-reach, reach);
                return (y, KeeperMode::SweepLine);
            }
            // Abierta, rodando por la línea de fondo hacia la boca: cerrar ese palo.
            return (ball_ahead.y.signum() * GOAL_HALF_Y, KeeperMode::NearPost);
        }

        // Tiro: la pelota viene hacia la línea de gol y cruza el riel dentro del horizonte.
        let toward = -ball_vel.x * self.inward();
        if toward > SHOT_MIN_SPEED {
            let t = (d_ball - self.depth) / toward;
            if t > 0.0 && t <= self.horizon_s {
                let y = ball_ahead.y + ball_vel.y * t;
                if y.abs() <= GOAL_HALF_Y + 0.10 {
                    return (y.clamp(-GOAL_HALF_Y, GOAL_HALF_Y), KeeperMode::Shot);
                }
            }
        }

        // Rival encima de la pelota y detrás de ella respecto de nuestro arco: el empuje
        // sale en la dirección rival → pelota; pararse donde esa recta cruza el riel.
        let rival = rivals
            .iter()
            .copied()
            .filter(|r| (*r - ball).length() <= self.contact_dist)
            .min_by(|a, b| (*a - ball).length().total_cmp(&(*b - ball).length()));
        if let Some(r) = rival {
            let dir = ball - r;
            let toward_goal = -dir.x * self.inward();
            if toward_goal > 1e-3 {
                let t = (self.rail_x() - ball.x) / dir.x;
                if t > 0.0 {
                    let y = ball.y + dir.y * t;
                    return (y.clamp(-GOAL_HALF_Y, GOAL_HALF_Y), KeeperMode::Rival);
                }
            }
        }

        // Sin amenaza directa: sobre la recta pelota → centro del arco.
        let y = ball_ahead.y * (self.depth / d_ball.max(self.depth));
        (y.clamp(-GOAL_HALF_Y, GOAL_HALF_Y), KeeperMode::Angle)
    }

    /// Velocidad lateral pedida (m/s, con signo de y) para un error `dy`: proporcional y
    /// sin pasar la que permite detenerse en el punto frenando a `accel`.
    fn lateral_speed(&self, dy: f32, accel: f64, v_max: f64) -> f64 {
        let d = dy.abs() as f64;
        if d < 0.005 {
            return 0.0;
        }
        let u = (self.gain * d).min((2.0 * accel * d).sqrt()).min(v_max);
        u * (dy as f64).signum()
    }
}

impl Skill for RailKeeperSkill {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        let ball = world.get_ball_state();
        let rivals: Vec<Vec2> = if robot.team == 0 {
            world.get_yellow_team_active()
        } else {
            world.get_blue_team_active()
        }
        .iter()
        .map(|r| r.position)
        .collect();
        let (y_t, mode) = self.target_y(ball.position, ball.velocity, &rivals);
        self.last_mode = mode;
        let rail_x = self.rail_x();
        let target = Vec2::new(rail_x, y_t);
        self.last_target = Some(target);

        // Fuera del riel: volver con la navegación normal, terminando a lo largo del riel.
        let off = rail_x - robot.position.x;
        if off.abs() > self.rail_tol {
            self.v_prev = 0.0;
            let along = target + Vec2::new(0.0, 1.0);
            return motion.move_and_face_contact(robot, target, along, world, BACK_KP, BACK_KI, BACK_KD);
        }

        let cfg = &motion.config;
        let u = self.lateral_speed(y_t - robot.position.y, cfg.max_linear_accel, cfg.max_linear_speed);
        // Dirección deseada: a lo largo del riel, inclinada hacia él si se desvió. Quieto,
        // solo mantiene la orientación a lo largo del riel.
        let theta_d = if u.abs() > 0.02 {
            let x_corr = (RAIL_X_GAIN * off as f64).clamp(-RAIL_X_MAX, RAIL_X_MAX);
            u.signum().atan2(x_corr)
        } else {
            FRAC_PI_2
        };
        // Dos caras: nunca gira más de 90°; avanza de espaldas si le queda detrás.
        let raw = Motion::normalize_angle(theta_d - robot.orientation);
        let (err, dir) = if raw.abs() > FRAC_PI_2 { (Motion::fold_bidirectional(raw), -1.0) } else { (raw, 1.0) };
        let max_w = cfg.max_angular_speed;
        let omega = (cfg.heading_gain * err).clamp(-max_w, max_w);
        let v_target = dir * u.abs() * err.cos().max(0.0);
        let dv = cfg.max_linear_accel * CONTROL_DT;
        let v = v_target.clamp(self.v_prev - dv, self.v_prev + dv);
        self.v_prev = v;

        let th = robot.orientation;
        MotionCommand { id: robot.id, team: robot.team, vx: v * th.cos(), vy: v * th.sin(), omega, orientation: th }
    }

    fn current_target(&self, _world: &World) -> Option<Vec2> {
        self.last_target
    }

    fn status(&self, robot: &RobotState, _world: &World) -> SkillStatus {
        let progress = match self.last_target {
            Some(t) => 1.0 - ((t - robot.position).length() / 0.30).min(1.0),
            None => 0.0,
        };
        SkillStatus {
            progress,
            done: false,
            feasible: true,
            face: if self.v_prev < 0.0 { Face::Back } else { Face::Front },
        }
    }

    fn reset(&mut self) {
        self.v_prev = 0.0;
        self.last_target = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Arco propio a la izquierda (−x): el riel queda en x = −0.70.
    fn keeper() -> RailKeeperSkill {
        RailKeeperSkill::new(Vec2::new(-0.75, 0.0))
    }

    #[test]
    fn a_shot_through_the_strip_before_the_line_is_blocked_not_swept() {
        // Pelota ya entre el riel (x = −0.70) y la línea (−0.75), entrando rápido en diagonal:
        // con la barrida primero se corría 10 cm hacia el palo; con el tiro primero, al cruce
        // con la línea de gol.
        let ball = Vec2::new(-0.71, 0.05);
        let vel = Vec2::new(-0.6, -0.3);
        let mut k = keeper();
        k.shot_first = false;
        let (y, mode) = k.target_y(ball, vel, &[]);
        assert_eq!(mode, KeeperMode::SweepLine);
        assert!((y - (ball.y + vel.y * k.lead_s + k.sweep)).abs() < 1e-5, "{y}");
        k.shot_first = true;
        let (y, mode) = k.target_y(ball, vel, &[]);
        assert_eq!(mode, KeeperMode::Shot);
        let t = 0.04 / 0.6;
        assert!((y - (ball.y + vel.y * t)).abs() < 1e-5, "{y}");
        // Quieta en la línea: sigue barriéndola.
        assert_eq!(k.target_y(ball, Vec2::ZERO, &[]).1, KeeperMode::SweepLine);
    }

    fn robot(x: f32, y: f32, theta_deg: f64) -> RobotState {
        let mut r = RobotState::new(2, 0);
        r.position = Vec2::new(x, y);
        r.orientation = theta_deg.to_radians();
        r
    }

    #[test]
    fn rail_sits_close_to_the_goal_on_either_side() {
        assert!((keeper().rail_x() - (-0.70)).abs() < 1e-6);
        assert!((RailKeeperSkill::new(Vec2::new(0.75, 0.0)).rail_x() - 0.70).abs() < 1e-6);
    }

    #[test]
    fn ball_rolling_along_the_goal_line_closes_the_near_post() {
        // Pelota abierta sobre la línea de fondo (x = −0.73), rodando hacia la boca.
        let (y, mode) = keeper().target_y(Vec2::new(-0.73, 0.43), Vec2::new(0.0, -0.25), &[]);
        assert_eq!(mode, KeeperMode::NearPost);
        assert_eq!(y, GOAL_HALF_Y);
    }

    #[test]
    fn ball_on_the_line_inside_the_mouth_is_swept_toward_its_post() {
        let (y, mode) = keeper().target_y(Vec2::new(-0.73, 0.08), Vec2::ZERO, &[]);
        assert_eq!(mode, KeeperMode::SweepLine);
        assert!(y > 0.08 && y <= GOAL_HALF_Y + ROBOT_HALF + BALL_RADIUS, "{y}");
        let (y, _) = keeper().target_y(Vec2::new(-0.73, -0.15), Vec2::ZERO, &[]);
        assert!(y < -0.15, "hacia su palo: {y}");
    }

    #[test]
    fn shot_goes_to_the_predicted_crossing() {
        // Pelota a 0.45 m del arco viniendo en diagonal: cruza el riel en y ≈ 0.05.
        let ball = Vec2::new(-0.30, 0.20);
        let vel = Vec2::new(-0.60, -0.20);
        let (y, mode) = keeper().target_y(ball, vel, &[]);
        assert_eq!(mode, KeeperMode::Shot);
        let t = (0.40 - 0.6 * 0.10) / 0.60; // del punto adelantado al riel
        let expected = 0.20 - 0.20 * 0.10 - 0.20 * t;
        assert!((y - expected).abs() < 0.01, "{y} vs {expected}");
    }

    #[test]
    fn rival_on_the_ball_is_anticipated_along_its_push_line() {
        // Rival detrás de la pelota empujando en diagonal hacia nuestro palo de abajo:
        // la recta rival → pelota cruza el riel por debajo del centro.
        let ball = Vec2::new(-0.40, 0.20);
        let rival = Vec2::new(-0.32, 0.26);
        let (y, mode) = keeper().target_y(ball, Vec2::ZERO, &[rival]);
        assert_eq!(mode, KeeperMode::Rival);
        // dir = (−0.08, −0.06); t = (−0.70 + 0.40) / −0.08 = 3.75 → y = 0.20 − 0.225.
        assert!((y - (-0.025)).abs() < 0.01, "{y}");
        // El mismo rival delante de la pelota (empujaría hacia su arco): sin anticipación.
        let (_, mode) = keeper().target_y(ball, Vec2::ZERO, &[Vec2::new(-0.48, 0.14)]);
        assert_eq!(mode, KeeperMode::Angle);
    }

    #[test]
    fn far_ball_keeps_the_keeper_on_the_ball_goal_line_and_inside_the_posts() {
        let (y, mode) = keeper().target_y(Vec2::new(0.30, 0.50), Vec2::ZERO, &[]);
        assert_eq!(mode, KeeperMode::Angle);
        assert!((y - 0.50 * 0.05 / 1.05).abs() < 1e-3, "{y}");
        // Pelota a 10 cm de la línea y muy abierta: recortada al palo.
        let (y, _) = keeper().target_y(Vec2::new(-0.65, -0.45), Vec2::ZERO, &[]);
        assert_eq!(y, -GOAL_HALF_Y);
    }

    #[test]
    fn on_the_rail_it_moves_sideways_without_turning() {
        // En el riel, orientado a +y, con la pelota arriba: avanza a lo largo de la línea
        // (comando paralelo a +y) y casi no gira.
        let mut k = keeper();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(-0.73, 0.43), Vec2::ZERO);
        let motion = Motion::new();
        let r = robot(-0.70, 0.0, 90.0);
        let mut c = k.tick(&r, &world, &motion);
        for _ in 0..20 {
            c = k.tick(&r, &world, &motion);
        }
        assert!(c.vy > 0.2 && c.vx.abs() < 1e-6, "{c:?}");
        assert!(c.omega.abs() < 0.2, "{c:?}");
        // Mirando a −y hace lo mismo de espaldas, sin girar 180°.
        let mut k = keeper();
        let r = robot(-0.70, 0.0, -90.0);
        for _ in 0..20 {
            c = k.tick(&r, &world, &motion);
        }
        assert!(c.vy > 0.2 && c.omega.abs() < 0.2, "{c:?}");
    }

    #[test]
    fn it_does_not_brake_far_from_the_point_like_the_motion_profile() {
        // A 0.15 m del punto, motion frena (perfil desde 0.50 m) y el riel no: pide lo
        // que permite detenerse frenando a max_linear_accel.
        let k = keeper();
        let cfg = crate::motion::MotionConfig::default();
        let u = k.lateral_speed(0.15, cfg.max_linear_accel, cfg.max_linear_speed);
        let expected = (k.gain * 0.15).min((2.0 * cfg.max_linear_accel * 0.15).sqrt());
        assert!((u - expected).abs() < 1e-6 && u > 0.5, "{u}");
        assert_eq!(k.lateral_speed(0.003, cfg.max_linear_accel, cfg.max_linear_speed), 0.0);
    }

    #[test]
    fn off_the_rail_it_navigates_back() {
        // Lejos del riel (después de un despeje) vuelve con la navegación normal.
        let mut k = keeper();
        let mut world = World::new(3, 3);
        world.update_ball(Vec2::new(0.0, 0.0), Vec2::ZERO);
        let motion = Motion::new();
        let r = robot(-0.45, 0.10, 180.0);
        let mut c = k.tick(&r, &world, &motion);
        for _ in 0..20 {
            c = k.tick(&r, &world, &motion);
        }
        // Mirando a −x (hacia el arco), avanza hacia el riel.
        assert!(c.vx < -0.05, "{c:?}");
    }
}
