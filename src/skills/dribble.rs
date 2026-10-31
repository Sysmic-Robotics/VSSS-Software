//! Conducción "ir al arco" (`SkillId::Dribble`): el atacante sigue el campo univectorial de
//! VSSS (espirales hiperbólicas de Kim et al., KAIST; el que optimizó ITAndroids para ganar
//! la LARC 2020), que lo lleva a la pelota ya orientado hacia `target` y lo deja seguir
//! empujándola sin frenar. Aproximación y conducción son el mismo movimiento
//! (`docs/investigacion/03_conduccion.md`).
//!
//! En los registros, la aproximación en dos fases (`ApproachAligned` a un punto detrás de la
//! pelota, parar, alinearse, y recién `ShootPush`) tardaba 0.70 s y solo el 28 % terminaba
//! tocando la pelota; empujando, la llevaba 7 cm.
//!
//! - **Campo:** en el marco de la pelota con `target` en +x, una espiral antihoraria con
//!   centro (0, +de) y una horaria con centro (0, −de), tangentes a la pelota hacia +x; con
//!   |y| < de se mezclan por y. Justo delante de la pelota las dos empujan contra ella: se usa
//!   la del lado en que está el robot.
//! - **Ley del diferencial** de motion (como `RailKeeper`): avance paralelo a la orientación
//!   con `cos(error)`, rampa de `max_linear_accel`; con dos caras el error se pliega a ±90°.
//!   El giro suma al proporcional el que pide la curva del campo, ω = κ·|v| + ganancia·error:
//!   solo con el proporcional, seguir una espiral de 7 cm exigía ~1 rad de error y el robot
//!   quedaba orbitando sin tocar la pelota (banco: 0/10 desde el costado). La cara (frente o
//!   espalda) tiene histéresis: al rodear la pelota el campo gira casi 180° y, plegando el
//!   error a ±90° en cada tick, el robot alternaba de cara y se quedaba girando en el lugar.
//! - **Delante de la pelota** (respecto de `target`) no sigue la espiral, que la rodea a
//!   pocos cm y con ruido la rozaba de frente (banco: 4/10 desde adelante): va con
//!   `move_and_face` de motion, que rodea la pelota por la tangente, a un punto detrás de ella,
//!   y vuelve al campo al quedar detrás (con histéresis).
//! - **Velocidad por curvatura, anticipada:** con `max_angular_speed` el robot no sigue una
//!   espiral de pocos cm a toda velocidad. Recorre el campo por delante hasta su distancia de
//!   frenado y pide como mucho, en cada punto a distancia d con curvatura κ,
//!   √((ω_max/κ)² + 2·a·d): la velocidad desde la que frena a tiempo para esa curva. Mirando
//!   solo la curvatura donde estaba, llegaba a 0.7 m/s a la espiral, se pasaba de la pelota y
//!   daba una vuelta entera.
//!
//! (Se probó frenar cerca de la pelota hasta tenerla centrada en la cara, porque las
//! conducciones que la perdían empezaban con ella a ~3 cm del centro: llegaba más tarde y cara
//! a cara quedó +0.58 ± 0.29 para no frenar; se descartó. En partido pesa más llegar primero.
//! `dribble_speed` a velocidad real (tope 0.45 m/s): 1.1 y 0.5 neutros contra 0.8 (0.5:
//! +0.27 en una serie y −0.02 en la confirmación, +0.13 ± 0.11 en 96 partidos), 0.4 peor
//! (−0.25 ± 0.18). Queda 0.8: pedir más de lo que da el robot no cuesta nada.)

use super::{Face, MarkSkill, Skill, SkillStatus};
use crate::motion::{CONTROL_DT, Motion, MotionCommand};
use crate::params::params;
use crate::world::{RobotState, World};
use glam::Vec2;
use std::f32::consts::FRAC_PI_2;

/// Paso con que se mide la curvatura del campo delante del robot (m).
const CURVATURE_STEP: f32 = 0.02;
/// Cuánto más allá de 90° tiene que quedar el error con la cara vigente para cambiar de cara (rad).
const FACE_HYSTERESIS: f64 = 0.5;
/// Delante de la pelota más de esto (m, a lo largo de pelota → objetivo) rodea con motion;
/// vuelve al campo al quedar detrás de −esto.
const AHEAD_BAND: f32 = 0.03;
/// Punto al que rodea: detrás de la pelota a esta distancia (m).
const GO_AROUND_BACK: f32 = 0.12;

pub struct DribbleSkill {
    pub target: Vec2,
    /// Radio de las espirales (m).
    pub de: f32,
    /// Suavizado de la espiral fuera de su círculo (m).
    pub kr: f32,
    /// Velocidad pedida (m/s), antes del recorte por curvatura.
    pub speed: f64,
    v_prev: f64,
    /// Cara con la que avanza: +1 frente, −1 espalda; 0 sin elegir.
    face: f64,
    /// Rodeando la pelota con motion (estaba delante de ella).
    going_around: bool,
}

impl DribbleSkill {
    pub fn new(target: Vec2) -> Self {
        let p = &params().skills;
        Self {
            target,
            de: p.dribble_de,
            kr: p.dribble_kr,
            speed: p.dribble_speed,
            v_prev: 0.0,
            face: 0.0,
            going_around: false,
        }
    }

    pub fn set_target(&mut self, target: Vec2) {
        self.target = target;
    }

    /// Espiral hiperbólica con centro en el origen, antihoraria (`ccw`) u horaria: lejos apunta
    /// al centro y converge al círculo de radio `de`.
    fn spiral(&self, p: Vec2, ccw: bool) -> f32 {
        let theta = p.y.atan2(p.x);
        let r = p.length();
        let turn = if r > self.de { 2.0 - (self.de + self.kr) / (r + self.kr) } else { (r / self.de).sqrt() };
        theta + if ccw { 1.0 } else { -1.0 } * FRAC_PI_2 * turn
    }

    /// Dirección (rad, mundo) del campo "ir al arco" en `p` con la pelota en `ball`.
    pub fn field(&self, p: Vec2, ball: Vec2) -> f32 {
        let aim = self.target - ball;
        let rot = if aim.length_squared() > 1e-9 { aim.y.atan2(aim.x) } else { 0.0 };
        let (c, s) = (rot.cos(), rot.sin());
        let rel = p - ball;
        let q = Vec2::new(c * rel.x + s * rel.y, -s * rel.x + c * rel.y);
        let de = self.de;
        let upper = Vec2::new(q.x, q.y - de);
        let lower = Vec2::new(q.x, q.y + de);
        let phi = if q.y >= de || (q.x > 0.0 && q.y >= 0.0) {
            self.spiral(upper, true)
        } else if q.y < -de || q.x > 0.0 {
            self.spiral(lower, false)
        } else {
            let a = self.spiral(upper, true);
            let b = self.spiral(lower, false);
            let v = Vec2::new(a.cos(), a.sin()) * (q.y + de) + Vec2::new(b.cos(), b.sin()) * (de - q.y);
            v.y.atan2(v.x)
        };
        phi + rot
    }

    /// Velocidad máxima (m/s) para seguir el campo desde `p` frenando a `accel` a tiempo para
    /// cada curva de los próximos `horizon` metros, con giro máximo `max_w`.
    fn speed_limit(&self, p: Vec2, ball: Vec2, max_w: f64, accel: f64, horizon: f32) -> f64 {
        let mut q = p;
        let mut limit = f64::INFINITY;
        let mut d = 0.0_f32;
        while d <= horizon {
            let k = self.curvature(q, ball).abs() as f64;
            if k > 1e-6 {
                limit = limit.min(((max_w / k).powi(2) + 2.0 * accel * d as f64).sqrt());
            }
            let a = self.field(q, ball);
            q += Vec2::new(a.cos(), a.sin()) * CURVATURE_STEP;
            d += CURVATURE_STEP;
            if (q - ball).length() < CURVATURE_STEP {
                break;
            }
        }
        limit
    }

    /// Curvatura del campo (1/m, positiva antihoraria) a lo largo de su dirección en `p`.
    fn curvature(&self, p: Vec2, ball: Vec2) -> f32 {
        let a = self.field(p, ball);
        let b = self.field(p + Vec2::new(a.cos(), a.sin()) * CURVATURE_STEP, ball);
        Motion::normalize_angle((b - a) as f64) as f32 / CURVATURE_STEP
    }
}

impl Skill for DribbleSkill {
    fn tick(&mut self, robot: &RobotState, world: &World, motion: &Motion) -> MotionCommand {
        let ball = world.get_ball_state().position;
        let cfg = &motion.config;
        let aim = (self.target - ball).normalize_or_zero();
        let ahead = (robot.position - ball).dot(aim);
        self.going_around = if self.going_around { ahead > -AHEAD_BAND } else { ahead > AHEAD_BAND };
        if self.going_around {
            let mark = MarkSkill::new(ball);
            let behind = ball - aim * GO_AROUND_BACK;
            let cmd = motion.move_and_face(robot, behind, ball, world, mark.kp, mark.ki, mark.kd);
            // Al volver al campo, la rampa sigue desde la velocidad que traía.
            let th = robot.orientation;
            self.v_prev = cmd.vx * th.cos() + cmd.vy * th.sin();
            self.face = 0.0;
            return cmd;
        }
        let theta_d = self.field(robot.position, ball) as f64;
        let raw = Motion::normalize_angle(theta_d - robot.orientation);
        let dir = if !cfg.bidirectional {
            1.0
        } else {
            let fresh = if raw.abs() > FRAC_PI_2 as f64 { -1.0 } else { 1.0 };
            let current = if self.face == 0.0 { fresh } else { self.face };
            let err_current = if current > 0.0 { raw } else { Motion::normalize_angle(raw + std::f64::consts::PI) };
            if err_current.abs() > FRAC_PI_2 as f64 + FACE_HYSTERESIS { -current } else { current }
        };
        self.face = dir;
        let err = if dir > 0.0 { raw } else { Motion::normalize_angle(raw + std::f64::consts::PI) };
        let max_w = cfg.max_angular_speed;
        let kappa = self.curvature(robot.position, ball) as f64;
        let v_max = self.speed.min(cfg.max_linear_speed);
        let horizon = (v_max * v_max / (2.0 * cfg.max_linear_accel)) as f32 + CURVATURE_STEP;
        let v_curve = self.speed_limit(robot.position, ball, max_w, cfg.max_linear_accel, horizon);
        let speed = v_max.min(v_curve).max(cfg.min_linear_speed);
        let v_target = dir * speed * err.cos().max(0.0);
        let dv = cfg.max_linear_accel * CONTROL_DT;
        let v = v_target.clamp(self.v_prev - dv, self.v_prev + dv);
        self.v_prev = v;
        let omega = (kappa * v.abs() + cfg.heading_gain * err).clamp(-max_w, max_w);
        let th = robot.orientation;
        MotionCommand { id: robot.id, team: robot.team, vx: v * th.cos(), vy: v * th.sin(), omega, orientation: th }
    }

    fn current_target(&self, _world: &World) -> Option<Vec2> {
        Some(self.target)
    }

    fn status(&self, robot: &RobotState, world: &World) -> SkillStatus {
        let ball = world.get_ball_state().position;
        SkillStatus {
            progress: (1.0 - (ball - robot.position).length() / 0.6).clamp(0.0, 1.0),
            done: false,
            feasible: true,
            face: if self.v_prev < 0.0 { Face::Back } else { Face::Front },
        }
    }

    fn reset(&mut self) {
        self.v_prev = 0.0;
        self.face = 0.0;
        self.going_around = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const GOAL: Vec2 = Vec2::new(0.75, 0.0);

    fn skill() -> DribbleSkill {
        DribbleSkill { de: 0.07, kr: 0.05, ..DribbleSkill::new(GOAL) }
    }

    fn deg(a: f32) -> f32 {
        Motion::normalize_angle(a as f64).to_degrees() as f32
    }

    #[test]
    fn at_the_ball_and_behind_it_the_field_points_to_the_target() {
        let s = skill();
        let ball = Vec2::new(0.0, 0.0);
        assert!(deg(s.field(ball, ball)).abs() < 1e-3);
        for x in [-0.05, -0.2, -0.5] {
            assert!(deg(s.field(Vec2::new(x, 0.0), ball)).abs() < 1e-2, "x {x}");
        }
        // Con el objetivo arriba (girado 90°), detrás de la pelota apunta hacia arriba.
        let up = DribbleSkill { target: Vec2::new(0.0, 0.6), ..skill() };
        assert!((deg(up.field(Vec2::new(0.0, -0.3), ball)) - 90.0).abs() < 1e-2);
    }

    #[test]
    fn from_the_sides_it_curls_in_from_behind() {
        let s = skill();
        let ball = Vec2::new(0.0, 0.0);
        // Arriba y a la par de la pelota: baja hacia atrás de ella (y hacia −x), no de frente.
        let d = s.field(Vec2::new(0.0, 0.3), ball);
        let v = Vec2::new(d.cos(), d.sin());
        assert!(v.y < 0.0 && v.x < 0.0, "{}", deg(d));
        // Simétrico abajo.
        let d = s.field(Vec2::new(0.0, -0.3), ball);
        let v = Vec2::new(d.cos(), d.sin());
        assert!(v.y > 0.0 && v.x < 0.0, "{}", deg(d));
    }

    #[test]
    fn in_front_of_the_ball_it_goes_around_on_its_own_side() {
        let s = skill();
        let ball = Vec2::ZERO;
        // Delante y apenas arriba del eje: se aparta hacia arriba, no contra la pelota.
        let d = s.field(Vec2::new(0.15, 0.01), ball);
        assert!(d.sin() > 0.3, "{}", deg(d));
        let d = s.field(Vec2::new(0.15, -0.01), ball);
        assert!(d.sin() < -0.3, "{}", deg(d));
    }

    #[test]
    fn following_the_field_reaches_the_ball_aligned_with_the_target() {
        // Integración cinemática (sin dinámica): desde varios lados llega a < 2 cm de la
        // pelota moviéndose a menos de 15° de la dirección al objetivo.
        let s = skill();
        let ball = Vec2::new(0.1, -0.1);
        let to_goal = (GOAL - ball).normalize();
        for start in [Vec2::new(-0.4, 0.3), Vec2::new(0.4, 0.2), Vec2::new(0.5, -0.12), Vec2::new(-0.2, -0.5)] {
            let mut p = start;
            let mut reached = None;
            for _ in 0..4000 {
                let d = s.field(p, ball);
                let step = Vec2::new(d.cos(), d.sin()) * 0.002;
                if (p - ball).length() < 0.02 {
                    reached = Some(step.normalize());
                    break;
                }
                p += step;
            }
            let dir = reached.unwrap_or_else(|| panic!("desde {start} no llegó (quedó en {p})"));
            assert!(dir.dot(to_goal) > 15f32.to_radians().cos(), "desde {start}: llega en {dir}");
        }
    }

    #[test]
    fn it_slows_down_before_the_tight_curve_around_the_ball() {
        // De costado y lejos, la espiral cerrada está más adelante: ya pide menos que en una
        // recta, y menos cuanto más cerca está la curva.
        let s = skill();
        let ball = Vec2::ZERO;
        let straight = s.speed_limit(Vec2::new(-0.6, 0.0), ball, 3.0, 1.0, 0.4);
        let far = s.speed_limit(Vec2::new(-0.1, -0.35), ball, 3.0, 1.0, 0.4);
        let near = s.speed_limit(Vec2::new(-0.1, -0.12), ball, 3.0, 1.0, 0.4);
        assert!(straight > 0.8, "{straight}");
        assert!(near < far && far < 0.8, "lejos {far}, cerca {near}");
    }

    #[test]
    fn curvature_is_high_near_the_ball_spirals_and_low_far_behind() {
        let s = skill();
        let ball = Vec2::ZERO;
        assert!(s.curvature(Vec2::new(-0.6, 0.0), ball).abs() < 0.5);
        assert!(s.curvature(Vec2::new(0.02, 0.10), ball).abs() > 5.0);
        // Arriba de la pelota la espiral es antihoraria: curvatura positiva.
        assert!(s.curvature(Vec2::new(0.0, 0.12), ball) > 0.0);
    }
}
