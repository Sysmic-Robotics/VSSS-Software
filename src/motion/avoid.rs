//! Rodeo de la pelota y de las áreas prohibidas por la tangente.
//!
//! Una sola regla para los dos obstáculos estáticos: si el segmento robot → destino toca
//! un obstáculo inflado, el robot apunta a la tangente del obstáculo del lado más cercano
//! al destino. Un lado no vale si su punto de tangencia sale de la cancha o si el camino
//! hasta él toca otro obstáculo (la pelota pegada al arco se rodea por fuera, la pelota
//! contra la pared por el lado de la cancha), y el lado elegido se mantiene mientras siga
//! valiendo (histéresis). Un obstáculo que contiene el destino no se rodea. Los robots
//! siguen en el UVF.

use glam::Vec2;

use crate::skills::zones::{AreaRect, AREA_CLEARANCE, AREA_HALF_Y, AREA_X, ARC_CENTER_X, ARC_RADIUS, ROBOT_HALF};

/// Holgura (m) del centro del robot al centro de la pelota: media diagonal del robot
/// (0.053) + radio de la pelota (0.021) + margen. Revisar con la latencia real (sysid).
pub const BALL_CLEARANCE: f32 = 0.09;
/// Límite (m) del centro del robot en la cancha física (0.75 × 0.65 menos medio robot).
const FIELD_X: f32 = 0.71;
const FIELD_Y: f32 = 0.61;
/// Distancia mínima (m) del punto de mira: si la tangencia está más cerca, se apunta más
/// lejos en la misma dirección.
const MIN_AIM: f32 = 0.15;
/// Paso (m) con que se muestrea un segmento.
const STEP: f32 = 0.01;
/// Con el robot dentro de la forma inflada, la holgura se reduce a su distancia menos esto
/// (m) para que la tangente exista.
const INSIDE_EPS: f32 = 0.005;
/// Dentro de la forma inflada, la tangente se inclina hacia afuera en proporción a la
/// penetración (`OUTWARD_GAIN · (holgura − distancia) / holgura`, sumado como vector a la
/// dirección de la tangente). Sin esto, con la latencia el robot llega mirando hacia el
/// obstáculo y la holgura reducida lo deja entrar unos milímetros por tick.
const OUTWARD_GAIN: f32 = 3.0;
/// |x| (m) de los dos puntos que cierran la silueta de un área detrás de la línea de
/// fondo: su tangente sale de la cancha, así que nunca se rodea un área por detrás.
const BEHIND_X: f32 = 1.5;

/// Obstáculo estático de la regla de la tangente.
#[derive(Debug, Clone, Copy)]
pub enum Obstacle {
    /// Centro de la pelota.
    Ball(Vec2),
    /// Área prohibida para el robot (rectángulo y arco).
    Area(AreaRect),
}

/// Obstáculo y lado elegidos en el tick anterior (histéresis). Lado +1: la tangente que
/// gira antihorario desde la dirección al destino; −1: la horaria.
pub type Memo = Option<(u8, i8)>;

impl Obstacle {
    fn key(&self) -> u8 {
        match self {
            Self::Ball(_) => 0,
            Self::Area(a) if a.side > 0.0 => 2,
            Self::Area(_) => 1,
        }
    }

    /// Distancia (m) de `p` a la forma del obstáculo (sin holgura).
    fn clearance(&self, p: Vec2) -> f32 {
        match self {
            Self::Ball(c) => (p - *c).length(),
            Self::Area(a) => a.clearance(p),
        }
    }

    fn margin(&self) -> f32 {
        match self {
            Self::Ball(_) => BALL_CLEARANCE,
            Self::Area(_) => AREA_CLEARANCE,
        }
    }

    /// Distancia a la que un destino "está sobre" el obstáculo y no se rodea: la holgura de
    /// la pelota (la skill la quiere tocar) o el contacto del guardia con el área (destino
    /// ilegal: lo frena el guardia).
    fn target_margin(&self) -> f32 {
        match self {
            Self::Ball(_) => BALL_CLEARANCE,
            Self::Area(_) => ROBOT_HALF,
        }
    }

    /// Holgura con el destino `t`: si el destino (legal) está más cerca que la holgura, la
    /// suya menos `INSIDE_EPS`, para poder llegar.
    fn margin_to(&self, t: Vec2) -> f32 {
        self.margin().min(self.clearance(t) - INSIDE_EPS).max(0.0)
    }

    /// Holgura vista desde `p` con el destino `t`: además, si el robot ya está más cerca, su
    /// distancia menos `INSIDE_EPS` (para que la tangente exista).
    fn margin_from(&self, p: Vec2, t: Vec2) -> f32 {
        self.margin_to(t).min(self.clearance(p) - INSIDE_EPS).max(0.0)
    }

    /// Discos (centro, radio) cuyo casco convexo es la forma inflada con `m`.
    fn disks(&self, m: f32) -> Vec<(Vec2, f32)> {
        match *self {
            Self::Ball(c) => vec![(c, m)],
            Self::Area(a) => {
                let s = a.side;
                vec![
                    (Vec2::new(s * AREA_X, AREA_HALF_Y), m),
                    (Vec2::new(s * AREA_X, -AREA_HALF_Y), m),
                    (Vec2::new(s * ARC_CENTER_X, 0.0), ARC_RADIUS + m),
                    (Vec2::new(s * BEHIND_X, AREA_HALF_Y), m),
                    (Vec2::new(s * BEHIND_X, -AREA_HALF_Y), m),
                ]
            }
        }
    }

    /// Dirección (unitaria) en que crece la distancia al obstáculo en `p`.
    fn outward(&self, p: Vec2) -> Vec2 {
        let h = 0.002;
        let g = Vec2::new(
            self.clearance(p + Vec2::X * h) - self.clearance(p - Vec2::X * h),
            self.clearance(p + Vec2::Y * h) - self.clearance(p - Vec2::Y * h),
        );
        g.normalize_or_zero()
    }

    /// Distancia a lo largo de `a → b` al primer punto a `m` o menos del obstáculo.
    fn first_hit(&self, a: Vec2, b: Vec2, m: f32) -> Option<f32> {
        let len = (b - a).length();
        let n = ((len / STEP).ceil() as usize).max(1);
        (0..=n)
            .map(|i| i as f32 / n as f32)
            .find(|&t| self.clearance(a.lerp(b, t)) <= m)
            .map(|t| t * len)
    }
}

fn wrap(a: f32) -> f32 {
    use std::f32::consts::{PI, TAU};
    let a = a % TAU;
    if a > PI {
        a - TAU
    } else if a < -PI {
        a + TAU
    } else {
        a
    }
}

/// Punto de mira para ir de `p` a `target` rodeando `obstacles` por la tangente. `memo`
/// guarda el lado elegido entre ticks (por robot).
pub fn aim_point(p: Vec2, target: Vec2, obstacles: &[Obstacle], memo: &mut Memo) -> Vec2 {
    // Se descarta el que contiene el destino, y aquel sobre cuya forma ya está el robot: no
    // hay tangente (encima de la pelota es una falla de visión; dentro del área, la salida
    // es del guardia).
    let live: Vec<&Obstacle> = obstacles
        .iter()
        .filter(|o| o.clearance(target) > o.target_margin() && o.clearance(p) > INSIDE_EPS)
        .collect();
    // Dentro de la holgura de un obstáculo: inclinación hacia afuera, proporcional a la
    // penetración. Vale también con el destino a la vista: si no, el robot que se metió en
    // la holgura al doblar (latencia) sigue pegado al borde, dentro del contacto del guardia.
    let push_out: Vec2 = live
        .iter()
        .map(|o| {
            let m = o.margin_to(target);
            let depth = if m > 0.0 { (m - o.clearance(p)).max(0.0) / m } else { 0.0 };
            o.outward(p) * (OUTWARD_GAIN * depth)
        })
        .sum();
    let blocker = live
        .iter()
        .filter_map(|o| o.first_hit(p, target, o.margin_from(p, target)).map(|h| (h, *o)))
        .min_by(|a, b| a.0.total_cmp(&b.0));
    let Some((_, o)) = blocker else {
        *memo = None;
        let to_target = target - p;
        if push_out == Vec2::ZERO || to_target.length() < 1e-6 {
            return target;
        }
        let dir = (to_target.normalize() + push_out).normalize_or_zero();
        return if dir == Vec2::ZERO { target } else { p + dir * to_target.length() };
    };
    let m = o.margin_from(p, target);
    let tdir = (target - p).y.atan2((target - p).x);
    // Bordes de la silueta: (ángulo relativo a la dirección al destino, largo de la tangente).
    let (mut lo, mut hi) = ((f32::INFINITY, 0.0f32), (f32::NEG_INFINITY, 0.0f32));
    for (c, r) in o.disks(m) {
        let d = (c - p).length();
        if d < 1e-6 {
            continue;
        }
        let rel = wrap((c - p).y.atan2((c - p).x) - tdir);
        let w = (r / d).min(1.0).asin();
        let len = (d * d - r * r).max(0.0).sqrt();
        if rel - w < lo.0 {
            lo = (rel - w, len);
        }
        if rel + w > hi.0 {
            hi = (rel + w, len);
        }
    }
    if !(lo.0.is_finite() && hi.0.is_finite()) {
        *memo = None;
        return target;
    }
    // Por lado: (desvío, válido, punto de mira).
    let side = |(rel, len): (f32, f32)| {
        let tangent = Vec2::from_angle(tdir + rel);
        let touch = p + tangent * len;
        let dir = Some((tangent + push_out).normalize_or_zero()).filter(|d| *d != Vec2::ZERO).unwrap_or(tangent);
        let valid = touch.x.abs() <= FIELD_X
            && touch.y.abs() <= FIELD_Y
            && live
                .iter()
                .filter(|x| x.key() != o.key())
                .all(|x| x.first_hit(p, touch, x.margin_from(p, target)).is_none());
        (rel.abs(), valid, p + dir * len.max(MIN_AIM))
    };
    let sides = [(1i8, side(hi)), (-1i8, side(lo))];
    let kept = memo
        .filter(|&(k, _)| k == o.key())
        .and_then(|(_, s)| sides.iter().find(|c| c.0 == s && c.1 .1));
    let any_valid = sides.iter().any(|c| c.1 .1);
    let chosen = kept.copied().unwrap_or_else(|| {
        *sides
            .iter()
            .filter(|c| c.1 .1 || !any_valid)
            .min_by(|a, b| a.1 .0.total_cmp(&b.1 .0))
            .expect("hay dos lados")
    });
    *memo = Some((o.key(), chosen.0));
    chosen.1 .2
}

#[cfg(test)]
mod tests {
    use super::*;

    const OWN: AreaRect = AreaRect { side: -1.0 };

    /// Distancia de `c` a la semirrecta que sale de `p` hacia `aim`.
    fn ray_dist(p: Vec2, aim: Vec2, c: Vec2) -> f32 {
        let d = (aim - p).normalize();
        let t = (c - p).dot(d).max(0.0);
        (c - (p + d * t)).length()
    }

    #[test]
    fn clear_path_aims_at_the_target() {
        let (p, t) = (Vec2::new(-0.4, 0.3), Vec2::new(0.4, 0.3));
        let obs = [Obstacle::Ball(Vec2::ZERO), Obstacle::Area(OWN)];
        assert_eq!(aim_point(p, t, &obs, &mut None), t);
    }

    #[test]
    fn ball_in_the_way_aims_at_the_tangent_on_the_target_side() {
        // La pelota queda 1 cm por debajo de la recta: se rodea por arriba, a la holgura.
        let (p, t, ball) = (Vec2::new(-0.45, 0.02), Vec2::new(0.45, 0.0), Vec2::ZERO);
        let aim = aim_point(p, t, &[Obstacle::Ball(ball)], &mut None);
        assert!(aim.y > p.y, "rodea por arriba: {aim:?}");
        assert!((ray_dist(p, aim, ball) - BALL_CLEARANCE).abs() < 1e-3, "tangente a la holgura");
    }

    #[test]
    fn an_obstacle_that_contains_the_target_is_not_avoided() {
        let p = Vec2::new(-0.3, 0.0);
        let near_ball = Vec2::new(0.05, 0.0);
        assert_eq!(aim_point(p, near_ball, &[Obstacle::Ball(Vec2::ZERO)], &mut None), near_ball);
        // Destino ilegal dentro del área: no se rodea (lo frena el guardia, como hoy).
        let (q, in_area) = (Vec2::new(-0.3, 0.35), Vec2::new(-0.72, -0.2));
        assert_eq!(aim_point(q, in_area, &[Obstacle::Area(OWN)], &mut None), in_area);
    }

    #[test]
    fn a_tangent_outside_the_field_is_not_valid() {
        // Pelota contra la pared lateral: el lado de la pared sale de la cancha.
        let ball = Vec2::new(0.0, -0.56);
        let (p, t) = (Vec2::new(-0.4, -0.56), Vec2::new(0.4, -0.56));
        let aim = aim_point(p, t, &[Obstacle::Ball(ball)], &mut None);
        assert!(aim.y > ball.y, "rodea por el lado de la cancha: {aim:?}");
    }

    #[test]
    fn a_ball_next_to_the_arc_is_rounded_on_the_far_side() {
        // Pelota a menos de un robot del arco: el lado que pasa entre la pelota y el área
        // toca el área, así que no vale.
        let ball = Vec2::new(-0.45, 0.0);
        let (p, t) = (Vec2::new(-0.45, 0.30), Vec2::new(-0.45, -0.30));
        let obs = [Obstacle::Area(OWN), Obstacle::Ball(ball)];
        let aim = aim_point(p, t, &obs, &mut None);
        assert!(aim.x > ball.x, "rodea por el lado de la cancha: {aim:?}");
    }

    #[test]
    fn an_area_is_never_rounded_through_the_back() {
        // Al costado del área, con el destino del otro lado: siempre por el frente, también
        // pegado a la línea de fondo (donde el lado de la pared desvía menos).
        for (x, y) in [(-0.66, -0.45), (-0.70, -0.44), (-0.72, -0.50), (-0.66, 0.45), (-0.70, 0.44)] {
            let p = Vec2::new(x, y);
            let t = Vec2::new(x, -y);
            let aim = aim_point(p, t, &[Obstacle::Area(OWN)], &mut None);
            assert!(aim.x > p.x, "desde {p:?} apunta hacia la cancha: {aim:?}");
        }
    }

    #[test]
    fn the_side_does_not_flip_with_camera_noise() {
        // Destino justo detrás de la pelota: sin memoria el lado cambia con el ruido del
        // proxy; con memoria se queda en el primero.
        let sigma = crate::params::VisionParams::default().proxy_sigma_pos_m as f32;
        let mut rng = 0x9E37_79B9_7F4A_7C15u64;
        let mut gauss = move || {
            let mut next = || {
                rng ^= rng >> 12;
                rng ^= rng << 25;
                rng ^= rng >> 27;
                ((rng.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64).max(1e-12)
            };
            let (u1, u2) = (next(), next());
            ((-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()) as f32
        };
        let (t, obs) = (Vec2::new(0.45, 0.0), [Obstacle::Ball(Vec2::ZERO)]);
        let side = |aim: Vec2, p: Vec2| (aim.y - p.y).signum();
        let (mut memo, mut first, mut flips_memo, mut flips_raw, mut last_raw) = (None, None, 0, 0, None);
        for _ in 0..300 {
            let p = Vec2::new(-0.45 + sigma * gauss(), sigma * gauss());
            let s = side(aim_point(p, t, &obs, &mut memo), p);
            if *first.get_or_insert(s) != s {
                flips_memo += 1;
            }
            let r = side(aim_point(p, t, &obs, &mut None), p);
            if last_raw.is_some_and(|l| l != r) {
                flips_raw += 1;
            }
            last_raw = Some(r);
        }
        assert!(flips_raw > 10, "el escenario debe ser simétrico: {flips_raw}");
        assert_eq!(flips_memo, 0, "con histéresis el lado no cambia");
    }

    #[test]
    fn inside_the_inflated_ball_the_robot_moves_outward() {
        // A 0.06 m de la pelota (holgura 0.09) con el destino detrás: rodea y se aleja (el
        // punto de mira queda más lejos de la pelota que el robot, y la semirrecta no se
        // acerca a ella).
        let ball = Vec2::ZERO;
        let p = Vec2::new(-0.06, 0.0);
        let aim = aim_point(p, Vec2::new(0.4, 0.0), &[Obstacle::Ball(ball)], &mut None);
        assert!((aim - ball).length() > 0.09, "{aim:?}");
        assert!(ray_dist(p, aim, ball) >= 0.06 - 1e-4, "{aim:?}");
    }

    #[test]
    fn inside_the_area_margin_with_the_target_in_sight_moves_outward() {
        // Caso de FIRASim (`blockline_across_area`): al doblar el arco el robot quedó a
        // 0.038 m del área (dentro del contacto del guardia, 0.04) y el destino, junto a la
        // esquina, ya está a la vista. Sin inclinación iba derecho, pegado al borde; con
        // ella, el punto de mira queda más lejos del área que el robot.
        let p = Vec2::new(-0.513, 0.02);
        let t = Vec2::new(-0.528, 0.296);
        assert!(OWN.clearance(p) < 0.04, "escenario: dentro del contacto del guardia");
        let aim = aim_point(p, t, &[Obstacle::Area(OWN)], &mut None);
        let toward = (aim - p).normalize();
        let step = p + toward * 0.02;
        assert!(OWN.clearance(step) > OWN.clearance(p) + 0.005, "se aleja del área: {aim:?}");
        // Con el robot fuera de la holgura, el destino a la vista es el punto de mira.
        let q = Vec2::new(-0.45, 0.0);
        assert_eq!(aim_point(q, t, &[Obstacle::Area(OWN)], &mut None), t);
    }

    #[test]
    fn a_robot_on_top_of_an_obstacle_aims_at_the_target() {
        // Robot sobre la pelota (en los tests, la pelota por defecto en el origen) o dentro
        // del área: no hay tangente; el punto de mira es el destino y es finito.
        let t = Vec2::new(0.4, 0.3);
        assert_eq!(aim_point(Vec2::ZERO, t, &[Obstacle::Ball(Vec2::ZERO)], &mut None), t);
        let inside = Vec2::new(-0.68, 0.0);
        assert_eq!(aim_point(inside, t, &[Obstacle::Area(OWN)], &mut None), t);
    }

    #[test]
    fn the_area_shape_matches_the_guard() {
        // `clearance ≤ m` implica `touches_with(m)` (el margen del guardia es cuadrado), y
        // todo punto que el guardia da por tocando está dentro de la holgura de navegación.
        for i in -80..=0 {
            for j in -70..=70 {
                let p = Vec2::new(i as f32 / 100.0, j as f32 / 100.0);
                let c = OWN.clearance(p);
                for m in [0.0, 0.04, AREA_CLEARANCE] {
                    if c <= m {
                        assert!(OWN.touches_with(p, m), "{p:?} m={m}");
                    }
                }
                if OWN.touches(p) {
                    assert!(c <= AREA_CLEARANCE, "{p:?}");
                }
            }
        }
    }
}
