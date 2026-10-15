//! La pelota en una rutina con pelota (D2–D6 de `rutinas-pelota-resiliente`): funciones puras
//! para el rescate (quién va y dónde se apartan los demás), el despegue de la pelota pegada a
//! la pared y la pelota fuera de juego. El decisor guarda el estado y arma las elecciones.

use super::validacion::{fuera_de_zona, nombre_area};
use crate::coach::{FIELD_HALF_X, FIELD_HALF_Y};
use crate::skills::ApproachAlignedSkill;
use crate::skills::zones::{AREA_CLEARANCE, AreaRect};
use glam::Vec2;

/// Empate en la elección del rescatador (m): dentro de esto va el dueño del próximo paso.
const EMPATE_M: f32 = 0.05;
/// La menor distancia a la que ApproachAligned acerca su lugar detrás de la pelota (m), con
/// un margen de redondeo.
const LUGAR_MIN_M: f32 = 0.08 - 1e-3;
/// El despegue empuja hacia un punto a esta distancia de la pelota (m).
pub(super) const DESPEGUE_ALCANCE_M: f32 = 0.40;

/// Las dos áreas: ningún robot de una rutina con pelota entra.
pub(super) fn areas() -> [AreaRect; 2] {
    [AreaRect::own(1.0), AreaRect::opp(1.0)]
}

/// Por qué no hay lugar detrás de la pelota para empujarla con `alinear`, o `None`. El lugar
/// es el de ApproachAligned (`staging_point`, la misma función de la skill): con la pelota
/// contra la pared, la skill lo acerca a la pelota y, si ni así entra al campo lógico, lo
/// recorta contra el borde, fuera de la línea del empuje (o, en una esquina, casi sobre la
/// línea pero más cerca que `LUGAR_MIN_M`, donde el robot no cabe). Junto a un área, alcanza
/// con que el robot quepa sin tocarla en el punto más cercano a la pelota que la skill acepta
/// (`pos_tol` hacia ella).
pub(super) fn sin_lugar_detras(alinear: &ApproachAlignedSkill, pelota: Vec2) -> Option<String> {
    let dir = (alinear.aim_point - pelota).normalize_or_zero();
    let lugar = alinear.staging_point(pelota)?;
    let atras = pelota - lugar;
    if atras.length() < LUGAR_MIN_M || (atras.dot(dir) - atras.length()).abs() > 1e-3 {
        return Some("está contra la pared".to_string());
    }
    let mas_cerca = lugar + dir * alinear.pos_tol;
    areas().iter().find(|a| a.touches(mas_cerca)).map(|a| format!("el lugar toca {}", nombre_area(a)))
}

/// Por qué la pelota está fuera de juego (D6).
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum FueraDeJuego {
    /// Su centro pasó la línea de fondo.
    Arco { izquierdo: bool },
    /// Su centro está dentro de un área (rectángulo o arco, sin holgura).
    Area(AreaRect),
    /// Pegada a la pared sin despegue posible.
    Pegada,
    /// Con el robot real, la visión no la ve (posición no finita).
    NoSeVe,
}

impl FueraDeJuego {
    /// Qué tiene que hacer la persona, en la pausa con el robot real.
    pub(super) fn que_hacer(&self) -> String {
        match self {
            FueraDeJuego::Arco { izquierdo } => {
                format!("saca la pelota del arco {}", if *izquierdo { "izquierdo" } else { "derecho" })
            }
            FueraDeJuego::Area(a) => {
                format!("saca la pelota del área {}", if a.side < 0.0 { "izquierda" } else { "derecha" })
            }
            FueraDeJuego::Pegada => "despega la pelota de la pared".to_string(),
            FueraDeJuego::NoSeVe => "no se ve la pelota".to_string(),
        }
    }
}

/// La pelota en un arco o dentro de un área; `None` si está en juego.
pub(super) fn fuera_de_juego(pelota: Vec2) -> Option<FueraDeJuego> {
    if pelota.x.abs() > FIELD_HALF_X {
        return Some(FueraDeJuego::Arco { izquierdo: pelota.x < 0.0 });
    }
    areas().into_iter().find(|a| a.touches_with(pelota, 0.0)).map(FueraDeJuego::Area)
}

/// Distancia de la pelota a la pared más cercana (m).
pub(super) fn distancia_a_pared(p: Vec2) -> f32 {
    (FIELD_HALF_X - p.x.abs()).min(FIELD_HALF_Y - p.y.abs())
}

/// El rescatador (D3): el candidato más cercano a la pelota en línea recta; con un empate
/// dentro de `EMPATE_M`, `preferido` (el dueño del próximo paso con pelota).
pub(super) fn elegir_rescatador(pelota: Vec2, candidatos: &[(usize, Vec2)], preferido: Option<usize>) -> Option<usize> {
    let dist = |p: Vec2| (p - pelota).length();
    let (mejor, d_mejor) = candidatos
        .iter()
        .map(|&(k, p)| (k, dist(p)))
        .min_by(|a, b| a.1.total_cmp(&b.1))?;
    let pref = preferido.and_then(|k| candidatos.iter().find(|(j, _)| *j == k));
    match pref {
        Some(&(k, p)) if dist(p) <= d_mejor + EMPATE_M => Some(k),
        _ => Some(mejor),
    }
}

/// Dónde espera un robot que no va por la pelota (D3): su punto de referencia, si queda a
/// `resguardo` o más de la pelota y es legal; si no, a `resguardo` de la pelota sobre la
/// recta pelota → robot (girándola de a 15° hasta que quede en la zona segura y fuera de las
/// áreas).
pub(super) fn punto_resguardo(referencia: Vec2, robot: Vec2, pelota: Vec2, resguardo: f32, zona: f32) -> Vec2 {
    let legal = |q: Vec2| fuera_de_zona(q, zona).is_none() && !areas().iter().any(|a| a.touches_with(q, AREA_CLEARANCE));
    if (referencia - pelota).length() >= resguardo && legal(referencia) {
        return referencia;
    }
    let mut d = (robot - pelota).normalize_or_zero();
    if d == Vec2::ZERO {
        d = (-pelota).normalize_or_zero();
        if d == Vec2::ZERO {
            d = Vec2::X;
        }
    }
    for j in 0..24 {
        let grados = ((j + 1) / 2) as f32 * 15.0 * if j % 2 == 0 { 1.0 } else { -1.0 };
        let q = pelota + Vec2::from_angle(grados.to_radians()).rotate(d) * resguardo;
        if legal(q) {
            return q;
        }
    }
    pelota + d * resguardo
}

/// Dirección del despegue (D5): a lo largo de la pared más cercana, en los dos sentidos,
/// girada hacia adentro desde `max_deg` hasta 0° de a 1°; la primera con lugar legal detrás
/// (`legal`) y que vaya hacia el centro de la cancha (hacia la esquina la deja trabada). Con
/// el mismo ángulo en los dos sentidos, la que acerca la pelota a `hacia`. `None` si ninguna
/// sirve (esquina, o la pelota tocando la pared).
pub(super) fn direccion_despegue(pelota: Vec2, hacia: Option<Vec2>, max_deg: f32, legal: impl Fn(Vec2) -> bool) -> Option<Vec2> {
    let banda = FIELD_HALF_Y - pelota.y.abs() <= FIELD_HALF_X - pelota.x.abs();
    let (t, adentro) = if banda {
        (Vec2::X, Vec2::new(0.0, -pelota.y.signum()))
    } else {
        (Vec2::Y, Vec2::new(-pelota.x.signum(), 0.0))
    };
    for grados in (0..=max_deg.floor() as i32).rev() {
        let a = (grados as f32).to_radians();
        let legales: Vec<Vec2> = [t, -t].into_iter().map(|s| s * a.cos() + adentro * a.sin())
            .filter(|d| d.dot(-pelota) > 0.0 && legal(*d))
            .collect();
        let mejor = match hacia {
            Some(h) => legales.into_iter().max_by(|x, y| x.dot(h - pelota).total_cmp(&y.dot(h - pelota))),
            None => legales.into_iter().next(),
        };
        if mejor.is_some() {
            return mejor;
        }
    }
    None
}

/// Si un empuje de despegue en la dirección `d` deja lugar legal detrás de la pelota.
pub(super) fn despegue_legal(pelota: Vec2, d: Vec2) -> bool {
    sin_lugar_detras(&ApproachAlignedSkill::new(pelota + d * DESPEGUE_ALCANCE_M), pelota).is_none()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_nearest_goes_and_a_tie_goes_to_the_owner() {
        let pelota = Vec2::new(0.0, 0.0);
        let c = [(0, Vec2::new(0.30, 0.0)), (1, Vec2::new(0.10, 0.0)), (2, Vec2::new(0.13, 0.0))];
        assert_eq!(elegir_rescatador(pelota, &c, None), Some(1));
        // El dueño del paso a 0.03 m del más cercano: empate, va el dueño.
        assert_eq!(elegir_rescatador(pelota, &c, Some(2)), Some(2));
        // A 0.20 m: va el más cercano.
        assert_eq!(elegir_rescatador(pelota, &c, Some(0)), Some(1));
        assert_eq!(elegir_rescatador(pelota, &[], None), None);
    }

    #[test]
    fn the_others_keep_their_distance_inside_the_safe_zone() {
        let (resguardo, zona) = (0.30, 0.10);
        let pelota = Vec2::new(0.0, 0.0);
        // Su referencia está lejos de la pelota: ahí.
        let lejos = Vec2::new(0.40, 0.0);
        assert_eq!(punto_resguardo(lejos, Vec2::new(0.1, 0.0), pelota, resguardo, zona), lejos);
        // Cerca: a 0.30 m de la pelota, del lado del robot.
        let q = punto_resguardo(Vec2::new(0.05, 0.0), Vec2::new(0.1, 0.0), pelota, resguardo, zona);
        assert!((q - Vec2::new(0.30, 0.0)).length() < 1e-5, "{q:?}");
        // Pelota junto a la banda y el robot entre ella y la pared: gira hasta la zona segura.
        let pelota = Vec2::new(0.0, 0.50);
        let q = punto_resguardo(pelota, Vec2::new(0.0, 0.55), pelota, resguardo, zona);
        assert!(((q - pelota).length() - resguardo).abs() < 1e-4, "{q:?}");
        assert!(fuera_de_zona(q, zona).is_none(), "{q:?}");
        // Pelota frente al área: el punto no toca el área.
        let pelota = Vec2::new(0.42, 0.0);
        let q = punto_resguardo(pelota, Vec2::new(0.50, 0.0), pelota, resguardo, zona);
        assert!(!areas().iter().any(|a| a.touches_with(q, AREA_CLEARANCE)), "{q:?}");
    }

    #[test]
    fn a_ball_near_the_side_wall_is_pushed_along_it_with_an_inward_angle() {
        // Como los `sin_lugar` medidos en FIRASim: hacia el receptor no hay lugar detrás.
        let pelota = Vec2::new(-0.27, -0.57);
        let receptor = Vec2::new(0.40, 0.0);
        assert!(sin_lugar_detras(&ApproachAlignedSkill::new(receptor), pelota).is_some());
        let d = direccion_despegue(pelota, Some(receptor), 45.0, |d| despegue_legal(pelota, d)).expect("hay despegue");
        let adentro = d.y.atan2(d.x.abs()).to_degrees();
        assert!((10.0..=30.0).contains(&adentro), "{adentro:.1}° hacia adentro: {d:?}");
        assert!(d.x > 0.0, "hacia el receptor: {d:?}");
    }

    #[test]
    fn a_ball_in_a_corner_has_no_way_out() {
        for pelota in [Vec2::new(0.67, -0.55), Vec2::new(-0.66, 0.56)] {
            assert_eq!(direccion_despegue(pelota, Some(Vec2::ZERO), 45.0, |d| despegue_legal(pelota, d)), None, "{pelota:?}");
        }
    }

    #[test]
    fn a_ball_in_a_goal_or_an_area_is_out_of_play() {
        assert_eq!(fuera_de_juego(Vec2::new(0.78, 0.05)), Some(FueraDeJuego::Arco { izquierdo: false }));
        assert_eq!(fuera_de_juego(Vec2::new(0.65, 0.20)), Some(FueraDeJuego::Area(AreaRect::opp(1.0))));
        assert_eq!(fuera_de_juego(Vec2::new(0.30, 0.20)), None);
        assert_eq!(FueraDeJuego::Arco { izquierdo: true }.que_hacer(), "saca la pelota del arco izquierdo");
        assert_eq!(FueraDeJuego::Area(AreaRect::opp(1.0)).que_hacer(), "saca la pelota del área derecha");
    }
}
