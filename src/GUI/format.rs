//! Formato de números y textos de la GUI:
//! punto decimal, signo menos tipográfico (U+2212), un espacio antes de "%" y de las
//! unidades, y listas en castellano ("0, 1 y 2").

/// Signo menos tipográfico.
pub const MENOS: char = '\u{2212}';

/// `x` con `decimales` decimales y el signo menos tipográfico. Un valor que redondea
/// a cero se escribe sin signo ("0.00", no "−0.00"). No finito: "—".
pub fn num(x: f32, decimales: usize) -> String {
    if !x.is_finite() {
        return "—".to_string();
    }
    let s = format!("{:.*}", decimales, x.abs());
    let es_cero = s.chars().all(|c| c == '0' || c == '.');
    if x < 0.0 && !es_cero {
        format!("{MENOS}{s}")
    } else {
        s
    }
}

/// Número con unidad: "0.62 m/s".
pub fn con_unidad(x: f32, decimales: usize, unidad: &str) -> String {
    format!("{} {unidad}", num(x, decimales))
}

/// Porcentaje entero: "93 %".
pub fn pct(fraccion: f32) -> String {
    format!("{} %", num(fraccion * 100.0, 0))
}

/// Lista en castellano: "0", "0 y 2", "0, 1 y 2".
pub fn lista<T: std::fmt::Display>(items: &[T]) -> String {
    match items {
        [] => String::new(),
        [a] => a.to_string(),
        [init @ .., ultimo] => {
            let cabeza: Vec<String> = init.iter().map(|x| x.to_string()).collect();
            format!("{} y {ultimo}", cabeza.join(", "))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn negative_numbers_use_the_typographic_minus() {
        assert_eq!(num(-0.6, 2), "\u{2212}0.60");
        assert_eq!(con_unidad(-0.6, 2, "m/s"), "\u{2212}0.60 m/s");
        assert_eq!(num(1.396, 2), "1.40");
    }

    #[test]
    fn zero_has_no_sign() {
        assert_eq!(num(-0.001, 2), "0.00");
        assert_eq!(num(-0.0, 1), "0.0");
    }

    #[test]
    fn non_finite_is_a_dash() {
        assert_eq!(num(f32::NAN, 2), "—");
    }

    #[test]
    fn percent_has_a_space() {
        assert_eq!(pct(0.928), "93 %");
        assert_eq!(pct(1.0), "100 %");
    }

    #[test]
    fn spanish_lists() {
        assert_eq!(lista::<u32>(&[]), "");
        assert_eq!(lista(&[2]), "2");
        assert_eq!(lista(&[0, 2]), "0 y 2");
        assert_eq!(lista(&[0, 1, 2]), "0, 1 y 2");
    }
}
