//! Registro de cartas, etapa 1 del selector de jugadas.
//!
//! En cada decisión del coach, las jugadas posibles del atacante ("cartas": tiro directo, tiro
//! de giro, pase y desvío) con los datos que deciden si sirven (arco abierto, recta libre,
//! distancias, quién llega primero a la pelota) y lo que eligió el coach. Una línea JSON por
//! decisión, y una línea por cada cambio de jugada del árbitro (para saber después en qué
//! terminó cada decisión: gol, pérdida, avance). No cambia cómo juega: es la materia prima para
//! estimar la probabilidad de éxito de cada carta (etapa 2), con partidos del simulador y con
//! los partidos reales grabados.
//!
//! Va junto al registro de partido (`VSSL_MATCH_LOG=x.csv` → `x.cartas.jsonl`), así los
//! partidos grabados con el lanzador lo traen sin configurar nada, o en `VSSL_CARTAS_LOG`. Pesa
//! ~0.5 MB por partido de 3 minutos (el CSV de partido, ~20 MB).

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

/// Ruta explícita del registro de cartas.
pub const CARTAS_LOG_ENV: &str = "VSSL_CARTAS_LOG";
/// Registro de partido junto al que va el de cartas.
const MATCH_LOG_ENV: &str = "VSSL_MATCH_LOG";

/// Ruta del registro de cartas: `cartas` si viene, si no junto al registro de partido
/// (`x.csv` → `x.cartas.jsonl`). `None` sin ninguno de los dos.
pub fn ruta(cartas: Option<&str>, partido: Option<&str>) -> Option<PathBuf> {
    if let Some(c) = cartas.map(str::trim).filter(|c| !c.is_empty()) {
        return Some(PathBuf::from(c));
    }
    let p = Path::new(partido.map(str::trim).filter(|p| !p.is_empty())?);
    let stem = p.file_stem()?.to_string_lossy().into_owned();
    Some(p.with_file_name(format!("{stem}.cartas.jsonl")))
}

/// Archivo de cartas abierto.
pub struct RegistroCartas {
    out: BufWriter<File>,
}

impl RegistroCartas {
    /// Abre el registro según `VSSL_CARTAS_LOG` o `VSSL_MATCH_LOG`; `None` sin ninguno (o si no
    /// se puede crear: avisa y el coach sigue sin registro).
    pub fn desde_entorno() -> Option<Self> {
        let r = ruta(
            std::env::var(CARTAS_LOG_ENV).ok().as_deref(),
            std::env::var(MATCH_LOG_ENV).ok().as_deref(),
        )?;
        match File::create(&r) {
            Ok(f) => {
                eprintln!("[coach] registro de cartas → {}", r.display());
                Some(Self { out: BufWriter::new(f) })
            }
            Err(e) => {
                eprintln!("[coach] no se pudo abrir el registro de cartas {}: {e}", r.display());
                None
            }
        }
    }

    /// Escribe una línea (y la vacía al disco: si el engine se corta, se pierde a lo más esa).
    pub fn escribir(&mut self, linea: &serde_json::Value) {
        let _ = writeln!(self.out, "{linea}");
        let _ = self.out.flush();
    }
}

/// Redondeo a milímetros / milésimas para que el registro pese poco.
pub fn r3(x: f32) -> f64 {
    if x.is_finite() { (x as f64 * 1000.0).round() / 1000.0 } else { f64::NAN }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_cards_log_goes_next_to_the_match_log() {
        assert_eq!(ruta(None, Some("logs/salita/5nov/1400_blue.csv")), Some(PathBuf::from("logs/salita/5nov/1400_blue.cartas.jsonl")));
        assert_eq!(ruta(Some(" /tmp/c.jsonl "), Some("x.csv")), Some(PathBuf::from("/tmp/c.jsonl")));
        assert_eq!(ruta(None, None), None);
        assert_eq!(ruta(Some(""), Some("  ")), None);
    }

    #[test]
    fn rounding_keeps_millimetres() {
        assert_eq!(r3(0.123456), 0.123);
        assert!(r3(f32::INFINITY).is_nan());
    }
}
