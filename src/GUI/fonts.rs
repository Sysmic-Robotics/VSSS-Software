//! Fuentes de la GUI, dentro del binario: Barlow (texto) y Barlow Semi Condensed
//! (títulos, números de robot, rótulos sobre la cancha y pestañas).
//!
//! Son versiones con las cifras tabulares por defecto y la familia renombrada (iced
//! 0.13 no aplica el rasgo `tnum`); ver `assets/fonts/LEEME.txt` y
//! `tools/fuentes_tabulares.py`. Licencia OFL 1.1 en `assets/fonts/OFL.txt`.

use iced::Font;
use iced::font::{Family, Weight};

pub const BARLOW_REGULAR: &[u8] = include_bytes!("../../assets/fonts/BarlowTabular-Regular.ttf");
pub const BARLOW_SEMIBOLD: &[u8] = include_bytes!("../../assets/fonts/BarlowTabular-SemiBold.ttf");
pub const BARLOW_SC_MEDIUM: &[u8] =
    include_bytes!("../../assets/fonts/BarlowSemiCondensedTabular-Medium.ttf");
pub const BARLOW_SC_SEMIBOLD: &[u8] =
    include_bytes!("../../assets/fonts/BarlowSemiCondensedTabular-SemiBold.ttf");

/// Todas las fuentes, para cargarlas con `iced::application(..).font(..)`.
pub const TODAS: [&[u8]; 4] = [BARLOW_REGULAR, BARLOW_SEMIBOLD, BARLOW_SC_MEDIUM, BARLOW_SC_SEMIBOLD];

const FAMILIA_TEXTO: &str = "Barlow Tabular";
const FAMILIA_TITULO: &str = "Barlow Semi Condensed Tabular";

/// Texto (fuente por defecto de la GUI).
pub const TEXTO: Font = Font {
    family: Family::Name(FAMILIA_TEXTO),
    ..Font::DEFAULT
};

/// Texto destacado.
pub const TEXTO_FUERTE: Font = Font {
    family: Family::Name(FAMILIA_TEXTO),
    weight: Weight::Semibold,
    ..Font::DEFAULT
};

/// Títulos, nombre de robot, rótulos sobre la cancha.
pub const TITULO: Font = Font {
    family: Family::Name(FAMILIA_TITULO),
    weight: Weight::Semibold,
    ..Font::DEFAULT
};

/// Pestañas.
pub const TITULO_MEDIO: Font = Font {
    family: Family::Name(FAMILIA_TITULO),
    weight: Weight::Medium,
    ..Font::DEFAULT
};

#[cfg(test)]
mod tests {
    use super::*;

    /// `texto` codificado como en la tabla `name` de la plataforma Windows (UTF-16BE).
    fn utf16be(texto: &str) -> Vec<u8> {
        texto.encode_utf16().flat_map(|u| u.to_be_bytes()).collect()
    }

    fn contiene(bytes: &[u8], aguja: &[u8]) -> bool {
        bytes.windows(aguja.len()).any(|w| w == aguja)
    }

    /// Caracteres fuera de ASCII que la GUI puede mostrar y que Barlow tiene
    /// (verificado con fontTools sobre los archivos embebidos). Barlow no tiene
    /// griego (θ, ω) ni flechas (→): WSL no trae una fuente de respaldo y salen
    /// como cuadrados.
    const PERMITIDOS: &str = "°±²·¿¡ÁÉÍÓÚÑÜáéíóúñü×—…−≈≤≥%";

    /// Los textos de la GUI (literales fuera de comentarios) no usan caracteres
    /// que la fuente no tiene.
    #[test]
    fn ui_strings_only_use_glyphs_the_font_has() {
        let fuentes = [
            include_str!("mod.rs"),
            include_str!("tools.rs"),
            include_str!("inspector.rs"),
            include_str!("robot_rows.rs"),
            include_str!("status_line.rs"),
            include_str!("charts.rs"),
            include_str!("field.rs"),
            include_str!("format.rs"),
        ];
        let mut fallas = Vec::new();
        for fuente in fuentes {
            // Solo lo que no es test ni comentario.
            let codigo = fuente.split("#[cfg(test)]").next().unwrap_or("");
            for linea in codigo.lines() {
                let l = linea.trim_start();
                if l.starts_with("//") {
                    continue;
                }
                // Sin el comentario al final de la línea (si lo hay).
                let l = l.split(" // ").next().unwrap_or(l);
                for c in l.chars() {
                    if !c.is_ascii() && !PERMITIDOS.contains(c) {
                        fallas.push(format!("{c:?} en: {l}"));
                    }
                }
            }
        }
        assert!(fallas.is_empty(), "caracteres que Barlow no tiene: {fallas:#?}");
    }

    #[test]
    fn embedded_fonts_carry_their_family_name() {
        for (bytes, familia) in [
            (BARLOW_REGULAR, FAMILIA_TEXTO),
            (BARLOW_SEMIBOLD, FAMILIA_TEXTO),
            (BARLOW_SC_MEDIUM, FAMILIA_TITULO),
            (BARLOW_SC_SEMIBOLD, FAMILIA_TITULO),
        ] {
            assert!(bytes.len() > 50_000, "fuente vacía o truncada");
            assert_eq!(&bytes[..4], &[0, 1, 0, 0], "no es un TrueType");
            assert!(contiene(bytes, &utf16be(familia)), "sin la familia {familia}");
        }
    }
}
