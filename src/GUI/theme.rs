//! Sistema de diseño de la GUI: la interfaz se ve como la mesa
//! de la salita; el único color es el del juego y la interfaz en sí es neutra.
//!
//! Fuente única de los tokens de la especificación de la GUI (§2) y de los estilos de
//! cada widget. Los demás módulos usan estos tokens, nunca literales de color. Es
//! puramente de presentación: no altera mensajes, canales ni lógica.

use iced::widget::{button, container, rule, text, text_input};
use iced::{Background, Border, Color, Shadow, Theme};

/// `Color` opaco desde `0xRRGGBB`.
pub const fn hex(rgb: u32) -> Color {
    Color {
        r: ((rgb >> 16) & 0xff) as f32 / 255.0,
        g: ((rgb >> 8) & 0xff) as f32 / 255.0,
        b: (rgb & 0xff) as f32 / 255.0,
        a: 1.0,
    }
}

/// El mismo color con otra opacidad (para dar jerarquía sobre el fieltro).
pub const fn alfa(c: Color, a: f32) -> Color {
    Color { a, ..c }
}

// ---------------------------------------------------------------------------
// Tokens de la especificación (§2)
// ---------------------------------------------------------------------------

/// Fondo de la ventana.
pub const MESA: Color = hex(0xdfe2dc);
/// Superficie de los paneles.
pub const HOJA: Color = hex(0xeef0eb);
/// Texto principal y curva "medida" de los gráficos.
pub const TINTA: Color = hex(0x1b211d);
/// Texto secundario.
pub const GRIS: Color = hex(0x5d665f);
/// Separadores.
pub const RAYA: Color = hex(0xc4c9c1);
/// Cancha (verde de la salita, por defecto).
pub const FIELTRO: Color = hex(0x2e6a3b);
/// Paredes, triángulos y cuerpo de los robots.
pub const PARED: Color = hex(0x151515);
/// Líneas de la cancha y todo lo dibujado sobre el fieltro.
pub const CAL: Color = hex(0xf3f3ee);
/// Equipo azul; curva "pedida".
pub const AZUL: Color = hex(0x1d5fd0);
/// Equipo amarillo.
pub const AMARILLO: Color = hex(0xe3b10a);
/// Pelota.
pub const PELOTA: Color = hex(0xf2691c);
/// Lo anormal: avisos, guardia, escapes, parada.
pub const ALERTA: Color = hex(0xc4361f);

// ---------------------------------------------------------------------------
// Derivados
// ---------------------------------------------------------------------------

/// Fieltro negro mate del reglamento.
pub const FIELTRO_NEGRO: Color = hex(0x141414);
/// Fondo de la fila seleccionada. El `#e3e8e0` del mockup deja el texto en alerta en
/// 4.34 de contraste; este llega a 4.51 (AA).
pub const SELECCION: Color = hex(0xe8ece5);
/// Borde de la pelota.
pub const PELOTA_BORDE: Color = hex(0xa8420a);
/// Marcas de la regla grabada sobre la pared.
pub const REGLA: Color = hex(0xe9e9e2);
/// Fondo de los gráficos y pista de las barras.
pub const PISTA: Color = hex(0xd3d8cf);
/// Texto sobre el botón de parada.
pub const BLANCO: Color = Color::WHITE;

/// Fondo de un arco: el fieltro algo más oscuro.
pub fn fondo_arco(fieltro: Color) -> Color {
    Color {
        r: fieltro.r * 0.86,
        g: fieltro.g * 0.86,
        b: fieltro.b * 0.86,
        a: 1.0,
    }
}

// ---------------------------------------------------------------------------
// Tipografía (px) y espaciados (px)
// ---------------------------------------------------------------------------

/// Cuerpo de texto.
pub const TXT: f32 = 14.0;
/// Notas y datos secundarios.
pub const TXT_CHICO: f32 = 13.0;
/// Etiquetas pequeñas (ejes, regla).
pub const TXT_MINI: f32 = 12.0;
/// Pestañas.
pub const TXT_PESTANA: f32 = 15.0;
/// Títulos de panel.
pub const TXT_TITULO: f32 = 17.0;
/// Nombre de robot y título de la barra de estado.
pub const TXT_NOMBRE: f32 = 18.0;

pub const SP_XS: f32 = 4.0;
pub const SP_SM: f32 = 6.0;
pub const SP_MD: f32 = 10.0;
pub const SP_LG: f32 = 14.0;

/// Radio de esquina de paneles y botones.
pub const RADIO: f32 = 3.0;

// ---------------------------------------------------------------------------
// Theme de iced y estilos por widget
// ---------------------------------------------------------------------------

/// Theme claro "mesa". Los widgets que no tienen estilo explícito heredan de aquí.
pub fn theme() -> Theme {
    Theme::custom(
        "Sysmic mesa".to_string(),
        iced::theme::Palette {
            background: MESA,
            text: TINTA,
            primary: AZUL,
            success: TINTA,
            danger: ALERTA,
        },
    )
}

fn borde(color: Color, width: f32) -> Border {
    Border {
        color,
        width,
        radius: RADIO.into(),
    }
}

/// Panel plano de hoja: radio 3 px, sin borde ni sombra.
pub fn panel(_theme: &Theme) -> container::Style {
    container::Style {
        text_color: Some(TINTA),
        background: Some(Background::Color(HOJA)),
        border: borde(Color::TRANSPARENT, 0.0),
        shadow: Shadow::default(),
    }
}

/// Botón plano sobre la mesa, con filete raya.
pub fn boton(_theme: &Theme, status: button::Status) -> button::Style {
    let fondo = match status {
        button::Status::Hovered => SELECCION,
        button::Status::Pressed => PISTA,
        _ => MESA,
    };
    button::Style {
        background: Some(Background::Color(fondo)),
        text_color: if status == button::Status::Disabled { GRIS } else { TINTA },
        border: borde(RAYA, 1.0),
        shadow: Shadow::default(),
    }
}

/// Botón de una opción elegida (filete en tinta).
pub fn boton_activo(theme: &Theme, status: button::Status) -> button::Style {
    button::Style {
        border: borde(TINTA, 1.5),
        ..boton(theme, status)
    }
}

/// Botón normal o elegido según `activo`.
pub fn boton_opcion(activo: bool) -> impl Fn(&Theme, button::Status) -> button::Style {
    move |theme, status| {
        if activo {
            boton_activo(theme, status)
        } else {
            boton(theme, status)
        }
    }
}

/// "Detener todo": alerta con texto blanco. Con la parada activa ("Soltar la
/// parada"), botón con filete en alerta.
pub fn boton_parada(activa: bool) -> impl Fn(&Theme, button::Status) -> button::Style {
    move |theme, status| {
        if activa {
            button::Style {
                text_color: ALERTA,
                border: borde(ALERTA, 2.0),
                ..boton(theme, status)
            }
        } else {
            let fondo = match status {
                button::Status::Hovered | button::Status::Pressed => hex(0xa82d19),
                _ => ALERTA,
            };
            button::Style {
                background: Some(Background::Color(fondo)),
                text_color: BLANCO,
                border: borde(Color::TRANSPARENT, 0.0),
                shadow: Shadow::default(),
            }
        }
    }
}

/// Pestaña: texto gris, o tinta con subrayado de 2 px si está activa (el subrayado lo
/// dibuja la vista con un filete; el botón queda sin fondo).
pub fn pestana(activa: bool) -> impl Fn(&Theme, button::Status) -> button::Style {
    move |_theme, status| button::Style {
        background: None,
        text_color: if activa || status == button::Status::Hovered { TINTA } else { GRIS },
        border: borde(Color::TRANSPARENT, 0.0),
        shadow: Shadow::default(),
    }
}

/// Fila de la planilla de robots (un botón sin borde): seleccionada, fondo
/// `SELECCION`; la barra de 3 px en tinta la agrega la vista.
pub fn fila(seleccionada: bool) -> impl Fn(&Theme, button::Status) -> button::Style {
    move |_theme, status| {
        let fondo = if seleccionada || status == button::Status::Hovered {
            Some(Background::Color(SELECCION))
        } else {
            None
        };
        button::Style {
            background: fondo,
            text_color: TINTA,
            border: Border::default(),
            shadow: Shadow::default(),
        }
    }
}

/// Fondo tinta del contenedor de la fila seleccionada: asoma como una barra de 3 px
/// a la izquierda (el resto lo tapa la fila).
pub fn barra_seleccion(activa: bool) -> impl Fn(&Theme) -> container::Style {
    move |_theme| container::Style {
        background: activa.then_some(Background::Color(TINTA)),
        ..container::Style::default()
    }
}

/// Entrada de texto: hoja con filete raya; con el foco, borde azul de 2 px.
pub fn entrada(_theme: &Theme, status: text_input::Status) -> text_input::Style {
    let border = match status {
        text_input::Status::Focused => borde(AZUL, 2.0),
        text_input::Status::Hovered => borde(GRIS, 1.0),
        _ => borde(RAYA, 1.0),
    };
    text_input::Style {
        background: Background::Color(HOJA),
        border,
        icon: GRIS,
        placeholder: GRIS,
        value: TINTA,
        selection: alfa(AZUL, 0.3),
    }
}

/// Filete separador de 1 px.
pub fn filete(_theme: &Theme) -> rule::Style {
    rule::Style {
        color: RAYA,
        width: 1,
        radius: 0.0.into(),
        fill_mode: rule::FillMode::Full,
    }
}

/// Subrayado de 2 px de la pestaña activa.
pub fn subrayado(_theme: &Theme) -> rule::Style {
    rule::Style {
        color: TINTA,
        width: 2,
        radius: 0.0.into(),
        fill_mode: rule::FillMode::Full,
    }
}

/// Texto secundario.
pub fn gris(_theme: &Theme) -> text::Style {
    text::Style { color: Some(GRIS) }
}

/// Texto de lo anormal.
pub fn alerta(_theme: &Theme) -> text::Style {
    text::Style { color: Some(ALERTA) }
}

/// Texto en alerta o en tinta según `es_alerta`.
pub fn tinta_o_alerta(es_alerta: bool) -> impl Fn(&Theme) -> text::Style {
    move |_theme| text::Style {
        color: Some(if es_alerta { ALERTA } else { TINTA }),
    }
}

// ---------------------------------------------------------------------------
// Contraste (WCAG 2.1)
// ---------------------------------------------------------------------------

/// Luminancia relativa WCAG de un color opaco.
#[cfg(test)]
fn luminancia(c: Color) -> f32 {
    let lin = |v: f32| {
        if v <= 0.03928 {
            v / 12.92
        } else {
            ((v + 0.055) / 1.055).powf(2.4)
        }
    };
    0.2126 * lin(c.r) + 0.7152 * lin(c.g) + 0.0722 * lin(c.b)
}

/// Contraste WCAG entre dos colores opacos (1 a 21).
#[cfg(test)]
pub fn contraste(a: Color, b: Color) -> f32 {
    let (la, lb) = (luminancia(a), luminancia(b));
    let (hi, lo) = if la > lb { (la, lb) } else { (lb, la) };
    (hi + 0.05) / (lo + 0.05)
}

/// Pares texto/fondo que usa la GUI. El texto en alerta nunca va sobre la mesa
/// (4.12): la barra de estado es una franja de hoja. Si se agrega un par en la vista,
/// va aquí, y el test de contraste lo revisa.
#[cfg(test)]
pub const PARES_TEXTO: [(&str, Color, Color); 11] = [
    ("tinta sobre hoja", TINTA, HOJA),
    ("tinta sobre mesa", TINTA, MESA),
    ("tinta sobre la fila seleccionada", TINTA, SELECCION),
    ("gris sobre hoja", GRIS, HOJA),
    ("gris sobre mesa (regla)", GRIS, MESA),
    ("gris sobre la fila seleccionada", GRIS, SELECCION),
    ("alerta sobre hoja", ALERTA, HOJA),
    ("alerta sobre la fila seleccionada", ALERTA, SELECCION),
    ("blanco sobre alerta (parada)", BLANCO, ALERTA),
    ("cal sobre el fieltro verde", CAL, FIELTRO),
    ("cal sobre el fieltro negro", CAL, FIELTRO_NEGRO),
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tokens_match_the_specification() {
        let rgb8 = |c: Color| {
            (
                (c.r * 255.0).round() as u8,
                (c.g * 255.0).round() as u8,
                (c.b * 255.0).round() as u8,
            )
        };
        assert_eq!(rgb8(MESA), (0xdf, 0xe2, 0xdc));
        assert_eq!(rgb8(TINTA), (0x1b, 0x21, 0x1d));
        assert_eq!(rgb8(FIELTRO), (0x2e, 0x6a, 0x3b));
        assert_eq!(rgb8(ALERTA), (0xc4, 0x36, 0x1f));
        assert_eq!(rgb8(PELOTA), (0xf2, 0x69, 0x1c));
    }

    #[test]
    fn every_text_pair_is_aa() {
        let fallas: Vec<String> = PARES_TEXTO
            .iter()
            .filter(|(_, t, f)| contraste(*t, *f) < 4.5)
            .map(|(n, t, f)| format!("{n}: {:.2}", contraste(*t, *f)))
            .collect();
        assert!(fallas.is_empty(), "pares sin contraste AA: {fallas:?}");
    }

    #[test]
    fn alert_on_the_table_is_not_aa() {
        // Por esto la barra de estado va sobre hoja y no sobre la mesa.
        let c = contraste(ALERTA, MESA);
        assert!((c - 4.12).abs() < 0.02, "{c}");
        assert!(c < 4.5);
    }

    #[test]
    fn contrast_reference_values() {
        assert!((contraste(Color::BLACK, Color::WHITE) - 21.0).abs() < 0.01);
        assert!((contraste(TINTA, HOJA) - 14.28).abs() < 0.05);
        assert!((contraste(ALERTA, SELECCION) - 4.51).abs() < 0.02);
    }
}
