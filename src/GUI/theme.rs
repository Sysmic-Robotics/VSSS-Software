//! Sistema de diseño central de la GUI (dark técnico moderno, acento cyan/verde).
//!
//! Fuente única de verdad para colores, tipografía y espaciados. El resto de los
//! módulos de la GUI SHALL referenciar estos tokens en vez de literales
//! `Color::from_rgb(...)` dispersos. Cambiar un token aquí lo cambia en todos los
//! paneles que lo usan.
//!
//! Este módulo es puramente de presentación: no altera mensajes, canales ni lógica.

use iced::widget::container;
use iced::{Border, Color, Theme};

/// Construye un `Color` opaco constante a partir de componentes 0..1.
const fn rgb(r: f32, g: f32, b: f32) -> Color {
    Color { r, g, b, a: 1.0 }
}

/// Construye un `Color` con alfa constante.
const fn rgba(r: f32, g: f32, b: f32, a: f32) -> Color {
    Color { r, g, b, a }
}

// ---------------------------------------------------------------------------
// Paleta — fondos y superficies
// ---------------------------------------------------------------------------

/// Fondo base de la ventana (azul-gris muy oscuro).
pub const BG: Color = rgb(0.07, 0.09, 0.11);
/// Fondo de un panel/card sobre el base.
pub const BG_PANEL: Color = rgb(0.11, 0.13, 0.16);
/// Fondo de una superficie elevada (chart, contenedor interno).
pub const BG_ELEVATED: Color = rgb(0.09, 0.11, 0.13);
/// Borde neutro de contenedores.
pub const BORDER: Color = rgb(0.20, 0.24, 0.28);

// ---------------------------------------------------------------------------
// Paleta — texto
// ---------------------------------------------------------------------------

/// Texto principal.
pub const TEXT: Color = rgb(0.88, 0.91, 0.94);
/// Texto atenuado (ayudas, notas, ejes).
pub const TEXT_DIM: Color = rgb(0.55, 0.60, 0.66);

// ---------------------------------------------------------------------------
// Paleta — acentos y estados
// ---------------------------------------------------------------------------

/// Acento primario (cyan/teal) — selección, primario de widgets.
pub const ACCENT: Color = rgb(0.13, 0.78, 0.83);
/// Estado OK / conectado.
pub const OK: Color = rgb(0.22, 0.85, 0.45);
/// Estado de advertencia (ámbar).
pub const WARN: Color = rgb(0.95, 0.72, 0.22);
/// Estado de error / desconectado.
pub const ERR: Color = rgb(0.95, 0.34, 0.34);
/// Estado neutro / sin datos.
pub const NEUTRAL: Color = rgb(0.55, 0.60, 0.66);

// ---------------------------------------------------------------------------
// Paleta — datos (charts)
// ---------------------------------------------------------------------------

/// Serie de datos "izquierda" (rueda L) — cyan.
pub const DATA_L: Color = rgb(0.15, 0.75, 1.0);
/// Serie de datos "derecha" (rueda R) — magenta.
pub const DATA_R: Color = rgb(1.0, 0.35, 0.75);
/// Barras de paquetes (chart de visión).
pub const DATA_BARS: Color = rgb(0.13, 0.78, 0.83);
/// Líneas de grilla en charts.
pub const GRID: Color = rgba(0.55, 0.60, 0.66, 0.28);
/// Texto de ejes en charts.
pub const AXIS_TEXT: Color = rgb(0.70, 0.75, 0.80);

// ---------------------------------------------------------------------------
// Paleta — cancha
// ---------------------------------------------------------------------------

/// Verde del campo (menos saturado que el original, más agradable).
pub const FIELD_GREEN: Color = rgb(0.13, 0.42, 0.21);
/// Verde del margen exterior (más oscuro → jerarquía respecto al campo).
pub const FIELD_MARGIN: Color = rgb(0.09, 0.28, 0.15);
/// Líneas del campo (blanco suave, no puro).
pub const FIELD_LINE: Color = rgb(0.90, 0.93, 0.95);
/// Relleno de los arcos.
pub const GOAL_FILL: Color = rgb(0.16, 0.18, 0.22);

// ---------------------------------------------------------------------------
// Paleta — robots, pelota y marcadores
// ---------------------------------------------------------------------------

/// Robot del equipo 0 (azul).
pub const TEAM_BLUE: Color = rgb(0.20, 0.48, 1.0);
/// Robot del equipo 1 (amarillo).
pub const TEAM_YELLOW: Color = rgb(1.0, 0.85, 0.12);
/// Borde de robot / línea de orientación.
pub const ROBOT_OUTLINE: Color = rgb(0.04, 0.05, 0.07);
/// Etiqueta (número) del robot.
pub const ROBOT_LABEL: Color = rgb(0.96, 0.97, 0.99);
/// Pelota (rojo, semántica conservada).
pub const BALL: Color = rgb(0.95, 0.26, 0.16);
/// Halo del robot seleccionado (naranja de acento).
pub const SELECT_HALO: Color = rgb(1.0, 0.60, 0.15);
/// Vector de velocidad medida (visión) — naranja.
pub const MEASURED_VEL: Color = rgb(1.0, 0.62, 0.10);
/// Flecha de velocidad comandada.
pub const CMD_ARROW: Color = rgb(0.95, 0.97, 1.0);
/// Traza del robot seleccionado (celeste).
pub const TRACE: Color = rgb(0.45, 0.85, 1.0);
/// Marcador de target de motion (cyan).
pub const TARGET: Color = rgb(0.10, 0.90, 0.95);
/// Marcador de target de la skill de GUI activa (verde).
pub const SKILL_TARGET: Color = rgb(0.18, 1.0, 0.45);

// ---------------------------------------------------------------------------
// Tipografía (tamaños en px)
// ---------------------------------------------------------------------------

/// Extra chico — notas, ayudas, ejes.
pub const FS_XS: u16 = 10;
/// Chico — cuerpo estándar de paneles.
pub const FS_SM: u16 = 12;
/// Medio — controles principales.
pub const FS_MD: u16 = 13;
/// Grande — títulos de sección.
pub const FS_LG: u16 = 14;
/// Extra grande — títulos de panel / barra superior.
pub const FS_XL: u16 = 16;

// ---------------------------------------------------------------------------
// Espaciados (px)
// ---------------------------------------------------------------------------

pub const SP_XS: u16 = 4;
pub const SP_SM: u16 = 6;
pub const SP_MD: u16 = 8;
pub const SP_LG: u16 = 12;

// ---------------------------------------------------------------------------
// Theme custom de iced
// ---------------------------------------------------------------------------

/// Theme propio (dark técnico) derivado de la paleta central. Los widgets estándar
/// (botones, inputs, contenedores, sliders) heredan sus colores de esta palette.
pub fn theme() -> Theme {
    Theme::custom(
        "VSSS Dark".to_string(),
        iced::theme::Palette {
            background: BG,
            text: TEXT,
            primary: ACCENT,
            success: OK,
            danger: ERR,
        },
    )
}

/// Estilo de contenedor tipo "card": fondo de panel, borde neutro y esquinas
/// redondeadas. Usado por los paneles del sidebar para un look consistente.
pub fn card(_theme: &Theme) -> container::Style {
    container::Style {
        background: Some(BG_PANEL.into()),
        border: Border {
            color: BORDER,
            width: 1.0,
            radius: 8.0.into(),
        },
        ..Default::default()
    }
}
