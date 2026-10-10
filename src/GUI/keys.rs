//! Teclado de la GUI.
//!
//! El control manual por teclado está activo solo dentro del modo manual, al que se
//! entra con un botón. Dentro de ese modo, las letras mueven el robot; fuera, son de
//! las capas de la cancha (fase 2). Espacio detiene todo siempre. Las teclas que
//! captura un campo de texto no llegan aquí (`keyboard::on_key_press` solo entrega
//! las que ningún widget capturó).

use iced::keyboard::{Key, key::Named};

/// Tecla que le interesa a la GUI.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tecla {
    Espacio,
    Escape,
    /// Letra, en minúscula.
    Letra(char),
}

/// Traduce una tecla de iced. Otras teclas (flechas, F1, …) no interesan.
pub fn tecla_de(key: &Key) -> Option<Tecla> {
    match key {
        Key::Named(Named::Space) => Some(Tecla::Espacio),
        Key::Named(Named::Escape) => Some(Tecla::Escape),
        Key::Character(s) => s.chars().next().map(|c| Tecla::Letra(c.to_ascii_lowercase())),
        _ => None,
    }
}

/// Capas de la cancha (fase 2). Declaradas con su tecla; en esta fase no dibujan.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Layer {
    Navegacion,
    Coach,
    Reflejos,
    Trazas,
    VisionCruda,
    Skills,
}

impl Layer {
    pub const ALL: [Layer; 6] = [
        Layer::Navegacion,
        Layer::Coach,
        Layer::Reflejos,
        Layer::Trazas,
        Layer::VisionCruda,
        Layer::Skills,
    ];

    /// Tecla de la capa (§5 de la especificación).
    pub fn tecla(self) -> char {
        match self {
            Layer::Navegacion => 'n',
            Layer::Coach => 'c',
            Layer::Reflejos => 'r',
            Layer::Trazas => 't',
            Layer::VisionCruda => 'v',
            Layer::Skills => 's',
        }
    }

    pub fn de_tecla(c: char) -> Option<Layer> {
        Layer::ALL.into_iter().find(|l| l.tecla() == c)
    }
}

/// Teclas que mueven el robot en el modo manual.
pub const TECLAS_MANUAL: [char; 6] = ['w', 'a', 's', 'd', 'q', 'e'];

/// Qué hace una tecla apretada.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AccionTecla {
    /// Activa la parada de emergencia.
    Parada,
    /// Sale del modo manual.
    SalirManual,
    /// Tecla de movimiento del modo manual.
    Mover(char),
    /// Prende o apaga una capa (sin efecto hasta la fase 2).
    Capa(Layer),
    Nada,
}

/// Efecto de una tecla apretada según el modo. Dentro del modo manual, ninguna letra
/// va a las capas; fuera de él, ninguna mueve el robot, y ninguna tecla entra al modo
/// manual.
pub fn ruta_tecla(tecla: Tecla, modo_manual: bool) -> AccionTecla {
    match (tecla, modo_manual) {
        (Tecla::Espacio, _) => AccionTecla::Parada,
        (Tecla::Escape, true) => AccionTecla::SalirManual,
        (Tecla::Escape, false) => AccionTecla::Nada,
        (Tecla::Letra(c), true) if TECLAS_MANUAL.contains(&c) => AccionTecla::Mover(c),
        (Tecla::Letra(_), true) => AccionTecla::Nada,
        (Tecla::Letra(c), false) => Layer::de_tecla(c).map_or(AccionTecla::Nada, AccionTecla::Capa),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn outside_manual_s_belongs_to_the_layers() {
        assert_eq!(ruta_tecla(Tecla::Letra('s'), false), AccionTecla::Capa(Layer::Skills));
    }

    #[test]
    fn inside_manual_s_moves_back() {
        assert_eq!(ruta_tecla(Tecla::Letra('s'), true), AccionTecla::Mover('s'));
    }

    #[test]
    fn layer_letter_inside_manual_does_nothing() {
        for c in ['n', 'c', 'r', 't', 'v'] {
            assert_eq!(ruta_tecla(Tecla::Letra(c), true), AccionTecla::Nada, "{c}");
        }
    }

    #[test]
    fn no_key_enters_manual_mode() {
        for c in TECLAS_MANUAL {
            let a = ruta_tecla(Tecla::Letra(c), false);
            assert!(!matches!(a, AccionTecla::Mover(_)), "{c}: {a:?}");
        }
        assert_eq!(ruta_tecla(Tecla::Letra('w'), false), AccionTecla::Nada);
    }

    #[test]
    fn escape_leaves_manual_mode() {
        assert_eq!(ruta_tecla(Tecla::Escape, true), AccionTecla::SalirManual);
        assert_eq!(ruta_tecla(Tecla::Escape, false), AccionTecla::Nada);
    }

    #[test]
    fn space_stops_in_every_mode() {
        assert_eq!(ruta_tecla(Tecla::Espacio, true), AccionTecla::Parada);
        assert_eq!(ruta_tecla(Tecla::Espacio, false), AccionTecla::Parada);
    }

    #[test]
    fn every_layer_has_its_own_key() {
        for l in Layer::ALL {
            assert_eq!(Layer::de_tecla(l.tecla()), Some(l));
        }
    }

    #[test]
    fn iced_keys_are_translated() {
        assert_eq!(tecla_de(&Key::Named(Named::Space)), Some(Tecla::Espacio));
        assert_eq!(tecla_de(&Key::Character("W".into())), Some(Tecla::Letra('w')));
        assert_eq!(tecla_de(&Key::Named(Named::ArrowUp)), None);
    }
}
