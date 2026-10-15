//! Teclado de la GUI.
//!
//! El control manual por teclado está activo solo dentro del modo manual, al que se
//! entra con un botón. Dentro de ese modo manejan dos jugadores: el 1 con W, A, S, D y
//! E, y el 2 con las flechas y Enter. Fuera, las letras son de las capas de la cancha.
//! Espacio detiene todo siempre y P alterna el coach. Las teclas apretadas que captura
//! un widget (un campo de texto con el foco, un slider bajo el cursor) no llegan al
//! ruteo, salvo Espacio y Esc; las soltadas llegan siempre (`evento_de`).

use iced::keyboard::{self, Key, key::Named};

/// Flecha del teclado.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Flecha {
    Arriba,
    Abajo,
    Izquierda,
    Derecha,
}

/// Tecla que le interesa a la GUI.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Tecla {
    Espacio,
    Escape,
    /// Enter, también el del teclado numérico.
    Enter,
    Flecha(Flecha),
    /// Letra, en minúscula.
    Letra(char),
}

/// Traduce una tecla de iced. Otras teclas (F1, Tab, …) no interesan.
pub fn tecla_de(key: &Key) -> Option<Tecla> {
    match key {
        Key::Named(Named::Space) => Some(Tecla::Espacio),
        Key::Named(Named::Escape) => Some(Tecla::Escape),
        Key::Named(Named::Enter) => Some(Tecla::Enter),
        Key::Named(Named::ArrowUp) => Some(Tecla::Flecha(Flecha::Arriba)),
        Key::Named(Named::ArrowDown) => Some(Tecla::Flecha(Flecha::Abajo)),
        Key::Named(Named::ArrowLeft) => Some(Tecla::Flecha(Flecha::Izquierda)),
        Key::Named(Named::ArrowRight) => Some(Tecla::Flecha(Flecha::Derecha)),
        Key::Character(s) => s.chars().next().map(|c| Tecla::Letra(c.to_ascii_lowercase())),
        _ => None,
    }
}

/// Lo que la GUI toma de un evento de iced.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EventoTeclado {
    Apretada(Tecla),
    Soltada(Tecla),
    /// Cambió el estado de Shift (para Shift y click en una fila).
    Shift(bool),
    /// La ventana perdió el foco: se suelta todo.
    FocoPerdido,
}

/// Traduce un evento de iced, sabiendo si un widget lo capturó. Una tecla apretada que
/// capturó un widget no cuenta, salvo Espacio (la parada) y Esc (salir del modo
/// manual); una soltada cuenta siempre, para que nunca quede apretada en la GUI.
pub fn evento_de(evento: &iced::Event, capturado: bool) -> Option<EventoTeclado> {
    match evento {
        iced::Event::Keyboard(keyboard::Event::KeyPressed { key, .. }) => {
            let t = tecla_de(key)?;
            (!capturado || matches!(t, Tecla::Espacio | Tecla::Escape)).then_some(EventoTeclado::Apretada(t))
        }
        iced::Event::Keyboard(keyboard::Event::KeyReleased { key, .. }) => {
            tecla_de(key).map(EventoTeclado::Soltada)
        }
        iced::Event::Keyboard(keyboard::Event::ModifiersChanged(m)) => Some(EventoTeclado::Shift(m.shift())),
        iced::Event::Window(iced::window::Event::Unfocused) => Some(EventoTeclado::FocoPerdido),
        _ => None,
    }
}

/// Capas de la cancha, con su tecla.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Layer {
    Navegacion,
    Coach,
    Reflejos,
    Trazas,
    VisionCruda,
    Skills,
    /// Recorrido de la rutina seleccionada o activa (capability `gui-rutinas`).
    Rutina,
}

impl Layer {
    pub const ALL: [Layer; 7] = [
        Layer::Navegacion,
        Layer::Coach,
        Layer::Reflejos,
        Layer::Trazas,
        Layer::VisionCruda,
        Layer::Skills,
        Layer::Rutina,
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
            Layer::Rutina => 'u',
        }
    }

    pub fn de_tecla(c: char) -> Option<Layer> {
        Layer::ALL.into_iter().find(|l| l.tecla() == c)
    }
}

/// Uno de los dos jugadores del modo manual.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Jugador {
    Uno,
    Dos,
}

impl Jugador {
    pub const AMBOS: [Jugador; 2] = [Jugador::Uno, Jugador::Dos];

    pub fn indice(self) -> usize {
        match self {
            Jugador::Uno => 0,
            Jugador::Dos => 1,
        }
    }

    /// Sus teclas, en el marco robot (arcade).
    pub fn teclas(self) -> TeclasJugador {
        match self {
            Jugador::Uno => TeclasJugador {
                adelante: Tecla::Letra('w'),
                atras: Tecla::Letra('s'),
                giro_ccw: Tecla::Letra('a'),
                giro_cw: Tecla::Letra('d'),
                giro_corto: Tecla::Letra('e'),
            },
            Jugador::Dos => TeclasJugador {
                adelante: Tecla::Flecha(Flecha::Arriba),
                atras: Tecla::Flecha(Flecha::Abajo),
                giro_ccw: Tecla::Flecha(Flecha::Izquierda),
                giro_cw: Tecla::Flecha(Flecha::Derecha),
                giro_corto: Tecla::Enter,
            },
        }
    }
}

/// Las teclas de un jugador.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TeclasJugador {
    pub adelante: Tecla,
    pub atras: Tecla,
    pub giro_ccw: Tecla,
    pub giro_cw: Tecla,
    pub giro_corto: Tecla,
}

impl TeclasJugador {
    fn mueve(&self, t: Tecla) -> bool {
        [self.adelante, self.atras, self.giro_ccw, self.giro_cw].contains(&t)
    }
}

/// La tecla del interruptor del coach.
pub const TECLA_COACH: Tecla = Tecla::Letra('p');

/// Qué pide una tecla apretada. Quien recibe el pedido lo ignora si el jugador no
/// tiene robot o si no hay coach que pausar.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AccionTecla {
    /// Activa la parada de emergencia.
    Parada,
    /// Sale del modo manual.
    SalirManual,
    /// Alterna el coach entre activo y en pausa (por flanco).
    AlternarCoach,
    /// Tecla de movimiento de un jugador.
    Mover { jugador: Jugador, tecla: Tecla },
    /// Giro corto de un jugador (por flanco).
    Giro(Jugador),
    /// Prende o apaga una capa.
    Capa(Layer),
    Nada,
}

/// Efecto de una tecla apretada según el modo. Dentro del modo manual, ninguna letra
/// va a las capas; fuera de él, ninguna mueve un robot, y ninguna tecla entra al modo
/// manual. Espacio y P valen en los dos modos.
pub fn ruta_tecla(tecla: Tecla, modo_manual: bool) -> AccionTecla {
    if tecla == Tecla::Espacio {
        return AccionTecla::Parada;
    }
    if tecla == TECLA_COACH {
        return AccionTecla::AlternarCoach;
    }
    if !modo_manual {
        return match tecla {
            Tecla::Letra(c) => Layer::de_tecla(c).map_or(AccionTecla::Nada, AccionTecla::Capa),
            _ => AccionTecla::Nada,
        };
    }
    if tecla == Tecla::Escape {
        return AccionTecla::SalirManual;
    }
    for jugador in Jugador::AMBOS {
        let t = jugador.teclas();
        if tecla == t.giro_corto {
            return AccionTecla::Giro(jugador);
        }
        if t.mueve(tecla) {
            return AccionTecla::Mover { jugador, tecla };
        }
    }
    AccionTecla::Nada
}

#[cfg(test)]
mod tests {
    use super::*;
    use iced::keyboard::{Location, Modifiers};

    const W: Tecla = Tecla::Letra('w');
    const ARRIBA: Tecla = Tecla::Flecha(Flecha::Arriba);

    #[test]
    fn outside_manual_s_belongs_to_the_layers() {
        assert_eq!(ruta_tecla(Tecla::Letra('s'), false), AccionTecla::Capa(Layer::Skills));
    }

    #[test]
    fn inside_manual_s_moves_player_one_back() {
        assert_eq!(
            ruta_tecla(Tecla::Letra('s'), true),
            AccionTecla::Mover { jugador: Jugador::Uno, tecla: Tecla::Letra('s') }
        );
    }

    #[test]
    fn layer_letters_and_q_inside_manual_do_nothing() {
        for c in ['n', 'c', 'r', 't', 'v', 'u', 'q'] {
            assert_eq!(ruta_tecla(Tecla::Letra(c), true), AccionTecla::Nada, "{c}");
        }
    }

    #[test]
    fn no_key_enters_manual_mode() {
        for jugador in Jugador::AMBOS {
            let t = jugador.teclas();
            for tecla in [t.adelante, t.atras, t.giro_ccw, t.giro_cw, t.giro_corto] {
                let a = ruta_tecla(tecla, false);
                assert!(
                    !matches!(a, AccionTecla::Mover { .. } | AccionTecla::Giro(_)),
                    "{tecla:?}: {a:?}"
                );
            }
        }
        assert_eq!(ruta_tecla(W, false), AccionTecla::Nada);
    }

    #[test]
    fn arrows_and_enter_outside_manual_do_nothing() {
        for f in [Flecha::Arriba, Flecha::Abajo, Flecha::Izquierda, Flecha::Derecha] {
            assert_eq!(ruta_tecla(Tecla::Flecha(f), false), AccionTecla::Nada, "{f:?}");
        }
        assert_eq!(ruta_tecla(Tecla::Enter, false), AccionTecla::Nada);
    }

    #[test]
    fn each_player_has_its_own_keys_inside_manual() {
        for (c, jugador) in [('w', Jugador::Uno), ('a', Jugador::Uno), ('d', Jugador::Uno)] {
            assert_eq!(
                ruta_tecla(Tecla::Letra(c), true),
                AccionTecla::Mover { jugador, tecla: Tecla::Letra(c) }
            );
        }
        for f in [Flecha::Arriba, Flecha::Abajo, Flecha::Izquierda, Flecha::Derecha] {
            assert_eq!(
                ruta_tecla(Tecla::Flecha(f), true),
                AccionTecla::Mover { jugador: Jugador::Dos, tecla: Tecla::Flecha(f) }
            );
        }
        assert_eq!(ruta_tecla(Tecla::Letra('e'), true), AccionTecla::Giro(Jugador::Uno));
        assert_eq!(ruta_tecla(Tecla::Enter, true), AccionTecla::Giro(Jugador::Dos));
    }

    #[test]
    fn p_toggles_the_coach_in_both_modes() {
        assert_eq!(ruta_tecla(Tecla::Letra('p'), false), AccionTecla::AlternarCoach);
        assert_eq!(ruta_tecla(Tecla::Letra('p'), true), AccionTecla::AlternarCoach);
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
    fn u_belongs_to_the_routine_layer_outside_manual_only() {
        assert_eq!(ruta_tecla(Tecla::Letra('u'), false), AccionTecla::Capa(Layer::Rutina));
        assert_eq!(ruta_tecla(Tecla::Letra('u'), true), AccionTecla::Nada);
    }

    #[test]
    fn every_layer_has_its_own_key() {
        for l in Layer::ALL {
            assert_eq!(Layer::de_tecla(l.tecla()), Some(l));
            assert_ne!(Tecla::Letra(l.tecla()), TECLA_COACH, "{l:?}");
        }
    }

    #[test]
    fn iced_keys_are_translated() {
        assert_eq!(tecla_de(&Key::Named(Named::Space)), Some(Tecla::Espacio));
        assert_eq!(tecla_de(&Key::Character("W".into())), Some(W));
        assert_eq!(tecla_de(&Key::Named(Named::ArrowUp)), Some(ARRIBA));
        assert_eq!(tecla_de(&Key::Named(Named::ArrowRight)), Some(Tecla::Flecha(Flecha::Derecha)));
        assert_eq!(tecla_de(&Key::Named(Named::Enter)), Some(Tecla::Enter));
        assert_eq!(tecla_de(&Key::Named(Named::Tab)), None);
    }

    fn apretada(key: Key, location: Location) -> iced::Event {
        iced::Event::Keyboard(keyboard::Event::KeyPressed {
            key: key.clone(),
            modified_key: key,
            physical_key: keyboard::key::Physical::Unidentified(
                keyboard::key::NativeCode::Unidentified,
            ),
            location,
            modifiers: Modifiers::empty(),
            text: None,
        })
    }

    fn soltada(key: Key) -> iced::Event {
        iced::Event::Keyboard(keyboard::Event::KeyReleased {
            key,
            location: Location::Standard,
            modifiers: Modifiers::empty(),
        })
    }

    #[test]
    fn captured_presses_do_not_count_except_space_and_escape() {
        let w = apretada(Key::Character("w".into()), Location::Standard);
        assert_eq!(evento_de(&w, false), Some(EventoTeclado::Apretada(W)));
        assert_eq!(evento_de(&w, true), None, "un campo de texto se la queda");
        let flecha = apretada(Key::Named(Named::ArrowLeft), Location::Standard);
        assert_eq!(evento_de(&flecha, true), None);
        let p = apretada(Key::Character("p".into()), Location::Standard);
        assert_eq!(evento_de(&p, true), None, "P no actúa mientras se escribe");
        for (key, t) in [(Named::Space, Tecla::Espacio), (Named::Escape, Tecla::Escape)] {
            let e = apretada(Key::Named(key), Location::Standard);
            assert_eq!(evento_de(&e, true), Some(EventoTeclado::Apretada(t)), "{key:?}");
        }
    }

    #[test]
    fn numpad_enter_is_enter() {
        let e = apretada(Key::Named(Named::Enter), Location::Numpad);
        assert_eq!(evento_de(&e, false), Some(EventoTeclado::Apretada(Tecla::Enter)));
    }

    #[test]
    fn releases_always_count() {
        for capturado in [false, true] {
            assert_eq!(
                evento_de(&soltada(Key::Character("w".into())), capturado),
                Some(EventoTeclado::Soltada(W))
            );
            assert_eq!(
                evento_de(&soltada(Key::Named(Named::ArrowUp)), capturado),
                Some(EventoTeclado::Soltada(ARRIBA))
            );
        }
    }

    #[test]
    fn shift_and_focus_loss() {
        let shift = iced::Event::Keyboard(keyboard::Event::ModifiersChanged(Modifiers::SHIFT));
        assert_eq!(evento_de(&shift, false), Some(EventoTeclado::Shift(true)));
        let nada = iced::Event::Keyboard(keyboard::Event::ModifiersChanged(Modifiers::empty()));
        assert_eq!(evento_de(&nada, true), Some(EventoTeclado::Shift(false)));
        let foco = iced::Event::Window(iced::window::Event::Unfocused);
        assert_eq!(evento_de(&foco, false), Some(EventoTeclado::FocoPerdido));
        assert_eq!(evento_de(&iced::Event::Window(iced::window::Event::Focused), false), None);
    }
}
