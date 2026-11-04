//! Pestaña **Árbitro** (`VSSL_ARBITRO_GUI=1`, apagada por defecto): botones para practicar las
//! pelotas paradas sin `tools/referee_cli.py`.
//!
//! Cada botón arma el mismo texto que `referee_cli` (`PENALTY YELLOW`, `FREE_BALL Q3`…), lo
//! lee con el parser del listener (`parse_text_command`) y lo deja en el estado del árbitro de
//! ESTE engine (`apply_command`), igual que si hubiera llegado por la red. No abre ningún
//! socket: nada sale del proceso, ni al grupo multicast ni a 127.0.0.1, así que en la
//! competencia no le llega al árbitro oficial, al rival ni a otro engine del mismo notebook.
//! Para dos engines a la vez (dos equipos en la salita) sigue `referee_cli.py`.

use super::rutinas::{boton_chico, etiqueta, nota, titulo};
use super::{DebugGui, Message, theme};
use crate::coach::referee::{apply_command, parse_text_command};
use iced::widget::{Column, row, text};
use iced::{Alignment, Element};

/// Variable que enciende la pestaña.
pub const ARBITRO_GUI_ENV: &str = "VSSL_ARBITRO_GUI";

/// Valor de `VSSL_ARBITRO_GUI`: sin definir, vacío, `0`, `false` u `off` → apagada; `1`,
/// `true` u `on` → encendida. Otro valor no arranca (un typo no debe encenderla ni
/// apagarla en silencio).
pub fn arbitro_gui_de(valor: Option<&str>) -> Result<bool, String> {
    match valor.map(|v| v.trim().to_ascii_lowercase()).as_deref() {
        None | Some("") | Some("0") | Some("false") | Some("off") => Ok(false),
        Some("1") | Some("true") | Some("on") => Ok(true),
        Some(otro) => Err(format!("{ARBITRO_GUI_ENV}='{otro}' no vale: 1 (pestaña Árbitro) o 0 (sin ella)")),
    }
}

pub fn arbitro_gui_from_env() -> Result<bool, String> {
    arbitro_gui_de(std::env::var(ARBITRO_GUI_ENV).ok().as_deref())
}

/// Filas de la pestaña: rótulo y botones (texto del botón, comando). Los comandos con equipo
/// van en pares azul / amarillo.
pub const FILAS: [(&str, &[(&str, &str)]); 6] = [
    ("Juego", &[("STOP", "STOP"), ("GAME ON", "GAME_ON"), ("HALT", "HALT")]),
    ("Saque inicial", &[("azul", "KICKOFF BLUE"), ("amarillo", "KICKOFF YELLOW")]),
    ("Penal", &[("azul", "PENALTY BLUE"), ("amarillo", "PENALTY YELLOW")]),
    ("Tiro libre", &[("azul", "FREE_KICK BLUE"), ("amarillo", "FREE_KICK YELLOW")]),
    ("Tiro de arco", &[("azul", "GOAL_KICK BLUE"), ("amarillo", "GOAL_KICK YELLOW")]),
    (
        "Free ball",
        &[("Q1", "FREE_BALL Q1"), ("Q2", "FREE_BALL Q2"), ("Q3", "FREE_BALL Q3"), ("Q4", "FREE_BALL Q4")],
    ),
];

impl DebugGui {
    /// Un botón de la pestaña: aplica `comando` al árbitro de este engine. Sin la pestaña
    /// encendida no hace nada.
    pub(super) fn mandar_arbitro(&mut self, comando: &'static str) {
        let Some(shared) = &self.arbitro else { return };
        match parse_text_command(comando) {
            Some(cmd) => {
                apply_command(shared, cmd);
                eprintln!("[GUI] árbitro de práctica: {comando}");
                self.arbitro_ultimo = Some(comando);
            }
            None => eprintln!("[GUI] árbitro de práctica: '{comando}' no se entiende"),
        }
    }

    pub(super) fn pestana_arbitro(&self) -> Element<'_, Message> {
        let mut col = Column::new().spacing(theme::SP_SM).push(titulo("Árbitro de práctica"));
        if self.arbitro.is_none() {
            return col
                .push(nota(format!("Apagada. Se enciende con {ARBITRO_GUI_ENV}=1, solo para practicar.")))
                .into();
        }
        col = col.push(nota(
            "Los comandos quedan en este engine: no salen a la red, así que no le llegan al árbitro \
             oficial, al rival ni a otro engine de este notebook. Con dos equipos a la vez, usar \
             tools/referee_cli.py. Igual que el VSSReferee: después de una pelota parada, GAME ON.",
        ));
        let nuestro = if self.run.own_team == 0 { "azul" } else { "amarillo" };
        for (rotulo, botones) in FILAS {
            let mut fila = row![etiqueta(rotulo)].spacing(theme::SP_XS).align_y(Alignment::Center);
            for (texto, comando) in botones.iter().copied() {
                let texto = if texto == nuestro { format!("{texto} (nosotros)") } else { texto.to_string() };
                fila = fila.push(boton_chico(&texto, Some(Message::Arbitro(comando))));
            }
            col = col.push(fila.wrap());
        }
        col = col.push(nota(
            "Cuadrantes de la cancha, no de nuestro lado: Q1 = +x +y, Q2 = −x +y, Q3 = −x −y, \
             Q4 = +x −y.",
        ));
        let ultimo = match self.arbitro_ultimo {
            Some(c) => format!("Último desde aquí: {c}"),
            None => "Todavía no se mandó nada desde aquí.".to_string(),
        };
        col.push(text(ultimo).size(theme::TXT)).into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::coach::referee::{Foul, Quadrant, RefTeam};

    #[test]
    fn apagada_por_defecto_y_un_valor_raro_no_arranca() {
        for v in [None, Some(""), Some("0"), Some("false"), Some(" OFF ")] {
            assert_eq!(arbitro_gui_de(v), Ok(false), "{v:?}");
        }
        for v in ["1", "true", "On"] {
            assert_eq!(arbitro_gui_de(Some(v)), Ok(true), "{v}");
        }
        assert!(arbitro_gui_de(Some("si")).unwrap_err().contains(ARBITRO_GUI_ENV));
    }

    #[test]
    fn cada_boton_es_un_comando_que_el_listener_entiende() {
        let mut n = 0;
        for (_, botones) in FILAS {
            for (_, comando) in botones.iter() {
                assert!(parse_text_command(comando).is_some(), "{comando}");
                n += 1;
            }
        }
        assert_eq!(n, 15);
        let c = parse_text_command("PENALTY YELLOW").unwrap();
        assert_eq!((c.foul, c.team), (Foul::PenaltyKick, RefTeam::Yellow));
        assert_eq!(parse_text_command("FREE_BALL Q3").unwrap().quadrant, Quadrant::Q3);
    }
}
