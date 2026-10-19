//! Pestaña **Por qué** (fase 2, §6): qué hace el robot elegido y por qué, con los
//! puntajes de las 5 opciones del atacante y los hechos de la decisión. Lee la
//! depuración del coach (`CoachDebug`) de la foto; las frases salen de funciones puras.

use super::capas::nombre_objetivo;
use super::format::{num, pct};
use super::{DebugGui, Message, fonts, theme};
use crate::coach::Role;
use crate::coach::debug::{CoachDebug, Trabada};
use crate::skills::SkillId;
use crate::snapshot::{CommandSource, OwnRobotView};
use iced::widget::{Column, progress_bar, row, text};
use iced::{Alignment, Element, Length};

/// La nota sin depuración del coach: con el coach en pausa y sin rutina, lo dice en
/// vez de que el coach no explica sus decisiones (`coach-pausa`).
pub fn nota_sin_depuracion(coach_en_pausa: bool) -> &'static str {
    if coach_en_pausa {
        "el coach está en pausa: los robots libres quedan quietos"
    } else {
        "Este coach no explica sus decisiones."
    }
}

/// Lo que hace el robot con su skill, para el título ("se está alineando").
fn haciendo(skill: SkillId) -> &'static str {
    match skill {
        SkillId::GoTo => "va a un punto",
        SkillId::FacePoint => "se está orientando",
        SkillId::ChaseBall => "persigue la pelota",
        SkillId::Spin => "está girando",
        SkillId::ApproachAligned => "se está alineando",
        SkillId::ShootPush => "está empujando la pelota",
        SkillId::Intercept => "sale a interceptar",
        SkillId::BlockLine => "bloquea la línea de tiro",
        SkillId::GoalKeep => "cuida el arco",
        SkillId::Clear => "está despejando",
        SkillId::SpinKick => "patea girando",
        SkillId::Mark => "está marcando",
        SkillId::Hold => "está esperando",
        SkillId::RailKeeper => "cuida el arco desde el riel",
    }
}

/// Título en frase: qué hace el robot `id`.
pub fn titulo(id: i32, vista: Option<&OwnRobotView>, visto: bool) -> String {
    let fuente = vista.and_then(|v| v.source);
    match (fuente, vista.and_then(|v| v.skill)) {
        (None, _) if !visto => format!("No vemos al robot {id}"),
        (None, _) => format!("El robot {id} no tiene comando"),
        (Some(CommandSource::Manual), _) => format!("El robot {id} está en control manual"),
        (Some(CommandSource::Halt), _) => format!("El robot {id} está detenido"),
        (Some(CommandSource::Stop), _) => format!("El robot {id} corrige la orientación"),
        (Some(_), Some(s)) => format!("El robot {id} {}", haciendo(s)),
        (Some(_), None) => format!("El robot {id} no tiene skill"),
    }
}

/// La causa en frase, con la depuración del coach. `None` si no se sabe.
pub fn causa(id: i32, vista: Option<&OwnRobotView>, coach: &CoachDebug) -> Option<String> {
    let vista = vista?;
    if vista.source != Some(CommandSource::Coach) {
        return None;
    }
    let skill = vista.skill?;
    if coach.atacante == Some(id) {
        let Some(op) = coach.opciones else {
            return Some(match skill {
                SkillId::BlockLine => "La pelota está en nuestra área: bloquea la línea de tiro mientras despeja el arquero.".to_string(),
                SkillId::Mark => "El arquero juega la pelota: el atacante se aparta.".to_string(),
                _ => "Jugada de pelota parada: el coach no elige entre las opciones.".to_string(),
            });
        };
        let objetivo = coach.objetivo.map(|(_, t)| nombre_objetivo(t));
        let mut frase = match op.aplicada {
            SkillId::ApproachAligned if !op.tiro.factible => {
                "Todavía no puede empujar: se pone detrás de la pelota.".to_string()
            }
            SkillId::ApproachAligned => "Se pone detrás de la pelota para empujar.".to_string(),
            SkillId::ShootPush => match objetivo {
                Some(o) => format!("Empuja la pelota hacia el objetivo: {o}."),
                None => "Empuja la pelota.".to_string(),
            },
            SkillId::Clear => "La pelota está en nuestra mitad: la despeja.".to_string(),
            SkillId::SpinKick => match coach.trabada {
                Trabada::Pateando(s) => format!(
                    "La pelota quedó trabada: la saca girando hace {} s.",
                    num(s, 1)
                ),
                _ => "Hay un rival encima de la pelota: la saca girando.".to_string(),
            },
            SkillId::Intercept => "La pelota viene hacia el robot: sale a cortarla.".to_string(),
            other => format!("Elige {other:?}."),
        };
        if op.compromiso.retuvo {
            frase.push_str(&format!(
                " Sigue con {:?} por el compromiso, aunque {:?} puntúa más.",
                op.aplicada, op.ganadora
            ));
        }
        return Some(frase);
    }
    Some(
        match (vista.role, skill) {
            (Some(Role::Keeper), SkillId::Clear) => "Sale a despejar la pelota.",
            (Some(Role::Keeper), _) => "Cuida el arco.",
            (_, SkillId::Mark) => "Marca a un rival o espera el rebote.",
            (_, SkillId::BlockLine) => "Cubre la línea de tiro hacia nuestro arco.",
            _ => "Acompaña la jugada.",
        }
        .to_string(),
    )
}

/// Hechos de la decisión, como pares etiqueta y valor. Omite lo que no aplica.
pub fn hechos(coach: &CoachDebug) -> Vec<(String, String)> {
    let mut out = Vec::new();
    let par = |a: &str, b: String| (a.to_string(), b);
    if let Some(op) = coach.opciones {
        out.push(par(
            "bono por seguir igual",
            match op.bono {
                Some((s, v)) => format!("+{} a {s:?}", num(v, 2)),
                None => "ninguno".to_string(),
            },
        ));
        let c = op.compromiso;
        out.push(par(
            "compromiso",
            if c.restantes > 0 {
                format!("{} de {} decisiones", c.minimo - c.restantes, c.minimo)
            } else {
                "cumplido".to_string()
            },
        ));
        out.push(par(
            "tiro",
            if op.tiro.factible {
                "factible"
            } else {
                "no factible"
            }
            .to_string(),
        ));
        out.push(par(
            "línea de tiro",
            if op.tiro.tapado {
                format!("tapada (×{})", num(op.tiro.factor, 1))
            } else {
                "libre".to_string()
            },
        ));
    }
    if let Some((_, tipo)) = coach.objetivo {
        out.push(par("objetivo", nombre_objetivo(tipo).to_string()));
    }
    out.push(par(
        "pelota trabada",
        match coach.trabada {
            Trabada::Libre => "no".to_string(),
            Trabada::Contando(s) => format!("empuja sin moverla hace {} s", num(s, 1)),
            Trabada::Pateando(s) => format!("la saca girando hace {} s", num(s, 1)),
            Trabada::GiroProhibido(s) => format!("giro prohibido hace {} s", num(s, 1)),
        },
    ));
    if let Some(c) = coach.cambio {
        let mut v = format!("otro llega en menos de {} s", num(c.umbral_s(), 2));
        if c.decisiones < c.minimo {
            v.push_str(&format!(", en {} decisiones", c.minimo - c.decisiones));
        }
        out.push(par("cambio de atacante si", v));
    }
    let a = coach.adaptacion;
    if a.activa {
        for (nombre, r) in [
            ("tiros por arriba", a.tiros[0]),
            ("tiros por el centro", a.tiros[1]),
            ("tiros por abajo", a.tiros[2]),
            ("duelos", a.duelos),
        ] {
            out.push(par(
                nombre,
                format!("{} de éxito ({} muestras)", pct(r.value), r.samples),
            ));
        }
    } else {
        out.push(par("adaptación", "apagada".to_string()));
    }
    out
}

impl DebugGui {
    pub(super) fn pestana_por_que(&self) -> Element<'_, Message> {
        let id = self.selected_robot as i32;
        let propio = self.selected_team as i32 == self.run.own_team;
        let vista = self.snap.own_robot(id).filter(|_| propio);
        let visto = self
            .snap
            .robot(self.selected_team as i32, id)
            .is_some_and(|r| r.active);
        let titulo_ = if propio {
            titulo(id, vista, visto)
        } else {
            format!("El robot {id} es rival")
        };
        let mut col = Column::new()
            .spacing(theme::SP_SM)
            .push(text(titulo_).font(fonts::TITULO).size(theme::TXT_TITULO));
        let coach = self.snap.debug.as_ref().and_then(|d| d.coach.as_ref());
        let Some(coach) = coach.filter(|_| propio) else {
            if propio {
                let pausa = self.coach_en_pausa() && self.snap.rutina.is_none();
                col = col.push(nota(nota_sin_depuracion(pausa)));
            }
            return col.into();
        };
        if let Some(c) = causa(id, vista, coach) {
            col = col.push(nota(c));
        }
        match coach.opciones {
            Some(op) => {
                let escala = op.puntajes.iter().map(|(_, v)| *v).fold(1.0_f32, f32::max);
                let atacante = coach
                    .atacante
                    .map_or(String::new(), |a| format!(" (robot {a})"));
                col = col.push(
                    text(format!("Opciones del atacante{atacante}"))
                        .size(theme::TXT_CHICO)
                        .style(theme::gris),
                );
                for (skill, v) in op.puntajes {
                    col = col.push(barra(skill, v, escala, skill == op.aplicada));
                }
            }
            None => {
                if let Some(s) = coach.skill_atacante {
                    col = col.push(nota(format!(
                        "El atacante no elige entre las 5 opciones: corre {s:?}."
                    )));
                }
            }
        }
        let mut lista = Column::new().spacing(theme::SP_XS);
        for (etiqueta, valor) in hechos(coach) {
            lista = lista.push(
                row![
                    text(etiqueta)
                        .size(theme::TXT_CHICO)
                        .style(theme::gris)
                        .width(Length::Fill),
                    text(valor).size(theme::TXT_CHICO),
                ]
                .spacing(theme::SP_MD),
            );
        }
        col.push(iced::widget::horizontal_rule(1).style(theme::filete))
            .push(lista)
            .push(nota(format!("Decisión {} del coach.", coach.decision)))
            .into()
    }
}

fn nota(t: impl ToString) -> Element<'static, Message> {
    text(t.to_string())
        .size(theme::TXT_CHICO)
        .style(theme::gris)
        .into()
}

/// Una barra de puntaje: nombre, pista y valor; la aplicada en azul y negrita.
fn barra(skill: SkillId, valor: f32, escala: f32, aplicada: bool) -> Element<'static, Message> {
    let fuente = if aplicada {
        fonts::TEXTO_FUERTE
    } else {
        fonts::TEXTO
    };
    let v = if valor.is_finite() {
        valor.max(0.0)
    } else {
        0.0
    };
    row![
        text(format!("{skill:?}"))
            .font(fuente)
            .size(theme::TXT_CHICO)
            .width(Length::Fixed(118.0)),
        progress_bar(0.0..=escala, v)
            .height(Length::Fixed(8.0))
            .style(move |_t: &iced::Theme| progress_bar::Style {
                background: theme::PISTA.into(),
                bar: if aplicada {
                    theme::AZUL
                } else {
                    theme::alfa(theme::GRIS, 0.7)
                }
                .into(),
                border: iced::Border::default(),
            }),
        text(num(valor, 2))
            .font(fuente)
            .size(theme::TXT_CHICO)
            .width(Length::Fixed(38.0)),
    ]
    .spacing(theme::SP_SM)
    .align_y(Alignment::Center)
    .into()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::coach::Play;
    use crate::coach::adaptation::{Lane, Rate};
    use crate::coach::debug::{
        AdaptacionDebug, CambioAtacante, Compromiso, OpcionesAtacante, TipoObjetivo, Tiro,
    };

    fn vista(skill: SkillId, role: Role) -> OwnRobotView {
        OwnRobotView {
            id: 0,
            source: Some(CommandSource::Coach),
            role: Some(role),
            skill: Some(skill),
            status: None,
            target: None,
            command: None,
            escaping: false,
        }
    }

    fn coach(aplicada: SkillId, factible: bool) -> CoachDebug {
        let r = Rate {
            value: 0.62,
            samples: 5,
        };
        CoachDebug {
            decision: 7,
            jugada: Play::Open,
            atacante: Some(0),
            arquero: 2,
            opciones: Some(OpcionesAtacante {
                puntajes: [
                    (SkillId::ShootPush, 0.0),
                    (SkillId::Clear, 0.0),
                    (SkillId::SpinKick, 0.0),
                    (SkillId::Intercept, 0.0),
                    (SkillId::ApproachAligned, 0.27),
                ],
                ganadora: aplicada,
                aplicada,
                bono: Some((aplicada, 0.15)),
                compromiso: Compromiso {
                    skill: aplicada,
                    restantes: 1,
                    minimo: 3,
                    retuvo: false,
                },
                tiro: Tiro {
                    factible,
                    tapado: true,
                    factor: 0.6,
                    carril: Lane::Center,
                },
            }),
            skill_atacante: Some(aplicada),
            objetivo: Some((glam::Vec2::new(0.75, -0.14), TipoObjetivo::PaloLejano)),
            roles: Vec::new(),
            cambio: Some(CambioAtacante {
                actual: 0,
                t_actual: 0.3,
                mejor: 1,
                t_mejor: 0.5,
                ganancia: 0.75,
                decisiones: 6,
                minimo: 5,
            }),
            trabada: Trabada::Libre,
            adaptacion: AdaptacionDebug {
                activa: true,
                tiros: [r; 3],
                duelos: r,
            },
        }
    }

    #[test]
    fn paused_coach_note_replaces_the_no_debug_note() {
        assert_eq!(nota_sin_depuracion(true), "el coach está en pausa: los robots libres quedan quietos");
        assert!(!nota_sin_depuracion(true).contains("no explica"));
        assert_eq!(nota_sin_depuracion(false), "Este coach no explica sus decisiones.");
    }

    #[test]
    fn title_says_what_the_robot_does() {
        let v = vista(SkillId::ApproachAligned, Role::Striker);
        assert_eq!(titulo(0, Some(&v), true), "El robot 0 se está alineando");
        assert_eq!(titulo(2, None, false), "No vemos al robot 2");
    }

    #[test]
    fn cause_explains_an_unfeasible_shot() {
        let v = vista(SkillId::ApproachAligned, Role::Striker);
        let c = causa(0, Some(&v), &coach(SkillId::ApproachAligned, false)).unwrap();
        assert_eq!(c, "Todavía no puede empujar: se pone detrás de la pelota.");
    }

    #[test]
    fn cause_of_a_stalled_ball() {
        let v = vista(SkillId::SpinKick, Role::Striker);
        let mut d = coach(SkillId::SpinKick, true);
        d.trabada = Trabada::Pateando(1.6);
        assert_eq!(
            causa(0, Some(&v), &d).unwrap(),
            "La pelota quedó trabada: la saca girando hace 1.6 s."
        );
    }

    #[test]
    fn facts_carry_bonus_commitment_line_target_and_switch() {
        let h = hechos(&coach(SkillId::ApproachAligned, false));
        let valor = |k: &str| {
            h.iter()
                .find(|(a, _)| a == k)
                .map(|(_, b)| b.clone())
                .unwrap()
        };
        assert_eq!(valor("bono por seguir igual"), "+0.15 a ApproachAligned");
        assert_eq!(valor("compromiso"), "2 de 3 decisiones");
        assert_eq!(valor("línea de tiro"), "tapada (×0.6)");
        assert_eq!(valor("objetivo"), "palo lejano");
        assert_eq!(
            valor("cambio de atacante si"),
            "otro llega en menos de 0.23 s"
        );
        assert_eq!(valor("tiros por arriba"), "62 % de éxito (5 muestras)");
    }

    #[test]
    fn blockline_explains_why_there_are_no_options() {
        let v = vista(SkillId::BlockLine, Role::Striker);
        let mut d = coach(SkillId::BlockLine, false);
        d.opciones = None;
        let c = causa(0, Some(&v), &d).unwrap();
        assert!(c.starts_with("La pelota está en nuestra área"), "{c}");
    }
}
