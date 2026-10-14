//! Capas sobre la cancha (fase 2 de la especificación, §5): Navegación (N), Coach (C),
//! Reflejos (R), Trazas (T) y Skills (S). Leen la foto del tick (la depuración
//! `LoopSnapshot::debug` y las vistas de los robots) y no deciden nada. La V (visión
//! cruda) llega en la fase 4.
//!
//! Debajo de los robots van T, R y N; encima, C y S (los textos). Un valor no finito
//! omite el elemento en ese cuadro, como en la cancha.

use super::field::{Vista, ancho_texto, escribir, polilinea, rect_mundo};
use super::keys::Layer;
use super::{Message, fonts, theme};
use crate::coach::debug::TipoObjetivo;
use crate::coach::{FIELD_HALF_X, Role};
use crate::motion::avoid::Rodeado;
use crate::skills::zones::{
    ARC_CENTER_X, ARC_RADIUS, AREA_HALF_Y, AREA_X, AccionGuardia, AreaRect,
};
use crate::skills::{Face, SkillId, SkillStatus};
use crate::snapshot::{CommandSource, LoopSnapshot, OwnRobotView};
use glam::Vec2;
use iced::widget::canvas::{self, Frame, LineDash, Path, Stroke};
use iced::widget::{button, row, text};
use iced::{Color, Element, Point};
use std::collections::{HashMap, HashSet, VecDeque};

/// Capas de esta fase, en el orden de la barra.
pub const CAPAS: [Layer; 6] = [
    Layer::Navegacion,
    Layer::Coach,
    Layer::Reflejos,
    Layer::Trazas,
    Layer::Skills,
    Layer::Rutina,
];
/// Encendidas al abrir la GUI.
pub const CAPAS_INICIALES: [Layer; 5] = [
    Layer::Navegacion,
    Layer::Coach,
    Layer::Reflejos,
    Layer::Trazas,
    Layer::Rutina,
];
/// Segundos de trazas.
pub const TRAZA_S: f32 = 3.0;
/// Largo de la flecha del escape (m).
const FLECHA_ESCAPE_M: f32 = 0.12;
const PUNTEADO: &[f32] = &[5.0, 4.0];
const PUNTEADO_FINO: &[f32] = &[2.0, 5.0];

pub fn nombre_capa(capa: Layer) -> &'static str {
    match capa {
        Layer::Navegacion => "Navegación",
        Layer::Coach => "Coach",
        Layer::Reflejos => "Reflejos",
        Layer::Trazas => "Trazas",
        Layer::VisionCruda => "Visión cruda",
        Layer::Skills => "Skills",
        Layer::Rutina => "Rutina",
    }
}

/// Barra de botones de las capas, con su tecla.
pub fn barra_capas(activas: &HashSet<Layer>) -> Element<'static, Message> {
    let mut fila = row![].spacing(theme::SP_XS);
    for capa in CAPAS {
        let on = activas.contains(&capa);
        let etiqueta = row![
            text(nombre_capa(capa)).size(theme::TXT_CHICO),
            text(capa.tecla().to_ascii_uppercase().to_string())
                .size(theme::TXT_CHICO)
                .style(theme::gris),
        ]
        .spacing(theme::SP_XS);
        fila = fila.push(
            button(etiqueta)
                .padding([2, 8])
                .style(theme::boton_capa(on))
                .on_press(Message::ToggleCapa(capa)),
        );
    }
    fila.into()
}

// ── Trazas (en la GUI, sin cambio de contrato) ──────────────────────────────

/// Qué deja traza.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Objeto {
    Robot(i32, i32),
    Pelota,
}

/// Los últimos `TRAZA_S` de cada robot (de los dos equipos) y de la pelota, con las
/// posiciones de las fotos que recibe la GUI.
#[derive(Debug, Default)]
pub struct Trazas {
    puntos: HashMap<Objeto, VecDeque<(u64, Vec2)>>,
    ultimo_ms: u64,
}

impl Trazas {
    pub fn registrar(&mut self, snap: &LoopSnapshot) {
        let t = snap.t_ms;
        if t < self.ultimo_ms {
            // El lazo volvió a arrancar: las trazas viejas no son de este partido.
            self.puntos.clear();
        }
        self.ultimo_ms = t;
        let mut anotar = |o: Objeto, p: Vec2| {
            if p.is_finite() {
                self.puntos.entry(o).or_default().push_back((t, p));
            }
        };
        for r in snap.robots.iter().filter(|r| r.active) {
            anotar(Objeto::Robot(r.team, r.id), r.position);
        }
        if let Some(b) = &snap.ball {
            anotar(Objeto::Pelota, b.position);
        }
        let desde = t.saturating_sub((TRAZA_S * 1000.0) as u64);
        self.puntos.retain(|_, q| {
            while q.front().is_some_and(|(ti, _)| *ti < desde) {
                q.pop_front();
            }
            !q.is_empty()
        });
    }

    pub fn de(&self, o: Objeto) -> impl Iterator<Item = Vec2> + '_ {
        self.puntos
            .get(&o)
            .into_iter()
            .flat_map(|q| q.iter().map(|(_, p)| *p))
    }

    pub fn objetos(&self) -> impl Iterator<Item = Objeto> + '_ {
        self.puntos.keys().copied()
    }
}

// ── Frases (funciones puras) ────────────────────────────────────────────────

/// Lo que hace un robot con su skill, en frase corta ("se alinea", "empuja"…).
pub fn accion(skill: SkillId) -> &'static str {
    match skill {
        SkillId::GoTo => "va al punto",
        SkillId::FacePoint => "se orienta",
        SkillId::ChaseBall => "persigue la pelota",
        SkillId::Spin => "gira",
        SkillId::ApproachAligned => "se alinea",
        SkillId::ShootPush => "empuja",
        SkillId::Intercept => "intercepta",
        SkillId::BlockLine => "bloquea",
        SkillId::GoalKeep => "ataja",
        SkillId::Clear => "despeja",
        SkillId::SpinKick => "patea girando",
        SkillId::Mark => "marca",
        SkillId::Hold => "espera",
    }
}

pub fn nombre_rol(rol: Role) -> &'static str {
    match rol {
        Role::Keeper => "Arquero",
        Role::Striker => "Atacante",
        Role::Support => "Apoyo",
    }
}

fn mayuscula(s: &str) -> String {
    let mut c = s.chars();
    c.next().map_or_else(String::new, |p| {
        p.to_uppercase().collect::<String>() + c.as_str()
    })
}

/// Rótulo de la capa Coach: rol y lo que hace ("Atacante, se alinea"). `None` si el
/// robot no tiene comando.
pub fn rotulo_coach(v: &OwnRobotView) -> Option<String> {
    let fuente = v.source?;
    let hace = v.skill.map(accion);
    Some(match fuente {
        CommandSource::Manual => "Control manual".to_string(),
        CommandSource::Halt => "Detenido".to_string(),
        CommandSource::Stop => match v.role {
            Some(r) => format!("{}, corrige la orientación", nombre_rol(r)),
            None => "Corrige la orientación".to_string(),
        },
        CommandSource::GuiSkill => match hace {
            Some(h) => format!("Skill de la GUI, {h}"),
            None => "Skill de la GUI".to_string(),
        },
        CommandSource::Coach => match (v.role, hace) {
            (Some(Role::Keeper), Some("ataja") | None) => "Arquero".to_string(),
            (Some(r), Some(h)) => format!("{}, {h}", nombre_rol(r)),
            (Some(r), None) => nombre_rol(r).to_string(),
            (None, Some(h)) => mayuscula(h),
            (None, None) => return None,
        },
    })
}

/// Línea de la capa Skills: progreso y cara, y si no es factible.
pub fn linea_skill(st: SkillStatus) -> (String, bool) {
    let cara = if st.face == Face::Front {
        "de frente"
    } else {
        "de espaldas"
    };
    let linea = if st.progress.is_finite() {
        format!(
            "{} %, {cara}",
            super::format::num((st.progress.clamp(0.0, 1.0) * 100.0).round(), 0)
        )
    } else {
        cara.to_string()
    };
    (linea, !st.feasible)
}

/// El lado del rodeo, visto desde el robot que va de `robot` a `destino` por `mira`.
pub fn lado_rodeo(robot: Vec2, destino: Vec2, mira: Vec2, obstaculo: Rodeado) -> String {
    let que = match obstaculo {
        Rodeado::Pelota => "la pelota",
        Rodeado::AreaIzquierda | Rodeado::AreaDerecha => "el área",
    };
    let cruz = (destino - robot).perp_dot(mira - robot);
    let lado = if cruz >= 0.0 {
        "por la izquierda"
    } else {
        "por la derecha"
    };
    format!("rodea {que} {lado}")
}

/// El tipo de objetivo del atacante, en palabras.
pub fn nombre_objetivo(tipo: TipoObjetivo) -> &'static str {
    match tipo {
        TipoObjetivo::PaloLejano => "palo lejano",
        TipoObjetivo::Arco => "arco rival",
        TipoObjetivo::FondoRival => "frente del arco rival",
        TipoObjetivo::FondoPropio => "despeje por la banda",
        TipoObjetivo::Banda => "a lo largo de la banda",
        TipoObjetivo::Despeje => "despeje",
        TipoObjetivo::Pase => "pase al compañero",
    }
}

/// Skills del atacante que tienen línea de tiro.
fn es_de_tiro(skill: SkillId) -> bool {
    matches!(
        skill,
        SkillId::ShootPush | SkillId::Clear | SkillId::SpinKick | SkillId::ApproachAligned
    )
}

// ── Dibujo ──────────────────────────────────────────────────────────────────

fn trazo(ancho: f32, color: Color) -> Stroke<'static> {
    Stroke::default().with_width(ancho).with_color(color)
}

fn punteado(ancho: f32, color: Color, segmentos: &'static [f32]) -> Stroke<'static> {
    Stroke {
        line_dash: LineDash {
            segments: segmentos,
            offset: 0,
        },
        ..trazo(ancho, color)
    }
}

fn texto(content: String, position: Point, color: Color, size: f32) -> canvas::Text {
    canvas::Text {
        content,
        position,
        color,
        size: size.into(),
        font: fonts::TEXTO,
        ..canvas::Text::default()
    }
}

fn finito(p: Point) -> bool {
    p.x.is_finite() && p.y.is_finite()
}

/// Lo que va debajo de los robots: trazas, reflejos y navegación.
pub struct Debajo<'a> {
    pub snap: &'a LoopSnapshot,
    pub capas: &'a HashSet<Layer>,
    pub trazas: &'a Trazas,
    pub own_team: i32,
    /// Robot propio seleccionado: su navegación va completa.
    pub seleccionado: Option<i32>,
    pub trazos: bool,
}

impl Debajo<'_> {
    pub fn dibujar(&self, frame: &mut Frame, vista: &Vista) {
        if self.capas.contains(&Layer::Trazas) {
            self.trazas_(frame, vista);
        }
        if self.capas.contains(&Layer::Reflejos) {
            self.reflejos(frame, vista);
        }
        if self.capas.contains(&Layer::Navegacion) {
            self.navegacion(frame, vista);
        }
    }

    fn pose(&self, id: i32) -> Option<Vec2> {
        self.snap
            .robot(self.own_team, id)
            .filter(|r| r.active && r.position.is_finite())
            .map(|r| r.position)
    }

    fn trazas_(&self, frame: &mut Frame, vista: &Vista) {
        for o in self.trazas.objetos() {
            let puntos: Vec<Vec2> = self.trazas.de(o).collect();
            if puntos.len() >= 2 {
                frame.stroke(
                    &polilinea(vista, &puntos),
                    trazo(3.0, theme::alfa(theme::CAL, 0.35)),
                );
            }
        }
    }

    fn reflejos(&self, frame: &mut Frame, vista: &Vista) {
        let Some(d) = self.snap.debug.as_deref() else {
            return;
        };
        // Áreas prohibidas para algún robot propio.
        let mut areas: Vec<AreaRect> = Vec::new();
        for a in d.robots.iter().flat_map(|r| r.prohibidas.iter()) {
            if !areas.contains(a) {
                areas.push(*a);
            }
        }
        for a in areas {
            let s = a.side;
            let relleno = theme::alfa(theme::ALERTA, 0.16);
            frame.fill(
                &rect_mundo(
                    vista,
                    s * AREA_X,
                    -AREA_HALF_Y,
                    s * FIELD_HALF_X,
                    AREA_HALF_Y,
                ),
                relleno,
            );
            frame.fill(&polilinea(vista, &super::field::arco_area(s)), relleno);
        }
        for r in &d.robots {
            let Some(p) = self.pose(r.id) else { continue };
            let c = vista.a_pantalla(p);
            if let Some(g) = r.guardia.filter(|g| g.proyeccion.is_finite()) {
                let q = vista.a_pantalla(g.proyeccion);
                let alerta = g.accion.is_some();
                let color = if alerta {
                    theme::ALERTA
                } else {
                    theme::alfa(theme::CAL, 0.55)
                };
                if (q.x - c.x).hypot(q.y - c.y) > 1.0 {
                    frame.stroke(
                        &Path::line(c, q),
                        punteado(if alerta { 3.0 } else { 1.4 }, color, PUNTEADO),
                    );
                }
                if let Some(acc) = g.accion {
                    let t = match acc {
                        AccionGuardia::Frena => "el guardia frena",
                        AccionGuardia::Saca => "el guardia lo saca",
                    };
                    let w = ancho_texto(t, theme::TXT_CHICO);
                    escribir(
                        frame,
                        texto(
                            t.to_string(),
                            Point::new(q.x - w / 2.0, q.y + 6.0),
                            theme::ALERTA,
                            theme::TXT_CHICO,
                        ),
                        self.trazos,
                    );
                }
            }
            if let Some(e) = r.escape.filter(|e| e.is_finite()) {
                if e != Vec2::ZERO {
                    let fin = vista.a_pantalla(p + e.normalize_or_zero() * FLECHA_ESCAPE_M);
                    flecha(frame, c, fin, theme::ALERTA);
                }
                escribir(
                    frame,
                    texto(
                        "escape".to_string(),
                        Point::new(c.x + 8.0, c.y + 10.0),
                        theme::ALERTA,
                        theme::TXT_CHICO,
                    ),
                    self.trazos,
                );
            }
        }
    }

    fn navegacion(&self, frame: &mut Frame, vista: &Vista) {
        let Some(d) = self.snap.debug.as_deref() else {
            return;
        };
        for r in &d.robots {
            let (Some(p), Some(nav)) = (self.pose(r.id), r.nav) else {
                continue;
            };
            if !(nav.destino.is_finite() && nav.mira.is_finite()) {
                continue;
            }
            let completa = self.seleccionado == Some(r.id);
            let (c, dest, mira) = (
                vista.a_pantalla(p),
                vista.a_pantalla(nav.destino),
                vista.a_pantalla(nav.mira),
            );
            let k = if completa { 1.0 } else { 0.45 };
            frame.stroke(
                &Path::circle(dest, vista.px(0.022).max(4.0)),
                trazo(1.8, theme::alfa(theme::CAL, k)),
            );
            frame.stroke(
                &Path::line(c, mira),
                trazo(if completa { 2.6 } else { 1.4 }, theme::alfa(theme::CAL, k)),
            );
            if !completa {
                continue;
            }
            frame.stroke(
                &Path::line(c, dest),
                punteado(1.4, theme::alfa(theme::CAL, 0.4), PUNTEADO_FINO),
            );
            if let Some(rodeo) = nav
                .rodeo
                .filter(|r| r.tangencia.is_finite() && r.holgura.is_finite())
            {
                let h = rodeo.holgura;
                let holgura = punteado(1.4, theme::alfa(theme::CAL, 0.75), PUNTEADO);
                match rodeo.obstaculo {
                    Rodeado::Pelota => {
                        if let Some(b) = self.snap.ball.as_ref().filter(|b| b.position.is_finite())
                        {
                            frame.stroke(
                                &Path::circle(vista.a_pantalla(b.position), vista.px(h)),
                                holgura,
                            );
                        }
                    }
                    Rodeado::AreaIzquierda | Rodeado::AreaDerecha => {
                        let s = if rodeo.obstaculo == Rodeado::AreaDerecha {
                            1.0
                        } else {
                            -1.0
                        };
                        frame.stroke(
                            &rect_mundo(
                                vista,
                                s * (AREA_X - h),
                                -(AREA_HALF_Y + h),
                                s * FIELD_HALF_X,
                                AREA_HALF_Y + h,
                            ),
                            holgura,
                        );
                        let arco = vista.a_pantalla(Vec2::new(s * ARC_CENTER_X, 0.0));
                        frame.stroke(&Path::circle(arco, vista.px(ARC_RADIUS + h)), holgura);
                    }
                }
                let t = vista.a_pantalla(rodeo.tangencia);
                frame.fill(&Path::circle(t, 3.0), theme::CAL);
                let frase = lado_rodeo(p, nav.destino, nav.mira, rodeo.obstaculo);
                escribir(
                    frame,
                    texto(
                        frase,
                        Point::new(t.x + 6.0, t.y - 18.0),
                        theme::CAL,
                        theme::TXT_CHICO,
                    ),
                    self.trazos,
                );
            }
        }
        // Línea de tiro del atacante hacia el objetivo del coach.
        let Some(coach) = d.coach.as_ref() else {
            return;
        };
        let (Some(atacante), Some(skill), Some((objetivo, tipo))) =
            (coach.atacante, coach.skill_atacante, coach.objetivo)
        else {
            return;
        };
        let Some(b) = self.snap.ball.as_ref().filter(|b| b.position.is_finite()) else {
            return;
        };
        if !(es_de_tiro(skill) && objetivo.is_finite() && self.pose(atacante).is_some()) {
            return;
        }
        let (desde, hasta) = (vista.a_pantalla(b.position), vista.a_pantalla(objetivo));
        frame.stroke(
            &Path::line(desde, hasta),
            punteado(1.4, theme::alfa(theme::TIRO, 0.8), &[8.0, 5.0]),
        );
        let nombre = nombre_objetivo(tipo);
        let w = ancho_texto(nombre, theme::TXT_CHICO);
        let pos = Point::new(hasta.x - w / 2.0, hasta.y + 6.0);
        if finito(pos) {
            escribir(
                frame,
                texto(nombre.to_string(), pos, theme::TIRO, theme::TXT_CHICO),
                self.trazos,
            );
        }
    }
}

fn flecha(frame: &mut Frame, desde: Point, hasta: Point, color: Color) {
    let (dx, dy) = (hasta.x - desde.x, hasta.y - desde.y);
    let largo = dx.hypot(dy);
    if largo < 1.0 {
        return;
    }
    let (ux, uy) = (dx / largo, dy / largo);
    let punta = |giro: f32| {
        let (s, c) = giro.sin_cos();
        Point::new(
            hasta.x - 9.0 * (ux * c - uy * s),
            hasta.y - 9.0 * (ux * s + uy * c),
        )
    };
    frame.stroke(&Path::line(desde, hasta), trazo(2.4, color));
    frame.stroke(&Path::line(hasta, punta(0.5)), trazo(2.4, color));
    frame.stroke(&Path::line(hasta, punta(-0.5)), trazo(2.4, color));
}

/// Lo que va encima de los robots: los rótulos del coach y la línea de skills.
pub struct Encima<'a> {
    pub snap: &'a LoopSnapshot,
    pub capas: &'a HashSet<Layer>,
    pub own_team: i32,
    pub trazos: bool,
}

impl Encima<'_> {
    pub fn dibujar(&self, frame: &mut Frame, vista: &Vista) {
        for v in &self.snap.own {
            let Some(p) = self
                .snap
                .robot(self.own_team, v.id)
                .filter(|r| r.active && r.position.is_finite())
                .map(|r| r.position)
            else {
                continue;
            };
            let c = vista.a_pantalla(p);
            if self.capas.contains(&Layer::Coach)
                && let Some(r) = rotulo_coach(v)
            {
                self.rotulo(frame, c, r);
            }
            if self.capas.contains(&Layer::Skills)
                && let Some(st) = v.status.filter(|_| v.skill.is_some())
            {
                let (linea, no_factible) = linea_skill(st);
                let y = c.y + vista.px(0.045) + 4.0;
                let w = ancho_texto(&linea, theme::TXT_CHICO);
                let total = if no_factible {
                    w + ancho_texto(", no factible", theme::TXT_CHICO)
                } else {
                    w
                };
                let x = c.x - total / 2.0;
                escribir(
                    frame,
                    texto(linea, Point::new(x, y), theme::CAL, theme::TXT_CHICO),
                    self.trazos,
                );
                if no_factible {
                    escribir(
                        frame,
                        texto(
                            ", no factible".to_string(),
                            Point::new(x + w, y),
                            theme::ALERTA,
                            theme::TXT_CHICO,
                        ),
                        self.trazos,
                    );
                }
            }
        }
    }

    fn rotulo(&self, frame: &mut Frame, c: Point, contenido: String) {
        let size = theme::TXT_CHICO;
        let w = ancho_texto(&contenido, size) + 14.0;
        let h = size * 1.6;
        let centro = Point::new(c.x - 46.0, c.y - 42.0);
        frame.stroke(
            &Path::line(c, centro),
            trazo(1.0, theme::alfa(theme::CAL, 0.7)),
        );
        let esquina = Point::new(centro.x - w / 2.0, centro.y - h / 2.0);
        frame.fill(&Path::rectangle(esquina, iced::Size::new(w, h)), theme::CAL);
        let pos = Point::new(esquina.x + 7.0, esquina.y + (h - size * 1.3) / 2.0);
        let t = canvas::Text {
            font: fonts::TITULO,
            ..texto(contenido, pos, theme::TINTA, size)
        };
        escribir(frame, t, self.trazos);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::snapshot::{BallView, RobotView};

    fn vista_propia(
        source: Option<CommandSource>,
        role: Option<Role>,
        skill: Option<SkillId>,
    ) -> OwnRobotView {
        OwnRobotView {
            id: 0,
            source,
            role,
            skill,
            status: None,
            target: None,
            command: None,
            escaping: false,
        }
    }

    #[test]
    fn coach_labels_say_role_and_action() {
        let v = vista_propia(
            Some(CommandSource::Coach),
            Some(Role::Striker),
            Some(SkillId::ApproachAligned),
        );
        assert_eq!(rotulo_coach(&v).unwrap(), "Atacante, se alinea");
        let v = vista_propia(
            Some(CommandSource::Coach),
            Some(Role::Support),
            Some(SkillId::BlockLine),
        );
        assert_eq!(rotulo_coach(&v).unwrap(), "Apoyo, bloquea");
        let v = vista_propia(
            Some(CommandSource::Coach),
            Some(Role::Keeper),
            Some(SkillId::GoalKeep),
        );
        assert_eq!(rotulo_coach(&v).unwrap(), "Arquero");
        let v = vista_propia(
            Some(CommandSource::Coach),
            Some(Role::Keeper),
            Some(SkillId::Clear),
        );
        assert_eq!(rotulo_coach(&v).unwrap(), "Arquero, despeja");
        let v = vista_propia(Some(CommandSource::Coach), None, Some(SkillId::GoTo));
        assert_eq!(
            rotulo_coach(&v).unwrap(),
            "Va al punto",
            "sin rol, solo lo que hace"
        );
        assert_eq!(
            rotulo_coach(&vista_propia(Some(CommandSource::Manual), None, None)).unwrap(),
            "Control manual"
        );
        assert!(
            rotulo_coach(&vista_propia(None, Some(Role::Striker), None)).is_none(),
            "sin comando, sin rótulo"
        );
    }

    #[test]
    fn skill_line_has_progress_face_and_feasibility() {
        let st = SkillStatus {
            progress: 0.72,
            done: false,
            feasible: true,
            face: Face::Back,
        };
        assert_eq!(linea_skill(st), ("72 %, de espaldas".to_string(), false));
        let st = SkillStatus {
            feasible: false,
            face: Face::Front,
            ..st
        };
        assert_eq!(linea_skill(st), ("72 %, de frente".to_string(), true));
    }

    #[test]
    fn detour_side_is_seen_from_the_robot() {
        let (robot, destino) = (Vec2::new(-0.4, 0.0), Vec2::new(0.4, 0.0));
        assert_eq!(
            lado_rodeo(robot, destino, Vec2::new(0.0, 0.1), Rodeado::Pelota),
            "rodea la pelota por la izquierda"
        );
        assert_eq!(
            lado_rodeo(robot, destino, Vec2::new(0.0, -0.1), Rodeado::AreaDerecha),
            "rodea el área por la derecha"
        );
    }

    fn foto(t_ms: u64, x: f32) -> LoopSnapshot {
        LoopSnapshot {
            t_ms,
            robots: vec![RobotView {
                team: 0,
                id: 1,
                position: Vec2::new(x, 0.0),
                orientation: 0.0,
                velocity: Vec2::ZERO,
                angular_velocity: 0.0,
                active: true,
                age_s: 0.0,
            }],
            ball: Some(BallView {
                position: Vec2::new(f32::NAN, 0.0),
                velocity: Vec2::ZERO,
            }),
            ..LoopSnapshot::default()
        }
    }

    #[test]
    fn trails_keep_the_last_three_seconds_and_skip_non_finite() {
        let mut t = Trazas::default();
        for k in 0..300u64 {
            // 10 s a 30 fotos/s.
            t.registrar(&foto(k * 33, k as f32 * 0.001));
        }
        let puntos: Vec<Vec2> = t.de(Objeto::Robot(0, 1)).collect();
        assert!(
            (88..=92).contains(&puntos.len()),
            "3 s a 30 fotos/s: {}",
            puntos.len()
        );
        assert_eq!(
            t.de(Objeto::Pelota).count(),
            0,
            "la pelota en NaN no deja traza"
        );
        // El lazo volvió a arrancar: se descarta lo viejo.
        t.registrar(&foto(10, 0.0));
        assert_eq!(t.de(Objeto::Robot(0, 1)).count(), 1);
    }
}
