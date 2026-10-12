//! Pestaña **Rutinas** y capa **Rutina** (U) de la GUI (capability `gui-rutinas`).
//!
//! La GUI lee y valida los archivos ella misma (con las funciones de
//! `crate::rutinas::formato`), así que la lista y la vista previa no pasan por el lazo:
//! seleccionar una rutina o cambiar sus opciones no manda nada. Preparar es la primera
//! orden que mueve robots; las órdenes van por `GuiSetup.rutina_tx` al conmutador de
//! `main`. El estado de la corrida llega en la foto (`LoopSnapshot::rutina`).

use super::field::{Vista, ancho_texto, escribir, polilinea};
use super::{DebugGui, Message, fonts, theme};
use crate::control_loop::TickMode;
use crate::params::params;
use crate::rutinas::formato::{Accion, Duracion, Entrada, Grupo, Repeticiones, Rutina};
use crate::rutinas::vista::{Fase, RutinaVista, segundos, vueltas};
use crate::rutinas::{ContextoValidacion, Eleccion, OpcionesCorrida, OrdenRutina, asignar};
use crate::snapshot::LoopSnapshot;
use glam::Vec2;
use iced::widget::canvas::{self, Frame, LineDash, Path, Stroke};
use iced::widget::{Column, button, horizontal_rule, row, text};
use iced::{Alignment, Color, Element, Length, Point};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::mpsc;

/// Botones de la corrida.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BotonRutina {
    Preparar,
    Iniciar,
    Pausar,
    Reanudar,
    Abortar,
    Cerrar,
}

impl BotonRutina {
    fn etiqueta(self) -> &'static str {
        match self {
            BotonRutina::Preparar => "Preparar",
            BotonRutina::Iniciar => "Iniciar",
            BotonRutina::Pausar => "Pausar",
            BotonRutina::Reanudar => "Reanudar",
            BotonRutina::Abortar => "Abortar",
            BotonRutina::Cerrar => "Cerrar",
        }
    }
}

/// Mensajes de la pestaña.
#[derive(Debug, Clone)]
pub enum MensajeRutina {
    Recargar,
    Seleccionar(String),
    RobotsManual(bool),
    /// Manual: el robot del puesto `k`.
    RobotDePuesto(usize, i32),
    Velocidad(u32),
    Repeticiones(Repeticiones),
    Teleport(bool),
    Orden(BotonRutina),
}

/// Botones que se muestran según la fase de la rutina activa (`None` = no hay).
pub fn botones(fase: Option<Fase>) -> Vec<BotonRutina> {
    use BotonRutina::*;
    match fase {
        None => vec![Preparar],
        Some(Fase::Preparando) => vec![Abortar],
        Some(Fase::Lista) => vec![Iniciar, Abortar],
        Some(Fase::Ejecutando) => vec![Pausar, Abortar],
        Some(Fase::Pausada) => vec![Reanudar, Abortar],
        Some(Fase::Terminada | Fase::Abortada) => vec![Cerrar, Preparar],
    }
}

/// Robots propios que la foto ve, con id < `num_robots` y pose finita.
pub fn vistos_propios(snap: &LoopSnapshot, own_team: i32, num_robots: usize) -> Vec<(i32, Vec2)> {
    snap.robots
        .iter()
        .filter(|r| {
            r.team == own_team && r.active && r.position.is_finite() && r.id >= 0 && (r.id as usize) < num_robots
        })
        .map(|r| (r.id, r.position))
        .collect()
}

/// Por qué no se puede preparar la rutina elegida, o `None` si se puede.
pub fn razon_no_preparar(
    hay_canal: bool,
    entrada: Option<&Entrada>,
    snap: &LoopSnapshot,
    eleccion: &Eleccion,
    own_team: i32,
    num_robots: usize,
    keeper_id: i32,
) -> Option<String> {
    if !hay_canal {
        return Some("Las rutinas se lanzan desde main con la GUI (VSSL_DEBUG_GUI=1).".to_string());
    }
    let Some(e) = entrada else {
        return Some("Elige una rutina de la lista.".to_string());
    };
    let Some(rutina) = e.valida() else {
        return Some(e.razones.first().cloned().unwrap_or_else(|| "La rutina no se pudo leer.".to_string()));
    };
    if snap.estop {
        return Some("La parada de emergencia está activa.".to_string());
    }
    match snap.mode {
        TickMode::Halt => return Some("El árbitro está en HALT.".to_string()),
        TickMode::Stop => return Some("El árbitro está en STOP.".to_string()),
        TickMode::SinVision => return Some("Corte sin visión: no se ve ningún robot propio.".to_string()),
        TickMode::Play => {}
    }
    asignar(rutina, &vistos_propios(snap, own_team, num_robots), keeper_id, eleccion)
        .err()
        .map(|e| format!("No se puede preparar: {e}."))
}

/// Estado de la pestaña.
pub struct EstadoRutinas {
    tx: Option<mpsc::Sender<OrdenRutina>>,
    pub entradas: Vec<Entrada>,
    error_carpeta: Option<String>,
    elegida: Option<String>,
    manual: bool,
    /// Manual: robot de cada puesto.
    robots: Vec<i32>,
    velocidad_pct: u32,
    /// `None` = las del archivo.
    repeticiones: Option<Repeticiones>,
    teletransportar: bool,
    aviso: Option<String>,
}

/// Porcentaje por defecto según el transporte.
fn velocidad_por_defecto(simulacion: bool) -> u32 {
    let p = &params().rutinas;
    if simulacion { p.velocidad_sim_pct } else { p.velocidad_real_pct }
}

impl EstadoRutinas {
    pub fn new(tx: Option<mpsc::Sender<OrdenRutina>>, own_team: i32, simulacion: bool) -> Self {
        let mut e = Self {
            tx,
            entradas: Vec::new(),
            error_carpeta: None,
            elegida: None,
            manual: false,
            robots: Vec::new(),
            velocidad_pct: velocidad_por_defecto(simulacion),
            repeticiones: None,
            teletransportar: false,
            aviso: None,
        };
        e.recargar(own_team);
        e
    }

    /// Vuelve a leer la carpeta de rutinas (sin recompilar).
    fn recargar(&mut self, own_team: i32) {
        let dir = crate::rutinas::carpeta();
        match crate::rutinas::cargar_carpeta(&dir, &ContextoValidacion::vigente(own_team)) {
            Ok(entradas) => {
                self.entradas = entradas;
                self.error_carpeta = None;
            }
            Err(e) => {
                self.entradas.clear();
                self.error_carpeta = Some(format!("No se pudo leer la carpeta de rutinas: {e}"));
            }
        }
    }

    fn entrada(&self) -> Option<&Entrada> {
        let id = self.elegida.as_deref()?;
        self.entradas.iter().find(|e| e.id == id)
    }

    fn eleccion(&self) -> Eleccion {
        if self.manual { Eleccion::Manual(self.robots.clone()) } else { Eleccion::Automatica }
    }

    fn repeticiones_de(&self, rutina: &Rutina) -> Repeticiones {
        self.repeticiones.unwrap_or(rutina.repeticiones)
    }
}

impl DebugGui {
    /// Atiende un mensaje de la pestaña Rutinas.
    pub(super) fn mensaje_rutina(&mut self, m: MensajeRutina) {
        let sim = !self.run.is_base_station();
        let r = &mut self.rutinas;
        match m {
            MensajeRutina::Recargar => r.recargar(self.run.own_team),
            MensajeRutina::Seleccionar(id) => {
                let puestos = r
                    .entradas
                    .iter()
                    .find(|e| e.id == id)
                    .and_then(|e| e.rutina.as_ref())
                    .map_or(0, |x| x.robots_max.min(x.puestos.len()));
                r.elegida = Some(id);
                r.manual = false;
                r.robots = (0..puestos as i32).collect();
                r.velocidad_pct = velocidad_por_defecto(sim);
                r.repeticiones = None;
                r.teletransportar = false;
                r.aviso = None;
            }
            MensajeRutina::RobotsManual(on) => r.manual = on,
            MensajeRutina::RobotDePuesto(k, id) => {
                if let Some(x) = r.robots.get_mut(k) {
                    *x = id.clamp(0, self.run.num_robots.saturating_sub(1) as i32);
                }
            }
            MensajeRutina::Velocidad(p) => r.velocidad_pct = p.clamp(20, 100) / 10 * 10,
            MensajeRutina::Repeticiones(n) => r.repeticiones = Some(n),
            MensajeRutina::Teleport(on) => r.teletransportar = on && sim,
            MensajeRutina::Orden(b) => self.enviar_orden(b),
        }
    }

    fn enviar_orden(&mut self, b: BotonRutina) {
        let sim = !self.run.is_base_station();
        let r = &self.rutinas;
        let orden = match b {
            BotonRutina::Preparar => {
                let Some(rutina) = r.entrada().and_then(|e| e.valida()).cloned() else {
                    return;
                };
                let opciones = OpcionesCorrida {
                    eleccion: r.eleccion(),
                    velocidad_pct: r.velocidad_pct,
                    repeticiones: r.repeticiones_de(&rutina),
                    teletransportar: r.teletransportar && sim,
                    simulacion: sim,
                    inicio_automatico: false,
                };
                OrdenRutina::Preparar { rutina, opciones }
            }
            BotonRutina::Iniciar => OrdenRutina::Iniciar,
            BotonRutina::Pausar => OrdenRutina::Pausar,
            BotonRutina::Reanudar => OrdenRutina::Reanudar,
            BotonRutina::Abortar => OrdenRutina::Abortar,
            BotonRutina::Cerrar => OrdenRutina::Cerrar,
        };
        let aviso = match &self.rutinas.tx {
            None => Some("No hay canal de rutinas.".to_string()),
            Some(tx) => tx.try_send(orden).err().map(|e| format!("No se pudo enviar la orden: {e}")),
        };
        self.rutinas.aviso = aviso;
    }

    /// Lo que dibuja la capa U: la rutina activa (de la foto) o, si no hay, la elegida
    /// con la asignación prevista.
    pub(super) fn dibujo_rutina(&self) -> Option<DibujoRutina> {
        if let Some(v) = self.snap.rutina.as_deref() {
            return Some(DibujoRutina::de_vista(v));
        }
        let rutina = self.rutinas.entrada()?.rutina.clone()?;
        let vistos = vistos_propios(&self.snap, self.run.own_team, self.run.num_robots);
        let asignados = asignar(&rutina, &vistos, self.keeper_id, &self.rutinas.eleccion()).unwrap_or_default();
        let puestos = if asignados.is_empty() { rutina.robots_max } else { asignados.len() };
        Some(DibujoRutina {
            asignacion: asignados.into_iter().enumerate().collect(),
            en_curso: Vec::new(),
            puestos: puestos.min(rutina.puestos.len()),
            rutina,
        })
    }

    /// Pestaña Rutinas.
    pub(super) fn pestana_rutinas(&self) -> Element<'_, Message> {
        let r = &self.rutinas;
        let sim = !self.run.is_base_station();
        let validas = r.entradas.iter().filter(|e| e.razones.is_empty()).count();
        let cuenta = format!(
            "{} {}, {} con errores",
            r.entradas.len(),
            if r.entradas.len() == 1 { "rutina" } else { "rutinas" },
            r.entradas.len() - validas
        );
        let mut col = Column::new().spacing(theme::SP_SM).push(titulo("Rutinas")).push(
            row![boton_chico("Recargar", Some(Message::Rutina(MensajeRutina::Recargar))), nota(cuenta)]
                .spacing(theme::SP_SM)
                .align_y(Alignment::Center),
        );
        if let Some(e) = &r.error_carpeta {
            col = col.push(text(e.clone()).size(theme::TXT_CHICO).style(theme::alerta));
        }
        // Lista por grupo; los archivos que no se pudieron leer, al final.
        for g in Grupo::TODOS {
            let del_grupo: Vec<&Entrada> =
                r.entradas.iter().filter(|e| e.rutina.as_ref().is_some_and(|x| x.grupo == g)).collect();
            if del_grupo.is_empty() {
                continue;
            }
            col = col.push(subtitulo(g.titulo()));
            for e in del_grupo {
                col = col.push(self.fila_rutina(e));
            }
        }
        let rotas: Vec<&Entrada> = r.entradas.iter().filter(|e| e.rutina.is_none()).collect();
        if !rotas.is_empty() {
            col = col.push(subtitulo("Archivos con errores"));
            for e in rotas {
                col = col.push(self.fila_rutina(e));
            }
        }
        col = col.push(horizontal_rule(1).style(theme::filete));
        // La activa manda; si no hay, la elegida.
        let activa = self.snap.rutina.as_deref();
        let fase = activa.map(|v| v.fase);
        if let Some(e) = r.entrada().filter(|_| activa.is_none() || fase.is_some_and(|f| !f.activa())) {
            col = col.push(self.opciones_rutina(e, sim));
        }
        // Botones según la fase.
        let razon = razon_no_preparar(
            r.tx.is_some(),
            r.entrada(),
            &self.snap,
            &r.eleccion(),
            self.run.own_team,
            self.run.num_robots,
            self.keeper_id,
        );
        let mut fila = row![].spacing(theme::SP_SM);
        for b in botones(fase) {
            let habilitado = b != BotonRutina::Preparar || razon.is_none();
            fila = fila.push(boton_chico(b.etiqueta(), habilitado.then_some(Message::Rutina(MensajeRutina::Orden(b)))));
        }
        col = col.push(fila);
        if botones(fase).contains(&BotonRutina::Preparar)
            && let Some(razon) = razon
        {
            col = col.push(text(razon).size(theme::TXT_CHICO).style(theme::alerta));
        }
        if let Some(v) = activa {
            col = col.push(text(v.frase.clone()).size(theme::TXT).style(theme::tinta_o_alerta(v.fase == Fase::Abortada)));
            if let Some(res) = &v.resumen {
                for l in res.lineas() {
                    col = col.push(nota(l));
                }
            }
        }
        if let Some(a) = &r.aviso {
            col = col.push(text(a.clone()).size(theme::TXT_CHICO).style(theme::alerta));
        }
        col.into()
    }

    fn fila_rutina(&self, e: &Entrada) -> Element<'_, Message> {
        let elegida = self.rutinas.elegida.as_deref() == Some(e.id.as_str());
        let valida = e.razones.is_empty();
        let tinta = if valida { theme::TINTA } else { theme::GRIS };
        let mut c = Column::new().spacing(2);
        match &e.rutina {
            Some(x) => {
                c = c
                    .push(text(x.nombre.clone()).font(fonts::TEXTO_FUERTE).size(theme::TXT).color(tinta))
                    .push(nota(format!(
                        "{} · {}",
                        x.texto_robots(),
                        if x.pelota.is_some() { "con pelota" } else { "sin pelota" }
                    )))
                    .push(nota(x.descripcion.clone()));
            }
            None => c = c.push(text(e.id.clone()).font(fonts::TEXTO_FUERTE).size(theme::TXT).color(tinta)),
        }
        if let Some(razon) = e.razones.first() {
            c = c.push(text(razon.clone()).size(theme::TXT_CHICO).style(theme::alerta));
            for otra in e.razones.iter().skip(1).filter(|_| elegida) {
                c = c.push(text(otra.clone()).size(theme::TXT_MINI).style(theme::alerta));
            }
        }
        button(c)
            .width(Length::Fill)
            .padding([4, 6])
            .style(theme::fila(elegida))
            .on_press(Message::Rutina(MensajeRutina::Seleccionar(e.id.clone())))
            .into()
    }

    fn opciones_rutina(&self, e: &Entrada, sim: bool) -> Element<'_, Message> {
        let r = &self.rutinas;
        let m = |x: MensajeRutina| Message::Rutina(x);
        let mut col = Column::new().spacing(theme::SP_SM);
        let Some(rutina) = &e.rutina else {
            return col.into();
        };
        col = col.push(subtitulo(&rutina.nombre));
        col = col.push(
            row![
                etiqueta("robots"),
                opcion("Automático", !r.manual, m(MensajeRutina::RobotsManual(false))),
                opcion("Manual", r.manual, m(MensajeRutina::RobotsManual(true))),
            ]
            .spacing(theme::SP_SM)
            .align_y(Alignment::Center),
        );
        if r.manual {
            for (k, id) in r.robots.iter().enumerate() {
                let nombre = rutina.puestos.get(k).map_or("?", |p| p.nombre.as_str());
                col = col.push(
                    row![
                        etiqueta(format!("puesto {nombre}, robot")),
                        opcion("−", false, m(MensajeRutina::RobotDePuesto(k, id - 1))),
                        text(id.to_string()).size(theme::TXT),
                        opcion("+", false, m(MensajeRutina::RobotDePuesto(k, id + 1))),
                    ]
                    .spacing(theme::SP_SM)
                    .align_y(Alignment::Center),
                );
            }
        }
        let pct = r.velocidad_pct;
        col = col.push(
            row![
                etiqueta("velocidad"),
                opcion("−", false, m(MensajeRutina::Velocidad(pct.saturating_sub(10).max(20)))),
                text(format!("{pct} %")).size(theme::TXT),
                opcion("+", false, m(MensajeRutina::Velocidad((pct + 10).min(100)))),
            ]
            .spacing(theme::SP_SM)
            .align_y(Alignment::Center),
        );
        let reps = r.repeticiones_de(rutina);
        let n = match reps {
            Repeticiones::Veces(n) => n,
            Repeticiones::Bucle => 1,
        };
        col = col.push(
            row![
                etiqueta("repeticiones"),
                opcion("−", false, m(MensajeRutina::Repeticiones(Repeticiones::Veces(n.saturating_sub(1).max(1))))),
                text(reps.texto()).size(theme::TXT),
                opcion("+", false, m(MensajeRutina::Repeticiones(Repeticiones::Veces(n + 1)))),
                opcion(
                    "bucle",
                    reps == Repeticiones::Bucle,
                    m(MensajeRutina::Repeticiones(if reps == Repeticiones::Bucle {
                        rutina.repeticiones
                    } else {
                        Repeticiones::Bucle
                    })),
                ),
            ]
            .spacing(theme::SP_SM)
            .align_y(Alignment::Center),
        );
        if sim {
            col = col.push(opcion("teletransportar", r.teletransportar, m(MensajeRutina::Teleport(!r.teletransportar))));
        }
        col.into()
    }
}

fn titulo(t: impl ToString) -> Element<'static, Message> {
    text(t.to_string()).font(fonts::TITULO).size(theme::TXT_TITULO).into()
}

fn subtitulo(t: impl ToString) -> Element<'static, Message> {
    text(t.to_string()).font(fonts::TITULO_MEDIO).size(theme::TXT).into()
}

fn nota(t: impl ToString) -> Element<'static, Message> {
    text(t.to_string()).size(theme::TXT_CHICO).style(theme::gris).into()
}

fn etiqueta(t: impl ToString) -> Element<'static, Message> {
    text(t.to_string()).size(theme::TXT_CHICO).style(theme::gris).into()
}

fn opcion<'a>(t: impl ToString, activa: bool, msg: Message) -> Element<'a, Message> {
    button(text(t.to_string()).size(theme::TXT_CHICO))
        .padding([3, 8])
        .style(theme::boton_opcion(activa))
        .on_press(msg)
        .into()
}

/// Botón chico; sin mensaje, deshabilitado.
fn boton_chico<'a>(t: &str, msg: Option<Message>) -> Element<'a, Message> {
    button(text(t.to_string()).size(theme::TXT))
        .padding([4, 10])
        .style(theme::boton)
        .on_press_maybe(msg)
        .into()
}

// ── Capa Rutina (U) ──────────────────────────────────────────────────────────

/// Lo que dibuja la capa U.
#[derive(Debug, Clone)]
pub struct DibujoRutina {
    pub rutina: Arc<Rutina>,
    /// (puesto, robot) de la asignación real o prevista.
    pub asignacion: Vec<(usize, i32)>,
    /// (puesto, paso en curso), en ejecución.
    pub en_curso: Vec<(usize, usize)>,
    /// Cuántos puestos dibujar.
    pub puestos: usize,
}

impl DibujoRutina {
    pub fn de_vista(v: &RutinaVista) -> Self {
        Self {
            rutina: v.rutina.clone(),
            asignacion: v.robots.iter().map(|r| (r.puesto, r.robot)).collect(),
            en_curso: v.robots.iter().filter_map(|r| r.paso.map(|p| (r.puesto, p))).collect(),
            puestos: v.robots.len().max(1).min(v.rutina.puestos.len()),
        }
    }
}

const PUNTEADO: &[f32] = &[5.0, 4.0];

fn trazo(ancho: f32, color: Color) -> Stroke<'static> {
    Stroke::default().with_width(ancho).with_color(color)
}

fn punteado(ancho: f32, color: Color) -> Stroke<'static> {
    Stroke {
        line_dash: LineDash { segments: PUNTEADO, offset: 0 },
        ..trazo(ancho, color)
    }
}

fn texto(content: String, position: Point, color: Color) -> canvas::Text {
    canvas::Text {
        content,
        position,
        color,
        size: theme::TXT_CHICO.into(),
        font: fonts::TEXTO,
        ..canvas::Text::default()
    }
}

fn finito(p: Point) -> bool {
    p.x.is_finite() && p.y.is_finite()
}

/// Nota corta de un paso sin traslación (`None` para `ir`).
pub fn nota_de_paso(a: &Accion) -> Option<String> {
    match a {
        Accion::Ir { .. } | Accion::Recibir { .. } => None,
        Accion::Mirar { .. } => Some("mira".to_string()),
        Accion::Girar { duracion: Duracion::Vueltas(v), .. } => Some(format!("gira {}", vueltas(*v).replace(" vueltas", " v").replace(" vuelta", " v"))),
        Accion::Girar { duracion: Duracion::Tiempo(t), .. } => Some(format!("gira {} s", segundos(*t))),
        Accion::Esperar { tiempo_s } => Some(format!("espera {} s", segundos(*tiempo_s))),
        otro => Some(otro.tipo().to_string()),
    }
}

/// Dibuja la rutina debajo de los robots. `pose` da la posición de un robot propio.
pub fn dibujar(frame: &mut Frame, vista: &Vista, d: &DibujoRutina, pose: impl Fn(i32) -> Option<Vec2>, trazos: bool) {
    let cal = |a: f32| theme::alfa(theme::CAL, a);
    let vuelta = d.rutina.repeticiones.mas_de_una();
    // Notas apiladas por lugar, para que no se pisen.
    let mut notas_en: HashMap<(i32, i32), u32> = HashMap::new();
    let mut nota = |frame: &mut Frame, p: Vec2, t: String, color: Color| {
        let c = vista.a_pantalla(p);
        if !finito(c) {
            return;
        }
        let clave = ((p.x * 100.0).round() as i32, (p.y * 100.0).round() as i32);
        let n = notas_en.entry(clave).or_insert(0);
        let pos = Point::new(c.x + 8.0, c.y - 18.0 + 14.0 * *n as f32);
        *n += 1;
        escribir(frame, texto(t, pos, color), trazos);
    };
    for (k, puesto) in d.rutina.puestos.iter().enumerate().take(d.puestos) {
        let en_curso = d.en_curso.iter().find(|(p, _)| *p == k).map(|(_, i)| *i);
        // Recorrido: del inicio a cada destino, y la vuelta punteada.
        let pts = puesto.recorrido(false);
        if pts.len() >= 2 {
            frame.stroke(&polilinea(vista, &pts), trazo(1.8, cal(0.7)));
        }
        if vuelta && let (Some(ultimo), Some(primero)) = (puesto.destinos().last(), puesto.destinos().next()) {
            let (a, b) = (vista.a_pantalla(ultimo), vista.a_pantalla(primero));
            if finito(a) && finito(b) {
                frame.stroke(&Path::line(a, b), punteado(1.4, cal(0.5)));
            }
        }
        // Inicio con su orientación y la letra del puesto.
        let ini = puesto.inicio;
        let c = vista.a_pantalla(ini.pos);
        if finito(c) {
            frame.stroke(&Path::circle(c, vista.px(0.04).max(5.0)), trazo(1.6, cal(0.9)));
            let dir = Vec2::new(ini.orientacion.cos() as f32, ini.orientacion.sin() as f32);
            let punta = vista.a_pantalla(ini.pos + dir * 0.07);
            if finito(punta) {
                frame.stroke(&Path::line(c, punta), trazo(2.0, cal(0.9)));
            }
            let w = ancho_texto(&puesto.nombre, theme::TXT_CHICO);
            escribir(frame, texto(puesto.nombre.clone(), Point::new(c.x - w / 2.0, c.y + 10.0), theme::CAL), trazos);
        }
        // Pasos numerados en el lugar donde ocurren.
        let mut lugar = ini.pos;
        for (i, paso) in puesto.pasos.iter().enumerate() {
            let actual = en_curso == Some(i);
            let color = if actual { theme::TIRO } else { theme::CAL };
            if let Some(dest) = paso.accion.destino() {
                lugar = dest;
                let q = vista.a_pantalla(dest);
                if finito(q) {
                    let r = if actual { vista.px(0.03).max(6.0) } else { vista.px(0.015).max(3.0) };
                    frame.stroke(&Path::circle(q, r), trazo(if actual { 2.6 } else { 1.6 }, color));
                }
                nota(frame, dest, (i + 1).to_string(), color);
            } else {
                if let Accion::Mirar { punto } = &paso.accion {
                    let (a, b) = (vista.a_pantalla(lugar), vista.a_pantalla(*punto));
                    if finito(a) && finito(b) {
                        frame.stroke(&Path::line(a, b), punteado(1.2, theme::alfa(color, 0.6)));
                    }
                }
                let t = format!("{} {}", i + 1, nota_de_paso(&paso.accion).unwrap_or_default());
                nota(frame, lugar, t, color);
            }
        }
    }
    // Asignación: del robot a su puesto, con "A ← 1".
    for (k, robot) in &d.asignacion {
        let Some(puesto) = d.rutina.puestos.get(*k) else { continue };
        let ini = vista.a_pantalla(puesto.inicio.pos);
        let Some(p) = pose(*robot).filter(|p| p.is_finite()) else { continue };
        let q = vista.a_pantalla(p);
        if !(finito(ini) && finito(q)) {
            continue;
        }
        if (p - puesto.inicio.pos).length() > 0.05 {
            frame.stroke(&Path::line(q, ini), punteado(1.4, cal(0.8)));
        }
        let t = format!("{} ← {robot}", puesto.nombre);
        escribir(frame, texto(t, Point::new(ini.x + 10.0, ini.y + 10.0), theme::CAL), trazos);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rutinas::prueba;
    use crate::snapshot::RobotView;

    fn robot(id: i32, x: f32, y: f32) -> RobotView {
        RobotView {
            team: 0,
            id,
            position: Vec2::new(x, y),
            orientation: 0.0,
            velocity: Vec2::ZERO,
            angular_velocity: 0.0,
            active: true,
            age_s: 0.0,
        }
    }

    fn entrada(texto: &str) -> Entrada {
        crate::rutinas::formato::cargar_texto(
            texto,
            "prueba",
            std::path::PathBuf::from("prueba.json"),
            &ContextoValidacion::por_defecto(1.0),
        )
    }

    #[test]
    fn buttons_by_phase() {
        use BotonRutina::*;
        assert_eq!(botones(None), vec![Preparar]);
        assert_eq!(botones(Some(Fase::Preparando)), vec![Abortar]);
        assert_eq!(botones(Some(Fase::Lista)), vec![Iniciar, Abortar]);
        assert_eq!(botones(Some(Fase::Ejecutando)), vec![Pausar, Abortar]);
        assert_eq!(botones(Some(Fase::Pausada)), vec![Reanudar, Abortar]);
        assert_eq!(botones(Some(Fase::Terminada)), vec![Cerrar, Preparar]);
        assert_eq!(botones(Some(Fase::Abortada)), vec![Cerrar, Preparar]);
    }

    #[test]
    fn why_it_cannot_be_prepared() {
        let ok = entrada(&prueba::un_puesto(r#"{ "paso": "ir", "x": 0.3, "y": -0.25 }"#));
        let snap = LoopSnapshot { robots: vec![robot(0, 0.0, 0.0)], ..LoopSnapshot::default() };
        let auto = Eleccion::Automatica;
        let razon = |canal, e: Option<&Entrada>, s: &LoopSnapshot, el: &Eleccion| razon_no_preparar(canal, e, s, el, 0, 3, 2);
        assert!(razon(false, Some(&ok), &snap, &auto).unwrap().contains("main con la GUI"));
        assert!(razon(true, None, &snap, &auto).unwrap().contains("Elige una rutina"));
        assert_eq!(razon(true, Some(&ok), &snap, &auto), None);
        let stop = LoopSnapshot { mode: TickMode::Stop, ..snap.clone() };
        assert_eq!(razon(true, Some(&ok), &stop, &auto).as_deref(), Some("El árbitro está en STOP."));
        let parada = LoopSnapshot { estop: true, mode: TickMode::Halt, ..snap.clone() };
        assert!(razon(true, Some(&ok), &parada, &auto).unwrap().contains("parada de emergencia"));
        let vacia = LoopSnapshot::default();
        assert!(razon(true, Some(&ok), &vacia, &auto).unwrap().contains("se ven 0 robots propios"));
        assert!(razon(true, Some(&ok), &snap, &Eleccion::Manual(vec![1])).unwrap().contains("el robot 1 no se ve"));
        let mala = entrada(&prueba::un_puesto(r#"{ "paso": "ir", "x": 0.0, "y": 0.62 }"#));
        assert!(razon(true, Some(&mala), &snap, &auto).unwrap().contains("de la pared"));
    }

    #[test]
    fn only_own_finite_robots_count() {
        let mut snap = LoopSnapshot { robots: vec![robot(0, 0.0, 0.0), robot(1, f32::NAN, 0.0), robot(5, 0.1, 0.1)], ..LoopSnapshot::default() };
        snap.robots.push(RobotView { team: 1, ..robot(2, 0.2, 0.2) });
        assert_eq!(vistos_propios(&snap, 0, 3), vec![(0, Vec2::ZERO)]);
    }

    #[test]
    fn step_notes() {
        let a = |t: &str| prueba::rutina(&prueba::un_puesto(t)).puestos[0].pasos[0].accion.clone();
        assert_eq!(nota_de_paso(&a(r#"{ "paso": "girar", "sentido": "horario", "vueltas": 1 }"#)).as_deref(), Some("gira 1 v"));
        assert_eq!(nota_de_paso(&a(r#"{ "paso": "girar", "sentido": "horario", "vueltas": 2 }"#)).as_deref(), Some("gira 2 v"));
        assert_eq!(nota_de_paso(&a(r#"{ "paso": "esperar", "tiempo_s": 2 }"#)).as_deref(), Some("espera 2 s"));
        assert_eq!(nota_de_paso(&a(r#"{ "paso": "mirar", "x": 0, "y": 0 }"#)).as_deref(), Some("mira"));
        assert_eq!(nota_de_paso(&a(r#"{ "paso": "ir", "x": 0.3, "y": -0.25 }"#)), None);
    }
}
