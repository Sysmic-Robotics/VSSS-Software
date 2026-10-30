//! Pestaña **Robots** (capability `gui-robots`), solo con la base station: qué patrón
//! lleva cada robot físico (posición de radio) y si está activo, editable en vivo, con lo
//! que ve la cámara, los avisos de la identidad y el botón Identificar.
//!
//! La pestaña muestra la identidad de la foto (la vigente en el lazo). Cada cambio arma la
//! identidad nueva, la manda al lazo (`OrdenIdentidad::Usar`, rige desde el tick
//! siguiente) y la guarda en el archivo (`RunInfo::identidad_ruta`). Identificar es una
//! máquina pura (`crate::identidad::identificar`) que la GUI avanza con cada foto; el lazo
//! garantiza la seguridad del giro (exclusivo, caduca, se corta con la parada y HALT).

use super::format::num;
use super::robot_rows::NO_VISTO_S;
use super::rutinas::{boton_chico, etiqueta, nota, opcion_si, subtitulo, titulo};
use super::{DebugGui, Message, theme};
use crate::control_loop::TickMode;
use crate::identidad::identificar::{Condiciones, FaseGiro, Identificacion, Paso, Resultado, puede_identificar};
use crate::identidad::{Identidad, OrdenIdentidad, Origen, PATRONES, con_mayuscula, lista, nombre_radio};
use crate::radio::base_station::SLOT_COUNT;
use crate::snapshot::LoopSnapshot;
use glam::Vec2;
use iced::widget::{Column, horizontal_rule, row, text};
use iced::{Alignment, Element};
use std::path::Path;
use std::time::Instant;
use tokio::sync::mpsc;

/// Mensajes de la pestaña.
#[derive(Debug, Clone)]
pub enum MensajeRobots {
    Patron { radio: usize, patron: Option<u8> },
    Activo { radio: usize, on: bool },
    Identificar,
    Cancelar,
    Aplicar,
    Descartar,
}

/// La identificación en curso, con la identidad de la que partió.
struct EnCurso {
    maquina: Identificacion,
    inicio: Identidad,
    /// La última orden fue un giro: al pasar a quieto se manda `FinGiro`.
    girando: bool,
}

/// Estado de la pestaña.
pub struct EstadoRobots {
    tx: Option<mpsc::Sender<OrdenIdentidad>>,
    /// La última identidad mandada al lazo, hasta que la foto la traiga: las ediciones
    /// seguidas parten de ella y no de una foto atrasada.
    pendiente: Option<Identidad>,
    /// Ya se guardó una vez en esta corrida (el archivo existe).
    guardado: bool,
    /// Resultado del último cambio, y si va en alerta.
    estado: Option<(String, bool)>,
    en_curso: Option<EnCurso>,
    corte: Option<String>,
    /// El resultado de la última identificación y su propuesta.
    resultado: Option<(Resultado, Identidad, Identidad)>,
}

impl EstadoRobots {
    pub fn new(tx: Option<mpsc::Sender<OrdenIdentidad>>) -> Self {
        Self { tx, pendiente: None, guardado: false, estado: None, en_curso: None, corte: None, resultado: None }
    }

    pub fn identificando(&self) -> bool {
        self.en_curso.is_some()
    }

    fn mandar(&self, orden: OrdenIdentidad) -> Result<(), String> {
        match &self.tx {
            None => Err("La identidad se edita desde main con la GUI.".to_string()),
            Some(tx) => tx.try_send(orden).map_err(|e| e.to_string()),
        }
    }
}

// ── Frases (funciones puras) ─────────────────────────────────────────────────

/// Robots propios a la vista en la foto: activos, con dato reciente y pose finita.
pub fn vistos_con_pose(snap: &LoopSnapshot, own_team: i32) -> Vec<(i32, Vec2, f64)> {
    snap.robots
        .iter()
        .filter(|r| {
            r.team == own_team
                && r.active
                && r.age_s <= NO_VISTO_S
                && r.position.is_finite()
                && r.orientation.is_finite()
        })
        .map(|r| (r.id, r.position, r.orientation))
        .collect()
}

/// La frase de la fila de la radio `radio`.
pub fn frase_fila(radio: usize, identidad: &Identidad, vistos: &[(i32, Vec2)]) -> String {
    let r = identidad.robots[radio];
    if !r.activo {
        return "desactivado: no recibe comandos".to_string();
    }
    match r.patron {
        None => "sin patrón: no recibe comandos".to_string(),
        Some(p) => match vistos.iter().find(|(id, _)| *id == i32::from(p)) {
            Some((_, pos)) => format!("patrón {p} · la cámara lo ve en ({}, {}) m", num(pos.x, 2), num(pos.y, 2)),
            None => format!("patrón {p} · la cámara no lo ve"),
        },
    }
}

/// Qué patrones propios ve la cámara.
pub fn frase_vistos(ids: &[i32]) -> String {
    let mut ids = ids.to_vec();
    ids.sort_unstable();
    ids.dedup();
    match ids.as_slice() {
        [] => "La cámara no ve ningún robot propio.".to_string(),
        [p] => format!("La cámara ve el patrón {p}."),
        _ => format!("La cámara ve los patrones {}.", lista(&ids)),
    }
}

/// De dónde vino la identidad y dónde se guarda.
pub fn frase_origen(ruta: &Path, origen: Origen, guardado: bool) -> String {
    if origen == Origen::Archivo || guardado {
        format!("Identidad de {}; los cambios se guardan ahí.", ruta.display())
    } else {
        format!("Sin archivo en {}: la radio N lleva el patrón N; se crea al guardar.", ruta.display())
    }
}

/// "Radio 2 (MI_ROBOT_ID 3), 2 de 5: girando ida".
pub fn frase_avance(radio: usize, k: usize, total: usize, fase: FaseGiro) -> String {
    format!("{}, {k} de {total}: {}", con_mayuscula(&nombre_radio(radio)), fase.texto())
}

/// Una línea por radio que cambia en la propuesta; vacía si no cambia nada.
pub fn lineas_propuesta(actual: &Identidad, propuesta: &Identidad) -> Vec<String> {
    (0..SLOT_COUNT)
        .filter_map(|r| {
            let (a, b) = (actual.robots[r].patron, propuesta.robots[r].patron);
            let antes = match a {
                Some(p) => format!("tenía el patrón {p}"),
                None => "no tenía patrón".to_string(),
            };
            let despues = match b {
                Some(p) => format!("queda con el {p}"),
                None => "queda sin patrón".to_string(),
            };
            (a != b).then(|| format!("radio {r}: {antes}, {despues}"))
        })
        .collect()
}

// ── Mensajes, identificación y vista ─────────────────────────────────────────

impl DebugGui {
    /// La identidad desde la que se edita: la última mandada si la foto todavía no la
    /// trae, o la de la foto.
    fn identidad_base(&self) -> Option<Identidad> {
        let foto = self.snap.identidad.as_deref();
        match &self.robots.pendiente {
            Some(p) if foto != Some(p) => Some(p.clone()),
            _ => foto.cloned(),
        }
    }

    /// Manda la identidad al lazo y, si salió, la guarda.
    fn aplicar_identidad(&mut self, nueva: Identidad) {
        let ruta = self.run.identidad_ruta.clone();
        self.robots.estado = Some(match self.robots.mandar(OrdenIdentidad::Usar(nueva.clone())) {
            Err(e) => (format!("No se pudo enviar el cambio al lazo: {e} No se guardó."), true),
            Ok(()) => {
                self.robots.pendiente = Some(nueva.clone());
                match nueva.guardar(&ruta) {
                    Ok(()) => {
                        self.robots.guardado = true;
                        (format!("Guardado en {}.", ruta.display()), false)
                    }
                    Err(e) => (
                        format!(
                            "No se pudo guardar en {}: {e}. El cambio rige hasta cerrar el programa.",
                            ruta.display()
                        ),
                        true,
                    ),
                }
            }
        });
    }

    /// Lo que puede cortar (o impedir) la identificación, con el estado de ahora.
    fn condiciones_identificar(&self, cancelar: bool) -> Condiciones {
        let parada = self.estop_activa() || self.snap.estop;
        Condiciones {
            parada,
            halt: self.snap.mode == TickMode::Halt && !parada,
            sin_vision: self.snap.mode == TickMode::SinVision,
            coach_activo: self.coach_activo(),
            rutina: self.snap.rutina.is_some(),
            manual: self.manual_enabled,
            skill_gui: self.active_skill.is_some(),
            cancelar,
        }
    }

    /// Patrones propios a la vista con su orientación (rad).
    fn vistos_theta(&self) -> Vec<(i32, f64)> {
        vistos_con_pose(&self.snap, self.run.own_team).into_iter().map(|(id, _, t)| (id, t)).collect()
    }

    /// Por qué no se puede identificar ahora, o `Ok`.
    pub(super) fn razon_identificar(&self) -> Result<(), String> {
        if self.robots.tx.is_none() {
            return Err("La identidad se edita desde main con la GUI.".to_string());
        }
        let Some(ident) = self.snap.identidad.as_deref() else {
            return Err("Esperando la primera foto del lazo.".to_string());
        };
        puede_identificar(&self.condiciones_identificar(false), ident, &self.vistos_theta())
    }

    /// Avanza la identificación con la foto vigente. Lo llaman cada foto, la parada de
    /// emergencia (botón y tecla) y Cancelar.
    pub(super) fn paso_identificacion(&mut self, ahora: Instant, cancelar: bool) {
        if self.robots.en_curso.is_none() {
            return;
        }
        let cond = self.condiciones_identificar(cancelar);
        let vistos = self.vistos_theta();
        let Some(c) = self.robots.en_curso.as_mut() else { return };
        let paso = c.maquina.paso(ahora, &vistos, &cond);
        let giraba = c.girando;
        c.girando = matches!(paso, Paso::Girar { .. });
        let inicio = c.inicio.clone();
        match paso {
            Paso::Girar { radio, w_deg_s } => {
                let _ = self.robots.mandar(OrdenIdentidad::Giro { radio, w_deg_s });
            }
            Paso::Quieto => {
                if giraba {
                    let _ = self.robots.mandar(OrdenIdentidad::FinGiro);
                }
            }
            Paso::Cortado(frase) => {
                self.robots.en_curso = None;
                self.robots.corte = Some(frase);
                let _ = self.robots.mandar(OrdenIdentidad::FinGiro);
            }
            Paso::Fin(res) => {
                self.robots.en_curso = None;
                if giraba {
                    let _ = self.robots.mandar(OrdenIdentidad::FinGiro);
                }
                let propuesta = res.propuesta(&inicio);
                self.robots.resultado = Some((res, inicio, propuesta));
            }
        }
    }

    /// Con cada foto: suelta la identidad pendiente si la foto ya la trae, y avanza la
    /// identificación.
    pub(super) fn foto_robots(&mut self) {
        if self.robots.pendiente.as_ref().is_some_and(|p| self.snap.identidad.as_deref() == Some(p)) {
            self.robots.pendiente = None;
        }
        self.paso_identificacion(Instant::now(), false);
    }

    /// Atiende un mensaje de la pestaña.
    pub(super) fn mensaje_robots(&mut self, m: MensajeRobots) {
        match m {
            MensajeRobots::Patron { radio, patron } => {
                if self.robots.identificando() {
                    return;
                }
                if let Some(mut i) = self.identidad_base().filter(|_| radio < SLOT_COUNT) {
                    i.robots[radio].patron = patron.filter(|p| *p < PATRONES);
                    self.aplicar_identidad(i);
                }
            }
            MensajeRobots::Activo { radio, on } => {
                if self.robots.identificando() {
                    return;
                }
                if let Some(mut i) = self.identidad_base().filter(|_| radio < SLOT_COUNT) {
                    i.robots[radio].activo = on;
                    self.aplicar_identidad(i);
                }
            }
            MensajeRobots::Identificar => {
                if self.robots.identificando() || self.razon_identificar().is_err() {
                    return;
                }
                let Some(inicio) = self.identidad_base() else { return };
                self.robots.corte = None;
                self.robots.resultado = None;
                self.robots.en_curso = Some(EnCurso {
                    maquina: Identificacion::nueva(&inicio, Instant::now()),
                    inicio,
                    girando: false,
                });
                self.paso_identificacion(Instant::now(), false);
            }
            MensajeRobots::Cancelar => self.paso_identificacion(Instant::now(), true),
            MensajeRobots::Aplicar => {
                if let Some((_, _, propuesta)) = self.robots.resultado.take() {
                    self.aplicar_identidad(propuesta);
                }
            }
            MensajeRobots::Descartar => self.robots.resultado = None,
        }
    }

    /// Pestaña Robots.
    pub(super) fn pestana_robots(&self) -> Element<'_, Message> {
        let r = &self.robots;
        let vistos: Vec<(i32, Vec2)> =
            vistos_con_pose(&self.snap, self.run.own_team).into_iter().map(|(id, p, _)| (id, p)).collect();
        let ids: Vec<i32> = vistos.iter().map(|(id, _)| *id).collect();
        let mut col = Column::new()
            .spacing(theme::SP_SM)
            .push(titulo("Robots"))
            .push(nota(frase_origen(&self.run.identidad_ruta, self.run.identidad_origen, r.guardado)))
            .push(text(frase_vistos(&ids)).size(theme::TXT));
        let Some(ident) = self.identidad_base() else {
            return col.push(nota("Esperando la primera foto del lazo.")).into();
        };
        let editable = r.tx.is_some() && !r.identificando();
        let m = |x: MensajeRobots| Message::Robots(x);
        for radio in 0..SLOT_COUNT {
            let robot = ident.robots[radio];
            let mut patrones = row![etiqueta("patrón")].spacing(theme::SP_XS).align_y(Alignment::Center);
            for p in std::iter::once(None).chain((0..PATRONES).map(Some)) {
                let texto = p.map_or("ninguno".to_string(), |p| p.to_string());
                let msg = editable.then(|| m(MensajeRobots::Patron { radio, patron: p }));
                patrones = patrones.push(opcion_si(texto, robot.patron == p, msg));
            }
            let activo = opcion_si(
                "activo",
                robot.activo,
                editable.then(|| m(MensajeRobots::Activo { radio, on: !robot.activo })),
            );
            col = col
                .push(horizontal_rule(1).style(theme::filete))
                .push(subtitulo(con_mayuscula(&nombre_radio(radio))))
                .push(patrones)
                .push(row![activo].spacing(theme::SP_SM))
                .push(text(frase_fila(radio, &ident, &vistos)).size(theme::TXT_CHICO));
        }
        let avisos = ident.avisos(&ids, self.run.num_robots);
        if !avisos.is_empty() {
            col = col.push(horizontal_rule(1).style(theme::filete));
            for a in avisos {
                col = col.push(text(a).size(theme::TXT_CHICO).style(theme::alerta));
            }
        }
        col = col
            .push(horizontal_rule(1).style(theme::filete))
            .push(subtitulo("Identificar"))
            .push(nota(
                "Con el coach en pausa, gira cada robot activo de a uno (90 °/s, ida y vuelta) y ve qué \
                 patrón giró. Al final propone la identidad; tú la aplicas o la descartas.",
            ));
        match &r.en_curso {
            Some(c) => {
                col = col.push(boton_chico("Cancelar", Some(m(MensajeRobots::Cancelar))));
                if let Some((radio, k, total, fase)) = c.maquina.avance() {
                    col = col.push(text(frase_avance(radio, k, total, fase)).size(theme::TXT));
                }
            }
            None => {
                let razon = self.razon_identificar();
                col = col.push(boton_chico("Identificar", razon.is_ok().then(|| m(MensajeRobots::Identificar))));
                if let Err(e) = razon {
                    col = col.push(text(e).size(theme::TXT_CHICO).style(theme::alerta));
                }
            }
        }
        if let Some(corte) = &r.corte {
            col = col.push(text(corte.clone()).size(theme::TXT_CHICO).style(theme::alerta));
        }
        if let Some((res, inicio, propuesta)) = &r.resultado {
            for f in res.frases() {
                col = col.push(text(f).size(theme::TXT_CHICO));
            }
            let lineas = lineas_propuesta(inicio, propuesta);
            if lineas.is_empty() {
                col = col
                    .push(text("La identidad ya era la correcta.").size(theme::TXT))
                    .push(boton_chico("Descartar", Some(m(MensajeRobots::Descartar))));
            } else {
                col = col.push(subtitulo("Propuesta"));
                for l in lineas {
                    col = col.push(text(l).size(theme::TXT_CHICO));
                }
                col = col.push(
                    row![
                        boton_chico("Aplicar", r.tx.is_some().then(|| m(MensajeRobots::Aplicar))),
                        boton_chico("Descartar", Some(m(MensajeRobots::Descartar))),
                    ]
                    .spacing(theme::SP_SM),
                );
            }
        }
        if let Some((e, alerta)) = &r.estado {
            col = col.push(text(e.clone()).size(theme::TXT_CHICO).style(theme::tinta_o_alerta(*alerta)));
        }
        col.into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::GUI::run_info::run_info_de_prueba;
    use crate::GUI::{GuiSetup, Tab};
    use crate::radio::RadioTarget;
    use crate::snapshot::RobotView;
    use std::path::PathBuf;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::time::Duration;

    fn robot(id: i32, x: f32, y: f32, theta: f64) -> RobotView {
        RobotView {
            team: 0,
            id,
            position: Vec2::new(x, y),
            orientation: theta,
            velocity: Vec2::ZERO,
            angular_velocity: 0.0,
            active: true,
            age_s: 0.0,
        }
    }

    struct Prueba {
        g: DebugGui,
        rx: mpsc::Receiver<OrdenIdentidad>,
        pausa: Arc<AtomicBool>,
        dir: PathBuf,
    }

    impl Drop for Prueba {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.dir);
        }
    }

    /// GUI con la base station, el coach en pausa y la identidad guardándose en un
    /// directorio temporal.
    fn gui(nombre: &str) -> Prueba {
        let dir = std::env::temp_dir().join(format!("vsss_gui_robots_{nombre}_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let (_fotos, snapshot_rx) = crate::snapshot::snapshot_channel();
        let (tx, rx) = mpsc::channel(256);
        let pausa = Arc::new(AtomicBool::new(true));
        let mut run = run_info_de_prueba();
        run.transport = RadioTarget::BaseStation;
        run.identidad_ruta = dir.join("identidad.json");
        let setup = GuiSetup {
            snapshot_rx,
            manual_tx: None,
            skill_tx: None,
            pid_tx: None,
            teleport_tx: None,
            rutina_tx: None,
            identidad_tx: Some(tx),
            estop: Arc::new(AtomicBool::new(false)),
            coach_pausa: Some(pausa.clone()),
            run,
        };
        let mut g = DebugGui::new(setup).0;
        g.snap = Arc::new(foto(Identidad::default(), &[(0, 0.0), (1, 0.0)]));
        Prueba { g, rx, pausa, dir }
    }

    fn foto(ident: Identidad, robots: &[(i32, f64)]) -> LoopSnapshot {
        LoopSnapshot {
            robots: robots.iter().map(|(id, t)| robot(*id, 0.1 * *id as f32, -0.3, *t)).collect(),
            identidad: Some(Arc::new(ident)),
            ..LoopSnapshot::default()
        }
    }

    fn ordenes(rx: &mut mpsc::Receiver<OrdenIdentidad>) -> Vec<OrdenIdentidad> {
        std::iter::from_fn(|| rx.try_recv().ok()).collect()
    }

    #[test]
    fn frase_fila_visto() {
        let vistos = [(0, Vec2::new(0.12, -0.30))];
        let mut ident = Identidad::default();
        ident.robots[1].patron = Some(0);
        ident.robots[0].patron = Some(1);
        assert_eq!(frase_fila(1, &ident, &vistos), "patrón 0 · la cámara lo ve en (0.12, \u{2212}0.30) m");
    }

    #[test]
    fn frase_fila_no_visto() {
        assert_eq!(frase_fila(2, &Identidad::default(), &[]), "patrón 2 · la cámara no lo ve");
    }

    #[test]
    fn frase_fila_sin_patron() {
        assert_eq!(frase_fila(4, &Identidad::default(), &[]), "sin patrón: no recibe comandos");
    }

    #[test]
    fn frase_fila_desactivado() {
        let mut ident = Identidad::default();
        ident.robots[2].activo = false;
        assert_eq!(frase_fila(2, &ident, &[(2, Vec2::ZERO)]), "desactivado: no recibe comandos");
    }

    #[test]
    fn frases_de_vistos() {
        assert_eq!(frase_vistos(&[]), "La cámara no ve ningún robot propio.");
        assert_eq!(frase_vistos(&[2]), "La cámara ve el patrón 2.");
        assert_eq!(frase_vistos(&[3, 0, 1]), "La cámara ve los patrones 0, 1 y 3.");
    }

    #[test]
    fn frase_origen_sin_archivo() {
        let ruta = Path::new("config/identidad_robots.json");
        assert_eq!(
            frase_origen(ruta, Origen::PorDefecto, false),
            "Sin archivo en config/identidad_robots.json: la radio N lleva el patrón N; se crea al guardar."
        );
        assert!(frase_origen(ruta, Origen::PorDefecto, true).starts_with("Identidad de config/identidad_robots.json"));
    }

    #[test]
    fn frase_origen_con_variable() {
        let ruta = Path::new("/tmp/engine_b/identidad.json");
        assert_eq!(
            frase_origen(ruta, Origen::Archivo, false),
            "Identidad de /tmp/engine_b/identidad.json; los cambios se guardan ahí."
        );
    }

    #[test]
    fn avance_en_frase() {
        assert_eq!(frase_avance(2, 2, 5, FaseGiro::Ida), "Radio 2 (MI_ROBOT_ID 3), 2 de 5: girando ida");
    }

    #[test]
    fn lineas_propuesta_solo_cambios() {
        let actual = Identidad::default();
        let mut p = actual.clone();
        p.robots[0].patron = Some(1);
        p.robots[1].patron = Some(0);
        p.robots[3].patron = None;
        p.robots[4].patron = Some(3);
        assert_eq!(
            lineas_propuesta(&actual, &p),
            vec![
                "radio 0: tenía el patrón 0, queda con el 1",
                "radio 1: tenía el patrón 1, queda con el 0",
                "radio 3: tenía el patrón 3, queda sin patrón",
                "radio 4: no tenía patrón, queda con el 3",
            ]
        );
    }

    #[test]
    fn propuesta_igual_dice_que_ya_era_correcta() {
        assert!(lineas_propuesta(&Identidad::default(), &Identidad::default()).is_empty());
    }

    #[test]
    fn editar_patron_manda_usar_y_guarda_en_la_ruta() {
        let mut p = gui("editar");
        let _ = p.g.update(Message::Robots(MensajeRobots::Patron { radio: 0, patron: Some(2) }));
        let mut esperada = Identidad::default();
        esperada.robots[0].patron = Some(2);
        assert_eq!(ordenes(&mut p.rx), vec![OrdenIdentidad::Usar(esperada.clone())]);
        let ruta = p.dir.join("identidad.json");
        assert_eq!(crate::identidad::cargar(&ruta).unwrap(), Some(esperada.clone()));
        assert!(p.g.robots.estado.as_ref().is_some_and(|(e, alerta)| e.contains("Guardado en") && !alerta));
        // Un segundo cambio antes de que llegue la foto parte del primero.
        let _ = p.g.update(Message::Robots(MensajeRobots::Activo { radio: 4, on: false }));
        esperada.robots[4].activo = false;
        assert_eq!(ordenes(&mut p.rx), vec![OrdenIdentidad::Usar(esperada.clone())]);
        assert_eq!(crate::identidad::cargar(&ruta).unwrap(), Some(esperada));
    }

    #[test]
    fn editar_bloqueado_mientras_identifica() {
        let mut p = gui("bloqueado");
        let _ = p.g.update(Message::Robots(MensajeRobots::Identificar));
        assert!(p.g.robots.identificando());
        ordenes(&mut p.rx);
        let _ = p.g.update(Message::Robots(MensajeRobots::Patron { radio: 0, patron: None }));
        let _ = p.g.update(Message::Robots(MensajeRobots::Activo { radio: 0, on: false }));
        assert!(ordenes(&mut p.rx).iter().all(|o| !matches!(o, OrdenIdentidad::Usar(_))));
    }

    #[test]
    fn identificar_manda_giros_y_fin_giro() {
        let mut p = gui("identificar");
        // Solo la radio 0 activa: el patrón 1 es el que gira con ella.
        let mut ident = Identidad::default();
        for r in 1..SLOT_COUNT {
            ident.robots[r].activo = false;
        }
        p.g.snap = Arc::new(foto(ident.clone(), &[(0, 0.0), (1, 0.0)]));
        let _ = p.g.update(Message::Robots(MensajeRobots::Identificar));
        let t0 = Instant::now();
        let mut vistas: Vec<OrdenIdentidad> = Vec::new();
        for ms in (0..2600u64).step_by(20) {
            // El patrón 1 gira con la consigna, nominal (sube hasta 54° y vuelve).
            let s = ms as f64 / 1000.0;
            let theta = if (0.5..1.1).contains(&s) {
                (s - 0.5) * 90.0
            } else if (1.1..1.7).contains(&s) {
                54.0 - (s - 1.1) * 90.0
            } else {
                0.0
            };
            p.g.snap = Arc::new(foto(ident.clone(), &[(0, 0.0), (1, theta.to_radians())]));
            p.g.paso_identificacion(t0 + Duration::from_millis(ms + 10), false);
            vistas.extend(ordenes(&mut p.rx));
        }
        assert!(vistas.contains(&OrdenIdentidad::Giro { radio: 0, w_deg_s: 90 }));
        assert!(vistas.contains(&OrdenIdentidad::Giro { radio: 0, w_deg_s: -90 }));
        assert_eq!(vistas.last(), Some(&OrdenIdentidad::FinGiro));
        assert!(!p.g.robots.identificando());
        let (_, inicio, propuesta) = p.g.robots.resultado.clone().expect("terminó");
        assert_eq!(propuesta.robots[0].patron, Some(1));
        assert_eq!(lineas_propuesta(&inicio, &propuesta)[0], "radio 0: tenía el patrón 0, queda con el 1");
        // Aplicar manda la propuesta y la guarda.
        let _ = p.g.update(Message::Robots(MensajeRobots::Aplicar));
        assert_eq!(ordenes(&mut p.rx), vec![OrdenIdentidad::Usar(propuesta.clone())]);
        assert_eq!(crate::identidad::cargar(&p.dir.join("identidad.json")).unwrap(), Some(propuesta));
    }

    #[test]
    fn parada_corta_identificar_en_la_gui() {
        let mut p = gui("parada");
        let _ = p.g.update(Message::Robots(MensajeRobots::Identificar));
        let t0 = Instant::now();
        // Hasta la ida de la primera radio.
        p.g.paso_identificacion(t0 + Duration::from_millis(600), false);
        p.g.paso_identificacion(t0 + Duration::from_millis(700), false);
        assert!(ordenes(&mut p.rx).iter().any(|o| matches!(o, OrdenIdentidad::Giro { .. })));
        let _ = p.g.update(Message::ToggleEstop);
        assert!(p.g.estop.load(Ordering::Relaxed));
        assert_eq!(ordenes(&mut p.rx), vec![OrdenIdentidad::FinGiro]);
        assert!(!p.g.robots.identificando());
        assert!(p.g.robots.corte.as_deref().is_some_and(|c| c.contains("parada de emergencia")));
        // Ya no gira más, aunque lleguen fotos.
        let _ = p.g.update(Message::Snapshot(Arc::new(foto(Identidad::default(), &[(0, 0.0)]))));
        assert!(ordenes(&mut p.rx).is_empty());
    }

    #[test]
    fn boton_deshabilitado_con_coach_activo() {
        let p = gui("coach");
        assert_eq!(p.g.razon_identificar(), Ok(()));
        p.pausa.store(false, Ordering::Relaxed);
        assert_eq!(
            p.g.razon_identificar(),
            Err("Pon el coach en pausa (P): con el coach activo, los robots se mueven.".to_string())
        );
    }

    #[test]
    fn la_vista_se_arma_en_todos_los_estados() {
        let mut p = gui("vista");
        p.g.tab = Tab::Robots;
        let _ = p.g.view();
        // Con avisos, identificando, con un corte, con un resultado y sin foto.
        let mut ident = Identidad::default();
        ident.robots[2].patron = Some(0);
        p.g.snap = Arc::new(foto(ident, &[(0, 0.0), (3, 0.0), (2, 0.1)]));
        let _ = p.g.pestana_robots();
        let _ = p.g.update(Message::Robots(MensajeRobots::Identificar));
        let _ = p.g.pestana_robots();
        let _ = p.g.update(Message::Robots(MensajeRobots::Cancelar));
        assert!(p.g.robots.corte.is_some());
        let _ = p.g.pestana_robots();
        p.g.robots.resultado = Some((
            Resultado::nuevo_para_tests(),
            Identidad::default(),
            Identidad::default(),
        ));
        let _ = p.g.pestana_robots();
        p.g.snap = Arc::new(LoopSnapshot::default());
        let _ = p.g.pestana_robots();
        let _ = p.g.view();
    }

    #[test]
    fn la_pestana_solo_con_base_station() {
        assert!(Tab::visibles(true).contains(&Tab::Robots));
        assert!(!Tab::visibles(false).contains(&Tab::Robots));
    }
}
