//! `RoutineDecider`: una corrida de una rutina, como un decisor más del lazo común (D1 y
//! D4–D10 del design).
//!
//! Solo elige skill y objetivo por robot y por tick (GoTo, FacePoint, Spin y Hold en la
//! fase 1); navegación, guardia, recuperación de atasco, HALT, parada, corte sin visión,
//! CSV y capas siguen siendo los del lazo. Las causas de aborto que dependen del lazo
//! (HALT, parada, STOP, corte, manual, teleport de la GUI) llegan por `observar`, que el
//! lazo llama al final de cada tick, también cuando no consulta al decisor.
//!
//! El reloj de la rutina es `tick · CONTROL_DT`: determinista en los tests (el CSV tiene
//! el `t_ms` de pared).

use super::asignacion::{Eleccion, asignar};
use super::formato::{Accion, Duracion, Paso, Pose, Repeticiones, Rutina, distancia_a_recorrido, distancia_a_tramo};
use super::resumen::{Resultado, ResumenCorrida, Vuelta, VueltaPuesto};
use super::vista::{
    CAUSA_DETENIDA, CAUSA_USUARIO, Falta, Fase, RobotEnRutina, RutinaVista, frase_cierre, frase_ejecucion,
    frase_pausa, frase_preparacion, segundos, verbo, vueltas,
};
use crate::coach::SkillChoice;
use crate::control_loop::{ContextoTick, PerfilVelocidad, TickDecider, TickMode, sin_vision};
use crate::motion::{CONTROL_DT, Motion, MotionConfig};
use crate::params::{RutinasParams, params};
use crate::radio::TeleportItem;
use crate::skills::{SkillConfig, SkillId};
use crate::world::{RobotState, World};
use glam::Vec2;
use std::collections::{HashSet, VecDeque};
use std::sync::Arc;
use std::time::{Duration, SystemTime};

/// Distancia del punto que mira FacePoint para orientar a un ángulo (m): lejos, el
/// desvío por estar a unos cm del destino es despreciable.
const MIRA_LEJOS_M: f32 = 10.0;
/// Lugares fuera de la cancha donde el teleport de la preparación deja a los robots sin
/// puesto (y = 0.90, separados 0.20 m) y a la pelota (y = −0.90), como el perfil de sysid.
const PARQUE_Y: f64 = 0.90;
const PARQUE_X0: f64 = -0.60;
const PARQUE_DX: f64 = 0.20;

/// Frase antes de asignar robots: la rutina espera el juego y la visión.
const FRASE_ESPERA: &str = "Preparando: esperando la visión de los robots propios y el juego";

/// Porcentaje de velocidad válido: de 20 a 100 en pasos de 10.
pub fn velocidad_valida(pct: u32) -> bool {
    (20..=100).contains(&pct) && pct.is_multiple_of(10)
}

/// Opciones de una corrida.
#[derive(Debug, Clone, PartialEq)]
pub struct OpcionesCorrida {
    pub eleccion: Eleccion,
    pub velocidad_pct: u32,
    pub repeticiones: Repeticiones,
    /// Teleport de la preparación (solo en simulación).
    pub teletransportar: bool,
    pub simulacion: bool,
    /// Arranca sola al quedar lista (consola).
    pub inicio_automatico: bool,
}

impl OpcionesCorrida {
    /// Robots automáticos, la velocidad por defecto del transporte, las repeticiones del
    /// archivo, sin teleport ni inicio automático.
    pub fn por_defecto(rutina: &Rutina, simulacion: bool) -> Self {
        let p = &params().rutinas;
        Self {
            eleccion: Eleccion::Automatica,
            velocidad_pct: if simulacion { p.velocidad_sim_pct } else { p.velocidad_real_pct },
            repeticiones: rutina.repeticiones,
            teletransportar: false,
            simulacion,
            inicio_automatico: false,
        }
    }
}

/// Lo que el decisor toma de afuera; `vigente` lo arma con los parámetros del proceso.
#[derive(Debug, Clone)]
pub struct Entorno {
    pub own_team: i32,
    pub num_robots: usize,
    pub keeper_id: i32,
    pub bidireccional: bool,
    pub blind_stop_s: f64,
    /// Giro base de Spin (`SkillConfig::default().spin_omega`).
    pub spin_omega: f64,
    pub params: RutinasParams,
}

impl Entorno {
    pub fn vigente(own_team: i32, num_robots: usize) -> Self {
        let p = params();
        Self {
            own_team,
            num_robots,
            keeper_id: p.coach.keeper_id,
            bidireccional: MotionConfig::from_env().bidirectional,
            blind_stop_s: p.vision.blind_stop_s,
            spin_omega: SkillConfig::default().spin_omega,
            params: p.rutinas.clone(),
        }
    }
}

/// Órdenes del usuario sobre una corrida (la de preparar es crearla).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OrdenCorrida {
    Iniciar,
    Pausar,
    Reanudar,
    Abortar,
}

/// Estado de un puesto en la vuelta en curso.
#[derive(Debug, Clone)]
struct EnCurso {
    paso: usize,
    /// Inicio del paso (s, corrido por las pausas).
    t_paso: f64,
    /// `ir` con orientación: ya llegó y se orienta.
    orientando: bool,
    /// `girar`: giro acumulado (rad) y la última orientación vista.
    giro: f64,
    theta_prev: Option<f64>,
    /// Terminó sus pasos de esta vuelta y espera a los demás.
    hecho: bool,
    err_suma: f64,
    err_n: u32,
    err_max: f64,
    escapes: u32,
    guardia: u32,
}

impl EnCurso {
    fn nuevo(t: f64) -> Self {
        Self {
            paso: 0,
            t_paso: t,
            orientando: false,
            giro: 0.0,
            theta_prev: None,
            hecho: false,
            err_suma: 0.0,
            err_n: 0,
            err_max: 0.0,
            escapes: 0,
            guardia: 0,
        }
    }

    fn siguiente_paso(&mut self, t: f64, total: usize) {
        self.paso += 1;
        self.t_paso = t;
        self.orientando = false;
        self.giro = 0.0;
        self.theta_prev = None;
        self.hecho = self.paso >= total;
    }
}

/// Resultado de evaluar el paso en curso de un puesto.
enum Evaluacion {
    Sigue(SkillChoice),
    Termina,
}

/// Una corrida de una rutina.
pub struct RoutineDecider {
    rutina: Arc<Rutina>,
    op: OpcionesCorrida,
    ent: Entorno,
    escala: f64,
    /// Recorrido ideal de cada puesto (con la vuelta si hay más de una repetición).
    recorridos: Vec<Vec<Vec2>>,
    fase: Fase,
    /// Robot de cada puesto en uso; vacío hasta asignar.
    asignados: Vec<i32>,
    /// Reloj de la rutina en el último tick visto (s).
    t: f64,
    t_prep: Option<f64>,
    ventanas: Vec<VecDeque<(Vec2, f64)>>,
    falta: Option<Falta>,
    teleport: Vec<TeleportItem>,
    en_curso: Vec<EnCurso>,
    repeticion: u32,
    t_vuelta: f64,
    pausa_desde: Option<f64>,
    pausa_en_vuelta: f64,
    preparacion_s: Option<f64>,
    vueltas: Vec<Vuelta>,
    /// Último modo del lazo (por `observar`).
    modo: TickMode,
    previo_escape: HashSet<i32>,
    previo_guardia: HashSet<i32>,
    causa: Option<String>,
    fase_aborto: Option<Fase>,
    resumen: Option<Arc<ResumenCorrida>>,
    vista: Arc<RutinaVista>,
    /// Tests del conmutador: entra en pánico en el próximo `decide` u `observar`.
    #[cfg(test)]
    pub(crate) panico: bool,
}

fn choice(robot_id: i32, skill_id: SkillId, target: Vec2) -> SkillChoice {
    SkillChoice { robot_id, skill_id, target }
}

/// Error de heading como lo calcula motion: normalizado y, en bidireccional, plegado.
fn error_heading(objetivo: f64, actual: f64, bidireccional: bool) -> f64 {
    let e = Motion::normalize_angle(objetivo - actual);
    if bidireccional { Motion::fold_bidirectional(e) } else { e }
}

/// Punto lejano en la dirección `theta` desde `p`, para orientar con FacePoint.
fn mira_hacia(p: Vec2, theta: f64) -> Vec2 {
    p + MIRA_LEJOS_M * Vec2::new(theta.cos() as f32, theta.sin() as f32)
}

/// Pose media de un tramo de la ventana (orientación por media circular).
fn media<'a>(muestras: impl Iterator<Item = &'a (Vec2, f64)>) -> (Vec2, f64) {
    let (mut p, mut s, mut c, mut n) = (Vec2::ZERO, 0.0, 0.0, 0.0_f32);
    for (q, th) in muestras {
        p += *q;
        s += th.sin();
        c += th.cos();
        n += 1.0;
    }
    (p / n.max(1.0), f64::atan2(s, c))
}

/// Causa de aborto de un paso que superó su tiempo máximo `max` (s).
fn frase_fuera_de_tiempo(robot: i32, i: usize, paso: &Paso, max: f64, orientando: bool) -> String {
    let (n, t) = (i + 1, segundos(max));
    match &paso.accion {
        Accion::Ir { .. } if orientando => format!("el robot {robot} no se orientó en el punto {n} en {t} s"),
        Accion::Ir { .. } => format!("el robot {robot} no llegó al punto {n} en {t} s"),
        Accion::Mirar { .. } => format!("el robot {robot} no se orientó al punto {n} en {t} s"),
        Accion::Girar { duracion: Duracion::Vueltas(v), .. } => {
            format!("el robot {robot} no completó {} en {t} s", vueltas(*v))
        }
        otro => format!("el robot {robot} no terminó el paso {n} ({}) en {t} s", otro.tipo()),
    }
}

/// Cómo está un robot respecto de su inicio.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Quietud {
    FueraDePosicion,
    Moviendose,
    Lista,
}

impl RoutineDecider {
    pub fn new(rutina: Arc<Rutina>, op: OpcionesCorrida, ent: Entorno) -> Self {
        let escala = f64::from(op.velocidad_pct.clamp(20, 100)) / 100.0;
        let vuelta = op.repeticiones.mas_de_una();
        let recorridos = rutina.puestos.iter().map(|p| p.recorrido(vuelta)).collect();
        let vista = Arc::new(RutinaVista {
            rutina: rutina.clone(),
            fase: Fase::Preparando,
            frase: "Preparando".to_string(),
            velocidad_pct: op.velocidad_pct,
            repeticion: 0,
            repeticiones: op.repeticiones,
            robots: Vec::new(),
            resumen: None,
        });
        Self {
            rutina,
            op,
            ent,
            escala,
            recorridos,
            fase: Fase::Preparando,
            asignados: Vec::new(),
            t: 0.0,
            t_prep: None,
            ventanas: Vec::new(),
            falta: None,
            teleport: Vec::new(),
            en_curso: Vec::new(),
            repeticion: 0,
            t_vuelta: 0.0,
            pausa_desde: None,
            pausa_en_vuelta: 0.0,
            preparacion_s: None,
            vueltas: Vec::new(),
            modo: TickMode::Play,
            previo_escape: HashSet::new(),
            previo_guardia: HashSet::new(),
            causa: None,
            fase_aborto: None,
            resumen: None,
            vista,
            #[cfg(test)]
            panico: false,
        }
    }

    /// Con los parámetros vigentes del proceso.
    pub fn vigente(rutina: Arc<Rutina>, op: OpcionesCorrida, own_team: i32, num_robots: usize) -> Self {
        Self::new(rutina, op, Entorno::vigente(own_team, num_robots))
    }

    pub fn fase(&self) -> Fase {
        self.fase
    }

    pub fn causa(&self) -> Option<&str> {
        self.causa.as_deref()
    }

    pub fn resumen(&self) -> Option<&Arc<ResumenCorrida>> {
        self.resumen.as_ref()
    }

    /// Robot de cada puesto en uso (vacío antes de asignar).
    pub fn asignados(&self) -> &[i32] {
        &self.asignados
    }

    pub fn vista(&self) -> &Arc<RutinaVista> {
        &self.vista
    }

    fn p(&self) -> &RutinasParams {
        &self.ent.params
    }

    // ─── Órdenes ─────────────────────────────────────────────────────────────

    /// Aplica una orden del usuario. Las que no corresponden a la fase se ignoran.
    pub fn ordenar(&mut self, orden: OrdenCorrida) {
        match (orden, self.fase) {
            (OrdenCorrida::Iniciar, Fase::Lista) => self.iniciar(),
            (OrdenCorrida::Pausar, Fase::Ejecutando) => {
                self.fase = Fase::Pausada;
                self.pausa_desde = Some(self.t);
            }
            (OrdenCorrida::Reanudar, Fase::Pausada) => {
                let d = self.pausa_desde.take().map_or(0.0, |t0| (self.t - t0).max(0.0));
                for e in &mut self.en_curso {
                    e.t_paso += d;
                    e.theta_prev = None;
                }
                self.pausa_en_vuelta += d;
                self.fase = Fase::Ejecutando;
            }
            (OrdenCorrida::Abortar, f) if f.activa() => {
                if self.op.repeticiones == Repeticiones::Bucle {
                    self.cerrar(Resultado::Detenida, Some(CAUSA_DETENIDA.to_string()));
                } else {
                    self.cerrar(Resultado::Abortada, Some(CAUSA_USUARIO.to_string()));
                }
            }
            _ => {}
        }
        self.refrescar_vista();
    }

    /// Aborta con una causa propia (p. ej. el tope de `--dur` de la consola).
    pub fn abortar(&mut self, causa: &str) {
        self.cerrar(Resultado::Abortada, Some(causa.to_string()));
        self.refrescar_vista();
    }

    fn iniciar(&mut self) {
        self.fase = Fase::Ejecutando;
        self.preparacion_s = self.t_prep.map(|t0| self.t - t0);
        self.repeticion = 1;
        self.t_vuelta = self.t;
        self.pausa_en_vuelta = 0.0;
        self.en_curso = (0..self.asignados.len()).map(|_| EnCurso::nuevo(self.t)).collect();
    }

    /// Termina la corrida: Hold a todos, la causa y el resumen. Sin efecto si ya terminó.
    fn cerrar(&mut self, resultado: Resultado, causa: Option<String>) {
        if !self.fase.activa() {
            return;
        }
        if resultado != Resultado::Completada {
            self.fase_aborto = Some(self.fase);
        }
        self.fase = if resultado == Resultado::Completada { Fase::Terminada } else { Fase::Abortada };
        self.causa = causa;
        self.resumen = Some(Arc::new(ResumenCorrida {
            rutina: self.rutina.id.clone(),
            nombre: self.rutina.nombre.clone(),
            velocidad_pct: self.op.velocidad_pct,
            repeticiones: self.op.repeticiones,
            asignacion: self
                .asignados
                .iter()
                .enumerate()
                .map(|(k, r)| (self.rutina.puestos[k].nombre.clone(), *r))
                .collect(),
            resultado,
            causa: self.causa.clone(),
            fase_aborto: self.fase_aborto,
            preparacion_s: self.preparacion_s,
            vueltas: self.vueltas.clone(),
        }));
    }

    fn abortar_por(&mut self, causa: String) {
        self.cerrar(Resultado::Abortada, Some(causa));
    }

    // ─── Tick ────────────────────────────────────────────────────────────────

    /// `decide` con el reloj de pared explícito (para los tests).
    pub fn decidir(&mut self, tick: u32, world: &World, ahora: SystemTime) -> Vec<SkillChoice> {
        #[cfg(test)]
        assert!(!self.panico, "pánico de prueba en decide");
        self.t = f64::from(tick) * CONTROL_DT;
        let mut out = self.decidir_fase(world, ahora);
        self.completar_con_hold(&mut out, world);
        self.refrescar_vista();
        out
    }

    fn decidir_fase(&mut self, world: &World, ahora: SystemTime) -> Vec<SkillChoice> {
        if self.fase.activa()
            && self.asignados.is_empty()
            && let Err(e) = self.preparar_asignacion(world)
        {
            self.abortar_por(format!("no se pudo preparar: {e}"));
        }
        if self.fase.activa()
            && let Some(c) = self.chequeo_vivo(world, ahora)
        {
            self.abortar_por(c);
        }
        match self.fase {
            Fase::Preparando | Fase::Lista => self.preparar(world),
            Fase::Ejecutando => self.ejecutar(world),
            Fase::Pausada | Fase::Terminada | Fase::Abortada => Vec::new(),
        }
    }

    /// Robots propios activos con id < `num_robots`, ordenados por id.
    fn propios(&self, world: &World) -> Vec<RobotState> {
        let todos = if self.ent.own_team == 0 {
            world.get_blue_team_active()
        } else {
            world.get_yellow_team_active()
        };
        let mut v: Vec<RobotState> = todos
            .into_iter()
            .filter(|r| r.id >= 0 && (r.id as usize) < self.ent.num_robots)
            .cloned()
            .collect();
        v.sort_by_key(|r| r.id);
        v
    }

    /// El robot `id` visto y con un dato reciente.
    fn robot(&self, world: &World, id: i32, ahora: SystemTime) -> Option<RobotState> {
        let umbral = Duration::from_secs_f64(self.ent.blind_stop_s);
        world
            .get_robot_state(id, self.ent.own_team)
            .filter(|r| r.active && !sin_vision(Some(r.last_update), ahora, umbral))
            .cloned()
    }

    /// Hold para cada robot propio visto sin elección.
    fn completar_con_hold(&self, out: &mut Vec<SkillChoice>, world: &World) {
        for r in self.propios(world) {
            if !out.iter().any(|c| c.robot_id == r.id) {
                out.push(choice(r.id, SkillId::Hold, Vec2::ZERO));
            }
        }
    }

    fn preparar_asignacion(&mut self, world: &World) -> Result<(), String> {
        let vistos: Vec<(i32, Vec2)> = self.propios(world).iter().map(|r| (r.id, r.position)).collect();
        self.asignados = asignar(&self.rutina, &vistos, self.ent.keeper_id, &self.op.eleccion)?;
        self.t_prep = Some(self.t);
        self.ventanas = vec![VecDeque::new(); self.asignados.len()];
        if self.op.teletransportar && self.op.simulacion {
            self.teleport = self.armar_teleport(world);
        }
        Ok(())
    }

    /// Asignados a su inicio; los demás robots (propios y rivales) y la pelota, fuera de
    /// la cancha. `theta` en grados, como espera el replacement del simulador.
    fn armar_teleport(&self, world: &World) -> Vec<TeleportItem> {
        let team = self.ent.own_team as u32;
        let mut items: Vec<TeleportItem> = self
            .asignados
            .iter()
            .enumerate()
            .map(|(k, id)| {
                let ini = self.rutina.puestos[k].inicio;
                TeleportItem::Robot {
                    team,
                    id: *id as u32,
                    x: f64::from(ini.pos.x),
                    y: f64::from(ini.pos.y),
                    theta: ini.orientacion.to_degrees(),
                }
            })
            .collect();
        let mut otros: Vec<(i32, i32)> = world
            .get_blue_team_active()
            .into_iter()
            .chain(world.get_yellow_team_active())
            .map(|r| (r.team, r.id))
            .filter(|(t, id)| !(*t == self.ent.own_team && self.asignados.contains(id)))
            .collect();
        otros.sort();
        for (j, (t, id)) in otros.into_iter().enumerate() {
            items.push(TeleportItem::Robot {
                team: t as u32,
                id: id as u32,
                x: PARQUE_X0 + PARQUE_DX * j as f64,
                y: PARQUE_Y,
                theta: 0.0,
            });
        }
        items.push(TeleportItem::Ball { x: 0.0, y: -PARQUE_Y, vx: 0.0, vy: 0.0 });
        items
    }

    /// Causas de aborto que se ven en el `World`: un asignado sin visión, o dos propios a
    /// menos de `distancia_choque_m`.
    fn chequeo_vivo(&self, world: &World, ahora: SystemTime) -> Option<String> {
        for &id in &self.asignados {
            if self.robot(world, id, ahora).is_none() {
                return Some(format!("el robot {id} dejó de verse"));
            }
        }
        let mut propios: Vec<&RobotState> = if self.ent.own_team == 0 {
            world.get_blue_team_active()
        } else {
            world.get_yellow_team_active()
        };
        propios.sort_by_key(|r| r.id);
        for (i, a) in propios.iter().enumerate() {
            for b in &propios[i + 1..] {
                let d = (a.position - b.position).length();
                if d < self.p().distancia_choque_m {
                    return Some(format!("los robots {} y {} quedaron a {d:.2} m", a.id, b.id));
                }
            }
        }
        None
    }

    // ─── Preparación ─────────────────────────────────────────────────────────

    fn largo_ventana(&self) -> usize {
        ((self.p().quieto_s / CONTROL_DT).round() as usize).max(2)
    }

    /// Posición y quietud de un robot respecto de su inicio (D6).
    fn quietud(&self, ventana: &VecDeque<(Vec2, f64)>, inicio: Pose) -> Quietud {
        let p = self.p();
        let n = self.largo_ventana();
        let (tol, tol_ang) = (p.tolerancia_m, p.tolerancia_deg.to_radians());
        let en_posicion = |pos: Vec2, th: f64| {
            (pos - inicio.pos).length() <= tol
                && error_heading(inicio.orientacion, th, self.ent.bidireccional).abs() <= tol_ang
        };
        if ventana.len() < n {
            return match ventana.back() {
                Some((pos, th)) if en_posicion(*pos, *th) => Quietud::Moviendose,
                _ => Quietud::FueraDePosicion,
            };
        }
        let h = n / 2;
        let (p1, a1) = media(ventana.iter().take(h));
        let (p2, a2) = media(ventana.iter().skip(n - h));
        if !en_posicion(p2, a2) {
            return Quietud::FueraDePosicion;
        }
        let quieto = (p2 - p1).length() <= p.quieto_m
            && Motion::normalize_angle(a2 - a1).abs() <= p.quieto_deg.to_radians();
        if quieto { Quietud::Lista } else { Quietud::Moviendose }
    }

    /// Un robot sin puesto a menos de `distancia_choque_m + tolerancia_m` del recorrido.
    fn cerca_del_recorrido(&self, world: &World) -> Option<Falta> {
        let umbral = self.p().distancia_choque_m + self.p().tolerancia_m;
        for r in self.propios(world) {
            if self.asignados.contains(&r.id) {
                continue;
            }
            for k in 0..self.asignados.len() {
                let d = distancia_a_recorrido(r.position, &self.recorridos[k]);
                if d < umbral {
                    return Some(Falta::CercaDelRecorrido {
                        robot: r.id,
                        distancia: d,
                        puesto: self.rutina.puestos[k].nombre.clone(),
                    });
                }
            }
        }
        None
    }

    fn preparar(&mut self, world: &World) -> Vec<SkillChoice> {
        let n = self.largo_ventana();
        let tol = self.p().tolerancia_m;
        let mut out = Vec::new();
        let mut falta: Option<Falta> = None;
        for k in 0..self.asignados.len() {
            let id = self.asignados[k];
            let Some(r) = world.get_robot_state(id, self.ent.own_team).filter(|r| r.active) else { continue };
            let puesto = &self.rutina.puestos[k];
            let ventana = &mut self.ventanas[k];
            ventana.push_back((r.position, r.orientation));
            while ventana.len() > n {
                ventana.pop_front();
            }
            if (r.position - puesto.inicio.pos).length() > tol {
                out.push(choice(id, SkillId::GoTo, puesto.inicio.pos));
                falta.get_or_insert(Falta::VaASuPuesto { robot: id, puesto: puesto.nombre.clone() });
                continue;
            }
            out.push(choice(id, SkillId::FacePoint, mira_hacia(puesto.inicio.pos, puesto.inicio.orientacion)));
            match self.quietud(&self.ventanas[k], puesto.inicio) {
                Quietud::Lista => {}
                Quietud::FueraDePosicion => {
                    falta.get_or_insert(Falta::SeOrienta { robot: id });
                }
                Quietud::Moviendose => {
                    falta.get_or_insert(Falta::Quieto { robot: id });
                }
            }
        }
        if falta.is_none() {
            falta = self.cerca_del_recorrido(world);
        }
        let lista = falta.is_none() && self.modo == TickMode::Play;
        self.falta = falta;
        match (self.fase, lista) {
            (Fase::Preparando, true) => {
                self.fase = Fase::Lista;
                if self.op.inicio_automatico {
                    self.iniciar();
                    return self.ejecutar(world);
                }
            }
            (Fase::Lista, false) => {
                // Se salió: vuelve a preparar, con el tiempo máximo desde ahora.
                self.fase = Fase::Preparando;
                self.t_prep = Some(self.t);
            }
            _ => {}
        }
        if self.fase == Fase::Preparando
            && let Some(t0) = self.t_prep
        {
            let max = self.p().preparacion_max_s / self.escala;
            if self.t - t0 > max {
                let causa = self.falta.as_ref().map_or_else(
                    || format!("no quedó lista en {} s", segundos(max)),
                    |f| f.vencida(max),
                );
                self.abortar_por(causa);
                return Vec::new();
            }
        }
        out
    }

    // ─── Ejecución ───────────────────────────────────────────────────────────

    fn ejecutar(&mut self, world: &World) -> Vec<SkillChoice> {
        let mut out = Vec::new();
        // A lo sumo un cierre de vuelta por tick.
        for _ in 0..2 {
            out.clear();
            for k in 0..self.asignados.len() {
                let id = self.asignados[k];
                let Some(r) = world.get_robot_state(id, self.ent.own_team).filter(|r| r.active).cloned() else {
                    continue;
                };
                match self.paso_puesto(k, &r) {
                    Ok(Some(c)) => out.push(c),
                    Ok(None) => {}
                    Err(causa) => {
                        self.abortar_por(causa);
                        return Vec::new();
                    }
                }
            }
            if self.en_curso.is_empty() || !self.en_curso.iter().all(|e| e.hecho) {
                break;
            }
            self.cerrar_vuelta();
            if self.fase != Fase::Ejecutando {
                return Vec::new();
            }
        }
        out
    }

    fn tolerancias(&self, paso: &Paso) -> (f32, f64) {
        (
            paso.tolerancia_m.unwrap_or(self.p().tolerancia_m),
            paso.tolerancia_deg.unwrap_or(self.p().tolerancia_deg).to_radians(),
        )
    }

    /// Tiempo máximo del paso (s, ya dividido por la escala); `None` para los que duran lo
    /// que piden (`esperar` y `girar` por tiempo).
    fn tiempo_max(&self, paso: &Paso) -> Option<f64> {
        let base = paso.tiempo_max_s.unwrap_or(self.p().tiempo_max_s) / self.escala;
        match paso.accion {
            Accion::Esperar { .. } | Accion::Girar { duracion: Duracion::Tiempo(_), .. } => None,
            _ => Some(base),
        }
    }

    /// Origen del tramo ideal del paso `i`: el destino anterior, o el inicio en la primera
    /// vuelta, o el último destino desde la segunda.
    fn origen_del_tramo(&self, k: usize, i: usize) -> Vec2 {
        let puesto = &self.rutina.puestos[k];
        if let Some(d) = puesto.pasos[..i].iter().rev().find_map(|p| p.accion.destino()) {
            return d;
        }
        if self.repeticion > 1
            && let Some(d) = puesto.destinos().last()
        {
            return d;
        }
        puesto.inicio.pos
    }

    /// Avanza los pasos del puesto `k` que terminan en este tick y devuelve la elección del
    /// paso en curso (`None` si ya hizo su vuelta), o la causa de aborto.
    fn paso_puesto(&mut self, k: usize, r: &RobotState) -> Result<Option<SkillChoice>, String> {
        let rutina = self.rutina.clone();
        let puesto = &rutina.puestos[k];
        let total = puesto.pasos.len();
        for _ in 0..=total {
            if self.en_curso[k].hecho {
                return Ok(None);
            }
            let i = self.en_curso[k].paso;
            let paso = &puesto.pasos[i];
            match self.evaluar(k, i, paso, r)? {
                Evaluacion::Termina => self.en_curso[k].siguiente_paso(self.t, total),
                Evaluacion::Sigue(c) => {
                    let e = &self.en_curso[k];
                    let (dt, orientando) = (self.t - e.t_paso, e.orientando);
                    if let Some(max) = self.tiempo_max(paso)
                        && dt > max
                    {
                        return Err(frase_fuera_de_tiempo(r.id, i, paso, max, orientando));
                    }
                    return Ok(Some(c));
                }
            }
        }
        Ok(None)
    }

    fn evaluar(&mut self, k: usize, i: usize, paso: &Paso, r: &RobotState) -> Result<Evaluacion, String> {
        let (tol, tol_ang) = self.tolerancias(paso);
        let bidir = self.ent.bidireccional;
        let t = self.t;
        let origen = self.origen_del_tramo(k, i);
        let e = &mut self.en_curso[k];
        let dt = t - e.t_paso;
        Ok(match &paso.accion {
            Accion::Ir { punto, orientacion } => {
                if !e.orientando {
                    if (r.position - *punto).length() > tol {
                        let err = f64::from(distancia_a_tramo(r.position, origen, *punto));
                        e.err_suma += err;
                        e.err_n += 1;
                        e.err_max = e.err_max.max(err);
                        return Ok(Evaluacion::Sigue(choice(r.id, SkillId::GoTo, *punto)));
                    }
                    if orientacion.is_none() {
                        return Ok(Evaluacion::Termina);
                    }
                    e.orientando = true;
                }
                let th = orientacion.unwrap_or(r.orientation);
                if error_heading(th, r.orientation, bidir).abs() <= tol_ang {
                    Evaluacion::Termina
                } else {
                    Evaluacion::Sigue(choice(r.id, SkillId::FacePoint, mira_hacia(*punto, th)))
                }
            }
            Accion::Mirar { punto } => {
                let dir = *punto - r.position;
                if dir.length_squared() < f32::EPSILON {
                    return Ok(Evaluacion::Termina);
                }
                let th = f64::from(dir.y).atan2(f64::from(dir.x));
                if error_heading(th, r.orientation, bidir).abs() <= tol_ang {
                    Evaluacion::Termina
                } else {
                    Evaluacion::Sigue(choice(r.id, SkillId::FacePoint, *punto))
                }
            }
            Accion::Girar { sentido, duracion } => {
                if let Some(prev) = e.theta_prev {
                    e.giro += Motion::normalize_angle(r.orientation - prev);
                }
                e.theta_prev = Some(r.orientation);
                let signo = sentido.signo();
                let hecho = match duracion {
                    Duracion::Vueltas(v) => e.giro * signo >= std::f64::consts::TAU * v,
                    Duracion::Tiempo(s) => dt + 1e-9 >= *s,
                };
                if hecho {
                    Evaluacion::Termina
                } else {
                    Evaluacion::Sigue(choice(r.id, SkillId::Spin, Vec2::new(signo as f32, 0.0)))
                }
            }
            Accion::Esperar { tiempo_s } => {
                if dt + 1e-9 >= *tiempo_s {
                    Evaluacion::Termina
                } else {
                    Evaluacion::Sigue(choice(r.id, SkillId::Hold, Vec2::ZERO))
                }
            }
            otro => return Err(format!("el paso `{}` no está disponible en esta fase", otro.tipo())),
        })
    }

    /// Cierra la vuelta en curso (todos los puestos hicieron sus pasos).
    fn cerrar_vuelta(&mut self) {
        let puestos: Vec<VueltaPuesto> = self
            .en_curso
            .iter()
            .enumerate()
            .map(|(k, e)| VueltaPuesto {
                puesto: self.rutina.puestos[k].nombre.clone(),
                robot: self.asignados[k],
                error_medio_m: (e.err_n > 0).then(|| e.err_suma / f64::from(e.err_n)),
                error_max_m: (e.err_n > 0).then_some(e.err_max),
                escapes: e.escapes,
                guardia: e.guardia,
            })
            .collect();
        self.vueltas.push(Vuelta {
            n: self.repeticion,
            duracion_s: self.t - self.t_vuelta - self.pausa_en_vuelta,
            puestos,
        });
        let ultima = matches!(self.op.repeticiones, Repeticiones::Veces(n) if self.repeticion >= n);
        if ultima {
            self.cerrar(Resultado::Completada, None);
        } else {
            self.repeticion += 1;
            self.t_vuelta = self.t;
            self.pausa_en_vuelta = 0.0;
            self.en_curso = (0..self.asignados.len()).map(|_| EnCurso::nuevo(self.t)).collect();
        }
    }

    // ─── Lo que llega del lazo ───────────────────────────────────────────────

    fn observar_contexto(&mut self, ctx: &ContextoTick<'_>) {
        #[cfg(test)]
        assert!(!self.panico, "pánico de prueba en observar");
        self.t = f64::from(ctx.tick) * CONTROL_DT;
        self.modo = ctx.modo;
        let escape: HashSet<i32> = ctx.escapando.iter().copied().collect();
        let guardia: HashSet<i32> = ctx.guardia.iter().copied().collect();
        if self.fase == Fase::Ejecutando {
            for (k, id) in self.asignados.iter().enumerate() {
                let e = &mut self.en_curso[k];
                if escape.contains(id) && !self.previo_escape.contains(id) {
                    e.escapes += 1;
                }
                if guardia.contains(id) && !self.previo_guardia.contains(id) {
                    e.guardia += 1;
                }
            }
        }
        self.previo_escape = escape;
        self.previo_guardia = guardia;
        // Antes de asignar, la rutina espera: el lazo arranca en corte sin visión hasta el
        // primer cuadro, y solo asigna en un tick en que consulta al decisor (juego o STOP).
        if !self.fase.activa() || self.asignados.is_empty() {
            return;
        }
        let causa = if ctx.estop {
            Some("parada de emergencia".to_string())
        } else {
            match ctx.modo {
                TickMode::Halt => Some("HALT del árbitro".to_string()),
                TickMode::Stop => Some("STOP del árbitro".to_string()),
                TickMode::SinVision => Some("corte sin visión de los robots propios".to_string()),
                TickMode::Play => None,
            }
        };
        let causa = causa
            .or_else(|| {
                self.asignados
                    .iter()
                    .find(|id| ctx.intervenidos.contains(id))
                    .map(|id| format!("el robot {id} pasó a control manual"))
            })
            .or_else(|| {
                (ctx.teleport_gui && matches!(self.fase, Fase::Ejecutando | Fase::Pausada))
                    .then(|| "teleport durante la ejecución".to_string())
            });
        if let Some(c) = causa {
            self.abortar_por(c);
        }
    }

    // ─── Vista ───────────────────────────────────────────────────────────────

    fn frase(&self) -> String {
        match self.fase {
            Fase::Preparando if self.asignados.is_empty() => FRASE_ESPERA.to_string(),
            Fase::Preparando | Fase::Lista => frase_preparacion(self.falta.as_ref()),
            Fase::Ejecutando => {
                let reps = self.op.repeticiones;
                self.en_curso
                    .iter()
                    .enumerate()
                    .find(|(_, e)| !e.hecho)
                    .map(|(k, e)| {
                        let paso = &self.rutina.puestos[k].pasos[e.paso];
                        let total = self.rutina.puestos[k].pasos.len();
                        frase_ejecucion(
                            self.repeticion,
                            reps,
                            e.paso,
                            total,
                            self.asignados[k],
                            &verbo(paso.accion.tipo(), e.paso, e.orientando),
                        )
                    })
                    .unwrap_or_else(|| format!("Vuelta {}: cerrando la vuelta", self.repeticion))
            }
            Fase::Pausada => {
                let (paso, total) = self
                    .en_curso
                    .iter()
                    .enumerate()
                    .find(|(_, e)| !e.hecho)
                    .map_or((0, 0), |(k, e)| (e.paso, self.rutina.puestos[k].pasos.len()));
                frase_pausa(self.repeticion, paso, total)
            }
            Fase::Terminada | Fase::Abortada => match &self.resumen {
                Some(r) => frase_cierre(r),
                None => "Abortada".to_string(),
            },
        }
    }

    fn robots_vista(&self) -> Vec<RobotEnRutina> {
        self.asignados
            .iter()
            .enumerate()
            .map(|(k, id)| {
                let puesto = &self.rutina.puestos[k];
                let total = puesto.pasos.len();
                let en_paso = match self.fase {
                    Fase::Ejecutando | Fase::Pausada => self.en_curso.get(k).filter(|e| !e.hecho).map(|e| e.paso),
                    _ => None,
                };
                let (accion, destino) = match en_paso {
                    Some(i) => {
                        let a = &puesto.pasos[i].accion;
                        let destino = match a {
                            Accion::Ir { punto, .. } | Accion::Mirar { punto } => Some(*punto),
                            _ => None,
                        };
                        (Some(a.tipo()), destino)
                    }
                    None if matches!(self.fase, Fase::Preparando | Fase::Lista) => (None, Some(puesto.inicio.pos)),
                    None => (None, None),
                };
                RobotEnRutina { robot: *id, puesto: k, paso: en_paso, total_pasos: total, accion, destino }
            })
            .collect()
    }

    fn refrescar_vista(&mut self) {
        let nueva = RutinaVista {
            rutina: self.rutina.clone(),
            fase: self.fase,
            frase: self.frase(),
            velocidad_pct: self.op.velocidad_pct,
            repeticion: self.repeticion,
            repeticiones: self.op.repeticiones,
            robots: self.robots_vista(),
            resumen: self.resumen.clone(),
        };
        if nueva != *self.vista {
            self.vista = Arc::new(nueva);
        }
    }
}

impl TickDecider for RoutineDecider {
    fn decide(&mut self, tick: u32, world: &World) -> Vec<SkillChoice> {
        self.decidir(tick, world, SystemTime::now())
    }

    fn observar(&mut self, ctx: &ContextoTick<'_>) {
        self.observar_contexto(ctx);
        self.refrescar_vista();
    }

    /// Desde Preparar hasta que termina o se aborta (D7).
    fn perfil_velocidad(&self) -> Option<PerfilVelocidad> {
        self.fase.activa().then(|| PerfilVelocidad {
            escala: self.escala,
            omega_giro: self.ent.spin_omega.min(self.p().giro_max_rad_s) * self.escala,
        })
    }

    fn tomar_teleport(&mut self) -> Vec<TeleportItem> {
        std::mem::take(&mut self.teleport)
    }

    fn rutina(&self) -> Option<Arc<RutinaVista>> {
        Some(self.vista.clone())
    }
}

/// Hold para cada robot propio activo con id < `num_robots` (lo usa el conmutador cuando
/// la rutina falló).
pub(crate) fn hold_a_todos(world: &World, own_team: i32, num_robots: usize) -> Vec<SkillChoice> {
    let propios = if own_team == 0 { world.get_blue_team_active() } else { world.get_yellow_team_active() };
    let mut ids: Vec<i32> = propios
        .into_iter()
        .map(|r| r.id)
        .filter(|id| *id >= 0 && (*id as usize) < num_robots)
        .collect();
    ids.sort();
    ids.into_iter().map(|id| choice(id, SkillId::Hold, Vec2::ZERO)).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rutinas::prueba;
    use crate::vision_tools::XorShift64;

    const INICIO: Vec2 = Vec2::new(-0.3, -0.25);

    fn entorno(bidireccional: bool) -> Entorno {
        Entorno {
            own_team: 0,
            num_robots: 3,
            keeper_id: 2,
            bidireccional,
            blind_stop_s: 0.4,
            spin_omega: 20.0,
            params: RutinasParams::default(),
        }
    }

    fn opciones(inicio_automatico: bool) -> OpcionesCorrida {
        OpcionesCorrida {
            eleccion: Eleccion::Automatica,
            velocidad_pct: 100,
            repeticiones: Repeticiones::Veces(1),
            teletransportar: false,
            simulacion: true,
            inicio_automatico,
        }
    }

    fn con_texto(texto: &str, op: OpcionesCorrida, bidir: bool) -> RoutineDecider {
        RoutineDecider::new(prueba::rutina(texto), op, entorno(bidir))
    }

    fn decisor(pasos: &str, op: OpcionesCorrida) -> RoutineDecider {
        con_texto(&prueba::un_puesto(pasos), op, false)
    }

    /// Mundo con robots propios (azules): (id, x, y, θ en grados).
    fn mundo(robots: &[(i32, f32, f32, f64)]) -> World {
        let mut w = World::new(3, 3);
        for &(id, x, y, deg) in robots {
            w.update_robot(id, 0, Vec2::new(x, y), deg.to_radians(), Vec2::ZERO, 0.0);
        }
        w.update_ball(Vec2::new(0.0, -0.55), Vec2::ZERO);
        w
    }

    fn en_inicio() -> World {
        mundo(&[(0, INICIO.x, INICIO.y, 0.0)])
    }

    /// `n` ticks desde `t0` con el mundo `w`; las elecciones del último.
    fn correr(d: &mut RoutineDecider, w: &World, t0: u32, n: u32) -> Vec<SkillChoice> {
        let ahora = SystemTime::now();
        let mut out = Vec::new();
        for t in t0..t0 + n {
            out = d.decidir(t, w, ahora);
        }
        out
    }

    /// Corre con el robot quieto en el inicio hasta que arranca (inicio automático) y
    /// devuelve el tick del arranque.
    fn hasta_ejecutar(d: &mut RoutineDecider, w: &World) -> u32 {
        let ahora = SystemTime::now();
        for t in 0..200 {
            d.decidir(t, w, ahora);
            if d.fase() == Fase::Ejecutando {
                return t;
            }
        }
        panic!("no arrancó: {}", d.vista().frase);
    }

    fn ctx<'a>(tick: u32, modo: TickMode, estop: bool, intervenidos: &'a [i32], teleport_gui: bool) -> ContextoTick<'a> {
        ContextoTick { tick, modo, estop, intervenidos, teleport_gui, escapando: &[], guardia: &[] }
    }

    const IR: &str = r#"{ "paso": "ir", "x": 0.3, "y": -0.25 }"#;

    #[test]
    fn prepares_with_goto_then_facepoint_and_gets_ready() {
        let mut d = decisor(IR, opciones(false));
        let c = correr(&mut d, &mundo(&[(0, 0.3, 0.3, 0.0)]), 0, 1);
        assert_eq!(c, vec![choice(0, SkillId::GoTo, INICIO)]);
        assert_eq!(d.vista().frase, "Preparando: el robot 0 va a su puesto (A)");
        let c = correr(&mut d, &mundo(&[(0, INICIO.x, INICIO.y, 90.0)]), 1, 1);
        assert_eq!(c, vec![choice(0, SkillId::FacePoint, INICIO + Vec2::new(MIRA_LEJOS_M, 0.0))]);
        assert_eq!(d.vista().frase, "Preparando: el robot 0 se orienta en su puesto");
        correr(&mut d, &en_inicio(), 2, 40);
        assert_eq!(d.fase(), Fase::Lista);
        assert_eq!(d.vista().frase, "Lista: robots en su puesto y quietos");
        // En la GUI espera a Iniciar.
        correr(&mut d, &en_inicio(), 42, 100);
        assert_eq!(d.fase(), Fase::Lista);
        d.ordenar(OrdenCorrida::Iniciar);
        let c = correr(&mut d, &en_inicio(), 142, 1);
        assert_eq!(c, vec![choice(0, SkillId::GoTo, Vec2::new(0.3, -0.25))]);
        assert_eq!(d.vista().frase, "Vuelta 1 de 1, paso 1 de 1: el robot 0 va al punto 1");
    }

    #[test]
    fn quiet_with_the_proxy_noise() {
        let mut d = decisor(IR, opciones(false));
        let mut rng = XorShift64::new(7);
        let ahora = SystemTime::now();
        for t in 0..60 {
            let x = INICIO.x + (0.00185 * rng.gauss()) as f32;
            let y = INICIO.y + (0.00185 * rng.gauss()) as f32;
            let th = (0.031 * rng.gauss()).to_degrees();
            d.decidir(t, &mundo(&[(0, x, y, th)]), ahora);
        }
        assert_eq!(d.fase(), Fase::Lista, "{}", d.vista().frase);
    }

    #[test]
    fn moving_slowly_is_not_quiet() {
        let mut d = decisor(IR, opciones(false));
        let ahora = SystemTime::now();
        for t in 0..90 {
            // 3 cm/s, siempre dentro de la tolerancia del inicio.
            let x = INICIO.x - 0.02 + 0.03 * (t as f32 * 0.016);
            d.decidir(t, &mundo(&[(0, x, INICIO.y, 0.0)]), ahora);
        }
        assert_eq!(d.fase(), Fase::Preparando);
        assert_eq!(d.vista().frase, "Preparando: esperando que el robot 0 quede quieto");
    }

    #[test]
    fn backwards_is_in_position_only_in_bidirectional() {
        let atras = mundo(&[(0, INICIO.x, INICIO.y, 180.0)]);
        let mut bidir = con_texto(&prueba::un_puesto(IR), opciones(false), true);
        correr(&mut bidir, &atras, 0, 40);
        assert_eq!(bidir.fase(), Fase::Lista);
        let mut frontal = con_texto(&prueba::un_puesto(IR), opciones(false), false);
        correr(&mut frontal, &atras, 0, 40);
        assert_eq!(frontal.vista().frase, "Preparando: el robot 0 se orienta en su puesto");
    }

    #[test]
    fn unassigned_robot_near_the_route_blocks_without_aborting() {
        let mut d = decisor(IR, opciones(true));
        // A 0.12 m del recorrido (abajo del tramo), sin puesto.
        let w = mundo(&[(0, INICIO.x, INICIO.y, 0.0), (1, 0.0, -0.37, 0.0)]);
        correr(&mut d, &w, 0, 100);
        assert_eq!(d.fase(), Fase::Preparando);
        assert_eq!(d.vista().frase, "Preparando: el robot 1 está a 0.12 m del recorrido de A");
        // Con el robot lejos, arranca.
        correr(&mut d, &mundo(&[(0, INICIO.x, INICIO.y, 0.0), (1, 0.0, 0.3, 0.0)]), 100, 2);
        assert_eq!(d.fase(), Fase::Ejecutando);
    }

    #[test]
    fn preparation_timeout_scales_with_speed() {
        let lejos = mundo(&[(0, 0.3, 0.3, 0.0)]);
        let mut d = decisor(IR, opciones(false));
        correr(&mut d, &lejos, 0, 1250);
        assert_eq!(d.fase(), Fase::Preparando, "antes de 20 s no aborta");
        correr(&mut d, &lejos, 1250, 3);
        assert_eq!(d.causa(), Some("el robot 0 no llegó a su puesto en 20 s"));
        // Al 60 %: 20 / 0.6 = 33.3 s.
        let mut d = decisor(IR, OpcionesCorrida { velocidad_pct: 60, ..opciones(false) });
        correr(&mut d, &lejos, 0, 2080);
        assert_eq!(d.fase(), Fase::Preparando, "a los 33.3 s, no a los 20");
        correr(&mut d, &lejos, 2080, 6);
        assert_eq!(d.causa(), Some("el robot 0 no llegó a su puesto en 33.3 s"));
    }

    #[test]
    fn a_step_that_does_not_arrive_aborts_with_its_phrase() {
        let mut d = decisor(IR, opciones(true));
        let w = en_inicio();
        let t0 = hasta_ejecutar(&mut d, &w);
        correr(&mut d, &w, t0 + 1, 498);
        assert_eq!(d.fase(), Fase::Ejecutando, "antes de 8 s no aborta");
        correr(&mut d, &w, t0 + 499, 5);
        assert_eq!(d.causa(), Some("el robot 0 no llegó al punto 1 en 8 s"));
        assert_eq!(d.vista().frase, "Abortada: el robot 0 no llegó al punto 1 en 8 s");
        // Abortada: Hold y sin perfil.
        assert_eq!(correr(&mut d, &w, t0 + 504, 1), vec![choice(0, SkillId::Hold, Vec2::ZERO)]);
        assert!(d.perfil_velocidad().is_none());
    }

    #[test]
    fn look_and_turn_time_out_with_their_phrases() {
        let mut d = decisor(r#"{ "paso": "mirar", "x": -0.3, "y": 0.5, "tiempo_max_s": 2 }"#, opciones(true));
        let w = en_inicio();
        let t0 = hasta_ejecutar(&mut d, &w);
        correr(&mut d, &w, t0 + 1, 130);
        assert_eq!(d.causa(), Some("el robot 0 no se orientó al punto 1 en 2 s"));
        let mut d = decisor(r#"{ "paso": "girar", "sentido": "horario", "vueltas": 2, "tiempo_max_s": 3 }"#, opciones(true));
        let t0 = hasta_ejecutar(&mut d, &w);
        correr(&mut d, &w, t0 + 1, 200);
        assert_eq!(d.causa(), Some("el robot 0 no completó 2 vueltas en 3 s"));
    }

    #[test]
    fn turn_counts_full_turns_across_pi() {
        let texto = prueba::un_puesto(r#"{ "paso": "girar", "sentido": "antihorario", "vueltas": 2 }"#)
            .replace("\"orientacion_deg\": 0", "\"orientacion_deg\": 170");
        let mut d = con_texto(&texto, opciones(true), false);
        let t0 = hasta_ejecutar(&mut d, &mundo(&[(0, INICIO.x, INICIO.y, 170.0)]));
        let ahora = SystemTime::now();
        for k in 1..=62u32 {
            let th = 170f64.to_radians() + 0.2 * f64::from(k);
            let c = d.decidir(t0 + k, &mundo(&[(0, INICIO.x, INICIO.y, th.to_degrees())]), ahora);
            assert_eq!(c, vec![choice(0, SkillId::Spin, Vec2::new(1.0, 0.0))], "k = {k}");
        }
        assert_eq!(d.fase(), Fase::Ejecutando, "a 4π le falta poco");
        let th = 170f64.to_radians() + 0.2 * 63.0;
        d.decidir(t0 + 63, &mundo(&[(0, INICIO.x, INICIO.y, th.to_degrees())]), ahora);
        assert_eq!(d.fase(), Fase::Terminada);
    }

    #[test]
    fn wait_takes_its_time() {
        let mut d = decisor(r#"{ "paso": "esperar", "tiempo_s": 2 }"#, opciones(true));
        let w = en_inicio();
        let t0 = hasta_ejecutar(&mut d, &w);
        let c = correr(&mut d, &w, t0 + 1, 124);
        assert_eq!(c, vec![choice(0, SkillId::Hold, Vec2::ZERO)]);
        assert_eq!(d.fase(), Fase::Ejecutando);
        correr(&mut d, &w, t0 + 125, 1);
        assert_eq!(d.fase(), Fase::Terminada);
        assert_eq!(d.vista().frase, "Terminada: 1 vuelta en 2.0 s");
    }

    #[test]
    fn pause_freezes_the_step_time() {
        let mut d = decisor(IR, opciones(true));
        let w = en_inicio();
        let t0 = hasta_ejecutar(&mut d, &w);
        correr(&mut d, &w, t0 + 1, 312);
        d.ordenar(OrdenCorrida::Pausar);
        assert_eq!(d.fase(), Fase::Pausada);
        assert_eq!(d.vista().frase, "Pausada en la vuelta 1, paso 1 de 1");
        let c = correr(&mut d, &w, t0 + 313, 625);
        assert_eq!(c, vec![choice(0, SkillId::Hold, Vec2::ZERO)]);
        d.ordenar(OrdenCorrida::Reanudar);
        correr(&mut d, &w, t0 + 938, 180);
        assert_eq!(d.fase(), Fase::Ejecutando, "la pausa no consume el tiempo del paso");
        correr(&mut d, &w, t0 + 1118, 20);
        assert_eq!(d.causa(), Some("el robot 0 no llegó al punto 1 en 8 s"));
    }

    #[test]
    fn a_robot_out_of_sight_aborts() {
        let mut d = decisor(IR, opciones(false));
        let w = en_inicio();
        correr(&mut d, &w, 0, 1);
        d.decidir(1, &w, SystemTime::now() + Duration::from_secs(1));
        assert_eq!(d.causa(), Some("el robot 0 dejó de verse"));
        assert_eq!(d.vista().frase, "Abortada: el robot 0 dejó de verse");
    }

    #[test]
    fn near_collision_aborts_and_a_crossing_does_not() {
        let mut d = decisor(IR, opciones(false));
        correr(&mut d, &mundo(&[(0, INICIO.x, INICIO.y, 0.0), (2, INICIO.x + 0.15, INICIO.y, 0.0)]), 0, 20);
        assert!(d.fase().activa(), "0.15 m no aborta: {}", d.vista().frase);
        correr(&mut d, &mundo(&[(0, INICIO.x, INICIO.y, 0.0), (2, INICIO.x + 0.08, INICIO.y, 0.0)]), 20, 1);
        assert_eq!(d.causa(), Some("los robots 0 y 2 quedaron a 0.08 m"));
    }

    #[test]
    fn loop_causes_arrive_through_observar() {
        let w = en_inicio();
        // (modo, parada, intervenidos, teleport de la GUI, causa esperada)
        type Caso<'a> = (TickMode, bool, &'a [i32], bool, Option<&'a str>);
        let casos: [Caso; 8] = [
            (TickMode::Halt, false, &[], false, Some("HALT del árbitro")),
            (TickMode::Halt, true, &[], false, Some("parada de emergencia")),
            (TickMode::Stop, false, &[], false, Some("STOP del árbitro")),
            (TickMode::SinVision, false, &[], false, Some("corte sin visión de los robots propios")),
            (TickMode::Play, false, &[0], false, Some("el robot 0 pasó a control manual")),
            (TickMode::Play, false, &[1], false, None),
            (TickMode::Play, false, &[], true, Some("teleport durante la ejecución")),
            (TickMode::Play, false, &[], false, None),
        ];
        for (modo, estop, interv, tp, causa) in casos {
            let mut d = decisor(IR, opciones(true));
            let t0 = hasta_ejecutar(&mut d, &w);
            d.observar(&ctx(t0, modo, estop, interv, tp));
            assert_eq!(d.causa(), causa, "{modo:?} estop={estop} {interv:?} tp={tp}");
            if causa.is_some() {
                assert_eq!(d.fase(), Fase::Abortada);
                assert_eq!(d.resumen().unwrap().fase_aborto, Some(Fase::Ejecutando));
            }
        }
    }

    #[test]
    fn before_assigning_it_waits_for_vision_and_play() {
        // El lazo arranca en corte sin visión hasta el primer cuadro, y en HALT no consulta
        // al decisor: la rutina espera, sin abortar.
        let mut d = decisor(IR, opciones(true));
        assert_eq!(d.vista().frase, "Preparando");
        d.observar(&ctx(0, TickMode::SinVision, false, &[], false));
        d.observar(&ctx(1, TickMode::Halt, false, &[], false));
        assert_eq!(d.fase(), Fase::Preparando);
        assert_eq!(d.vista().frase, FRASE_ESPERA);
        // Con el primer tick de juego asigna; desde ahí, el corte aborta.
        correr(&mut d, &en_inicio(), 2, 1);
        assert_eq!(d.asignados(), &[0]);
        d.observar(&ctx(2, TickMode::SinVision, false, &[], false));
        assert_eq!(d.causa(), Some("corte sin visión de los robots propios"));
    }

    #[test]
    fn gui_teleport_while_preparing_does_not_abort() {
        let mut d = decisor(IR, opciones(false));
        correr(&mut d, &en_inicio(), 0, 5);
        d.observar(&ctx(5, TickMode::Play, false, &[], true));
        assert_eq!(d.fase(), Fase::Preparando);
        // STOP en la preparación sí aborta.
        d.observar(&ctx(6, TickMode::Stop, false, &[], false));
        assert_eq!(d.causa(), Some("STOP del árbitro"));
        assert_eq!(d.resumen().unwrap().fase_aborto, Some(Fase::Preparando));
    }

    #[test]
    fn user_abort_and_loop_stop() {
        let w = en_inicio();
        let mut d = decisor(IR, opciones(true));
        let t0 = hasta_ejecutar(&mut d, &w);
        d.ordenar(OrdenCorrida::Abortar);
        assert_eq!(d.resumen().unwrap().resultado, Resultado::Abortada);
        assert_eq!(d.vista().frase, "Abortada por el usuario");
        let mut b = decisor(IR, OpcionesCorrida { repeticiones: Repeticiones::Bucle, ..opciones(true) });
        hasta_ejecutar(&mut b, &w);
        b.ordenar(OrdenCorrida::Abortar);
        assert_eq!(b.resumen().unwrap().resultado, Resultado::Detenida);
        assert_eq!(b.vista().frase, "Detenida por el usuario sin vueltas completas");
        // Una orden fuera de fase no hace nada.
        let mut e = decisor(IR, opciones(false));
        correr(&mut e, &w, t0, 1);
        e.ordenar(OrdenCorrida::Pausar);
        e.ordenar(OrdenCorrida::Iniciar);
        assert_eq!(e.fase(), Fase::Preparando);
    }

    #[test]
    fn profile_while_active_only() {
        let mut d = decisor(IR, OpcionesCorrida { velocidad_pct: 60, ..opciones(false) });
        let p = d.perfil_velocidad().expect("desde Preparar");
        assert_eq!(p.escala, 0.6);
        assert_eq!(p.omega_giro, 20f64.min(12.0) * 0.6);
        d.ordenar(OrdenCorrida::Abortar);
        assert!(d.perfil_velocidad().is_none());
        assert!(velocidad_valida(60) && !velocidad_valida(65) && !velocidad_valida(10) && !velocidad_valida(110));
    }

    #[test]
    fn teleport_once_and_only_in_simulation() {
        let texto = prueba::un_puesto(IR).replace("\"orientacion_deg\": 0", "\"orientacion_deg\": 90");
        let mut w = mundo(&[(0, 0.3, 0.3, 0.0), (1, 0.6, -0.5, 0.0)]);
        w.update_robot(0, 1, Vec2::ZERO, 0.0, Vec2::ZERO, 0.0);
        let mut d = con_texto(&texto, OpcionesCorrida { teletransportar: true, ..opciones(false) }, false);
        correr(&mut d, &w, 0, 1);
        let items = d.tomar_teleport();
        assert_eq!(items.len(), 4, "{items:?}");
        match items[0] {
            TeleportItem::Robot { team: 0, id: 0, x, y, theta } => {
                assert!((x + 0.3).abs() < 1e-6 && (y + 0.25).abs() < 1e-6);
                assert!((theta - 90.0).abs() < 1e-9, "en grados: {theta}");
            }
            ref o => panic!("{o:?}"),
        }
        assert!(matches!(items[1], TeleportItem::Robot { team: 0, id: 1, y, .. } if y == PARQUE_Y));
        assert!(matches!(items[2], TeleportItem::Robot { team: 1, id: 0, y, .. } if y == PARQUE_Y));
        assert!(matches!(items[3], TeleportItem::Ball { y, .. } if y == -PARQUE_Y));
        assert!(d.tomar_teleport().is_empty(), "una sola vez");
        let mut real = con_texto(&texto, OpcionesCorrida { teletransportar: true, simulacion: false, ..opciones(false) }, false);
        correr(&mut real, &w, 0, 1);
        assert!(real.tomar_teleport().is_empty(), "con el robot real no hay teleport");
    }

    #[test]
    fn summary_error_from_the_ideal_route() {
        let mut d = decisor(IR, opciones(true));
        let t0 = hasta_ejecutar(&mut d, &mundo(&[(0, INICIO.x, INICIO.y + 0.02, 0.0)]));
        let ahora = SystemTime::now();
        let mut k = 1;
        while d.fase() == Fase::Ejecutando && k < 100 {
            let x = INICIO.x + 0.01 * k as f32;
            d.decidir(t0 + k, &mundo(&[(0, x, INICIO.y + 0.02, 0.0)]), ahora);
            k += 1;
        }
        assert_eq!(d.fase(), Fase::Terminada);
        let r = d.resumen().unwrap();
        let a = &r.vueltas[0].puestos[0];
        assert!((a.error_medio_m.unwrap() - 0.02).abs() < 1e-4, "{a:?}");
        assert!((a.error_max_m.unwrap() - 0.02).abs() < 1e-4, "{a:?}");
        assert_eq!(r.asignacion, vec![("A".to_string(), 0)]);
        assert!(r.preparacion_s.is_some());
    }

    #[test]
    fn escapes_and_guard_count_episodes() {
        let mut d = decisor(r#"{ "paso": "esperar", "tiempo_s": 1 }"#, opciones(true));
        let w = en_inicio();
        let t0 = hasta_ejecutar(&mut d, &w);
        let ahora = SystemTime::now();
        for k in 1..=70u32 {
            if d.fase() != Fase::Ejecutando {
                break;
            }
            d.decidir(t0 + k, &w, ahora);
            let escapa: &[i32] = if (5..30).contains(&k) { &[0] } else { &[] };
            let frena: &[i32] = if k == 10 || k == 20 { &[0] } else { &[] };
            d.observar(&ContextoTick { escapando: escapa, guardia: frena, ..ctx(t0 + k, TickMode::Play, false, &[], false) });
        }
        let r = d.resumen().expect("terminó");
        assert_eq!(r.resultado, Resultado::Completada);
        assert_eq!((r.vueltas[0].puestos[0].escapes, r.vueltas[0].puestos[0].guardia), (1, 2));
    }

    #[test]
    fn no_robots_means_it_cannot_prepare() {
        let mut d = decisor(IR, opciones(false));
        let c = correr(&mut d, &World::new(3, 3), 0, 1);
        assert!(c.is_empty());
        assert_eq!(d.causa(), Some("no se pudo preparar: se ven 0 robots propios; la rutina pide 1 robot"));
    }

    #[test]
    fn unassigned_robots_hold_and_only_catalog_skills_are_used() {
        let texto = prueba::un_puesto(
            r#"{ "paso": "ir", "x": 0.3, "y": -0.25, "orientacion_deg": 90 }, { "paso": "mirar", "x": 0.3, "y": 0.5 },
               { "paso": "girar", "sentido": "horario", "tiempo_s": 0.5 }, { "paso": "esperar", "tiempo_s": 0.5 }"#,
        );
        let mut d = con_texto(&texto, opciones(true), false);
        let otro = (2, 0.3, 0.4, 0.0);
        let mut usadas = HashSet::new();
        let ahora = SystemTime::now();
        // Recorre los pasos moviendo el robot a mano: inicio, punto 1 (mirando al norte), etc.
        let poses = [(INICIO.x, INICIO.y, 0.0, 40), (0.3, -0.25, 0.0, 5), (0.3, -0.25, 90.0, 5), (0.3, -0.25, 90.0, 80)];
        let mut t = 0;
        for (x, y, th, n) in poses {
            for _ in 0..n {
                for c in d.decidir(t, &mundo(&[(0, x, y, th), otro]), ahora) {
                    if c.robot_id == 2 {
                        assert_eq!(c.skill_id, SkillId::Hold, "el robot sin puesto hace Hold");
                    }
                    usadas.insert(c.skill_id);
                }
                t += 1;
            }
        }
        assert_eq!(d.fase(), Fase::Terminada, "{}", d.vista().frase);
        let permitidas = [SkillId::GoTo, SkillId::FacePoint, SkillId::Spin, SkillId::Hold];
        assert!(usadas.iter().all(|s| permitidas.contains(s)), "{usadas:?}");
        assert_eq!(usadas.len(), 4, "usó las cuatro: {usadas:?}");
    }
}
