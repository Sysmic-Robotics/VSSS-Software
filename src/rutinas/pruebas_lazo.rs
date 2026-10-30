//! Tests de las rutinas dentro del lazo: el mismo orden del tick que `run_control_loop`
//! (perfil → `tick_commands` → teleport → `observar`), sin sockets, con la planta de test
//! para la corrida de punta a punta de los circuitos.

use super::conmutador::{DecisorConRutinas, OrdenRutina};
use super::decisor::{Entorno, OpcionesCorrida, OrdenCorrida, RoutineDecider};
use super::formato::{Repeticiones, Rutina, cargar_archivo};
use super::resumen::{Resultado, ResumenCorrida};
use super::validacion::ContextoValidacion;
use super::vista::Fase;
use super::{Eleccion, prueba};
use crate::coach::{Role, SkillChoice};
use crate::control_loop::{
    ContextoTick, GuiSkillCommand, ManualCommand, PerfilEnLazo, TickDecider, TickMode, escapes_y_guardia,
    intervenidos, tick_commands,
};
use crate::motion::test_plant::Plant;
use crate::motion::{BorderRecovery, Motion, MotionCommand, MotionConfig};
use crate::params::RutinasParams;
use crate::skills::zones::ZoneGuard;
use crate::skills::{SkillCatalog, SkillId};
use crate::world::World;
use glam::Vec2;
use std::collections::{HashMap, HashSet};
use std::path::Path;
use std::sync::{Arc, Mutex};
use tokio::sync::mpsc;

/// El estado del lazo que rodea al decisor.
struct Lazo {
    catalog: SkillCatalog,
    motion: Motion,
    recovery: BorderRecovery,
    guard: ZoneGuard,
    manual: HashMap<(i32, i32), (ManualCommand, u32)>,
    gui: HashMap<(i32, i32), (GuiSkillCommand, u32)>,
    perfil: PerfilEnLazo,
    tick: u32,
}

/// Topes de motion (lineal, angular) y giro de Spin de cada robot.
type Topes = (f64, f64, Vec<f64>);

impl Lazo {
    fn new(bidireccional: bool) -> Self {
        let cfg = MotionConfig { bidirectional: bidireccional, ..MotionConfig::default() };
        Self {
            catalog: SkillCatalog::new(3),
            motion: Motion::with_config(cfg).with_areas(1.0, 2),
            recovery: BorderRecovery::new(true),
            guard: ZoneGuard::new(1.0, 2),
            manual: HashMap::new(),
            gui: HashMap::new(),
            perfil: PerfilEnLazo::default(),
            tick: 0,
        }
    }

    /// Un tick como el de `run_control_loop`: comandos y elecciones aplicadas.
    fn paso(
        &mut self,
        modo: TickMode,
        estop: bool,
        teleport_gui: bool,
        world: &World,
        d: &mut dyn TickDecider,
    ) -> (Vec<MotionCommand>, Vec<SkillChoice>) {
        self.perfil.aplicar(d.perfil_velocidad(), &mut self.motion, &mut self.catalog);
        let out = tick_commands(
            modo,
            world,
            d,
            self.tick,
            &mut self.catalog,
            &mut self.motion,
            &mut self.recovery,
            &mut self.guard,
            &self.manual,
            &self.gui,
            0,
            3,
        );
        let _ = d.tomar_teleport();
        let (escapando, guardia) = escapes_y_guardia(&out, 0);
        let interv = intervenidos(&self.manual, &self.gui, 0);
        d.observar(&ContextoTick {
            tick: self.tick,
            modo,
            estop,
            intervenidos: &interv,
            teleport_gui,
            escapando: &escapando,
            guardia: &guardia,
        });
        self.tick += 1;
        (out.0, out.2)
    }

    fn juego(&mut self, world: &World, d: &mut dyn TickDecider) -> Vec<MotionCommand> {
        self.paso(TickMode::Play, false, false, world, d).0
    }

    fn topes(&self) -> Topes {
        (
            self.motion.config.max_linear_speed,
            self.motion.config.max_angular_speed,
            (0..3).map(|i| self.catalog.spin_omega_for(i)).collect(),
        )
    }
}

fn entorno(bidireccional: bool) -> Entorno {
    Entorno {
        own_team: 0,
        num_robots: 3,
        keeper_id: 2,
        bidireccional,
        blind_stop_s: 0.4,
        spin_omega: 20.0,
        params: RutinasParams::default(),
        alcance_m: 0.09,
        pelota_lenta: 0.08,
    }
}

fn opciones(pct: u32, inicio_automatico: bool) -> OpcionesCorrida {
    OpcionesCorrida {
        eleccion: Eleccion::Automatica,
        velocidad_pct: pct,
        repeticiones: Repeticiones::Veces(1),
        teletransportar: false,
        simulacion: true,
        inicio_automatico,
    }
}

const INICIO: Vec2 = Vec2::new(-0.3, -0.25);
const IR_LEJOS: &str = r#"{ "paso": "ir", "x": 0.3, "y": -0.25 }"#;

/// El robot 0 quieto en el inicio (un `World` nuevo, con el dato recién llegado).
fn en_inicio() -> World {
    let mut w = World::new(3, 3);
    w.update_robot(0, 0, INICIO, 0.0, Vec2::ZERO, 0.0);
    w.update_ball(Vec2::new(0.0, -0.9), Vec2::ZERO);
    w
}

fn all_zero(cmds: &[MotionCommand]) -> bool {
    cmds.iter().all(|c| c.vx == 0.0 && c.vy == 0.0 && c.omega == 0.0)
}

// ─── HALT y parada dentro del lazo ──────────────────────────────────────────

#[test]
fn halt_and_estop_stop_a_running_routine_from_the_first_tick() {
    for (estop, causa) in [(false, "HALT del árbitro"), (true, "parada de emergencia")] {
        let mut lazo = Lazo::new(false);
        let mut d = RoutineDecider::new(prueba::rutina(&prueba::un_puesto(IR_LEJOS)), opciones(100, true), entorno(false));
        let mut ultimo = Vec::new();
        for _ in 0..60 {
            ultimo = lazo.juego(&en_inicio(), &mut d);
        }
        assert_eq!(d.fase(), Fase::Ejecutando);
        assert!(!all_zero(&ultimo), "avanzaba: {ultimo:?}");
        let (cmds, _) = lazo.paso(TickMode::Halt, estop, false, &en_inicio(), &mut d);
        let mut ids: Vec<i32> = cmds.iter().map(|c| c.id).collect();
        ids.sort();
        assert_eq!(ids, vec![0, 1, 2]);
        assert!(all_zero(&cmds), "cero desde el primer tick: {cmds:?}");
        assert_eq!(d.fase(), Fase::Abortada);
        assert_eq!(d.causa(), Some(causa));
        // Al soltar, Hold: no retoma.
        for _ in 0..10 {
            assert!(all_zero(&lazo.juego(&en_inicio(), &mut d)));
        }
        assert_eq!(d.fase(), Fase::Abortada);
    }
}

// ─── Perfil de velocidad ────────────────────────────────────────────────────

#[test]
fn the_profile_scales_the_motion_limits_and_spin() {
    let mut lazo = Lazo::new(false);
    let antes = lazo.topes();
    let mut d = RoutineDecider::new(prueba::rutina(&prueba::un_puesto(IR_LEJOS)), opciones(60, false), entorno(false));
    lazo.juego(&en_inicio(), &mut d);
    let (lin, ang, spin) = lazo.topes();
    assert_eq!(lin, antes.0 * 0.6);
    assert_eq!(ang, antes.1 * 0.6);
    assert!(spin.iter().all(|w| *w == 20f64.min(12.0) * 0.6), "{spin:?}");
    assert!((lin - 0.72).abs() < 1e-12 && (ang - 1.8).abs() < 1e-12);
}

#[test]
fn without_a_routine_the_loop_writes_nothing() {
    let mut lazo = Lazo::new(false);
    lazo.catalog.set_spin_omega_for(1, 15.0);
    let antes = lazo.topes();
    let mut d = crate::control_loop::FixedSkillDecider::new(0, SkillId::GoTo, Vec2::new(0.3, 0.0));
    for _ in 0..30 {
        lazo.juego(&en_inicio(), &mut d);
        assert_eq!(lazo.topes(), antes);
    }
    assert!(lazo.perfil == PerfilEnLazo::default(), "nunca aplicó un perfil");
}

/// Cómo termina la corrida en `restaura`.
#[derive(Debug, Clone, Copy)]
enum Final {
    Terminada,
    Tiempo,
    Usuario,
    Halt,
    Parada,
    Stop,
    Corte,
    Manual,
}

/// Corre una rutina al 60 % hasta ejecutarla, la termina con `final_` y verifica que los
/// topes y el Spin de cada robot (con un override de la GUI previo en el robot 1) vuelven
/// bit a bit desde el tick siguiente.
fn restaura(final_: Final) {
    let mut lazo = Lazo::new(false);
    lazo.catalog.set_spin_omega_for(1, 15.0);
    let antes = lazo.topes();
    let pasos = match final_ {
        Final::Terminada => r#"{ "paso": "esperar", "tiempo_s": 0.3 }"#,
        Final::Tiempo => r#"{ "paso": "ir", "x": 0.3, "y": -0.25, "tiempo_max_s": 0.3 }"#,
        _ => IR_LEJOS,
    };
    let mut d = RoutineDecider::new(prueba::rutina(&prueba::un_puesto(pasos)), opciones(60, true), entorno(false));
    for _ in 0..40 {
        lazo.juego(&en_inicio(), &mut d);
    }
    assert_eq!(d.fase(), Fase::Ejecutando, "{final_:?}");
    assert_ne!(lazo.topes(), antes, "escalados durante la rutina");
    match final_ {
        Final::Terminada | Final::Tiempo => {
            for _ in 0..200 {
                if !d.fase().activa() {
                    break;
                }
                lazo.juego(&en_inicio(), &mut d);
            }
        }
        Final::Usuario => d.ordenar(OrdenCorrida::Abortar),
        Final::Halt => {
            lazo.paso(TickMode::Halt, false, false, &en_inicio(), &mut d);
        }
        Final::Parada => {
            lazo.paso(TickMode::Halt, true, false, &en_inicio(), &mut d);
        }
        Final::Stop => {
            lazo.paso(TickMode::Stop, false, false, &en_inicio(), &mut d);
        }
        Final::Corte => {
            lazo.paso(TickMode::SinVision, false, false, &en_inicio(), &mut d);
        }
        Final::Manual => {
            let m = ManualCommand { team: 0, id: 0, vx: 0.1, vy: 0.0, omega: 0.0 };
            lazo.manual.insert((0, 0), (m, lazo.tick));
            lazo.juego(&en_inicio(), &mut d);
        }
    }
    assert!(!d.fase().activa(), "{final_:?}: {}", d.vista().frase);
    let esperado = match final_ {
        Final::Terminada => Resultado::Completada,
        _ => Resultado::Abortada,
    };
    assert_eq!(d.resumen().map(|r| r.resultado), Some(esperado), "{final_:?}");
    lazo.juego(&en_inicio(), &mut d);
    assert_eq!(lazo.topes(), antes, "{final_:?}: topes restaurados");
    for _ in 0..5 {
        lazo.juego(&en_inicio(), &mut d);
        assert_eq!(lazo.topes(), antes, "{final_:?}: siguen restaurados");
    }
}

#[test]
fn limits_are_restored_exactly_after_every_ending() {
    for f in [
        Final::Terminada,
        Final::Tiempo,
        Final::Usuario,
        Final::Halt,
        Final::Parada,
        Final::Stop,
        Final::Corte,
        Final::Manual,
    ] {
        restaura(f);
    }
}

// ─── Conmutador de la GUI ───────────────────────────────────────────────────

/// Decisor de base espía: anota los ticks en que decide.
struct Espia {
    ticks: Arc<Mutex<Vec<u32>>>,
}

impl TickDecider for Espia {
    fn decide(&mut self, tick: u32, _world: &World) -> Vec<SkillChoice> {
        self.ticks.lock().unwrap().push(tick);
        vec![SkillChoice { robot_id: 0, skill_id: SkillId::GoTo, target: Vec2::new(0.2, 0.2) }]
    }

    fn role(&self, _robot_id: i32) -> Option<Role> {
        Some(Role::Striker)
    }
}

/// Decisor de base espía: anota la lista sin comandos que recibe.
struct EspiaSinComandos {
    lista: Arc<Mutex<Option<Vec<i32>>>>,
}

impl TickDecider for EspiaSinComandos {
    fn decide(&mut self, _tick: u32, _world: &World) -> Vec<SkillChoice> {
        Vec::new()
    }

    fn usar_sin_comandos(&mut self, patrones: &[i32]) {
        *self.lista.lock().unwrap() = Some(patrones.to_vec());
    }
}

/// El robot 0 en el inicio del puesto y el 1 lejos.
fn dos_robots() -> World {
    let mut w = en_inicio();
    w.update_robot(1, 0, Vec2::new(0.3, 0.35), 0.0, Vec2::ZERO, 0.0);
    w
}

/// Prepara la rutina de un puesto con el conmutador `d` y devuelve el robot asignado.
fn asignado_por(d: &mut DecisorConRutinas, tx: &mpsc::Sender<OrdenRutina>) -> Vec<i32> {
    let rutina = prueba::rutina(&prueba::un_puesto(IR_LEJOS));
    tx.try_send(OrdenRutina::Preparar { rutina, opciones: opciones(100, false) }).unwrap();
    let mut lazo = Lazo::new(false);
    let w = dos_robots();
    for _ in 0..5 {
        lazo.paso(TickMode::Play, false, false, &w, d);
    }
    d.rutina().expect("rutina preparada").robots.iter().map(|r| r.robot).collect()
}

#[test]
fn conmutador_reparte_sin_comandos() {
    let (tx, rx) = mpsc::channel(8);
    let lista = Arc::new(Mutex::new(None));
    let mut d = DecisorConRutinas::new(Box::new(EspiaSinComandos { lista: lista.clone() }), rx, 0, 3);
    d.usar_sin_comandos(&[0]);
    assert_eq!(*lista.lock().unwrap(), Some(vec![0]), "la base recibe la lista");
    // Con una rutina activa, la base la sigue recibiendo.
    assert_eq!(asignado_por(&mut d, &tx), vec![1]);
    d.usar_sin_comandos(&[]);
    assert_eq!(*lista.lock().unwrap(), Some(vec![]));
}

#[test]
fn rutina_nueva_recibe_la_lista_vigente() {
    // Sin lista, el más cercano (el 0); con el 0 sin comandos, el 1.
    let (tx, rx) = mpsc::channel(8);
    let mut d = DecisorConRutinas::new(Box::new(EspiaSinComandos { lista: Arc::new(Mutex::new(None)) }), rx, 0, 3);
    assert_eq!(asignado_por(&mut d, &tx), vec![0]);
    let (tx, rx) = mpsc::channel(8);
    let mut d = DecisorConRutinas::new(Box::new(EspiaSinComandos { lista: Arc::new(Mutex::new(None)) }), rx, 0, 3);
    d.usar_sin_comandos(&[0]);
    assert_eq!(asignado_por(&mut d, &tx), vec![1]);
}

struct ConCoach {
    lazo: Lazo,
    d: DecisorConRutinas,
    tx: mpsc::Sender<OrdenRutina>,
    ticks: Arc<Mutex<Vec<u32>>>,
    antes: Topes,
    decisiones_del_coach: usize,
}

impl ConCoach {
    fn new() -> Self {
        let (tx, rx) = mpsc::channel(8);
        let ticks = Arc::new(Mutex::new(Vec::new()));
        let espia = Espia { ticks: ticks.clone() };
        let mut lazo = Lazo::new(false);
        lazo.catalog.set_spin_omega_for(1, 15.0);
        let antes = lazo.topes();
        Self { lazo, d: DecisorConRutinas::new(Box::new(espia), rx, 0, 3), tx, ticks, antes, decisiones_del_coach: 0 }
    }

    /// Un tick; si decidió el coach, los topes de ese tick deben ser los base.
    fn tick_en(&mut self, modo: TickMode) {
        let n = self.ticks.lock().unwrap().len();
        self.lazo.paso(modo, false, false, &en_inicio(), &mut self.d);
        if self.ticks.lock().unwrap().len() > n {
            self.decisiones_del_coach += 1;
            assert_eq!(self.lazo.topes(), self.antes, "el coach decidió con los topes escalados");
        }
    }

    fn tick(&mut self) {
        self.tick_en(TickMode::Play);
    }

    fn ordenar(&mut self, o: OrdenRutina) {
        self.tx.try_send(o).expect("canal");
    }

    fn preparar(&mut self, pasos: &str, pct: u32) {
        let rutina = prueba::rutina(&prueba::un_puesto(pasos));
        self.ordenar(OrdenRutina::Preparar { rutina, opciones: opciones(pct, false) });
    }

    fn hasta(&mut self, fase: Option<Fase>) {
        for _ in 0..300 {
            if self.d.fase() == fase {
                return;
            }
            self.tick();
        }
        panic!("no llegó a {fase:?}: {:?}", self.d.fase());
    }
}

#[test]
fn the_coach_never_decides_with_scaled_limits() {
    // A: coach → Preparar → ejecución → HALT → juego → Cerrar → coach.
    let mut c = ConCoach::new();
    for _ in 0..10 {
        c.tick();
    }
    c.preparar(IR_LEJOS, 60);
    c.hasta(Some(Fase::Lista));
    c.ordenar(OrdenRutina::Iniciar);
    c.hasta(Some(Fase::Ejecutando));
    for _ in 0..20 {
        c.tick();
    }
    assert_ne!(c.lazo.topes(), c.antes);
    c.tick_en(TickMode::Halt);
    assert_eq!(c.d.fase(), Some(Fase::Abortada));
    for _ in 0..5 {
        c.tick();
    }
    c.ordenar(OrdenRutina::Cerrar);
    for _ in 0..20 {
        c.tick();
    }
    assert_eq!(c.d.fase(), None);
    // B: Preparar → terminada → Cerrar → coach.
    c.preparar(r#"{ "paso": "esperar", "tiempo_s": 0.3 }"#, 60);
    c.hasta(Some(Fase::Lista));
    c.ordenar(OrdenRutina::Iniciar);
    c.hasta(Some(Fase::Terminada));
    c.tick();
    c.ordenar(OrdenRutina::Cerrar);
    for _ in 0..20 {
        c.tick();
    }
    assert_eq!(c.lazo.topes(), c.antes);
    assert!(c.decisiones_del_coach >= 40, "el coach decidió antes y después: {}", c.decisiones_del_coach);
}

#[test]
fn closing_hands_control_back_and_preparing_during_execution_is_ignored() {
    let mut c = ConCoach::new();
    c.tick();
    assert_eq!(c.d.role(0), Some(Role::Striker), "sin rutina, el rol es del coach");
    c.preparar(IR_LEJOS, 100);
    c.tick();
    assert_eq!(c.d.role(0), None);
    assert!(c.d.coach_debug().is_none());
    assert!(c.d.rutina().is_some());
    c.hasta(Some(Fase::Lista));
    c.ordenar(OrdenRutina::Iniciar);
    c.hasta(Some(Fase::Ejecutando));
    // Preparar con otra rutina en ejecución: se ignora.
    c.preparar(r#"{ "paso": "esperar", "tiempo_s": 1 }"#, 100);
    c.tick();
    let vista = c.d.rutina().unwrap();
    assert_eq!(vista.fase, Fase::Ejecutando);
    assert_eq!(vista.rutina.puestos[0].pasos[0].accion.tipo(), "ir");
    // Cerrar en ejecución tampoco hace nada.
    c.ordenar(OrdenRutina::Cerrar);
    c.tick();
    assert_eq!(c.d.fase(), Some(Fase::Ejecutando));
    c.ordenar(OrdenRutina::Abortar);
    c.tick();
    assert_eq!(c.d.fase(), Some(Fase::Abortada));
    let n = c.ticks.lock().unwrap().len();
    c.ordenar(OrdenRutina::Cerrar);
    c.tick();
    c.tick();
    assert!(c.d.rutina().is_none());
    assert!(c.ticks.lock().unwrap().len() > n, "vuelve a decidir el coach");
}

#[test]
fn replacing_a_routine_does_not_stack_the_scales() {
    let mut c = ConCoach::new();
    c.preparar(IR_LEJOS, 60);
    c.hasta(Some(Fase::Lista));
    c.preparar(IR_LEJOS, 80);
    c.tick();
    c.tick();
    let (lin, ang, spin) = c.lazo.topes();
    assert_eq!(lin, c.antes.0 * 0.8, "desde lo guardado, no desde lo escalado");
    assert_eq!(ang, c.antes.1 * 0.8);
    assert!(spin.iter().all(|w| *w == 20f64.min(12.0) * 0.8));
    c.ordenar(OrdenRutina::Abortar);
    c.tick();
    c.tick();
    assert_eq!(c.lazo.topes(), c.antes);
}

#[test]
fn a_routine_panic_is_caught_holds_everyone_and_restores_the_limits() {
    for en_observar in [false, true] {
        let mut c = ConCoach::new();
        c.preparar(IR_LEJOS, 60);
        c.hasta(Some(Fase::Lista));
        c.ordenar(OrdenRutina::Iniciar);
        c.hasta(Some(Fase::Ejecutando));
        c.d.provocar_panico();
        // En HALT el lazo no llama a `decide`: el pánico sale de `observar`.
        let (cmds, _) = c.lazo.paso(
            if en_observar { TickMode::Halt } else { TickMode::Play },
            false,
            false,
            &en_inicio(),
            &mut c.d,
        );
        assert!(all_zero(&cmds), "Hold desde ese tick: {cmds:?}");
        let vista = c.d.rutina().expect("queda la vista");
        assert_eq!(vista.fase, Fase::Abortada);
        let donde = if en_observar { "observar" } else { "decide" };
        assert_eq!(vista.frase, format!("Abortada: error interno de la rutina: pánico de prueba en {donde}"));
        c.tick();
        assert_eq!(c.lazo.topes(), c.antes, "topes restaurados tras el pánico");
        let (cmds, aplicadas) = c.lazo.paso(TickMode::Play, false, false, &en_inicio(), &mut c.d);
        assert!(all_zero(&cmds));
        assert!(aplicadas.iter().all(|a| a.skill_id == SkillId::Hold));
        c.ordenar(OrdenRutina::Cerrar);
        c.tick();
        assert_eq!(c.d.fase(), None);
    }
}

#[test]
fn observar_gets_the_manual_the_gui_teleport_and_the_reflexes() {
    /// (intervenidos, teleport de la GUI) de cada tick.
    type Vistos = Arc<Mutex<Vec<(Vec<i32>, bool)>>>;
    struct Copia(Vistos);
    impl TickDecider for Copia {
        fn decide(&mut self, _t: u32, _w: &World) -> Vec<SkillChoice> {
            Vec::new()
        }
        fn observar(&mut self, ctx: &ContextoTick<'_>) {
            self.0.lock().unwrap().push((ctx.intervenidos.to_vec(), ctx.teleport_gui));
        }
    }
    let visto = Arc::new(Mutex::new(Vec::new()));
    let mut d = Copia(visto.clone());
    let mut lazo = Lazo::new(false);
    lazo.manual.insert((0, 1), (ManualCommand { team: 0, id: 1, vx: 0.0, vy: 0.0, omega: 0.0 }, 0));
    let skill = GuiSkillCommand { team: 0, id: 2, skill_id: SkillId::Hold, target: Vec2::ZERO, spin_omega: 20.0 };
    lazo.gui.insert((0, 2), (skill, 0));
    lazo.manual.insert((1, 0), (ManualCommand { team: 1, id: 0, vx: 0.0, vy: 0.0, omega: 0.0 }, 0));
    lazo.paso(TickMode::Play, false, true, &en_inicio(), &mut d);
    assert_eq!(visto.lock().unwrap()[0], (vec![1, 2], true), "solo los propios, ordenados");
}

// ─── Corrida de punta a punta en la planta ──────────────────────────────────

fn circuito(id: &str) -> Arc<Rutina> {
    let ruta = Path::new(env!("CARGO_MANIFEST_DIR")).join("config/rutinas").join(format!("{id}.json"));
    let e = cargar_archivo(&ruta, &ContextoValidacion::por_defecto(1.0));
    assert!(e.razones.is_empty(), "{id}: {:?}", e.razones);
    e.rutina.expect("leída")
}

/// Corre el circuito en la planta, con el robot a 0.28 m del inicio, hasta que cierra.
fn correr_circuito(id: &str, bidireccional: bool) -> (Arc<ResumenCorrida>, HashSet<SkillId>) {
    let rutina = circuito(id);
    let op = OpcionesCorrida { repeticiones: rutina.repeticiones, ..opciones(100, true) };
    let ini = rutina.puestos[0].inicio.pos;
    let mut d = RoutineDecider::new(rutina, op, entorno(bidireccional));
    let mut lazo = Lazo::new(bidireccional);
    let mut planta = Plant::new(ini.x + 0.2, ini.y + 0.2, 45.0);
    let mut usadas = HashSet::new();
    for _ in 0..7500 {
        let mut w = World::new(3, 3);
        w.update_robot(0, 0, Vec2::new(planta.x, planta.y), planta.th, planta.velocity(), 0.0);
        w.update_ball(Vec2::new(0.0, -0.9), Vec2::ZERO);
        let (cmds, aplicadas) = lazo.paso(TickMode::Play, false, false, &w, &mut d);
        usadas.extend(aplicadas.iter().map(|c| c.skill_id));
        let cero = MotionCommand { id: 0, team: 0, vx: 0.0, vy: 0.0, omega: 0.0, orientation: planta.th };
        let c = cmds.iter().find(|c| c.id == 0).cloned().unwrap_or(cero);
        planta.step(&c);
        if !d.fase().activa() {
            break;
        }
    }
    let r = d.resumen().cloned().unwrap_or_else(|| panic!("{id}: no cerró en 120 s: {}", d.vista().frase));
    (r, usadas)
}

#[test]
fn the_included_circuits_complete_in_the_test_plant() {
    let permitidas = [SkillId::GoTo, SkillId::FacePoint, SkillId::Spin, SkillId::Hold];
    let (mut fallas, mut tabla) = (Vec::new(), Vec::new());
    for id in ["triangulo", "ida_vuelta", "zigzag", "ocho", "cuadrado_giros"] {
        for bidireccional in [false, true] {
            let (r, usadas) = correr_circuito(id, bidireccional);
            let modo = if bidireccional { "bidireccional" } else { "frontal" };
            let pedidas = match r.repeticiones {
                Repeticiones::Veces(n) => n as usize,
                Repeticiones::Bucle => unreachable!(),
            };
            let err_max = r.vueltas.iter().filter_map(|v| v.puestos[0].error_max_m).fold(0.0, f64::max);
            let err_medio = r.vueltas.iter().filter_map(|v| v.puestos[0].error_medio_m).fold(0.0, f64::max);
            tabla.push(format!(
                "{id:<15} {modo:<13} {:<10} {} vueltas, {:.2} s por vuelta, error medio ≤ {:.3} m, máximo {:.3} m",
                r.resultado.etiqueta(),
                r.vueltas.len(),
                r.duraciones().0.unwrap_or(f64::NAN),
                err_medio,
                err_max
            ));
            if r.resultado != Resultado::Completada || r.vueltas.len() != pedidas {
                fallas.push(format!("{id} {modo}: {:?}, {} de {pedidas} vueltas", r.causa, r.vueltas.len()));
            }
            if !usadas.iter().all(|s| permitidas.contains(s)) {
                fallas.push(format!("{id} {modo}: skills {usadas:?}"));
            }
            if err_max >= ERROR_MAX_CORDURA_M {
                fallas.push(format!("{id} {modo}: se apartó {err_max:.3} m del recorrido"));
            }
        }
    }
    eprintln!("{}", tabla.join("\n"));
    assert!(fallas.is_empty(), "{}\n{}", fallas.join("\n"), tabla.join("\n"));
}

/// Cota de cordura del error respecto del recorrido ideal en la planta: detecta un destino
/// equivocado o un robot que se va, no mide calidad (eso es la comparación sim↔real). En la
/// planta, `ir` se cumple al entrar en la tolerancia con el robot todavía a ~0.5 m/s, y la
/// rampa de motion (1 m/s²) frena menos que el perfil: se pasa 12–18 cm del punto antes de
/// volver (medido al 100 %, en los dos modos).
const ERROR_MAX_CORDURA_M: f64 = 0.25;


// ─── Coach en pausa y robot manual (control-manual-doble) ───────────────────

/// Decisor de base espía de tres robots: GoTo para los ids 0–2, y anota los ticks.
struct EspiaTres {
    ticks: Arc<Mutex<Vec<u32>>>,
    /// Arquero que declara (como `CoachDecider`), o ninguno.
    arquero: Option<i32>,
}

impl TickDecider for EspiaTres {
    fn decide(&mut self, tick: u32, _world: &World) -> Vec<SkillChoice> {
        self.ticks.lock().unwrap().push(tick);
        (0..3)
            .map(|id| SkillChoice { robot_id: id, skill_id: SkillId::GoTo, target: Vec2::new(0.2, 0.1 * id as f32) })
            .collect()
    }

    fn role(&self, _robot_id: i32) -> Option<Role> {
        Some(Role::Support)
    }

    fn arquero(&self) -> Option<i32> {
        self.arquero
    }
}

/// Los tres robots propios vistos: el 0 en el inicio de `prueba::un_puesto`, los otros
/// lejos de su recorrido y de las áreas.
fn tres_robots() -> World {
    let mut w = World::new(3, 3);
    for (id, p) in [(0, INICIO), (1, Vec2::new(-0.2, 0.3)), (2, Vec2::new(0.4, 0.3))] {
        w.update_robot(id, 0, p, 0.0, Vec2::ZERO, 0.0);
    }
    w.update_ball(Vec2::new(0.0, -0.9), Vec2::ZERO);
    w
}

/// El conmutador de `main` con el interruptor del coach.
struct ConPausa {
    lazo: Lazo,
    d: DecisorConRutinas,
    tx: mpsc::Sender<OrdenRutina>,
    ticks: Arc<Mutex<Vec<u32>>>,
    pausa: Arc<std::sync::atomic::AtomicBool>,
}

impl ConPausa {
    fn nuevo(en_pausa: bool) -> Self {
        Self::con_arquero(en_pausa, None)
    }

    /// Con una base que declara `arquero`.
    fn con_arquero(en_pausa: bool, arquero: Option<i32>) -> Self {
        let (tx, rx) = mpsc::channel(8);
        let ticks = Arc::new(Mutex::new(Vec::new()));
        let pausa = Arc::new(std::sync::atomic::AtomicBool::new(en_pausa));
        let d = DecisorConRutinas::new(Box::new(EspiaTres { ticks: ticks.clone(), arquero }), rx, 0, 3)
            .con_pausa(pausa.clone());
        Self { lazo: Lazo::new(false), d, tx, ticks, pausa }
    }

    /// Un tick de juego: comandos y elecciones aplicadas.
    fn tick(&mut self) -> (Vec<MotionCommand>, Vec<SkillChoice>) {
        self.lazo.paso(TickMode::Play, false, false, &tres_robots(), &mut self.d)
    }

    fn decisiones(&self) -> usize {
        self.ticks.lock().unwrap().len()
    }

    fn pausar(&self, en_pausa: bool) {
        self.pausa.store(en_pausa, std::sync::atomic::Ordering::Relaxed);
    }

    fn manual(&mut self, id: i32, vx: f64) {
        let m = ManualCommand { team: 0, id, vx, vy: 0.0, omega: 0.0 };
        self.lazo.manual.insert((0, id), (m, self.lazo.tick));
    }
}

fn skill_de(elecciones: &[SkillChoice], id: i32) -> Option<SkillId> {
    elecciones.iter().find(|e| e.robot_id == id).map(|e| e.skill_id)
}

fn comando_de(cmds: &[MotionCommand], id: i32) -> MotionCommand {
    cmds.iter().find(|c| c.id == id && c.team == 0).expect("comando del robot").clone()
}

fn rapidez(c: &MotionCommand) -> f64 {
    c.vx.hypot(c.vy)
}

#[test]
fn paused_coach_is_not_consulted_and_every_seen_robot_holds() {
    let mut c = ConPausa::nuevo(true);
    for _ in 0..5 {
        let (cmds, elecciones) = c.tick();
        for id in 0..3 {
            assert_eq!(skill_de(&elecciones, id), Some(SkillId::Hold), "robot {id}");
        }
        assert!(all_zero(&cmds), "{cmds:?}");
    }
    assert_eq!(c.decisiones(), 0, "en pausa el coach no se consulta");
    assert_eq!(c.d.role(0), None);
    assert!(c.d.coach_debug().is_none());
    // Reanudar: desde el tick siguiente decide el coach.
    c.pausar(false);
    let (_, elecciones) = c.tick();
    assert_eq!(c.decisiones(), 1);
    assert_eq!(skill_de(&elecciones, 1), Some(SkillId::GoTo));
    assert_eq!(c.d.role(0), Some(Role::Support));
}

#[test]
fn gui_skill_and_manual_still_work_while_paused() {
    let mut c = ConPausa::nuevo(true);
    c.manual(0, 0.2);
    let skill = GuiSkillCommand { team: 0, id: 1, skill_id: SkillId::GoTo, target: Vec2::new(0.3, 0.3), spin_omega: 20.0 };
    c.lazo.gui.insert((0, 1), (skill, c.lazo.tick));
    for _ in 0..3 {
        let (cmds, elecciones) = c.tick();
        assert_eq!(comando_de(&cmds, 0).vx, 0.2, "el manual pasa");
        assert_eq!(skill_de(&elecciones, 1), Some(SkillId::GoTo), "la skill de la GUI corre");
        assert_eq!(skill_de(&elecciones, 2), Some(SkillId::Hold), "el robot libre, quieto");
    }
    assert_eq!(c.decisiones(), 0);
}

#[test]
fn a_routine_runs_while_paused_and_closing_leaves_everyone_holding() {
    let mut c = ConPausa::nuevo(true);
    let rutina = prueba::rutina(&prueba::un_puesto(r#"{ "paso": "esperar", "tiempo_s": 0.3 }"#));
    c.tx.try_send(OrdenRutina::Preparar { rutina, opciones: opciones(100, true) }).unwrap();
    let mut termino = false;
    for _ in 0..300 {
        c.tick();
        if c.d.fase() == Some(Fase::Terminada) {
            termino = true;
            break;
        }
    }
    assert!(termino, "la rutina corre igual con el coach en pausa: {:?}", c.d.fase());
    c.tx.try_send(OrdenRutina::Cerrar).unwrap();
    c.tick();
    assert_eq!(c.d.fase(), None);
    let (cmds, elecciones) = c.tick();
    for id in 0..3 {
        assert_eq!(skill_de(&elecciones, id), Some(SkillId::Hold), "robot {id}");
    }
    assert!(all_zero(&cmds));
    assert_eq!(c.decisiones(), 0, "el coach no se consultó en toda la corrida");
}

#[test]
fn active_coach_holds_the_manual_robot_from_the_next_tick() {
    let mut c = ConPausa::nuevo(false);
    c.tick();
    c.manual(1, 0.25);
    let (cmds, elecciones) = c.tick();
    assert_eq!(skill_de(&elecciones, 1), Some(SkillId::GoTo), "en el tick N todavía no lo sabe");
    assert_eq!(comando_de(&cmds, 1).vx, 0.25);
    for _ in 0..3 {
        let (cmds, elecciones) = c.tick();
        assert_eq!(skill_de(&elecciones, 1), Some(SkillId::Hold), "desde N + 1, Hold por dentro");
        assert_eq!(comando_de(&cmds, 1).vx, 0.25, "lo enviado es el manual");
        assert_eq!(skill_de(&elecciones, 0), Some(SkillId::GoTo), "los otros siguen con el coach");
        assert_eq!(skill_de(&elecciones, 2), Some(SkillId::GoTo));
    }
    assert!(c.decisiones() >= 5, "el coach sigue decidiendo");
}

#[test]
fn releasing_the_manual_restarts_the_coach_skill_ramp() {
    let mut c = ConPausa::nuevo(false);
    for _ in 0..3 {
        c.tick();
    }
    c.manual(1, 0.25);
    for _ in 0..15 {
        c.tick();
    }
    c.lazo.manual.clear();
    // Un tick más con Hold (los intervenidos son del tick anterior) y vuelve el coach.
    let (cmds, elecciones) = c.tick();
    assert_eq!(skill_de(&elecciones, 1), Some(SkillId::Hold));
    assert_eq!(rapidez(&comando_de(&cmds, 1)), 0.0);
    let (cmds, elecciones) = c.tick();
    assert_eq!(skill_de(&elecciones, 1), Some(SkillId::GoTo));
    let (v0, v1) = (rapidez(&comando_de(&cmds, 0)), rapidez(&comando_de(&cmds, 1)));
    assert!(v0 > 0.1, "el robot 0 lleva rato en GoTo: {v0}");
    assert!(v1 < 0.05, "el robot 1 arranca con la rampa desde cero: {v1}");
}

#[test]
fn the_pause_shows_also_in_ticks_without_decide() {
    // Desde el arranque, aunque los primeros ticks sean de corte sin visión.
    let c = ConPausa::nuevo(true);
    assert_eq!(c.d.role(0), None, "antes del primer tick");
    // Pausar durante una parada (HALT): el lazo no llama a decide.
    let mut c = ConPausa::nuevo(false);
    c.tick();
    assert_eq!(c.d.role(0), Some(Role::Support));
    c.pausar(true);
    c.lazo.paso(TickMode::Halt, true, false, &tres_robots(), &mut c.d);
    assert_eq!(c.d.role(0), None, "en pausa durante la parada");
    c.lazo.paso(TickMode::SinVision, false, false, &tres_robots(), &mut c.d);
    assert_eq!(c.d.role(0), None, "y en el corte sin visión");
    assert_eq!(c.decisiones(), 1, "el coach no se consultó en esos ticks");
    c.pausar(false);
    c.lazo.paso(TickMode::Halt, false, false, &tres_robots(), &mut c.d);
    assert_eq!(c.d.role(0), Some(Role::Support), "al reanudar, otra vez el coach");
}

// ─── Arquero vigente (change `arquero`) ─────────────────────────────────────

/// Rutina de un robot cuyo único puesto es de arquero.
fn rutina_de_arquero() -> Arc<Rutina> {
    let texto = prueba::un_puesto(r#"{ "paso": "esperar", "tiempo_s": 0.3 }"#)
        .replace(r#""nombre": "A","#, r#""nombre": "A", "arquero": true,"#);
    prueba::rutina(&texto)
}

#[test]
fn the_coach_keeper_stays_while_paused_and_with_a_manual_robot() {
    let mut c = ConPausa::con_arquero(false, Some(0));
    c.tick();
    assert_eq!(c.lazo.guard.keeper(), 0, "el arquero del coach");
    c.manual(0, 0.2);
    c.tick();
    assert_eq!(c.lazo.guard.keeper(), 0, "tomarlo en manual no cambia el arquero");
    c.lazo.manual.clear();
    c.pausar(true);
    for _ in 0..5 {
        c.tick();
        assert_eq!(c.d.role(0), None, "en pausa no hay roles");
        assert_eq!((c.d.arquero(), c.lazo.guard.keeper()), (Some(0), 0), "pero sí arquero");
    }
}

#[test]
fn a_routine_without_a_keeper_post_keeps_the_last_keeper() {
    let mut c = ConPausa::con_arquero(false, Some(0));
    c.tick();
    let rutina = prueba::rutina(&prueba::un_puesto(r#"{ "paso": "esperar", "tiempo_s": 0.3 }"#));
    c.tx.try_send(OrdenRutina::Preparar { rutina, opciones: opciones(100, true) }).unwrap();
    for _ in 0..30 {
        c.tick();
        assert_eq!(c.d.arquero(), None, "la rutina no tiene puesto de arquero");
        assert_eq!(c.lazo.guard.keeper(), 0, "queda el último: {:?}", c.d.fase());
    }
}

#[test]
fn a_routine_keeper_post_is_the_keeper_until_it_is_closed() {
    let mut c = ConPausa::con_arquero(false, Some(0));
    c.tick();
    assert_eq!(c.lazo.guard.keeper(), 0);
    // La asignación lleva el puesto de arquero al `keeper_id` de los parámetros (el 2).
    c.tx.try_send(OrdenRutina::Preparar { rutina: rutina_de_arquero(), opciones: opciones(100, false) }).unwrap();
    c.tick();
    c.tick();
    assert_eq!(c.d.fase(), Some(Fase::Preparando));
    assert_eq!((c.d.arquero(), c.lazo.guard.keeper()), (Some(2), 2));
    // Una rutina fallida no opina: queda el 2.
    c.d.provocar_panico();
    c.tick();
    assert_eq!(c.d.fase(), Some(Fase::Abortada));
    assert_eq!((c.d.arquero(), c.lazo.guard.keeper()), (None, 2));
    // Al cerrarla vuelve el arquero del coach.
    c.tx.try_send(OrdenRutina::Cerrar).unwrap();
    c.tick();
    c.tick();
    assert_eq!(c.d.fase(), None);
    assert_eq!((c.d.arquero(), c.lazo.guard.keeper()), (Some(0), 0));
}
