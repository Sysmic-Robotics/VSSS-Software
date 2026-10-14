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
            &self.motion,
            &mut self.recovery,
            &self.guard,
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

