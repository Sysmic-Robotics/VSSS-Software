//! Aceptación de motion y skills contra FIRASim: casos fijos (guion de skills + pose
//! inicial + pelota) que corren por el mismo `run_control_loop` que `main` y `skill_test`,
//! con métricas y criterio de éxito por caso, y un resumen de los casos de navegación.
//!
//! Uso (FIRASim corriendo; la configuración es la de producción, por variables de entorno:
//! `VSSL_PARAMS`, `VSSL_BIDIRECTIONAL`, `VSSL_VISION_NOISE`, `VSSL_BORDER_RECOVERY`):
//!
//! ```text
//! cargo run --release --bin motion_bench -- all
//! VSSL_VISION_NOISE=1 VSSL_BIDIRECTIONAL=1 cargo run --release --bin motion_bench -- all --csv-dir /tmp/mb
//! cargo run --release --bin motion_bench -- --list
//! cargo run --release --bin motion_bench -- goto_side mark_point
//! cargo run --release --bin motion_bench -- skills --repeat 20 --out r.jsonl --csv-dir csv --csv-failures-only
//! ```
//!
//! El robot de prueba es el azul 0 (defiende la izquierda), salvo en los casos de GoalKeep,
//! donde es el arquero de los parámetros (`coach.keeper_id`); los otros cinco quedan fuera de
//! la cancha. Imprime una línea JSON por repetición y al final los resúmenes.
//!
//! - `--repeat N`: N repeticiones por caso; la 0 con la pose del caso y las demás con un
//!   jitter determinista (±0.02 m, ±10°).
//! - `--out FILE`: agrega cada repetición a un JSONL y, al relanzar con el mismo archivo,
//!   salta las que ya tienen resultado (reanudar tras una caída de FIRASim).
//! - Falla de infraestructura (FIRASim caído, sin datos, física explotada): no cuenta en la
//!   tasa y se reintenta; si no se recupera, el binario termina con código 3.
//! - `--csv-dir DIR` escribe un CSV por tick y por repetición; con `--csv-failures-only`,
//!   solo de las repeticiones que fallan.

use glam::Vec2;
use rustengine::coach::{RefereeCommand, SharedReferee, SkillChoice};
use rustengine::coach::referee::{REFEREE_ADDR_DEFAULT, REFEREE_ADDR_ENV, apply_command};
use rustengine::control_loop::{ControlLoopConfig, TickDecider, TickRecord, run_control_loop};
use rustengine::motion::{Motion, MotionConfig};
use rustengine::params::params;
use rustengine::radio::{FIRASimClient, RadioTarget, TeleportItem};
use rustengine::skills::zones::AreaRect;
use rustengine::skills::{
    ApproachAlignedSkill, BlockLineSkill, DefendGoalLineSkill, SkillConfig, SkillId, clear_direction,
};
use rustengine::vision::VisionSource;
use rustengine::world::World;
use serde_json::{Value, json};
use std::collections::HashSet;
use std::io::Write;
use std::sync::{Arc, Mutex, atomic::AtomicBool};
use std::time::Duration;

/// Fase de un guion: skill, target y duración (s).
type Phase = (SkillId, (f32, f32), f64);

/// Criterio de éxito de un caso.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Goal {
    /// Llega al destino de la skill (ver `arrived`).
    Arrive,
    /// Llega, no toca el área propia en ningún tick y el destino que elige la skill nunca
    /// la toca (BlockLine: bug B2).
    ArriveNoArea,
    /// No toca el área propia en ningún tick.
    NoArea,
    /// La pelota se acerca al objetivo al menos `min_prog` m y su desplazamiento forma a lo
    /// sumo `max_dir` grados con la dirección pelota → objetivo (última fase del guion).
    Kick { min_prog: f32, max_dir: f32 },
    /// Como `Kick`, pero el desvío se mide contra la **dirección efectiva** de Clear (la
    /// legal, `clear_direction` con el área propia): con la pelota frente al arco, la
    /// dirección al objetivo deja el staging dentro del área.
    ClearKick { min_prog: f32, max_dir: f32 },
    /// No empuja la pelota de costado: se mueve menos de 0.03 m o sale bien hacia el objetivo.
    NoSidePush,
    /// Arquero en su punto de defensa mirando a la pelota.
    Keeper,
    /// Spin: desde 1.0 s, velocidad angular medida con el signo de `target.x` y magnitud
    /// ≥ 80 % de `SkillConfig::default().spin_omega`; traslación ≤ 0.03 m en todo el caso.
    Spin,
    /// Quieto: comando cero en todos los ticks y desplazamiento ≤ 0.01 m desde 0.3 s.
    Still,
    /// Llega y termina con la cara del modo a < 20° de la pelota (Mark).
    ArriveFacingBall,
    /// Llega sin mover la pelota más de 0.03 m (GoTo con la pelota en el camino: A4).
    ArriveNoBallPush,
    /// Árbitro: en la ventana HALT/STOP la traslación comandada es cero (en HALT también
    /// `omega`) y el robot no se desplaza después de frenar (desde 1.0 s); al salir, cumple el objetivo
    /// de su skill (`arrived`) sin escapes en los primeros 0.5 s.
    Referee,
    None,
}

#[derive(Clone, Copy, Debug)]
struct Case {
    name: &'static str,
    skill: SkillId,
    target: (f32, f32),
    /// x, y (m) y heading (grados).
    robot: (f32, f32, f32),
    ball: (f32, f32),
    /// Velocidad inicial de la pelota en el teleport (m/s).
    ball_vel: (f32, f32),
    dur: f64,
    /// Tolerancia de llegada (m); en FacePoint, grados.
    tol: f32,
    goal: Goal,
    /// Fases que siguen a la principal (`skill`, `target`, `dur`).
    then: &'static [Phase],
    /// Línea de tiempo del árbitro: (s desde el inicio del caso, comando de texto). Se
    /// envía por UDP al listener real (`run_referee_listener`) que corre en el bench.
    referee: &'static [(f64, &'static str)],
}

const FAR: (f32, f32) = (0.6, 0.5);
const OWN_GOAL: Vec2 = Vec2::new(-0.75, 0.0);
/// Salto de pose entre dos ticks que solo explica una falla del simulador (m).
const POSE_JUMP_M: f32 = 0.3;
/// Reintentos de una repetición que falló por infraestructura.
const INFRA_RETRIES: u32 = 2;
/// Código de salida cuando la infraestructura no se recupera.
const EXIT_INFRA: i32 = 3;

const ARRIVE: Goal = Goal::Arrive;
const KICK_PUSH: Goal = Goal::Kick { min_prog: 0.10, max_dir: 30.0 };
const KICK_SPIN: Goal = Goal::Kick { min_prog: 0.10, max_dir: 45.0 };

#[allow(clippy::too_many_arguments)]
const fn c(name: &'static str, skill: SkillId, target: (f32, f32), robot: (f32, f32, f32), ball: (f32, f32), dur: f64, tol: f32, goal: Goal) -> Case {
    Case { name, skill, target, robot, ball, ball_vel: (0.0, 0.0), dur, tol, goal, then: &[], referee: &[] }
}

#[rustfmt::skip]
const CASES: &[Case] = &[
    c("goto_fwd", SkillId::GoTo, (0.3, 0.0), (-0.4, 0.0, 0.0), FAR, 6.0, 0.08, ARRIVE),
    c("goto_side", SkillId::GoTo, (0.0, 0.3), (0.0, -0.3, 0.0), FAR, 6.0, 0.08, ARRIVE),
    c("goto_back", SkillId::GoTo, (-0.3, 0.0), (0.3, 0.0, 0.0), FAR, 6.0, 0.08, ARRIVE),
    c("goto_short_side", SkillId::GoTo, (0.0, 0.15), (0.0, 0.0, 0.0), FAR, 5.0, 0.08, ARRIVE),
    c("goto_ball", SkillId::GoTo, (0.45, 0.0), (-0.45, 0.02, 0.0), (0.0, 0.0), 6.0, 0.08, Goal::ArriveNoBallPush),
    c("goto_wall_along", SkillId::GoTo, (0.4, -0.55), (-0.4, -0.55, 0.0), FAR, 6.0, 0.08, ARRIVE),
    c("goto_wall_out", SkillId::GoTo, (0.3, -0.2), (-0.3, -0.52, 0.0), FAR, 6.0, 0.08, ARRIVE),
    c("goto_wall_facing", SkillId::GoTo, (0.0, 0.0), (0.0, -0.57, -90.0), FAR, 6.0, 0.08, ARRIVE),
    c("face_90", SkillId::FacePoint, (0.0, 0.5), (0.0, 0.0, 0.0), FAR, 3.0, 5.0, ARRIVE),
    c("face_180", SkillId::FacePoint, (-0.5, 0.0), (0.0, 0.0, 0.0), FAR, 3.0, 5.0, ARRIVE),
    c("chase_fwd", SkillId::ChaseBall, (0.0, 0.0), (-0.4, -0.3, 0.0), (0.2, 0.2), 6.0, 0.075, ARRIVE),
    c("chase_back", SkillId::ChaseBall, (0.0, 0.0), (0.3, 0.0, 0.0), (-0.2, 0.1), 6.0, 0.075, ARRIVE),
    c("b1a_hold_in_area", SkillId::Hold, (0.0, 0.0), (-0.66, 0.0, 90.0), FAR, 4.0, 0.0, ARRIVE),
    c("b1b_diag_into_area", SkillId::GoTo, (-0.72, -0.2), (-0.3, 0.35, 0.0), FAR, 6.0, 0.08, Goal::NoArea),
    c("b1c_goto_through_area", SkillId::GoTo, (-0.66, -0.3), (-0.66, 0.45, -90.0), FAR, 6.0, 0.08, Goal::NoArea),
    // Casos de B2 con tolerancia `arrival_threshold` + 0.01: con 0.08 un robot frenado por
    // el guardia a 6–8 cm de un punto prohibido contaba como llegado.
    c("b2_blockline_open", SkillId::BlockLine, (-0.75, 0.0), (-0.3, 0.0, 0.0), (-0.45, 0.55), 6.0, 0.05, Goal::ArriveNoArea),
    c("b2_blockline_center", SkillId::BlockLine, (-0.75, 0.0), (-0.3, 0.2, 0.0), (0.2, 0.0), 6.0, 0.08, Goal::ArriveNoArea),
    c("b2_blockline_corner", SkillId::BlockLine, (-0.75, 0.0), (-0.3, -0.5, 0.0), (-0.65, -0.5), 6.0, 0.05, Goal::ArriveNoArea),
    c("b3_shoot_side", SkillId::ShootPush, (0.75, 0.0), (0.0, 0.10, -90.0), (0.0, 0.0), 3.0, 0.0, Goal::NoSidePush),
    c("b3_shoot_side25", SkillId::ShootPush, (0.75, 0.0), (0.0, 0.25, -90.0), (0.0, 0.0), 3.0, 0.0, Goal::NoSidePush),
    c("b3_shoot_behind", SkillId::ShootPush, (0.75, 0.0), (-0.12, 0.0, 0.0), (0.0, 0.0), 3.0, 0.0, Goal::Kick { min_prog: 0.30, max_dir: 30.0 }),
    c("approach_front", SkillId::ApproachAligned, (0.75, 0.0), (0.3, 0.05, 180.0), (0.0, 0.0), 8.0, 0.0, ARRIVE),
    c("approach_side", SkillId::ApproachAligned, (0.75, 0.0), (-0.1, -0.35, 90.0), (0.0, 0.0), 8.0, 0.0, ARRIVE),
    c("mark_point", SkillId::Mark, (-0.3, 0.0), (0.2, -0.3, 0.0), (0.3, 0.3), 6.0, 0.08, Goal::ArriveFacingBall),
    c("intercept_static", SkillId::Intercept, (0.0, 0.0), (-0.4, -0.3, 0.0), (0.2, 0.2), 6.0, 0.09, ARRIVE),
    // La pelota arranca rodando (teleport con velocidad) y cruza la cancha hacia abajo.
    Case {
        ball_vel: (-0.15, -0.7),
        ..c("intercept_moving", SkillId::Intercept, (0.0, 0.0), (-0.3, -0.3, 0.0), (0.35, 0.45), 5.0, 0.09, ARRIVE)
    },
    // Pelota en (−0.30, 0). Con la pelota frente al arco, ver `clear_own_arc`.
    c("clear_own", SkillId::Clear, (0.2, 0.45), (-0.2, 0.3, 0.0), (-0.30, 0.0), 6.0, 0.0, KICK_PUSH),
    c("clear_lateral", SkillId::Clear, (0.3, -0.2), (-0.4, 0.0, 0.0), (-0.4, -0.2), 6.0, 0.0, KICK_PUSH),
    c("spinkick_wall", SkillId::SpinKick, (0.0, 0.0), (-0.2, -0.3, 0.0), (0.2, -0.6), 6.0, 0.0, KICK_SPIN),
    c("spinkick_side", SkillId::SpinKick, (0.75, 0.0), (-0.10, 0.065, 90.0), (0.0, 0.0), 5.0, 0.0, KICK_SPIN),
    // Girando junto a la pelota → GoTo a 0.16 m (dentro del radio en que SpinKick daba el
    // giro por vigente) → SpinKick de nuevo: la segunda activación debe acercarse y patear.
    Case {
        then: &[(SkillId::GoTo, (0.0, 0.16), 1.0), (SkillId::SpinKick, (0.75, 0.0), 4.0)],
        ..c("spinkick_interrupted", SkillId::SpinKick, (0.75, 0.0), (0.0, 0.074, 0.0), (0.0, 0.0), 0.07, 0.0, KICK_SPIN)
    },
    c("a4_ball_behind_target", SkillId::GoTo, (0.33, 0.0), (-0.4, 0.0, 0.0), (0.0, 0.0), 6.0, 0.08, Goal::ArriveNoBallPush),
    c("a4_ball_wall", SkillId::GoTo, (0.3, -0.25), (-0.3, -0.5, 0.0), (0.0, -0.56), 6.0, 0.08, Goal::ArriveNoBallPush),
    // Navegación con obstáculos: el camino recto cruza el área propia (GoTo, BlockLine) y la
    // pelota frente al arco deja el staging de Clear dentro del área con la dirección pedida.
    c("goto_around_area", SkillId::GoTo, (-0.66, 0.45), (-0.66, -0.45, 90.0), FAR, 8.0, 0.08, Goal::ArriveNoArea),
    c("blockline_across_area", SkillId::BlockLine, (-0.75, 0.0), (-0.66, -0.45, 90.0), (-0.45, 0.40), 8.0, 0.05, Goal::ArriveNoArea),
    c("clear_own_arc", SkillId::Clear, (0.2, 0.45), (-0.2, 0.3, 0.0), (-0.45, 0.0), 6.0, 0.0, Goal::ClearKick { min_prog: 0.10, max_dir: 30.0 }),
    c("spin_20", SkillId::Spin, (1.0, 0.0), (0.0, 0.0, 0.0), FAR, 3.0, 0.0, Goal::Spin),
    c("spin_cw", SkillId::Spin, (-1.0, 0.0), (0.0, 0.0, 0.0), FAR, 3.0, 0.0, Goal::Spin),
    c("hold_still", SkillId::Hold, (0.0, 0.0), (0.2, -0.2, 30.0), FAR, 3.0, 0.0, Goal::Still),
    c("goalkeep_keeper", SkillId::GoalKeep, (-0.75, 0.0), (-0.4, 0.2, 0.0), (0.0, -0.15), 5.0, 0.0, Goal::Keeper),
    // Árbitro por texto: HALT a velocidad de crucero; HALT y STOP con un jugador de campo
    // dentro del área propia (sin el árbitro, el `ZoneGuard` lo sacaría).
    Case {
        referee: &[(0.8, "HALT"), (2.3, "GAME_ON")],
        ..c("halt_game_on", SkillId::GoTo, (0.4, 0.0), (-0.4, 0.0, 0.0), FAR, 5.0, 0.08, Goal::Referee)
    },
    Case {
        referee: &[(0.0, "HALT"), (1.5, "GAME_ON")],
        ..c("halt_in_area", SkillId::Hold, (0.0, 0.0), (-0.66, 0.0, 90.0), FAR, 4.0, 0.0, Goal::Referee)
    },
    Case {
        referee: &[(0.0, "STOP"), (1.5, "GAME_ON")],
        ..c("stop_in_area", SkillId::Hold, (0.0, 0.0), (-0.66, 0.0, 90.0), FAR, 4.0, 0.0, Goal::Referee)
    },
];

/// Suite completa de regresión de las 13 skills (alias `suite`): para hitos.
const SUITE: &[&str] = &[
    "goto_side", "goto_back", "goto_short_side", "goto_wall_facing", "goto_ball",
    "a4_ball_behind_target", "b1b_diag_into_area", "face_90", "face_180", "chase_fwd",
    "chase_back", "spin_20", "spin_cw", "approach_front", "approach_side", "b3_shoot_side",
    "b3_shoot_side25", "b3_shoot_behind", "intercept_static", "intercept_moving",
    "b2_blockline_open", "b2_blockline_corner", "b2_blockline_center", "goalkeep_keeper",
    "clear_own", "clear_lateral", "spinkick_wall", "spinkick_side", "spinkick_interrupted",
    "mark_point", "hold_still", "b1a_hold_in_area", "goto_around_area", "blockline_across_area",
    "clear_own_arc",
];

/// Suite rápida, un caso por skill (alias `suite-rapida`): para cada change.
const SUITE_QUICK: &[&str] = &[
    "goto_side", "face_90", "chase_fwd", "spin_20", "approach_side", "b3_shoot_behind",
    "intercept_moving", "b2_blockline_open", "goalkeep_keeper", "clear_lateral",
    "spinkick_wall", "mark_point", "hold_still",
];

/// Casos del árbitro (alias `referee` en la línea de comandos).
const REFEREE_CASES: &[&str] = &["halt_game_on", "halt_in_area", "stop_in_area"];

/// Casos de navegación del resumen. `b2_blockline_open` se informa aparte: es el caso del
/// bug B2 de BlockLine (punto de bloqueo en la zona prohibida).
const NAV: &[&str] = &[
    "goto_fwd", "goto_side", "goto_back", "goto_short_side", "goto_ball", "goto_wall_along",
    "goto_wall_out", "goto_wall_facing", "chase_fwd", "chase_back", "mark_point",
    "intercept_static", "approach_front", "approach_side", "b2_blockline_center",
    "a4_ball_behind_target", "a4_ball_wall",
];

/// Casos de los bugs de skills (alias `skills` en la línea de comandos).
const SKILL_CASES: &[&str] = &[
    "b2_blockline_open", "b2_blockline_corner", "b3_shoot_side", "b3_shoot_side25",
    "b3_shoot_behind", "clear_own", "clear_lateral", "spinkick_wall", "spinkick_side",
    "spinkick_interrupted", "goalkeep_keeper",
];

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Row {
    t: f64,
    x: f32,
    y: f32,
    th: f64,
    bx: f32,
    by: f32,
    vx: f64,
    vy: f64,
    w: f64,
    escaping: bool,
    /// Fase del guion vigente en el tick.
    phase: usize,
}

#[derive(Clone, Debug, Default)]
struct Metrics {
    arrived: Option<f64>,
    path: f32,
    turn_deg: f64,
    escape_ticks: usize,
    area_ticks: usize,
    max_pen: f32,
    ball_disp: f32,
}

fn phases(case: &Case) -> Vec<Phase> {
    let mut p = vec![(case.skill, case.target, case.dur)];
    p.extend_from_slice(case.then);
    p
}

fn total_dur(case: &Case) -> f64 {
    phases(case).iter().map(|p| p.2).sum()
}

/// Índice de la fase vigente en el tick `tick` (desde 0) del caso.
fn phase_at(case: &Case, tick: u32) -> usize {
    let p = phases(case);
    let mut end = 0.0;
    for (i, ph) in p.iter().enumerate() {
        end += ph.2;
        if (tick as f64) < (end * 60.0).round() {
            return i;
        }
    }
    p.len() - 1
}

/// Robot de prueba: el arquero en los casos de GoalKeep (el `ZoneGuard` solo a él lo deja
/// entrar al área propia) y el azul 0 en el resto.
fn robot_id(case: &Case) -> u32 {
    if case.skill == SkillId::GoalKeep { params().coach.keeper_id as u32 } else { 0 }
}

/// Estado del robot de prueba (azul `rid`) en el `World`, cuya firma es `(id, equipo)`.
fn test_robot(world: &World, rid: u32) -> Option<&rustengine::world::RobotState> {
    world.get_robot_state(rid as i32, 0)
}

/// Pose inicial de la repetición `rep`: la del caso en la 0 y, en las demás, perturbada con
/// un jitter determinista (±0.02 m en x e y, ±10° de heading) sembrado con el nombre y `rep`.
fn start_pose(case: &Case, rep: u32) -> (f32, f32, f32) {
    let (x, y, deg) = case.robot;
    if rep == 0 {
        return case.robot;
    }
    let hash = case.name.bytes().fold(0x811c_9dc5u32, |h, b| (h ^ b as u32).wrapping_mul(0x0100_0193));
    let mut s = (hash ^ rep.wrapping_mul(0x9e37_79b9)) | 1;
    let mut u = || {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        s as f32 / u32::MAX as f32 * 2.0 - 1.0
    };
    (x + 0.02 * u(), y + 0.02 * u(), deg + 10.0 * u())
}

/// Motivo por el que las filas de una repetición no sirven para juzgar a la skill (falla de
/// la infraestructura, no del robot), o `None` si sirven.
fn infra_reason(rows: &[Row]) -> Option<&'static str> {
    if rows.is_empty() {
        return Some("sin datos de visión del robot de prueba");
    }
    if rows.iter().any(|r| !(r.x.is_finite() && r.y.is_finite() && r.th.is_finite())) {
        return Some("pose no finita");
    }
    let jump = rows.windows(2).any(|w| (Vec2::new(w[1].x, w[1].y) - Vec2::new(w[0].x, w[0].y)).length() > POSE_JUMP_M);
    jump.then_some("salto de pose (física del simulador)")
}

/// Punto que el robot debe alcanzar en el tick `r`, según la skill (None si la skill no
/// tiene un destino de posición: FacePoint, Hold, empuje, giro).
fn reference(case: &Case, r: &Row) -> Option<Vec2> {
    let target = Vec2::new(case.target.0, case.target.1);
    let ball = Vec2::new(r.bx, r.by);
    match case.skill {
        SkillId::GoTo | SkillId::Mark => Some(target),
        SkillId::ChaseBall | SkillId::Intercept => Some(ball),
        SkillId::ApproachAligned => ApproachAlignedSkill::new(target).staging_point(ball),
        SkillId::BlockLine => BlockLineSkill::new(target).block_point(ball),
        _ => None,
    }
}

/// Error de heading (rad, ≥ 0) hacia `dir`, plegado a ±90° si el robot es bidireccional.
fn heading_err(dir: Vec2, th: f64, bidirectional: bool) -> f64 {
    let e = Motion::normalize_angle((dir.y as f64).atan2(dir.x as f64) - th);
    if bidirectional { Motion::fold_bidirectional(e) } else { e }.abs()
}

/// ¿El robot cumple el objetivo del caso en el tick `r`?
fn arrived(case: &Case, r: &Row, bidirectional: bool) -> bool {
    let p = Vec2::new(r.x, r.y);
    match case.skill {
        SkillId::FacePoint => {
            heading_err(Vec2::new(case.target.0, case.target.1) - p, r.th, bidirectional).to_degrees() < case.tol as f64
        }
        SkillId::Hold => !AreaRect::own(1.0).touches(p),
        SkillId::ApproachAligned => {
            let tol = &params().skills;
            let line = Vec2::new(case.target.0, case.target.1) - Vec2::new(r.bx, r.by);
            reference(case, r).is_some_and(|s| (s - p).length() <= tol.approach_pos_tol)
                && heading_err(line, r.th, bidirectional) <= tol.approach_angle_tol
        }
        _ => reference(case, r).is_some_and(|q| (q - p).length() < case.tol),
    }
}

fn metrics(case: &Case, rows: &[Row], bidirectional: bool) -> Metrics {
    let mut m = Metrics::default();
    let area = AreaRect::own(1.0);
    let (mut path, mut turn) = (0.0f32, 0.0f64);
    for (i, r) in rows.iter().enumerate() {
        if i > 0 {
            let q = &rows[i - 1];
            path += (Vec2::new(r.x, r.y) - Vec2::new(q.x, q.y)).length();
            turn += Motion::normalize_angle(r.th - q.th).abs();
        }
        m.escape_ticks += r.escaping as usize;
        if area.touches(Vec2::new(r.x, r.y)) {
            m.area_ticks += 1;
            m.max_pen = m.max_pen.max(r.x.abs() - 0.56);
        }
        if m.arrived.is_none() && arrived(case, r, bidirectional) {
            m.arrived = Some(r.t - rows[0].t);
            m.path = path;
            m.turn_deg = turn.to_degrees();
        }
    }
    if m.arrived.is_none() {
        m.path = path;
        m.turn_deg = turn.to_degrees();
    }
    if let (Some(a), Some(b)) = (rows.first(), rows.last()) {
        m.ball_disp = (Vec2::new(b.bx, b.by) - Vec2::new(a.bx, a.by)).length();
    }
    m
}

/// Tiro en la última fase del guion: (progreso máximo de la pelota hacia el objetivo (m),
/// desvío (°) de su desplazamiento en ese instante respecto de la dirección pelota →
/// objetivo, desplazamiento máximo (m)). El desvío es `None` si la pelota casi no se movió.
fn kick(case: &Case, rows: &[Row]) -> (f32, Option<f32>, f32) {
    let (b0, target) = kick_start(case, rows);
    kick_against(case, rows, b0.map(|b| target - b).unwrap_or(Vec2::ZERO))
}

/// Pelota al inicio de la última fase del guion (si hay filas) y objetivo de esa fase.
fn kick_start(case: &Case, rows: &[Row]) -> (Option<Vec2>, Vec2) {
    let p = phases(case);
    let last = p.len() - 1;
    let target = Vec2::new(p[last].1.0, p[last].1.1);
    let b0 = rows.iter().find(|r| r.phase == last).map(|r| Vec2::new(r.bx, r.by));
    (b0, target)
}

/// `kick` con el desvío medido contra `want` (una dirección) en vez de pelota → objetivo.
fn kick_against(case: &Case, rows: &[Row], want: Vec2) -> (f32, Option<f32>, f32) {
    let p = phases(case);
    let last = p.len() - 1;
    let target = Vec2::new(p[last].1.0, p[last].1.1);
    let mut it = rows.iter().filter(|r| r.phase == last).map(|r| Vec2::new(r.bx, r.by));
    let Some(b0) = it.next() else { return (0.0, None, 0.0) };
    let d0 = (target - b0).length();
    let (mut best, mut at, mut disp) = (d0, b0, 0.0f32);
    for b in it {
        disp = disp.max((b - b0).length());
        let d = (target - b).length();
        if d < best {
            best = d;
            at = b;
        }
    }
    let moved = at - b0;
    let dir = (moved.length() > 0.02 && want.length() > 1e-4)
        .then(|| (moved.dot(want) / (moved.length() * want.length())).clamp(-1.0, 1.0).acos().to_degrees());
    (d0 - best, dir, disp)
}

/// Criterio de éxito del caso (`None` si no tiene).
fn success(case: &Case, rows: &[Row], m: &Metrics, bidirectional: bool) -> Option<bool> {
    let kick_ok = |min_prog: f32, max_dir: f32| {
        let (prog, dir, _) = kick(case, rows);
        prog >= min_prog && dir.is_some_and(|d| d <= max_dir)
    };
    Some(match case.goal {
        Goal::Arrive => m.arrived.is_some(),
        Goal::ArriveNoArea => {
            let area = AreaRect::own(1.0);
            let legal = rows.iter().all(|r| reference(case, r).is_none_or(|q| !area.touches(q)));
            m.arrived.is_some() && m.area_ticks == 0 && legal
        }
        Goal::NoArea => m.area_ticks == 0,
        Goal::Kick { min_prog, max_dir } => kick_ok(min_prog, max_dir),
        Goal::ClearKick { min_prog, max_dir } => {
            let (b0, target) = kick_start(case, rows);
            let offset = params().skills.clear_staging_offset;
            let eff = b0.and_then(|b| clear_direction(b, target, offset, &[AreaRect::own(1.0)], Some(Vec2::X)));
            let (prog, dir, _) = kick_against(case, rows, eff.unwrap_or(Vec2::ZERO));
            prog >= min_prog && dir.is_some_and(|d| d <= max_dir)
        }
        Goal::NoSidePush => kick(case, rows).2 < 0.03 || kick_ok(0.05, 20.0),
        Goal::Keeper => {
            let last = rows.last()?;
            let gk = DefendGoalLineSkill::new(OWN_GOAL);
            let spot = Vec2::new(gk.defend_x, last.by.clamp(-gk.goal_half_y, gk.goal_half_y));
            let p = Vec2::new(last.x, last.y);
            (spot - p).length() <= params().motion.arrival_threshold + 0.01
                && heading_err(Vec2::new(last.bx, last.by) - p, last.th, bidirectional).to_degrees() <= 20.0
        }
        Goal::Referee => referee_ok(case, rows, bidirectional),
        Goal::Spin => spin_ok(case, rows),
        Goal::Still => still_ok(rows),
        Goal::ArriveFacingBall => {
            let last = rows.last()?;
            let p = Vec2::new(last.x, last.y);
            m.arrived.is_some() && heading_err(Vec2::new(last.bx, last.by) - p, last.th, bidirectional).to_degrees() < 20.0
        }
        Goal::ArriveNoBallPush => {
            let b0 = rows.first().map(|r| Vec2::new(r.bx, r.by))?;
            let pushed = rows.iter().map(|r| (Vec2::new(r.bx, r.by) - b0).length()).fold(0.0f32, f32::max);
            m.arrived.is_some() && pushed < 0.03
        }
        Goal::None => return None,
    })
}

/// Criterio `Goal::Spin`: velocidad angular media medida desde 1.0 s (Δθ normalizado tick
/// a tick: a 20 rad/s y 60 Hz son 0.33 rad por tick) con el signo de `target.x` y ≥ 80 %
/// de `spin_omega`; el robot no se traslada más de 0.03 m.
fn spin_ok(case: &Case, rows: &[Row]) -> bool {
    let Some(first) = rows.first() else { return false };
    let p0 = Vec2::new(first.x, first.y);
    if rows.iter().any(|r| (Vec2::new(r.x, r.y) - p0).length() > 0.03) {
        return false;
    }
    let win: Vec<&Row> = rows.iter().filter(|r| r.t - first.t >= 1.0).collect();
    let (Some(a), Some(b)) = (win.first(), win.last()) else { return false };
    if b.t <= a.t {
        return false;
    }
    let turned: f64 = win.windows(2).map(|w| Motion::normalize_angle(w[1].th - w[0].th)).sum();
    let omega = turned / (b.t - a.t);
    let want = SkillConfig::default().spin_omega * case.target.0.signum() as f64;
    omega * want > 0.0 && omega.abs() >= 0.8 * want.abs()
}

/// Criterio `Goal::Still`: comando cero en todos los ticks y desplazamiento ≤ 0.01 m desde
/// 0.3 s (después de asentarse el teleport).
fn still_ok(rows: &[Row]) -> bool {
    let Some(t0) = rows.first().map(|r| r.t) else { return false };
    let zero = rows.iter().all(|r| r.vx == 0.0 && r.vy == 0.0 && r.w == 0.0);
    let settled: Vec<Vec2> = rows.iter().filter(|r| r.t - t0 >= 0.3).map(|r| Vec2::new(r.x, r.y)).collect();
    zero && settled.first().is_some_and(|p0| settled.iter().all(|p| (*p - *p0).length() <= 0.01))
}

/// Criterio `Goal::Referee` sobre la primera ventana HALT/STOP de la línea de tiempo.
fn referee_ok(case: &Case, rows: &[Row], bidirectional: bool) -> bool {
    let Some(k) = case.referee.iter().position(|(_, c)| *c != "GAME_ON") else { return false };
    let (t_on, cmd) = case.referee[k];
    let t_off = case.referee.get(k + 1).map_or(f64::INFINITY, |e| e.0);
    let Some(t0) = rows.first().map(|r| r.t) else { return false };
    let rel = |r: &Row| r.t - t0;
    let window: Vec<&Row> = rows.iter().filter(|r| rel(r) >= t_on + 0.10 && rel(r) < t_off).collect();
    let still_cmd = window.iter().all(|r| r.vx == 0.0 && r.vy == 0.0 && (cmd != "HALT" || r.w == 0.0));
    // Quieto desde 1.0 s: con el comando ya en cero, frenando desde ~0.8 m/s en FIRASim la
    // pose (EKF con la latencia del proxy) se pasa y vuelve hasta 4 cm durante ~0.8 s.
    let braked: Vec<Vec2> = window.iter().filter(|r| rel(r) >= t_on + 1.0).map(|r| Vec2::new(r.x, r.y)).collect();
    let still = braked.first().is_none_or(|p0| braked.iter().all(|p| (*p - *p0).length() <= 0.01));
    let after: Vec<&Row> = rows.iter().filter(|r| rel(r) >= t_off).collect();
    let no_escape = after.iter().filter(|r| rel(r) < t_off + 0.5).all(|r| !r.escaping);
    let resumed = after.iter().any(|r| arrived(case, r, bidirectional));
    !window.is_empty() && still_cmd && still && no_escape && resumed
}

/// Índice del próximo comando del árbitro a enviar cuando el caso lleva `rel` s, dados
/// `sent` comandos ya enviados.
fn referee_due(timeline: &[(f64, &str)], sent: usize, rel: f64) -> usize {
    let mut k = sent;
    while k < timeline.len() && timeline[k].0 <= rel {
        k += 1;
    }
    k
}

fn is_valid(v: &Value) -> bool {
    v["infra"] == json!(false)
}

/// Resumen de los casos de navegación sobre la repetición 0: (llegan, total, giro
/// acumulado, ticks de escape, tiempo medio de los que llegan).
fn summary(results: &[Value]) -> (usize, usize, f64, u64, f64) {
    let nav: Vec<&Value> = results
        .iter()
        .filter(|v| is_valid(v) && v["rep"] == json!(0) && NAV.iter().any(|n| v["case"] == json!(n)))
        .collect();
    let times: Vec<f64> = nav.iter().filter_map(|v| v["t_arrive"].as_f64()).collect();
    let mean_t = if times.is_empty() { f64::NAN } else { times.iter().sum::<f64>() / times.len() as f64 };
    (
        times.len(),
        nav.len(),
        nav.iter().filter_map(|v| v["turn_deg"].as_f64()).sum(),
        nav.iter().filter_map(|v| v["escape_ticks"].as_u64()).sum(),
        mean_t,
    )
}

/// Éxitos y repeticiones válidas con criterio del caso `name` (reps < `repeat`).
fn rate(results: &[Value], name: &str, repeat: u32) -> (usize, usize) {
    let judged: Vec<&Value> = results
        .iter()
        .filter(|v| is_valid(v) && v["case"] == json!(name))
        .filter(|v| v["rep"].as_u64().is_some_and(|r| r < repeat as u64) && !v["success"].is_null())
        .collect();
    (judged.iter().filter(|v| v["success"] == json!(true)).count(), judged.len())
}

/// Éxitos y repeticiones válidas por skill, sumando sus casos, en el orden del catálogo.
fn skill_rates(cases: &[&&Case], results: &[Value], repeat: u32) -> Vec<(SkillId, usize, usize)> {
    (0..SkillId::COUNT as u8)
        .filter_map(SkillId::from_u8)
        .filter_map(|skill| {
            let of: Vec<&&&Case> = cases.iter().filter(|c| c.skill == skill).collect();
            if of.is_empty() {
                return None;
            }
            let (k, n) = of.iter().map(|c| rate(results, c.name, repeat)).fold((0, 0), |a, b| (a.0 + b.0, a.1 + b.1));
            Some((skill, k, n))
        })
        .collect()
}

/// Pares (caso, repetición) que ya tienen un resultado válido.
fn done_set(results: &[Value]) -> HashSet<(String, u64)> {
    results
        .iter()
        .filter(|v| is_valid(v))
        .filter_map(|v| Some((v["case"].as_str()?.to_string(), v["rep"].as_u64()?)))
        .collect()
}

/// Decider del guion: la skill de la fase vigente según el tick.
struct ScriptDecider {
    case: Case,
    robot_id: i32,
}

impl TickDecider for ScriptDecider {
    fn decide(&mut self, tick: u32, _world: &World) -> Vec<SkillChoice> {
        let (skill_id, (x, y), _) = phases(&self.case)[phase_at(&self.case, tick)];
        vec![SkillChoice { robot_id: self.robot_id, skill_id, target: Vec2::new(x, y) }]
    }
}

async fn run_case(case: &Case, pose: (f32, f32, f32), referee: Option<SharedReferee>) -> Result<Vec<Row>, String> {
    let client = FIRASimClient::new("127.0.0.1", 20011).await.map_err(|e| e.to_string())?;
    let rid = robot_id(case);
    let mut items: Vec<TeleportItem> = (0u32..2)
        .flat_map(|team| (0u32..3).map(move |id| (team, id)))
        .filter(|&(team, id)| (team, id) != (0, rid))
        .enumerate()
        .map(|(k, (team, id))| TeleportItem::Robot { team, id, x: -0.5 + 0.25 * k as f64, y: 1.0, theta: 0.0 })
        .collect();
    // El teleport lleva la orientación en grados (la unidad del replacement de FIRASim).
    let (x, y, deg) = pose;
    items.push(TeleportItem::Robot { team: 0, id: rid, x: x as f64, y: y as f64, theta: deg as f64 });
    let (bvx, bvy) = case.ball_vel;
    items.push(TeleportItem::Ball { x: case.ball.0 as f64, y: case.ball.1 as f64, vx: bvx as f64, vy: bvy as f64 });
    // El replacement va por UDP y el robot puede traer inercia del caso anterior.
    for _ in 0..5 {
        client.teleport(&items).await.map_err(|e| e.to_string())?;
        tokio::time::sleep(Duration::from_millis(80)).await;
    }
    tokio::time::sleep(Duration::from_millis(300)).await;

    let rows = Arc::new(Mutex::new(Vec::<Row>::new()));
    let sink = rows.clone();
    let script = *case;
    // Árbitro: el caso arranca en GAME_ON (no hereda el comando del anterior) y el hook
    // envía cada comando de la línea de tiempo como texto al listener, por UDP.
    if let Some(r) = &referee {
        apply_command(r, RefereeCommand::GAME_ON);
    }
    let timeline = case.referee;
    let sender = if timeline.is_empty() { None } else { Some(referee_sender()?) };
    let (mut sent, mut t0) = (0usize, None::<f64>);
    let hook = Box::new(move |rec: &TickRecord<'_>| {
        let Some(r) = test_robot(rec.world, rid) else { return };
        // El reloj de la línea de tiempo arranca en la primera fila (robot visible), el
        // mismo origen que usa el criterio `Goal::Referee`.
        if let Some((sock, addr)) = &sender {
            let t = rec.t_ms as f64 / 1000.0;
            let due = referee_due(timeline, sent, t - *t0.get_or_insert(t));
            for (_, cmd) in &timeline[sent..due] {
                if let Err(e) = sock.send_to(cmd.as_bytes(), addr) {
                    eprintln!("[motion_bench] no pude enviar {cmd} al árbitro ({addr}): {e}");
                }
            }
            sent = due;
        }
        let b = rec.world.get_ball_state().position;
        let k = rec.commands.iter().position(|c| c.id == rid as i32 && c.team == 0);
        let (vx, vy, w) = k.map(|k| (rec.commands[k].vx, rec.commands[k].vy, rec.commands[k].omega)).unwrap_or_default();
        let escaping = k.and_then(|k| rec.escaping.get(k).copied()).unwrap_or(false);
        sink.lock().unwrap().push(Row {
            t: rec.t_ms as f64 / 1000.0,
            x: r.position.x,
            y: r.position.y,
            th: r.orientation,
            bx: b.x,
            by: b.y,
            vx,
            vy,
            w,
            escaping,
            // `rec.tick` ya cuenta el tick en curso; el decider lo vio como `tick - 1`.
            phase: phase_at(&script, rec.tick.saturating_sub(1)),
        });
    });
    let cfg = ControlLoopConfig {
        own_team: 0,
        num_robots: 3,
        vision_source: VisionSource::FiraSim,
        radio_target: RadioTarget::FiraSim,
        max_ticks: Some((total_dur(case) * 60.0).round() as u32),
        vision_timeout: Some(Duration::from_secs(3)),
        referee,
    };
    let decider = Box::new(ScriptDecider { case: *case, robot_id: rid as i32 });
    run_control_loop(cfg, decider, Some(hook), None, Arc::new(AtomicBool::new(false)))
        .await
        .map_err(|e| format!("{e:?}"))?;
    let out = rows.lock().unwrap().clone();
    Ok(out)
}

/// Socket y dirección para mandar comandos de texto al listener del árbitro (la misma
/// dirección que escucha: `VSSL_REFEREE_ADDR` o el grupo multicast por defecto).
fn referee_sender() -> Result<(std::net::UdpSocket, std::net::SocketAddr), String> {
    let text = std::env::var(REFEREE_ADDR_ENV).ok().filter(|s| !s.trim().is_empty());
    let addr: std::net::SocketAddr = text.as_deref().unwrap_or(REFEREE_ADDR_DEFAULT).trim().parse().map_err(|e| format!("{REFEREE_ADDR_ENV}: {e}"))?;
    let sock = std::net::UdpSocket::bind("0.0.0.0:0").map_err(|e| e.to_string())?;
    sock.set_multicast_loop_v4(true).map_err(|e| e.to_string())?;
    Ok((sock, addr))
}

fn write_csv(path: &str, rows: &[Row]) -> std::io::Result<()> {
    let mut f = std::fs::File::create(path)?;
    writeln!(f, "t,x,y,theta,ball_x,ball_y,cmd_vx,cmd_vy,cmd_omega,v_body,escaping")?;
    for r in rows {
        let vb = r.vx * r.th.cos() + r.vy * r.th.sin();
        writeln!(
            f,
            "{:.3},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{}",
            r.t, r.x, r.y, r.th, r.bx, r.by, r.vx, r.vy, r.w, vb, r.escaping as u8
        )?;
    }
    Ok(())
}

/// Imprime la línea y la agrega al JSONL de `--out` (si hay).
fn emit(line: &Value, out: &mut Option<std::fs::File>) {
    println!("{line}");
    if let Some(f) = out
        && let Err(e) = writeln!(f, "{line}").and_then(|_| f.flush())
    {
        eprintln!("[motion_bench] no pude escribir --out: {e}");
    }
}

#[tokio::main]
async fn main() {
    let mut names = Vec::new();
    let (mut csv_dir, mut tag, mut out_path) = (None::<String>, String::new(), None::<String>);
    let (mut repeat, mut failures_only) = (1u32, false);
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--list" => {
                CASES.iter().for_each(|c| println!("{}", c.name));
                return;
            }
            "--csv-dir" => csv_dir = args.next(),
            "--tag" => tag = args.next().unwrap_or_default(),
            "--out" => out_path = args.next(),
            "--csv-failures-only" => failures_only = true,
            "--repeat" => match args.next().and_then(|n| n.parse().ok()).filter(|&n: &u32| n > 0) {
                Some(n) => repeat = n,
                None => {
                    eprintln!("[motion_bench] --repeat espera un entero > 0");
                    std::process::exit(2);
                }
            },
            other => names.push(other.to_string()),
        }
    }
    rustengine::params::TeamParams::install_or_exit("motion_bench");
    let bidirectional = MotionConfig::from_env().bidirectional;
    let all = names.is_empty() || names.iter().any(|n| n == "all");
    let skills = names.iter().any(|n| n == "skills");
    let referee_alias = names.iter().any(|n| n == "referee");
    let suite = names.iter().any(|n| n == "suite");
    let suite_quick = names.iter().any(|n| n == "suite-rapida");
    let selected: Vec<&Case> = CASES
        .iter()
        .filter(|c| {
            all || names.iter().any(|n| n == c.name)
                || (skills && SKILL_CASES.contains(&c.name))
                || (referee_alias && REFEREE_CASES.contains(&c.name))
                || (suite && SUITE.contains(&c.name))
                || (suite_quick && SUITE_QUICK.contains(&c.name))
        })
        .collect();
    if selected.is_empty() {
        eprintln!("[motion_bench] ningún caso coincide; ver --list");
        std::process::exit(2);
    }

    // Reanudación: las repeticiones con resultado válido en `--out` no se vuelven a correr.
    let mut results: Vec<Value> = out_path
        .as_ref()
        .and_then(|p| std::fs::read_to_string(p).ok())
        .map(|s| s.lines().filter_map(|l| serde_json::from_str(l).ok()).collect())
        .unwrap_or_default();
    let done = done_set(&results);
    let mut out = out_path.as_ref().map(|p| {
        std::fs::OpenOptions::new().create(true).append(true).open(p).unwrap_or_else(|e| {
            eprintln!("[motion_bench] no pude abrir {p}: {e}");
            std::process::exit(2);
        })
    });

    // Listener real del árbitro, solo si algún caso lo usa (el resto corre sin árbitro).
    let referee = selected.iter().any(|c| !c.referee.is_empty()).then(|| {
        let shared = rustengine::coach::new_shared_referee();
        tokio::spawn(rustengine::coach::run_referee_listener(shared.clone()));
        shared
    });
    if referee.is_some() {
        tokio::time::sleep(Duration::from_millis(300)).await;
    }

    for case in &selected {
        for rep in 0..repeat {
            if done.contains(&(case.name.to_string(), rep as u64)) {
                continue;
            }
            let pose = start_pose(case, rep);
            let mut attempt = 0;
            let line = loop {
                let shared = if case.referee.is_empty() { None } else { referee.clone() };
                let res = run_case(case, pose, shared).await;
                let reason = match &res {
                    Ok(rows) => infra_reason(rows).map(str::to_string),
                    Err(e) => Some(e.clone()),
                };
                let Some(reason) = reason else { break (res.unwrap_or_default(), pose) };
                let line = json!({ "case": case.name, "rep": rep, "infra": true, "reason": reason, "attempt": attempt });
                emit(&line, &mut out);
                results.push(line);
                if attempt == INFRA_RETRIES {
                    eprintln!(
                        "[motion_bench] {} rep {rep}: falla de infraestructura tras {} intentos ({reason}); \
                         relanzar FIRASim y reanudar con el mismo --out",
                        case.name,
                        attempt + 1
                    );
                    std::process::exit(EXIT_INFRA);
                }
                attempt += 1;
                tokio::time::sleep(Duration::from_secs(2)).await;
            };
            let (rows, (x0, y0, deg0)) = line;
            let m = metrics(case, &rows, bidirectional);
            let ok = success(case, &rows, &m, bidirectional);
            if let Some(dir) = &csv_dir
                && (!failures_only || ok == Some(false))
            {
                let path = format!("{dir}/{}{tag}_r{rep}.csv", case.name);
                if let Err(e) = write_csv(&path, &rows) {
                    eprintln!("[motion_bench] no pude escribir {path}: {e}");
                }
            }
            let last = rows.last().copied().unwrap_or_default();
            let (prog, dir, _) = kick(case, &rows);
            let line = json!({
                "case": case.name, "rep": rep, "infra": false, "success": ok,
                "arrived": m.arrived.is_some(), "t_arrive": m.arrived,
                "path": m.path, "turn_deg": m.turn_deg.round(), "escape_ticks": m.escape_ticks,
                "area_ticks": m.area_ticks, "max_pen": m.max_pen, "ball_disp": m.ball_disp,
                "ball_prog": prog, "ball_dir_deg": dir.map(f32::round),
                "start": [x0, y0, deg0.round()],
                "final": [last.x, last.y, last.th.to_degrees().round()],
            });
            emit(&line, &mut out);
            results.push(line);
        }
    }

    let names: HashSet<&str> = selected.iter().map(|c| c.name).collect();
    results.retain(|v| v["case"].as_str().is_some_and(|n| names.contains(n)));
    let (ok, n, turn, esc, mean_t) = summary(&results);
    if n > 0 {
        println!("\nresumen ({} casos de navegación, rep 0, bidireccional={bidirectional}):", n);
        println!("  llegan {ok}/{n} · giro hasta llegar {turn:.0}° · ticks de escape {esc} · t medio {mean_t:.2} s");
    }
    if let Some(v) = results.iter().find(|v| is_valid(v) && v["case"] == json!("b2_blockline_open") && v["rep"] == json!(0)) {
        println!(
            "  b2_blockline_open (aparte): penetración {:.3} m · {} ticks en el área · {} ticks de escape",
            v["max_pen"].as_f64().unwrap_or(0.0),
            v["area_ticks"],
            v["escape_ticks"]
        );
    }
    let judged: Vec<&&Case> = selected.iter().filter(|c| c.goal != Goal::None).collect();
    if repeat > 1 || judged.iter().any(|c| SKILL_CASES.contains(&c.name)) {
        println!("\néxito por caso ({repeat} repeticiones, bidireccional={bidirectional}):");
        for c in &judged {
            let (k, n) = rate(&results, c.name, repeat);
            let skill = if SKILL_CASES.contains(&c.name) { " [skill]" } else { "" };
            println!("  {:24} éxito {k}/{n}{skill}", c.name);
        }
    }
    let by_skill = skill_rates(&judged, &results, repeat);
    if by_skill.len() > 1 {
        println!("\néxito por skill ({repeat} repeticiones, bidireccional={bidirectional}):");
        for (skill, k, n) in by_skill {
            println!("  {:16} éxito {k}/{n}", format!("{skill:?}"));
        }
    }
    let lost = results.iter().filter(|v| v["infra"] == json!(true)).count();
    println!("\nrepeticiones perdidas por infraestructura (reintentadas, no cuentan): {lost}");
}

#[cfg(test)]
mod tests {
    use super::*;

    fn case(name: &str) -> Case {
        *CASES.iter().find(|c| c.name == name).unwrap()
    }

    fn row(t: f64, x: f32, y: f32, th: f64) -> Row {
        Row { t, x, y, th, bx: 0.6, by: 0.5, ..Default::default() }
    }

    fn ball_row(t: f64, bx: f32, by: f32) -> Row {
        Row { t, x: -0.3, y: 0.0, bx, by, ..Default::default() }
    }

    #[test]
    fn arrival_time_path_and_turn_until_arrival() {
        // goto_fwd: destino (0.3, 0) con tolerancia 0.08.
        let rows = vec![
            row(0.0, -0.4, 0.0, 0.0),
            row(0.5, 0.0, 0.0, 0.5),
            row(1.0, 0.25, 0.0, 0.0),
            row(1.5, 0.3, 0.0, 1.0),
        ];
        let m = metrics(&case("goto_fwd"), &rows, false);
        assert_eq!(m.arrived, Some(1.0));
        assert!((m.path - 0.65).abs() < 1e-6);
        assert!((m.turn_deg - 1.0f64.to_degrees()).abs() < 1e-9, "giro hasta llegar: {}", m.turn_deg);
        assert_eq!(success(&case("goto_fwd"), &rows, &m, false), Some(true));
    }

    #[test]
    fn escapes_and_area_penetration_are_counted() {
        let mut rows = vec![row(0.0, -0.5, 0.0, 0.0), row(0.1, -0.60, 0.0, 0.0), row(0.2, -0.62, 0.1, 0.0)];
        rows[1].escaping = true;
        rows[2].escaping = true;
        let c = case("b1b_diag_into_area");
        let m = metrics(&c, &rows, false);
        assert_eq!(m.escape_ticks, 2);
        assert_eq!(m.area_ticks, 2);
        assert!((m.max_pen - 0.06).abs() < 1e-6);
        assert_eq!(m.arrived, None);
        assert_eq!(success(&c, &rows, &m, false), Some(false));
    }

    #[test]
    fn face_point_folds_in_bidirectional() {
        // face_180: el destino está atrás; con dos caras ya está alineado.
        let rows = vec![row(0.0, 0.0, 0.0, 0.0)];
        assert_eq!(metrics(&case("face_180"), &rows, true).arrived, Some(0.0));
        assert_eq!(metrics(&case("face_180"), &rows, false).arrived, None);
    }

    #[test]
    fn summary_only_counts_navigation_cases_on_rep_zero() {
        let line = |case: &str, rep: u32, t: Option<f64>, turn: f64, esc: u64| {
            json!({ "case": case, "rep": rep, "infra": false, "t_arrive": t, "turn_deg": turn, "escape_ticks": esc })
        };
        let results = vec![
            line("goto_fwd", 0, Some(1.0), 90.0, 0),
            line("goto_side", 0, None, 30.0, 26),
            line("goto_side", 1, Some(2.0), 30.0, 0),
            line("spin_20", 0, Some(1.0), 90.0, 0),
            json!({ "case": "goto_back", "rep": 0, "infra": true, "reason": "x" }),
        ];
        let (arr, n, turn, esc, t) = summary(&results);
        assert_eq!((arr, n, esc), (1, 2, 26));
        assert!((turn - 120.0).abs() < 1e-9 && (t - 1.0).abs() < 1e-9);
    }

    #[test]
    fn block_line_success_needs_a_legal_point_reached_closely() {
        let c = case("b2_blockline_center");
        // Pelota al centro: punto (−0.45, 0) legal; llegar a 0.03 basta.
        let rows = vec![Row { x: -0.42, y: 0.0, bx: 0.2, by: 0.0, ..Default::default() }];
        assert_eq!(success(&c, &rows, &metrics(&c, &rows, false), false), Some(true));
        // b2_blockline_open: a 0.07 m del punto ya no cuenta como llegado.
        let c = case("b2_blockline_open");
        let p = BlockLineSkill::new(OWN_GOAL).block_point(Vec2::new(-0.45, 0.55)).unwrap();
        let far = vec![Row { x: p.x + 0.07, y: p.y, bx: -0.45, by: 0.55, ..Default::default() }];
        assert_eq!(success(&c, &far, &metrics(&c, &far, false), false), Some(false));
    }

    #[test]
    fn kick_criterion_toward_target_and_sideways() {
        // b3_shoot_behind: objetivo (0.75, 0), progreso ≥ 0.30 con desvío ≤ 30°.
        let c = case("b3_shoot_behind");
        let good = vec![ball_row(0.0, 0.0, 0.0), ball_row(0.5, 0.2, 0.02), ball_row(1.0, 0.40, 0.07)];
        let m = metrics(&c, &good, false);
        let (prog, dir, _) = kick(&c, &good);
        assert!(prog > 0.39 && dir.unwrap() < 11.0, "prog {prog} dir {dir:?}");
        assert_eq!(success(&c, &good, &m, false), Some(true));
        let side = vec![ball_row(0.0, 0.0, 0.0), ball_row(1.0, 0.0, -0.4)];
        assert_eq!(success(&c, &side, &metrics(&c, &side, false), false), Some(false));
    }

    #[test]
    fn no_side_push_accepts_a_still_ball_and_rejects_a_lateral_push() {
        let c = case("b3_shoot_side");
        let still = vec![ball_row(0.0, 0.0, 0.0), ball_row(1.0, 0.004, -0.003)];
        assert_eq!(success(&c, &still, &metrics(&c, &still, false), false), Some(true));
        // La pelota sale a 90° del objetivo.
        let pushed = vec![ball_row(0.0, 0.0, 0.0), ball_row(1.0, 0.0, -0.3)];
        assert_eq!(success(&c, &pushed, &metrics(&c, &pushed, false), false), Some(false));
    }

    #[test]
    fn keeper_facing_its_own_goal_fails_in_frontal() {
        let c = case("goalkeep_keeper");
        let gk = DefendGoalLineSkill::new(OWN_GOAL);
        let at = |th: f64| vec![Row { x: gk.defend_x, y: -0.15, th, bx: 0.0, by: -0.15, ..Default::default() }];
        let facing_ball = at(0.0);
        let facing_goal = at(std::f64::consts::PI);
        let ok = |rows: &[Row], bidir: bool| success(&c, rows, &metrics(&c, rows, bidir), bidir);
        assert_eq!(ok(&facing_ball, false), Some(true));
        assert_eq!(ok(&facing_goal, false), Some(false));
        assert_eq!(ok(&facing_goal, true), Some(true));
        let off = vec![Row { x: -0.55, y: -0.15, th: 0.0, bx: 0.0, by: -0.15, ..Default::default() }];
        assert_eq!(ok(&off, false), Some(false));
    }

    #[test]
    fn script_picks_the_phase_by_tick_and_kick_uses_the_last_phase() {
        let c = case("spinkick_interrupted");
        // 0.07 s de SpinKick = 4 ticks; después 1 s de GoTo (60 ticks); después SpinKick.
        assert_eq!(phase_at(&c, 0), 0);
        assert_eq!(phase_at(&c, 3), 0);
        assert_eq!(phase_at(&c, 4), 1);
        assert_eq!(phase_at(&c, 63), 1);
        assert_eq!(phase_at(&c, 64), 2);
        assert_eq!(phase_at(&c, 10_000), 2);
        assert_eq!(phases(&c)[phase_at(&c, 30)].0, SkillId::GoTo);
        assert!((total_dur(&c) - 5.07).abs() < 1e-9);
        // Lo que la pelota se mueve en las fases anteriores no cuenta para el tiro.
        let mut rows = vec![ball_row(0.0, 0.0, 0.0), ball_row(0.5, 0.0, -0.3), ball_row(1.0, 0.0, -0.3), ball_row(2.0, 0.3, -0.18)];
        rows[0].phase = 0;
        rows[1].phase = 1;
        rows[2].phase = 2;
        rows[3].phase = 2;
        let (prog, dir, _) = kick(&c, &rows);
        assert!(prog > 0.2 && dir.unwrap() < 20.0, "prog {prog} dir {dir:?}");
        // Un caso de una fase: siempre la 0.
        assert_eq!(phase_at(&case("goto_fwd"), 1_000), 0);
    }

    #[test]
    fn jitter_is_reproducible_and_bounded() {
        let c = case("clear_lateral");
        assert_eq!(start_pose(&c, 0), c.robot);
        assert_eq!(start_pose(&c, 3), start_pose(&c, 3));
        assert_ne!(start_pose(&c, 3), start_pose(&c, 4));
        for rep in 1..50 {
            let (x, y, deg) = start_pose(&c, rep);
            assert!((x - c.robot.0).abs() <= 0.02 && (y - c.robot.1).abs() <= 0.02);
            assert!((deg - c.robot.2).abs() <= 10.0);
        }
    }

    #[test]
    fn infrastructure_failures_are_told_apart() {
        assert!(infra_reason(&[]).is_some());
        let normal: Vec<Row> = (0..60).map(|i| row(i as f64 / 60.0, -0.4 + 0.01 * i as f32, 0.0, 0.0)).collect();
        assert_eq!(infra_reason(&normal), None);
        let mut jump = normal.clone();
        jump[30].x += 0.5;
        assert!(infra_reason(&jump).is_some());
        let mut nan = normal;
        nan[10].y = f32::NAN;
        assert!(infra_reason(&nan).is_some());
    }

    #[test]
    fn resume_skips_valid_results_but_not_infrastructure_failures() {
        let results = vec![
            json!({ "case": "clear_lateral", "rep": 0, "infra": false, "success": true }),
            json!({ "case": "clear_lateral", "rep": 1, "infra": true, "reason": "x" }),
            json!({ "case": "clear_lateral", "rep": 2, "infra": false, "success": false }),
        ];
        let done = done_set(&results);
        assert!(done.contains(&("clear_lateral".to_string(), 0)));
        assert!(!done.contains(&("clear_lateral".to_string(), 1)));
        assert!(done.contains(&("clear_lateral".to_string(), 2)));
        assert_eq!(rate(&results, "clear_lateral", 3), (1, 2));
        assert_eq!(rate(&results, "clear_lateral", 1), (1, 1));
    }

    #[test]
    fn goalkeep_cases_use_the_keeper() {
        assert_eq!(robot_id(&case("goalkeep_keeper")), params().coach.keeper_id as u32);
        assert_eq!(robot_id(&case("goto_fwd")), 0);
        // El hook lee al azul `rid`, no al robot `0` de otro equipo.
        let mut w = World::new(3, 3);
        w.update_robot(2, 0, Vec2::new(-0.6, 0.1), 0.0, Vec2::ZERO, 0.0);
        assert_eq!(test_robot(&w, 2).map(|r| r.position), Some(Vec2::new(-0.6, 0.1)));
        assert!(test_robot(&w, 0).is_none());
    }

    /// Filas a 60 Hz: avanza hasta `t_on`, frena 0.2 s sin comando y queda quieto hasta
    /// `t_off`; después avanza otra vez hacia +x. `cmd_in_window` es el comando en HALT/STOP.
    fn referee_rows(t_on: f64, t_off: f64, cmd_in_window: (f64, f64, f64)) -> Vec<Row> {
        let mut rows = Vec::new();
        let mut x = -0.4f32;
        for k in 0..300 {
            let t = k as f64 / 60.0;
            let (vx, vy, w) = if t < t_on || t >= t_off { (0.5, 0.0, 0.0) } else { cmd_in_window };
            if t < t_on + 0.2 || t >= t_off {
                x += 0.5 / 60.0;
            }
            rows.push(Row { t, x, y: 0.0, th: 0.0, bx: 0.6, by: 0.5, vx, vy, w, ..Default::default() });
        }
        rows
    }

    #[test]
    fn referee_criterion_needs_zero_command_stillness_and_resume() {
        let c = case("halt_game_on"); // HALT 0.8 s → GAME_ON 2.3 s, GoTo a (0.4, 0)
        let ok = referee_rows(0.8, 2.3, (0.0, 0.0, 0.0));
        assert!(ok.iter().any(|r| r.x >= 0.33), "la fila de prueba llega al destino");
        assert_eq!(success(&c, &ok, &metrics(&c, &ok, false), false), Some(true));
        // Un tick con avance dentro de la ventana de HALT → falla.
        let mut moved = ok.clone();
        moved[100].vx = 0.3;
        assert_eq!(success(&c, &moved, &metrics(&c, &moved, false), false), Some(false));
        // En HALT tampoco vale girar.
        let turning = referee_rows(0.8, 2.3, (0.0, 0.0, 1.0));
        assert_eq!(success(&c, &turning, &metrics(&c, &turning, false), false), Some(false));
        // En STOP sí: giro sin traslación cumple.
        let stop = Case { referee: &[(0.8, "STOP"), (2.3, "GAME_ON")], ..c };
        let rows = referee_rows(0.8, 2.3, (0.0, 0.0, 1.0));
        assert_eq!(success(&stop, &rows, &metrics(&stop, &rows, false), false), Some(true));
        // Desplazarse durante HALT ya asentado (aunque el comando sea cero) → falla.
        let mut drift = ok.clone();
        for r in drift.iter_mut().filter(|r| r.t > 1.8 && r.t < 2.3) {
            r.x += (r.t - 1.8) as f32 * 0.1;
        }
        assert_eq!(success(&c, &drift, &metrics(&c, &drift, false), false), Some(false));
    }

    #[test]
    fn referee_commands_are_sent_when_their_time_comes() {
        let tl = [(0.0, "HALT"), (1.5, "GAME_ON")];
        assert_eq!(referee_due(&tl, 0, 0.0), 1, "el HALT de t = 0 sale en el primer tick");
        assert_eq!(referee_due(&tl, 1, 1.49), 1);
        assert_eq!(referee_due(&tl, 1, 1.5), 2);
        assert_eq!(referee_due(&tl, 2, 9.0), 2);
        assert_eq!(referee_due(&[], 0, 1.0), 0);
        assert!(REFEREE_CASES.iter().all(|n| !case(n).referee.is_empty() && case(n).goal == Goal::Referee));
    }

    /// Filas a 60 Hz durante `dur` s girando a `w` rad/s en (x0, 0) con traslación `drift`
    /// m/s en +x.
    fn spin_rows(dur: f64, w: f64, drift: f32) -> Vec<Row> {
        (0..(dur * 60.0) as usize)
            .map(|k| {
                let t = k as f64 / 60.0;
                let th = Motion::normalize_angle(w * t);
                Row { t, x: drift * t as f32, y: 0.0, th, bx: 0.6, by: 0.5, w, ..Default::default() }
            })
            .collect()
    }

    #[test]
    fn spin_criterion_checks_direction_rate_and_translation() {
        let ccw = case("spin_20");
        let cw = case("spin_cw");
        let ok = |c: &Case, rows: &[Row]| success(c, rows, &metrics(c, rows, false), false);
        let fast_ccw = spin_rows(3.0, 18.0, 0.0);
        assert_eq!(ok(&ccw, &fast_ccw), Some(true));
        assert_eq!(ok(&cw, &fast_ccw), Some(false), "sentido contrario");
        assert_eq!(ok(&cw, &spin_rows(3.0, -17.0, 0.0)), Some(true));
        assert_eq!(ok(&ccw, &spin_rows(3.0, 10.0, 0.0)), Some(false), "demasiado lento");
        assert_eq!(ok(&ccw, &spin_rows(3.0, 18.0, 0.02)), Some(false), "se traslada 6 cm");
    }

    #[test]
    fn still_criterion_needs_zero_command_and_no_motion() {
        let c = case("hold_still");
        let ok = |rows: &[Row]| success(&c, rows, &metrics(&c, rows, false), false);
        let still: Vec<Row> = (0..180).map(|k| row(k as f64 / 60.0, 0.2, -0.2, 0.5)).collect();
        assert_eq!(ok(&still), Some(true));
        let mut moved = still.clone();
        for r in moved.iter_mut().skip(120) {
            r.x += 0.03;
        }
        assert_eq!(ok(&moved), Some(false));
        let mut cmd = still.clone();
        cmd[50].vx = 0.05;
        assert_eq!(ok(&cmd), Some(false));
    }

    #[test]
    fn mark_must_end_facing_the_ball() {
        let c = case("mark_point"); // punto (−0.3, 0), pelota (0.3, 0.3)
        let at = |th: f64| vec![Row { t: 0.0, x: -0.3, y: 0.0, th, bx: 0.3, by: 0.3, ..Default::default() }];
        let to_ball = 0.3f64.atan2(0.6);
        let ok = |rows: &[Row], bidir: bool| success(&c, rows, &metrics(&c, rows, bidir), bidir);
        assert_eq!(ok(&at(to_ball), false), Some(true));
        assert_eq!(ok(&at(to_ball + std::f64::consts::PI), false), Some(false), "de espaldas en frontal");
        assert_eq!(ok(&at(to_ball + std::f64::consts::PI), true), Some(true), "de espaldas vale con dos caras");
    }

    #[test]
    fn goto_must_not_push_the_ball_in_its_way() {
        let c = case("a4_ball_behind_target"); // destino (0.33, 0), pelota en (0, 0)
        let mut rows = vec![
            Row { t: 0.0, x: -0.4, y: 0.0, bx: 0.0, by: 0.0, ..Default::default() },
            Row { t: 2.0, x: 0.33, y: 0.0, bx: 0.0, by: 0.0, ..Default::default() },
        ];
        let ok = |rows: &[Row]| success(&c, rows, &metrics(&c, rows, false), false);
        assert_eq!(ok(&rows), Some(true));
        rows[1].bx = 0.10;
        assert_eq!(ok(&rows), Some(false), "llegó empujando la pelota");
    }

    #[test]
    fn obstacle_navigation_cases_are_in_the_full_suite() {
        for n in ["goto_around_area", "blockline_across_area", "clear_own_arc"] {
            assert!(SUITE.contains(&n), "{n} falta en la suite completa");
        }
        assert_eq!(case("goto_ball").goal, Goal::ArriveNoBallPush);
        assert_eq!(case("a4_ball_behind_target").goal, Goal::ArriveNoBallPush);
    }

    #[test]
    fn clear_own_arc_is_measured_against_the_effective_direction() {
        // La pelota sale a 80°: a 45° de la dirección al objetivo (34.7°) y a ~2° de la
        // efectiva (la legal, ~78°), y avanza más de 0.10 m hacia el objetivo → éxito. A 35°
        // (justo hacia el objetivo) queda a 43° de la efectiva → falla; a 100° casi no avanza
        // hacia el objetivo → falla.
        let c = case("clear_own_arc");
        let m = Metrics::default();
        let shot = |deg: f32| {
            let d = Vec2::from_angle(deg.to_radians());
            let rows: Vec<Row> = (0..=10)
                .map(|i| ball_row(i as f64 * 0.1, -0.45 + d.x * 0.03 * i as f32, d.y * 0.03 * i as f32))
                .collect();
            success(&c, &rows, &m, false)
        };
        assert_eq!(shot(80.0), Some(true));
        assert_eq!(shot(65.0), Some(true));
        assert_eq!(shot(35.0), Some(false));
        assert_eq!(shot(100.0), Some(false));
        let (b0, target) = (Vec2::new(-0.45, 0.0), Vec2::new(0.2, 0.45));
        let eff = clear_direction(b0, target, params().skills.clear_staging_offset, &[AreaRect::own(1.0)], Some(Vec2::X))
            .unwrap();
        let eff_deg = eff.y.atan2(eff.x).to_degrees();
        assert!((72.0..=82.0).contains(&eff_deg), "efectiva {eff_deg:.1}°");
    }

    #[test]
    fn suites_cover_the_whole_catalog() {
        let skills = |names: &[&str]| -> Vec<SkillId> { names.iter().map(|n| case(n).skill).collect() };
        let all: Vec<SkillId> = (0..SkillId::COUNT as u8).filter_map(SkillId::from_u8).collect();
        // Rápida: exactamente un caso por skill.
        let quick = skills(SUITE_QUICK);
        assert_eq!(quick.len(), SkillId::COUNT);
        assert!(all.iter().all(|s| quick.iter().filter(|q| *q == s).count() == 1));
        // Completa: todas las skills, todos los casos con criterio.
        let full = skills(SUITE);
        assert!(all.iter().all(|s| full.contains(s)), "falta alguna skill en la suite completa");
        assert!(SUITE.iter().all(|n| case(n).goal != Goal::None));
        assert!(SUITE_QUICK.iter().all(|n| SUITE.contains(n)));
    }

    #[test]
    fn case_names_are_unique_and_lists_exist() {
        let mut names: Vec<&str> = CASES.iter().map(|c| c.name).collect();
        names.sort();
        names.dedup();
        assert_eq!(names.len(), CASES.len());
        assert!(NAV.iter().chain(SKILL_CASES).chain(REFEREE_CASES).chain(SUITE).all(|n| CASES.iter().any(|c| c.name == *n)));
        assert!(SKILL_CASES.iter().all(|n| case(n).goal != Goal::None));
    }
}
