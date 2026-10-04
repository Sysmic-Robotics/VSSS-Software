//! Aceptación de motion contra FIRASim: casos fijos (skill + pose inicial + pelota) que
//! corren por el mismo `run_control_loop` que `main` y `skill_test`, con métricas por caso
//! y un resumen de los casos de navegación.
//!
//! Uso (FIRASim corriendo; la configuración es la de producción, por variables de entorno:
//! `VSSL_PARAMS`, `VSSL_BIDIRECTIONAL`, `VSSL_VISION_NOISE`, `VSSL_BORDER_RECOVERY`):
//!
//! ```text
//! cargo run --release --bin motion_bench -- all
//! VSSL_VISION_NOISE=1 VSSL_BIDIRECTIONAL=1 cargo run --release --bin motion_bench -- all --csv-dir /tmp/mb
//! cargo run --release --bin motion_bench -- --list
//! cargo run --release --bin motion_bench -- goto_side mark_point
//! ```
//!
//! El robot de prueba es el azul 0 (defiende la izquierda); los otros cinco quedan fuera de
//! la cancha. Imprime una línea JSON por caso y al final el resumen.

use glam::Vec2;
use rustengine::control_loop::{ControlLoopConfig, FixedSkillDecider, TickRecord, run_control_loop};
use rustengine::motion::{Motion, MotionConfig};
use rustengine::radio::{FIRASimClient, RadioTarget, TeleportItem};
use rustengine::skills::zones::AreaRect;
use rustengine::skills::{ApproachAlignedSkill, BlockLineSkill, SkillId};
use rustengine::vision::VisionSource;
use std::io::Write;
use std::sync::{Arc, Mutex, atomic::AtomicBool};
use std::time::Duration;

#[derive(Clone, Copy, Debug)]
struct Case {
    name: &'static str,
    skill: SkillId,
    target: (f32, f32),
    /// x, y (m) y heading (grados).
    robot: (f32, f32, f32),
    ball: (f32, f32),
    dur: f64,
    /// Tolerancia de llegada (m); en FacePoint, grados.
    tol: f32,
}

const FAR: (f32, f32) = (0.6, 0.5);

const fn c(name: &'static str, skill: SkillId, target: (f32, f32), robot: (f32, f32, f32), ball: (f32, f32), dur: f64, tol: f32) -> Case {
    Case { name, skill, target, robot, ball, dur, tol }
}

#[rustfmt::skip]
const CASES: &[Case] = &[
    c("goto_fwd", SkillId::GoTo, (0.3, 0.0), (-0.4, 0.0, 0.0), FAR, 6.0, 0.08),
    c("goto_side", SkillId::GoTo, (0.0, 0.3), (0.0, -0.3, 0.0), FAR, 6.0, 0.08),
    c("goto_back", SkillId::GoTo, (-0.3, 0.0), (0.3, 0.0, 0.0), FAR, 6.0, 0.08),
    c("goto_short_side", SkillId::GoTo, (0.0, 0.15), (0.0, 0.0, 0.0), FAR, 5.0, 0.08),
    c("goto_ball", SkillId::GoTo, (0.45, 0.0), (-0.45, 0.02, 0.0), (0.0, 0.0), 6.0, 0.08),
    c("goto_wall_along", SkillId::GoTo, (0.4, -0.55), (-0.4, -0.55, 0.0), FAR, 6.0, 0.08),
    c("goto_wall_out", SkillId::GoTo, (0.3, -0.2), (-0.3, -0.52, 0.0), FAR, 6.0, 0.08),
    c("goto_wall_facing", SkillId::GoTo, (0.0, 0.0), (0.0, -0.57, -90.0), FAR, 6.0, 0.08),
    c("face_90", SkillId::FacePoint, (0.0, 0.5), (0.0, 0.0, 0.0), FAR, 3.0, 5.0),
    c("face_180", SkillId::FacePoint, (-0.5, 0.0), (0.0, 0.0, 0.0), FAR, 3.0, 5.0),
    c("chase_fwd", SkillId::ChaseBall, (0.0, 0.0), (-0.4, -0.3, 0.0), (0.2, 0.2), 6.0, 0.075),
    c("chase_back", SkillId::ChaseBall, (0.0, 0.0), (0.3, 0.0, 0.0), (-0.2, 0.1), 6.0, 0.075),
    c("b1a_hold_in_area", SkillId::Hold, (0.0, 0.0), (-0.66, 0.0, 90.0), FAR, 4.0, 0.0),
    c("b1b_diag_into_area", SkillId::GoTo, (-0.72, -0.2), (-0.3, 0.35, 0.0), FAR, 6.0, 0.08),
    c("b1c_goto_through_area", SkillId::GoTo, (-0.66, -0.3), (-0.66, 0.45, -90.0), FAR, 6.0, 0.08),
    c("b2_blockline_open", SkillId::BlockLine, (-0.75, 0.0), (-0.3, 0.0, 0.0), (-0.45, 0.55), 6.0, 0.08),
    c("b2_blockline_center", SkillId::BlockLine, (-0.75, 0.0), (-0.3, 0.2, 0.0), (0.2, 0.0), 6.0, 0.08),
    c("b3_shoot_side", SkillId::ShootPush, (0.75, 0.0), (0.0, 0.10, -90.0), (0.0, 0.0), 3.0, 0.0),
    c("b3_shoot_side25", SkillId::ShootPush, (0.75, 0.0), (0.0, 0.25, -90.0), (0.0, 0.0), 3.0, 0.0),
    c("b3_shoot_behind", SkillId::ShootPush, (0.75, 0.0), (-0.12, 0.0, 0.0), (0.0, 0.0), 3.0, 0.0),
    c("approach_front", SkillId::ApproachAligned, (0.75, 0.0), (0.3, 0.05, 180.0), (0.0, 0.0), 8.0, 0.0),
    c("approach_side", SkillId::ApproachAligned, (0.75, 0.0), (-0.1, -0.35, 90.0), (0.0, 0.0), 8.0, 0.0),
    c("mark_point", SkillId::Mark, (-0.3, 0.0), (0.2, -0.3, 0.0), (0.3, 0.3), 6.0, 0.08),
    c("intercept_static", SkillId::Intercept, (0.0, 0.0), (-0.4, -0.3, 0.0), (0.2, 0.2), 6.0, 0.09),
    c("clear_own", SkillId::Clear, (0.2, 0.45), (-0.2, 0.3, 0.0), (-0.45, 0.0), 6.0, 0.0),
    c("spinkick_wall", SkillId::SpinKick, (0.0, 0.0), (-0.2, -0.3, 0.0), (0.2, -0.6), 6.0, 0.0),
    c("a4_ball_behind_target", SkillId::GoTo, (0.33, 0.0), (-0.4, 0.0, 0.0), (0.0, 0.0), 6.0, 0.08),
    c("a4_ball_wall", SkillId::GoTo, (0.3, -0.25), (-0.3, -0.5, 0.0), (0.0, -0.56), 6.0, 0.08),
    c("spin_20", SkillId::Spin, (1.0, 0.0), (0.0, 0.0, 0.0), FAR, 2.0, 0.0),
    c("goalkeep_keeper", SkillId::GoalKeep, (-0.75, 0.0), (-0.4, 0.2, 0.0), (0.0, -0.15), 5.0, 0.0),
];

/// Casos de navegación del resumen. `b2_blockline_open` se informa aparte: su punto de
/// bloqueo cae en la zona prohibida (bug B2 de BlockLine), así que su criterio es no
/// penetrar el área ni escapar.
const NAV: &[&str] = &[
    "goto_fwd", "goto_side", "goto_back", "goto_short_side", "goto_ball", "goto_wall_along",
    "goto_wall_out", "goto_wall_facing", "chase_fwd", "chase_back", "mark_point",
    "intercept_static", "approach_front", "approach_side", "b2_blockline_center",
    "a4_ball_behind_target", "a4_ball_wall",
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

/// ¿El robot cumple el objetivo del caso en el tick `r`?
fn arrived(case: &Case, r: &Row, bidirectional: bool) -> bool {
    let p = Vec2::new(r.x, r.y);
    let heading_err = |dir: Vec2| {
        let e = Motion::normalize_angle((dir.y as f64).atan2(dir.x as f64) - r.th);
        if bidirectional { Motion::fold_bidirectional(e) } else { e }.abs()
    };
    match case.skill {
        SkillId::FacePoint => {
            heading_err(Vec2::new(case.target.0, case.target.1) - p).to_degrees() < case.tol as f64
        }
        SkillId::Hold => !AreaRect::own(1.0).touches(p),
        SkillId::ApproachAligned => {
            let tol = &rustengine::params::params().skills;
            let line = Vec2::new(case.target.0, case.target.1) - Vec2::new(r.bx, r.by);
            reference(case, r).is_some_and(|s| (s - p).length() <= tol.approach_pos_tol)
                && heading_err(line) <= tol.approach_angle_tol
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

/// Resumen de los casos de navegación: (llegan, total, giro acumulado, ticks de escape,
/// tiempo medio de los que llegan).
fn summary(results: &[(Case, Metrics)]) -> (usize, usize, f64, usize, f64) {
    let nav: Vec<&Metrics> = results.iter().filter(|(c, _)| NAV.contains(&c.name)).map(|(_, m)| m).collect();
    let times: Vec<f64> = nav.iter().filter_map(|m| m.arrived).collect();
    let mean_t = if times.is_empty() { f64::NAN } else { times.iter().sum::<f64>() / times.len() as f64 };
    (
        times.len(),
        nav.len(),
        nav.iter().map(|m| m.turn_deg).sum(),
        nav.iter().map(|m| m.escape_ticks).sum(),
        mean_t,
    )
}

async fn run_case(case: &Case) -> Result<Vec<Row>, String> {
    let client = FIRASimClient::new("127.0.0.1", 20011).await.map_err(|e| e.to_string())?;
    let mut items: Vec<TeleportItem> = [(0u32, 1u32, -0.5), (0, 2, -0.25), (1, 0, 0.0), (1, 1, 0.25), (1, 2, 0.5)]
        .into_iter()
        .map(|(team, id, x)| TeleportItem::Robot { team, id, x, y: 1.0, theta: 0.0 })
        .collect();
    // FIRASim recibe la orientación del replacement en GRADOS (`CRobot::setDir`).
    let (x, y, deg) = case.robot;
    items.push(TeleportItem::Robot { team: 0, id: 0, x: x as f64, y: y as f64, theta: deg as f64 });
    items.push(TeleportItem::Ball { x: case.ball.0 as f64, y: case.ball.1 as f64 });
    // El replacement va por UDP y el robot puede traer inercia del caso anterior.
    for _ in 0..5 {
        client.teleport(&items).await.map_err(|e| e.to_string())?;
        tokio::time::sleep(Duration::from_millis(80)).await;
    }
    tokio::time::sleep(Duration::from_millis(300)).await;

    let rows = Arc::new(Mutex::new(Vec::<Row>::new()));
    let sink = rows.clone();
    let hook = Box::new(move |rec: &TickRecord<'_>| {
        let Some(r) = rec.world.get_robot_state(0, 0) else { return };
        let b = rec.world.get_ball_state().position;
        let k = rec.commands.iter().position(|c| c.id == 0 && c.team == 0);
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
        });
    });
    let cfg = ControlLoopConfig {
        own_team: 0,
        num_robots: 3,
        vision_source: VisionSource::FiraSim,
        radio_target: RadioTarget::FiraSim,
        max_ticks: Some((case.dur * 60.0) as u32),
        vision_timeout: Some(Duration::from_secs(3)),
    };
    let target = Vec2::new(case.target.0, case.target.1);
    let decider = Box::new(FixedSkillDecider::new(0, case.skill, target));
    run_control_loop(cfg, decider, Some(hook), None, Arc::new(AtomicBool::new(false)))
        .await
        .map_err(|e| format!("{e:?}"))?;
    let out = rows.lock().unwrap().clone();
    if out.is_empty() {
        return Err("sin datos de visión del robot azul 0 (¿FIRASim corriendo?)".into());
    }
    Ok(out)
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

#[tokio::main]
async fn main() {
    let mut names = Vec::new();
    let (mut csv_dir, mut tag) = (None::<String>, String::new());
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--list" => {
                CASES.iter().for_each(|c| println!("{}", c.name));
                return;
            }
            "--csv-dir" => csv_dir = args.next(),
            "--tag" => tag = args.next().unwrap_or_default(),
            other => names.push(other.to_string()),
        }
    }
    rustengine::params::TeamParams::install_or_exit("motion_bench");
    let bidirectional = MotionConfig::from_env().bidirectional;
    let all = names.is_empty() || names.iter().any(|n| n == "all");
    let selected: Vec<&Case> = CASES.iter().filter(|c| all || names.iter().any(|n| n == c.name)).collect();
    if selected.is_empty() {
        eprintln!("[motion_bench] ningún caso coincide; ver --list");
        std::process::exit(2);
    }
    let mut results = Vec::new();
    for case in selected {
        let rows = match run_case(case).await {
            Ok(r) => r,
            Err(e) => {
                eprintln!("[motion_bench] {}: {e}", case.name);
                std::process::exit(1);
            }
        };
        if let Some(dir) = &csv_dir {
            let path = format!("{dir}/{}{tag}.csv", case.name);
            if let Err(e) = write_csv(&path, &rows) {
                eprintln!("[motion_bench] no pude escribir {path}: {e}");
            }
        }
        let m = metrics(case, &rows, bidirectional);
        let last = rows.last().copied().unwrap_or_default();
        println!(
            "{}",
            serde_json::json!({
                "case": case.name, "arrived": m.arrived.is_some(), "t_arrive": m.arrived,
                "path": m.path, "turn_deg": m.turn_deg.round(), "escape_ticks": m.escape_ticks,
                "area_ticks": m.area_ticks, "max_pen": m.max_pen, "ball_disp": m.ball_disp,
                "final": [last.x, last.y, last.th.to_degrees().round()],
            })
        );
        results.push((*case, m));
    }
    let (ok, n, turn, esc, mean_t) = summary(&results);
    if n > 0 {
        println!("\nresumen ({} casos de navegación, bidireccional={bidirectional}):", n);
        println!("  llegan {ok}/{n} · giro hasta llegar {turn:.0}° · ticks de escape {esc} · t medio {mean_t:.2} s");
    }
    if let Some((_, m)) = results.iter().find(|(c, _)| c.name == "b2_blockline_open") {
        println!(
            "  b2_blockline_open (aparte): penetración {:.3} m · {} ticks en el área · {} ticks de escape",
            m.max_pen, m.area_ticks, m.escape_ticks
        );
    }
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
    }

    #[test]
    fn escapes_and_area_penetration_are_counted() {
        let mut rows = vec![row(0.0, -0.5, 0.0, 0.0), row(0.1, -0.60, 0.0, 0.0), row(0.2, -0.62, 0.1, 0.0)];
        rows[1].escaping = true;
        rows[2].escaping = true;
        let m = metrics(&case("b1b_diag_into_area"), &rows, false);
        assert_eq!(m.escape_ticks, 2);
        assert_eq!(m.area_ticks, 2);
        assert!((m.max_pen - 0.06).abs() < 1e-6);
        assert_eq!(m.arrived, None);
    }

    #[test]
    fn face_point_folds_in_bidirectional() {
        // face_180: el destino está atrás; con dos caras ya está alineado.
        let rows = vec![row(0.0, 0.0, 0.0, 0.0)];
        assert_eq!(metrics(&case("face_180"), &rows, true).arrived, Some(0.0));
        assert_eq!(metrics(&case("face_180"), &rows, false).arrived, None);
    }

    #[test]
    fn summary_only_counts_navigation_cases() {
        let ok = Metrics { arrived: Some(1.0), turn_deg: 90.0, ..Default::default() };
        let no = Metrics { arrived: None, turn_deg: 30.0, escape_ticks: 26, ..Default::default() };
        let results = vec![(case("goto_fwd"), ok.clone()), (case("goto_side"), no), (case("spin_20"), ok)];
        let (arr, n, turn, esc, t) = summary(&results);
        assert_eq!((arr, n, esc), (1, 2, 26));
        assert!((turn - 120.0).abs() < 1e-9 && (t - 1.0).abs() < 1e-9);
    }

    #[test]
    fn case_names_are_unique_and_nav_cases_exist() {
        let mut names: Vec<&str> = CASES.iter().map(|c| c.name).collect();
        names.sort();
        names.dedup();
        assert_eq!(names.len(), CASES.len());
        assert!(NAV.iter().all(|n| CASES.iter().any(|c| c.name == *n)));
    }
}
