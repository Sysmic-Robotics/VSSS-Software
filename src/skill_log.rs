//! Logging CSV estructurado compartido entre `skill_test` y `scenario`.
//!
//! - `CsvLogger`: writer mínimo sobre `File`. Header en la primera línea.
//! - `CsvRow<'a>`: forma de una fila, con labels como `&'static str` para evitar arrastrar lifetimes.
//! - `SkillLogCtx`: contexto fijo de una corrida (transport, vision, robot, team, skill).
//! - `SkillLogCtx::build_skill_row(rec)`: única fuente de verdad para armar la fila a partir de un
//!   `TickRecord`. Los dos binarios la invocan; las filas resultantes son byte-idénticas si el
//!   `TickRecord` y el contexto coinciden.
//! - `format_human_summary`: línea humana corta para el print rate-limited a stderr (NO CSV).

use crate::control_loop::TickRecord;
use crate::radio::base_station::{SLOT_COUNT, build_frame_from_wheels, command_to_wheel_mm_s};
use crate::radio::{RadioTarget, TeamColor};
use crate::skills::SkillId;
use crate::vision::VisionSource;
use crate::world::RobotState;
use std::fmt::Display;
use std::fs::File;
use std::io::Write;
use std::path::Path;

pub const CSV_HEADER: &str = "t_ms,tick,mode,transport,vision,robot,team,skill,pose_x,pose_y,pose_theta,target_x,target_y,cmd_vx,cmd_vy,cmd_omega,wheel_L_mm_s,wheel_R_mm_s,frame_str,err_dist,err_heading,ball_x,ball_y,ball_vx,ball_vy\n";

pub struct CsvLogger {
    file: File,
}

impl CsvLogger {
    pub fn new<P: AsRef<Path>>(path: P) -> std::io::Result<Self> {
        let mut file = File::create(path)?;
        file.write_all(CSV_HEADER.as_bytes())?;
        Ok(Self { file })
    }

    pub fn write_row(&mut self, row: &CsvRow<'_>) -> std::io::Result<()> {
        writeln!(
            self.file,
            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
            row.t_ms,
            row.tick,
            row.mode,
            row.transport,
            row.vision,
            row.robot,
            row.team,
            row.skill,
            opt(row.pose_x),
            opt(row.pose_y),
            opt(row.pose_theta),
            opt(row.target_x),
            opt(row.target_y),
            opt(row.cmd_vx),
            opt(row.cmd_vy),
            opt(row.cmd_omega),
            opt(row.wheel_l),
            opt(row.wheel_r),
            row.frame_str,
            opt(row.err_dist),
            opt(row.err_heading),
            opt(row.ball_x),
            opt(row.ball_y),
            opt(row.ball_vx),
            opt(row.ball_vy),
        )
    }
}

pub fn opt<T: Display>(v: Option<T>) -> String {
    v.map(|x| format!("{x}")).unwrap_or_default()
}

#[derive(Debug, Clone)]
pub struct CsvRow<'a> {
    pub t_ms: u64,
    pub tick: u32,
    pub mode: &'a str,
    pub transport: &'a str,
    pub vision: &'a str,
    pub robot: usize,
    pub team: &'a str,
    pub skill: &'a str,
    pub pose_x: Option<f32>,
    pub pose_y: Option<f32>,
    pub pose_theta: Option<f64>,
    pub target_x: Option<f32>,
    pub target_y: Option<f32>,
    pub cmd_vx: Option<f64>,
    pub cmd_vy: Option<f64>,
    pub cmd_omega: Option<f64>,
    pub wheel_l: Option<i16>,
    pub wheel_r: Option<i16>,
    pub frame_str: String,
    pub err_dist: Option<f32>,
    pub err_heading: Option<f64>,
    /// Pelota (posición y velocidad filtradas por el tracker, o crudas si está
    /// apagado). Vacías en modo wheels. Necesarias para medir contacto por cara.
    pub ball_x: Option<f32>,
    pub ball_y: Option<f32>,
    pub ball_vx: Option<f32>,
    pub ball_vy: Option<f32>,
}

// ─────────────────────────────────────────────────────────────────────────────
//  Helpers de etiquetado (centralizados aquí: única fuente de verdad)
// ─────────────────────────────────────────────────────────────────────────────

pub fn transport_label(t: RadioTarget) -> &'static str {
    match t {
        RadioTarget::FiraSim => "firasim",
        RadioTarget::GrSim => "grsim",
        RadioTarget::BaseStation => "base-station",
    }
}

pub fn vision_label(v: Option<VisionSource>) -> &'static str {
    match v {
        Some(VisionSource::FiraSim) => "sim",
        Some(VisionSource::SslVision) => "real",
        None => "",
    }
}

pub fn team_label(t: TeamColor) -> &'static str {
    match t {
        TeamColor::Blue => "blue",
        TeamColor::Yellow => "yellow",
    }
}

pub fn skill_label(s: Option<SkillId>) -> &'static str {
    match s {
        Some(SkillId::GoTo) => "goto",
        Some(SkillId::FacePoint) => "facepoint",
        Some(SkillId::ChaseBall) => "chaseball",
        Some(SkillId::Spin) => "spin",
        Some(SkillId::ApproachAligned) => "approach",
        Some(SkillId::ShootPush) => "shoot",
        Some(SkillId::Intercept) => "intercept",
        Some(SkillId::BlockLine) => "blockline",
        Some(SkillId::GoalKeep) => "goalkeep",
        Some(SkillId::Clear) => "clear",
        Some(SkillId::SpinKick) => "spinkick",
        Some(SkillId::Mark) => "mark",
        Some(SkillId::Hold) => "hold",
        None => "",
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Row builder compartido — fuente única de verdad para modo skill
// ─────────────────────────────────────────────────────────────────────────────

/// Contexto fijo durante una corrida en modo skill: identifica el experimento
/// (transport, visión, robot, equipo, skill) y se mantiene constante por todo
/// el lazo. Solo el `TickRecord` cambia tick a tick.
#[derive(Debug, Clone, Copy)]
pub struct SkillLogCtx {
    pub transport: RadioTarget,
    pub vision: Option<VisionSource>,
    pub robot: usize,
    pub team: TeamColor,
    pub skill: SkillId,
}

impl SkillLogCtx {
    /// Construye una `CsvRow<'static>` a partir de un `TickRecord` y el contexto.
    ///
    /// Reglas (deben coincidir con la requirement "Logging CSV estructurado por tick"
    /// de la capability skill-test-harness):
    /// - `wheel_L/R` = `command_to_wheel_mm_s(&cmd.motion)` SIEMPRE, independiente del transport.
    /// - `frame_str` = ASCII completo SOLO si `transport == BaseStation`, sino vacío.
    /// - `err_dist`/`err_heading` solo si la skill usa target espacial (GoTo, FacePoint, Spin).
    pub fn build_skill_row(&self, rec: &TickRecord<'_>) -> CsvRow<'static> {
        // Pose del robot comandado, si está activo
        let robot_state = find_robot_state(rec.world, self.robot as i32, self.team.as_team_id());
        let pose_x = robot_state.as_ref().map(|r| r.position.x);
        let pose_y = robot_state.as_ref().map(|r| r.position.y);
        let pose_theta = robot_state.as_ref().map(|r| r.orientation);

        // Comando emitido (si hay)
        let cmd = rec.commands.first();
        let cmd_vx = cmd.map(|c| c.vx);
        let cmd_vy = cmd.map(|c| c.vy);
        let cmd_omega = cmd.map(|c| c.omega);

        // Wheels via command_to_wheel_mm_s SIEMPRE (independiente del transport)
        let (wl, wr) = cmd.map(command_to_wheel_mm_s).unwrap_or((0, 0));
        let wheel_l = cmd.map(|_| wl);
        let wheel_r = cmd.map(|_| wr);

        // Target espacial visible
        let target_xy = rec.targets.first().and_then(|t| *t);
        let target_x = target_xy.map(|t| t.x);
        let target_y = target_xy.map(|t| t.y);

        // Frame str (solo para base-station)
        let frame_str = match self.transport {
            RadioTarget::BaseStation => {
                let slots = wheels_to_slots(self.robot, wl, wr);
                build_frame_from_wheels(slots).trim_end().to_string()
            }
            _ => String::new(),
        };

        // Errores: solo si skill usa target espacial y robot está visible
        let skill_uses_target = matches!(
            self.skill,
            SkillId::GoTo
                | SkillId::FacePoint
                | SkillId::Spin
                | SkillId::ApproachAligned
                | SkillId::ShootPush
                | SkillId::Intercept
                | SkillId::BlockLine
                | SkillId::GoalKeep
                | SkillId::Clear
                | SkillId::SpinKick
                | SkillId::Mark
        );

        // Pelota (siempre en modo skill: la necesitan las mediciones de contacto)
        let ball = rec.world.get_ball_state();
        let (ball_x, ball_y, ball_vx, ball_vy) = (
            Some(ball.position.x),
            Some(ball.position.y),
            Some(ball.velocity.x),
            Some(ball.velocity.y),
        );
        let (err_dist, err_heading) = if skill_uses_target {
            match (robot_state.as_ref(), target_xy) {
                (Some(r), Some(t)) => {
                    let dx = t.x - r.position.x;
                    let dy = t.y - r.position.y;
                    let dist = (dx * dx + dy * dy).sqrt();
                    let desired = (dy as f64).atan2(dx as f64);
                    let mut dtheta = desired - r.orientation;
                    while dtheta > std::f64::consts::PI {
                        dtheta -= 2.0 * std::f64::consts::PI;
                    }
                    while dtheta < -std::f64::consts::PI {
                        dtheta += 2.0 * std::f64::consts::PI;
                    }
                    (Some(dist), Some(dtheta))
                }
                _ => (None, None),
            }
        } else {
            (None, None)
        };

        CsvRow {
            t_ms: rec.t_ms,
            tick: rec.tick,
            mode: "skill",
            transport: transport_label(self.transport),
            vision: vision_label(self.vision),
            robot: self.robot,
            team: team_label(self.team),
            skill: skill_label(Some(self.skill)),
            pose_x,
            pose_y,
            pose_theta,
            target_x,
            target_y,
            cmd_vx,
            cmd_vy,
            cmd_omega,
            wheel_l,
            wheel_r,
            frame_str,
            err_dist,
            err_heading,
            ball_x,
            ball_y,
            ball_vx,
            ball_vy,
        }
    }
}

fn find_robot_state(world: &crate::world::World, robot_id: i32, team: i32) -> Option<RobotState> {
    let robots = if team == 0 {
        world.get_blue_team_active()
    } else {
        world.get_yellow_team_active()
    };
    robots.into_iter().find(|r| r.id == robot_id).cloned()
}

fn wheels_to_slots(robot: usize, l: i16, r: i16) -> [(i16, i16); SLOT_COUNT] {
    let mut slots: [(i16, i16); SLOT_COUNT] = [(0, 0); SLOT_COUNT];
    if robot < SLOT_COUNT {
        slots[robot] = (l, r);
    }
    slots
}

// ─────────────────────────────────────────────────────────────────────────────
//  Registro de partido (VSSL_MATCH_LOG) — una fila por robot y tick
// ─────────────────────────────────────────────────────────────────────────────

pub const MATCH_CSV_HEADER: &str = "t_ms,tick,team,robot,own,skill,target_x,target_y,pose_x,pose_y,pose_theta,vel_x,vel_y,omega,cmd_vx,cmd_vy,cmd_omega,ball_x,ball_y,ball_vx,ball_vy\n";

/// CSV de partido: por tick, una fila por robot propio (skill, target y comando)
/// y una por rival (solo pose, `own=0`). La pelota se repite en cada fila para
/// que cualquier fila sea autocontenida. Base de las métricas de evaluación
/// (goles, posesión, toques, tiempo a la pelota) y de la revisión offline de
/// decisiones.
pub struct MatchLogger {
    /// Con buffer: 6 escrituras sin buffer por tick sobre un disco lento (p. ej.
    /// `/mnt/d` desde WSL) bajaban el loop de 60 Hz a 13 Hz y hacían divergir el
    /// tracker (paquetes de visión procesados en ráfagas).
    file: std::io::BufWriter<File>,
    own_team: i32,
    ticks: u64,
}

impl MatchLogger {
    /// Cada cuántos ticks se vacía el buffer al disco (~1 s a 60 Hz).
    const FLUSH_EVERY_TICKS: u64 = 60;

    pub fn new<P: AsRef<Path>>(path: P, own_team: i32) -> std::io::Result<Self> {
        if let Some(parent) = path.as_ref().parent()
            && !parent.as_os_str().is_empty()
        {
            std::fs::create_dir_all(parent)?;
        }
        let mut file = std::io::BufWriter::with_capacity(1 << 16, File::create(path)?);
        file.write_all(MATCH_CSV_HEADER.as_bytes())?;
        Ok(Self {
            file,
            own_team,
            ticks: 0,
        })
    }

    pub fn write_tick(&mut self, rec: &TickRecord<'_>) -> std::io::Result<()> {
        self.ticks += 1;
        if self.ticks.is_multiple_of(Self::FLUSH_EVERY_TICKS) {
            self.file.flush()?;
        }
        let ball = rec.world.get_ball_state();
        let ball_cols = format!(
            "{},{},{},{}",
            ball.position.x, ball.position.y, ball.velocity.x, ball.velocity.y
        );
        // Propios: filas paralelas a `commands`. La choice/target pueden faltar si
        // el comando vino de control manual (no del coach).
        for (i, cmd) in rec.commands.iter().enumerate() {
            let choice = rec.choices.get(i);
            let target = rec.targets.get(i).copied().flatten();
            let robot = rec.world.get_robot_state(cmd.id, cmd.team);
            writeln!(
                self.file,
                "{},{},{},{},1,{},{},{},{},{},{},{},{},{},{},{},{},{}",
                rec.t_ms,
                rec.tick,
                cmd.team,
                cmd.id,
                skill_label(choice.map(|c| c.skill_id)),
                opt(target.map(|t| t.x)),
                opt(target.map(|t| t.y)),
                opt(robot.map(|r| r.position.x)),
                opt(robot.map(|r| r.position.y)),
                opt(robot.map(|r| r.orientation)),
                opt(robot.map(|r| r.velocity.x)),
                opt(robot.map(|r| r.velocity.y)),
                opt(robot.map(|r| r.angular_velocity)),
                cmd.vx,
                cmd.vy,
                cmd.omega,
                ball_cols
            )?;
        }
        // Rivales: solo pose.
        let opp = 1 - self.own_team;
        let opps = if opp == 0 {
            rec.world.get_blue_team_active()
        } else {
            rec.world.get_yellow_team_active()
        };
        for r in opps {
            writeln!(
                self.file,
                "{},{},{},{},0,,,,{},{},{},{},{},{},,,,{}",
                rec.t_ms,
                rec.tick,
                r.team,
                r.id,
                r.position.x,
                r.position.y,
                r.orientation,
                r.velocity.x,
                r.velocity.y,
                r.angular_velocity,
                ball_cols
            )?;
        }
        Ok(())
    }
}

impl Drop for MatchLogger {
    fn drop(&mut self) {
        let _ = self.file.flush();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Print humano (NO CSV) — feedback rate-limited a stderr durante el run
// ─────────────────────────────────────────────────────────────────────────────

/// Línea corta humana derivada de una fila CSV. NO es CSV; el print rate-limited
/// a stderr (~1 Hz) usa este formato para que el operador audite sin abrir el log.
pub fn format_human_summary(row: &CsvRow<'_>) -> String {
    let t = row.t_ms as f64 / 1000.0;
    let pose_str = match (row.pose_x, row.pose_y, row.pose_theta) {
        (Some(x), Some(y), Some(th)) => format!("pose=({x:.2},{y:.2},{th:.2})"),
        _ => "pose=N/A".to_string(),
    };
    let cmd_str = match (row.cmd_vx, row.cmd_vy, row.cmd_omega) {
        (Some(vx), Some(vy), Some(om)) => format!("cmd=({vx:.2},{vy:.2},{om:.2})"),
        _ => "cmd=N/A".to_string(),
    };
    let wheels_str = match (row.wheel_l, row.wheel_r) {
        (Some(l), Some(r)) => format!("wheels=({l},{r})"),
        _ => "wheels=N/A".to_string(),
    };
    format!(
        "[{} t={t:.1}s tick={} {pose_str} {cmd_str} {wheels_str}]",
        row.mode, row.tick
    )
}

// ─────────────────────────────────────────────────────────────────────────────
//  Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::MotionCommand;
    use crate::world::World;
    use glam::Vec2;

    fn empty_tick_record<'a>(
        world: &'a World,
        commands: &'a [MotionCommand],
        targets: &'a [Option<Vec2>],
        choices: &'a [crate::coach::SkillChoice],
    ) -> TickRecord<'a> {
        TickRecord {
            tick: 0,
            t_ms: 0,
            world,
            commands,
            targets,
            choices,
        }
    }

    #[test]
    fn csv_header_starts_with_t_ms_tick_mode() {
        assert!(CSV_HEADER.starts_with("t_ms,tick,mode,"));
    }

    #[test]
    fn csv_header_has_25_columns() {
        let cols = CSV_HEADER.trim_end().split(',').count();
        assert_eq!(cols, 25);
        assert!(CSV_HEADER.trim_end().ends_with("ball_x,ball_y,ball_vx,ball_vy"));
    }

    #[test]
    fn build_row_no_command_zero_wheels() {
        let world = World::new(3, 3);
        let ctx = SkillLogCtx {
            transport: RadioTarget::FiraSim,
            vision: Some(VisionSource::FiraSim),
            robot: 0,
            team: TeamColor::Blue,
            skill: SkillId::GoTo,
        };
        let rec = empty_tick_record(&world, &[], &[], &[]);
        let row = ctx.build_skill_row(&rec);
        assert_eq!(row.wheel_l, None);
        assert_eq!(row.wheel_r, None);
        assert!(row.frame_str.is_empty());
    }

    #[test]
    fn build_row_forward_command_wheels_500() {
        // (vx=0.5, vy=0, omega=0, orientation=0) → (500, 500) via command_to_wheel_mm_s.
        let world = World::new(3, 3);
        let cmd = MotionCommand {
            id: 0,
            team: 0,
            vx: 0.5,
            vy: 0.0,
            omega: 0.0,
            orientation: 0.0,
        };
        let commands = vec![cmd];
        let ctx = SkillLogCtx {
            transport: RadioTarget::FiraSim,
            vision: Some(VisionSource::FiraSim),
            robot: 0,
            team: TeamColor::Blue,
            skill: SkillId::GoTo,
        };
        let targets = [Some(Vec2::new(0.3, 0.0))];
        let rec = empty_tick_record(&world, &commands, &targets, &[]);
        let row = ctx.build_skill_row(&rec);
        assert_eq!(row.wheel_l, Some(500));
        assert_eq!(row.wheel_r, Some(500));
        // transport != BaseStation → frame_str vacío
        assert!(row.frame_str.is_empty());
        // labels
        assert_eq!(row.transport, "firasim");
        assert_eq!(row.vision, "sim");
        assert_eq!(row.team, "blue");
        assert_eq!(row.skill, "goto");
        assert_eq!(row.mode, "skill");
    }

    #[test]
    fn build_row_base_station_emits_frame_str() {
        let world = World::new(3, 3);
        let cmd = MotionCommand {
            id: 0,
            team: 0,
            vx: 0.5,
            vy: 0.0,
            omega: 0.0,
            orientation: 0.0,
        };
        let commands = vec![cmd];
        let ctx = SkillLogCtx {
            transport: RadioTarget::BaseStation,
            vision: None,
            robot: 0,
            team: TeamColor::Blue,
            skill: SkillId::GoTo,
        };
        let targets = [Some(Vec2::new(0.3, 0.0))];
        let rec = empty_tick_record(&world, &commands, &targets, &[]);
        let row = ctx.build_skill_row(&rec);
        // (500,500) en slot 0, resto cero
        assert_eq!(row.frame_str, "500,500,0,0,0,0,0,0,0,0");
        assert_eq!(row.transport, "base-station");
    }

    #[test]
    fn build_row_chase_ball_no_err_columns() {
        // ChaseBall no usa target espacial → err_dist/err_heading None.
        let world = World::new(3, 3);
        let cmd = MotionCommand {
            id: 0,
            team: 0,
            vx: 0.5,
            vy: 0.0,
            omega: 0.0,
            orientation: 0.0,
        };
        let commands = vec![cmd];
        let ctx = SkillLogCtx {
            transport: RadioTarget::FiraSim,
            vision: Some(VisionSource::FiraSim),
            robot: 0,
            team: TeamColor::Blue,
            skill: SkillId::ChaseBall,
        };
        let targets: [Option<Vec2>; 1] = [None];
        let rec = empty_tick_record(&world, &commands, &targets, &[]);
        let row = ctx.build_skill_row(&rec);
        assert_eq!(row.err_dist, None);
        assert_eq!(row.err_heading, None);
    }

    #[test]
    fn match_logger_writes_one_row_per_robot_with_fixed_columns() {
        let mut world = World::new(3, 3);
        world.update_robot(0, 0, Vec2::new(0.1, 0.2), 0.3, Vec2::ZERO, 0.0); // azul 0 (propio)
        world.update_robot(1, 1, Vec2::new(-0.4, 0.0), 1.0, Vec2::ZERO, 0.0); // amarillo 1 (rival)
        world.update_ball(Vec2::new(0.5, -0.1), Vec2::new(0.2, 0.0));
        let cmd = MotionCommand {
            id: 0,
            team: 0,
            vx: 0.5,
            vy: 0.0,
            omega: 0.1,
            orientation: 0.3,
        };
        let commands = vec![cmd];
        let targets: [Option<Vec2>; 1] = [Some(Vec2::new(0.75, 0.0))];
        let choices = [crate::coach::SkillChoice::new(0, SkillId::ShootPush, Vec2::new(0.75, 0.0))];
        let rec = empty_tick_record(&world, &commands, &targets, &choices);

        let path = std::env::temp_dir().join(format!("vsss_match_log_test_{}.csv", std::process::id()));
        {
            let mut logger = MatchLogger::new(&path, 0).unwrap();
            logger.write_tick(&rec).unwrap();
        }
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let lines: Vec<&str> = text.lines().collect();
        let ncols = MATCH_CSV_HEADER.trim_end().split(',').count();
        assert_eq!(lines.len(), 3, "header + propio + rival: {text}");
        for l in &lines {
            assert_eq!(l.split(',').count(), ncols, "columnas en '{l}'");
        }
        assert!(lines[1].contains(",1,shoot,0.75,0,"), "fila propia: {}", lines[1]);
        assert!(lines[2].starts_with("0,0,1,1,0,,,,"), "fila rival: {}", lines[2]);
        assert!(lines[2].ends_with("0.5,-0.1,0.2,0"), "pelota en fila rival: {}", lines[2]);
    }

    #[test]
    fn format_human_summary_includes_tick_and_pose() {
        let row = CsvRow {
            t_ms: 1500,
            tick: 90,
            mode: "skill",
            transport: "firasim",
            vision: "sim",
            robot: 0,
            team: "blue",
            skill: "goto",
            pose_x: Some(0.12),
            pose_y: Some(-0.05),
            pose_theta: Some(0.1),
            target_x: Some(0.3),
            target_y: Some(0.0),
            cmd_vx: Some(0.4),
            cmd_vy: Some(0.0),
            cmd_omega: Some(0.0),
            wheel_l: Some(400),
            wheel_r: Some(400),
            frame_str: String::new(),
            err_dist: Some(0.18),
            err_heading: Some(0.05),
            ball_x: Some(0.5),
            ball_y: Some(0.0),
            ball_vx: Some(0.0),
            ball_vy: Some(0.0),
        };
        let s = format_human_summary(&row);
        assert!(s.contains("tick=90"));
        assert!(s.contains("pose=(0.12"));
        assert!(s.contains("wheels=(400,400)"));
    }
}
