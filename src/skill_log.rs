//! Logging CSV estructurado compartido entre `skill_test` y `scenario`.
//!
//! - `CsvLogger`: writer mínimo sobre `File`. Header en la primera línea. CSV estándar: el
//!   `frame_str` de la base (con comas) va entre comillas (`csv_field`).
//! - `CsvRow<'a>`: forma de una fila, con labels como `&'static str` para evitar arrastrar lifetimes.
//! - `SkillLogCtx`: contexto fijo de una corrida (transport, vision, robot, team, skill).
//! - `SkillLogCtx::build_skill_row(rec)`: única fuente de verdad para armar la fila a partir de un
//!   `TickRecord`. Los dos binarios la invocan; las filas resultantes son byte-idénticas si el
//!   `TickRecord` y el contexto coinciden.
//! - `format_human_summary`: línea humana corta para el print rate-limited a stderr (NO CSV).

use crate::coach::Role;
use crate::coach::debug::Trabada;
use crate::control_loop::TickRecord;
use crate::motion::avoid::Rodeado;
use crate::radio::base_station::{SLOT_COUNT, build_frame_from_vw, command_to_vw};
use crate::radio::{RadioTarget, TeamColor};
use crate::skills::zones::AccionGuardia;
use crate::skills::{Face, SkillId};
use crate::rutinas::RutinaVista;
use crate::snapshot::TickDebug;
use crate::vision::VisionSource;
use crate::world::RobotState;
use std::fmt::Display;
use std::fs::File;
use std::io::Write;
use std::path::Path;

pub const CSV_HEADER: &str = "t_ms,tick,mode,transport,vision,robot,team,skill,pose_x,pose_y,pose_theta,target_x,target_y,cmd_vx,cmd_vy,cmd_omega,v_mm_s,w_deg_s,frame_str,err_dist,err_heading,ball_x,ball_y,ball_vx,ball_vy\n";

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
            opt(row.v_mm_s),
            opt(row.w_deg_s),
            csv_field(&row.frame_str),
            opt(row.err_dist),
            opt(row.err_heading),
            opt(row.ball_x),
            opt(row.ball_y),
            opt(row.ball_vx),
            opt(row.ball_vy),
        )
    }
}

/// Campo de texto en CSV estándar: tal cual, o entre comillas dobles (con las internas
/// duplicadas) si tiene coma, comillas o salto de línea. El frame de la base
/// (`V1,W1,…`) tiene comas: sin comillas correría las columnas siguientes.
pub fn csv_field(s: &str) -> std::borrow::Cow<'_, str> {
    if s.contains([',', '"', '\n', '\r']) {
        format!("\"{}\"", s.replace('"', "\"\"")).into()
    } else {
        s.into()
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
    /// Lo que LLEGA al robot: velocidad lineal (mm/s) y angular (grados/s) del
    /// frame `V,W` (`command_to_vw`).
    pub v_mm_s: Option<i16>,
    pub w_deg_s: Option<i16>,
    pub frame_str: String,
    pub err_dist: Option<f32>,
    pub err_heading: Option<f64>,
    /// Pelota (posición y velocidad filtradas por el tracker, o crudas si está
    /// apagado). Vacías en modo vw. Necesarias para medir contacto por cara.
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
    /// Id de VISIÓN del robot comandado (columna `robot`, búsqueda en `World`).
    pub robot: usize,
    /// Posición en el frame de la base: `radio_slot(robot, mapa de params)`. Es la
    /// que usa `frame_str`, para mostrar el frame realmente enviado.
    pub radio_slot: Option<usize>,
    pub team: TeamColor,
    pub skill: SkillId,
}

impl SkillLogCtx {
    /// Construye una `CsvRow<'static>` a partir de un `TickRecord` y el contexto.
    ///
    /// Reglas (deben coincidir con la requirement "Logging CSV estructurado por tick"
    /// de la capability skill-test-harness):
    /// - `v_mm_s`/`w_deg_s` = `command_to_vw(&cmd.motion)` SIEMPRE, independiente del transport.
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

        // (v, w) vía command_to_vw SIEMPRE (independiente del transport)
        let (v, w) = cmd.map(command_to_vw).unwrap_or((0, 0));
        let v_mm_s = cmd.map(|_| v);
        let w_deg_s = cmd.map(|_| w);

        // Target espacial visible
        let target_xy = rec.targets.first().and_then(|t| *t);
        let target_x = target_xy.map(|t| t.x);
        let target_y = target_xy.map(|t| t.y);

        // Frame str (solo para base-station)
        let frame_str = match self.transport {
            RadioTarget::BaseStation => {
                let slots = vw_to_slots(self.radio_slot, v, w);
                build_frame_from_vw(slots).trim_end().to_string()
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
            v_mm_s,
            w_deg_s,
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

fn vw_to_slots(slot: Option<usize>, v: i16, w: i16) -> [(i16, i16); SLOT_COUNT] {
    let mut slots: [(i16, i16); SLOT_COUNT] = [(0, 0); SLOT_COUNT];
    if let Some(slot) = slot.filter(|s| *s < SLOT_COUNT) {
        slots[slot] = (v, w);
    }
    slots
}

// ─────────────────────────────────────────────────────────────────────────────
//  Registro de partido (VSSL_MATCH_LOG) — una fila por robot y tick
// ─────────────────────────────────────────────────────────────────────────────

pub const MATCH_CSV_HEADER: &str = "t_ms,tick,team,robot,own,skill,target_x,target_y,pose_x,pose_y,pose_theta,vel_x,vel_y,omega,cmd_vx,cmd_vy,cmd_omega,ball_x,ball_y,ball_vx,ball_vy,\
role,progress,feasible,face,nav_dest_x,nav_dest_y,nav_aim_x,nav_aim_y,nav_detour,nav_side,nav_reverse,\
guard,guard_proj_x,guard_proj_y,escape_dx,escape_dy,arrival_s,coach_decision,striker,score_shoot,score_clear,\
score_spin,score_intercept,score_approach,pick_score,pick,bonus_skill,commit_left,stall,stall_s,\
adapt_shot_upper,adapt_shot_center,adapt_shot_lower,adapt_duel,\
rutina,rutina_fase,puesto,repeticion,paso,accion\n";

/// Cantidad de columnas de depuración después de las 21 de base de `MATCH_CSV_HEADER`.
const MATCH_DEBUG_COLS: usize = 34;
/// Cantidad de columnas de rutina al final de `MATCH_CSV_HEADER` (capability
/// `rutinas-ejecucion`).
const MATCH_RUTINA_COLS: usize = 6;

/// Columnas de rutina de la fila del robot propio `id`: `rutina` (el id, sin comas),
/// fase, puesto, repetición, paso (desde 1) y tipo del paso. Vacías sin rutina o si el
/// robot no tiene puesto.
fn columnas_rutina(rutina: Option<&RutinaVista>, id: i32) -> String {
    let Some((v, r)) = rutina.and_then(|v| v.de_robot(id).map(|r| (v, r))) else {
        return ",".repeat(MATCH_RUTINA_COLS - 1);
    };
    [
        v.rutina.id.clone(),
        v.fase.etiqueta().to_string(),
        v.nombre_puesto(r.puesto).to_string(),
        if v.repeticion > 0 { v.repeticion.to_string() } else { String::new() },
        r.paso.map(|p| (p + 1).to_string()).unwrap_or_default(),
        r.accion.unwrap_or_default().to_string(),
    ]
    .join(",")
}

/// Columnas de depuración de la fila del robot propio `id` (capability
/// `depuracion-tick`), separadas por coma y sin comas adentro. Un dato que no existe
/// queda vacío; sin depuración, todas vacías.
fn columnas_depuracion(debug: Option<&TickDebug>, id: i32) -> String {
    let num = |x: Option<f32>| x.map(|v| format!("{v:.3}")).unwrap_or_default();
    let Some(d) = debug else {
        return ",".repeat(MATCH_DEBUG_COLS - 1);
    };
    let r = d.robot(id);
    let nav = r.and_then(|r| r.nav);
    let guardia = r.and_then(|r| r.guardia);
    let escape = r.and_then(|r| r.escape);
    let status = r.and_then(|r| r.status);
    let coach = d.coach.as_ref();
    let rol = coach.and_then(|c| c.roles.iter().find(|x| x.id == id));
    let opciones = coach.and_then(|c| c.opciones);
    let puntaje = |s: SkillId| num(opciones.and_then(|o| o.puntajes.iter().find(|p| p.0 == s).map(|p| p.1)));
    let (trabada, trabada_s) = match coach.map(|c| c.trabada) {
        None => ("", None),
        Some(Trabada::Libre) => ("none", None),
        Some(Trabada::Contando(s)) => ("counting", Some(s)),
        Some(Trabada::Pateando(s)) => ("kicking", Some(s)),
        Some(Trabada::GiroProhibido(s)) => ("banned", Some(s)),
    };
    let adapt = coach.map(|c| c.adaptacion);
    [
        rol.and_then(|x| x.rol)
            .map(|x| match x {
                Role::Keeper => "keeper",
                Role::Striker => "striker",
                Role::Support => "support",
            })
            .unwrap_or_default()
            .to_string(),
        num(status.map(|s| s.progress)),
        status.map(|s| u8::from(s.feasible).to_string()).unwrap_or_default(),
        status
            .map(|s| if s.face == Face::Front { "front" } else { "back" })
            .unwrap_or_default()
            .to_string(),
        num(nav.map(|n| n.destino.x)),
        num(nav.map(|n| n.destino.y)),
        num(nav.map(|n| n.mira.x)),
        num(nav.map(|n| n.mira.y)),
        nav.and_then(|n| n.rodeo)
            .map(|r| match r.obstaculo {
                Rodeado::Pelota => "ball",
                Rodeado::AreaIzquierda => "area_left",
                Rodeado::AreaDerecha => "area_right",
            })
            .unwrap_or_default()
            .to_string(),
        nav.and_then(|n| n.rodeo).map(|r| r.lado.to_string()).unwrap_or_default(),
        nav.map(|n| u8::from(n.espaldas).to_string()).unwrap_or_default(),
        guardia
            .and_then(|g| g.accion)
            .map(|a| match a {
                AccionGuardia::Frena => "brake",
                AccionGuardia::Saca => "exit",
            })
            .unwrap_or_default()
            .to_string(),
        num(guardia.map(|g| g.proyeccion.x)),
        num(guardia.map(|g| g.proyeccion.y)),
        num(escape.map(|e| e.x)),
        num(escape.map(|e| e.y)),
        num(rol.and_then(|x| x.llegada_s)),
        coach.map(|c| c.decision.to_string()).unwrap_or_default(),
        coach.and_then(|c| c.atacante).map(|a| a.to_string()).unwrap_or_default(),
        puntaje(SkillId::ShootPush),
        puntaje(SkillId::Clear),
        puntaje(SkillId::SpinKick),
        puntaje(SkillId::Intercept),
        puntaje(SkillId::ApproachAligned),
        opciones.map(|o| skill_label(Some(o.ganadora)).to_string()).unwrap_or_default(),
        coach
            .and_then(|c| c.skill_atacante)
            .map(|s| skill_label(Some(s)).to_string())
            .unwrap_or_default(),
        opciones
            .and_then(|o| o.bono)
            .map(|b| skill_label(Some(b.0)).to_string())
            .unwrap_or_default(),
        opciones.map(|o| o.compromiso.restantes.to_string()).unwrap_or_default(),
        trabada.to_string(),
        num(trabada_s),
        num(adapt.map(|a| a.tiros[0].value)),
        num(adapt.map(|a| a.tiros[1].value)),
        num(adapt.map(|a| a.tiros[2].value)),
        num(adapt.map(|a| a.duelos.value)),
    ]
    .join(",")
}

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
        // Propios: filas paralelas a `commands` (y `targets`). La elección aplicada se
        // busca por id de robot: con un comando manual o con STOP, las listas no van en
        // paralelo. Puede faltar si el comando vino de control manual.
        for (i, cmd) in rec.commands.iter().enumerate() {
            let choice = rec.choices.iter().find(|c| c.robot_id == cmd.id);
            let target = rec.targets.get(i).copied().flatten();
            let robot = rec.world.get_robot_state(cmd.id, cmd.team);
            writeln!(
                self.file,
                "{},{},{},{},1,{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
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
                ball_cols,
                columnas_depuracion(rec.debug, cmd.id),
                columnas_rutina(rec.rutina, cmd.id)
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
                "{},{},{},{},0,,,,{},{},{},{},{},{},,,,{},{}",
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
                ball_cols,
                ",".repeat(MATCH_DEBUG_COLS - 1 + MATCH_RUTINA_COLS)
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
    let vw_str = match (row.v_mm_s, row.w_deg_s) {
        (Some(v), Some(w)) => format!("vw=({v},{w})"),
        _ => "vw=N/A".to_string(),
    };
    format!(
        "[{} t={t:.1}s tick={} {pose_str} {cmd_str} {vw_str}]",
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
            escaping: &[],
            debug: None,
            rutina: None,
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
        assert!(CSV_HEADER.contains(",cmd_omega,v_mm_s,w_deg_s,frame_str,"));
        assert!(!CSV_HEADER.contains("wheel_"));
        assert!(CSV_HEADER.trim_end().ends_with("ball_x,ball_y,ball_vx,ball_vy"));
    }

    #[test]
    fn build_row_no_command_no_vw() {
        let world = World::new(3, 3);
        let ctx = SkillLogCtx {
            transport: RadioTarget::FiraSim,
            vision: Some(VisionSource::FiraSim),
            robot: 0,
            radio_slot: Some(0),
            team: TeamColor::Blue,
            skill: SkillId::GoTo,
        };
        let rec = empty_tick_record(&world, &[], &[], &[]);
        let row = ctx.build_skill_row(&rec);
        assert_eq!(row.v_mm_s, None);
        assert_eq!(row.w_deg_s, None);
        assert!(row.frame_str.is_empty());
    }

    #[test]
    fn build_row_forward_command_vw_500_0() {
        // (vx=0.5, vy=0, omega=0, orientation=0) → (500, 0) vía command_to_vw.
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
            radio_slot: Some(0),
            team: TeamColor::Blue,
            skill: SkillId::GoTo,
        };
        let targets = [Some(Vec2::new(0.3, 0.0))];
        let rec = empty_tick_record(&world, &commands, &targets, &[]);
        let row = ctx.build_skill_row(&rec);
        assert_eq!(row.v_mm_s, Some(500));
        assert_eq!(row.w_deg_s, Some(0));
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
            radio_slot: Some(0),
            team: TeamColor::Blue,
            skill: SkillId::GoTo,
        };
        let targets = [Some(Vec2::new(0.3, 0.0))];
        let rec = empty_tick_record(&world, &commands, &targets, &[]);
        let row = ctx.build_skill_row(&rec);
        // (500, 0) en slot 0, resto cero
        assert_eq!(row.frame_str, "500,0,0,0,0,0,0,0,0,0");
        assert_eq!(row.transport, "base-station");
    }

    #[test]
    fn build_row_base_station_frame_str_uses_mapped_slot() {
        // Visión ve el robot como #1; escucha la posición 0 (mapa {1: 0, 0: 1}).
        let world = World::new(3, 3);
        let cmd = MotionCommand {
            id: 1,
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
            robot: 1,
            radio_slot: Some(0),
            team: TeamColor::Blue,
            skill: SkillId::GoTo,
        };
        let targets = [Some(Vec2::new(0.3, 0.0))];
        let rec = empty_tick_record(&world, &commands, &targets, &[]);
        let row = ctx.build_skill_row(&rec);
        assert_eq!(row.robot, 1);
        assert_eq!(row.frame_str, "500,0,0,0,0,0,0,0,0,0");
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
            radio_slot: Some(0),
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
        let col = |l: &str, nombre: &str| -> String {
            let i = MATCH_CSV_HEADER.trim_end().split(',').position(|h| h == nombre).unwrap();
            l.split(',').nth(i).unwrap().to_string()
        };
        assert_eq!(col(lines[2], "ball_vx"), "0.2", "pelota en fila rival: {}", lines[2]);
        assert!(
            lines[2].ends_with(&",".repeat(MATCH_DEBUG_COLS + MATCH_RUTINA_COLS)),
            "depuración y rutina vacías en el rival"
        );
        assert_eq!(col(lines[1], "role"), "", "sin depuración, vacía");
        assert_eq!(col(lines[1], "rutina"), "", "sin rutina, vacía");
        assert!(MATCH_CSV_HEADER.trim_end().ends_with(",adapt_duel,rutina,rutina_fase,puesto,repeticion,paso,accion"));
    }

    #[test]
    fn match_csv_carries_the_routine_columns() {
        use crate::rutinas::vista::{Fase, RobotEnRutina, RutinaVista};
        let mut world = World::new(3, 3);
        world.update_robot(0, 0, Vec2::new(0.1, 0.2), 0.0, Vec2::ZERO, 0.0);
        world.update_robot(1, 0, Vec2::new(-0.3, -0.2), 0.0, Vec2::ZERO, 0.0);
        world.update_robot(0, 1, Vec2::new(0.4, 0.0), 0.0, Vec2::ZERO, 0.0);
        let rutina = crate::rutinas::prueba::rutina(&crate::rutinas::prueba::un_puesto(
            r#"{ "paso": "ir", "x": 0.3, "y": -0.25 }, { "paso": "ir", "x": 0.0, "y": 0.27 }"#,
        ));
        let mut rutina_id = (*rutina).clone();
        rutina_id.id = "triangulo".to_string();
        let vista = RutinaVista {
            rutina: std::sync::Arc::new(rutina_id),
            fase: Fase::Ejecutando,
            frase: String::new(),
            velocidad_pct: 100,
            repeticion: 3,
            repeticiones: crate::rutinas::Repeticiones::Veces(3),
            robots: vec![RobotEnRutina {
                robot: 1,
                puesto: 0,
                paso: Some(1),
                total_pasos: 2,
                accion: Some("ir"),
                destino: Some(Vec2::new(0.0, 0.27)),
            }],
            resumen: None,
        };
        let cmd = |id: i32| MotionCommand { id, team: 0, vx: 0.0, vy: 0.0, omega: 0.0, orientation: 0.0 };
        let commands = vec![cmd(0), cmd(1)];
        let targets = [None, None];
        let mut rec = empty_tick_record(&world, &commands, &targets, &[]);
        rec.rutina = Some(&vista);
        let path = std::env::temp_dir().join(format!("vsss_match_rutina_test_{}.csv", std::process::id()));
        {
            let mut logger = MatchLogger::new(&path, 0).unwrap();
            logger.write_tick(&rec).unwrap();
        }
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let lines: Vec<&str> = text.lines().collect();
        let ncols = MATCH_CSV_HEADER.trim_end().split(',').count();
        for l in &lines {
            assert_eq!(l.split(',').count(), ncols, "columnas en '{l}'");
        }
        assert!(lines[2].ends_with(",triangulo,ejecutando,A,3,2,ir"), "robot en rutina: {}", lines[2]);
        assert!(lines[1].ends_with(&",".repeat(MATCH_RUTINA_COLS)), "robot sin puesto: {}", lines[1]);
        assert!(lines[3].ends_with(&",".repeat(MATCH_RUTINA_COLS)), "rival: {}", lines[3]);
    }

    #[test]
    fn match_csv_carries_the_debug_columns_matched_by_robot_id() {
        use crate::coach::{Coach, HeuristicCoach, Observation};
        use crate::motion::NavDebug;
        use crate::motion::avoid::{Rodeado, Rodeo};
        use crate::skills::SkillStatus;
        use crate::skills::zones::{AccionGuardia, GuardiaDebug};
        use crate::snapshot::RobotDebug;
        // Una decisión real del coach heurístico.
        let mut world = World::new(3, 3);
        world.update_robot(0, 0, Vec2::new(-0.5, 0.3), 0.0, Vec2::ZERO, 0.0);
        world.update_robot(1, 0, Vec2::new(0.0, -0.1), 0.0, Vec2::ZERO, 0.0);
        world.update_robot(2, 0, Vec2::new(-0.63, 0.0), 0.0, Vec2::ZERO, 0.0);
        world.update_ball(Vec2::new(0.2, 0.0), Vec2::ZERO);
        let mut c = HeuristicCoach::with_options(Vec2::new(0.75, 0.0), Vec2::new(-0.75, 0.0), 2, true);
        let choices = c.decide(&Observation::from_world(&world, 0));
        let coach = c.debug().unwrap();
        let striker = coach.atacante.unwrap();
        let elegida = *choices.iter().find(|ch| ch.robot_id == striker).unwrap();
        let debug = TickDebug {
            coach: Some(coach),
            robots: vec![RobotDebug {
                id: striker,
                nav: Some(NavDebug {
                    destino: Vec2::new(0.1, 0.05),
                    mira: Vec2::new(0.05, 0.12),
                    llego: false,
                    espaldas: true,
                    evita_pelota: true,
                    rodeo: Some(Rodeo { obstaculo: Rodeado::Pelota, lado: 1, tangencia: Vec2::new(0.15, 0.1), holgura: 0.09 }),
                }),
                guardia: Some(GuardiaDebug { proyeccion: Vec2::new(0.2, 0.0), accion: Some(AccionGuardia::Frena) }),
                prohibidas: Vec::new(),
                escape: None,
                status: Some(SkillStatus { progress: 0.5, done: false, feasible: true, face: Face::Back }),
            }],
        };
        // El robot manual va primero en la lista de comandos y no tiene elección: la del
        // atacante se busca por id, no por posición.
        let manual = 3 - striker - 2;
        let cmd = |id: i32| MotionCommand { id, team: 0, vx: 0.1, vy: 0.0, omega: 0.0, orientation: 0.0 };
        let commands = vec![cmd(manual), cmd(striker)];
        let targets = [None, Some(elegida.target)];
        let applied = [elegida];
        let mut rec = empty_tick_record(&world, &commands, &targets, &applied);
        rec.debug = Some(&debug);
        let path = std::env::temp_dir().join(format!("vsss_match_debug_test_{}.csv", std::process::id()));
        {
            let mut logger = MatchLogger::new(&path, 0).unwrap();
            logger.write_tick(&rec).unwrap();
        }
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let lines: Vec<&str> = text.lines().collect();
        let header: Vec<&str> = lines[0].split(',').collect();
        assert_eq!(header.len(), 21 + MATCH_DEBUG_COLS + MATCH_RUTINA_COLS);
        let col = |l: &str, nombre: &str| -> String {
            let i = header.iter().position(|h| *h == nombre).unwrap();
            l.split(',').nth(i).unwrap().to_string()
        };
        for l in &lines {
            assert_eq!(l.split(',').count(), header.len(), "columnas en '{l}'");
        }
        let (fila_manual, fila_atacante) = (lines[1], lines[2]);
        assert_eq!(col(fila_manual, "skill"), "", "el manual no tiene elección");
        assert_eq!(col(fila_atacante, "skill"), skill_label(Some(elegida.skill_id)));
        assert_eq!(col(fila_atacante, "role"), "striker");
        assert_eq!(col(fila_atacante, "pick"), skill_label(Some(elegida.skill_id)));
        assert!(!col(fila_atacante, "score_approach").is_empty());
        assert_eq!(col(fila_atacante, "nav_detour"), "ball");
        assert_eq!(col(fila_atacante, "nav_side"), "1");
        assert_eq!(col(fila_atacante, "nav_reverse"), "1");
        assert_eq!(col(fila_atacante, "guard"), "brake");
        assert_eq!(col(fila_atacante, "face"), "back");
        assert_eq!(col(fila_atacante, "progress"), "0.500");
        assert_eq!(col(fila_atacante, "coach_decision"), "1");
        assert_eq!(col(fila_manual, "role"), "support", "los datos de coach van en cada fila propia");
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
            v_mm_s: Some(400),
            w_deg_s: Some(-90),
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
        assert!(s.contains("vw=(400,-90)"));
    }

    /// Separa una línea CSV estándar (comillas dobles, `""` = comilla).
    fn split_csv(line: &str) -> Vec<String> {
        let (mut out, mut cur, mut quoted) = (Vec::new(), String::new(), false);
        let mut chars = line.chars().peekable();
        while let Some(c) = chars.next() {
            match (c, quoted) {
                ('"', true) if chars.peek() == Some(&'"') => {
                    cur.push('"');
                    chars.next();
                }
                ('"', _) => quoted = !quoted,
                (',', false) => out.push(std::mem::take(&mut cur)),
                _ => cur.push(c),
            }
        }
        out.push(cur);
        out
    }

    fn written_line(frame: &str) -> String {
        let path = std::env::temp_dir().join(format!("skill_log_{}_{}.csv", std::process::id(), frame.len()));
        let mut log = CsvLogger::new(&path).unwrap();
        let row = CsvRow {
            t_ms: 50,
            tick: 3,
            mode: "vw",
            transport: "base-station",
            vision: "",
            robot: 1,
            team: "blue",
            skill: "",
            pose_x: None,
            pose_y: None,
            pose_theta: None,
            target_x: None,
            target_y: None,
            cmd_vx: None,
            cmd_vy: None,
            cmd_omega: None,
            v_mm_s: Some(300),
            w_deg_s: Some(-45),
            frame_str: frame.to_string(),
            err_dist: Some(0.25),
            err_heading: None,
            ball_x: Some(0.1),
            ball_y: Some(-0.2),
            ball_vx: None,
            ball_vy: None,
        };
        log.write_row(&row).unwrap();
        drop(log);
        let text = std::fs::read_to_string(&path).unwrap();
        std::fs::remove_file(&path).ok();
        text.lines().nth(1).unwrap().to_string()
    }

    #[test]
    fn base_station_frame_is_quoted_and_reads_as_one_column() {
        let line = written_line("0,0,300,-45,0,0,0,0,0,0");
        assert!(line.contains(r#","0,0,300,-45,0,0,0,0,0,0","#), "{line}");
        let header: Vec<&str> = CSV_HEADER.trim_end().split(',').collect();
        let fields = split_csv(&line);
        assert_eq!(fields.len(), 25, "{fields:?}");
        let col = |name: &str| fields[header.iter().position(|h| *h == name).unwrap()].as_str();
        assert_eq!(col("frame_str"), "0,0,300,-45,0,0,0,0,0,0");
        assert_eq!((col("v_mm_s"), col("w_deg_s")), ("300", "-45"));
        assert_eq!((col("err_dist"), col("ball_x"), col("ball_y")), ("0.25", "0.1", "-0.2"));
    }

    #[test]
    fn row_without_frame_is_unchanged() {
        assert_eq!(written_line(""), "50,3,vw,base-station,,1,blue,,,,,,,,,,300,-45,,0.25,,0.1,-0.2,,");
    }

    #[test]
    fn csv_field_escapes_only_what_needs_it() {
        assert_eq!(csv_field("goto"), "goto");
        assert_eq!(csv_field(""), "");
        assert_eq!(csv_field("1,2"), "\"1,2\"");
        assert_eq!(csv_field("a\"b"), "\"a\"\"b\"");
        assert_eq!(csv_field("a\nb"), "\"a\nb\"");
        assert_eq!(split_csv(&format!("x,{},y", csv_field("p,\"q\""))), ["x", "p,\"q\"", "y"]);
    }
}
