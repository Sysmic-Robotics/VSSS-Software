//! Planta diferencial para tests (solo `cfg(test)`).
//!
//! Reproduce lo que importa del FIRASim para motion, sin depender de él:
//! - conversión `(v, ω)` → ruedas con la geometría de `params().sim`, escalando ambas
//!   ruedas por el mismo factor si una satura (como el firmware y el serializador FIRA);
//! - límite de aceleración por rueda: si las dos ruedas tienen que acelerar mucho, las
//!   dos van al límite y no queda diferencial (se pierde el giro, como en FIRASim con
//!   el torque limitado). Acelera a 1.2 m/s² y frena a 3.0 m/s², como lo medido en
//!   FIRASim (frenado de 2.6–3.6 m/s² cuando el comando cae a cero);
//! - latencia de N ticks entre el comando y las ruedas;
//! - paredes físicas: el centro no sale de ±(0.75 − 0.04) × ±(0.65 − 0.04).
//!
//! `run` arma el `World` con la pose y la velocidad de la planta, despacha con el mismo
//! `SkillCatalog` que el loop (con motion rodeando las áreas, como en el loop) y aplica
//! `control_loop::apply_reflexes`.

use std::collections::{HashSet, VecDeque};

use glam::Vec2;

use super::{BorderRecovery, Motion, MotionCommand, MotionConfig};
use crate::control_loop::apply_reflexes;
use crate::skills::zones::ZoneGuard;
use crate::skills::{SkillCatalog, SkillId};
use crate::world::World;

/// Paso de integración (s): la planta corre a 60 Hz, como el loop.
pub(crate) const DT: f64 = 1.0 / 60.0;

pub(crate) struct Plant {
    pub x: f32,
    pub y: f32,
    pub th: f64,
    /// Velocidad lineal de cada rueda (m/s).
    wl: f64,
    wr: f64,
    queue: VecDeque<(f64, f64)>,
    /// Aceleración máxima por rueda al ganar velocidad (m/s²).
    pub wheel_accel: f64,
    /// Desaceleración máxima por rueda al perder velocidad (m/s²).
    pub wheel_brake: f64,
    /// Ticks de latencia comando → ruedas.
    pub latency: usize,
}

impl Plant {
    pub fn new(x: f32, y: f32, deg: f64) -> Self {
        Self {
            x,
            y,
            th: deg.to_radians(),
            wl: 0.0,
            wr: 0.0,
            queue: VecDeque::new(),
            wheel_accel: 1.2,
            wheel_brake: 3.0,
            latency: 5,
        }
    }

    pub fn v(&self) -> f64 {
        (self.wl + self.wr) * 0.5
    }

    pub fn velocity(&self) -> Vec2 {
        let v = self.v();
        Vec2::new((v * self.th.cos()) as f32, (v * self.th.sin()) as f32)
    }

    /// Velocidad de una rueda tras un tick hacia `target`: frena (pierde magnitud) a
    /// `wheel_brake` y acelera a `wheel_accel`.
    fn wheel_toward(&self, w: f64, target: f64) -> f64 {
        let braking = w != 0.0 && (target - w).signum() != w.signum();
        let lim = if braking { self.wheel_brake } else { self.wheel_accel } * DT;
        w + (target - w).clamp(-lim, lim)
    }

    /// Avanza un tick con el comando `cmd` (marco mundo, proyectado al heading con
    /// `cmd.orientation`, como hacen `command_to_vw` y el serializador FIRA).
    pub fn step(&mut self, cmd: &MotionCommand) {
        let sim = &crate::params::params().sim;
        let half_track = sim.wheel_base_m / 2.0;
        let max_wheel = sim.max_wheel_rad_s * sim.wheel_radius_m;
        let v = cmd.vx * cmd.orientation.cos() + cmd.vy * cmd.orientation.sin();
        let (mut l, mut r) = (v - cmd.omega * half_track, v + cmd.omega * half_track);
        let m = l.abs().max(r.abs());
        if m > max_wheel {
            l *= max_wheel / m;
            r *= max_wheel / m;
        }
        self.queue.push_back((l, r));
        let (tl, tr) = if self.queue.len() > self.latency {
            self.queue.pop_front().unwrap()
        } else {
            (0.0, 0.0)
        };
        self.wl = self.wheel_toward(self.wl, tl);
        self.wr = self.wheel_toward(self.wr, tr);
        let v = self.v();
        let w = (self.wr - self.wl) / sim.wheel_base_m;
        self.x = (self.x + (v * self.th.cos() * DT) as f32).clamp(-0.71, 0.71);
        self.y = (self.y + (v * self.th.sin() * DT) as f32).clamp(-0.61, 0.61);
        self.th = Motion::normalize_angle(self.th + w * DT);
    }
}

/// Un tick de una corrida: pose antes del comando, si la recuperación estaba en escape,
/// el `done` de la skill y la `omega` comandada (después de los reflejos).
#[derive(Clone)]
pub(crate) struct Step {
    pub x: f32,
    pub y: f32,
    pub th: f64,
    pub escaping: bool,
    pub done: bool,
    pub omega: f64,
}

pub(crate) struct Trace {
    pub steps: Vec<Step>,
}

impl Trace {
    /// Primer tick en que el robot está a menos de `tol` de `p`.
    pub fn first_within(&self, p: Vec2, tol: f32) -> Option<usize> {
        self.steps
            .iter()
            .position(|s| (Vec2::new(s.x, s.y) - p).length() < tol)
    }

    /// Giro acumulado (grados) en los primeros `ticks` ticks.
    pub fn turn_deg(&self, ticks: usize) -> f64 {
        self.steps
            .windows(2)
            .take(ticks)
            .map(|w| Motion::normalize_angle(w[1].th - w[0].th).abs())
            .sum::<f64>()
            .to_degrees()
    }

    pub fn last(&self) -> &Step {
        self.steps.last().expect("corrida vacía")
    }
}

/// Configuración de una corrida en la planta (robot azul 0, defiende la izquierda).
pub(crate) struct Case {
    pub skill: SkillId,
    pub target: Vec2,
    /// x, y, heading en grados.
    pub pose: (f32, f32, f64),
    pub ball: Vec2,
    pub ticks: usize,
    pub motion: MotionConfig,
    pub keeper_id: i32,
    /// Ticks de latencia comando → ruedas de la planta.
    pub latency: usize,
}

impl Case {
    pub fn new(skill: SkillId, target: Vec2, pose: (f32, f32, f64), ball: Vec2) -> Self {
        Self {
            skill,
            target,
            pose,
            ball,
            ticks: 360,
            motion: MotionConfig::default(),
            keeper_id: 2,
            latency: 5,
        }
    }

    pub fn bidirectional(mut self) -> Self {
        self.motion.bidirectional = true;
        self
    }
}

pub(crate) fn run(case: &Case) -> Trace {
    let motion = Motion::with_config(case.motion.clone()).with_areas(1.0, case.keeper_id);
    let mut catalog = SkillCatalog::new(3);
    let mut recovery = BorderRecovery::new(true);
    let guard = ZoneGuard::new(1.0, case.keeper_id);
    let manual = HashSet::new();
    let mut plant = Plant::new(case.pose.0, case.pose.1, case.pose.2);
    plant.latency = case.latency;
    let mut steps = Vec::with_capacity(case.ticks);
    for _ in 0..case.ticks {
        let mut world = World::new(3, 3);
        world.update_robot(0, 0, Vec2::new(plant.x, plant.y), plant.th, plant.velocity(), 0.0);
        world.update_ball(case.ball, Vec2::ZERO);
        let robot = world.get_robot_state(0, 0).expect("robot 0").clone();
        let cmd = catalog.tick(0, case.skill, case.target, &robot, &world, &motion);
        let mut cmds = vec![cmd];
        let escaping = apply_reflexes(&mut cmds, &world, &mut recovery, &guard, &manual, 0)[0];
        let done = catalog.status(0, case.skill, case.target, &robot, &world).done;
        steps.push(Step {
            x: plant.x,
            y: plant.y,
            th: plant.th,
            escaping,
            done,
            omega: cmds[0].omega,
        });
        plant.step(&cmds[0]);
    }
    Trace { steps }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cmd(v: f64, w: f64, th: f64) -> MotionCommand {
        MotionCommand {
            id: 0,
            team: 0,
            vx: v * th.cos(),
            vy: v * th.sin(),
            omega: w,
            orientation: th,
        }
    }

    #[test]
    fn constant_forward_command_goes_straight() {
        let mut p = Plant::new(-0.5, 0.0, 0.0);
        for _ in 0..120 {
            let c = cmd(0.5, 0.0, p.th);
            p.step(&c);
        }
        assert!(p.x > -0.1, "avanzó: x={}", p.x);
        assert!(p.y.abs() < 1e-6 && p.th.abs() < 1e-9);
    }

    #[test]
    fn speed_step_loses_the_turn_while_wheels_accelerate() {
        // Saltar de 0 a 1.2 m/s pidiendo ω = 3: las dos ruedas aceleran al límite y no
        // hay diferencial (en 0.5 s una planta ideal giraría 1.5 rad).
        let mut p = Plant::new(-0.5, 0.0, 0.0);
        for _ in 0..30 {
            let c = cmd(1.2, 3.0, p.th);
            p.step(&c);
        }
        assert!(p.th.abs() < 0.05, "giro perdido: θ={:.3}", p.th);
        // Con v constante a 0.3 m/s (ruedas sin saturar) ω = 3 sí se ejecuta.
        let mut q = Plant::new(-0.5, 0.0, 0.0);
        for _ in 0..60 {
            let c = cmd(0.3, 3.0, q.th);
            q.step(&c);
        }
        assert!(q.th.abs() > 1.0, "con ruedas sin saturar gira: θ={:.3}", q.th);
    }
}
