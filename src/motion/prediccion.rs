//! Predicción de la pose propia para compensar el retraso del robot real.
//!
//! Con los robots reales, desde que el engine manda un comando hasta que la visión ve el
//! movimiento pasan ~250-350 ms (cámara, visión, base a 20 Hz, PID del firmware y la inercia
//! del robot; medido el 5-nov-2026 con el registro de partido). El control de `Motion` y las
//! skills es proporcional y actúa sobre la pose medida, que llega atrasada: se pasa del
//! objetivo y el arquero oscila. En el simulador se prueba con ~90 ms y no pasa.
//!
//! La compensación es la del predictor de Smith: a la pose medida se le suma lo que el robot
//! va a recorrer con los comandos que ya se mandaron y todavía no se ven. Por robot propio se
//! guardan los (v, ω) enviados (los mismos de `command_to_vw`: el avance sobre el rumbo y el
//! giro), y en cada tick se integran los de los últimos `robot.prediccion_ms` sobre la pose
//! del EKF. El lazo le pasa al decisor, a las skills y a motion ese mundo predicho; la GUI y
//! el registro siguen con lo medido.
//!
//! Se usan los comandos y no la velocidad del EKF porque esa va atrasada (~0.18 s) por el
//! filtro. El avance se multiplica por `robot.prediccion_ganancia` y se topa en
//! `robot.prediccion_v_max_m_s`: en la cancha de madera el robot avanzaba ~1.5 veces lo pedido
//! (hasta 0.6 m/s con comandos de 0.41; registro de partido del 5-nov-2026), y el giro, lo pedido.
//!
//! Apagado por defecto (`robot.prediccion_ms = 0`): el mundo pasa igual.

use std::collections::{HashMap, VecDeque};

use super::MotionCommand;
use crate::world::World;

/// Un comando enviado: desde `t` (s) hasta el siguiente, avance `v` (m/s) y giro `w` (rad/s).
#[derive(Debug, Clone, Copy)]
struct Enviado {
    t: f64,
    v: f64,
    w: f64,
}

/// Comandos enviados a los robots propios y la predicción de su pose.
#[derive(Debug, Default)]
pub struct PrediccionPose {
    enviados: HashMap<i32, VecDeque<Enviado>>,
}

impl PrediccionPose {
    pub fn new() -> Self {
        Self::default()
    }

    /// Anota los comandos que se mandaron en `t` (s, monótono). `ganancia_v` es cuánto avanza
    /// de verdad el robot por cada m/s pedido; `v_max` (m/s) y `w_max` (rad/s), sus topes
    /// reales; `horizonte_s`, cuánto historial se guarda.
    #[allow(clippy::too_many_arguments)]
    pub fn registrar(
        &mut self,
        t: f64,
        comandos: &[MotionCommand],
        equipo: i32,
        horizonte_s: f64,
        ganancia_v: f64,
        v_max: f64,
        w_max: f64,
    ) {
        for c in comandos.iter().filter(|c| c.team == equipo) {
            let finito = c.vx.is_finite() && c.vy.is_finite() && c.omega.is_finite() && c.orientation.is_finite();
            // El avance que de verdad se manda: la proyección sobre el rumbo (`command_to_vw`).
            let (v, w) = if finito {
                (
                    (ganancia_v * (c.vx * c.orientation.cos() + c.vy * c.orientation.sin())).clamp(-v_max, v_max),
                    c.omega.clamp(-w_max, w_max),
                )
            } else {
                (0.0, 0.0)
            };
            let cola = self.enviados.entry(c.id).or_default();
            cola.push_back(Enviado { t, v, w });
        }
        // Se guarda un comando anterior al horizonte: es el que rige al comienzo de la ventana.
        for cola in self.enviados.values_mut() {
            while cola.len() >= 2 && cola[1].t <= t - horizonte_s {
                cola.pop_front();
            }
        }
    }

    /// El mundo con las poses de los robots propios activos adelantadas `horizonte_s` con
    /// los comandos enviados en ese lapso. Con `horizonte_s <= 0` o sin comandos, igual.
    pub fn predecir(&self, world: &World, equipo: i32, t: f64, horizonte_s: f64) -> World {
        let mut predicho = world.clone();
        if horizonte_s <= 0.0 {
            return predicho;
        }
        let inicio = t - horizonte_s;
        for robot in predicho.robots_mut(equipo) {
            if !robot.active {
                continue;
            }
            let Some(cola) = self.enviados.get(&robot.id) else { continue };
            let mut x = robot.position.x as f64;
            let mut y = robot.position.y as f64;
            let mut th = robot.orientation;
            for (i, c) in cola.iter().enumerate() {
                let desde = c.t.max(inicio);
                let hasta = cola.get(i + 1).map_or(t, |s| s.t).min(t);
                let dt = hasta - desde;
                if dt <= 0.0 {
                    continue;
                }
                // Integración con el rumbo en el punto medio del tramo.
                let th_medio = th + c.w * dt / 2.0;
                x += c.v * th_medio.cos() * dt;
                y += c.v * th_medio.sin() * dt;
                th += c.w * dt;
            }
            robot.position.x = x as f32;
            robot.position.y = y as f32;
            robot.orientation = (th + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI;
        }
        predicho
    }

    /// Olvida los comandos (por ejemplo, al salir de una parada).
    pub fn reiniciar(&mut self) {
        self.enviados.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec2;
    use std::f64::consts::{FRAC_PI_2, PI};

    fn mundo_con(robots: &[(i32, i32, f32, f32, f64)]) -> World {
        let mut w = World::new(3, 3);
        for &(id, team, x, y, th) in robots {
            w.update_robot(id, team, Vec2::new(x, y), th, Vec2::ZERO, 0.0);
        }
        w
    }

    fn cmd(id: i32, team: i32, v: f64, w: f64, th: f64) -> MotionCommand {
        MotionCommand { id, team, vx: v * th.cos(), vy: v * th.sin(), omega: w, orientation: th }
    }

    fn pose(w: &World, id: i32, team: i32) -> (f64, f64, f64) {
        let r = w.get_robot_state(id, team).expect("robot");
        (r.position.x as f64, r.position.y as f64, r.orientation)
    }

    /// Manda el mismo comando cada 1/60 s desde `t0` hasta `t1`.
    fn mandar(p: &mut PrediccionPose, t0: f64, t1: f64, c: &MotionCommand, h: f64) {
        let mut t = t0;
        while t < t1 - 1e-9 {
            p.registrar(t, std::slice::from_ref(c), 0, h, 1.0, 0.45, 12.0);
            t += 1.0 / 60.0;
        }
    }

    #[test]
    fn apagada_deja_el_mundo_igual() {
        let w = mundo_con(&[(0, 0, 0.1, 0.2, 0.3)]);
        let mut p = PrediccionPose::new();
        mandar(&mut p, 0.0, 1.0, &cmd(0, 0, 0.4, 0.0, 0.3), 0.3);
        assert_eq!(pose(&p.predecir(&w, 0, 1.0, 0.0), 0, 0), pose(&w, 0, 0));
    }

    #[test]
    fn avance_constante_adelanta_v_por_el_horizonte() {
        let w = mundo_con(&[(1, 0, 0.0, 0.0, 0.0)]);
        let mut p = PrediccionPose::new();
        mandar(&mut p, 0.0, 2.0, &cmd(1, 0, 0.4, 0.0, 0.0), 0.3);
        let (x, y, th) = pose(&p.predecir(&w, 0, 2.0, 0.3), 1, 0);
        assert!((x - 0.4 * 0.3).abs() < 1e-3, "x = {x}");
        assert!(y.abs() < 1e-6 && th.abs() < 1e-9);
    }

    #[test]
    fn los_comandos_viejos_no_cuentan_y_el_ultimo_rige_hasta_ahora() {
        let w = mundo_con(&[(0, 0, 0.0, 0.0, 0.0)]);
        let mut p = PrediccionPose::new();
        // 0.4 m/s hasta t = 1.0, después 0: en la ventana [1.2, 1.5] el robot ya va a 0.
        mandar(&mut p, 0.0, 1.0, &cmd(0, 0, 0.4, 0.0, 0.0), 0.3);
        mandar(&mut p, 1.0, 1.5, &cmd(0, 0, 0.0, 0.0, 0.0), 0.3);
        assert!(pose(&p.predecir(&w, 0, 1.5, 0.3), 0, 0).0.abs() < 1e-9);
        // Ventana [0.8, 1.1]: 0.2 s a 0.4 m/s y 0.1 s parado.
        let mut q = PrediccionPose::new();
        mandar(&mut q, 0.0, 1.0, &cmd(0, 0, 0.4, 0.0, 0.0), 0.3);
        mandar(&mut q, 1.0, 1.1, &cmd(0, 0, 0.0, 0.0, 0.0), 0.3);
        let x = pose(&q.predecir(&w, 0, 1.1, 0.3), 0, 0).0;
        assert!((x - 0.08).abs() < 0.008, "x = {x}");
    }

    #[test]
    fn el_avance_se_topa_y_respeta_el_rumbo_y_la_marcha_atras() {
        let w = mundo_con(&[(2, 0, 0.0, 0.0, FRAC_PI_2)]);
        let mut p = PrediccionPose::new();
        // Pide 1 m/s hacia atrás: el robot real no pasa de 0.45.
        mandar(&mut p, 0.0, 1.0, &cmd(2, 0, -1.0, 0.0, FRAC_PI_2), 0.2);
        let (x, y, _) = pose(&p.predecir(&w, 0, 1.0, 0.2), 2, 0);
        assert!(x.abs() < 1e-6 && (y + 0.45 * 0.2).abs() < 1e-3, "({x}, {y})");
    }

    #[test]
    fn el_giro_se_integra_y_curva_la_trayectoria() {
        let w = mundo_con(&[(0, 0, 0.0, 0.0, 0.0)]);
        let mut p = PrediccionPose::new();
        // Giro puro de π rad/s durante 0.3 s.
        mandar(&mut p, 0.0, 1.0, &cmd(0, 0, 0.0, PI, 0.0), 0.3);
        let (_, _, th) = pose(&p.predecir(&w, 0, 1.0, 0.3), 0, 0);
        assert!((th - 0.3 * PI).abs() < 1e-3, "θ = {th}");
        // Avance con giro: termina a la izquierda (y > 0) y no tan adelante como en recta.
        let mut q = PrediccionPose::new();
        mandar(&mut q, 0.0, 1.0, &cmd(0, 0, 0.4, 2.0, 0.0), 0.3);
        let (x, y, _) = pose(&q.predecir(&w, 0, 1.0, 0.3), 0, 0);
        assert!(y > 0.01 && x < 0.12 && x > 0.08, "({x}, {y})");
    }

    #[test]
    fn solo_cambia_a_los_propios_activos() {
        let w = mundo_con(&[(0, 0, 0.0, 0.0, 0.0), (0, 1, 0.5, 0.5, 0.0)]);
        let mut p = PrediccionPose::new();
        mandar(&mut p, 0.0, 1.0, &cmd(0, 0, 0.4, 0.0, 0.0), 0.3);
        p.registrar(1.0, &[cmd(0, 1, 0.4, 0.0, 0.0)], 0, 0.3, 1.0, 0.45, 12.0); // de otro equipo: se ignora
        let predicho = p.predecir(&w, 0, 1.0, 0.3);
        assert!(pose(&predicho, 0, 0).0 > 0.1);
        assert_eq!(pose(&predicho, 0, 1), pose(&w, 0, 1));
    }

    #[test]
    fn la_ganancia_escala_el_avance_y_no_el_giro_y_respeta_el_tope() {
        let w = mundo_con(&[(0, 0, 0.0, 0.0, 0.0)]);
        let mut p = PrediccionPose::new();
        let mut t = 0.0;
        while t < 1.0 - 1e-9 {
            p.registrar(t, &[cmd(0, 0, 0.3, 1.0, 0.0)], 0, 0.3, 1.5, 0.7, 12.0);
            t += 1.0 / 60.0;
        }
        let (_, _, th) = pose(&p.predecir(&w, 0, 1.0, 0.3), 0, 0);
        assert!((th - 0.3).abs() < 1e-3, "θ = {th}");
        // Recta: 0.3 m/s pedidos y 1.5 de ganancia = 0.45 m/s reales durante 0.3 s.
        let mut q = PrediccionPose::new();
        let mut t = 0.0;
        while t < 1.0 - 1e-9 {
            q.registrar(t, &[cmd(0, 0, 0.3, 0.0, 0.0)], 0, 0.3, 1.5, 0.7, 12.0);
            t += 1.0 / 60.0;
        }
        let x = pose(&q.predecir(&w, 0, 1.0, 0.3), 0, 0).0;
        assert!((x - 0.45 * 0.3).abs() < 1e-3, "x = {x}");
        // 0.6 pedidos con 1.5 = 0.9: se topa en 0.7.
        let mut r = PrediccionPose::new();
        let mut t = 0.0;
        while t < 1.0 - 1e-9 {
            r.registrar(t, &[cmd(0, 0, 0.6, 0.0, 0.0)], 0, 0.3, 1.5, 0.7, 12.0);
            t += 1.0 / 60.0;
        }
        let x = pose(&r.predecir(&w, 0, 1.0, 0.3), 0, 0).0;
        assert!((x - 0.7 * 0.3).abs() < 1e-3, "x = {x}");
    }

    #[test]
    fn un_comando_no_finito_cuenta_como_parado() {
        let w = mundo_con(&[(0, 0, 0.0, 0.0, 0.0)]);
        let mut p = PrediccionPose::new();
        let malo = MotionCommand { id: 0, team: 0, vx: f64::NAN, vy: 0.0, omega: 0.0, orientation: 0.0 };
        mandar(&mut p, 0.0, 1.0, &malo, 0.3);
        assert_eq!(pose(&p.predecir(&w, 0, 1.0, 0.3), 0, 0).0, 0.0);
    }
}
