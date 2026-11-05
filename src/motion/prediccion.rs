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
//! Todo el mundo se adelanta al mismo instante: la pelota con su velocidad y el frenado de
//! `skills.intercept_ball_decay_per_s`, y los rivales con su velocidad. Si solo se adelantan los
//! propios, el robot que empuja la pelota se ve encima de una pelota que quedó 250 ms atrás,
//! frena y la pierde: en FIRASim con 250 ms de retraso de visión, nuestros penales bajaron de 40
//! de 40 goles a 10 de 40 (5-nov-2026).
//!
//! Adelantar también la pelota no alcanza cuando la empujamos: la pelota medida sigue quieta
//! (el empujón todavía no se ve) y el robot adelantado queda adentro de ella (penales: 6 de 40).
//! Por eso, cerca de la pelota no se adelanta: un robot propio a menos de
//! `robot.prediccion_radio_pelota_m` de la pelota se adelanta menos (nada a la mitad de ese
//! radio), y la pelota se adelanta lo que permita el robot propio más cerca. El que la empuja
//! juega como sin predicción; el arquero y los que van hacia un punto, compensados.
//!
//! Se usan los comandos y no la velocidad del EKF porque esa va atrasada (~0.18 s) por el
//! filtro. El avance se multiplica por `robot.prediccion_ganancia` y se topa en
//! `robot.prediccion_v_max_m_s`: en la cancha de madera el robot avanzaba ~1.5 veces lo pedido
//! (hasta 0.6 m/s con comandos de 0.41; registro de partido del 5-nov-2026), y el giro, lo pedido.
//!
//! Apagado por defecto (`robot.prediccion_ms = 0`): el mundo pasa igual.

use std::collections::{HashMap, VecDeque};

use glam::Vec2;

use super::MotionCommand;
use crate::world::World;

/// Hasta dónde se adelanta la pelota o un rival (m): la cancha con los arcos (el adelanto choca
/// con la pared en vez de salirse).
const LIMITE_X_M: f32 = 0.85;
const LIMITE_Y_M: f32 = 0.65;

fn dentro(p: Vec2) -> Vec2 {
    Vec2::new(p.x.clamp(-LIMITE_X_M, LIMITE_X_M), p.y.clamp(-LIMITE_Y_M, LIMITE_Y_M))
}

/// Cuánto se adelanta (0 a 1) un robot propio a distancia `d` (m) de la pelota: nada hasta
/// `radio / 2`, todo desde `radio`, lineal entre medio. Con `radio <= 0`, siempre todo.
fn peso_por_pelota(d: f32, radio: f32) -> f32 {
    if radio <= 0.0 || !d.is_finite() {
        return 1.0;
    }
    ((d - radio / 2.0) / (radio / 2.0)).clamp(0.0, 1.0)
}

fn angulo(a: f64) -> f64 {
    (a + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

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

    /// El mundo adelantado `horizonte_s`: los robots propios activos con los comandos enviados
    /// en ese lapso (sin comandos, quietos), los rivales activos con su velocidad y la pelota
    /// con su velocidad y el frenado exponencial `frenado_pelota` (1/s). Cerca de la pelota
    /// (`radio_pelota`, m) los propios y la pelota se adelantan menos (ver el módulo). Con
    /// `horizonte_s <= 0`, igual.
    pub fn predecir(
        &self,
        world: &World,
        equipo: i32,
        t: f64,
        horizonte_s: f64,
        frenado_pelota: f64,
        radio_pelota: f64,
    ) -> World {
        let mut predicho = world.clone();
        if horizonte_s <= 0.0 {
            return predicho;
        }
        let pelota = world.get_ball_state();
        let radio = radio_pelota as f32;
        let peso_de = |p: Vec2| {
            if pelota.position.is_finite() { peso_por_pelota(p.distance(pelota.position), radio) } else { 1.0 }
        };
        let peso_pelota = world
            .robots_posiciones(equipo)
            .fold(1.0_f32, |m, (activo, p)| if activo { m.min(peso_de(p)) } else { m });
        let h = horizonte_s as f32;
        if pelota.position.is_finite() && pelota.velocity.is_finite() && peso_pelota > 0.0 {
            // v(t) = v0 e^(-k t): recorre v0 (1 - e^(-k h)) / k.
            let k = frenado_pelota.max(0.0) as f32;
            let (recorre, queda) = if k > 1e-6 {
                ((1.0 - (-k * h).exp()) / k, (-k * h).exp())
            } else {
                (h, 1.0)
            };
            let recorre = recorre * peso_pelota;
            let queda = 1.0 + (queda - 1.0) * peso_pelota;
            predicho.update_ball(dentro(pelota.position + pelota.velocity * recorre), pelota.velocity * queda);
        }
        for rival in predicho.robots_mut(1 - equipo) {
            if rival.active && rival.position.is_finite() && rival.velocity.is_finite() {
                rival.position = dentro(rival.position + rival.velocity * h);
                if rival.angular_velocity.is_finite() {
                    rival.orientation = (rival.orientation + rival.angular_velocity * horizonte_s + std::f64::consts::PI)
                        .rem_euclid(std::f64::consts::TAU)
                        - std::f64::consts::PI;
                }
            }
        }
        let inicio = t - horizonte_s;
        for robot in predicho.robots_mut(equipo) {
            if !robot.active {
                continue;
            }
            let Some(cola) = self.enviados.get(&robot.id) else { continue };
            let peso = f64::from(peso_de(robot.position));
            if peso <= 0.0 {
                continue;
            }
            let (x0, y0, th0) = (robot.position.x as f64, robot.position.y as f64, robot.orientation);
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
            robot.position.x = (x0 + peso * (x - x0)) as f32;
            robot.position.y = (y0 + peso * (y - y0)) as f32;
            robot.orientation = angulo(th0 + peso * angulo(th - th0));
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
        assert_eq!(pose(&p.predecir(&w, 0, 1.0, 0.0, 0.3, 0.0), 0, 0), pose(&w, 0, 0));
    }

    #[test]
    fn avance_constante_adelanta_v_por_el_horizonte() {
        let w = mundo_con(&[(1, 0, 0.0, 0.0, 0.0)]);
        let mut p = PrediccionPose::new();
        mandar(&mut p, 0.0, 2.0, &cmd(1, 0, 0.4, 0.0, 0.0), 0.3);
        let (x, y, th) = pose(&p.predecir(&w, 0, 2.0, 0.3, 0.3, 0.0), 1, 0);
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
        assert!(pose(&p.predecir(&w, 0, 1.5, 0.3, 0.3, 0.0), 0, 0).0.abs() < 1e-9);
        // Ventana [0.8, 1.1]: 0.2 s a 0.4 m/s y 0.1 s parado.
        let mut q = PrediccionPose::new();
        mandar(&mut q, 0.0, 1.0, &cmd(0, 0, 0.4, 0.0, 0.0), 0.3);
        mandar(&mut q, 1.0, 1.1, &cmd(0, 0, 0.0, 0.0, 0.0), 0.3);
        let x = pose(&q.predecir(&w, 0, 1.1, 0.3, 0.3, 0.0), 0, 0).0;
        assert!((x - 0.08).abs() < 0.008, "x = {x}");
    }

    #[test]
    fn el_avance_se_topa_y_respeta_el_rumbo_y_la_marcha_atras() {
        let w = mundo_con(&[(2, 0, 0.0, 0.0, FRAC_PI_2)]);
        let mut p = PrediccionPose::new();
        // Pide 1 m/s hacia atrás: el robot real no pasa de 0.45.
        mandar(&mut p, 0.0, 1.0, &cmd(2, 0, -1.0, 0.0, FRAC_PI_2), 0.2);
        let (x, y, _) = pose(&p.predecir(&w, 0, 1.0, 0.2, 0.3, 0.0), 2, 0);
        assert!(x.abs() < 1e-6 && (y + 0.45 * 0.2).abs() < 1e-3, "({x}, {y})");
    }

    #[test]
    fn el_giro_se_integra_y_curva_la_trayectoria() {
        let w = mundo_con(&[(0, 0, 0.0, 0.0, 0.0)]);
        let mut p = PrediccionPose::new();
        // Giro puro de π rad/s durante 0.3 s.
        mandar(&mut p, 0.0, 1.0, &cmd(0, 0, 0.0, PI, 0.0), 0.3);
        let (_, _, th) = pose(&p.predecir(&w, 0, 1.0, 0.3, 0.3, 0.0), 0, 0);
        assert!((th - 0.3 * PI).abs() < 1e-3, "θ = {th}");
        // Avance con giro: termina a la izquierda (y > 0) y no tan adelante como en recta.
        let mut q = PrediccionPose::new();
        mandar(&mut q, 0.0, 1.0, &cmd(0, 0, 0.4, 2.0, 0.0), 0.3);
        let (x, y, _) = pose(&q.predecir(&w, 0, 1.0, 0.3, 0.3, 0.0), 0, 0);
        assert!(y > 0.01 && x < 0.12 && x > 0.08, "({x}, {y})");
    }

    #[test]
    fn los_comandos_del_otro_equipo_no_mueven_a_nadie() {
        let w = mundo_con(&[(0, 0, 0.0, 0.0, 0.0), (0, 1, 0.5, 0.5, 0.0)]);
        let mut p = PrediccionPose::new();
        mandar(&mut p, 0.0, 1.0, &cmd(0, 0, 0.4, 0.0, 0.0), 0.3);
        p.registrar(1.0, &[cmd(0, 1, 0.4, 0.0, 0.0)], 0, 0.3, 1.0, 0.45, 12.0); // de otro equipo: se ignora
        let predicho = p.predecir(&w, 0, 1.0, 0.3, 0.3, 0.0);
        assert!(pose(&predicho, 0, 0).0 > 0.1);
        // El rival está quieto (velocidad cero): no se mueve aunque haya comandos con su id.
        assert_eq!(pose(&predicho, 0, 1), pose(&w, 0, 1));
    }

    #[test]
    fn la_pelota_y_los_rivales_se_adelantan_con_su_velocidad() {
        let mut w = World::new(3, 3);
        w.update_robot(1, 1, Vec2::new(0.0, 0.2), 0.0, Vec2::new(0.4, 0.0), 1.0);
        w.update_ball(Vec2::new(0.1, 0.0), Vec2::new(1.0, 0.0));
        let p = PrediccionPose::new();
        let predicho = p.predecir(&w, 0, 1.0, 0.25, 0.3, 0.0);
        let b = predicho.get_ball_state();
        // 1 m/s con k = 0.3 durante 0.25 s: (1 - e^-0.075) / 0.3 = 0.2408 m; queda a 0.928 m/s.
        assert!((b.position.x - (0.1 + 0.2408)).abs() < 1e-3, "pelota x = {}", b.position.x);
        assert!((b.velocity.x - 0.9277).abs() < 1e-3, "pelota vx = {}", b.velocity.x);
        let (x, y, th) = pose(&predicho, 1, 1);
        assert!((x - 0.1).abs() < 1e-4 && (y - 0.2).abs() < 1e-6 && (th - 0.25).abs() < 1e-6, "({x}, {y}, {th})");
        // Sin frenado, v * h; y el adelanto no sale de la cancha.
        let mut w2 = World::new(3, 3);
        w2.update_ball(Vec2::new(0.8, 0.0), Vec2::new(1.0, 0.0));
        let b2 = p.predecir(&w2, 0, 1.0, 0.25, 0.0, 0.0).get_ball_state().position;
        assert!((b2.x - LIMITE_X_M).abs() < 1e-6, "pelota x = {}", b2.x);
        // Apagada: la pelota queda donde está.
        assert_eq!(p.predecir(&w, 0, 1.0, 0.0, 0.3, 0.0).get_ball_state().position, Vec2::new(0.1, 0.0));
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
        let (_, _, th) = pose(&p.predecir(&w, 0, 1.0, 0.3, 0.3, 0.0), 0, 0);
        assert!((th - 0.3).abs() < 1e-3, "θ = {th}");
        // Recta: 0.3 m/s pedidos y 1.5 de ganancia = 0.45 m/s reales durante 0.3 s.
        let mut q = PrediccionPose::new();
        let mut t = 0.0;
        while t < 1.0 - 1e-9 {
            q.registrar(t, &[cmd(0, 0, 0.3, 0.0, 0.0)], 0, 0.3, 1.5, 0.7, 12.0);
            t += 1.0 / 60.0;
        }
        let x = pose(&q.predecir(&w, 0, 1.0, 0.3, 0.3, 0.0), 0, 0).0;
        assert!((x - 0.45 * 0.3).abs() < 1e-3, "x = {x}");
        // 0.6 pedidos con 1.5 = 0.9: se topa en 0.7.
        let mut r = PrediccionPose::new();
        let mut t = 0.0;
        while t < 1.0 - 1e-9 {
            r.registrar(t, &[cmd(0, 0, 0.6, 0.0, 0.0)], 0, 0.3, 1.5, 0.7, 12.0);
            t += 1.0 / 60.0;
        }
        let x = pose(&r.predecir(&w, 0, 1.0, 0.3, 0.3, 0.0), 0, 0).0;
        assert!((x - 0.7 * 0.3).abs() < 1e-3, "x = {x}");
    }

    #[test]
    fn cerca_de_la_pelota_no_se_adelanta_ni_el_robot_ni_la_pelota() {
        // Robot en x = 0 avanzando a 0.4 m/s hacia +x; pelota adelante, con velocidad.
        let con_pelota_a = |d: f32| {
            let mut w = mundo_con(&[(0, 0, 0.0, 0.0, 0.0)]);
            w.update_ball(Vec2::new(d, 0.0), Vec2::new(0.5, 0.0));
            w
        };
        let mut p = PrediccionPose::new();
        mandar(&mut p, 0.0, 1.0, &cmd(0, 0, 0.4, 0.0, 0.0), 0.3);
        // Lejos (0.5 m, radio 0.3): todo. Robot +0.12 m y pelota adelantada.
        let lejos = p.predecir(&con_pelota_a(0.5), 0, 1.0, 0.3, 0.0, 0.3);
        assert!((pose(&lejos, 0, 0).0 - 0.12).abs() < 1e-3);
        assert!((lejos.get_ball_state().position.x - (0.5 + 0.15)).abs() < 1e-4);
        // Pegado (0.1 m < radio / 2): nada, ni el robot ni la pelota.
        let cerca = p.predecir(&con_pelota_a(0.1), 0, 1.0, 0.3, 0.0, 0.3);
        assert_eq!(pose(&cerca, 0, 0).0, 0.0);
        assert_eq!(cerca.get_ball_state().position.x, 0.1);
        // A 0.225 m (en medio): la mitad.
        let medio = p.predecir(&con_pelota_a(0.225), 0, 1.0, 0.3, 0.0, 0.3);
        assert!((pose(&medio, 0, 0).0 - 0.06).abs() < 1e-3, "x = {}", pose(&medio, 0, 0).0);
        assert!((medio.get_ball_state().position.x - (0.225 + 0.075)).abs() < 1e-4);
        // Radio 0: siempre todo.
        let siempre = p.predecir(&con_pelota_a(0.1), 0, 1.0, 0.3, 0.0, 0.0);
        assert!((pose(&siempre, 0, 0).0 - 0.12).abs() < 1e-3);
    }

    #[test]
    fn un_comando_no_finito_cuenta_como_parado() {
        let w = mundo_con(&[(0, 0, 0.0, 0.0, 0.0)]);
        let mut p = PrediccionPose::new();
        let malo = MotionCommand { id: 0, team: 0, vx: f64::NAN, vy: 0.0, omega: 0.0, orientation: 0.0 };
        mandar(&mut p, 0.0, 1.0, &malo, 0.3);
        assert_eq!(pose(&p.predecir(&w, 0, 1.0, 0.3, 0.3, 0.0), 0, 0).0, 0.0);
    }
}
