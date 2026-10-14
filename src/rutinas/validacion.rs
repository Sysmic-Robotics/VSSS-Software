//! Validación de una rutina al cargarla (D3 del design). Junta todas las razones, cada
//! una en frase con su ubicación; sin razones, la rutina es válida.
//!
//! Reglas:
//! 1. `1 ≤ min ≤ max ≤ 3`, un puesto por robot, nombres únicos, a lo sumo un arquero;
//! 2. tiempos, vueltas y radios > 0;
//! 3. zona segura: todo destino a ≥ `zona_segura_m` de las paredes y de los triángulos
//!    de las esquinas (el punto de `mirar` no es un destino);
//! 4. áreas: ningún destino ni tramo recto toca un área prohibida con la holgura de
//!    navegación (`AREA_CLEARANCE`);
//! 5. tolerancias ≥ `motion.arrival_threshold`;
//! 6. inicios a ≥ `distancia_minima_m` entre sí (regla del archivo; no aborta en vivo);
//! 7. los pasos de las fases 2 y 3 y la pelota dejan la rutina "no disponible".

use super::formato::{Accion, Duracion, Figura, Hacia, Rutina};
use crate::coach::{FIELD_HALF_X, FIELD_HALF_Y};
use crate::params::{RutinasParams, params};
use crate::skills::zones::{AREA_CLEARANCE, AreaRect, TRIANGLE_M, attack_sign_from_env};
use glam::Vec2;

/// Robots por equipo en VSSS (3 vs 3).
pub const MAX_ROBOTS: usize = 3;
/// Paso de muestreo de los tramos rectos contra las áreas (m).
const PASO_MUESTREO: f32 = 0.01;
/// Holgura numérica en las comparaciones de distancias (m).
const EPS: f32 = 1e-5;

/// Lo que la validación necesita de afuera: los parámetros y el lado de la cancha.
#[derive(Debug, Clone, PartialEq)]
pub struct ContextoValidacion {
    pub params: RutinasParams,
    pub arrival_threshold: f32,
    /// +1 = atacamos hacia +x (área propia a la izquierda).
    pub attack_sign: f32,
}

impl ContextoValidacion {
    /// Con los parámetros vigentes y el lado de `VSSL_SIDE` para el equipo propio.
    pub fn vigente(own_team: i32) -> Self {
        let p = params();
        Self {
            params: p.rutinas.clone(),
            arrival_threshold: p.motion.arrival_threshold,
            attack_sign: attack_sign_from_env(own_team),
        }
    }

    /// Con los parámetros por defecto y el lado dado (tests).
    pub fn por_defecto(attack_sign: f32) -> Self {
        let p = crate::params::TeamParams::default();
        Self { params: p.rutinas, arrival_threshold: p.motion.arrival_threshold, attack_sign }
    }
}

fn punto(p: Vec2) -> String {
    format!("({:.2}, {:.2})", p.x, p.y)
}

fn nombre_area(a: &AreaRect) -> &'static str {
    if a.side < 0.0 { "el área izquierda" } else { "el área derecha" }
}

/// Por qué un destino queda fuera de la zona segura, o `None`.
pub fn fuera_de_zona(p: Vec2, margen: f32) -> Option<String> {
    let pared = (FIELD_HALF_X - p.x.abs()).min(FIELD_HALF_Y - p.y.abs());
    if pared < margen - EPS {
        return Some(format!(
            "{} está a {:.2} m de la pared (mínimo {:.2})",
            punto(p),
            pared.max(0.0),
            margen
        ));
    }
    // Hipotenusa del triángulo de la esquina: |x| + |y| = X + Y − cateto.
    let tri = (FIELD_HALF_X + FIELD_HALF_Y - TRIANGLE_M - p.x.abs() - p.y.abs()) / std::f32::consts::SQRT_2;
    if tri < margen - EPS {
        return Some(format!(
            "{} está a {:.2} m del triángulo de la esquina (mínimo {:.2})",
            punto(p),
            tri.max(0.0),
            margen
        ));
    }
    None
}

/// `true` si algún punto del tramo `a`–`b` toca el área con la holgura de navegación.
pub fn tramo_toca(a: Vec2, b: Vec2, area: &AreaRect) -> bool {
    let n = ((b - a).length() / PASO_MUESTREO).ceil().max(1.0) as usize;
    (0..=n).any(|i| area.touches_with(a.lerp(b, i as f32 / n as f32), AREA_CLEARANCE))
}

/// Áreas prohibidas para un puesto: las dos para uno común; la rival para el arquero.
fn areas_prohibidas(arquero: bool, attack_sign: f32) -> Vec<AreaRect> {
    if arquero {
        vec![AreaRect::opp(attack_sign)]
    } else {
        vec![AreaRect::own(attack_sign), AreaRect::opp(attack_sign)]
    }
}

fn positivo(v: f64) -> bool {
    v.is_finite() && v > 0.0
}

fn nombre_de_puesto_valido(n: &str) -> bool {
    !n.is_empty() && n.len() <= 8 && n.chars().all(|c| c.is_ascii_alphanumeric())
}

/// Todas las razones por las que la rutina no se puede correr.
pub fn validar(r: &Rutina, ctx: &ContextoValidacion) -> Vec<String> {
    let mut razones = Vec::new();
    let p = &ctx.params;

    // 1. Estructura.
    if r.nombre.trim().is_empty() {
        razones.push("falta el nombre".to_string());
    }
    if r.descripcion.contains('\n') {
        razones.push("la descripción va en una sola línea".to_string());
    }
    if !(1 <= r.robots_min && r.robots_min <= r.robots_max && r.robots_max <= MAX_ROBOTS) {
        razones.push(format!(
            "robots: min {} y max {} deben cumplir 1 ≤ min ≤ max ≤ {MAX_ROBOTS}",
            r.robots_min, r.robots_max
        ));
    }
    if r.puestos.len() != r.robots_max {
        let n = r.puestos.len();
        razones.push(format!(
            "la rutina tiene {n} {} y robots.max es {}: va un puesto por robot",
            if n == 1 { "puesto" } else { "puestos" },
            r.robots_max
        ));
    }
    for (i, a) in r.puestos.iter().enumerate() {
        if !nombre_de_puesto_valido(&a.nombre) {
            razones.push(format!(
                "el nombre de puesto `{}` solo admite letras y dígitos (hasta 8)",
                a.nombre
            ));
        }
        if r.puestos[..i].iter().any(|b| b.nombre == a.nombre) {
            razones.push(format!("el puesto {} está repetido", a.nombre));
        }
        if a.pasos.is_empty() {
            razones.push(format!("el puesto {} no tiene pasos", a.nombre));
        }
    }
    let arqueros = r.puestos.iter().filter(|a| a.arquero).count();
    if arqueros > 1 {
        razones.push(format!("hay {arqueros} puestos de arquero; va a lo sumo uno"));
    }

    // 2 y 5. Valores de cada paso.
    for a in &r.puestos {
        if !a.inicio.orientacion.is_finite() {
            razones.push(format!("puesto {}, inicio: la orientación no es un número", a.nombre));
        }
        for (i, paso) in a.pasos.iter().enumerate() {
            let donde = format!("puesto {}, paso {}", a.nombre, i + 1);
            let mut malo = |que: &str| razones.push(format!("{donde}: {que}"));
            match &paso.accion {
                Accion::Ir { orientacion: Some(o), .. } if !o.is_finite() => malo("la orientación no es un número"),
                Accion::Girar { duracion: Duracion::Vueltas(v), .. } if !positivo(*v) => malo("`vueltas` debe ser mayor que cero"),
                Accion::Girar { duracion: Duracion::Tiempo(t), .. } if !positivo(*t) => malo("`tiempo_s` debe ser mayor que cero"),
                Accion::Esperar { tiempo_s } if !positivo(*tiempo_s) => malo("`tiempo_s` debe ser mayor que cero"),
                Accion::Seguir { figura, periodo_s, duracion, .. } => {
                    if !positivo(*periodo_s) {
                        malo("`periodo_s` debe ser mayor que cero");
                    }
                    let d = match duracion {
                        Duracion::Vueltas(v) | Duracion::Tiempo(v) => *v,
                    };
                    if !positivo(d) {
                        malo("la duración debe ser mayor que cero");
                    }
                    if let Figura::Circulo { radio, .. } = figura
                        && !positivo(*radio as f64)
                    {
                        malo("`radio` debe ser mayor que cero");
                    }
                }
                Accion::Empujar { hacia: Hacia::Puesto(n) } if !r.puestos.iter().any(|b| b.nombre == *n) => {
                    malo(&format!("`a_puesto` {n} no es un puesto de la rutina"))
                }
                _ => {}
            }
            if let Some(t) = paso.tolerancia_m
                && !(t.is_finite() && t >= ctx.arrival_threshold - EPS)
            {
                malo(&format!(
                    "tolerancia_m {t} es menor que motion.arrival_threshold ({}): no se puede pedir más precisión que la cámara y motion",
                    ctx.arrival_threshold
                ));
            }
            if let Some(t) = paso.tolerancia_deg
                && !positivo(t)
            {
                malo("`tolerancia_deg` debe ser mayor que cero");
            }
            if let Some(t) = paso.tiempo_max_s
                && !positivo(t)
            {
                malo("`tiempo_max_s` debe ser mayor que cero");
            }
        }
    }

    // 3 y 4. Zona segura y áreas.
    let vuelta = r.repeticiones.mas_de_una();
    for a in &r.puestos {
        let areas = areas_prohibidas(a.arquero, ctx.attack_sign);
        let toca = |q: Vec2| areas.iter().find(|ar| ar.touches_with(q, AREA_CLEARANCE));
        let revisar_destino = |q: Vec2, donde: &str, razones: &mut Vec<String>| -> bool {
            if let Some(m) = fuera_de_zona(q, p.zona_segura_m) {
                razones.push(format!("{donde}: {m}"));
            }
            if let Some(ar) = toca(q) {
                razones.push(format!(
                    "{donde}: {} toca {} (holgura {:.2} m)",
                    punto(q),
                    nombre_area(ar),
                    AREA_CLEARANCE
                ));
                return true;
            }
            false
        };
        let mut previo = a.inicio.pos;
        let mut previo_toca = revisar_destino(previo, &format!("puesto {}, inicio", a.nombre), &mut razones);
        for (i, paso) in a.pasos.iter().enumerate() {
            let donde = format!("puesto {}, paso {}", a.nombre, i + 1);
            if let Accion::Seguir { figura, .. } = &paso.accion {
                let muestras = figura.muestras();
                if let Some(m) = muestras.iter().find_map(|q| fuera_de_zona(*q, p.zona_segura_m)) {
                    razones.push(format!("{donde}: la figura: {m}"));
                }
                if let Some((q, ar)) = muestras.iter().find_map(|q| toca(*q).map(|ar| (*q, ar))) {
                    razones.push(format!(
                        "{donde}: la figura pasa por {} y toca {} (holgura {:.2} m)",
                        punto(q),
                        nombre_area(ar),
                        AREA_CLEARANCE
                    ));
                }
            }
            let Some(d) = paso.accion.destino() else { continue };
            let d_toca = revisar_destino(d, &donde, &mut razones);
            if !previo_toca
                && !d_toca
                && let Some(ar) = areas.iter().find(|ar| tramo_toca(previo, d, ar))
            {
                razones.push(format!(
                    "{donde}: el tramo de {} a {} toca {} (holgura {:.2} m)",
                    punto(previo),
                    punto(d),
                    nombre_area(ar),
                    AREA_CLEARANCE
                ));
            }
            previo = d;
            previo_toca = d_toca;
        }
        if vuelta
            && let Some(primero) = a.destinos().next()
            && !previo_toca
            && toca(primero).is_none()
            && let Some(ar) = areas.iter().find(|ar| tramo_toca(previo, primero, ar))
        {
            razones.push(format!(
                "puesto {}: el tramo de vuelta de {} a {} toca {} (holgura {:.2} m)",
                a.nombre,
                punto(previo),
                punto(primero),
                nombre_area(ar),
                AREA_CLEARANCE
            ));
        }
    }

    // 6. Inicios separados.
    for (i, a) in r.puestos.iter().enumerate() {
        for b in &r.puestos[i + 1..] {
            let d = (a.inicio.pos - b.inicio.pos).length();
            if d < p.distancia_minima_m - EPS {
                razones.push(format!(
                    "{} y {} empiezan a {:.2} m (mínimo {:.2})",
                    a.nombre, b.nombre, d, p.distancia_minima_m
                ));
            }
        }
    }

    // 7. Lo que llega en fases posteriores.
    let mut reservados: Vec<(&'static str, u8)> = Vec::new();
    for paso in r.puestos.iter().flat_map(|a| &a.pasos) {
        let (tipo, fase) = (paso.accion.tipo(), paso.accion.fase());
        if fase > 1 && !reservados.iter().any(|(t, _)| *t == tipo) {
            reservados.push((tipo, fase));
        }
    }
    for (tipo, fase) in reservados {
        razones.push(format!("no disponible: el paso `{tipo}` llega en la fase {fase}"));
    }
    if r.pelota.is_some() {
        razones.push("no disponible: las rutinas con pelota llegan en la fase 3".to_string());
    }
    razones
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rutinas::formato::{Entrada, cargar_texto};
    use std::path::PathBuf;

    fn con_ctx(texto: &str, ctx: &ContextoValidacion) -> Entrada {
        cargar_texto(texto, "prueba", PathBuf::from("prueba.json"), ctx)
    }

    fn razones(texto: &str) -> Vec<String> {
        con_ctx(texto, &ContextoValidacion::por_defecto(1.0)).razones
    }

    /// Rutina de un robot con inicio en (−0.3, −0.25), 1 repetición, con los pasos dados.
    fn un_puesto(pasos: &str) -> String {
        crate::rutinas::prueba::un_puesto(pasos)
    }

    fn tiene(rs: &[String], parte: &str) -> bool {
        rs.iter().any(|r| r.contains(parte))
    }

    #[test]
    fn valid_routine_has_no_reasons() {
        let rs = razones(&un_puesto(r#"{ "paso": "ir", "x": 0.3, "y": -0.25 }, { "paso": "ir", "x": -0.3, "y": -0.25 }"#));
        assert!(rs.is_empty(), "{rs:?}");
    }

    #[test]
    fn point_against_the_wall() {
        let rs = razones(&un_puesto(r#"{ "paso": "ir", "x": 0.0, "y": 0.6 }"#));
        assert!(
            rs.iter().any(|r| r.starts_with("puesto A, paso 1:") && r.contains("de la pared (mínimo 0.10)")),
            "{rs:?}"
        );
    }

    #[test]
    fn point_near_a_corner_triangle() {
        // Dentro del rectángulo seguro (|x| ≤ 0.65, |y| ≤ 0.55) pero a 9 cm de la hipotenusa.
        let rs = razones(&un_puesto(r#"{ "paso": "ir", "x": 0.4, "y": 0.55 }, { "paso": "ir", "x": 0.65, "y": -0.55 }"#));
        assert!(rs.iter().any(|r| r.starts_with("puesto A, paso 2:") && r.contains("triángulo de la esquina")), "{rs:?}");
        assert!(!rs.iter().any(|r| r.starts_with("puesto A, paso 1:")), "{rs:?}");
    }

    #[test]
    fn look_at_point_may_be_outside_the_field() {
        let rs = razones(&un_puesto(r#"{ "paso": "mirar", "x": 0.75, "y": 0.0 }"#));
        assert!(rs.is_empty(), "{rs:?}");
    }

    #[test]
    fn point_in_an_area() {
        let rs = razones(&un_puesto(r#"{ "paso": "ir", "x": 0.5, "y": 0.0 }"#));
        assert!(tiene(&rs, "puesto A, paso 1: (0.50, 0.00) toca el área derecha"), "{rs:?}");
    }

    #[test]
    fn leg_across_an_area() {
        // Los dos puntos fuera de la holgura; el tramo pasa por delante del área izquierda.
        let rs = razones(&un_puesto(r#"{ "paso": "ir", "x": -0.5, "y": -0.5 }, { "paso": "ir", "x": -0.5, "y": 0.5 }"#));
        assert!(tiene(&rs, "puesto A, paso 2: el tramo de (-0.50, -0.50) a (-0.50, 0.50) toca el área izquierda"), "{rs:?}");
        assert!(!tiene(&rs, "paso 1"), "{rs:?}");
    }

    #[test]
    fn return_leg_is_checked_with_repetitions() {
        // Ida por arriba y por abajo del área izquierda; la vuelta del último al primero la cruza.
        let pasos = r#"{ "paso": "ir", "x": -0.5, "y": 0.5 }, { "paso": "ir", "x": -0.2, "y": 0.0 }, { "paso": "ir", "x": -0.5, "y": -0.5 }"#;
        let una = razones(&un_puesto(pasos));
        assert!(!tiene(&una, "tramo de vuelta"), "{una:?}");
        let varias = razones(&un_puesto(pasos).replace("\"repeticiones\": 1", "\"repeticiones\": 3"));
        assert!(tiene(&varias, "puesto A: el tramo de vuelta de (-0.50, -0.50) a (-0.50, 0.50) toca el área izquierda"), "{varias:?}");
    }

    #[test]
    fn keeper_post_may_enter_its_own_area_only() {
        let texto = un_puesto(r#"{ "paso": "ir", "x": -0.65, "y": 0.0 }"#).replace("\"nombre\": \"A\",", "\"nombre\": \"A\", \"arquero\": true,");
        // Atacando hacia +x, el área propia es la izquierda: el arquero puede entrar.
        assert!(con_ctx(&texto, &ContextoValidacion::por_defecto(1.0)).razones.is_empty());
        // Con el lado cambiado, esa es el área rival.
        let rs = con_ctx(&texto, &ContextoValidacion::por_defecto(-1.0)).razones;
        assert!(tiene(&rs, "toca el área izquierda"), "{rs:?}");
    }

    #[test]
    fn tolerance_finer_than_arrival_threshold() {
        let rs = razones(&un_puesto(r#"{ "paso": "ir", "x": 0.3, "y": -0.25, "tolerancia_m": 0.02 }"#));
        assert!(tiene(&rs, "puesto A, paso 1: tolerancia_m 0.02 es menor que motion.arrival_threshold (0.04)"), "{rs:?}");
        let ok = razones(&un_puesto(r#"{ "paso": "ir", "x": 0.3, "y": -0.25, "tolerancia_m": 0.04 }"#));
        assert!(ok.is_empty(), "{ok:?}");
    }

    #[test]
    fn non_positive_values() {
        let rs = razones(&un_puesto(
            r#"{ "paso": "esperar", "tiempo_s": 0 }, { "paso": "girar", "sentido": "horario", "vueltas": -1 },
               { "paso": "ir", "x": 0.3, "y": -0.25, "tiempo_max_s": 0, "tolerancia_deg": -5 }"#,
        ));
        assert!(tiene(&rs, "puesto A, paso 1: `tiempo_s` debe ser mayor que cero"), "{rs:?}");
        assert!(tiene(&rs, "puesto A, paso 2: `vueltas` debe ser mayor que cero"), "{rs:?}");
        assert!(tiene(&rs, "puesto A, paso 3: `tiempo_max_s`"), "{rs:?}");
        assert!(tiene(&rs, "puesto A, paso 3: `tolerancia_deg`"), "{rs:?}");
    }

    #[test]
    fn robots_and_posts() {
        let texto = un_puesto(r#"{ "paso": "ir", "x": 0.3, "y": -0.25 }"#).replace("\"max\": 1", "\"max\": 2");
        let rs = razones(&texto);
        assert!(tiene(&rs, "la rutina tiene 1 puesto y robots.max es 2"), "{rs:?}");
        let texto = un_puesto(r#"{ "paso": "ir", "x": 0.3, "y": -0.25 }"#).replace("\"min\": 1", "\"min\": 0");
        assert!(tiene(&razones(&texto), "1 ≤ min ≤ max ≤ 3"));
        let texto = un_puesto("").replace("\"nombre\": \"A\"", "\"nombre\": \"A,1\"");
        let rs = razones(&texto);
        assert!(tiene(&rs, "solo admite letras y dígitos"), "{rs:?}");
        assert!(tiene(&rs, "no tiene pasos"), "{rs:?}");
    }

    #[test]
    fn posts_too_close_at_the_start() {
        let rs = razones(&crate::rutinas::prueba::dos_puestos((-0.3, 0.0), (-0.15, 0.0)));
        assert!(tiene(&rs, "A y B empiezan a 0.15 m (mínimo 0.20)"), "{rs:?}");
        assert!(razones(&crate::rutinas::prueba::dos_puestos((-0.3, 0.0), (0.0, 0.0))).is_empty());
    }

    #[test]
    fn several_reasons_together() {
        let rs = razones(&un_puesto(r#"{ "paso": "ir", "x": 0.0, "y": 0.62, "tolerancia_m": 0.02 }"#));
        assert!(tiene(&rs, "de la pared"), "{rs:?}");
        assert!(tiene(&rs, "arrival_threshold"), "{rs:?}");
    }

    #[test]
    fn reserved_steps_and_ball_are_not_available() {
        let rs = razones(&un_puesto(
            r#"{ "paso": "seguir", "figura": "circulo", "centro": { "x": 0, "y": 0 }, "radio": 0.25, "periodo_s": 4, "vueltas": 2 },
               { "paso": "sincronizar", "barrera": "medio" },
               { "paso": "sincronizar", "barrera": "fin" },
               { "paso": "empujar", "a_puesto": "B" },
               { "paso": "recibir", "x": 0.2, "y": 0.1 }"#,
        ));
        assert!(tiene(&rs, "no disponible: el paso `seguir` llega en la fase 2"), "{rs:?}");
        assert_eq!(rs.iter().filter(|r| r.contains("`sincronizar`")).count(), 1, "una vez por tipo: {rs:?}");
        assert!(tiene(&rs, "no disponible: el paso `empujar` llega en la fase 3"), "{rs:?}");
        assert!(tiene(&rs, "no disponible: el paso `recibir` llega en la fase 3"), "{rs:?}");
        // Los campos de los reservados también se validan.
        assert!(tiene(&rs, "`a_puesto` B no es un puesto de la rutina"), "{rs:?}");
        let pelota = un_puesto(r#"{ "paso": "ir", "x": 0.3, "y": -0.25 }"#)
            .replace("\"repeticiones\": 1,", "\"repeticiones\": 1, \"pelota\": { \"marca\": { \"x\": 0, \"y\": 0 } },");
        assert!(tiene(&razones(&pelota), "las rutinas con pelota llegan en la fase 3"));
    }

    #[test]
    fn the_included_circuits_are_valid_on_both_sides() {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("config/rutinas");
        for lado in [1.0, -1.0] {
            let entradas = crate::rutinas::formato::cargar_carpeta(&dir, &ContextoValidacion::por_defecto(lado)).unwrap();
            let ids: Vec<&str> = entradas.iter().map(|e| e.id.as_str()).collect();
            assert_eq!(ids, ["cuadrado_giros", "ida_vuelta", "ocho", "triangulo", "zigzag"]);
            for e in &entradas {
                assert!(e.razones.is_empty(), "{} (lado {lado}): {:?}", e.id, e.razones);
                let r = e.valida().unwrap();
                assert_eq!(r.grupo, crate::rutinas::Grupo::Circuito, "{}", e.id);
                assert_eq!((r.robots_min, r.robots_max), (1, 1), "{}", e.id);
            }
        }
    }

    #[test]
    fn follow_figure_points_are_checked() {
        let rs = razones(&un_puesto(
            r#"{ "paso": "seguir", "figura": "circulo", "centro": { "x": 0.4, "y": 0 }, "radio": 0.2, "periodo_s": 4, "vueltas": 1 }"#,
        ));
        assert!(tiene(&rs, "puesto A, paso 1:") && tiene(&rs, "toca el área derecha"), "{rs:?}");
    }
}
