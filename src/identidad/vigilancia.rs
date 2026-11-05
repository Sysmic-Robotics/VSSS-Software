//! Vigilancia de la identidad durante el juego: avisa si un patrón no responde a sus comandos
//! y sí a los de otro, y propone la identidad que lo arregla.
//!
//! En el segundo tiempo del partido del 5-nov-2026 se cambiaron las etiquetas en el
//! entretiempo, se identificó y se volvieron a mover: durante ~7 minutos cada robot recibió
//! los comandos de otro (el arquero, los de un jugador de campo) y nadie lo notó. Con el
//! registro de partido se veía claro: el giro mandado al patrón 2 se correlacionaba con el giro
//! medido del patrón 0 (0.58) y no con el suyo (−0.05).
//!
//! Por cada patrón propio se guardan el giro mandado (ω de su comando, rad/s) y el rumbo que da
//! la visión. Cada `EVALUAR_S` se calcula, en los últimos `VENTANA_S`, la correlación entre el
//! giro mandado al patrón i (unos 150-350 ms antes, el retraso del robot real) y el giro medido
//! del patrón j. Se usa el giro y no el avance porque no depende de hacia dónde mira el robot y
//! los comandos de giro son casi independientes entre robots. Un patrón i está cruzado con j si
//! casi no se correlaciona consigo mismo y sí, claramente, con j, y j tampoco responde a sus
//! propios comandos (si responde, el parecido con i es azar), `SEGUIDAS` evaluaciones seguidas;
//! vuelve a estar bien con otras tantas evaluaciones en que responde a lo suyo. Sin giros
//! mandados (robots quietos, parada) no se dice nada.
//!
//! Solo con la base station (en simulación cada robot es su patrón). Máquina pura: la mueve el
//! lazo con la hora del tick; no toca la identidad, solo propone.

use std::collections::{BTreeMap, VecDeque};

use super::Identidad;

/// Ventana de la correlación (s).
pub const VENTANA_S: f64 = 8.0;
/// Cada cuánto se evalúa (s).
pub const EVALUAR_S: f64 = 0.5;
/// Evaluaciones seguidas para avisar un cruce o darlo por arreglado.
pub const SEGUIDAS: u32 = 3;
/// Retrasos que se prueban entre el comando y el giro medido (s).
const RETRASOS_S: [f64; 3] = [0.15, 0.25, 0.35];
/// Paso de la grilla en que se comparan las señales (s).
const PASO_S: f64 = 1.0 / 30.0;
/// Medio intervalo de la derivada del rumbo (s).
const MEDIO_DT_S: f64 = 2.0 / 60.0;
/// Hueco máximo entre dos vistas del mismo patrón para derivar el rumbo (s).
const HUECO_MAX_S: f64 = 0.15;
/// Mínimo de puntos de grilla con las dos señales.
const PUNTOS_MIN: usize = 90;
/// Desviación mínima del giro mandado para evaluar (rad/s): sin esto no hay qué comparar.
const EXCITACION_CMD: f64 = 0.4;
/// Desviación mínima del giro medido de j para compararlo (rad/s).
const EXCITACION_MED: f64 = 0.1;
/// Umbrales de correlación.
const PROPIA_BIEN: f64 = 0.30;
const PROPIA_MAL: f64 = 0.20;
const OTRA_CLARA: f64 = 0.45;
const MARGEN: f64 = 0.30;

/// Un patrón cruzado: sus comandos los recibe el robot que la cámara ve con otro patrón.
#[derive(Debug, Clone, PartialEq)]
pub struct Cruce {
    /// Patrón cuyos comandos se mandan.
    pub mandado: i32,
    /// Patrón del robot que se mueve con esos comandos.
    pub se_mueve: i32,
    /// Correlación con el que se mueve y con el propio.
    pub correlacion: f64,
    pub propia: f64,
}

/// Lo que la vigilancia tiene para decir en este momento.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct AlertaIdentidad {
    pub cruces: Vec<Cruce>,
    /// La identidad que deja a cada patrón con los comandos que le corresponden, si los cruces
    /// la determinan sin ambigüedad.
    pub propuesta: Option<Identidad>,
}

impl AlertaIdentidad {
    pub fn frases(&self) -> Vec<String> {
        let mut out: Vec<String> = self
            .cruces
            .iter()
            .map(|c| {
                format!(
                    "Los comandos del robot {} los está recibiendo el robot {} (correlación {:.2}; con el \
                     suyo, {:.2}).",
                    c.mandado, c.se_mueve, c.correlacion, c.propia
                )
            })
            .collect();
        out.push(match &self.propuesta {
            Some(_) => "Si recién movieron etiquetas, aplica la propuesta o vuelve a identificar; si no, \
                        puede ser la visión que confunde dos etiquetas (revisa los colores)."
                .to_string(),
            None => "Vuelve a identificar; si no movieron etiquetas, puede ser la visión que confunde dos \
                     etiquetas (revisa los colores)."
                .to_string(),
        });
        out
    }

    /// El aviso corto de un robot propio en su fila.
    pub fn aviso_de(&self, patron: i32) -> Option<String> {
        self.cruces
            .iter()
            .find(|c| c.mandado == patron)
            .map(|c| format!("sus comandos los recibe el robot {}: identidad cruzada", c.se_mueve))
    }
}

/// Historial y estado de la vigilancia.
#[derive(Debug, Default)]
pub struct VigilanciaIdentidad {
    /// Por patrón: (t, ω mandado).
    mandado: BTreeMap<i32, VecDeque<(f64, f64)>>,
    /// Por patrón: (t, rumbo desenrollado).
    rumbo: BTreeMap<i32, VecDeque<(f64, f64)>>,
    ultima_eval: Option<f64>,
    /// Por patrón mandado: evaluaciones seguidas cruzado (con quién) o bien.
    cruzado: BTreeMap<i32, (u32, i32, f64, f64)>,
    bien: BTreeMap<i32, u32>,
    /// Cruces vigentes (avisados).
    vigentes: BTreeMap<i32, Cruce>,
}

impl VigilanciaIdentidad {
    pub fn new() -> Self {
        Self::default()
    }

    /// Olvida todo (por ejemplo, con una identidad nueva).
    pub fn reiniciar(&mut self) {
        *self = Self::default();
    }

    /// Un tick: el giro mandado a cada patrón propio y el rumbo de los que ve la cámara.
    pub fn registrar(&mut self, t: f64, mandados: &[(i32, f64)], vistos: &[(i32, f64)]) {
        for &(p, w) in mandados {
            if w.is_finite() {
                self.mandado.entry(p).or_default().push_back((t, w));
            }
        }
        for &(p, th) in vistos {
            if !th.is_finite() {
                continue;
            }
            let cola = self.rumbo.entry(p).or_default();
            let th = match cola.back() {
                Some(&(_, prev)) => prev + angulo(th - prev),
                None => th,
            };
            cola.push_back((t, th));
        }
        let desde = t - VENTANA_S - RETRASOS_S[RETRASOS_S.len() - 1] - 1.0;
        for cola in self.mandado.values_mut().chain(self.rumbo.values_mut()) {
            while cola.front().is_some_and(|&(ti, _)| ti < desde) {
                cola.pop_front();
            }
        }
    }

    /// Si toca, evalúa con lo registrado hasta `t`. Devuelve la alerta vigente (`None` si no
    /// hay cruces) solo cuando cambió; `Some(None)` = se arregló.
    pub fn evaluar(&mut self, t: f64, actual: &Identidad) -> Option<Option<AlertaIdentidad>> {
        if self.ultima_eval.is_some_and(|u| t - u < EVALUAR_S) {
            return None;
        }
        self.ultima_eval = Some(t);
        let antes: Vec<i32> = self.vigentes.keys().copied().collect();
        let patrones: Vec<i32> = self.mandado.keys().copied().collect();
        let filas: BTreeMap<i32, BTreeMap<i32, f64>> =
            patrones.iter().filter_map(|&i| self.fila(i, t).map(|f| (i, f))).collect();
        // Cuánto responde j a sus propios comandos (si se le mandó girar).
        let propia_de = |j: i32| filas.get(&j).and_then(|f| f.get(&j)).copied();
        for (&i, fila) in &filas {
            let propia = fila.get(&i).copied().unwrap_or(0.0);
            let otra = fila.iter().filter(|(j, _)| **j != i).max_by(|a, b| a.1.total_cmp(b.1)).map(|(j, c)| (*j, *c));
            // Exclusividad: si j responde a lo suyo, que también se parezca a lo de i es azar.
            let cruzado = otra.filter(|&(j, c)| {
                propia < PROPIA_MAL
                    && c >= OTRA_CLARA
                    && c - propia >= MARGEN
                    && propia_de(j).is_none_or(|pj| c - pj >= MARGEN)
            });
            if let Some((j, c)) = cruzado {
                self.bien.remove(&i);
                let e = self.cruzado.entry(i).or_insert((0, j, c, propia));
                *e = if e.1 == j { (e.0 + 1, j, c, propia) } else { (1, j, c, propia) };
                if e.0 >= SEGUIDAS {
                    self.vigentes.insert(i, Cruce { mandado: i, se_mueve: j, correlacion: c, propia });
                }
            } else if propia >= PROPIA_BIEN || otra.is_none_or(|(_, c)| propia >= c && propia >= PROPIA_MAL) {
                self.cruzado.remove(&i);
                let b = self.bien.entry(i).or_insert(0);
                *b += 1;
                if *b >= SEGUIDAS {
                    self.vigentes.remove(&i);
                }
            }
        }
        let ahora: Vec<i32> = self.vigentes.keys().copied().collect();
        if ahora == antes {
            return None;
        }
        if ahora.is_empty() {
            return Some(None);
        }
        Some(Some(self.alerta(actual)))
    }

    /// La alerta vigente, con la propuesta para `actual`.
    pub fn alerta(&self, actual: &Identidad) -> AlertaIdentidad {
        let cruces: Vec<Cruce> = self.vigentes.values().cloned().collect();
        let propuesta = proponer(actual, &cruces);
        AlertaIdentidad { cruces, propuesta }
    }

    pub fn hay_cruces(&self) -> bool {
        !self.vigentes.is_empty()
    }

    /// Correlación del giro mandado a `i` con el giro medido de cada patrón visto (la mejor
    /// entre los retrasos), o `None` si no hubo giros mandados que comparar.
    fn fila(&self, i: i32, t: f64) -> Option<BTreeMap<i32, f64>> {
        let cmd = self.mandado.get(&i)?;
        let n = (VENTANA_S / PASO_S) as usize;
        let grilla: Vec<f64> = (0..n).map(|k| t - VENTANA_S + k as f64 * PASO_S).collect();
        let mut fila = BTreeMap::new();
        let mut hubo_excitacion = false;
        for (&j, rumbo) in &self.rumbo {
            let medido: Vec<Option<f64>> = grilla.iter().map(|&tau| derivada(rumbo, tau)).collect();
            let mut mejor: Option<f64> = None;
            for &l in &RETRASOS_S {
                let mut xs = Vec::new();
                let mut ys = Vec::new();
                for (k, &tau) in grilla.iter().enumerate() {
                    if let (Some(x), Some(y)) = (retenido(cmd, tau - l), medido[k]) {
                        xs.push(x);
                        ys.push(y);
                    }
                }
                if xs.len() < PUNTOS_MIN || desviacion(&xs) < EXCITACION_CMD {
                    continue;
                }
                hubo_excitacion = true;
                if desviacion(&ys) < EXCITACION_MED {
                    mejor = Some(mejor.map_or(0.0, |m: f64| m.max(0.0)));
                    continue;
                }
                let c = correlacion(&xs, &ys);
                mejor = Some(mejor.map_or(c, |m| m.max(c)));
            }
            if let Some(c) = mejor {
                fila.insert(j, c);
            }
        }
        hubo_excitacion.then_some(fila)
    }
}

/// La identidad con cada cruce corregido: si los comandos de `i` (radio r) mueven al robot
/// de `j`, la radio r lleva el patrón j. `None` si queda un patrón en dos radios.
fn proponer(actual: &Identidad, cruces: &[Cruce]) -> Option<Identidad> {
    let mut nueva = actual.clone();
    for c in cruces {
        let r = actual.radio_de(c.mandado).ok()?;
        let j = u8::try_from(c.se_mueve).ok()?;
        nueva.robots[r].patron = Some(j);
    }
    let mut usados = std::collections::BTreeSet::new();
    for r in nueva.robots.iter().filter(|r| r.activo) {
        if let Some(p) = r.patron
            && !usados.insert(p)
        {
            return None;
        }
    }
    (nueva != *actual).then_some(nueva)
}

fn angulo(a: f64) -> f64 {
    (a + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

/// El último valor mandado en o antes de `t` (retención), si hay uno de hace poco.
fn retenido(cola: &VecDeque<(f64, f64)>, t: f64) -> Option<f64> {
    let k = cola.partition_point(|&(ti, _)| ti <= t);
    (k > 0).then(|| cola[k - 1]).filter(|&(ti, _)| t - ti < 0.2).map(|(_, w)| w)
}

/// Giro medido en `t`: rumbo interpolado en t ± `MEDIO_DT_S`.
fn derivada(cola: &VecDeque<(f64, f64)>, t: f64) -> Option<f64> {
    let a = interpolar(cola, t - MEDIO_DT_S)?;
    let b = interpolar(cola, t + MEDIO_DT_S)?;
    Some((b - a) / (2.0 * MEDIO_DT_S))
}

fn interpolar(cola: &VecDeque<(f64, f64)>, t: f64) -> Option<f64> {
    let k = cola.partition_point(|&(ti, _)| ti <= t);
    if k == 0 || k >= cola.len() {
        return None;
    }
    let (t0, v0) = cola[k - 1];
    let (t1, v1) = cola[k];
    if t1 - t0 > HUECO_MAX_S || t1 <= t0 {
        return None;
    }
    Some(v0 + (v1 - v0) * (t - t0) / (t1 - t0))
}

fn desviacion(v: &[f64]) -> f64 {
    let n = v.len() as f64;
    let m = v.iter().sum::<f64>() / n;
    (v.iter().map(|x| (x - m).powi(2)).sum::<f64>() / n).sqrt()
}

fn correlacion(x: &[f64], y: &[f64]) -> f64 {
    let n = x.len() as f64;
    let (mx, my) = (x.iter().sum::<f64>() / n, y.iter().sum::<f64>() / n);
    let (mut sxy, mut sxx, mut syy) = (0.0, 0.0, 0.0);
    for (a, b) in x.iter().zip(y) {
        sxy += (a - mx) * (b - my);
        sxx += (a - mx).powi(2);
        syy += (b - my).powi(2);
    }
    if sxx <= 0.0 || syy <= 0.0 { 0.0 } else { sxy / (sxx * syy).sqrt() }
}

/// Un giro mandado de prueba distinto por patrón (para los tests del lazo y de la GUI).
#[cfg(test)]
pub(crate) fn giro_de_prueba(p: i32, t: f64) -> f64 {
    let f = [0.37, 0.53, 0.71, 0.29][p.rem_euclid(4) as usize];
    1.5 * (std::f64::consts::TAU * f * t + p as f64).sin() + 0.8 * (std::f64::consts::TAU * 1.3 * f * t).cos()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::identidad::RobotFisico;

    /// Identidad radio k ↔ patrón k para los patrones dados (las demás radios sin patrón).
    fn identidad(patrones: &[(usize, u8)]) -> Identidad {
        let mut i = Identidad { robots: [RobotFisico { patron: None, activo: true }; 5] };
        for &(r, p) in patrones {
            i.robots[r].patron = Some(p);
        }
        i
    }

    /// Giro mandado distinto por patrón (sumas de senos de frecuencias distintas, como los giros
    /// del coach, que no van juntos).
    fn giro_mandado(p: i32, t: f64) -> f64 {
        giro_de_prueba(p, t)
    }

    /// Simula `segundos` de juego a 60 Hz: el robot que la cámara ve con el patrón `fisico[p]`
    /// gira con el comando de `p`, 250 ms después y con ruido. Evalúa cada tick.
    fn correr(
        v: &mut VigilanciaIdentidad,
        actual: &Identidad,
        fisico: &BTreeMap<i32, i32>,
        desde: f64,
        segundos: f64,
    ) -> Vec<(f64, Option<AlertaIdentidad>)> {
        let mut cambios = Vec::new();
        let mut rumbo: BTreeMap<i32, f64> = fisico.values().map(|&j| (j, 0.0)).collect();
        let mut rng = 12345u64;
        let mut ruido = || {
            rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((rng >> 33) as f64 / (1u64 << 31) as f64 - 0.5) * 0.02
        };
        let n = (segundos * 60.0) as usize;
        for k in 0..n {
            let t = desde + k as f64 / 60.0;
            let mandados: Vec<(i32, f64)> = fisico.keys().map(|&p| (p, giro_mandado(p, t))).collect();
            for (&p, &j) in fisico {
                let w = giro_mandado(p, t - 0.25);
                *rumbo.get_mut(&j).unwrap() += w / 60.0;
            }
            let vistos: Vec<(i32, f64)> = rumbo.iter().map(|(&j, &th)| (j, angulo(th + ruido()))).collect();
            v.registrar(t, &mandados, &vistos);
            if let Some(c) = v.evaluar(t, actual) {
                cambios.push((t, c));
            }
        }
        cambios
    }

    #[test]
    fn con_la_identidad_bien_no_avisa() {
        let actual = identidad(&[(0, 0), (1, 1), (2, 2)]);
        let fisico: BTreeMap<i32, i32> = [(0, 0), (1, 1), (2, 2)].into();
        let mut v = VigilanciaIdentidad::new();
        assert!(correr(&mut v, &actual, &fisico, 0.0, 30.0).is_empty());
        assert!(!v.hay_cruces());
    }

    #[test]
    fn el_ciclo_del_segundo_tiempo_se_avisa_y_se_propone_el_arreglo() {
        // Lo del 5-nov: los comandos del 2 los recibe el robot del 0, los del 0 el del 1 y los
        // del 1 el del 2 (la identidad cargada dice radio k ↔ patrón k).
        let actual = identidad(&[(0, 0), (1, 1), (2, 2)]);
        let fisico: BTreeMap<i32, i32> = [(2, 0), (0, 1), (1, 2)].into();
        let mut v = VigilanciaIdentidad::new();
        let cambios = correr(&mut v, &actual, &fisico, 0.0, 20.0);
        let (t, alerta) = cambios.first().expect("debería avisar");
        assert!(*t < 12.0, "avisó tarde: {t:.1} s");
        let a = alerta.as_ref().expect("alerta");
        let mut pares: Vec<(i32, i32)> = a.cruces.iter().map(|c| (c.mandado, c.se_mueve)).collect();
        pares.sort();
        assert_eq!(pares, vec![(0, 1), (1, 2), (2, 0)], "{a:?}");
        // Propuesta: la radio del 0 lleva el 1, la del 1 el 2 y la del 2 el 0.
        let p = a.propuesta.as_ref().expect("propuesta");
        assert_eq!([p.robots[0].patron, p.robots[1].patron, p.robots[2].patron], [Some(1), Some(2), Some(0)]);
        assert!(a.frases().iter().any(|f| f.contains("robot 2 los está recibiendo el robot 0")), "{:?}", a.frases());
        assert_eq!(a.aviso_de(0).as_deref(), Some("sus comandos los recibe el robot 1: identidad cruzada"));

        // Con la propuesta aplicada (el lazo reinicia la vigilancia), deja de avisar.
        let arreglada = p.clone();
        let fisico_ok: BTreeMap<i32, i32> = [(0, 0), (1, 1), (2, 2)].into();
        let mut v2 = VigilanciaIdentidad::new();
        assert!(correr(&mut v2, &arreglada, &fisico_ok, 20.0, 20.0).is_empty());
    }

    #[test]
    fn dos_cruzados_y_uno_bien() {
        let actual = identidad(&[(0, 0), (1, 1), (2, 2)]);
        let fisico: BTreeMap<i32, i32> = [(0, 1), (1, 0), (2, 2)].into();
        let mut v = VigilanciaIdentidad::new();
        let cambios = correr(&mut v, &actual, &fisico, 0.0, 20.0);
        let a = cambios.iter().rev().find_map(|(_, a)| a.clone()).expect("alerta");
        let mut pares: Vec<(i32, i32)> = a.cruces.iter().map(|c| (c.mandado, c.se_mueve)).collect();
        pares.sort();
        assert_eq!(pares, vec![(0, 1), (1, 0)]);
        let p = a.propuesta.expect("propuesta");
        assert_eq!([p.robots[0].patron, p.robots[1].patron, p.robots[2].patron], [Some(1), Some(0), Some(2)]);
    }

    #[test]
    fn se_arregla_solo_si_vuelven_a_responder() {
        let actual = identidad(&[(0, 0), (1, 1)]);
        let mut v = VigilanciaIdentidad::new();
        let cruzado: BTreeMap<i32, i32> = [(0, 1), (1, 0)].into();
        let c1 = correr(&mut v, &actual, &cruzado, 0.0, 20.0);
        assert!(c1.iter().any(|(_, a)| a.is_some()));
        // Alguien cambia las etiquetas de vuelta: después de unos segundos, `Some(None)`.
        let bien: BTreeMap<i32, i32> = [(0, 0), (1, 1)].into();
        let c2 = correr(&mut v, &actual, &bien, 20.0, 20.0);
        assert!(matches!(c2.last(), Some((_, None))), "{c2:?}");
        assert!(!v.hay_cruces());
    }

    #[test]
    fn sin_giros_mandados_no_dice_nada() {
        let actual = identidad(&[(0, 0), (1, 1)]);
        let mut v = VigilanciaIdentidad::new();
        for k in 0..1200 {
            let t = k as f64 / 60.0;
            // Comandos de giro en cero y robots quietos, pero cruzados: no hay cómo saberlo.
            v.registrar(t, &[(0, 0.0), (1, 0.0)], &[(0, 0.3), (1, -1.0)]);
            assert_eq!(v.evaluar(t, &actual), None);
        }
    }

    #[test]
    fn un_robot_trabado_no_es_un_cruce() {
        // El 1 está trabado contra la pared: no gira aunque se le mande. El 0 responde bien.
        let actual = identidad(&[(0, 0), (1, 1)]);
        let mut v = VigilanciaIdentidad::new();
        let mut th0 = 0.0;
        for k in 0..1800 {
            let t = k as f64 / 60.0;
            th0 += giro_mandado(0, t - 0.25) / 60.0;
            v.registrar(t, &[(0, giro_mandado(0, t)), (1, giro_mandado(1, t))], &[(0, angulo(th0)), (1, 0.5)]);
            assert_eq!(v.evaluar(t, &actual), None, "t = {t:.1}");
        }
    }

    #[test]
    fn la_propuesta_ambigua_no_se_propone() {
        let actual = identidad(&[(0, 0), (1, 1), (2, 2)]);
        // Dos patrones cuyos comandos mueven al mismo robot: no hay una identidad que lo explique.
        let cruces = vec![
            Cruce { mandado: 0, se_mueve: 2, correlacion: 0.6, propia: 0.0 },
            Cruce { mandado: 1, se_mueve: 2, correlacion: 0.6, propia: 0.0 },
        ];
        assert_eq!(proponer(&actual, &cruces), None);
    }
}

/// Pasa un registro de partido (`VSSL_MATCH_LOG`) por la vigilancia, como si fuera en vivo, e
/// imprime cuándo habría avisado. `VSSL_CSV=ruta cargo test --lib vigilancia_sobre_registro --
/// --ignored --nocapture`. La identidad es la de `VSSL_IDENTIDAD_REGISTRO` (por ejemplo
/// "0,1,2": la radio k lleva ese patrón) o radio k ↔ patrón k.
#[cfg(test)]
mod registro {
    use super::*;
    use crate::identidad::RobotFisico;

    #[test]
    #[ignore = "necesita VSSL_CSV con un registro de partido"]
    fn vigilancia_sobre_registro() {
        let ruta = std::env::var("VSSL_CSV").expect("VSSL_CSV");
        let mut actual = Identidad { robots: [RobotFisico { patron: None, activo: true }; 5] };
        let mapa = std::env::var("VSSL_IDENTIDAD_REGISTRO").unwrap_or_else(|_| "0,1,2".to_string());
        for (r, p) in mapa.split(',').enumerate() {
            actual.robots[r].patron = p.trim().parse().ok();
        }
        let texto = std::fs::read_to_string(&ruta).expect("leer");
        let mut lineas = texto.lines();
        let cab: Vec<&str> = lineas.next().unwrap().split(',').collect();
        let col = |n: &str| cab.iter().position(|c| *c == n).unwrap();
        let (ct, cr, co, cth, cw) = (col("t_ms"), col("robot"), col("own"), col("pose_theta"), col("cmd_omega"));
        let mut v = VigilanciaIdentidad::new();
        let (mut t_actual, mut mandados, mut vistos) = (f64::NAN, Vec::new(), Vec::new());
        let mut avisos = 0;
        for l in lineas {
            let c: Vec<&str> = l.split(',').collect();
            if c.len() <= cw || c[co] != "1" {
                continue;
            }
            let t = c[ct].parse::<f64>().unwrap_or(f64::NAN) / 1000.0;
            if t != t_actual && t_actual.is_finite() {
                v.registrar(t_actual, &mandados, &vistos);
                if let Some(a) = v.evaluar(t_actual, &actual) {
                    avisos += 1;
                    match a {
                        Some(a) => println!("{t_actual:7.1} s: {}", a.frases().join(" ")),
                        None => println!("{t_actual:7.1} s: la identidad volvió a estar bien"),
                    }
                }
                mandados.clear();
                vistos.clear();
            }
            t_actual = t;
            let id: i32 = c[cr].parse().unwrap_or(-1);
            if let Ok(w) = c[cw].parse::<f64>() {
                mandados.push((id, w));
            }
            if let Ok(th) = c[cth].parse::<f64>() {
                vistos.push((id, th));
            }
        }
        println!("{ruta}: {avisos} cambios de aviso");
    }
}
