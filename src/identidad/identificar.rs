//! Identificar: recorre las radios activas de a una, gira solo ese robot (lento, ida y
//! vuelta), ve qué patrón giró y propone la identidad. Máquina pura, sin reloj propio: la
//! mueve la GUI con cada foto (`paso`) y el lazo solo garantiza la seguridad del giro
//! (exclusivo, vigencia corta, cortado por la parada y HALT).

use super::{Identidad, RobotFisico, con_mayuscula, lista, nombre_radio};
use crate::radio::base_station::SLOT_COUNT;
use std::collections::BTreeMap;
use std::time::{Duration, Instant};

/// Velocidad del giro (°/s): ±59 mm/s por rueda, por encima de la zona muerta de 20 mm/s.
pub const GIRO_DEG_S: i16 = 90;
/// Reposo antes de cada radio: al final se toma la línea base.
pub const REPOSO: Duration = Duration::from_millis(500);
/// Ida a +ω y vuelta a −ω: ±54° nominales, y el robot termina donde empezó.
pub const IDA: Duration = Duration::from_millis(600);
pub const VUELTA: Duration = Duration::from_millis(600);
/// Asentamiento después de la vuelta (se sigue midiendo).
pub const ASENTAR: Duration = Duration::from_millis(400);
/// Un patrón "giró" con este cambio máximo de orientación o más (°).
pub const UMBRAL_GIRO_DEG: f64 = 25.0;
/// Un patrón quedó "quieto" por debajo de esto (°); en medio, dudoso.
pub const UMBRAL_QUIETO_DEG: f64 = 10.0;
/// Más del doble de lo pedido (2 · 90 °/s · 0.6 s): señal de un gyro o un PI de guiñada mal.
pub const DE_MAS_DEG: f64 = 108.0;

/// Fase de la radio en curso.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaseGiro {
    Reposo,
    Ida,
    Vuelta,
    Asentar,
}

impl FaseGiro {
    fn duracion(self) -> Duration {
        match self {
            FaseGiro::Reposo => REPOSO,
            FaseGiro::Ida => IDA,
            FaseGiro::Vuelta => VUELTA,
            FaseGiro::Asentar => ASENTAR,
        }
    }

    pub fn texto(self) -> &'static str {
        match self {
            FaseGiro::Reposo => "en reposo",
            FaseGiro::Ida => "girando ida",
            FaseGiro::Vuelta => "girando vuelta",
            FaseGiro::Asentar => "asentando",
        }
    }
}

/// El estado que puede cortar (o impedir) la identificación.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Condiciones {
    /// Parada de emergencia.
    pub parada: bool,
    /// HALT del árbitro.
    pub halt: bool,
    /// Corte sin visión.
    pub sin_vision: bool,
    pub coach_activo: bool,
    pub rutina: bool,
    pub manual: bool,
    pub skill_gui: bool,
    /// El usuario apretó Cancelar.
    pub cancelar: bool,
}

/// Por qué no se puede arrancar, en el orden de D10, o `Ok`. `vistos`: patrones propios a
/// la vista con su orientación.
pub fn puede_identificar(c: &Condiciones, identidad: &Identidad, vistos: &[(i32, f64)]) -> Result<(), String> {
    let razon = if c.parada {
        "Suelta la parada de emergencia para identificar."
    } else if c.halt {
        "El árbitro tiene al equipo en HALT."
    } else if c.coach_activo {
        "Pon el coach en pausa (P): con el coach activo, los robots se mueven."
    } else if c.rutina {
        "Hay una rutina en curso: ciérrala para identificar."
    } else if c.manual {
        "Apaga el control manual para identificar."
    } else if c.skill_gui {
        "Quita la skill de la GUI (ninguna) para identificar."
    } else if !identidad.robots.iter().any(|r| r.activo) {
        "No hay robots activos para identificar."
    } else if vistos.is_empty() {
        "La cámara no ve ningún robot propio."
    } else {
        return Ok(());
    };
    Err(razon.to_string())
}

/// Causa de corte mientras corre, o `None`.
fn causa_de_corte(c: &Condiciones) -> Option<&'static str> {
    if c.cancelar {
        Some("cancelada")
    } else if c.parada {
        Some("parada de emergencia")
    } else if c.halt {
        Some("HALT del árbitro")
    } else if c.sin_vision {
        Some("corte sin visión")
    } else if c.coach_activo {
        Some("se activó el coach")
    } else if c.rutina {
        Some("empezó una rutina")
    } else if c.manual {
        Some("se encendió el control manual")
    } else if c.skill_gui {
        Some("se eligió una skill de la GUI")
    } else {
        None
    }
}

/// Lo que da cada radio.
#[derive(Debug, Clone, PartialEq)]
pub enum ResultadoRadio {
    /// Giró exactamente un patrón y ninguno quedó dudoso.
    Uno { patron: i32, grados: f64 },
    /// Giraron dos o más: dos robots con el mismo `MI_ROBOT_ID`.
    Varios(Vec<(i32, f64)>),
    /// No giró nada y nada quedó dudoso.
    Ninguno,
    /// Cualquier otra combinación: los patrones que se movieron 10° o más.
    Dudoso(Vec<(i32, f64)>),
    /// La radio está desactivada: no se giró.
    Omitida,
}

/// Lo que dio la identificación completa.
#[derive(Debug, Clone, PartialEq)]
pub struct Resultado {
    /// Un resultado por radio, en orden (0 a 4).
    pub por_radio: Vec<ResultadoRadio>,
    /// Patrones que dieron `Uno` en dos o más radios, con esas radios.
    pub repetidos: Vec<(i32, Vec<usize>)>,
}

impl Resultado {
    fn nuevo(por_radio: Vec<ResultadoRadio>) -> Self {
        let mut radios_de: BTreeMap<i32, Vec<usize>> = BTreeMap::new();
        for (r, res) in por_radio.iter().enumerate() {
            if let ResultadoRadio::Uno { patron, .. } = res {
                radios_de.entry(*patron).or_default().push(r);
            }
        }
        let repetidos = radios_de.into_iter().filter(|(_, rs)| rs.len() >= 2).collect();
        Self { por_radio, repetidos }
    }

    /// Un resultado sin nada girado (todas las radios `Ninguno`), para los tests de la GUI.
    #[cfg(test)]
    pub(crate) fn nuevo_para_tests() -> Self {
        Self::nuevo(vec![ResultadoRadio::Ninguno; SLOT_COUNT])
    }

    fn repetido(&self, patron: i32) -> bool {
        self.repetidos.iter().any(|(p, _)| *p == patron)
    }

    /// La identidad propuesta a partir de la vigente:
    /// - `Uno(P)` sin repetir → P;
    /// - `Uno` repetido (`MismoPatron`), `Varios` o `Ninguno` → sin patrón;
    /// - `Dudoso` u `Omitida` → el patrón actual, salvo que la propuesta se lo dé a otra radio;
    /// - "activo" no cambia.
    pub fn propuesta(&self, actual: &Identidad) -> Identidad {
        let asignados: Vec<i32> = self
            .por_radio
            .iter()
            .filter_map(|r| match r {
                ResultadoRadio::Uno { patron, .. } if !self.repetido(*patron) => Some(*patron),
                _ => None,
            })
            .collect();
        let robots = std::array::from_fn(|i| {
            let actual_r = actual.robots[i];
            let patron = match self.por_radio.get(i) {
                Some(ResultadoRadio::Uno { patron, .. }) if !self.repetido(*patron) => u8::try_from(*patron).ok(),
                Some(ResultadoRadio::Dudoso(_) | ResultadoRadio::Omitida) | None => {
                    actual_r.patron.filter(|p| !asignados.contains(&i32::from(*p)))
                }
                Some(_) => None,
            };
            RobotFisico { patron, activo: actual_r.activo }
        });
        Identidad { robots }
    }

    /// Una frase por radio y una por cada patrón repetido.
    pub fn frases(&self) -> Vec<String> {
        let mut out: Vec<String> = self
            .por_radio
            .iter()
            .enumerate()
            .map(|(r, res)| frase_radio(r, res))
            .collect();
        for (p, radios) in &self.repetidos {
            out.push(format!(
                "Las radios {} hicieron girar el patrón {p}: dos robots llevan el mismo patrón. Cámbiale el \
                 patrón a uno.",
                lista(radios)
            ));
        }
        out
    }
}

/// "15°".
fn grados(g: f64) -> String {
    format!("{:.0}°", g)
}

fn frase_radio(r: usize, res: &ResultadoRadio) -> String {
    let nombre = con_mayuscula(&nombre_radio(r));
    match res {
        ResultadoRadio::Uno { patron, grados: g } => {
            let mut s = format!("{nombre}: giró el patrón {patron} ({}).", grados(*g));
            if *g > DE_MAS_DEG {
                s.push_str(
                    " Giró más del doble de lo pedido: revisa el signo del gyro (GYRO_Z_SIGN) y el PI de guiñada.",
                );
            }
            s
        }
        ResultadoRadio::Varios(ps) => {
            let ids: Vec<i32> = ps.iter().map(|(p, _)| *p).collect();
            let quienes = if ps.len() == 2 { "Dos robots" } else { "Varios robots" };
            format!(
                "{nombre}: giraron los patrones {}. {quienes} tienen MI_ROBOT_ID {}: hay que reflashear uno.",
                lista(&ids),
                r + 1
            )
        }
        ResultadoRadio::Ninguno => format!(
            "{nombre}: no giró ningún patrón. No hay robot con ese MI_ROBOT_ID, está apagado o la cámara no lo ve."
        ),
        ResultadoRadio::Dudoso(ps) => {
            let consejo = "Separa los robots, revisa la batería y repite.";
            match ps.as_slice() {
                [(p, g)] => format!("{nombre}: el patrón {p} giró {}, poco para estar seguro. {consejo}", grados(*g)),
                _ => {
                    let partes: Vec<String> = ps.iter().map(|(p, g)| format!("{p} ({})", grados(*g))).collect();
                    format!(
                        "{nombre}: se movieron los patrones {}, y no se puede saber cuál es. {consejo}",
                        lista(&partes)
                    )
                }
            }
        }
        ResultadoRadio::Omitida => format!("{nombre}: desactivada, no se identificó."),
    }
}

/// Clasifica una radio con el cambio máximo de cada patrón de la línea base (°).
fn clasificar(maximos: &BTreeMap<i32, f64>) -> ResultadoRadio {
    let giraron: Vec<(i32, f64)> =
        maximos.iter().filter(|(_, g)| **g >= UMBRAL_GIRO_DEG).map(|(p, g)| (*p, *g)).collect();
    let dudosos: Vec<(i32, f64)> = maximos
        .iter()
        .filter(|(_, g)| **g >= UMBRAL_QUIETO_DEG && **g < UMBRAL_GIRO_DEG)
        .map(|(p, g)| (*p, *g))
        .collect();
    match (giraron.as_slice(), dudosos.is_empty()) {
        ([(p, g)], true) => ResultadoRadio::Uno { patron: *p, grados: *g },
        ([], true) => ResultadoRadio::Ninguno,
        (g, _) if g.len() >= 2 => ResultadoRadio::Varios(giraron),
        _ => {
            let mut todos: Vec<(i32, f64)> = giraron.into_iter().chain(dudosos).collect();
            todos.sort_by(|a, b| b.1.total_cmp(&a.1));
            ResultadoRadio::Dudoso(todos)
        }
    }
}

/// Diferencia angular `a − b` normalizada a (−180°, 180°], en valor absoluto (°).
pub fn diferencia_deg(a: f64, b: f64) -> f64 {
    let d = a - b;
    d.sin().atan2(d.cos()).abs().to_degrees()
}

/// Lo que pide la máquina en cada foto.
#[derive(Debug, Clone, PartialEq)]
pub enum Paso {
    /// Girar esta radio (refrescar el giro en el lazo).
    Girar { radio: usize, w_deg_s: i16 },
    /// Nada gira.
    Quieto,
    /// Se cortó: nada gira y se descarta lo medido.
    Cortado(String),
    /// Terminó.
    Fin(Resultado),
}

/// La identificación en curso.
#[derive(Debug, Clone)]
pub struct Identificacion {
    /// Radios activas a recorrer, en orden.
    radios: Vec<usize>,
    /// Índice en `radios`.
    k: usize,
    fase: FaseGiro,
    desde: Instant,
    /// Último θ visto de cada patrón durante el reposo (rad).
    ultimo: BTreeMap<i32, f64>,
    /// Línea base de la radio en curso (rad).
    base: BTreeMap<i32, f64>,
    /// Cambio máximo desde la línea base (°).
    maximos: BTreeMap<i32, f64>,
    por_radio: Vec<ResultadoRadio>,
    /// `Cortado` o `Fin`: se repite en cada `paso` siguiente.
    final_: Option<Paso>,
}

impl Identificacion {
    /// Empieza por la primera radio activa, en reposo.
    pub fn nueva(identidad: &Identidad, ahora: Instant) -> Self {
        let radios: Vec<usize> = (0..SLOT_COUNT).filter(|r| identidad.robots[*r].activo).collect();
        let por_radio = (0..SLOT_COUNT)
            .map(|r| if identidad.robots[r].activo { ResultadoRadio::Ninguno } else { ResultadoRadio::Omitida })
            .collect();
        let mut i = Self {
            radios,
            k: 0,
            fase: FaseGiro::Reposo,
            desde: ahora,
            ultimo: BTreeMap::new(),
            base: BTreeMap::new(),
            maximos: BTreeMap::new(),
            por_radio,
            final_: None,
        };
        if i.radios.is_empty() {
            i.final_ = Some(Paso::Fin(Resultado::nuevo(i.por_radio.clone())));
        }
        i
    }

    /// La radio en curso, su número de orden (desde 1), el total y la fase; `None` al terminar.
    pub fn avance(&self) -> Option<(usize, usize, usize, FaseGiro)> {
        if self.final_.is_some() {
            return None;
        }
        self.radios.get(self.k).map(|r| (*r, self.k + 1, self.radios.len(), self.fase))
    }

    /// Un paso con la foto de `ahora`. Después de `Cortado` o `Fin`, devuelve lo mismo y
    /// nunca más `Girar`.
    pub fn paso(&mut self, ahora: Instant, vistos: &[(i32, f64)], cond: &Condiciones) -> Paso {
        if let Some(f) = &self.final_ {
            return f.clone();
        }
        if let Some(causa) = causa_de_corte(cond) {
            let frase = if causa == "cancelada" {
                "Identificación cancelada. Ningún robot está girando; se descarta lo medido.".to_string()
            } else {
                format!("Identificación cortada: {causa}. Ningún robot está girando; se descarta lo medido.")
            };
            let p = Paso::Cortado(frase);
            self.final_ = Some(p.clone());
            return p;
        }
        self.observar(vistos);
        if ahora.saturating_duration_since(self.desde) >= self.fase.duracion() {
            self.avanzar(ahora);
            if let Some(f) = &self.final_ {
                return f.clone();
            }
        }
        let radio = self.radios[self.k];
        match self.fase {
            FaseGiro::Ida => Paso::Girar { radio, w_deg_s: GIRO_DEG_S },
            FaseGiro::Vuelta => Paso::Girar { radio, w_deg_s: -GIRO_DEG_S },
            FaseGiro::Reposo | FaseGiro::Asentar => Paso::Quieto,
        }
    }

    fn observar(&mut self, vistos: &[(i32, f64)]) {
        for (p, theta) in vistos.iter().filter(|(_, t)| t.is_finite()) {
            match self.fase {
                FaseGiro::Reposo => {
                    self.ultimo.insert(*p, *theta);
                }
                _ => {
                    if let Some(b) = self.base.get(p) {
                        let d = diferencia_deg(*theta, *b);
                        let m = self.maximos.entry(*p).or_insert(0.0);
                        *m = m.max(d);
                    }
                }
            }
        }
    }

    /// Una fase por paso, medida desde la transición: si la GUI se atrasa, la fase dura más,
    /// nunca menos.
    fn avanzar(&mut self, ahora: Instant) {
        self.desde = ahora;
        self.fase = match self.fase {
            FaseGiro::Reposo => {
                self.base = std::mem::take(&mut self.ultimo);
                self.maximos = self.base.keys().map(|p| (*p, 0.0)).collect();
                FaseGiro::Ida
            }
            FaseGiro::Ida => FaseGiro::Vuelta,
            FaseGiro::Vuelta => FaseGiro::Asentar,
            FaseGiro::Asentar => {
                let radio = self.radios[self.k];
                self.por_radio[radio] = clasificar(&self.maximos);
                self.base.clear();
                self.maximos.clear();
                self.k += 1;
                if self.k >= self.radios.len() {
                    self.final_ = Some(Paso::Fin(Resultado::nuevo(self.por_radio.clone())));
                }
                FaseGiro::Reposo
            }
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Una identidad con solo esas radios activas.
    fn activas(radios: &[usize]) -> Identidad {
        let mut i = Identidad::default();
        for (r, robot) in i.robots.iter_mut().enumerate() {
            robot.activo = radios.contains(&r);
        }
        i
    }

    /// Corre la máquina con fotos cada 10 ms. `theta(radio, patrón, s_en_ida_o_vuelta)`:
    /// la orientación de cada patrón; el robot que gira sigue la consigna.
    struct Banco {
        t0: Instant,
        t: Duration,
        /// Patrones a la vista y su θ actual (rad).
        theta: BTreeMap<i32, f64>,
        pasos: Vec<(Duration, Paso)>,
    }

    impl Banco {
        fn new(patrones: &[i32]) -> Self {
            Self {
                t0: Instant::now(),
                t: Duration::ZERO,
                theta: patrones.iter().map(|p| (*p, 0.0)).collect(),
                pasos: Vec::new(),
            }
        }

        /// `gira(radio) -> [(patrón, ganancia)]`: qué patrones giran (y cuánto de la consigna)
        /// cuando se gira esa radio.
        fn correr(
            &mut self,
            id: &mut Identificacion,
            cond: impl Fn(Duration) -> Condiciones,
            gira: impl Fn(usize) -> Vec<(i32, f64)>,
        ) -> Paso {
            let dt = Duration::from_millis(10);
            for _ in 0..3000 {
                let vistos: Vec<(i32, f64)> = self.theta.iter().map(|(p, t)| (*p, *t)).collect();
                let paso = id.paso(self.t0 + self.t, &vistos, &cond(self.t));
                self.pasos.push((self.t, paso.clone()));
                match &paso {
                    Paso::Girar { radio, w_deg_s } => {
                        for (p, g) in gira(*radio) {
                            if let Some(t) = self.theta.get_mut(&p) {
                                *t += (f64::from(*w_deg_s) * g).to_radians() * dt.as_secs_f64();
                            }
                        }
                    }
                    Paso::Fin(_) | Paso::Cortado(_) => return paso,
                    Paso::Quieto => {}
                }
                self.t += dt;
            }
            panic!("no terminó");
        }
    }

    fn fin(p: Paso) -> Resultado {
        match p {
            Paso::Fin(r) => r,
            otro => panic!("esperaba Fin, vino {otro:?}"),
        }
    }

    #[test]
    fn secuencia_de_fases() {
        let ident = activas(&[0]);
        let mut id = Identificacion::nueva(&ident, Instant::now());
        let mut b = Banco::new(&[0]);
        b.correr(&mut id, |_| Condiciones::default(), |_| vec![(0, 1.0)]);
        // Duración de cada tramo (±1 foto) y su acción.
        let mut tramos: Vec<(Paso, Duration, Duration)> = Vec::new();
        for (t, p) in &b.pasos {
            match tramos.last_mut() {
                Some((q, _, fin)) if q == p => *fin = *t,
                _ => tramos.push((p.clone(), *t, *t)),
            }
        }
        let acciones: Vec<&Paso> = tramos.iter().map(|(p, _, _)| p).collect();
        assert_eq!(acciones.len(), 5, "{acciones:?}");
        assert_eq!(*acciones[0], Paso::Quieto);
        assert_eq!(*acciones[1], Paso::Girar { radio: 0, w_deg_s: 90 });
        assert_eq!(*acciones[2], Paso::Girar { radio: 0, w_deg_s: -90 });
        assert_eq!(*acciones[3], Paso::Quieto);
        assert!(matches!(acciones[4], Paso::Fin(_)));
        let dur = |k: usize| tramos[k + 1].1 - tramos[k].1;
        let cerca = |d: Duration, ms: u64| d.as_millis().abs_diff(u128::from(ms)) <= 10;
        assert!(cerca(dur(0), 500), "reposo {:?}", dur(0));
        assert!(cerca(dur(1), 600), "ida {:?}", dur(1));
        assert!(cerca(dur(2), 600), "vuelta {:?}", dur(2));
        assert!(cerca(dur(3), 400), "asentar {:?}", dur(3));
    }

    #[test]
    fn salta_radios_desactivadas() {
        let ident = activas(&[1, 3]);
        let mut id = Identificacion::nueva(&ident, Instant::now());
        let mut b = Banco::new(&[0, 1]);
        let r = fin(b.correr(&mut id, |_| Condiciones::default(), |_| vec![]));
        let giradas: std::collections::BTreeSet<usize> = b
            .pasos
            .iter()
            .filter_map(|(_, p)| match p {
                Paso::Girar { radio, .. } => Some(*radio),
                _ => None,
            })
            .collect();
        assert_eq!(giradas.into_iter().collect::<Vec<_>>(), vec![1, 3]);
        for radio in [0, 2, 4] {
            assert_eq!(r.por_radio[radio], ResultadoRadio::Omitida);
        }
        assert_eq!(frase_radio(4, &r.por_radio[4]), "Radio 4 (MI_ROBOT_ID 5): desactivada, no se identificó.");
    }

    #[test]
    fn un_patron_gira() {
        let ident = activas(&[0]);
        let mut id = Identificacion::nueva(&ident, Instant::now());
        let mut b = Banco::new(&[0, 1, 2]);
        // La radio 0 hace girar al patrón 1 (los patrones están cruzados).
        let r = fin(b.correr(&mut id, |_| Condiciones::default(), |radio| if radio == 0 { vec![(1, 0.9)] } else { vec![] }));
        match r.por_radio[0] {
            ResultadoRadio::Uno { patron: 1, grados } => assert!((40.0..60.0).contains(&grados), "{grados}"),
            ref otro => panic!("{otro:?}"),
        }
        assert_eq!(r.propuesta(&ident).robots[0].patron, Some(1));
    }

    #[test]
    fn ninguno_gira() {
        let ident = activas(&[3]);
        let mut id = Identificacion::nueva(&ident, Instant::now());
        let mut b = Banco::new(&[0, 1, 2]);
        let r = fin(b.correr(&mut id, |_| Condiciones::default(), |_| vec![]));
        assert_eq!(r.por_radio[3], ResultadoRadio::Ninguno);
        assert_eq!(r.propuesta(&ident).robots[3].patron, None);
        assert_eq!(
            frase_radio(3, &r.por_radio[3]),
            "Radio 3 (MI_ROBOT_ID 4): no giró ningún patrón. No hay robot con ese MI_ROBOT_ID, está apagado o la \
             cámara no lo ve."
        );
    }

    #[test]
    fn dos_giran_con_una_radio() {
        let ident = activas(&[1]);
        let mut id = Identificacion::nueva(&ident, Instant::now());
        let mut b = Banco::new(&[0, 1, 2]);
        let r = fin(b.correr(&mut id, |_| Condiciones::default(), |_| vec![(0, 1.0), (2, 1.0)]));
        assert!(matches!(&r.por_radio[1], ResultadoRadio::Varios(ps) if ps.iter().map(|(p, _)| *p).collect::<Vec<_>>() == vec![0, 2]));
        assert_eq!(r.propuesta(&ident).robots[1].patron, None);
        assert!(frase_radio(1, &r.por_radio[1]).contains("Dos robots tienen MI_ROBOT_ID 2"), "{}", frase_radio(1, &r.por_radio[1]));
    }

    #[test]
    fn mismo_patron_en_dos_radios() {
        let ident = activas(&[0, 2]);
        let mut id = Identificacion::nueva(&ident, Instant::now());
        let mut b = Banco::new(&[0, 1, 2]);
        let r = fin(b.correr(&mut id, |_| Condiciones::default(), |_| vec![(1, 1.0)]));
        assert_eq!(r.repetidos, vec![(1, vec![0, 2])]);
        let prop = r.propuesta(&ident);
        assert_eq!(prop.robots[0].patron, None);
        assert_eq!(prop.robots[2].patron, None);
        assert!(r.frases().iter().any(|f| f.starts_with("Las radios 0 y 2 hicieron girar el patrón 1: dos robots llevan el mismo patrón")));
    }

    #[test]
    fn giro_dudoso_mantiene_el_actual() {
        let ident = activas(&[2]);
        let mut id = Identificacion::nueva(&ident, Instant::now());
        let mut b = Banco::new(&[0, 1, 2]);
        // El patrón 0 gira ~15° (ganancia 0.28 de 54°).
        let mut i2 = ident.clone();
        i2.robots[2].patron = Some(0);
        i2.robots[0].patron = None;
        let r = fin(b.correr(&mut id, |_| Condiciones::default(), |_| vec![(0, 0.28)]));
        assert!(matches!(&r.por_radio[2], ResultadoRadio::Dudoso(ps) if ps.len() == 1 && ps[0].0 == 0));
        assert_eq!(r.propuesta(&i2).robots[2].patron, Some(0));
        let f = frase_radio(2, &r.por_radio[2]);
        assert!(f.starts_with("Radio 2 (MI_ROBOT_ID 3): el patrón 0 giró 15°, poco para estar seguro."), "{f}");
        assert!(f.ends_with("Separa los robots, revisa la batería y repite."), "{f}");
    }

    #[test]
    fn giro_de_mas_avisa_el_gyro() {
        let ident = activas(&[1]);
        let mut id = Identificacion::nueva(&ident, Instant::now());
        let mut b = Banco::new(&[0]);
        let r = fin(b.correr(&mut id, |_| Condiciones::default(), |_| vec![(0, 2.8)]));
        assert!(matches!(r.por_radio[1], ResultadoRadio::Uno { patron: 0, grados } if grados > 108.0));
        assert!(frase_radio(1, &r.por_radio[1]).contains("revisa el signo del gyro (GYRO_Z_SIGN)"));
    }

    #[test]
    fn angulo_cruza_180() {
        let base = 170f64.to_radians();
        let visto = (-150f64).to_radians();
        assert!((diferencia_deg(visto, base) - 40.0).abs() < 1e-9);
        assert!((diferencia_deg(base, visto) - 40.0).abs() < 1e-9);
    }

    #[test]
    fn patron_que_aparece_tarde_no_cuenta() {
        let ident = activas(&[0]);
        let t0 = Instant::now();
        let mut id = Identificacion::nueva(&ident, t0);
        let ms = |m: u64| t0 + Duration::from_millis(m);
        // Reposo solo con el patrón 0; el patrón 1 aparece en la ida, ya girado.
        assert_eq!(id.paso(ms(0), &[(0, 0.0)], &Condiciones::default()), Paso::Quieto);
        assert_eq!(id.paso(ms(510), &[(0, 0.0)], &Condiciones::default()), Paso::Girar { radio: 0, w_deg_s: 90 });
        id.paso(ms(600), &[(0, 0.0), (1, 1.0)], &Condiciones::default());
        id.paso(ms(1200), &[(0, 0.0), (1, 2.0)], &Condiciones::default());
        id.paso(ms(1900), &[(0, 0.0), (1, 2.0)], &Condiciones::default());
        let r = fin(id.paso(ms(2400), &[(0, 0.0), (1, 2.0)], &Condiciones::default()));
        assert_eq!(r.por_radio[0], ResultadoRadio::Ninguno);
    }

    #[test]
    fn propuesta_libera_patron_tomado() {
        // La radio 0 da Uno(1); la radio 1 tenía el patrón 1 y da Dudoso: queda sin patrón.
        let r = Resultado::nuevo(vec![
            ResultadoRadio::Uno { patron: 1, grados: 50.0 },
            ResultadoRadio::Dudoso(vec![(3, 12.0)]),
            ResultadoRadio::Omitida,
            ResultadoRadio::Dudoso(vec![(3, 12.0)]),
            ResultadoRadio::Omitida,
        ]);
        let actual = Identidad::default();
        let p = r.propuesta(&actual);
        assert_eq!(p.robots[0].patron, Some(1));
        assert_eq!(p.robots[1].patron, None, "el 1 se lo llevó la radio 0");
        assert_eq!(p.robots[2].patron, Some(2), "omitida conserva el suyo");
        assert_eq!(p.robots[3].patron, Some(3), "dudosa conserva el suyo");
    }

    #[test]
    fn propuesta_no_toca_activo() {
        let mut actual = Identidad::default();
        actual.robots[4].activo = false;
        actual.robots[1].activo = false;
        let r = Resultado::nuevo(vec![
            ResultadoRadio::Uno { patron: 2, grados: 50.0 },
            ResultadoRadio::Omitida,
            ResultadoRadio::Ninguno,
            ResultadoRadio::Ninguno,
            ResultadoRadio::Omitida,
        ]);
        let p = r.propuesta(&actual);
        let activos: Vec<bool> = p.robots.iter().map(|x| x.activo).collect();
        assert_eq!(activos, vec![true, false, true, true, false]);
    }

    fn cortado_por(cond: Condiciones, palabra: &str) {
        let ident = activas(&[0, 1]);
        let mut id = Identificacion::nueva(&ident, Instant::now());
        let mut b = Banco::new(&[0, 1]);
        // Se corta a los 0.8 s, en la ida de la radio 0.
        let p = b.correr(&mut id, |t| if t >= Duration::from_millis(800) { cond } else { Condiciones::default() }, |_| vec![(0, 1.0)]);
        match p {
            Paso::Cortado(f) => assert!(f.contains(palabra), "{f}"),
            otro => panic!("{otro:?}"),
        }
        // Después del corte, nunca más Girar, aunque las condiciones vuelvan a estar bien.
        let t = Instant::now() + Duration::from_secs(5);
        for k in 0..10 {
            let p = id.paso(t + Duration::from_millis(k * 100), &[(0, 0.0)], &Condiciones::default());
            assert!(matches!(p, Paso::Cortado(_)), "{p:?}");
        }
        assert_eq!(id.avance(), None);
    }

    #[test]
    fn cortado_por_parada() {
        cortado_por(Condiciones { parada: true, ..Default::default() }, "parada de emergencia");
    }

    #[test]
    fn cortado_por_coach() {
        cortado_por(Condiciones { coach_activo: true, ..Default::default() }, "se activó el coach");
    }

    #[test]
    fn cortado_por_rutina() {
        cortado_por(Condiciones { rutina: true, ..Default::default() }, "rutina");
    }

    #[test]
    fn cortado_sin_vision() {
        cortado_por(Condiciones { sin_vision: true, ..Default::default() }, "corte sin visión");
    }

    #[test]
    fn cancelar() {
        cortado_por(Condiciones { cancelar: true, ..Default::default() }, "cancelada");
    }

    #[test]
    fn no_arranca_con_coach_activo() {
        let c = Condiciones { coach_activo: true, ..Default::default() };
        assert_eq!(
            puede_identificar(&c, &Identidad::default(), &[(0, 0.0)]),
            Err("Pon el coach en pausa (P): con el coach activo, los robots se mueven.".to_string())
        );
        // La parada va primero.
        let c = Condiciones { coach_activo: true, parada: true, ..Default::default() };
        assert!(puede_identificar(&c, &Identidad::default(), &[(0, 0.0)]).unwrap_err().contains("parada"));
    }

    #[test]
    fn no_arranca_sin_robots_activos() {
        assert_eq!(
            puede_identificar(&Condiciones::default(), &activas(&[]), &[(0, 0.0)]),
            Err("No hay robots activos para identificar.".to_string())
        );
    }

    #[test]
    fn no_arranca_sin_patrones_a_la_vista() {
        assert_eq!(
            puede_identificar(&Condiciones::default(), &Identidad::default(), &[]),
            Err("La cámara no ve ningún robot propio.".to_string())
        );
        assert_eq!(puede_identificar(&Condiciones::default(), &Identidad::default(), &[(2, 0.1)]), Ok(()));
    }

    #[test]
    fn rutina_manual_y_skill_impiden_arrancar() {
        let id = Identidad::default();
        let v = [(0, 0.0)];
        assert!(puede_identificar(&Condiciones { rutina: true, ..Default::default() }, &id, &v).unwrap_err().contains("rutina"));
        assert!(puede_identificar(&Condiciones { manual: true, ..Default::default() }, &id, &v).unwrap_err().contains("control manual"));
        assert!(puede_identificar(&Condiciones { skill_gui: true, ..Default::default() }, &id, &v).unwrap_err().contains("skill de la GUI"));
        assert!(puede_identificar(&Condiciones { halt: true, ..Default::default() }, &id, &v).unwrap_err().contains("HALT"));
    }

    #[test]
    fn frases_de_resultados() {
        let r = Resultado::nuevo(vec![
            ResultadoRadio::Uno { patron: 1, grados: 48.0 },
            ResultadoRadio::Varios(vec![(0, 50.0), (2, 47.0)]),
            ResultadoRadio::Dudoso(vec![(0, 15.0)]),
            ResultadoRadio::Ninguno,
            ResultadoRadio::Omitida,
        ]);
        let f = r.frases();
        assert_eq!(f[0], "Radio 0 (MI_ROBOT_ID 1): giró el patrón 1 (48°).");
        assert_eq!(f[1], "Radio 1 (MI_ROBOT_ID 2): giraron los patrones 0 y 2. Dos robots tienen MI_ROBOT_ID 2: hay que reflashear uno.");
        assert_eq!(f[2], "Radio 2 (MI_ROBOT_ID 3): el patrón 0 giró 15°, poco para estar seguro. Separa los robots, revisa la batería y repite.");
        assert!(f[3].starts_with("Radio 3 (MI_ROBOT_ID 4): no giró ningún patrón."));
        assert_eq!(f[4], "Radio 4 (MI_ROBOT_ID 5): desactivada, no se identificó.");
        let dos = Resultado::nuevo(vec![
            ResultadoRadio::Uno { patron: 1, grados: 48.0 },
            ResultadoRadio::Ninguno,
            ResultadoRadio::Uno { patron: 1, grados: 45.0 },
            ResultadoRadio::Omitida,
            ResultadoRadio::Omitida,
        ]);
        assert_eq!(
            dos.frases().last().unwrap(),
            "Las radios 0 y 2 hicieron girar el patrón 1: dos robots llevan el mismo patrón. Cámbiale el patrón a uno."
        );
    }

    #[test]
    fn clasificacion_con_un_giro_y_un_dudoso_es_dudosa() {
        let m: BTreeMap<i32, f64> = [(0, 50.0), (1, 15.0), (2, 2.0)].into_iter().collect();
        assert!(matches!(clasificar(&m), ResultadoRadio::Dudoso(ps) if ps == vec![(0, 50.0), (1, 15.0)]));
    }
}
