//! Identidad de los robots reales: qué patrón de ssl-vision lleva cada robot físico.
//!
//! El robot real tiene dos números:
//! - el **patrón** que ve la cámara (id de visión): ssl-vision numera los patrones por su
//!   celda en `VsssPatern.png`, 4 por color de equipo (0–3). Con él aparece en `World`,
//!   en el coach y en la GUI;
//! - la **posición de radio** que escucha el firmware (`MI_ROBOT_ID − 1`, 5 posiciones en
//!   el frame de la base).
//!
//! La identidad dice, por cada posición de radio, qué patrón lleva el robot y si está
//! activo. Un patrón llega a un robot solo si una sola radio lo tiene y está activa
//! (`radio_de`); si no, ese patrón queda "sin comandos": el frame no le manda nada, el
//! coach no lo cuenta y la asignación automática de las rutinas no lo elige.
//!
//! Solo se aplica con la base station: en simulación cada robot es su patrón.
//! Vive en `config/identidad_robots.json` (o en la ruta de `VSSL_IDENTIDAD`), que no se
//! versiona; la GUI la edita en vivo (pestaña Robots) y la guarda en cada cambio.

pub mod identificar;

use crate::radio::base_station::SLOT_COUNT;
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

/// Patrones de ssl-vision por color de equipo.
pub const PATRONES: u8 = 4;

/// Ruta por defecto del archivo, relativa al directorio de trabajo.
pub const DEFAULT_PATH: &str = "config/identidad_robots.json";

/// Variable de entorno con otra ruta (por ejemplo, un archivo por engine).
pub const ENV_VAR: &str = "VSSL_IDENTIDAD";

/// Un robot físico (una posición de radio).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RobotFisico {
    /// Patrón de ssl-vision que lleva, en `0..PATRONES`, o ninguno.
    pub patron: Option<u8>,
    pub activo: bool,
}

/// Identidad de los 5 robots físicos; el índice es la posición de radio.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Identidad {
    pub robots: [RobotFisico; SLOT_COUNT],
}

impl Default for Identidad {
    /// Radio N ↔ patrón N para N en 0..3, la radio 4 sin patrón, todas activas: con
    /// cualquier patrón que da la cámara, el mismo frame que antes sin mapa.
    fn default() -> Self {
        Self {
            robots: std::array::from_fn(|i| RobotFisico {
                patron: (i < PATRONES as usize).then_some(i as u8),
                activo: true,
            }),
        }
    }
}

/// Por qué los comandos de un patrón no llegan a ningún robot.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SinRadio {
    /// Ninguna radio tiene el patrón.
    SinRobot,
    /// Dos o más radios lo tienen (activas o no), en orden.
    Repetido(Vec<usize>),
    /// Una sola radio lo tiene, y está desactivada.
    Desactivado(usize),
}

impl SinRadio {
    /// Por qué no llega, en frase: "ninguna radio lo tiene", "las radios 0 y 2 lo tienen",
    /// "la radio 1 está desactivada".
    pub fn frase(&self) -> String {
        match self {
            SinRadio::SinRobot => "ninguna radio lo tiene".to_string(),
            SinRadio::Repetido(rs) => format!("las radios {} lo tienen", lista(rs)),
            SinRadio::Desactivado(r) => format!("la radio {r} está desactivada"),
        }
    }
}

/// Giro de identificación: va directo a una posición del frame, sin pasar por la
/// identidad (que puede estar mal). `w_deg_s` en grados/s, + = antihorario.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GiroIdentificacion {
    pub radio: usize,
    pub w_deg_s: i16,
}

/// Órdenes de la GUI al lazo (pestaña Robots).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum OrdenIdentidad {
    /// Usar esta identidad desde el tick en que llega.
    Usar(Identidad),
    /// Pedir (o refrescar) un giro de identificación; caduca sin refresco.
    Giro { radio: usize, w_deg_s: i16 },
    /// Cortar el giro al instante.
    FinGiro,
}

/// "radio P (MI_ROBOT_ID P+1)".
pub fn nombre_radio(p: usize) -> String {
    format!("radio {p} (MI_ROBOT_ID {})", p + 1)
}

/// Lista en castellano: "0", "0 y 2", "0, 1 y 2".
pub fn lista<T: std::fmt::Display>(items: &[T]) -> String {
    match items {
        [] => String::new(),
        [a] => a.to_string(),
        [init @ .., ultimo] => {
            let cabeza: Vec<String> = init.iter().map(|x| x.to_string()).collect();
            format!("{} y {ultimo}", cabeza.join(", "))
        }
    }
}

/// La primera letra en mayúscula.
pub fn con_mayuscula(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        Some(p) => p.to_uppercase().chain(c).collect(),
        None => String::new(),
    }
}

impl Identidad {
    /// Las posiciones de radio que tienen el patrón `patron`.
    fn radios_con(&self, patron: i32) -> Vec<usize> {
        self.robots
            .iter()
            .enumerate()
            .filter(|(_, r)| r.patron.is_some_and(|p| i32::from(p) == patron))
            .map(|(i, _)| i)
            .collect()
    }

    /// La posición de radio a la que llegan los comandos del patrón, o por qué no llegan.
    /// Un patrón negativo o fuera de `0..PATRONES` no tiene robot.
    pub fn radio_de(&self, patron: i32) -> Result<usize, SinRadio> {
        match self.radios_con(patron).as_slice() {
            [] => Err(SinRadio::SinRobot),
            [p] if self.robots[*p].activo => Ok(*p),
            [p] => Err(SinRadio::Desactivado(*p)),
            varias => Err(SinRadio::Repetido(varias.to_vec())),
        }
    }

    /// Patrones de `0..PATRONES` cuyos comandos no llegan a ningún robot (desactivado, sin
    /// robot o repetido). Con la identidad por defecto, vacía.
    pub fn sin_comandos(&self) -> Vec<i32> {
        (0..i32::from(PATRONES)).filter(|p| self.radio_de(*p).is_err()).collect()
    }

    /// Una línea por radio, sin flechas (la fuente de la GUI no las tiene): la usan el lazo
    /// al arrancar y en cada cambio, y la sección Radio del Inspector.
    pub fn describir(&self) -> Vec<String> {
        self.robots
            .iter()
            .enumerate()
            .map(|(i, r)| {
                let patron = match r.patron {
                    Some(p) => format!("patrón {p}"),
                    None => "sin patrón".to_string(),
                };
                let estado = if r.activo { "" } else { ", desactivado" };
                format!("{}: {patron}{estado}", nombre_radio(i))
            })
            .collect()
    }

    /// Avisos de la identidad (en alerta en la GUI): patrón repetido, y de lo que ve la
    /// cámara, un patrón que el coach no manda (≥ `num_robots`) o un patrón sin robot.
    /// `vistos`: patrones propios a la vista. El patrón que el coach no manda se avisa solo
    /// si la cámara lo ve: con la identidad por defecto la radio 3 lleva el 3, y sin un
    /// robot que lo use no hay nada que avisar.
    pub fn avisos(&self, vistos: &[i32], num_robots: usize) -> Vec<String> {
        let mut out = Vec::new();
        for p in 0..i32::from(PATRONES) {
            let radios = self.radios_con(p);
            if radios.len() >= 2 {
                let cuantas = if radios.len() == 2 { "las dos quedan quietas" } else { "todas quedan quietas" };
                out.push(format!(
                    "Las radios {} tienen el patrón {p}: {cuantas}, y el coach y las rutinas no cuentan \
                     al robot {p}, hasta que corrijas una.",
                    lista(&radios)
                ));
            }
        }
        let mut vistos = vistos.to_vec();
        vistos.sort_unstable();
        vistos.dedup();
        let tope = num_robots.saturating_sub(1);
        for p in vistos {
            if usize::try_from(p).is_ok_and(|p| p >= num_robots) {
                let quien = match self.radios_con(p).as_slice() {
                    [] => format!("La cámara ve el patrón {p}"),
                    [r] => format!("La radio {r} tiene el patrón {p}"),
                    varias => format!("Las radios {} tienen el patrón {p}", lista(varias)),
                };
                out.push(format!(
                    "{quien}: el coach solo manda los patrones 0 a {tope}, así que ese robot no juega."
                ));
            } else if self.radio_de(p) == Err(SinRadio::SinRobot) {
                out.push(format!(
                    "La cámara ve el patrón {p} y ningún robot lo tiene: sus comandos no llegan a nadie, \
                     y el coach y las rutinas no lo cuentan."
                ));
            }
        }
        out
    }

    /// Lee el JSON del archivo y lo valida (radio en 0..4 sin repetir, patrón en 0..3). Una
    /// radio que no aparece queda sin patrón y activa; un patrón repetido es válido.
    pub fn desde_json(text: &str) -> Result<Self, String> {
        let archivo: Archivo = serde_json::from_str(text).map_err(|e| e.to_string())?;
        let mut robots = [RobotFisico { patron: None, activo: true }; SLOT_COUNT];
        let mut vistas = [false; SLOT_COUNT];
        for e in &archivo.robots {
            let radio = usize::try_from(e.radio)
                .ok()
                .filter(|r| *r < SLOT_COUNT)
                .ok_or_else(|| format!("la radio {} está fuera de rango (0..{})", e.radio, SLOT_COUNT - 1))?;
            if std::mem::replace(&mut vistas[radio], true) {
                return Err(format!("la radio {radio} aparece dos veces"));
            }
            let patron = match e.patron {
                None => None,
                Some(p) => Some(
                    u8::try_from(p).ok().filter(|p| *p < PATRONES).ok_or_else(|| {
                        format!(
                            "la radio {radio} tiene el patrón {p}, fuera de rango (0..{})",
                            PATRONES - 1
                        )
                    })?,
                ),
            };
            robots[radio] = RobotFisico { patron, activo: e.activo };
        }
        Ok(Self { robots })
    }

    /// El JSON que se guarda: siempre las 5 radios.
    pub fn a_json(&self) -> String {
        let archivo = Archivo {
            robots: self
                .robots
                .iter()
                .enumerate()
                .map(|(i, r)| EntradaArchivo {
                    radio: i as i64,
                    patron: r.patron.map(i64::from),
                    activo: r.activo,
                })
                .collect(),
        };
        let mut s = serde_json::to_string_pretty(&archivo).unwrap_or_default();
        s.push('\n');
        s
    }

    /// Guarda en `ruta` de forma atómica (temporal y `rename`), creando la carpeta si
    /// falta.
    pub fn guardar(&self, ruta: &Path) -> Result<(), String> {
        if let Some(dir) = ruta.parent().filter(|d| !d.as_os_str().is_empty()) {
            std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
        }
        let mut tmp = ruta.as_os_str().to_owned();
        tmp.push(".tmp");
        let tmp = PathBuf::from(tmp);
        std::fs::write(&tmp, self.a_json()).map_err(|e| e.to_string())?;
        std::fs::rename(&tmp, ruta).map_err(|e| {
            let _ = std::fs::remove_file(&tmp);
            e.to_string()
        })
    }
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Archivo {
    robots: Vec<EntradaArchivo>,
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct EntradaArchivo {
    radio: i64,
    patron: Option<i64>,
    #[serde(default = "verdadero")]
    activo: bool,
}

fn verdadero() -> bool {
    true
}

/// La ruta según el valor de `VSSL_IDENTIDAD`: el valor si no está vacío, si no
/// `config/identidad_robots.json`.
pub fn ruta_de(var: Option<String>) -> PathBuf {
    match var.filter(|v| !v.trim().is_empty()) {
        Some(v) => PathBuf::from(v.trim()),
        None => PathBuf::from(DEFAULT_PATH),
    }
}

/// La ruta del proceso (se resuelve una vez, al arrancar).
pub fn ruta() -> PathBuf {
    ruta_de(std::env::var(ENV_VAR).ok())
}

/// Lee el archivo: `None` si no existe (también con `VSSL_IDENTIDAD`: lo crea la GUI al
/// guardar), error con la ruta si no se puede leer o es inválido.
pub fn cargar(ruta: &Path) -> Result<Option<Identidad>, String> {
    match std::fs::read_to_string(ruta) {
        Ok(text) => Identidad::desde_json(&text)
            .map(Some)
            .map_err(|e| format!("{}: {e}", ruta.display())),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(e) => Err(format!("{}: {e}", ruta.display())),
    }
}

/// De dónde salió la identidad al arrancar.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Origen {
    Archivo,
    PorDefecto,
}

/// La identidad del arranque, con su ruta (donde la GUI guarda) y su origen.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Cargada {
    pub identidad: Identidad,
    pub ruta: PathBuf,
    pub origen: Origen,
}

impl Cargada {
    /// Sin leer el archivo (simulación): la identidad por defecto, con la ruta del proceso.
    pub fn por_defecto() -> Self {
        Self { identidad: Identidad::default(), ruta: ruta(), origen: Origen::PorDefecto }
    }
}

/// La identidad de un binario según su transporte: con la base station la carga (y con un
/// archivo inválido termina el proceso); en simulación no lee el archivo.
pub fn para_transporte(quien: &str, base_station: bool) -> Cargada {
    if base_station { cargar_o_salir(quien) } else { Cargada::por_defecto() }
}

/// Para los binarios con la base station: carga e imprime de dónde salió; con un archivo
/// inválido, termina el proceso antes de abrir nada.
pub fn cargar_o_salir(quien: &str) -> Cargada {
    let ruta = ruta();
    match cargar(&ruta) {
        Ok(Some(identidad)) => {
            eprintln!("[{quien}] identidad de los robots: {}", ruta.display());
            Cargada { identidad, ruta, origen: Origen::Archivo }
        }
        Ok(None) => {
            eprintln!(
                "[{quien}] sin {}: la radio N lleva el patrón N; el archivo se crea al guardar desde \
                 la pestaña Robots",
                ruta.display()
            );
            Cargada { identidad: Identidad::default(), ruta, origen: Origen::PorDefecto }
        }
        Err(e) => {
            eprintln!("[{quien}] ✗ identidad de los robots inválida: {e}");
            std::process::exit(2);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn con(cambios: &[(usize, Option<u8>, bool)]) -> Identidad {
        let mut i = Identidad::default();
        for (radio, patron, activo) in cambios {
            i.robots[*radio] = RobotFisico { patron: *patron, activo: *activo };
        }
        i
    }

    fn dir_temporal(nombre: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("vsss_identidad_{nombre}_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        d
    }

    #[test]
    fn por_defecto_es_radio_n_patron_n() {
        let i = Identidad::default();
        for n in 0..4 {
            assert_eq!(i.robots[n], RobotFisico { patron: Some(n as u8), activo: true });
            assert_eq!(i.radio_de(n as i32), Ok(n));
        }
        assert_eq!(i.robots[4], RobotFisico { patron: None, activo: true });
    }

    #[test]
    fn radio_de_unico_activo() {
        let i = con(&[(1, Some(0), true), (0, Some(1), true)]);
        assert_eq!(i.radio_de(0), Ok(1));
        assert_eq!(i.radio_de(1), Ok(0));
    }

    #[test]
    fn radio_de_sin_robot() {
        let i = con(&[(2, None, true)]);
        assert_eq!(i.radio_de(2), Err(SinRadio::SinRobot));
    }

    #[test]
    fn radio_de_repetido_cuenta_desactivados() {
        let i = con(&[(2, Some(1), false), (0, Some(1), true)]);
        assert_eq!(i.radio_de(1), Err(SinRadio::Repetido(vec![0, 1, 2])));
        let i = con(&[(0, Some(1), true), (1, None, true), (2, Some(1), false)]);
        assert_eq!(i.radio_de(1), Err(SinRadio::Repetido(vec![0, 2])));
    }

    #[test]
    fn radio_de_desactivado() {
        let i = con(&[(3, Some(3), false)]);
        assert_eq!(i.radio_de(3), Err(SinRadio::Desactivado(3)));
    }

    #[test]
    fn radio_de_fuera_de_rango() {
        let i = Identidad::default();
        assert_eq!(i.radio_de(-1), Err(SinRadio::SinRobot));
        assert_eq!(i.radio_de(4), Err(SinRadio::SinRobot));
        assert_eq!(i.radio_de(9), Err(SinRadio::SinRobot));
    }

    #[test]
    fn sin_comandos_vacia_por_defecto() {
        assert!(Identidad::default().sin_comandos().is_empty());
    }

    #[test]
    fn sin_comandos_las_tres_causas() {
        // La radio 1 (patrón 1) desactivada y la radio 2 pasa al patrón 0: el 0 repetido,
        // el 1 desactivado y el 2 sin robot; el 3 sigue en la radio 3.
        let i = con(&[(1, Some(1), false), (2, Some(0), true)]);
        assert_eq!(i.sin_comandos(), vec![0, 1, 2]);
    }

    #[test]
    fn carga_valida_cruzada() {
        let i = Identidad::desde_json(
            r#"{ "robots": [
                { "radio": 0, "patron": 1, "activo": true },
                { "radio": 1, "patron": 0, "activo": true }
            ] }"#,
        )
        .unwrap();
        assert_eq!(i.radio_de(1), Ok(0));
        assert_eq!(i.radio_de(0), Ok(1));
    }

    #[test]
    fn rechaza_patron_4() {
        let e = Identidad::desde_json(r#"{ "robots": [ { "radio": 2, "patron": 4, "activo": true } ] }"#)
            .unwrap_err();
        assert!(e.contains("radio 2") && e.contains("patrón 4") && e.contains("0..3"), "{e}");
    }

    #[test]
    fn rechaza_radio_repetida() {
        let e = Identidad::desde_json(
            r#"{ "robots": [ { "radio": 1, "patron": 0 }, { "radio": 1, "patron": 2 } ] }"#,
        )
        .unwrap_err();
        assert!(e.contains("radio 1"), "{e}");
    }

    #[test]
    fn rechaza_radio_7() {
        let e = Identidad::desde_json(r#"{ "robots": [ { "radio": 7, "patron": 0 } ] }"#).unwrap_err();
        assert!(e.contains("radio 7") && e.contains("0..4"), "{e}");
    }

    #[test]
    fn rechaza_campo_desconocido() {
        assert!(Identidad::desde_json(r#"{ "robots": [], "mapa": {} }"#).is_err());
        assert!(Identidad::desde_json(r#"{ "robots": [ { "radio": 0, "patorn": 1 } ] }"#).is_err());
    }

    #[test]
    fn radio_ausente_queda_sin_patron_y_activa() {
        let i = Identidad::desde_json(r#"{ "robots": [ { "radio": 0, "patron": 2, "activo": false } ] }"#)
            .unwrap();
        assert_eq!(i.robots[0], RobotFisico { patron: Some(2), activo: false });
        for r in &i.robots[1..] {
            assert_eq!(*r, RobotFisico { patron: None, activo: true });
        }
    }

    #[test]
    fn patron_repetido_carga_y_avisa() {
        let i = Identidad::desde_json(
            r#"{ "robots": [ { "radio": 0, "patron": 1 }, { "radio": 2, "patron": 1 } ] }"#,
        )
        .unwrap();
        let avisos = i.avisos(&[], 3);
        assert!(avisos.iter().any(|a| a.starts_with("Las radios 0 y 2 tienen el patrón 1")), "{avisos:?}");
    }

    #[test]
    fn guardar_y_cargar_da_lo_mismo() {
        let dir = dir_temporal("guardar");
        let ruta = dir.join("identidad.json");
        let i = con(&[(2, Some(2), false), (4, Some(3), true), (3, None, true)]);
        i.guardar(&ruta).unwrap();
        assert_eq!(cargar(&ruta).unwrap(), Some(i));
        let quedan: Vec<_> = std::fs::read_dir(&dir).unwrap().map(|e| e.unwrap().file_name()).collect();
        assert_eq!(quedan, vec![std::ffi::OsString::from("identidad.json")], "sin el temporal");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn sin_archivo_da_none() {
        let dir = dir_temporal("sin_archivo");
        assert_eq!(cargar(&dir.join("no_existe.json")).unwrap(), None);
    }

    #[test]
    fn archivo_invalido_nombra_la_ruta() {
        let dir = dir_temporal("invalido");
        std::fs::create_dir_all(&dir).unwrap();
        let ruta = dir.join("identidad.json");
        std::fs::write(&ruta, r#"{ "robots": [ { "radio": 7, "patron": 0 } ] }"#).unwrap();
        let e = cargar(&ruta).unwrap_err();
        assert!(e.contains("identidad.json") && e.contains("radio 7"), "{e}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn ruta_por_variable() {
        assert_eq!(ruta_de(Some("/tmp/engine_b/identidad.json".to_string())), PathBuf::from("/tmp/engine_b/identidad.json"));
        assert_eq!(ruta_de(None), PathBuf::from(DEFAULT_PATH));
    }

    #[test]
    fn ruta_variable_vacia_usa_la_por_defecto() {
        assert_eq!(ruta_de(Some(String::new())), PathBuf::from(DEFAULT_PATH));
        assert_eq!(ruta_de(Some("  ".to_string())), PathBuf::from(DEFAULT_PATH));
    }

    #[test]
    fn variable_sin_archivo_da_none_y_guardar_lo_crea() {
        let dir = dir_temporal("variable");
        let ruta = ruta_de(Some(dir.join("engine_b").join("identidad.json").display().to_string()));
        assert_eq!(cargar(&ruta).unwrap(), None);
        let i = con(&[(0, Some(2), true)]);
        i.guardar(&ruta).unwrap();
        assert_eq!(cargar(&ruta).unwrap(), Some(i));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn describir_cruzada() {
        let i = con(&[(0, Some(1), true), (1, Some(0), true), (2, Some(2), false), (3, None, true)]);
        let l = i.describir();
        assert_eq!(l.len(), 5);
        assert_eq!(l[0], "radio 0 (MI_ROBOT_ID 1): patrón 1");
        assert_eq!(l[2], "radio 2 (MI_ROBOT_ID 3): patrón 2, desactivado");
        assert_eq!(l[4], "radio 4 (MI_ROBOT_ID 5): sin patrón");
        assert!(l.iter().all(|x| !x.contains('←') && !x.contains('→')));
    }

    #[test]
    fn describir_por_defecto() {
        let l = Identidad::default().describir();
        for (n, linea) in l.iter().enumerate().take(4) {
            assert_eq!(*linea, format!("radio {n} (MI_ROBOT_ID {}): patrón {n}", n + 1));
        }
        assert_eq!(l[4], "radio 4 (MI_ROBOT_ID 5): sin patrón");
    }

    #[test]
    fn aviso_patron_repetido() {
        let i = con(&[(2, Some(1), true)]);
        assert_eq!(
            i.avisos(&[], 3),
            vec![
                "Las radios 1 y 2 tienen el patrón 1: las dos quedan quietas, y el coach y las rutinas \
                 no cuentan al robot 1, hasta que corrijas una."
                    .to_string()
            ]
        );
    }

    #[test]
    fn aviso_patron_3_con_tres_robots() {
        let i = con(&[(2, Some(3), true), (3, None, true)]);
        assert_eq!(
            i.avisos(&[0, 3], 3),
            vec!["La radio 2 tiene el patrón 3: el coach solo manda los patrones 0 a 2, así que ese robot no juega."
                .to_string()]
        );
        // Sin radio que lo tenga, lo dice igual (y no lo repite como "sin robot").
        let i = con(&[(3, None, true)]);
        assert_eq!(
            i.avisos(&[3], 3),
            vec!["La cámara ve el patrón 3: el coach solo manda los patrones 0 a 2, así que ese robot no juega."
                .to_string()]
        );
        // Si la cámara no lo ve, no hay robot que lo use: sin aviso.
        assert!(Identidad::default().avisos(&[], 3).is_empty());
    }

    #[test]
    fn aviso_visto_sin_robot() {
        let i = con(&[(2, None, true), (3, None, true)]);
        let avisos = i.avisos(&[0, 2, 2], 3);
        assert_eq!(
            avisos,
            vec![
                "La cámara ve el patrón 2 y ningún robot lo tiene: sus comandos no llegan a nadie, y el coach \
                 y las rutinas no lo cuentan."
                    .to_string()
            ]
        );
    }

    #[test]
    fn sin_avisos_por_defecto() {
        assert!(Identidad::default().avisos(&[0, 1, 2], 3).is_empty());
    }

    #[test]
    fn lista_y_mayuscula() {
        assert_eq!(lista(&[0, 2]), "0 y 2");
        assert_eq!(lista(&[0, 1, 3]), "0, 1 y 3");
        assert_eq!(con_mayuscula("radio 1"), "Radio 1");
    }
}
