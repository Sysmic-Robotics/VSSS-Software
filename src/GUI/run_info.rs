//! Configuración de arranque que muestra la GUI (barra de estado, Inspector). Sale del
//! entorno con las mismas funciones que usan el lazo y la visión, y es de solo
//! lectura: cambiarla exige reiniciar el proceso.

use crate::identidad::{Cargada, Origen};
use crate::radio::RadioTarget;
use crate::skills::SkillId;
use crate::vision::VisionSource;
use std::path::PathBuf;

/// Qué decide los comandos de los robots propios.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoachKind {
    Heuristic,
    RuleBased,
    /// `VSSL_COACH=none`: visión y radio vivos, sin decisiones.
    Off,
    /// Una skill fija (el banco `scenario`).
    Fixed(SkillId),
}

impl CoachKind {
    /// `VSSL_COACH`, con la misma regla que `main::make_coach` (inválido → heurístico).
    pub fn from_env() -> Self {
        match std::env::var("VSSL_COACH").unwrap_or_default().as_str() {
            "rule_based" => CoachKind::RuleBased,
            "none" => CoachKind::Off,
            _ => CoachKind::Heuristic,
        }
    }
}

impl CoachKind {
    /// Hay un coach que se puede pausar (`coach-pausa`): el heurístico o el de reglas.
    pub fn se_puede_pausar(self) -> bool {
        matches!(self, CoachKind::Heuristic | CoachKind::RuleBased)
    }
}

/// Variable que hace partir la GUI con el coach en pausa (`coach-pausa`).
pub const COACH_PAUSA_ENV: &str = "VSSL_COACH_PAUSA";

/// `VSSL_COACH_PAUSA`, sin mayúsculas ni espacios: `1`, `true` u `on` → en pausa;
/// `0`, `false`, `off`, vacía o ausente → activo. Otro valor es un error, con la frase
/// de los valores aceptados: con un error de tipeo, el coach no debe jugar.
pub fn coach_pausa_de(valor: Option<&str>) -> Result<bool, String> {
    let v = valor.unwrap_or("").trim().to_ascii_lowercase();
    match v.as_str() {
        "1" | "true" | "on" => Ok(true),
        "" | "0" | "false" | "off" => Ok(false),
        _ => Err(format!(
            "{COACH_PAUSA_ENV}='{}' no vale: usa 1, true u on (coach en pausa), o 0, false u off (activo)",
            valor.unwrap_or("")
        )),
    }
}

/// `coach_pausa_de` con lo que dio `std::env::var`: un valor que no es texto válido
/// tampoco arranca (no se toma como ausente, que dejaría jugar al coach).
pub fn coach_pausa_de_var(var: Result<String, std::env::VarError>) -> Result<bool, String> {
    match var {
        Ok(v) => coach_pausa_de(Some(&v)),
        Err(std::env::VarError::NotPresent) => coach_pausa_de(None),
        Err(std::env::VarError::NotUnicode(_)) => Err(format!(
            "{COACH_PAUSA_ENV} no es texto válido: usa 1, true u on (coach en pausa), o 0, false u off (activo)"
        )),
    }
}

/// `coach_pausa_de` con el entorno.
pub fn coach_pausa_from_env() -> Result<bool, String> {
    coach_pausa_de_var(std::env::var(COACH_PAUSA_ENV))
}

/// Configuración de la corrida.
#[derive(Debug, Clone)]
pub struct RunInfo {
    pub transport: RadioTarget,
    pub vision_source: VisionSource,
    /// Proxy de ruido de cámara (`VSSL_VISION_NOISE`).
    pub noise_proxy: bool,
    /// Filtro de Kalman encendido (`VSSL_TRACKER`).
    pub tracker_on: bool,
    pub coach: CoachKind,
    /// Motion de dos caras (`MotionConfig::from_env`, misma lectura que el lazo).
    pub bidirectional: bool,
    /// El equipo propio defiende el arco izquierdo (`VSSL_SIDE`).
    pub defend_left: bool,
    /// 0 = azul, 1 = amarillo.
    pub own_team: i32,
    pub num_robots: usize,
    /// Puerto y baud de la base station con que arrancó el proceso.
    pub radio_port: String,
    pub radio_baud: String,
    /// Archivo de la identidad de los robots: de donde se leyó al arrancar y donde la
    /// pestaña Robots la guarda (`VSSL_IDENTIDAD` o `config/identidad_robots.json`).
    pub identidad_ruta: PathBuf,
    /// Si la identidad del arranque salió del archivo o es la por defecto.
    pub identidad_origen: Origen,
}

impl RunInfo {
    /// Lee del entorno lo que el caller no fija.
    pub fn from_env(
        transport: RadioTarget,
        vision_source: VisionSource,
        own_team: i32,
        num_robots: usize,
        coach: CoachKind,
    ) -> Self {
        Self {
            transport,
            vision_source,
            noise_proxy: crate::vision_tools::vision_noise_enabled(),
            tracker_on: crate::control_loop::tracker_on_from_env(),
            coach,
            bidirectional: crate::motion::MotionConfig::from_env().bidirectional,
            defend_left: crate::skills::zones::defend_left_from_env(own_team),
            own_team,
            num_robots,
            radio_port: std::env::var("VSSL_BASESTATION_DEVICE")
                .unwrap_or_else(|_| "/dev/ttyUSB0".to_string()),
            radio_baud: std::env::var("VSSL_BASESTATION_BAUD")
                .unwrap_or_else(|_| "115200".to_string()),
            identidad_ruta: crate::identidad::ruta(),
            identidad_origen: Origen::PorDefecto,
        }
    }

    /// Con la ruta y el origen de la identidad que cargó el binario.
    pub fn con_identidad(mut self, cargada: &Cargada) -> Self {
        self.identidad_ruta = cargada.ruta.clone();
        self.identidad_origen = cargada.origen;
        self
    }

    pub fn is_base_station(&self) -> bool {
        self.transport == RadioTarget::BaseStation
    }

}

#[cfg(test)]
pub(crate) fn run_info_de_prueba() -> RunInfo {
    RunInfo {
        transport: RadioTarget::FiraSim,
        vision_source: VisionSource::FiraSim,
        noise_proxy: false,
        tracker_on: true,
        coach: CoachKind::Heuristic,
        bidirectional: false,
        defend_left: true,
        own_team: 0,
        num_robots: 3,
        radio_port: "/dev/ttyUSB0".to_string(),
        radio_baud: "115200".to_string(),
        identidad_ruta: PathBuf::from(crate::identidad::DEFAULT_PATH),
        identidad_origen: Origen::PorDefecto,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coach_pause_values() {
        for v in ["1", "true", "on", " TRUE ", "On"] {
            assert_eq!(coach_pausa_de(Some(v)), Ok(true), "{v:?}");
        }
        for v in [None, Some(""), Some("0"), Some("false"), Some("OFF"), Some(" 0 ")] {
            assert_eq!(coach_pausa_de(v), Ok(false), "{v:?}");
        }
        let err = coach_pausa_de(Some("si")).unwrap_err();
        assert!(err.contains("VSSL_COACH_PAUSA='si'") && err.contains("1, true u on"), "{err}");
        assert_eq!(coach_pausa_de(Some("of f")).map_err(|_| ()), Err(()), "solo se recortan los extremos");
        // Un valor que no es texto solo se puede construir así en Unix.
        #[cfg(unix)]
        {
            use std::os::unix::ffi::OsStringExt;
            let no_texto = std::env::VarError::NotUnicode(std::ffi::OsString::from_vec(vec![0xff]));
            assert!(coach_pausa_de_var(Err(no_texto)).is_err(), "un valor que no es texto no arranca");
        }
        assert_eq!(coach_pausa_de_var(Err(std::env::VarError::NotPresent)), Ok(false));
        assert_eq!(coach_pausa_de_var(Ok("1".to_string())), Ok(true));
    }

    #[test]
    fn only_a_real_coach_can_be_paused() {
        assert!(CoachKind::Heuristic.se_puede_pausar());
        assert!(CoachKind::RuleBased.se_puede_pausar());
        assert!(!CoachKind::Off.se_puede_pausar());
        assert!(!CoachKind::Fixed(SkillId::GoTo).se_puede_pausar());
    }
}
