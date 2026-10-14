//! Configuración de arranque que muestra la GUI (barra de estado, Inspector). Sale del
//! entorno con las mismas funciones que usan el lazo y la visión, y es de solo
//! lectura: cambiarla exige reiniciar el proceso.

use crate::radio::RadioTarget;
use crate::skills::SkillId;
use crate::vision::VisionSource;
use std::collections::BTreeMap;

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
    /// `robot.radio_slot_by_vision_id`.
    pub slot_map: BTreeMap<u32, u32>,
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
            slot_map: crate::params::params().robot.radio_slot_by_vision_id.clone(),
        }
    }

    pub fn is_base_station(&self) -> bool {
        self.transport == RadioTarget::BaseStation
    }

    /// El mapa visión → radio, solo con la base station (en simulación el id va directo).
    pub fn radio_slots(&self) -> Option<&BTreeMap<u32, u32>> {
        self.is_base_station().then_some(&self.slot_map)
    }

    /// Posición de radio de un robot propio, solo con la base station.
    pub fn radio_slot(&self, id: i32) -> Option<usize> {
        self.radio_slots()
            .and_then(|m| crate::radio::base_station::radio_slot(id, m))
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
        slot_map: BTreeMap::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn radio_slot_only_with_the_base_station() {
        let mut run = run_info_de_prueba();
        run.slot_map.insert(1, 0);
        run.slot_map.insert(0, 1);
        assert_eq!(run.radio_slot(1), None, "en simulación no hay radio");
        run.transport = RadioTarget::BaseStation;
        assert_eq!(run.radio_slot(1), Some(0));
        assert_eq!(run.radio_slot(2), Some(2), "sin entrada, el mismo id");
    }
}
