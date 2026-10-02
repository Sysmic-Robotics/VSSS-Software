use crate::motion::RobotCommand;
use crate::radio::transport::{RobotTransport, TransportError};
use async_trait::async_trait;
use std::collections::BTreeMap;
use std::time::Duration;
use tokio::io::AsyncWriteExt;
use tokio_serial::{SerialPortBuilderExt, SerialStream};

/// Equipo propio que la base station controla.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TeamColor {
    Blue,
    Yellow,
}

impl TeamColor {
    pub fn as_team_id(self) -> i32 {
        match self {
            TeamColor::Blue => 0,
            TeamColor::Yellow => 1,
        }
    }

    pub fn from_env() -> Self {
        match std::env::var("VSSL_TEAM_COLOR")
            .unwrap_or_default()
            .to_ascii_lowercase()
            .as_str()
        {
            "yellow" | "amarillo" | "1" => TeamColor::Yellow,
            "" | "blue" | "azul" | "0" => TeamColor::Blue,
            other => {
                eprintln!(
                    "[BaseStation] VSSL_TEAM_COLOR='{other}' inválido, usando 'blue' por defecto"
                );
                TeamColor::Blue
            }
        }
    }
}

/// Cantidad de slots en el frame ASCII (la base espera 5 pares V,W).
pub const SLOT_COUNT: usize = 5;

// Contrato del frame ASCII PC → base ESP32. Verificado el 2026-09-30 contra
// `VSSL-firmware/test/base_station_lineal_angulo.ino` y `VSSL-firmware/src`
// (rama `Peluche`); detalle en `docs/contrato_comunicacion_v2.md`.
//   - Formato exacto: "V1,W1,V2,W2,V3,W3,V4,W4,V5,W5\n". Son 10 enteros decimales
//     separados por coma, con terminador '\n' (sin '\r'). La base hace
//     sscanf("%d,...") y, si la línea no trae 10 enteros (p. ej. con decimales),
//     la descarta y sigue enviando el comando anterior.
//   - V = velocidad lineal del robot en mm/s; W = velocidad angular en GRADOS/s.
//     La base recorta a ±1500 mm/s y ±720 °/s; aquí se aplican los mismos topes,
//     leídos de `params().robot` (`max_v_mm_s`, `max_w_deg_s`).
//   - Slots desde 0: el robot con MI_ROBOT_ID = N (firmware, desde 1) lee
//     robots[N - 1] (communication.cpp:96), es decir, la posición N - 1 del frame.
//     El robot del checkout actual del firmware tiene MI_ROBOT_ID 2 → slot 1
//     (`skill_test --mode vw --robot 1`).
//   - cmd.id es el id de VISIÓN (parche de colores; con él aparece en World y en
//     la GUI). La posición sale de `radio_slot`: `robot.radio_slot_by_vision_id`
//     de team_params.json si el id tiene entrada, o el mismo id si no. Sin mapa,
//     posición = cmd.id. FIRASim/grSim no usan esto (id directo).
//   - La cinemática diferencial (reparto a cada rueda con WHEEL_TRACK_MM) y un PI
//     de guiñada con el giroscopio los hace el FIRMWARE. El PC no calcula ruedas.
//   - El firmware topa cada rueda en 450 mm/s (MAX_WHEEL_MM_S) escalando AMBAS
//     ruedas por el mismo factor: conserva la curvatura (v/ω) y baja la velocidad.
//     El PC no replica ese tope: necesitaría WHEEL_TRACK_MM, y recortar solo v
//     cambiaría la curva. Por eso el robot puede ejecutar menos de lo enviado.
//   - Convención: omega > 0 → W > 0 → giro antihorario visto desde arriba (en el
//     firmware, rueda derecha más rápida). Coincide con la convención de Spin.

/// Convierte un `MotionCommand` (marco mundo, m/s y rad/s) en lo que llega al
/// robot real: `(v_mm_s, w_deg_s)`.
///
/// - `v` = proyección de `(vx, vy)` al heading (`orientation`), en mm/s. La
///   componente lateral se descarta porque el robot diferencial no la ejecuta.
/// - `w` = `omega` en grados/s.
/// - Ambos se redondean al entero más cercano y se recortan con `clamp_vw`.
/// - Si algún campo no es finito, devuelve `(0, 0)` (estado seguro).
///
/// Es la ÚNICA fuente de verdad de lo que se envía: la usan el frame, el CSV de
/// auditoría (`skill_log`) y el debug de la GUI. No replica el tope de 450 mm/s
/// por rueda del firmware (ver el comentario del contrato, arriba).
pub fn command_to_vw(motion: &crate::motion::MotionCommand) -> (i16, i16) {
    if !motion.vx.is_finite()
        || !motion.vy.is_finite()
        || !motion.omega.is_finite()
        || !motion.orientation.is_finite()
    {
        return (0, 0);
    }

    let v_m_s = motion.vx * motion.orientation.cos() + motion.vy * motion.orientation.sin();
    clamp_vw(v_m_s * 1000.0, motion.omega.to_degrees())
}

/// Redondea y recorta un par `(v_mm_s, w_deg_s)` a ±`max_v_mm_s` / ±`max_w_deg_s`
/// de `params().robot`. El clamp se hace en f64 y el cast `as i16` satura, así
/// que ningún valor de params puede hacer overflow ni wrap.
pub fn clamp_vw(v_mm_s: f64, w_deg_s: f64) -> (i16, i16) {
    let robot = &crate::params::params().robot;
    (
        clamp_round(v_mm_s, robot.max_v_mm_s),
        clamp_round(w_deg_s, robot.max_w_deg_s),
    )
}

fn clamp_round(value: f64, max: i32) -> i16 {
    let max = f64::from(max.max(0));
    value.round().clamp(-max, max) as i16
}

/// Posición en el frame del robot con id de visión `vision_id`: la entrada del
/// mapa (`robot.radio_slot_by_vision_id`) o el mismo id si no tiene. `None` si el
/// id es negativo o la posición queda fuera de `0..SLOT_COUNT` (se descarta).
pub fn radio_slot(vision_id: i32, slot_map: &BTreeMap<u32, u32>) -> Option<usize> {
    let id = u32::try_from(vision_id).ok()?;
    let slot = slot_map.get(&id).copied().unwrap_or(id) as usize;
    (slot < SLOT_COUNT).then_some(slot)
}

/// Mapeo vigente en texto, una línea por entrada más la regla del resto. Única
/// fuente para el log de `BaseStationTransport::new` y el panel de radio de la GUI.
pub fn describe_slot_map(slot_map: &BTreeMap<u32, u32>) -> Vec<String> {
    if slot_map.is_empty() {
        return vec!["sin mapa: visión #N → radio pos N (MI_ROBOT_ID N + 1)".to_string()];
    }
    slot_map
        .iter()
        .map(|(id, slot)| format!("visión #{id} → radio pos {slot} (MI_ROBOT_ID {})", slot + 1))
        .chain(std::iter::once("resto: visión #N → radio pos N".to_string()))
        .collect()
}

/// Construye el frame ASCII "V1,W1,...,V5,W5\n" a partir de comandos del equipo propio.
/// Posición = `radio_slot(cmd.id, slot_map)`; ids fuera de rango se ignoran; los
/// slots sin comando salen 0,0.
pub fn build_frame(
    commands: &[RobotCommand],
    own_team: TeamColor,
    slot_map: &BTreeMap<u32, u32>,
) -> String {
    let mut slots = [(0i16, 0i16); SLOT_COUNT];
    for cmd in commands {
        if cmd.team != own_team.as_team_id() {
            continue;
        }
        if let Some(idx) = radio_slot(cmd.id, slot_map) {
            slots[idx] = command_to_vw(&cmd.motion);
        }
    }
    build_frame_from_vw(slots)
}

/// Arma el frame ASCII directamente desde pares `(v_mm_s, w_deg_s)` ya
/// calculados (modo `vw` de `skill_test`, bring-up en lazo abierto).
///
/// **No clampea**: el caller entrega valores ya en rango (normalmente vía
/// `send_raw_vw_frame`, que sí clampea). Así la función queda pura y barata.
///
/// Para los mismos slots coincide byte a byte con `build_frame` (ver el test
/// `build_frame_paths_equivalent`).
pub fn build_frame_from_vw(slots: [(i16, i16); SLOT_COUNT]) -> String {
    let mut out = String::with_capacity(64);
    for (i, (v, w)) in slots.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        out.push_str(&format!("{v},{w}"));
    }
    out.push('\n');
    out
}

pub struct BaseStationTransport {
    port: SerialStream,
    own_team: TeamColor,
    device: String,
    /// `robot.radio_slot_by_vision_id`, copiado de los params al abrir.
    slot_map: BTreeMap<u32, u32>,
}

impl BaseStationTransport {
    pub fn new(device: &str, baud: u32, own_team: TeamColor) -> Result<Self, TransportError> {
        let port = tokio_serial::new(device, baud)
            .timeout(Duration::from_millis(50))
            .open_native_async()?;
        eprintln!(
            "[BaseStation] Abierto {device} @ {baud} baud, equipo propio = {:?}",
            own_team
        );
        let slot_map = crate::params::params().robot.radio_slot_by_vision_id.clone();
        for line in describe_slot_map(&slot_map) {
            eprintln!("[BaseStation] {line}");
        }
        Ok(Self {
            port,
            own_team,
            device: device.to_string(),
            slot_map,
        })
    }

    pub fn from_env(own_team: TeamColor) -> Result<Self, TransportError> {
        let device =
            std::env::var("VSSL_BASESTATION_DEVICE").unwrap_or_else(|_| "/dev/ttyUSB0".to_string());
        let baud = std::env::var("VSSL_BASESTATION_BAUD")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(115200u32);
        Self::new(&device, baud, own_team)
    }

    /// Envío directo de pares `(v_mm_s, w_deg_s)` sin pasar por `command_to_vw`.
    /// Pensado para el modo `vw` del probador `skill_test` (bring-up del robot real).
    ///
    /// Clampea con `clamp_vw` antes de armar el frame, como defensa en profundidad
    /// (el CLI ya clampea); `build_frame_from_vw`, como función pura, no lo hace.
    pub async fn send_raw_vw_frame(
        &mut self,
        slots: [(i16, i16); SLOT_COUNT],
    ) -> Result<(), TransportError> {
        let clamped = slots.map(|(v, w)| clamp_vw(f64::from(v), f64::from(w)));
        let frame = build_frame_from_vw(clamped);
        match self.port.write_all(frame.as_bytes()).await {
            Ok(()) => Ok(()),
            Err(e) => {
                eprintln!("[BaseStation] error escribiendo en {}: {e}", self.device);
                Err(e.into())
            }
        }
    }
}

#[async_trait]
impl RobotTransport for BaseStationTransport {
    async fn send_commands(&mut self, commands: &[RobotCommand]) -> Result<(), TransportError> {
        let frame = build_frame(commands, self.own_team, &self.slot_map);
        match self.port.write_all(frame.as_bytes()).await {
            Ok(()) => Ok(()),
            Err(e) => {
                eprintln!("[BaseStation] error escribiendo en {}: {e}", self.device);
                Err(e.into())
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::{KickerCommand, MotionCommand};
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

    fn make_cmd(
        team: i32,
        id: i32,
        vx: f64,
        vy: f64,
        omega: f64,
        orientation: f64,
    ) -> RobotCommand {
        RobotCommand {
            id,
            team,
            motion: MotionCommand {
                id,
                team,
                vx,
                vy,
                omega,
                orientation,
            },
            kicker: KickerCommand {
                id,
                team,
                kick_x: false,
                kick_z: false,
                dribbler: 0.0,
            },
        }
    }

    fn motion(vx: f64, vy: f64, omega: f64, orientation: f64) -> MotionCommand {
        MotionCommand {
            id: 0,
            team: 0,
            vx,
            vy,
            omega,
            orientation,
        }
    }

    /// Topes con los que trabaja `command_to_vw` (los mismos params que la función).
    fn limits() -> (i16, i16) {
        let robot = &crate::params::params().robot;
        (robot.max_v_mm_s as i16, robot.max_w_deg_s as i16)
    }

    // ---------- Conversión (v, ω) ----------

    #[test]
    fn vw_pure_forward_zero_heading() {
        assert_eq!(command_to_vw(&motion(1.0, 0.0, 0.0, 0.0)), (1000, 0));
    }

    #[test]
    fn vw_pure_forward_heading_pi_over_2() {
        // Robot mirando a +Y: la proyección al heading recupera v = vy.
        assert_eq!(command_to_vw(&motion(0.0, 0.5, 0.0, FRAC_PI_2)), (500, 0));
    }

    #[test]
    fn vw_lateral_velocity_is_discarded() {
        // Velocidad perpendicular al heading: el diferencial no la ejecuta.
        assert_eq!(command_to_vw(&motion(0.0, 0.5, 0.0, 0.0)), (0, 0));
    }

    #[test]
    fn vw_pure_ccw_spin() {
        assert_eq!(command_to_vw(&motion(0.0, 0.0, FRAC_PI_2, 0.0)), (0, 90));
    }

    #[test]
    fn vw_pure_cw_spin_negative_omega() {
        assert_eq!(command_to_vw(&motion(0.0, 0.0, -FRAC_PI_2, 0.0)), (0, -90));
    }

    #[test]
    fn vw_forward_plus_turn() {
        assert_eq!(command_to_vw(&motion(0.3, 0.0, FRAC_PI_4, 0.0)), (300, 45));
    }

    #[test]
    fn vw_rounds_small_omega_to_nearest_degree() {
        // 0.01 rad/s = 0.573 °/s → 1.
        assert_eq!(command_to_vw(&motion(0.0, 0.0, 0.01, 0.0)), (0, 1));
    }

    // ---------- Clamp ----------

    #[test]
    fn clamp_v_forward() {
        let (max_v, _) = limits();
        assert_eq!(command_to_vw(&motion(5.0, 0.0, 0.0, 0.0)), (max_v, 0));
    }

    #[test]
    fn clamp_v_backward() {
        let (max_v, _) = limits();
        assert_eq!(command_to_vw(&motion(-5.0, 0.0, 0.0, 0.0)), (-max_v, 0));
    }

    #[test]
    fn clamp_w_both_signs() {
        let (_, max_w) = limits();
        assert_eq!(command_to_vw(&motion(0.0, 0.0, 100.0, 0.0)), (0, max_w));
        assert_eq!(command_to_vw(&motion(0.0, 0.0, -100.0, 0.0)), (0, -max_w));
    }

    #[test]
    fn clamp_spin_20_rad_s_saturates_w() {
        // Spin manda 20 rad/s (≈ 1146 °/s): en el robot real se recorta al tope angular.
        let (_, max_w) = limits();
        assert!(20.0_f64.to_degrees() > f64::from(max_w));
        assert_eq!(command_to_vw(&motion(0.0, 0.0, 20.0, 0.0)), (0, max_w));
    }

    #[test]
    fn clamp_borderline_does_not_overflow() {
        let (max_v, _) = limits();
        let edge = f64::from(max_v) / 1000.0;
        assert_eq!(command_to_vw(&motion(edge, 0.0, 0.0, 0.0)), (max_v, 0));
        assert_eq!(command_to_vw(&motion(-edge, 0.0, 0.0, 0.0)), (-max_v, 0));
    }

    #[test]
    fn firmware_wheel_cap_is_not_replicated() {
        // (0.45 m/s, 180 °/s) le pide > 450 mm/s a una rueda en el firmware, que
        // escala ambas en bloque. El PC lo manda tal cual.
        let (max_v, max_w) = limits();
        assert!(450 <= max_v && 180 <= max_w);
        assert_eq!(command_to_vw(&motion(0.45, 0.0, PI, 0.0)), (450, 180));
    }

    #[test]
    fn clamp_vw_saturates_raw_pairs() {
        // Mismo helper que usa `send_raw_vw_frame` antes de armar el frame.
        let (max_v, max_w) = limits();
        assert_eq!(clamp_vw(3000.0, -3000.0), (max_v, -max_w));
        assert_eq!(clamp_vw(-300.0, 45.0), (-300, 45));
    }

    // ---------- Robustez ante no-finitos ----------

    #[test]
    fn robustness_nan_vx_returns_zero() {
        assert_eq!(command_to_vw(&motion(f64::NAN, 0.0, 0.0, 0.0)), (0, 0));
    }

    #[test]
    fn robustness_nan_vy_returns_zero() {
        assert_eq!(command_to_vw(&motion(0.0, f64::NAN, 0.0, 0.0)), (0, 0));
    }

    #[test]
    fn robustness_inf_omega_returns_zero() {
        assert_eq!(command_to_vw(&motion(0.0, 0.0, f64::INFINITY, 0.0)), (0, 0));
    }

    #[test]
    fn robustness_neg_inf_orientation_returns_zero() {
        assert_eq!(
            command_to_vw(&motion(1.0, 0.0, 0.0, f64::NEG_INFINITY)),
            (0, 0)
        );
    }

    // ---------- Frame golden ----------

    #[test]
    fn frame_golden_forward_blue_slot_0() {
        let cmds = vec![make_cmd(0, 0, 0.5, 0.0, 0.0, 0.0)];
        assert_eq!(
            build_frame(&cmds, TeamColor::Blue, &no_map()),
            "500,0,0,0,0,0,0,0,0,0\n"
        );
    }

    #[test]
    fn frame_golden_ccw_spin_blue_slot_1() {
        // Slot 1 = robot con MI_ROBOT_ID 2 (el del checkout actual del firmware).
        let cmds = vec![make_cmd(0, 1, 0.0, 0.0, FRAC_PI_2, 0.0)];
        assert_eq!(
            build_frame(&cmds, TeamColor::Blue, &no_map()),
            "0,0,0,90,0,0,0,0,0,0\n"
        );
    }

    #[test]
    fn frame_negative_omega_slot_4() {
        let cmds = vec![make_cmd(0, 4, 0.0, 0.0, -FRAC_PI_2, 0.0)];
        assert_eq!(
            build_frame(&cmds, TeamColor::Blue, &no_map()),
            "0,0,0,0,0,0,0,0,0,-90\n"
        );
    }

    // ---------- Filtro de team y descarte ----------

    #[test]
    fn frame_filters_opposing_team() {
        // own_team = Blue, comando con team = Yellow → ignorado.
        let cmds = vec![make_cmd(1, 0, 1.0, 0.0, 0.0, 0.0)];
        assert_eq!(build_frame(&cmds, TeamColor::Blue, &no_map()), "0,0,0,0,0,0,0,0,0,0\n");
    }

    #[test]
    fn frame_drops_id_out_of_range() {
        let cmds = vec![make_cmd(0, 9, 1.0, 0.0, 0.0, 0.0)];
        assert_eq!(build_frame(&cmds, TeamColor::Blue, &no_map()), "0,0,0,0,0,0,0,0,0,0\n");
    }

    #[test]
    fn frame_yellow_team_filters_blue_out() {
        // own_team = Yellow → comando team=0 (Blue) ignorado; team=1 (Yellow) entra.
        let cmds = vec![
            make_cmd(0, 0, 1.0, 0.0, 0.0, 0.0),
            make_cmd(1, 1, 0.5, 0.0, 0.0, 0.0),
        ];
        assert_eq!(
            build_frame(&cmds, TeamColor::Yellow, &no_map()),
            "0,0,500,0,0,0,0,0,0,0\n"
        );
    }

    // ---------- Mapa id de visión → posición de radio ----------

    fn no_map() -> BTreeMap<u32, u32> {
        BTreeMap::new()
    }

    fn lab_map() -> BTreeMap<u32, u32> {
        BTreeMap::from([(0, 1), (1, 0)])
    }

    #[test]
    fn frame_maps_vision_id_to_radio_slot() {
        // La visión ve el robot como #1, pero escucha la posición 0 (MI_ROBOT_ID 1).
        let cmds = vec![make_cmd(0, 1, 0.5, 0.0, 0.0, 0.0)];
        assert_eq!(
            build_frame(&cmds, TeamColor::Blue, &lab_map()),
            "500,0,0,0,0,0,0,0,0,0\n"
        );
    }

    #[test]
    fn frame_unmapped_id_keeps_its_own_slot() {
        let cmds = vec![make_cmd(0, 2, 0.5, 0.0, 0.0, 0.0)];
        assert_eq!(
            build_frame(&cmds, TeamColor::Blue, &lab_map()),
            "0,0,0,0,500,0,0,0,0,0\n"
        );
    }

    #[test]
    fn radio_slot_rules() {
        assert_eq!(radio_slot(1, &no_map()), Some(1));
        assert_eq!(radio_slot(1, &lab_map()), Some(0));
        assert_eq!(radio_slot(0, &lab_map()), Some(1));
        assert_eq!(radio_slot(-1, &lab_map()), None);
        assert_eq!(radio_slot(9, &no_map()), None);
        // Un id de visión alto puede ir a una posición válida.
        assert_eq!(radio_slot(7, &BTreeMap::from([(7, 2)])), Some(2));
    }

    #[test]
    fn describe_slot_map_with_and_without_map() {
        assert_eq!(
            describe_slot_map(&lab_map()),
            vec![
                "visión #0 → radio pos 1 (MI_ROBOT_ID 2)",
                "visión #1 → radio pos 0 (MI_ROBOT_ID 1)",
                "resto: visión #N → radio pos N",
            ]
        );
        assert_eq!(
            describe_slot_map(&no_map()),
            vec!["sin mapa: visión #N → radio pos N (MI_ROBOT_ID N + 1)"]
        );
    }

    // ---------- build_frame_from_vw (camino crudo) ----------

    #[test]
    fn build_from_vw_golden_slot_2_with_signs() {
        let slots: [(i16, i16); SLOT_COUNT] = [(0, 0), (0, 0), (-300, 45), (0, 0), (0, 0)];
        assert_eq!(build_frame_from_vw(slots), "0,0,0,0,-300,45,0,0,0,0\n");
    }

    #[test]
    fn build_from_vw_does_not_clamp() {
        // Función pura: el caller es responsable del clamp.
        let slots: [(i16, i16); SLOT_COUNT] = [(3000, -1000), (0, 0), (0, 0), (0, 0), (0, 0)];
        assert_eq!(build_frame_from_vw(slots), "3000,-1000,0,0,0,0,0,0,0,0\n");
    }

    /// Equivalencia byte a byte: si build_frame resuelve los slots S para un set de
    /// comandos, build_frame_from_vw(S) produce el mismo string.
    #[test]
    fn build_frame_paths_equivalent() {
        let cmds = vec![
            make_cmd(0, 0, 0.5, 0.0, 0.0, 0.0),
            make_cmd(0, 2, 0.3, 0.0, FRAC_PI_4, 0.0),
        ];
        let slots: [(i16, i16); SLOT_COUNT] = [
            command_to_vw(&cmds[0].motion),
            (0, 0),
            command_to_vw(&cmds[1].motion),
            (0, 0),
            (0, 0),
        ];
        assert_eq!(
            build_frame(&cmds, TeamColor::Blue, &no_map()),
            build_frame_from_vw(slots)
        );
    }
}
