//! Barra de estado en frases: modo, coach, árbitro y salud del
//! sistema, con lo anormal en alerta, y "Detener todo" siempre visible.
//!
//! Las tasas se calculan en la GUI con los contadores monótonos de las fotos (perder
//! fotos no las cambia); el período del lazo lo mide el propio lazo. Las frases salen
//! de funciones puras con tests.

use super::format::{num, pct};
use super::run_info::{CoachKind, RunInfo};
use super::{DebugGui, Message, fonts, theme};
use crate::coach::Foul;
use crate::radio::RadioTarget;
use crate::snapshot::{LoopSnapshot, RadioCounters, VisionCounters};
use crate::vision::VisionSource;
use iced::widget::{Row, button, container, row, text};
use iced::{Alignment, Element, Length};
use std::collections::VecDeque;

// Umbrales de "anormal" (alerta).
/// Frecuencia mínima del lazo (nominal 62.5 Hz).
pub const LAZO_MIN_HZ: f32 = 61.5;
/// Peor intervalo tolerado en la ventana (dos ticks).
pub const LAZO_MAX_INTERVALO_MS: f32 = 33.0;
/// Cuadros por segundo mínimos de la visión.
pub const VISION_MIN_FPS: f32 = 55.0;
/// Sin cuadros nuevos por más que esto, "sin visión".
pub const SIN_VISION_S: f32 = 0.5;
/// Pérdidas que se muestran y pérdidas en alerta (fracción).
pub const PERDIDAS_MOSTRAR: f32 = 0.005;
pub const PERDIDAS_ALERTA: f32 = 0.02;
/// Velocidad del simulador aceptable respecto del tiempo real.
pub const SIM_MIN: f32 = 0.97;
pub const SIM_MAX: f32 = 1.03;

/// Ventana de las tasas.
const VENTANA_MS: u64 = 2000;
/// Lapso mínimo para dar una tasa.
const LAPSO_MIN_MS: u64 = 500;

/// Una frase de la barra.
#[derive(Debug, Clone, PartialEq)]
pub struct Frase {
    pub texto: String,
    /// Lo anormal: color alerta.
    pub alerta: bool,
    /// Importante pero normal (el modo manual): tinta en vez de gris.
    pub destacada: bool,
}

impl Frase {
    fn normal(texto: impl Into<String>) -> Self {
        Self {
            texto: texto.into(),
            alerta: false,
            destacada: false,
        }
    }

    fn alerta_si(texto: impl Into<String>, alerta: bool) -> Self {
        Self {
            texto: texto.into(),
            alerta,
            destacada: false,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct Muestra {
    t_ms: u64,
    vision: VisionCounters,
    radio: RadioCounters,
}

/// Tasas de la salud del sistema, sobre los últimos 2 s de fotos.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Tasas {
    pub cuadros_s: Option<f32>,
    /// Fracción de cuadros perdidos.
    pub perdidas: Option<f32>,
    pub envios_s: Option<f32>,
    /// Tiempo simulado / tiempo del lazo (FIRASim).
    pub velocidad_sim: Option<f32>,
    /// Segundos desde el último cuadro nuevo (`None` = nunca llegó uno).
    pub sin_cuadros_s: Option<f32>,
}

/// Calcula las tasas con los contadores monótonos de las fotos que recibe la GUI.
#[derive(Debug, Default)]
pub struct Salud {
    muestras: VecDeque<Muestra>,
    /// `t_ms` de la última foto en que cambió la cuenta de cuadros.
    ultimo_cuadro_ms: Option<u64>,
}

impl Salud {
    pub fn push(&mut self, t_ms: u64, vision: VisionCounters, radio: RadioCounters) {
        if self.muestras.back().is_some_and(|m| t_ms < m.t_ms) {
            // El lazo arrancó de nuevo.
            *self = Self::default();
        }
        let cambio = self
            .muestras
            .back()
            .is_none_or(|m| m.vision.frames != vision.frames);
        if cambio && vision.frames > 0 {
            self.ultimo_cuadro_ms = Some(t_ms);
        }
        self.muestras.push_back(Muestra { t_ms, vision, radio });
        while self
            .muestras
            .front()
            .is_some_and(|m| m.t_ms + VENTANA_MS < t_ms)
        {
            self.muestras.pop_front();
        }
    }

    pub fn tasas(&self) -> Tasas {
        let (Some(a), Some(b)) = (self.muestras.front(), self.muestras.back()) else {
            return Tasas::default();
        };
        let sin_cuadros_s = self
            .ultimo_cuadro_ms
            .map(|t| (b.t_ms - t) as f32 / 1000.0);
        let lapso_ms = b.t_ms - a.t_ms;
        if lapso_ms < LAPSO_MIN_MS {
            return Tasas {
                sin_cuadros_s,
                ..Tasas::default()
            };
        }
        let s = lapso_ms as f32 / 1000.0;
        let cuadros = b.vision.frames.saturating_sub(a.vision.frames);
        let perdidos = b.vision.lost.saturating_sub(a.vision.lost);
        let velocidad_sim = match (a.vision.sim_time_ms, b.vision.sim_time_ms) {
            (Some(x), Some(y)) if y > x => Some((y - x) as f32 / lapso_ms as f32),
            _ => None,
        };
        Tasas {
            cuadros_s: Some(cuadros as f32 / s),
            perdidas: (cuadros + perdidos > 0).then(|| perdidos as f32 / (cuadros + perdidos) as f32),
            envios_s: Some(b.radio.sends.saturating_sub(a.radio.sends) as f32 / s),
            velocidad_sim,
            sin_cuadros_s,
        }
    }
}

/// Frases de la barra, en orden: contexto (modo, coach, árbitro, modo manual, parada
/// y configuraciones anormales) y después la salud.
pub fn frases_estado(
    run: &RunInfo,
    snap: &LoopSnapshot,
    tasas: &Tasas,
    manual: Option<u32>,
) -> Vec<Frase> {
    let mut out = Vec::new();

    let base = match run.transport {
        RadioTarget::FiraSim => "Simulación en FIRASim",
        RadioTarget::GrSim => "Simulación en grSim",
        RadioTarget::BaseStation => "Robots reales por la base",
    };
    let modo = if run.bidirectional { "modo bidireccional" } else { "modo frontal" };
    let mut texto = format!("{base}, {modo}");
    if run.noise_proxy && run.vision_source == VisionSource::FiraSim {
        texto.push_str(", ruido de cámara activo");
    }
    out.push(Frase::normal(texto));

    out.push(Frase::normal(match run.coach {
        CoachKind::Heuristic => "coach heurístico".to_string(),
        CoachKind::RuleBased => "coach por reglas".to_string(),
        CoachKind::Off => "sin coach".to_string(),
        CoachKind::Fixed(skill) => format!("skill fija: {skill:?}"),
    }));

    out.push(match snap.referee {
        None => Frase::normal("sin árbitro"),
        Some(r) if r.seq == 0 => Frase::normal("Árbitro: sin comandos, juego en curso"),
        Some(r) => {
            let (texto, alerta) = match r.foul {
                Foul::GameOn => ("juego en curso", false),
                Foul::Stop => ("STOP, solo corrigen orientación", false),
                Foul::Halt => ("HALT, todos detenidos", true),
                Foul::FreeKick => ("tiro libre", false),
                Foul::PenaltyKick => ("penal", false),
                Foul::GoalKick => ("tiro de meta", false),
                Foul::FreeBall => ("bola libre", false),
                Foul::Kickoff => ("saque inicial", false),
            };
            Frase::alerta_si(format!("Árbitro: {texto}"), alerta)
        }
    });

    if let Some(id) = manual {
        out.push(Frase {
            texto: format!("Modo manual: las teclas mueven el robot {id}, Esc para salir"),
            alerta: false,
            destacada: true,
        });
    }
    if snap.estop {
        out.push(Frase::alerta_si("Parada de emergencia: todos en cero", true));
    }
    if !run.tracker_on {
        out.push(Frase::alerta_si("Filtro de Kalman apagado (VSSL_TRACKER)", true));
    }
    if run.noise_proxy && run.vision_source == VisionSource::SslVision {
        out.push(Frase::alerta_si(
            "Ruido de cámara simulado activo sobre la cámara real (VSSL_VISION_NOISE)",
            true,
        ));
    }

    // Salud.
    if snap.vision.frames == 0 {
        out.push(Frase::alerta_si("Sin visión", true));
    } else if let Some(s) = tasas.sin_cuadros_s.filter(|s| *s > SIN_VISION_S) {
        out.push(Frase::alerta_si(format!("Sin visión hace {} s", num(s, 1)), true));
    } else if let Some(fps) = tasas.cuadros_s {
        let latencia = snap
            .vision
            .latency_ms
            .map(|ms| format!(", {} ms", num(ms, 0)))
            .unwrap_or_default();
        out.push(Frase::alerta_si(
            format!("Visión {} cuadros/s{latencia}", num(fps, 0)),
            fps < VISION_MIN_FPS,
        ));
    }
    if let Some(p) = tasas.perdidas.filter(|p| *p > PERDIDAS_MOSTRAR) {
        out.push(Frase::alerta_si(
            format!("Pierde el {} % de los cuadros", num(p * 100.0, 1)),
            p > PERDIDAS_ALERTA,
        ));
    }
    if snap.timing.samples > 0 {
        out.push(Frase::alerta_si(
            format!(
                "Lazo {} Hz, ±{} ms",
                num(snap.timing.hz, 1),
                num(snap.timing.jitter_ms, 1)
            ),
            snap.timing.hz < LAZO_MIN_HZ || snap.timing.worst_ms > LAZO_MAX_INTERVALO_MS,
        ));
    }
    if snap.radio.last_ok == Some(false) {
        out.push(Frase::alerta_si("La radio no está enviando", true));
    } else if let Some(e) = tasas.envios_s.filter(|_| snap.radio.sends > 0) {
        out.push(Frase::normal(format!("Radio {} envíos/s", num(e, 0))));
    }
    if let Some(v) = tasas.velocidad_sim {
        out.push(Frase::alerta_si(
            format!("FIRASim corre al {} del tiempo real", pct(v)),
            !(SIM_MIN..=SIM_MAX).contains(&v),
        ));
    }
    out
}

impl DebugGui {
    /// Franja de hoja a todo el ancho: título, frases (pasan a otra línea si no
    /// entran) y "Detener todo".
    pub(super) fn vista_estado(&self) -> Element<'_, Message> {
        let manual = self.manual_enabled.then_some(self.selected_robot);
        let frases = frases_estado(&self.run, &self.snap, &self.salud.tasas(), manual);

        let mut fila = Row::new()
            .spacing(theme::SP_LG + 8.0)
            .align_y(Alignment::Center)
            .width(Length::Fill)
            .push(text("Sysmic VSSS").font(fonts::TITULO).size(theme::TXT_NOMBRE));
        for f in frases {
            let es_alerta = f.alerta;
            let destacada = f.destacada;
            let fuente = if destacada { fonts::TEXTO_FUERTE } else { fonts::TEXTO };
            fila = fila.push(text(f.texto).font(fuente).size(theme::TXT).style(move |t| {
                if es_alerta {
                    theme::alerta(t)
                } else if destacada {
                    theme::tinta_o_alerta(false)(t)
                } else {
                    theme::gris(t)
                }
            }));
        }

        let parada_activa = self.estop_activa();
        let parada = button(
            text(if parada_activa { "Soltar la parada" } else { "Detener todo (Espacio)" })
                .font(fonts::TITULO)
                .size(15.0),
        )
        .padding([7, 16])
        .style(theme::boton_parada(parada_activa))
        .on_press(Message::ToggleEstop);

        container(
            // El container afloja los límites: a un hijo `Fill` de una fila, iced le
            // pasa `min = max`, y `Wrapping` se los pasaría a cada frase.
            row![container(fila.wrap()).width(Length::Fill), parada]
                .spacing(theme::SP_LG)
                .align_y(Alignment::Center),
        )
        .padding([6, 12])
        .width(Length::Fill)
        .style(theme::panel)
        .into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::GUI::run_info::run_info_de_prueba;
    use crate::snapshot::{RefereeView, TimingStats};

    fn muestra_fira(sim_ms: u64, frames: u64) -> VisionCounters {
        VisionCounters {
            frames,
            sim_time_ms: Some(sim_ms),
            ..VisionCounters::default()
        }
    }

    #[test]
    fn simulator_speed_from_simulated_time() {
        let mut s = Salud::default();
        s.push(1000, muestra_fira(10_000, 100), RadioCounters::default());
        s.push(3000, muestra_fira(11_856, 216), RadioCounters::default());
        let t = s.tasas();
        assert!((t.velocidad_sim.unwrap() - 0.928).abs() < 1e-4);
        assert!((t.cuadros_s.unwrap() - 58.0).abs() < 1e-3);
        assert_eq!(pct(t.velocidad_sim.unwrap()), "93 %");
    }

    #[test]
    fn losing_photos_does_not_change_the_rates() {
        let mut todas = Salud::default();
        let mut pocas = Salud::default();
        for i in 0..=125u64 {
            let t = 1000 + i * 16;
            let v = muestra_fira(t * 9 / 10, i);
            let r = RadioCounters {
                sends: i,
                last_ok: Some(true),
            };
            todas.push(t, v, r);
            if i % 3 == 0 || i == 125 {
                pocas.push(t, v, r);
            }
        }
        assert_eq!(todas.tasas(), pocas.tasas());
    }

    #[test]
    fn losses_are_a_fraction_of_received_plus_lost() {
        let mut s = Salud::default();
        s.push(0, VisionCounters { frames: 10, lost: 0, ..VisionCounters::default() }, RadioCounters::default());
        s.push(1600, VisionCounters { frames: 108, lost: 2, ..VisionCounters::default() }, RadioCounters::default());
        assert!((s.tasas().perdidas.unwrap() - 0.02).abs() < 1e-6);
    }

    #[test]
    fn time_without_frames() {
        let mut s = Salud::default();
        s.push(0, muestra_fira(0, 5), RadioCounters::default());
        s.push(2100, muestra_fira(0, 5), RadioCounters::default());
        assert!((s.tasas().sin_cuadros_s.unwrap() - 2.1).abs() < 1e-6);
    }

    fn snap_sano() -> LoopSnapshot {
        LoopSnapshot {
            referee: Some(RefereeView {
                foul: Foul::GameOn,
                seq: 1,
            }),
            timing: TimingStats {
                hz: 62.5,
                jitter_ms: 0.4,
                worst_ms: 17.0,
                samples: 125,
            },
            vision: VisionCounters {
                frames: 1000,
                ..VisionCounters::default()
            },
            radio: RadioCounters {
                sends: 900,
                last_ok: Some(true),
            },
            ..LoopSnapshot::default()
        }
    }

    fn tasas_sanas() -> Tasas {
        Tasas {
            cuadros_s: Some(60.0),
            perdidas: Some(0.0),
            envios_s: Some(62.0),
            velocidad_sim: Some(1.0),
            sin_cuadros_s: Some(0.0),
        }
    }

    fn textos(f: &[Frase]) -> Vec<&str> {
        f.iter().map(|x| x.texto.as_str()).collect()
    }

    #[test]
    fn typical_simulation() {
        let mut run = run_info_de_prueba();
        run.bidirectional = true;
        run.noise_proxy = true;
        let f = frases_estado(&run, &snap_sano(), &tasas_sanas(), None);
        let t = textos(&f);
        assert!(t.contains(&"Simulación en FIRASim, modo bidireccional, ruido de cámara activo"), "{t:?}");
        assert!(t.contains(&"coach heurístico"));
        assert!(t.contains(&"Árbitro: juego en curso"));
        assert!(t.contains(&"Visión 60 cuadros/s"));
        assert!(t.contains(&"Lazo 62.5 Hz, ±0.4 ms"));
        assert!(t.contains(&"FIRASim corre al 100 % del tiempo real"));
        assert!(f.iter().all(|x| !x.alerta), "nada anormal: {f:?}");
    }

    #[test]
    fn real_robots() {
        let mut run = run_info_de_prueba();
        run.transport = RadioTarget::BaseStation;
        run.vision_source = VisionSource::SslVision;
        let mut t = tasas_sanas();
        t.velocidad_sim = None;
        let f = frases_estado(&run, &snap_sano(), &t, None);
        assert_eq!(f[0].texto, "Robots reales por la base, modo frontal");
        assert!(!textos(&f).iter().any(|x| x.contains("FIRASim")));
    }

    #[test]
    fn late_simulator_is_an_alert() {
        let mut t = tasas_sanas();
        t.velocidad_sim = Some(0.928);
        let f = frases_estado(&run_info_de_prueba(), &snap_sano(), &t, None);
        let sim = f.iter().find(|x| x.texto.starts_with("FIRASim")).unwrap();
        assert_eq!(sim.texto, "FIRASim corre al 93 % del tiempo real");
        assert!(sim.alerta);
    }

    #[test]
    fn radio_down_is_an_alert() {
        let mut snap = snap_sano();
        snap.radio.last_ok = Some(false);
        let f = frases_estado(&run_info_de_prueba(), &snap, &tasas_sanas(), None);
        assert!(f.iter().any(|x| x.texto == "La radio no está enviando" && x.alerta));
    }

    #[test]
    fn noise_proxy_on_the_real_camera_is_an_alert() {
        let mut run = run_info_de_prueba();
        run.vision_source = VisionSource::SslVision;
        run.noise_proxy = true;
        let f = frases_estado(&run, &snap_sano(), &tasas_sanas(), None);
        assert!(f.iter().any(|x| x.alerta && x.texto.contains("cámara real")));
        assert!(!f[0].texto.contains("ruido"), "con la cámara real no es el ruido del simulador");
    }

    #[test]
    fn tracker_off_is_an_alert() {
        let mut run = run_info_de_prueba();
        run.tracker_on = false;
        let f = frases_estado(&run, &snap_sano(), &tasas_sanas(), None);
        assert!(f.iter().any(|x| x.alerta && x.texto.contains("Kalman")));
    }

    #[test]
    fn stop_halt_and_estop() {
        let mut snap = snap_sano();
        snap.referee = Some(RefereeView { foul: Foul::Stop, seq: 2 });
        let f = frases_estado(&run_info_de_prueba(), &snap, &tasas_sanas(), None);
        assert!(f.iter().any(|x| x.texto == "Árbitro: STOP, solo corrigen orientación" && !x.alerta));
        snap.referee = Some(RefereeView { foul: Foul::Halt, seq: 3 });
        snap.estop = true;
        let f = frases_estado(&run_info_de_prueba(), &snap, &tasas_sanas(), None);
        assert!(f.iter().any(|x| x.texto == "Árbitro: HALT, todos detenidos" && x.alerta));
        assert!(f.iter().any(|x| x.texto == "Parada de emergencia: todos en cero" && x.alerta));
    }

    #[test]
    fn manual_mode_is_highlighted_not_an_alert() {
        let f = frases_estado(&run_info_de_prueba(), &snap_sano(), &tasas_sanas(), Some(1));
        let m = f.iter().find(|x| x.texto.starts_with("Modo manual")).unwrap();
        assert_eq!(m.texto, "Modo manual: las teclas mueven el robot 1, Esc para salir");
        assert!(m.destacada && !m.alerta);
    }

    #[test]
    fn slow_loop_and_lost_frames() {
        let mut snap = snap_sano();
        snap.timing.hz = 58.0;
        let mut t = tasas_sanas();
        t.perdidas = Some(0.03);
        let f = frases_estado(&run_info_de_prueba(), &snap, &t, None);
        assert!(f.iter().any(|x| x.texto.starts_with("Lazo 58.0 Hz") && x.alerta));
        assert!(f.iter().any(|x| x.texto == "Pierde el 3.0 % de los cuadros" && x.alerta));
    }

    #[test]
    fn latency_when_known() {
        let mut snap = snap_sano();
        snap.vision.latency_ms = Some(92.4);
        let f = frases_estado(&run_info_de_prueba(), &snap, &tasas_sanas(), None);
        assert!(textos(&f).contains(&"Visión 60 cuadros/s, 92 ms"));
    }

    #[test]
    fn no_vision() {
        let mut snap = snap_sano();
        snap.vision.frames = 0;
        let f = frases_estado(&run_info_de_prueba(), &snap, &Tasas::default(), None);
        assert!(f.iter().any(|x| x.texto == "Sin visión" && x.alerta));
    }

    #[test]
    fn no_text_uses_middle_dots_or_warning_signs() {
        let mut snap = snap_sano();
        snap.estop = true;
        let f = frases_estado(&run_info_de_prueba(), &snap, &tasas_sanas(), Some(0));
        assert!(f.iter().all(|x| !x.texto.contains('·') && !x.texto.contains('⚠')));
    }
}
