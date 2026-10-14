//! `DecisorConRutinas`: en `main` con GUI, envuelve al decisor de base (coach o ninguno)
//! y a lo sumo una rutina, y recibe las órdenes de la GUI (D1 del design).
//!
//! Las órdenes se drenan **solo** en `observar`, al final del tick: el perfil que el lazo
//! aplica al empezar el tick siguiente y el decisor que decide en ese tick salen del mismo
//! estado, así que el coach nunca decide con los topes de una rutina.
//!
//! Un pánico de la rutina (en `decide`, `observar` u `ordenar`) se atrapa: la rutina se
//! descarta, queda abortada con "error interno de la rutina" y todos reciben Hold hasta
//! `Cerrar`. El resto del lazo sigue la ruta de `parada-segura`.

use super::decisor::{OpcionesCorrida, OrdenCorrida, RoutineDecider, hold_a_todos};
use super::formato::Rutina;
use super::resumen::VigiaResumen;
use super::vista::{Fase, RutinaVista};
use crate::coach::debug::CoachDebug;
use crate::coach::{Role, SkillChoice};
use crate::control_loop::{ContextoTick, PerfilVelocidad, TickDecider};
use crate::radio::TeleportItem;
use crate::world::World;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::Arc;
use tokio::sync::mpsc;

/// Orden de la GUI.
#[derive(Debug, Clone)]
pub enum OrdenRutina {
    Preparar { rutina: Arc<Rutina>, opciones: OpcionesCorrida },
    Iniciar,
    Pausar,
    Reanudar,
    Abortar,
    /// Suelta una rutina terminada o abortada: el control vuelve a la base.
    Cerrar,
}

enum Activa {
    Corriendo(Box<RoutineDecider>),
    /// La rutina entró en pánico: queda su última vista, abortada.
    Fallida(Arc<RutinaVista>),
}

pub struct DecisorConRutinas {
    base: Box<dyn TickDecider>,
    rutina: Option<Activa>,
    ordenes: mpsc::Receiver<OrdenRutina>,
    own_team: i32,
    num_robots: usize,
    /// Para imprimir el resumen de cada corrida una vez, al cerrarse.
    vigia: VigiaResumen,
}

/// Mensaje de un pánico atrapado.
fn mensaje(p: &(dyn std::any::Any + Send)) -> String {
    p.downcast_ref::<&str>()
        .map(|s| s.to_string())
        .or_else(|| p.downcast_ref::<String>().cloned())
        .unwrap_or_else(|| "sin mensaje".to_string())
}

impl DecisorConRutinas {
    pub fn new(base: Box<dyn TickDecider>, ordenes: mpsc::Receiver<OrdenRutina>, own_team: i32, num_robots: usize) -> Self {
        Self { base, rutina: None, ordenes, own_team, num_robots, vigia: VigiaResumen::default() }
    }

    /// Fase de la rutina, si hay una.
    pub fn fase(&self) -> Option<Fase> {
        match &self.rutina {
            None => None,
            Some(Activa::Corriendo(r)) => Some(r.fase()),
            Some(Activa::Fallida(_)) => Some(Fase::Abortada),
        }
    }

    /// Tests: la rutina en curso entra en pánico en su próximo `decide` u `observar`.
    #[cfg(test)]
    pub(crate) fn provocar_panico(&mut self) {
        if let Some(Activa::Corriendo(r)) = &mut self.rutina {
            r.panico = true;
        }
    }

    /// Descarta la rutina que entró en pánico, con su última vista abortada.
    fn fallar(&mut self, panico: &(dyn std::any::Any + Send)) {
        let causa = format!("error interno de la rutina: {}", mensaje(panico));
        eprintln!("[rutinas] ✗ {causa}");
        let vista = match &self.rutina {
            Some(Activa::Corriendo(r)) => {
                let v = r.vista();
                RutinaVista {
                    fase: Fase::Abortada,
                    frase: format!("Abortada: {causa}"),
                    robots: Vec::new(),
                    resumen: None,
                    ..(**v).clone()
                }
            }
            _ => return,
        };
        self.rutina = Some(Activa::Fallida(Arc::new(vista)));
    }

    /// Corre `f` sobre la rutina en curso, atrapando un pánico.
    fn proteger<T>(&mut self, f: impl FnOnce(&mut RoutineDecider) -> T) -> Option<T> {
        let Some(Activa::Corriendo(r)) = &mut self.rutina else { return None };
        match catch_unwind(AssertUnwindSafe(|| f(r))) {
            Ok(v) => Some(v),
            Err(p) => {
                self.fallar(p.as_ref());
                None
            }
        }
    }

    fn aplicar(&mut self, orden: OrdenRutina) {
        match orden {
            OrdenRutina::Preparar { rutina, opciones } => {
                let acepta = match &self.rutina {
                    None | Some(Activa::Fallida(_)) => true,
                    Some(Activa::Corriendo(r)) => {
                        matches!(r.fase(), Fase::Preparando | Fase::Lista | Fase::Terminada | Fase::Abortada)
                    }
                };
                if acepta {
                    let r = RoutineDecider::vigente(rutina, opciones, self.own_team, self.num_robots);
                    self.rutina = Some(Activa::Corriendo(Box::new(r)));
                } else {
                    eprintln!("[rutinas] Preparar ignorado: hay una rutina en ejecución");
                }
            }
            OrdenRutina::Cerrar => {
                let suelta = match &self.rutina {
                    Some(Activa::Corriendo(r)) => !r.fase().activa(),
                    Some(Activa::Fallida(_)) => true,
                    None => false,
                };
                if suelta {
                    self.rutina = None;
                }
            }
            OrdenRutina::Iniciar => {
                self.proteger(|r| r.ordenar(OrdenCorrida::Iniciar));
            }
            OrdenRutina::Pausar => {
                self.proteger(|r| r.ordenar(OrdenCorrida::Pausar));
            }
            OrdenRutina::Reanudar => {
                self.proteger(|r| r.ordenar(OrdenCorrida::Reanudar));
            }
            OrdenRutina::Abortar => {
                self.proteger(|r| r.ordenar(OrdenCorrida::Abortar));
            }
        }
    }
}

impl TickDecider for DecisorConRutinas {
    fn decide(&mut self, tick: u32, world: &World) -> Vec<SkillChoice> {
        match self.rutina {
            None => self.base.decide(tick, world),
            Some(Activa::Corriendo(_)) => self
                .proteger(|r| r.decide(tick, world))
                .unwrap_or_else(|| hold_a_todos(world, self.own_team, self.num_robots)),
            Some(Activa::Fallida(_)) => hold_a_todos(world, self.own_team, self.num_robots),
        }
    }

    fn role(&self, robot_id: i32) -> Option<Role> {
        if self.rutina.is_some() { None } else { self.base.role(robot_id) }
    }

    fn coach_debug(&self) -> Option<CoachDebug> {
        if self.rutina.is_some() { None } else { self.base.coach_debug() }
    }

    fn observar(&mut self, ctx: &ContextoTick<'_>) {
        if self.rutina.is_some() {
            self.proteger(|r| r.observar(ctx));
        } else {
            self.base.observar(ctx);
        }
        while let Ok(orden) = self.ordenes.try_recv() {
            self.aplicar(orden);
        }
        let vista = self.rutina();
        if let Some(r) = self.vigia.nuevo(vista.as_deref()) {
            for l in r.lineas() {
                eprintln!("[rutinas] {l}");
            }
        }
    }

    fn perfil_velocidad(&self) -> Option<PerfilVelocidad> {
        match &self.rutina {
            None => self.base.perfil_velocidad(),
            Some(Activa::Corriendo(r)) => r.perfil_velocidad(),
            Some(Activa::Fallida(_)) => None,
        }
    }

    fn tomar_teleport(&mut self) -> Vec<TeleportItem> {
        match &mut self.rutina {
            None => self.base.tomar_teleport(),
            Some(Activa::Corriendo(r)) => r.tomar_teleport(),
            Some(Activa::Fallida(_)) => Vec::new(),
        }
    }

    fn rutina(&self) -> Option<Arc<RutinaVista>> {
        match &self.rutina {
            None => self.base.rutina(),
            Some(Activa::Corriendo(r)) => r.rutina(),
            Some(Activa::Fallida(v)) => Some(v.clone()),
        }
    }
}
