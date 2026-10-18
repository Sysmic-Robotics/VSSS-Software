//! Armado del `CoachDebug` del coach heurístico. Es un submódulo para leer el estado
//! privado del coach sin hacerlo público: en `heuristic_coach.rs` solo hay anotaciones de
//! una línea (design de `gui-capas`, D2).
//!
//! Lo que se anota en la decisión va en `Registro`. El resto se compone al leer, con el
//! estado del coach y las mismas funciones puras que deciden (sin estado: dan los mismos
//! valores con las mismas entradas).

use super::{HeuristicCoach, RobotView, Stall};
use crate::coach::adaptation::Lane;
use crate::coach::debug::{
    AdaptacionDebug, CambioAtacante, CoachDebug, Compromiso, OpcionesAtacante, RolRobot,
    TipoObjetivo, Tiro, Trabada,
};
use crate::skills::{SkillId, shoot_push_feasible_now};
use glam::Vec2;
use std::sync::Mutex;

/// Lo anotado en las decisiones del coach.
#[derive(Debug, Default)]
pub(super) struct Registro {
    /// Tipo del último objetivo que eligió `striker_target` (desde `&self`).
    tipo_objetivo: Mutex<Option<TipoObjetivo>>,
    /// Decisión, opciones y objetivo del atacante.
    atacante: Option<(u64, OpcionesAtacante, Vec2, TipoObjetivo)>,
    /// Decisión y tiempos de llegada de los jugadores de campo.
    tiempos: Option<(u64, Vec<(i32, f32)>)>,
}

impl Registro {
    /// Anota de qué tipo es el objetivo que `striker_target` está por devolver.
    pub(super) fn tipo_objetivo(&self, tipo: TipoObjetivo) {
        if let Ok(mut t) = self.tipo_objetivo.lock() {
            *t = Some(tipo);
        }
    }
}

impl HeuristicCoach {
    /// Anota la elección del atacante (al final de `striker_choice`). `target` es el
    /// objetivo final (con la pelota trabada puede ser el compañero).
    #[allow(clippy::too_many_arguments)]
    pub(super) fn depuracion_atacante(
        &mut self,
        r: &RobotView,
        ball: Vec2,
        opp: &[RobotView],
        target: Vec2,
        scores: &[(SkillId, f32); 5],
        current: Option<SkillId>,
        wanted: SkillId,
        aplicada: SkillId,
    ) {
        // El objetivo de `striker_target` (función de `&self`, sin estado): los factores
        // del tiro de `striker_scores` se calcularon con este, no con el del pase.
        let original = self.striker_target(ball, opp);
        let tipo = if original != target {
            TipoObjetivo::Pase
        } else {
            self.depuracion
                .tipo_objetivo
                .lock()
                .ok()
                .and_then(|t| *t)
                .unwrap_or(TipoObjetivo::Arco)
        };
        let tapado = Self::shot_blocked(ball, original, opp);
        let tiro = Tiro {
            factible: shoot_push_feasible_now(r.pos, ball, original),
            tapado,
            factor: if tapado {
                self.p.score_shot_blocked_factor
            } else {
                1.0
            },
            carril: Lane::of(ball.y, self.adaptation.lane_half_width),
        };
        // Las mismas reglas que `pick_with_hysteresis` (bono a la vigente con puntaje > 0)
        // y que `commit` (después de llamarlo, `held` ya cuenta esta decisión).
        let bono = current
            .filter(|c| scores.iter().any(|(s, v)| s == c && *v > 0.0))
            .map(|c| (c, self.p.score_hysteresis));
        let minimo = self.p.skill_min_hold;
        let held = self.committed[r.id as usize].map_or(0, |c| c.held);
        let opciones = OpcionesAtacante {
            puntajes: *scores,
            ganadora: wanted,
            aplicada,
            bono,
            compromiso: Compromiso {
                skill: aplicada,
                restantes: minimo.saturating_sub(held),
                minimo,
                retuvo: aplicada != wanted,
            },
            tiro,
        };
        self.depuracion.atacante = Some((self.decision_count, opciones, target, tipo));
    }

    /// Anota los tiempos de llegada de los jugadores de campo (en `assign_roles`).
    pub(super) fn depuracion_roles(&mut self, field: &[(&RobotView, f32)]) {
        let tiempos = field.iter().map(|(r, t)| (r.id, *t)).collect();
        self.depuracion.tiempos = Some((self.decision_count, tiempos));
    }

    /// La depuración de la última decisión, compuesta con el estado del coach.
    pub(super) fn depuracion(&self) -> CoachDebug {
        let decision = self.decision_count;
        let atacante = self
            .depuracion
            .atacante
            .as_ref()
            .filter(|a| a.0 == decision);
        let tiempos: &[(i32, f32)] = self
            .depuracion
            .tiempos
            .as_ref()
            .filter(|t| t.0 == decision)
            .map_or(&[], |t| &t.1);
        let llegada = |id: i32| tiempos.iter().find(|(i, _)| *i == id).map(|(_, t)| *t);
        let roles = (0..self.committed.len() as i32)
            .map(|id| RolRobot {
                id,
                rol: self.role_of(id),
                llegada_s: llegada(id),
            })
            .collect();
        let cambio = self.striker_id.and_then(|actual| {
            let t_actual = llegada(actual)?;
            let (mejor, t_mejor) = tiempos
                .iter()
                .filter(|(i, _)| *i != actual)
                .copied()
                .min_by(|a, b| a.1.total_cmp(&b.1))?;
            Some(CambioAtacante {
                actual,
                t_actual,
                mejor,
                t_mejor,
                ganancia: self.p.role_switch_gain,
                decisiones: self.decisions_since_role_switch,
                minimo: self.p.role_min_hold,
            })
        });
        let segundos =
            |desde: u64| decision.saturating_sub(desde) as f32 / self.p.decision_hz.max(1e-3);
        let trabada = match self.push_stall {
            Stall::None => Trabada::Libre,
            Stall::Counting(d) => Trabada::Contando(segundos(d)),
            Stall::Kicking(d) => Trabada::Pateando(segundos(d)),
            Stall::SpinBanned(d) => Trabada::GiroProhibido(segundos(d)),
        };
        let a = &self.adaptation;
        CoachDebug {
            decision,
            jugada: self.current_play,
            atacante: self.striker_id,
            arquero: self.keeper_id,
            opciones: atacante.map(|a| a.1),
            skill_atacante: atacante.map(|a| a.1.aplicada),
            objetivo: atacante.map(|a| (a.2, a.3)),
            roles,
            cambio,
            trabada,
            adaptacion: AdaptacionDebug {
                activa: a.enabled,
                tiros: [
                    a.shot_rate(Lane::Upper),
                    a.shot_rate(Lane::Center),
                    a.shot_rate(Lane::Lower),
                ],
                duelos: a.duel_rate(),
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::coach::debug::Trabada;
    use crate::coach::{
        BallObs, Coach, FIELD_HALF_X, FIELD_HALF_Y, Observation, RobotObs, SkillChoice,
    };

    const ATTACK: Vec2 = Vec2::new(0.75, 0.0);
    const OWN: Vec2 = Vec2::new(-0.75, 0.0);

    fn robot(pos: Vec2, theta_deg: f32) -> RobotObs {
        RobotObs {
            x: pos.x / FIELD_HALF_X,
            y: pos.y / FIELD_HALF_Y,
            vx: 0.0,
            vy: 0.0,
            sin_theta: theta_deg.to_radians().sin(),
            cos_theta: theta_deg.to_radians().cos(),
            omega: 0.0,
            active: 1.0,
        }
    }

    fn obs(ball: Vec2, own: [Vec2; 3]) -> Observation {
        Observation {
            ball: BallObs {
                x: ball.x / FIELD_HALF_X,
                y: ball.y / FIELD_HALF_Y,
                vx: 0.0,
                vy: 0.0,
            },
            own_robots: own.iter().map(|p| robot(*p, 0.0)).collect(),
            opp_robots: vec![RobotObs::default(); 3],
            own_team: 0,
        }
    }

    fn coach() -> HeuristicCoach {
        HeuristicCoach::with_options(ATTACK, OWN, 2, true)
    }

    fn choice_of(choices: &[SkillChoice], id: i32) -> SkillChoice {
        *choices
            .iter()
            .find(|c| c.robot_id == id)
            .expect("robot sin choice")
    }

    const EQUIPO: [Vec2; 3] = [
        Vec2::new(-0.5, 0.3),
        Vec2::new(0.0, -0.1),
        Vec2::new(-0.63, 0.0),
    ];

    #[test]
    fn exported_scores_and_pick_are_the_ones_that_decided() {
        let mut c = coach();
        let ch = c.decide(&obs(Vec2::new(0.2, 0.0), EQUIPO));
        let d = c.debug().unwrap();
        assert_eq!(d.decision, 1);
        let striker = d.atacante.expect("hay atacante");
        let op = d.opciones.expect("el atacante eligió entre las 5");
        assert_eq!(op.puntajes, *c.last_striker_scores());
        assert_eq!(op.aplicada, choice_of(&ch, striker).skill_id);
        assert_eq!(d.skill_atacante, Some(op.aplicada));
        assert_eq!(d.objetivo.unwrap().0, choice_of(&ch, striker).target);
    }

    #[test]
    fn bonus_and_commitment_follow_the_current_option() {
        let mut c = coach();
        let o = obs(Vec2::new(0.2, 0.0), EQUIPO);
        c.decide(&o);
        c.decide(&o);
        let op = c.debug().unwrap().opciones.unwrap();
        let puntaje = op
            .puntajes
            .iter()
            .find(|(s, _)| *s == op.aplicada)
            .unwrap()
            .1;
        if puntaje > 0.0 {
            assert_eq!(op.bono, Some((op.aplicada, c.p.score_hysteresis)));
        }
        assert!(
            !op.compromiso.retuvo,
            "misma situación: la ganadora es la vigente"
        );
        assert_eq!(
            op.compromiso.restantes,
            c.p.skill_min_hold.saturating_sub(2)
        );
    }

    #[test]
    fn blockline_leaves_the_options_empty() {
        // Pelota dentro del área propia con el arquero activo: el atacante bloquea.
        let mut c = coach();
        let ch = c.decide(&obs(Vec2::new(-0.68, 0.0), EQUIPO));
        let d = c.debug().unwrap();
        let striker = d.atacante.unwrap();
        assert_eq!(choice_of(&ch, striker).skill_id, SkillId::BlockLine);
        assert!(d.opciones.is_none() && d.objetivo.is_none());
    }

    #[test]
    fn far_post_when_the_opponent_keeper_is_off_center() {
        let mut c = coach();
        let mut o = obs(Vec2::new(0.2, 0.0), EQUIPO);
        o.opp_robots[0] = robot(Vec2::new(0.70, 0.10), 180.0);
        c.decide(&o);
        let (punto, tipo) = c.debug().unwrap().objetivo.unwrap();
        assert_eq!(tipo, TipoObjetivo::PaloLejano);
        assert!((punto - Vec2::new(0.75, -0.14)).length() < 1e-5, "{punto}");
        // Con el arquero rival centrado, al arco.
        let mut c = coach();
        o.opp_robots[0] = robot(Vec2::new(0.70, 0.0), 180.0);
        c.decide(&o);
        assert_eq!(c.debug().unwrap().objetivo.unwrap().1, TipoObjetivo::Arco);
    }

    #[test]
    fn roles_carry_arrival_times_and_the_switch_condition() {
        let mut c = coach();
        c.decide(&obs(Vec2::new(0.2, 0.0), EQUIPO));
        let d = c.debug().unwrap();
        let keeper = d.roles.iter().find(|r| r.id == 2).unwrap();
        assert!(
            keeper.llegada_s.is_none(),
            "el arquero no compite por la pelota"
        );
        let cambio = d.cambio.expect("dos jugadores de campo");
        assert_eq!(Some(cambio.actual), d.atacante);
        assert_ne!(cambio.mejor, cambio.actual);
        assert!((cambio.umbral_s() - cambio.t_actual * c.p.role_switch_gain).abs() < 1e-6);
        assert_eq!(cambio.decisiones, c.decisions_since_role_switch);
    }

    #[test]
    fn pinned_push_shows_the_stall_and_the_spin_target() {
        // El mismo escenario que `pinned_push_turns_into_spin_kick_after_stall`.
        let mut c = coach();
        let mut o = obs(
            Vec2::new(-0.50, 0.58),
            [
                Vec2::new(-0.60, 0.60),
                Vec2::new(0.2, -0.2),
                Vec2::new(-0.63, 0.0),
            ],
        );
        o.opp_robots[0] = robot(Vec2::new(-0.42, 0.56), 180.0);
        c.decide(&o);
        assert_eq!(c.debug().unwrap().trabada, Trabada::Libre);
        let decisions = (c.p.push_stall_s * c.p.decision_hz) as usize + 3;
        for _ in 0..decisions {
            c.decide(&o);
        }
        let d = c.debug().unwrap();
        assert!(
            matches!(d.trabada, Trabada::Pateando(s) if s >= 0.0),
            "{:?}",
            d.trabada
        );
        assert_eq!(d.skill_atacante, Some(SkillId::SpinKick));
        // El giro sale hacia el mismo objetivo del empuje: a lo largo de la banda.
        assert_eq!(d.objetivo.unwrap().1, TipoObjetivo::Banda);
    }
}
