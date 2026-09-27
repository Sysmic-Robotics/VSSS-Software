//! Plays de pelota parada (LARC VSSS 2026 §6.6 y §10) para el coach heurístico.
//!
//! Geometría del reglamento (Fig. 2, en m, origen al centro, +x hacia el arco
//! amarillo/derecho):
//! - cruces (marcas de penal/free kick/free ball) en x = ±0.375, y ∈ {0, ±0.40};
//! - puntos de colocación de robots a 0.20 m de cada cruz sobre el eje x;
//! - línea frontal del área en |x| = 0.60 (área 0.70 × 0.15), círculo central r = 0.20;
//! - penal y free kick: pelota en la cruz central del lado rival, (±0.375, 0);
//! - goal kick: pelota frente al área propia, (∓0.575, 0), la saca el portero.
//!
//! Cada play define DÓNDE se para cada rol antes del silbato (formación). Al
//! GAME_ON la táctica normal toma el control; el pateador queda colocado en el
//! staging detrás de la pelota alineado al objetivo, así `ShootPush` es factible
//! en el primer tick (32 % de los goles de ZJUNlict son de pelota parada).

use super::referee::{Foul, Quadrant, RefereeCommand};
use glam::Vec2;

/// Cruz central (penal/free kick) medida desde el centro (m).
pub const MARK_X: f32 = 0.375;
/// Cruces laterales (free ball) en y = ±MARK_Y.
pub const MARK_Y: f32 = 0.40;
/// Distancia de los puntos de colocación a la cruz (m).
pub const MARK_OFFSET: f32 = 0.20;
/// Radio del círculo central (m).
pub const CENTER_CIRCLE_R: f32 = 0.20;
/// Línea frontal del área (|x|).
pub const AREA_X: f32 = 0.60;
/// Mitad del lado del robot (m): "tocando una línea" = centro a esta distancia.
pub const ROBOT_HALF: f32 = 0.04;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Play {
    /// Juego abierto (GAME_ON): táctica normal.
    Open,
    /// STOP / HALT: todos quietos.
    Hold,
    KickoffOurs,
    KickoffTheirs,
    FreeBall(Quadrant),
    PenaltyOurs,
    PenaltyTheirs,
    FreeKickOurs,
    FreeKickTheirs,
    GoalKickOurs,
    GoalKickTheirs,
}

impl Play {
    /// Play desde el comando del árbitro visto por el equipo `own_team` (0 azul, 1 amarillo).
    pub fn from_command(cmd: &RefereeCommand, own_team: i32) -> Self {
        let ours = cmd.team.team_id() == Some(own_team);
        match cmd.foul {
            Foul::GameOn => Play::Open,
            Foul::Stop | Foul::Halt => Play::Hold,
            Foul::Kickoff => {
                if ours {
                    Play::KickoffOurs
                } else {
                    Play::KickoffTheirs
                }
            }
            Foul::FreeBall => Play::FreeBall(cmd.quadrant),
            Foul::PenaltyKick => {
                if ours {
                    Play::PenaltyOurs
                } else {
                    Play::PenaltyTheirs
                }
            }
            Foul::FreeKick => {
                if ours {
                    Play::FreeKickOurs
                } else {
                    Play::FreeKickTheirs
                }
            }
            Foul::GoalKick => {
                if ours {
                    Play::GoalKickOurs
                } else {
                    Play::GoalKickTheirs
                }
            }
        }
    }

    pub fn is_set_piece(self) -> bool {
        !matches!(self, Play::Open | Play::Hold)
    }

    /// Jugada a favor: el pateador debe quedar listo para empujar al silbato.
    pub fn we_kick(self) -> bool {
        matches!(
            self,
            Play::KickoffOurs | Play::PenaltyOurs | Play::FreeKickOurs | Play::GoalKickOurs
        )
    }
}

/// Posiciones objetivo de los tres roles antes del silbato.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Formation {
    /// Pateador / primer atacante.
    pub striker: Vec2,
    pub support: Vec2,
    /// `None` = el arquero sigue con `GoalKeep` (línea de gol); `Some` = punto explícito.
    pub keeper: Option<Vec2>,
    /// Dónde estará la pelota según el reglamento (para alinear al pateador).
    pub ball: Vec2,
}

/// Posición reglamentaria de la pelota para la play (m). `s` = signo de ataque
/// (+1 si atacamos hacia +x).
pub fn ball_spot(play: Play, s: f32) -> Option<Vec2> {
    Some(match play {
        Play::KickoffOurs | Play::KickoffTheirs => Vec2::ZERO,
        Play::FreeBall(q) => {
            let (qx, qy) = q.signs();
            Vec2::new(qx * MARK_X, qy * MARK_Y)
        }
        Play::PenaltyOurs | Play::FreeKickOurs => Vec2::new(s * MARK_X, 0.0),
        Play::PenaltyTheirs | Play::FreeKickTheirs => Vec2::new(-s * MARK_X, 0.0),
        Play::GoalKickOurs => Vec2::new(-s * (AREA_X - 0.025), 0.0),
        Play::GoalKickTheirs => Vec2::new(s * (AREA_X - 0.025), 0.0),
        Play::Open | Play::Hold => return None,
    })
}

/// Formación para la play. `s` = signo de ataque; `aim` = punto del arco rival al
/// que apunta el pateador; `staging` = distancia del pateador detrás de la pelota
/// sobre la línea pelota→aim (la misma de `ApproachAligned`, así `ShootPush` es
/// factible al silbato).
pub fn formation(play: Play, s: f32, aim: Vec2, staging: f32) -> Option<Formation> {
    let ball = ball_spot(play, s)?;
    let behind = |ball: Vec2, dist: f32| {
        let dir = (aim - ball).normalize_or_zero();
        ball - dir * dist
    };
    let f = match play {
        // §6.6: pateador dentro del círculo central detrás de la pelota; los demás
        // en su mitad, fuera del círculo y del área.
        Play::KickoffOurs => Formation {
            striker: behind(ball, 0.12),
            support: Vec2::new(-s * 0.35, 0.30),
            keeper: None,
            ball,
        },
        Play::KickoffTheirs => Formation {
            striker: Vec2::new(-s * (CENTER_CIRCLE_R + ROBOT_HALF + 0.02), 0.0),
            support: Vec2::new(-s * 0.40, 0.30),
            keeper: None,
            ball,
        },
        // §10.4: un robot en el punto a 0.20 m de la cruz del lado de nuestro arco,
        // con un lado paralelo a los límites; el resto fuera del cuadrante.
        Play::FreeBall(q) => {
            let (_, qy) = q.signs();
            let striker = Vec2::new(ball.x - s * MARK_OFFSET, ball.y);
            let support_y = if qy == 0.0 { 0.30 } else { -qy * 0.33 };
            Formation {
                striker,
                support: Vec2::new(-s * 0.33, support_y),
                keeper: None,
                ball,
            }
        }
        // §10.2: pateador detrás de la pelota (cruz rival); los demás en el otro lado
        // desde la línea de medio campo (nuestra mitad).
        Play::PenaltyOurs => Formation {
            striker: behind(ball, staging),
            support: Vec2::new(-s * 0.10, 0.35),
            keeper: None,
            ball,
        },
        // Penal en contra: arquero en la línea; los demás en la mitad rival.
        Play::PenaltyTheirs => Formation {
            striker: Vec2::new(s * 0.12, 0.30),
            support: Vec2::new(s * 0.12, -0.30),
            keeper: None,
            ball,
        },
        // §10.1: pateador detrás de la pelota; el resto sin sobrepasar la línea de la pelota.
        Play::FreeKickOurs => Formation {
            striker: behind(ball, staging),
            support: Vec2::new(s * 0.10, 0.35),
            keeper: None,
            ball,
        },
        // Free kick en contra: defensores tocando la línea del área, uno por cuadrante,
        // fuera del arco central; arquero en la línea de gol.
        Play::FreeKickTheirs => Formation {
            striker: Vec2::new(-s * (AREA_X - ROBOT_HALF - 0.005), 0.26),
            support: Vec2::new(-s * (AREA_X - ROBOT_HALF - 0.005), -0.26),
            keeper: None,
            ball,
        },
        // §10.3: solo el arquero en el área (saca él); los demás fuera del área.
        Play::GoalKickOurs => Formation {
            striker: Vec2::new(-s * 0.30, 0.30),
            support: Vec2::new(-s * 0.30, -0.30),
            keeper: Some(Vec2::new(-s * (AREA_X + 0.07), 0.0)),
            ball,
        },
        // Goal kick rival: solo un jugador en campo de ataque, tocando la línea de
        // medio campo y fuera del semicírculo; el otro en nuestra mitad.
        Play::GoalKickTheirs => Formation {
            striker: Vec2::new(s * ROBOT_HALF, 0.35),
            support: Vec2::new(-s * 0.25, -0.25),
            keeper: None,
            ball,
        },
        Play::Open | Play::Hold => return None,
    };
    Some(f)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::coach::referee::RefTeam;

    const AIM: Vec2 = Vec2::new(0.75, 0.0);

    fn in_own_half(p: Vec2, s: f32) -> bool {
        p.x * s < 0.0
    }
    fn in_circle(p: Vec2) -> bool {
        p.length() < CENTER_CIRCLE_R
    }
    fn in_area(p: Vec2, side: f32) -> bool {
        p.x * side >= AREA_X && p.y.abs() <= 0.35
    }

    #[test]
    fn play_from_command_depends_on_own_team() {
        let c = RefereeCommand {
            foul: Foul::Kickoff,
            team: RefTeam::Blue,
            ..RefereeCommand::GAME_ON
        };
        assert_eq!(Play::from_command(&c, 0), Play::KickoffOurs);
        assert_eq!(Play::from_command(&c, 1), Play::KickoffTheirs);
        let fb = RefereeCommand {
            foul: Foul::FreeBall,
            quadrant: Quadrant::Q2,
            ..RefereeCommand::GAME_ON
        };
        assert_eq!(Play::from_command(&fb, 0), Play::FreeBall(Quadrant::Q2));
        assert_eq!(Play::from_command(&RefereeCommand::GAME_ON, 1), Play::Open);
        let halt = RefereeCommand {
            foul: Foul::Halt,
            ..RefereeCommand::GAME_ON
        };
        assert_eq!(Play::from_command(&halt, 0), Play::Hold);
    }

    #[test]
    fn kickoff_ours_kicker_inside_circle_behind_ball_others_legal() {
        let f = formation(Play::KickoffOurs, 1.0, AIM, 0.14).unwrap();
        assert!(in_circle(f.striker) && f.striker.x < 0.0);
        assert!(in_own_half(f.support, 1.0) && !in_circle(f.support));
        assert!(!in_area(f.support, -1.0));
        let t = formation(Play::KickoffTheirs, 1.0, AIM, 0.14).unwrap();
        assert!(in_own_half(t.striker, 1.0) && !in_circle(t.striker));
        assert!(in_own_half(t.support, 1.0) && !in_circle(t.support));
    }

    #[test]
    fn free_ball_puts_robot_on_our_side_dot_and_support_outside_quadrant() {
        for (q, qx, qy) in [
            (Quadrant::Q1, 1.0, 1.0),
            (Quadrant::Q2, -1.0, 1.0),
            (Quadrant::Q3, -1.0, -1.0),
            (Quadrant::Q4, 1.0, -1.0),
        ] {
            for s in [1.0f32, -1.0] {
                let f = formation(Play::FreeBall(q), s, AIM * s, 0.14).unwrap();
                assert_eq!(f.ball, Vec2::new(qx * MARK_X, qy * MARK_Y));
                // Punto a 0.20 m de la cruz, del lado de nuestro arco.
                assert!(((f.striker - f.ball).length() - MARK_OFFSET).abs() < 1e-6);
                assert!((f.striker.x - f.ball.x) * s < 0.0);
                assert_eq!(f.striker.y, f.ball.y);
                // Support fuera del cuadrante de la pelota.
                let same_quadrant = f.support.x * qx > 0.0 && f.support.y * qy > 0.0;
                assert!(!same_quadrant, "s={s} q={q:?} support={:?}", f.support);
            }
        }
        // Free ball al centro (2.º/3.º consecutivo): a 0.20 m hacia nuestro arco.
        let f = formation(Play::FreeBall(Quadrant::None), 1.0, AIM, 0.14).unwrap();
        assert_eq!(f.striker, Vec2::new(-0.20, 0.0));
    }

    #[test]
    fn penalty_and_free_kick_geometry() {
        let p = formation(Play::PenaltyOurs, 1.0, AIM, 0.14).unwrap();
        assert_eq!(p.ball, Vec2::new(MARK_X, 0.0));
        assert!(p.striker.x < p.ball.x && (p.striker - p.ball).length() < 0.15);
        assert!(in_own_half(p.support, 1.0));
        let pt = formation(Play::PenaltyTheirs, 1.0, AIM, 0.14).unwrap();
        assert_eq!(pt.ball, Vec2::new(-MARK_X, 0.0));
        assert!(!in_own_half(pt.striker, 1.0) && !in_own_half(pt.support, 1.0));
        let fk = formation(Play::FreeKickTheirs, 1.0, AIM, 0.14).unwrap();
        // Defensores tocando la línea del área por fuera, uno por cuadrante.
        for d in [fk.striker, fk.support] {
            assert!(!in_area(d, -1.0));
            assert!((d.x.abs() - (AREA_X - ROBOT_HALF - 0.005)).abs() < 1e-6);
        }
        assert!(fk.striker.y > 0.0 && fk.support.y < 0.0);
        let fo = formation(Play::FreeKickOurs, 1.0, AIM, 0.14).unwrap();
        assert!(fo.support.x < fo.ball.x, "no sobrepasar la línea de la pelota");
    }

    #[test]
    fn goal_kick_geometry() {
        let g = formation(Play::GoalKickOurs, 1.0, AIM, 0.14).unwrap();
        assert!(in_area(g.keeper.unwrap(), -1.0));
        assert!(!in_area(g.striker, -1.0) && !in_area(g.support, -1.0));
        assert!(!in_area(g.ball, -1.0), "pelota justo fuera del área: {:?}", g.ball);
        let t = formation(Play::GoalKickTheirs, 1.0, AIM, 0.14).unwrap();
        // Un solo jugador en campo rival, tocando la línea de medio campo, fuera del círculo.
        assert!(t.striker.x > 0.0 && t.striker.x <= ROBOT_HALF + 1e-6 && !in_circle(t.striker));
        assert!(in_own_half(t.support, 1.0));
    }

    #[test]
    fn mirrored_team_mirrors_formations() {
        let a = formation(Play::FreeKickOurs, 1.0, AIM, 0.14).unwrap();
        let b = formation(Play::FreeKickOurs, -1.0, -AIM, 0.14).unwrap();
        assert!((a.striker.x + b.striker.x).abs() < 1e-6);
        assert!((a.ball.x + b.ball.x).abs() < 1e-6);
    }
}
