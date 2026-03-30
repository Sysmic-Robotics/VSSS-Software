use crate::motion::{Motion, MotionCommand};
use crate::plays::Play;
use crate::tactics::{AttackerTactic, GoalkeeperTactic, SupportTactic, Tactic};
use crate::world::World;
use glam::Vec2;

// ──────────────────────────────────────────────────────────────────────────────
// StandardPlay: asignación fija de roles para el equipo azul.
//
//   Robot 0 → AttackerTactic  (persigue la pelota y la empuja al goal)
//   Robot 1 → SupportTactic   (se posiciona como respaldo del atacante)
//   Robot 2 → GoalkeeperTactic (defiende el arco propio)
//
// Conservado para A/B testing vs CoachPlay(RuleBasedCoach).
// ──────────────────────────────────────────────────────────────────────────────
pub struct StandardPlay {
    attacker: AttackerTactic,
    support: SupportTactic,
    goalkeeper: GoalkeeperTactic,
}

impl StandardPlay {
    /// `attack_goal`: centro del arco rival (e.g. Vec2::new(0.75, 0.0))
    /// `own_goal`:    centro del arco propio (e.g. Vec2::new(-0.75, 0.0))
    pub fn new(attack_goal: Vec2, own_goal: Vec2) -> Self {
        Self {
            attacker: AttackerTactic::new(attack_goal),
            support: SupportTactic::new(own_goal),
            goalkeeper: GoalkeeperTactic::new(own_goal),
        }
    }
}

impl Play for StandardPlay {
    fn tick(&mut self, world: &World, motion: &Motion) -> Vec<MotionCommand> {
        let robots: Vec<_> = world
            .get_blue_team_active()
            .into_iter()
            .cloned()
            .collect();

        robots
            .iter()
            .map(|robot| match robot.id {
                0 => self.attacker.tick(robot, world, motion),
                1 => self.support.tick(robot, world, motion),
                2 => self.goalkeeper.tick(robot, world, motion),
                _ => MotionCommand {
                    id: robot.id,
                    team: robot.team,
                    vx: 0.0,
                    vy: 0.0,
                    omega: 0.0,
                    orientation: robot.orientation,
                },
            })
            .collect()
    }
}
