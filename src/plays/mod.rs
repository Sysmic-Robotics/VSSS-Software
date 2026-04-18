mod coach_play;
mod standard_play;

pub use coach_play::CoachPlay;
pub use standard_play::StandardPlay;

use crate::motion::{Motion, MotionCommand};
use crate::world::World;

/// Play: gestiona roles de todo el equipo y retorna un MotionCommand por robot activo por tick.
pub trait Play: Send + Sync {
    fn tick(&mut self, world: &World, motion: &Motion) -> Vec<MotionCommand>;
}
