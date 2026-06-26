mod coach_trait;
mod observation;
#[cfg(feature = "rl")]
pub mod rl_coach;
mod robot_target;
pub mod rule_based_coach;
mod skill_choice;

pub use coach_trait::Coach;
#[cfg(feature = "rl")]
pub use rl_coach::RlCoach;
pub use observation::{BallObs, FIELD_HALF_X, FIELD_HALF_Y, Observation, RobotObs};
// `RobotTarget` queda exportado pero deprecado — mantener solo para no
// romper consumidores externos hasta que el plan RL avance lo suficiente.
// Ver `coach/robot_target.rs` para el plan de retiro.
pub use robot_target::RobotTarget;
pub use rule_based_coach::RuleBasedCoach;
pub use skill_choice::SkillChoice;
