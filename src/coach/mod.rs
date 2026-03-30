mod observation;
mod robot_target;
mod coach_trait;
pub mod rule_based_coach;

pub use observation::{Observation, RobotObs, BallObs, FIELD_HALF_X, FIELD_HALF_Y};
pub use robot_target::RobotTarget;
pub use coach_trait::Coach;
pub use rule_based_coach::RuleBasedCoach;
