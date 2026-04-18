mod coach_trait;
mod observation;
mod robot_target;
pub mod rule_based_coach;

pub use coach_trait::Coach;
pub use observation::{BallObs, FIELD_HALF_X, FIELD_HALF_Y, Observation, RobotObs};
pub use robot_target::RobotTarget;
pub use rule_based_coach::RuleBasedCoach;
