mod coach_trait;
pub mod heuristic_coach;
mod observation;
pub mod plays;
pub mod referee;
mod robot_target;
pub mod rule_based_coach;
mod skill_choice;

pub use coach_trait::Coach;
pub use heuristic_coach::{HeuristicCoach, Role};
pub use plays::{Formation, Play};
pub use referee::{
    Foul, Quadrant, RefTeam, RefereeCommand, RefereeState, SharedReferee, new_shared_referee,
    run_referee_listener,
};
pub use observation::{BallObs, FIELD_HALF_X, FIELD_HALF_Y, Observation, RobotObs};
// `RobotTarget` queda exportado pero deprecado — mantener solo para no
// romper consumidores externos hasta que el plan RL avance lo suficiente.
// Ver `coach/robot_target.rs` para el plan de retiro.
pub use robot_target::RobotTarget;
pub use rule_based_coach::RuleBasedCoach;
pub use skill_choice::SkillChoice;
