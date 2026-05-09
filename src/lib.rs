// Módulos públicos del engine — usados por `src/bin/scenario.rs` y tests.
// La GUI queda solo en `src/main.rs` (es exclusiva del binario principal).
//
// Fase 1 del plan RL (2026): se eliminan `tactics/` y `plays/` del crate.
// El control multi-robot va a vivir en `coach/` con la nueva interfaz
// `Coach::decide -> Vec<SkillChoice>` (Fase 3). `RuleBasedCoach` se mantiene
// como baseline para A/B testing contra el modelo RL.
pub mod coach;
pub mod motion;
pub mod protos;
pub mod radio;
pub mod skills;
pub mod tracker;
pub mod vision;
pub mod world;
