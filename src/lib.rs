// Módulos públicos del engine — usados por `src/bin/scenario.rs` y tests.
// La GUI queda solo en `src/main.rs` (es exclusiva del binario principal).
pub mod protos;
pub mod vision;
pub mod tracker;
pub mod world;
pub mod motion;
pub mod radio;
pub mod skills;
pub mod tactics;
pub mod plays;
pub mod coach;
