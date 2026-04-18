# VSSL RustEngine

Motor de control en Rust para robots de fútbol **VSS/VSSS (Very Small Size Soccer)** compatible con **FIRASim**. Recibe visión por multicast UDP, mantiene un modelo del mundo con filtrado Kalman, computa comandos de movimiento a ~60 Hz usando Univector Field y los envía al simulador por protobuf/UDP.

---

## Requisitos

- **Rust** stable 1.70+
- **FIRASim** corriendo con:
  - Visión multicast en `224.0.0.1:10002`
  - Control/actuadores en `127.0.0.1:20011`

No se necesita `protoc` — los bindings protobuf se generan en compilación vía `build.rs`.

---

## Comandos rápidos

```bash
cargo build --release   # build optimizado (necesario para tiempo real)
cargo run --release     # correr el main headless actual
cargo run --bin scenario --release  # probar una skill manualmente en FIRASim
cargo test              # suite de tests
cargo clippy            # lint
cargo fmt               # formato
```

---

## Arquitectura

```
FIRASim (multicast 224.0.0.1:10002)
  └─ vision.rs         parse SSL-Vision protobuf
      └─ tracker/ekf   Extended Kalman Filter por robot/balón
          └─ world/    estado compartido Arc<RwLock<World>>
              └─ skills/   primitives reactivas simples
                  └─ motion/   UVF + PID → MotionCommand
                      └─ radio/    serializa protobuf → UDP 127.0.0.1:20011
                          └─ FIRASim recibe y aplica

Camino alternativo / futuro:
  world/ → coach/ → plays/ → motion/ → radio/
```

### Módulos principales

| Módulo | Función |
|--------|---------|
| `vision.rs` | Receptor UDP multicast; parsea SSL-Vision protobuf; emite `VisionEvent` al resto del sistema |
| `tracker/` | EKF por entidad (robot/balón). Estado: posición + orientación + velocidades. Activable desde GUI |
| `world/` | Estado canónico del juego: poses de robots, posición/velocidad del balón, flags de inactividad |
| `coach/` | Base futura para estrategia/RL. Hoy expone observaciones y coaches, pero no es la ruta por defecto del `main` |
| `plays/` | Orquestación de alto nivel (`StandardPlay`, `CoachPlay`). Quedó como base futura; el `main` actual no entra por aquí |
| `skills/` | Primitives reactivas simples con trait `Skill -> MotionCommand`: `ChaseBallSkill`, `GoToSkill`, `FacePointSkill`, `HoldPositionSkill`, etc. |
| `tactics/` | Wrappers de rol sobre skills (`AttackerTactic`, `SupportTactic`, `GoalkeeperTactic`) con `StuckDetector`; útiles como base futura |
| `motion/` | UVF para evasión de obstáculos, PID para heading y velocidad, braking profile |
| `radio/` | Clientes UDP para FIRASim y GrSim; serializa `RobotCommand` a protobuf |
| `GUI/` | Interfaz Iced para inspección visual. Sigue en el repo, pero el flujo actual de `main` y `scenario` es headless |
| `protos/` | Bindings Rust generados en compilación desde `.proto` (FIRA, SSL-Vision, grSim) |

---

## Estructura de carpetas

```
src/
├── main.rs                # Entry point headless actual: visión + world + una skill simple + radio
├── vision.rs              # Recepción y parsing de visión
├── world/
│   ├── mod.rs             # World struct, Arc<RwLock> state
│   ├── robot_state.rs     # RobotState: posición, velocidad, orientación, active
│   └── ball_state.rs      # BallState: posición, velocidad
├── tracker/
│   ├── mod.rs             # Tracker: despacha EKFs por (team, id)
│   └── ekf.rs             # Extended Kalman Filter
├── coach/                 # ── CAPA DE ESTRATEGIA (RL-ready) ──
│   ├── mod.rs
│   ├── observation.rs     # Observation: 52 floats normalizados para el modelo RL
│   ├── robot_target.rs    # RobotTarget: {robot_id, position, face_target}
│   ├── coach_trait.rs     # Coach trait — el único trait que implementa el modelo RL
│   └── rule_based_coach.rs # Implementación clásica del Coach (fallback y A/B testing)
├── plays/
│   ├── mod.rs             # Play trait
│   ├── standard_play.rs   # Orquestación fija de tácticas (camino alternativo, no default)
│   └── coach_play.rs      # Puente Coach → Motion para integración futura de strategy/RL
├── skills/
│   └── mod.rs             # trait Skill -> MotionCommand + primitives reactivas simples
├── tactics/
│   └── mod.rs             # AttackerTactic, SupportTactic, GoalkeeperTactic, StuckDetector
├── motion/
│   ├── mod.rs             # Motion struct: move_to, move_and_face, move_direct, face_to
│   ├── uvf.rs             # Univector Field — evasión de obstáculos tangencial
│   ├── environment.rs     # Environment: obstáculos (robots + pelota) + límites de campo
│   ├── pid.rs             # PIDController con anti-windup e initialized flag
│   ├── commands.rs        # MotionCommand, KickerCommand, RobotCommand
│   └── benchmark.rs       # KPI de movimiento (velocidad media, etc.)
├── radio/
│   ├── mod.rs             # Radio: coordina clientes, cola de comandos
│   ├── firasim.rs         # FIRASimClient: UDP → 127.0.0.1:20011
│   ├── grsim.rs           # GrSimClient: UDP → grSim (soporte parcial)
│   └── commands.rs        # Serialización MotionCommand → protobuf
├── GUI/
│   ├── mod.rs             # App Iced, update/view principal
│   ├── field.rs           # Canvas 2D del campo
│   └── vision_status.rs   # Panel de estado de visión
└── protos/                # Bindings auto-generados — NO editar manualmente
```

---

## Parámetros clave (`src/main.rs`)

| Constante | Valor | Descripción |
|-----------|-------|-------------|
| `OWN_TEAM` | `0` | Equipo controlado por el binario principal (`0` azul, `1` amarillo) |
| `ROBOT_ID` | `0` | Robot que controla la base mínima actual |

El `main` actual es deliberadamente pequeño: usa la misma base headless que `scenario`, sin GUI ni plays.

### Parámetros de movimiento (`src/motion/mod.rs`)

| Constante | Valor | Descripción |
|-----------|-------|-------------|
| `MAX_LINEAR_SPEED` | `1.2 m/s` | Velocidad máxima lineal |
| `MAX_ANGULAR_SPEED` | `3.0 rad/s` | Velocidad angular máxima |
| `BRAKE_DISTANCE` | `0.50 m` | Distancia al goal desde la que empieza frenado |
| `ARRIVAL_THRESHOLD` | `0.06 m` | Radio de llegada al target |

### Parámetros del Univector Field (`src/motion/uvf.rs`)

| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `influence_radius` | `0.20 m` | Radio de influencia de obstáculos |
| `k_rep` | `1.5` | Ganancia repulsiva tangencial |

### Parámetros de skills de balón (`src/skills/mod.rs`)

Estas skills siguen disponibles como primitives reactivas. Hoy se usan sobre todo para pruebas manuales en `scenario` y como base para una capa futura de strategy.

| Parámetro | Skill | Valor | Descripción |
|-----------|-------|-------|-------------|
| `staging_offset` | `ApproachBallBehindSkill` | `0.16 m` | Distancia detrás de la pelota para el staging point |
| `staging_tol` | `ApproachBallBehindSkill` | `0.08 m` | Radio desde el cual la skill deja de trasladar y prioriza orientar al robot |
| `push_overshoot` | `PushBallSkill` | `0.12 m` | Distancia más allá de la pelota al empujar |
| `lose_radius` | `PushBallSkill` | `0.25 m` | Si el robot pierde demasiado la pelota, la skill pasa a frenar |
| `kp/ki/kd` | `ApproachBallBehindSkill`, `PushBallSkill`, `AlignBallToTargetSkill` | `1.2/0.0/0.10` | Ganancias PID de heading para primitives orientadas a la pelota |

---

## Integración del modelo RL

El engine está diseñado para recibir un modelo de RL con cambios mínimos. El seam de integración es el trait `Coach` en `src/coach/coach_trait.rs`.

### Cómo conectar el modelo (cuando esté listo)

**1.** Crear `src/coach/rl_coach.rs`:

```rust
use crate::coach::{Coach, Observation, RobotTarget};

pub struct RlCoach { /* model handle */ }

impl RlCoach {
    pub fn load(path: &str) -> Self { ... }
}

impl Coach for RlCoach {
    fn decide(&mut self, obs: &Observation) -> Vec<RobotTarget> {
        let input = obs.to_flat_vec(); // 52 floats — ver contrato abajo
        // inferencia → Vec<RobotTarget>
    }
}
```

**2.** Conectar ese coach a un entry point que instancie `CoachPlay` o a tu futuro módulo `strategy`.

Hoy `main.rs` no tiene un switch para `CoachPlay`; el seam RL sigue existiendo en `coach/` y `plays/`, pero la ruta por defecto del runtime es la base headless de skills simples.

### Contrato de la observación (`Observation::to_flat_vec()`)

Vector fijo de **52 floats**. Este layout es el contrato entre el engine Rust y el trainer Python — no cambiar sin actualizar ambos lados.

```
Índice  Campo
0       ball.x          / 0.75    (field half-x)
1       ball.y          / 0.65    (field half-y)
2       ball.vx         / 1.5     (max vel norm)
3       ball.vy         / 1.5
4..11   own_robots[0]:  x, y, vx, vy, sin(θ), cos(θ), ω/π, active
12..19  own_robots[1]:  ...
20..27  own_robots[2]:  ...
28..35  opp_robots[0]:  ...
36..43  opp_robots[1]:  ...
44..51  opp_robots[2]:  ...
```

Robots inactivos → todos los campos `0.0`, `active = 0.0`. El tamaño es siempre 52.

La orientación se codifica como `(sin θ, cos θ)` (no ángulo crudo) para evitar la discontinuidad en ±π que rompe gradientes de redes neuronales.

---

## Campo VSS (referencia)

```
         Y
         ^
         |
 ←───────┼────────→ X
(-0.75,0)│          (0.75,0)
  arco   │           arco
 amarillo│            azul
         │
Azul ataca hacia +X (DEBUG_MOTION_CAPTURE_GOAL_X = 0.75)
```

- Campo físico: ±0.75 m × ±0.65 m
- Campo lógico (con margen 5cm): ±0.70 m × ±0.60 m
- Radio de colisión robot: 0.06 m
- Radio de colisión pelota: 0.05 m

---

## Concurrencia

```
Tokio runtime:
  ├─ vision_task      (event-driven) UDP multicast → World
  ├─ world_updater    (100 ms)       marca robots inactivos
  └─ control_loop     (16 ms, 60 Hz) Skill → Radio → FIRASim
```

Estado compartido: `Arc<TokioRwLock<World>>`. Comunicación inter-task: canales `mpsc`.
La GUI sigue en el repo, pero no forma parte del flujo por defecto del `main`.

---

## Tests

```bash
cargo test                                              # todos los tests
cargo test test_robot_motion_simulation -- --nocapture  # simulación headless 8s
```

68 tests unitarios que cubren: UVF, motion, PID, Environment, radio, skills, tactics, observation/coach, world, tracker y vision.
