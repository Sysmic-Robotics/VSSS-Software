# VSSL RustEngine

Motor de control en Rust para robots de fútbol **VSS/VSSS (Very Small Size Soccer)** compatible con **FIRASim** y robots reales (firmware ESP32-C3 + base station ESP32). Recibe visión por multicast UDP, mantiene un modelo del mundo con filtrado Kalman, computa comandos de movimiento a ~60 Hz usando Univector Field y los envía al simulador (protobuf/UDP) o a los robots reales (frame ASCII por USB serial → ESP-NOW).

---

## Requisitos

- **Rust** stable 1.70+
- Una fuente de visión (al menos una):
  - **FIRASim** (default): visión multicast en `224.0.0.1:10002`, control/actuadores en `127.0.0.1:20011`.
  - **vsss-vision-sysmic** (visión real): publica `SSL_WrapperPacket` en `224.5.23.2:10015`. Ver [NETWORK_PROTOCOL.md](NETWORK_PROTOCOL.md).
- Para robots reales: base station USB (ESP32) flasheada + 3 robots con [VSSL-firmware](https://github.com/Sysmic-Robotics/VSSL-firmware) parcheado con el watchdog ([FIRMWARE_WATCHDOG.md](FIRMWARE_WATCHDOG.md)).

No se necesita `protoc` — los bindings protobuf se generan en compilación vía `build.rs`.

### Variables de entorno

| Variable | Default | Descripción |
|----------|---------|-------------|
| `VSSL_VISION_SOURCE` | `firasim` | `firasim` o `sslvision`. Selecciona la fuente y el parser del `vision_task`. |
| `VSSL_MULTICAST_IFACE` | (auto) | IPv4 local para forzar la interfaz de multicast (útil si hay varias NICs). |
| `VSSL_RADIO_TARGET` | `firasim` | `firasim`, `grsim` o `basestation`. Selecciona a quién se le envían los `MotionCommand`. |
| `VSSL_TEAM_COLOR` | `blue` | `blue` o `yellow`. Sólo afecta a `basestation`: filtra qué comandos van al frame serial. |
| `VSSL_BASESTATION_DEVICE` | `/dev/ttyUSB0` | Path del puerto serial a la base station. |
| `VSSL_BASESTATION_BAUD` | `115200` | Baudrate del enlace USB↔ESP32 base station. |

---

## Comandos rápidos

```bash
cargo build --release   # build optimizado (necesario para tiempo real)
cargo run --release     # correr el main headless actual (default: FIRASim)
cargo run --bin scenario --release  # probar una skill manualmente en FIRASim
cargo test              # suite de tests
cargo clippy            # lint
cargo fmt               # formato
```

### Probar contra robots reales

Pre-requisitos físicos:

1. Cámara FLIR + `vsss-vision-sysmic` corriendo y calibrado en la misma PC. Validar primero con su cliente Python:
   ```bash
   cd vsss-vision-sysmic && python3 client/python/client.py
   ```
2. Base station enchufada por USB. Confirmar que aparece (ej. `/dev/ttyUSB0`):
   ```bash
   ls /dev/ttyUSB* /dev/ttyACM*
   ```
   Si necesita permisos: `sudo usermod -aG dialout $USER` (requiere re-login).
3. 3 robots encendidos, dentro del campo, con firmware parcheado (ver [FIRMWARE_WATCHDOG.md](FIRMWARE_WATCHDOG.md)).

Levantar el engine:

```bash
# Visión real + radio a la base station, equipo azul (default).
VSSL_VISION_SOURCE=sslvision \
VSSL_RADIO_TARGET=basestation \
cargo run --release
```

Variantes:

```bash
# Equipo amarillo.
VSSL_VISION_SOURCE=sslvision VSSL_RADIO_TARGET=basestation VSSL_TEAM_COLOR=yellow cargo run --release

# Otro device serial.
VSSL_VISION_SOURCE=sslvision VSSL_RADIO_TARGET=basestation VSSL_BASESTATION_DEVICE=/dev/ttyACM0 cargo run --release

# Visión real, comandos a FIRASim (debug visual: ver qué decide el engine sin mover los robots).
VSSL_VISION_SOURCE=sslvision cargo run --release
```

Smoke test del watchdog: con todo corriendo y los robots moviéndose, matar el proceso con `Ctrl+C`. Los robots deben detenerse en menos de 250 ms. Si no, revisar [FIRMWARE_WATCHDOG.md](FIRMWARE_WATCHDOG.md).

---

## Arquitectura

```
Fuente de visión (FIRASim 224.0.0.1:10002 | vsss-vision-sysmic 224.5.23.2:10015)
  └─ vision.rs         parse FIRA o SSL_WrapperPacket según VSSL_VISION_SOURCE
      └─ tracker/ekf   Extended Kalman Filter por robot/balón
          └─ world/    estado compartido Arc<RwLock<World>>
              └─ skills/   primitives reactivas simples
                  └─ motion/   UVF + PID → MotionCommand
                      └─ radio/    despacha vía RobotTransport (VSSL_RADIO_TARGET):
                                   ├─ FiraSimTransport  → UDP protobuf 127.0.0.1:20011
                                   ├─ GrSimTransport    → UDP protobuf grSim
                                   └─ BaseStationTransport → ASCII por USB serial
                                                              └─ ESP32 base → ESP-NOW → robots

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
| `radio/` | Trait `RobotTransport` + 3 implementaciones: `FiraSimTransport`, `GrSimTransport`, `BaseStationTransport` (USB serial → base station ESP32 → ESP-NOW). Selección por `VSSL_RADIO_TARGET` |
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
│   ├── mod.rs             # Radio: cola de comandos + dispatch sobre Box<dyn RobotTransport>
│   ├── transport.rs       # trait RobotTransport + FiraSimTransport / GrSimTransport
│   ├── base_station.rs    # BaseStationTransport: serial USB → ESP32 base, frame ASCII "x,y,..."
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

### Parámetros de la base station (`src/radio/base_station.rs`)

Convierte cada `MotionCommand` (frame mundial: `vx`, `vy`, `omega`, `orientation`) a el `(X, Y)` joystick que espera el firmware (`X` = giro diferencial, `Y` = avance en body frame, ambos `int8` ±100):

| Constante | Valor | Descripción |
|-----------|-------|-------------|
| `SCALE_LIN` | `200.0` | Multiplicador de la velocidad longitudinal `(vx·cosθ + vy·sinθ)` antes del clamp |
| `SCALE_ANG` | `15.0` | Multiplicador de `omega` antes del clamp |
| `JOYSTICK_MAX` | `100` | Límite duro del frame (firmware satura en ±100) |
| `SLOT_COUNT` | `5` | Slots del frame ASCII; slot index = robot id, ids fuera de rango se descartan |

Frame serial: `"x1,y1,x2,y2,x3,y3,x4,y4,x5,y5\n"` a 115200 baud. Sólo se incluyen comandos del equipo propio (`VSSL_TEAM_COLOR`); los del rival se descartan.

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
Por convención, "azul ataca hacia +X" en sim. En real, +X depende de cómo
esté montada la cámara y de qué arco sea el propio en cada partido — se
configura pasando `attack_goal` / `own_goal` a `StandardPlay::new` y
`RuleBasedCoach::new`.
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
