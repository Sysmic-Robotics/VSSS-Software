# VSSL RustEngine

Motor de control en Rust para robots de fútbol **VSS/VSSS (Very Small Size Soccer)** compatible con **FIRASim** y robots reales (firmware ESP32-C3 + base station ESP32). Recibe visión por multicast UDP, mantiene un modelo del mundo con filtrado Kalman, computa comandos de movimiento a ~60 Hz usando Univector Field y los envía al simulador (protobuf/UDP) o a los robots reales (frame ASCII por USB serial → ESP-NOW).

---

## Requisitos

- **Rust** stable 1.70+
- Una fuente de visión (al menos una):
  - **FIRASim** (default): visión multicast en `224.0.0.1:10002`, control/actuadores en `127.0.0.1:20011`.
  - **vsss-vision-sysmic** (visión real): publica `SSL_WrapperPacket` en `224.5.23.2:10015`.
- Para robots reales: base station USB (ESP32) flasheada + robots con [VSSL-firmware](https://github.com/Sysmic-Robotics/VSSL-firmware) en modo ESP-NOW (`#define MODO_BASESTATION` activo en `config.h`).

No se necesita `protoc` — los bindings protobuf se generan en compilación vía `build.rs`.

### Variables de entorno

| Variable | Default | Descripción |
|----------|---------|-------------|
| `VSSL_VISION_SOURCE` | `firasim` | `firasim` o `sslvision`. Selecciona la fuente y el parser del `vision_task`. |
| `VSSL_MULTICAST_IFACE` | (auto) | IPv4 local para forzar la interfaz de multicast (útil si hay varias NICs). |
| `VSSL_RADIO_TARGET` | `firasim` | `firasim`, `grsim` o `basestation`. Selecciona a quién se le envían los `MotionCommand`. |
| `VSSL_COACH` | `rule_based` | `rule_based`, `rl` o `none`. `rl` carga el modelo ONNX entrenado (requiere compilar con `--features rl`). |
| `VSSL_RL_MODEL` | `training/models/policy.onnx` | Path al ONNX de la política de campo (sólo con `VSSL_COACH=rl`). |
| `VSSL_RL_GK_MODEL` | `training/models/policy_gk.onnx` | ONNX del arquero entrenado (sólo con `VSSL_COACH=rl`). `none` fuerza el arquero rule-based. |
| `VSSL_TEAM_COLOR` | `blue` | `blue` o `yellow`. Sólo afecta a `basestation`: filtra qué comandos van al frame serial. |
| `VSSL_BASESTATION_DEVICE` | `/dev/ttyUSB0` | Path del puerto serial a la base station. |
| `VSSL_BASESTATION_BAUD` | `115200` | Baudrate del enlace USB↔ESP32 base station. |

---

## Comandos rápidos

```bash
cargo build --release                # build optimizado (necesario para tiempo real)
cargo run --release                  # main headless (coach decide) — default FIRASim
VSSL_DEBUG_GUI=1 cargo run --release # main con GUI Iced
cargo run --bin scenario --release   # banco "editar y correr" para 1 skill, GUI siempre on, CSV opcional a logs/
cargo run --bin skill_test --release -- --help   # probador CLI (modo skill o wheels)
cargo test                           # suite completa (lib + bin + doctests)
cargo clippy                         # lint
cargo fmt                            # formato
```

**3 binarios:**
- `rustengine` (default): producción headless o con `VSSL_DEBUG_GUI=1`. Coach decide qué skill correr.
- `scenario`: banco de pruebas tipo "editar y correr". Una skill a la vez, configurada como constantes en la zona de edición al inicio de `src/bin/scenario.rs`. GUI siempre activa. CSV opcional a `logs/scenario_<skill>_<epoch>.csv` (toggle con la constante `log: Option<PathBuf>`: `Some(scenario_log_path(&scenario))` escribe, `None` desactiva el archivo y deja solo el resumen humano a stderr).
- `skill_test`: probador CLI. Modo `skill` (lazo cerrado, sim o real) y modo `wheels` (lazo abierto, mm/s directos al robot real para bring-up). Ver `--help`.

### Probar contra robots reales

Pre-requisitos físicos:

1. Cámara FLIR + `vsss-vision-sysmic` corriendo y calibrado en la misma PC. Validar primero con su cliente Python:
   ```bash
   cd vsss-vision-sysmic && python3 client/python/client.py
   ```
2. Base station enchufada por USB. Confirmar **qué device asignó el kernel** (NO siempre es `/dev/ttyUSB0`):
   ```bash
   ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
   # Si recién la enchufaste: dmesg | tail -20
   ```
   Si aparece `/dev/ttyUSB1` (u otro), exportá `VSSL_BASESTATION_DEVICE=/dev/ttyUSB1` antes de correr el engine. El default es `/dev/ttyUSB0` y si no existe vas a ver `[control_loop] radio error: No such file or directory` y todo se cae (incluida la visión, porque el runtime termina).
   Si necesita permisos: `sudo usermod -aG dialout $USER` (requiere re-login) o `sudo chmod 666 /dev/ttyUSB1` como parche puntual.
3. Robots encendidos con firmware en modo ESP-NOW (`#define MODO_BASESTATION` activo en `VSSL-firmware/include/config.h` y `MI_ROBOT_ID` asignado por robot).

Levantar el engine:

```bash
# Visión real + radio a la base station, equipo azul (default).
# Ajustar VSSL_BASESTATION_DEVICE al device que el kernel le asignó a la base.
VSSL_VISION_SOURCE=sslvision \
VSSL_RADIO_TARGET=basestation \
VSSL_BASESTATION_DEVICE=/dev/ttyUSB0 \
VSSL_DEBUG_GUI=1 \
cargo run --release
```

Variantes:

```bash
# Equipo amarillo.
VSSL_VISION_SOURCE=sslvision VSSL_RADIO_TARGET=basestation VSSL_TEAM_COLOR=yellow cargo run --release

# Visión real, comandos a FIRASim (debug visual: ver qué decide el engine sin mover los robots).
VSSL_VISION_SOURCE=sslvision cargo run --release

# Bring-up de hardware: mandar L/R mm/s directos a un robot sin pasar por visión ni skills.
# Útil para diagnosticar signos de rueda, comunicación y unidades.
# Secuencia recomendada para descubrir signos invertidos:
cargo run --bin skill_test --release -- --transport base-station --mode wheels --team blue --robot 0 --left 500  --right 0    --dur 1   # solo izquierda → pivota a la derecha
cargo run --bin skill_test --release -- --transport base-station --mode wheels --team blue --robot 0 --left 0    --right 500  --dur 1   # solo derecha → pivota a la izquierda
cargo run --bin skill_test --release -- --transport base-station --mode wheels --team blue --robot 0 --left 500  --right 500  --dur 1   # ambas → debe AVANZAR RECTO (si gira: signo invertido en alguna rueda)
cargo run --bin skill_test --release -- --transport base-station --mode wheels --team blue --robot 0 --left -500 --right 500  --dur 1   # giro CCW sobre el eje (confirma convención Spin)
```

Smoke test del watchdog (200 ms en firmware, `COMM_TIMEOUT_MS` en `config.h`): con los robots moviéndose, `Ctrl+C` y deben detenerse rápido.

---

## Arquitectura

```
Fuente de visión (FIRASim 224.0.0.1:10002 | vsss-vision-sysmic 224.5.23.2:10015)
  └─ vision.rs           parse FIRA o SSL_WrapperPacket según VSSL_VISION_SOURCE
      └─ tracker/ekf     Extended Kalman Filter por robot/balón
          └─ world/      estado compartido Arc<RwLock<World>>
              ↓
       control_loop.rs   ← TickDecider decide qué skill correr para cada robot
              │            ├─ CoachDecider (main): RuleBasedCoach o RL futuro, frame-skip 6 (10 Hz)
              │            ├─ FixedSkillDecider (scenario, skill_test): una skill fija por CLI/constante
              │            └─ NoOpDecider (main con VSSL_COACH=none)
              ↓
       skills/catalog.rs ← SkillCatalog::tick(robot_id, skill_id, target, robot, world, motion)
              ↓
          motion/        UVF + PID → MotionCommand (vx, vy, omega)
              ↓
          radio/         despacha vía RobotTransport (VSSL_RADIO_TARGET):
                         ├─ FiraSimTransport     → UDP protobuf 127.0.0.1:20011
                         ├─ GrSimTransport       → UDP protobuf grSim
                         └─ BaseStationTransport → cinemática inversa diferencial
                                                   → ASCII "L,R mm/s" por USB serial
                                                   → ESP32 base → ESP-NOW → robots
```

### Módulos principales

| Módulo | Función |
|--------|---------|
| `vision.rs` | Receptor UDP multicast; parsea FIRA o SSL_WrapperPacket según `VSSL_VISION_SOURCE`; emite `VisionEvent` |
| `tracker/` | EKF por entidad (robot/balón). Estado: posición + orientación + velocidades |
| `world/` | Estado canónico del juego: poses de robots, posición/velocidad del balón, flags de inactividad |
| `coach/` | Coach trait + `RuleBasedCoach` baseline + contrato `Observation` (52 floats) para el modelo RL futuro |
| `skills/` | Catálogo congelado RL: `SkillId::{GoTo, FacePoint, ChaseBall, Spin, PushBall}` (`SkillCatalog::tick`). Skills out-of-catalog viven en `skills/mod.rs` para otros usos |
| `motion/` | UVF para evasión de obstáculos, PID para heading y velocidad, braking profile |
| `radio/` | Trait `RobotTransport` + 3 implementaciones (`FiraSimTransport`, `GrSimTransport`, `BaseStationTransport`). Selección por `VSSL_RADIO_TARGET`. `Radio::from_target` explícito |
| `control_loop.rs` | **Loop 60 Hz único** que `main`, `scenario` y `skill_test` invocan. `TickDecider` (`CoachDecider`/`FixedSkillDecider`/etc) decide qué skill; el resto del lazo (visión → world → dispatch → transport) es el mismo |
| `skill_log.rs` | `CsvLogger`, `CsvRow`, `SkillLogCtx::build_skill_row` — fuente única del formato CSV compartida por `scenario` y `skill_test` |
| `GUI/` | Interfaz Iced para inspección visual. Encendida siempre en `scenario`, opcional en `main` (`VSSL_DEBUG_GUI=1`) |
| `protos/` | Bindings Rust generados en compilación desde `.proto` (FIRA, SSL-Vision, grSim) |

---

## Estructura de carpetas

```
src/
├── main.rs                # Wrapper: lee env, arma CoachDecider, llama run_control_loop
├── lib.rs
├── control_loop.rs        # Loop 60 Hz único. TickDecider trait + CoachDecider + FixedSkillDecider
├── skill_log.rs           # CsvLogger + CsvRow + SkillLogCtx::build_skill_row (fuente única del CSV)
├── vision.rs              # Recepción multicast (FIRA o SSL_WrapperPacket) + filtros
├── bin/
│   ├── scenario.rs        # Banco "editar y correr": 1 skill, GUI on, CSV a logs/
│   └── skill_test.rs      # Probador CLI: --mode skill|wheels (bring-up real)
├── world/                 # World, RobotState, BallState (Arc<RwLock>)
├── tracker/               # EKF por entidad
├── coach/                 # Coach trait + RuleBasedCoach + Observation (52 floats RL)
├── skills/                # SkillCatalog congelado (GoTo, FacePoint, ChaseBall, Spin, PushBall) + skills out-of-catalog
├── motion/                # UVF + PID + MotionCommand
├── radio/
│   ├── mod.rs             # Radio + RadioTarget. Radio::from_target explícito.
│   ├── transport.rs       # trait RobotTransport + FiraSimTransport / GrSimTransport
│   ├── base_station.rs    # BaseStationTransport: cinemática inversa diferencial → ASCII "L1,R1,...,L5,R5\n" mm/s
│   ├── firasim.rs         # FIRASimClient: UDP → 127.0.0.1:20011
│   ├── grsim.rs           # GrSimClient: UDP protobuf
│   └── commands.rs        # Serialización MotionCommand → protobuf
├── GUI/                   # App Iced (campo 2D + paneles de visión / robots)
└── protos/                # Bindings auto-generados — NO editar
```

---

## Parámetros clave (`src/main.rs`)

| Constante | Valor | Descripción |
|-----------|-------|-------------|
| `OWN_TEAM` | `0` | Equipo controlado por el binario principal (`0` azul, `1` amarillo) |
| `NUM_ROBOTS` | `3` | Cantidad de slots de `SkillCatalog` (un slot por robot del equipo propio) |
| `COACH_DECISION_PERIOD` | `6` | Frame-skip del coach: decide cada 6 ticks (10 Hz) a 60 Hz de control |

`main` arma un `CoachDecider` con esos parámetros y delega TODO el lazo en `run_control_loop`. Los demás binarios (`scenario`, `skill_test`) corren el mismo loop con un decisor distinto.

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

`BaseStationTransport` hace la **cinemática inversa diferencial en Rust** y envía velocidades de rueda en mm/s a la base ESP32 nueva (`base_station2.ino` en la raíz del repo monorepo). La base reenvía un binario de 24 bytes por ESP-NOW al robot, que las aplica directamente con su PID interno.

| Constante | Valor | Descripción |
|-----------|-------|-------------|
| `WHEEL_BASE_M` | `0.07` | Separación física entre ruedas del robot real, en metros (medida 2026-06-13). Calibración fina del giro pendiente de validar |
| `MAX_WHEEL_MM_S` | `1500` | Clamp duro de seguridad (mismo límite que aplica la base en `base_station2.ino`) |
| `SLOT_COUNT` | `5` | Slots del frame ASCII; el robot físico con `MI_ROBOT_ID = N` (firmware, 1-based) lee `slots[N-1]` |

**Cinemática inversa** (en `command_to_wheel_mm_s`):
```
v     = vx·cos(orientation) + vy·sin(orientation)   // m/s (proyección al heading)
v_izq = v − (omega · WHEEL_BASE_M) / 2              // m/s
v_der = v + (omega · WHEEL_BASE_M) / 2              // m/s
```
Convención: `omega > 0` → CCW visto desde arriba → rueda derecha más rápida (matchea la convención `Spin` del catálogo).

**Frame serial:** `"L1,R1,L2,R2,L3,R3,L4,R4,L5,R5\n"` (enteros decimales, mm/s, terminador `\n`), 115200 baud. Solo se incluyen comandos del equipo propio (`VSSL_TEAM_COLOR`).

**NaN/Inf safety:** si cualquier campo del `MotionCommand` es no-finito, `command_to_wheel_mm_s` devuelve `(0, 0)` sin pánico (estado seguro, igual al watchdog del firmware).

**Bring-up directo de ruedas** (sin pasar por cinemática inversa): `BaseStationTransport::send_raw_wheels_frame(slots)` o el binario `skill_test --mode wheels`.

---

## Integración del modelo RL

El engine **ya integra** el modelo de RL entrenado en Python. El seam es el trait
`Coach` (`src/coach/coach_trait.rs`); la implementación viva está en
`src/coach/rl_coach.rs`, gateada tras el feature `rl` (usa `tract-onnx`, motor de
inferencia ONNX en Rust puro — no se fuerza en el build por defecto).

### Arquitectura del deploy

- **1 política de campo compartida** (entrenada con self-play): recibe la
  observación de 52 floats y controla los **2 robots de campo** (salida de 6
  floats = 2 × `[skill_sel, tx, ty]`).
- **Arquero (robot id 2) aparte.** `RlCoach` carga un 2º ONNX para el arquero
  **entrenado** (`policy_gk.onnx`, por defecto). Si el archivo no carga, o con
  `VSSL_RL_GK_MODEL=none`, cae al arquero **rule-based**.

### Correr el modelo contra FIRASim (turnkey)

Visión y radio ya hacen default a FiraSim, así que sólo hace falta la feature
`rl`, el modelo ONNX y `VSSL_COACH=rl`:

```bash
# 1. Levantar FIRASim.
# 2. Compilar con la feature rl (trae tract-onnx).
cargo build --release --features rl
# 3. Correr el engine con el coach RL. El modelo se lee de
#    training/models/policy.onnx (override con VSSL_RL_MODEL).
VSSL_COACH=rl cargo run --release --features rl
#    (GUI de debug: anteponer VSSL_DEBUG_GUI=1)
```

El ONNX (`training/models/policy.onnx`) viene **versionado en el repo** (forzado
pese a `*.onnx` en .gitignore) — no hace falta el entorno Python para correrlo.
Para re-exportarlo desde un checkpoint:

```bash
cd training && python export_onnx.py --checkpoint checkpoints/phasemixedsp_final.zip --out models/policy.onnx
```

### Contrato discreto de skills (CONGELADO)

`skill_id ∈ {GoTo=0, FacePoint=1, ChaseBall=2, Spin=3, PushBall=4}` — binning del
`skill_sel ∈ [-1,1]` en `SkillId::COUNT`=5, idéntico en Python
(`VsssSoccerEnv._bin_skill`) y Rust (`RlCoach::bin_skill`). Nunca renumerar ni
eliminar entradas; sólo agregar al final (cambia el mapeo y obliga a re-fine-tune).

### Arquero entrenado (deploy)

`RlCoach` carga `policy_gk.onnx` (Box 3) y lo corre para el robot id 2 con la
misma observación de 52 floats. Override con `VSSL_RL_GK_MODEL`; `none` vuelve al
arquero rule-based. Para re-exportarlo desde el checkpoint:

```bash
cd training && python export_onnx.py --checkpoint checkpoints/phasegk_final.zip --out models/policy_gk.onnx
```

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

Toda esta topología vive en `run_control_loop` (`src/control_loop.rs`) y la invocan los 3 binarios:

```
Tokio runtime (un solo loop común para main, scenario y skill_test):
  ├─ vision_task        (event-driven) UDP multicast → World
  ├─ world_updater      (100 ms)       marca robots inactivos
  ├─ vision_watchdog    (opcional)     aborta si --vision real no recibe paquetes en N s
  └─ control_loop       (16 ms, 60 Hz) TickDecider → SkillCatalog::tick → Radio → transport
```

Estado compartido: `Arc<TokioRwLock<World>>`. Comunicación inter-task: canales `mpsc`.

---

## Tests

```bash
cargo test                       # toda la suite
cargo test --lib                 # solo lib (132 tests)
cargo test --bin scenario        # constructores de Scenario (6 tests)
cargo test --bin skill_test      # parser del CLI (15 tests)
```

**153 tests** cubriendo: UVF, motion, PID, Environment, radio (cinemática inversa + frames + golden tests del contrato base station), skills (catálogo congelado), observation/coach, world, tracker, vision, control_loop (FixedSkillDecider, CoachDecider frame-skip), skill_log (CsvLogger + row-builder compartido).

### Plotting de runs (`tools/plot_run.py`)

Convierte el CSV de un run (`scenario` con `log = Some(...)` o `skill_test --log`) en 4 paneles: trayectoria pose vs target, errores en el tiempo, velocidades de rueda L/R, y comando (vx, vy, omega).

```bash
python tools/plot_run.py logs/scenario_go_to_<epoch>.csv            # abre ventana
python tools/plot_run.py logs/run.csv --save run.png                # guarda PNG y muestra
python tools/plot_run.py logs/run.csv --save run.png --no-show      # solo guarda (headless / WSL)
```

Requiere `matplotlib` (`pip install matplotlib`). Sin `matplotlib` funciona el resumen de texto que imprime stats por columna.

### Troubleshooting

| Síntoma | Causa probable | Fix |
|---|---|---|
| `[control_loop] radio error: No such file or directory` | Base station no enchufada o `VSSL_BASESTATION_DEVICE` mal | `ls /dev/ttyUSB* /dev/ttyACM*`; exportar la ruta correcta |
| `[control_loop] radio error: Permission denied` | Usuario no está en grupo `dialout` | `sudo usermod -aG dialout $USER` + re-login, o `sudo chmod 666 /dev/ttyUSBX` |
| `[Vision] Sin paquetes` con `sslvision` | Publisher caído, grupo/puerto equivocado, o multicast en interfaz que no es | Verificar con `sudo tcpdump -ni any udp port 10015`; si llega pero el Rust no ve, probar `VSSL_MULTICAST_IFACE=<ip_local>` |
| Robots detectados en GUI pero no se mueven | `MI_ROBOT_ID` (firmware) no matchea el id que la visión asigna; o `#define MODO_BASESTATION` comentado en firmware | Confirmar IDs en GUI y en `config.h`. Probar bring-up directo: `cargo run --bin skill_test --release -- --transport base-station --mode wheels --team blue --robot 0 --left 500 --right 500 --dur 1` |
| Frame `?,?,?,?,...` en consola pero robot inerte | `#define MODO_BASESTATION` está comentado → firmware compila en modo BLE/RemoteXY y no escucha ESP-NOW | Descomentar línea en `VSSL-firmware/include/config.h:10` y reflashear |
