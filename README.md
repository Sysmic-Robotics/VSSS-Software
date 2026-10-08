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
| `VSSL_TEAM_COLOR` | `blue` | `blue` o `yellow`. Equipo que controla el engine (coach, skills) y, en `basestation`, qué comandos van al frame serial. Permite dos engines en la misma máquina, uno por equipo. |
| `VSSL_SIDE` | azul `left`, amarillo `right` | Arco que defendemos (`left` = atacamos hacia +X). Cambia en el segundo tiempo y según cómo esté puesto el simulador; el coach avisa al arrancar si los robots están en la mitad contraria. |
| `VSSL_COACH` | `heuristic` | `heuristic` (equipo STP de dos caras), `rule_based` (baseline de roles fijos) o `none`. |
| `VSSL_BIDIRECTIONAL` | (off) | `1`: heading módulo 180° — el robot usa la cara (frente/espalda) que requiera menos giro y avanza de espaldas cuando conviene. |
| `VSSL_BORDER_RECOVERY` | (on) | `0`: apaga la recuperación de atasco. Hay atasco si durante 30 ticks se le comanda avanzar (> 0.1 m/s a lo largo del heading) y todas sus poses medidas quedan a menos de 0.03 m de la primera de la ventana (un salto de la pose estimada, como el sobrepaso del estimador al chocar la pelota, cuenta como movimiento); entonces hace una maniobra de escape de 25 ticks. Si la racha de avance arranca desde el reposo (las últimas 30 poses a menos de 0.01 m de la más vieja, aunque gire en el lugar, o el robot recién aparece), los primeros 30 ticks no cuentan: al alinearse y acelerar, la latencia y el torque limitado le impiden recorrer 3 cm en 0.5 s. Un robot trabado desde quieto escapa a los 60 ticks; uno que venía andando, a los 30. |
| `VSSL_TRACKER` | (on) | `off`: arranca con el EKF apagado (poses crudas de visión; para medir ruido de cámara). |
| `VSSL_MATCH_LOG` | (off) | Ruta de un CSV de partido: una fila por robot y tick (skill, target, pose, comando, pelota). |
| `VSSL_VISION_NOISE` | (off) | `1`: proxy de ruido de cámara en el simulador (σ, latencia, pérdida de frames de `vision.proxy_*`). Regla: nada se acepta en sim limpio. |
| `VSSL_VISION_RECORD` | (off) | Ruta donde grabar los paquetes crudos de visión, para `vision_replay`. |
| `VSSL_REFEREE_ADDR` | `224.5.23.2:10003` | Dónde escucha el engine los comandos del árbitro (VSSReferee o texto de `tools/referee_cli.py`). |
| `VSSL_BASESTATION_DEVICE` | `/dev/ttyUSB0` | Path del puerto serial a la base station. |
| `VSSL_BASESTATION_BAUD` | `115200` | Baudrate del enlace USB↔ESP32 base station. |

---

## Comandos rápidos

```bash
cargo build --release                # build optimizado (necesario para tiempo real)
cargo run --release                  # main headless (coach decide) — default FIRASim
VSSL_DEBUG_GUI=1 cargo run --release # main con GUI Iced
cargo run --bin scenario --release   # banco "editar y correr" para 1 skill, GUI siempre on, CSV opcional a logs/
cargo run --bin skill_test --release -- --help   # probador CLI (modo skill o vw)
cargo test                           # suite completa (lib + bin + doctests)
cargo clippy                         # lint
cargo fmt                            # formato
```

### GUI de debug (`VSSL_DEBUG_GUI=1`)

Layout: **barra superior** (botón STOP + estado de conexión), **sidebar izquierdo con
secciones colapsables** (Control, Skills, Inspector, Tuning, Teleport, Radio, Visión, Telemetría),
**cancha central**, y
**barra de estado** inferior (robot/equipo, θ, PPS, ESTOP). Cada sección se expande/colapsa
con su encabezado.

La sección **Control** permite **manejar un robot a mano**, útil para el bring-up del robot
real (ver `docs/bringup_robot_real.md`):

- **Manual: ON/OFF** activa el control manual del robot/equipo seleccionados.
- Selector de **Robot** (`-`/`+`) y de **Equipo** (Azul/Amarillo).
- **Marco de referencia** (toggle Mundo/Robot):
  - **Robot** (default, arcade drive): **W/S** = adelante/atrás según hacia dónde mira el
    robot, **A/D** = giro. Usa la orientación de visión del robot seleccionado.
  - **Mundo**: **W/S** = ±Y de la cancha, **A/D** = ±X, **Q/E** = giro CCW/CW.
- **Escalas ajustables** en la GUI: velocidad lineal máx (m/s) y angular máx (rad/s).
- **Rampa de aceleración**: el comando arranca y frena suave (no salta de 0 a máximo),
  evitando tirones/patinaje. Al soltar las teclas decae a cero por la misma rampa.
- El comando pasa por la MISMA cinemática inversa y el mismo `RobotTransport` que el coach
  (no hay ruta paralela): lo que ves en el chart L/R es lo que se envía.
- El robot seleccionado se resalta con un halo naranjo, y el chart inferior grafica sus
  velocidades de rueda comandadas **L/R (mm/s)** en el tiempo.

**Parada de emergencia:** el botón rojo **STOP** (o la tecla **Espacio**) enclava una
parada que comanda velocidad cero a todos los ids del equipo propio, aunque la visión no
los vea (así siempre sale un frame en cero), y prevalece sobre coach, control manual y
skills. Mientras está activa no actúan el `ZoneGuard` ni la recuperación de atasco. Al
liberarla con el mismo botón, cada robot retoma sin estado heredado: sin la maniobra de
escape interrumpida, con la rampa desde cero y la skill reiniciada. Es el mismo camino que
toma el loop en HALT del árbitro. El watchdog del firmware (200 ms) queda como red final.

**Runner de skills (validador visual):** en la fila de control eliges una de las 4 skills
(`GoTo`, `FacePoint`, `ChaseBall`, `Spin`) o **Ninguna**, y haces **click en la cancha**
para fijar el target (marcador verde). La skill se inyecta como `SkillChoice` al mismo
`SkillCatalog::tick` que el coach — reemplaza la decisión del coach solo para ese robot.
Precedencia: control manual (velocidad cruda) > skill de GUI > coach. Con "Ninguna" el robot
vuelve al coach.

El número de robot que se elige en la GUI es el **id de visión** (el del parche de colores).
Una skill solo corre si ese robot es del equipo propio y está activo en el `World` (lo ve la
visión). Si no, la sección Skills muestra `⚠ la skill no corre: el robot azul #1 no está en la
visión. La visión ve: azul #0, amarillo #2`, y la terminal imprime lo mismo, como máximo una
vez por segundo. Si el robot aparece con otro número o color, hay que corregir la selección o
el mapa `robot.radio_slot_by_vision_id`. El control manual no tiene esta restricción: manda
aunque la visión no vea al robot.

La sección **Inspector** muestra los datos de visión del robot seleccionado: posición, θ,
rapidez (m/s), ω (rad/s), estado activo/inactivo y antigüedad del último dato. En la cancha,
overlays de diagnóstico: **vector de velocidad medida** (naranja, distinto de la flecha blanca
de velocidad comandada), **número de robot** y **traza** del recorrido reciente del seleccionado (activable/desactivable en Inspector).

La sección **Tuning** ajusta en vivo (sin recompilar): escalas de velocidad, rampa, **Spin ω**
y el **PID de heading** (`kp/ki/kd`, que gobierna el giro de GoTo/FacePoint/ChaseBall — afecta
también al coach). Además guarda/carga **presets** de todo el tuning en un archivo JSON.

La sección **Teleport (sim)** reposiciona el robot seleccionado a `(x, y, θ)` y la pelota a
`(x, y)` en el simulador (FIRASim/grSim) — útil para armar situaciones reproducibles de test.
`x, y` van en metros y **θ en grados** (la unidad del replacement de los dos simuladores).
En base station (robots reales) es no-op.

La sección **Radio** muestra el transporte activo, puerto/baud (base station), equipo propio,
estado de conexión, PPS y, con base station, el **mapeo visión → radio** vigente (por ejemplo
`visión #1 → radio pos 0 (MI_ROBOT_ID 1)`). Editar puerto/baud requiere reiniciar el proceso (se aplican por
`VSSL_BASESTATION_DEVICE` / `VSSL_BASESTATION_BAUD`). En headless nada de esto aplica: la
config sigue viniendo del entorno y los bytes enviados son idénticos.

**3 binarios:**
- `rustengine` (default): producción headless o con `VSSL_DEBUG_GUI=1`. Coach decide qué skill correr.
- `scenario`: banco de pruebas tipo "editar y correr". Una skill a la vez, configurada como constantes en la zona de edición al inicio de `src/bin/scenario.rs`. GUI siempre activa. CSV opcional a `logs/scenario_<skill>_<epoch>.csv` (toggle con la constante `log: Option<PathBuf>`: `Some(scenario_log_path(&scenario))` escribe, `None` desactiva el archivo y deja solo el resumen humano a stderr).
- `skill_test`: probador CLI. Modo `skill` (lazo cerrado, sim o real) y modo `vw` (lazo abierto: v en mm/s y ω en °/s directos al robot real, para bring-up). Ver `--help`.

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

**Línea de lanzamiento del laboratorio** (visión real + base station + GUI, robots de dos caras):

```bash
VSSL_VISION_SOURCE=sslvision VSSL_RADIO_TARGET=basestation VSSL_BASESTATION_DEVICE=/dev/ttyUSB0 \
VSSL_DEBUG_GUI=1 VSSL_BIDIRECTIONAL=1 cargo run --release
```

Al arrancar, confirmar en la terminal:
- `[Vision] orientación de la visión real corregida en +180°`;
- el mapeo `[BaseStation] visión #N → radio pos …`.

En la GUI, el robot se elige por su **id de visión**. Si una skill no mueve al robot, la sección Skills dice por qué: el robot no está en la visión, o es de otro equipo, y muestra qué robots sí ve la visión.

Variantes:

```bash
# Equipo amarillo.
VSSL_VISION_SOURCE=sslvision VSSL_RADIO_TARGET=basestation VSSL_TEAM_COLOR=yellow cargo run --release

# Partido en FIRASim: dos engines en la misma máquina (el socket de visión se comparte).
# Terminal 1 — azul, coach heurístico, con GUI y registro:
VSSL_TEAM_COLOR=blue VSSL_BIDIRECTIONAL=1 VSSL_DEBUG_GUI=1 VSSL_MATCH_LOG=logs/partido_azul.csv cargo run --release
# Terminal 2 — amarillo, baseline de regresión:
VSSL_TEAM_COLOR=yellow VSSL_COACH=rule_based VSSL_MATCH_LOG=logs/partido_amarillo.csv cargo run --release

# Visión real, comandos a FIRASim (debug visual: ver qué decide el engine sin mover los robots).
VSSL_VISION_SOURCE=sslvision cargo run --release

# Bring-up de hardware: mandar (v mm/s, ω °/s) directos a un robot sin pasar por visión ni skills.
# Útil para diagnosticar signos de rueda, giroscopio, comunicación y unidades.
# Encender el robot QUIETO (calibra el gyro). --robot 1 = robot con MI_ROBOT_ID 2.
# Qué significa cada falla: ver `skill_test --help` (SECUENCIA DE BRING-UP).
cargo run --bin skill_test --release -- --transport base-station --mode vw --team blue --robot 1 --v 300  --w 0   --dur 2   # avanza recto
cargo run --bin skill_test --release -- --transport base-station --mode vw --team blue --robot 1 --v -300 --w 0   --dur 2   # retrocede recto
cargo run --bin skill_test --release -- --transport base-station --mode vw --team blue --robot 1 --v 0    --w 90  --dur 2   # gira antihorario (~1 vuelta/4 s)
cargo run --bin skill_test --release -- --transport base-station --mode vw --team blue --robot 1 --v 0    --w -90 --dur 2   # gira horario
cargo run --bin skill_test --release -- --transport base-station --mode vw --team blue --robot 1 --v 300  --w 45  --dur 2   # arco hacia la izquierda (radio ≈ 0.38 m)
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
                         └─ BaseStationTransport → (v, ω) proyectados al heading
                                                   → ASCII "V,W" (mm/s, °/s) por USB serial
                                                   → ESP32 base → ESP-NOW → robots
                                                     (el firmware reparte a las ruedas)
```

### Módulos principales

| Módulo | Función |
|--------|---------|
| `vision.rs` | Receptor UDP multicast; parsea FIRA o SSL_WrapperPacket según `VSSL_VISION_SOURCE`; emite `VisionEvent` |
| `tracker/` | EKF por entidad (robot/balón). Estado: posición + orientación + velocidades |
| `world/` | Estado canónico del juego: poses de robots, posición/velocidad del balón, flags de inactividad |
| `coach/` | Coach trait + `RuleBasedCoach` baseline + contrato `Observation` (52 floats) para el modelo RL futuro |
| `skills/` | Catálogo congelado RL: `SkillId::{GoTo, FacePoint, ChaseBall, Spin}` (`SkillCatalog::tick`). Skills out-of-catalog viven en `skills/mod.rs` para otros usos |
| `motion/` | UVF para evasión de robots, rodeo de la pelota y de las áreas por la tangente, seguimiento de heading para diferencial con rampa de aceleración, PID de orientación al llegar, recuperación de atasco |
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
│   └── skill_test.rs      # Probador CLI: --mode skill|vw (bring-up real)
├── world/                 # World, RobotState, BallState (Arc<RwLock>)
├── tracker/               # EKF por entidad
├── coach/                 # Coach trait + RuleBasedCoach + Observation (52 floats RL)
├── skills/                # SkillCatalog congelado (GoTo, FacePoint, ChaseBall, Spin) + skills out-of-catalog
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

## Parámetros calibrables (`config/team_params.json`)

Todo lo que se ajusta con mediciones o partidos de prueba vive en `config/team_params.json`
(`src/params.rs`), no en el código: umbrales del coach (`coach`), distancias de las skills
tácticas (`skills`), navegación (`motion`), geometría del robot real (`robot`: wheelbase, tope de
rueda) y del robot de FIRASim (`sim`). Los tres binarios lo cargan al arrancar y lo anuncian en el
log (`[main] parámetros: archivo config/team_params.json`).

- `VSSL_PARAMS=<ruta>` usa otro archivo (útil para comparar dos calibraciones en el mismo partido).
- El JSON puede ser parcial: lo que falta toma el default. Un campo con nombre desconocido es
  error y el proceso no arranca (evita que un typo en una calibración pase en silencio).
- Cambiar un parámetro NO requiere recompilar: editar el JSON y volver a lanzar.

## Percepción: EKF calibrable, proxy de ruido y replay

La derrota pasada fue de percepción (ruido de cámara + EKF mal tuneado), así que la cadena completa
es calibrable y verificable con datos reales:

- **EKF desde el JSON.** Q, R, gating y límites físicos del tracker viven en `config/team_params.json`
  → `vision` (`r_pos`, `r_theta`, `q_*`, `gating_chi2`, ...). Sin recompilar.
- **Orientación de la visión real.** vsss-vision-sysmic entrega el heading de los robots girado 180°
  (lo que llama "adelante" es la espalda). `vision.real_theta_offset_deg` se suma a la orientación
  de cada robot y se normaliza a [-π, π], **solo** con `VSSL_VISION_SOURCE=sslvision`, al parsear el
  paquete: antes del EKF, de la GUI y del `World`. FIRASim no se toca. Default en código `0`; en el
  JSON del repo `180`. Al arrancar, el engine imprime `[Vision] orientación de la visión real
  corregida en +180°`. Si algún día se corrige en la visión, hay que volver el JSON a `0`. El
  replay offline (`vision_replay --ekf-csv`) aplica la misma corrección.
- **Proxy de ruido en el simulador.** `VSSL_VISION_NOISE=1` inyecta, ANTES del tracker, el ruido
  de `vision.proxy_*` (σ de posición y orientación, latencia como retención de paquetes, pérdida de
  frames). Los defaults son de literatura; la medición M1 los reemplaza por los de nuestra cámara.
- **Grabación y replay.** `VSSL_VISION_RECORD=logs/vision.bin` guarda los paquetes crudos. Luego:

```bash
# Re-publicar la grabación en el multicast (el engine la consume como si fuera en vivo):
cargo run --release --bin vision_replay -- --file logs/vision.bin --publish --loop

# Pasar la grabación por el EKF offline y comparar dos calibraciones sobre los MISMOS datos:
cargo run --release --bin vision_replay -- --file logs/vision.bin --ekf-csv logs/ekf_a.csv
cargo run --release --bin vision_replay -- --file logs/vision.bin --ekf-csv logs/ekf_b.csv --params config/otro.json
```

El modo `--ekf-csv` imprime por entidad el RMS del residuo crudo−filtrado; con el robot quieto ese
RMS es la σ de la cámara (base de M1). Protocolo de torneo: 15 min de grabación al llegar → replay
→ confirmar Q/R → jugar.

## Árbitro y pelota parada (`src/coach/referee.rs`, `src/coach/plays.rs`)

El coach heurístico lee el estado de juego del árbitro y ejecuta las formaciones del reglamento
LARC 2026 (§6.6 y §10) antes del silbato: kickoff, free ball (cruz del cuadrante, robot en el punto a
20 cm del lado propio), penal, free kick (defensores tocando la línea del área, uno por cuadrante) y
goal kick (lo saca el arquero). Con `GAME_ON` vuelve la táctica normal de inmediato. El pateador
queda colocado en el staging de `ShootPush`, así empuja en el primer tick tras el silbato.

`HALT` y `STOP` los aplica el **control loop** en el mismo tick, con cualquier coach (o sin coach),
leyendo el estado del árbitro (`ControlLoopConfig.referee`):

- **HALT:** el mismo camino que la parada de emergencia. Cero a todos los ids propios, sin
  decider, skills, `ZoneGuard` ni recuperación de atasco.
- **STOP** (§9.5, "mantener y solo corregir orientación"): las skills siguen corriendo, pero la
  traslación es cero y se conserva su `omega`. No actúan el `ZoneGuard` ni la recuperación. Con el
  coach heurístico, en STOP las skills son `Hold` y los robots quedan quietos del todo.
- Al volver a `GAME_ON`, cada robot retoma sin estado heredado (sin escape interrumpido, rampa
  desde cero, skill reiniciada). La terminal avisa al entrar y al salir de HALT.

Fuentes, por el mismo puerto (`VSSL_REFEREE_ADDR`): **VSSReferee** (simulador, protobuf
`VSSRef_Command`) o **texto del operador** para el árbitro humano de la cancha:

```bash
python tools/referee_cli.py            # interactivo: k b (kickoff azul), b 1 (free ball Q1), go, s, h
python tools/referee_cli.py penalty yellow
```

## Métricas de partido (`tools/match_metrics.py`)

Con `VSSL_MATCH_LOG=logs/partido.csv` el engine escribe el registro del partido; el script lo
convierte en números (solo biblioteca estándar de Python):

```bash
python tools/match_metrics.py logs/partido_azul.csv            # tabla
python tools/match_metrics.py logs/a.csv logs/b.csv            # comparar corridas
python tools/match_metrics.py logs/partido_azul.csv --json     # para scripts
```

Reporta goles a favor/en contra, posesión, toques por robot, distancia media a la pelota, tiempo
de pelota en pared/esquina, cambios de striker por minuto, uso de cada skill y el porcentaje de
ticks en que un robot recibe comando de avance pero no se mueve (atascado).

## Parámetros clave (`src/main.rs`)

| Constante | Valor | Descripción |
|-----------|-------|-------------|
| (env) `VSSL_TEAM_COLOR` | `blue` | Equipo controlado por el binario principal (antes era la constante `OWN_TEAM`) |
| `NUM_ROBOTS` | `3` | Cantidad de slots de `SkillCatalog` (un slot por robot del equipo propio) |
| `COACH_DECISION_PERIOD` | `6` | Frame-skip del coach: decide cada 6 ticks (10 Hz) a 60 Hz de control |

`main` arma un `CoachDecider` con esos parámetros y delega TODO el lazo en `run_control_loop`. Los demás binarios (`scenario`, `skill_test`) corren el mismo loop con un decisor distinto.

### Parámetros de movimiento (sección `motion` de `config/team_params.json`)

`move_and_face` sigue el heading como un uniciclo: `ω = heading_gain · e` hacia la dirección del UVF y avance `v = v_perfil · max(cos e, 0)` a lo largo del heading (en bidireccional el error se pliega a ±90° y el avance va de espaldas). El `face_target` se usa recién al llegar. El comando sale paralelo al heading, así que el diferencial lo ejecuta sin pérdida.

| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `max_linear_speed` | `1.2 m/s` | Velocidad máxima lineal |
| `max_angular_speed` | `3.0 rad/s` | Velocidad angular máxima |
| `brake_distance` | `0.50 m` | Distancia al destino desde la que empieza el frenado |
| `arrival_threshold` | `0.04 m` | Radio de llegada. Debe ser menor que `skills.approach_pos_tol`: la carga lo valida |
| `heading_gain` | `3.0` | Ganancia del seguimiento de heading (rad/s por rad) |
| `max_linear_accel` | `1.0 m/s²` | Rampa del avance: cambio máximo por tick sobre el comando anterior (no usa la velocidad medida) |

`heading_gain`, `max_linear_accel` y `max_angular_speed` se calibraron en FIRASim con el proxy de ruido (90 ms de latencia, ~1.2 m/s² de aceleración física). **Hay que recalibrarlos con la sysid del robot real.**

**`motion.coupling_floor` ya no existe.** Como los parámetros rechazan campos desconocidos, un JSON local que todavía tenga la clave **deja de cargar** con error: hay que borrarla.

El estado de control de cada robot (PID de heading y rampa) se reinicia cuando el robot cambia de skill.

### Parámetros del Univector Field (`src/motion/uvf.rs`)

| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `influence_radius` | `0.20 m` | Radio de influencia de obstáculos |
| `k_rep` | `1.5` | Ganancia repulsiva tangencial |

El UVF solo desvía por los **robots** que están por delante respecto del destino. No hay paredes virtuales: el borde lo cubren el `ZoneGuard` (áreas) y la recuperación de atasco.

**Pelota y áreas: rodeo por la tangente (`src/motion/avoid.rs`).** Si el segmento robot → destino toca la pelota inflada (0.09 m) o un área prohibida para ese robot (rectángulo + arco, con medio robot + 0.06 m, la misma holgura con que BlockLine y Clear eligen sus puntos), el robot apunta a la tangente del obstáculo del lado más cercano al destino:
- no vale un lado cuya tangencia sale de la cancha o cuyo camino toca otro obstáculo (la pelota pegada al arco se rodea por fuera; un área nunca se rodea por detrás);
- el lado elegido se mantiene mientras siga valiendo;
- no se rodea la pelota si el destino está a su holgura (contacto), ni un área si el destino la toca (ilegal: lo frena el guardia); un destino legal más cerca del área que la holgura se rodea a su propia distancia.

Las áreas son las mismas que frena el guardia: la propia para todos menos el arquero, y la rival cuando otro robot propio ya la toca. El guardia no cambia y sigue como restricción dura. `move_and_face` (movimiento de posición) rodea la pelota; `move_and_face_contact` (ChaseBall, Intercept, el empuje de Clear, GoalKeep) no. Clear gira su dirección de despeje lo mínimo para que el staging quede fuera de las áreas prohibidas (el arquero no gira). Las holguras se revisan con la latencia real (sysid).

El `ZoneGuard` cuenta como área el rectángulo de 70 × 15 cm **más el arco del reglamento** sobre su frente (cuerda de 20 cm, flecha de 5 cm), con margen de medio robot. Es más conservador que VSSReferee, que cuenta solo el rectángulo y por el centro del robot; el auditor de faltas sigue con el criterio de VSSReferee.

### Parámetros de las skills tácticas (`skills` en `config/team_params.json`)

| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `shoot_lateral_tol` | `0.05 m` | ShootPush, Clear y el coach empujan solo con el robot a esta distancia lateral (o menos) de la recta de empuje: de costado, la pelota sale de lado. Debe ser ≥ `approach_pos_tol`: la carga lo valida |
| `spin_pos_tol` | `0.05 m` | A esta distancia del punto de contacto SpinKick deja de navegar a él y cierra sobre la pelota; gira solo con la pelota al alcance de las esquinas. Debe ser mayor que `motion.arrival_threshold`: la carga lo valida |
| `block_distance` | `0.30 m` | Distancia del punto de bloqueo al arco propio. Si cae en el área, BlockLine lo corre hacia afuera por la misma línea (mismo criterio que el `ZoneGuard`) |

**`skills.block_max_abs_x` ya no existe**: un JSON local que todavía tenga la clave **deja de cargar** con error, hay que borrarla. Un JSON con `spin_pos_tol ≤ arrival_threshold` o con `shoot_lateral_tol < approach_pos_tol` tampoco carga.

Al cambiar de skill, el catálogo reinicia el estado interno de la skill que se activa (`Skill::reset`; hoy solo SpinKick guarda estado).

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

`BaseStationTransport` envía a la base ESP32 la consigna **(v, ω)** de cada robot. La base vigente es `VSSL-firmware/test/base_station_lineal_angulo.ino` (rama `Peluche` del firmware). Reenvía un binario de 24 bytes por ESP-NOW cada 50 ms, y el **firmware** hace la cinemática diferencial (`WHEEL_TRACK_MM`), corrige ω con el giroscopio y cierra el PID de cada rueda. Contrato verificado en `docs/contrato_comunicacion_v2.md`.

| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `robot.max_v_mm_s` (`config/team_params.json`) | `1500` | Tope de v, el mismo clamp que aplica la base (`MAX_V_MM_S`) |
| `robot.max_w_deg_s` (`config/team_params.json`) | `720` | Tope de ω en grados/s, el mismo clamp que aplica la base (`MAX_W_DEG_S`) |
| `SLOT_COUNT` | `5` | Slots del frame ASCII; el robot físico con `MI_ROBOT_ID = N` (firmware, desde 1) lee `slots[N-1]`. El robot del checkout actual es `MI_ROBOT_ID 2` → slot 1 |
| `robot.radio_slot_by_vision_id` (`config/team_params.json`) | `{}` | Mapa id de visión → posición del frame (ver abajo) |

**Ids de visión y de radio.** Cada robot real tiene dos ids:
- **id de visión:** el del parche de colores. Con él aparece en el `World` y en la GUI, y es el número que se elige en la GUI y en `skill_test --mode skill --robot`.
- **id de radio:** `MI_ROBOT_ID` del firmware. El robot escucha la posición `MI_ROBOT_ID − 1` del frame.

Si no calzan, se declara el mapa en `robot.radio_slot_by_vision_id` como `{"<id de visión>": <posición>}`. Por ejemplo, `{"0": 1, "1": 0}`: la visión ve el robot como #1, pero ese robot tiene `MI_ROBOT_ID 1`, es decir, la posición 0. Un id sin entrada usa su propio número. Así, el manual, las skills y el coach le llegan al robot correcto.

Reglas, validadas al cargar los params (si no se cumplen, el proceso no arranca y el error dice qué corregir):
- toda posición está en `0..4`;
- dos ids de visión nunca caen en la misma posición, **contando los ids sin entrada**. Por eso `{"1": 0}` solo no vale: el #0 seguiría yendo a la posición 0. Hay que mapear también el #0 a una posición libre.

El mapa solo afecta a la base station; FIRASim y grSim usan el id directo. Al abrir la base, la terminal imprime el mapeo vigente (`[BaseStation] visión #1 → radio pos 0 (MI_ROBOT_ID 1)`), y la sección Radio de la GUI lo muestra. `skill_test --mode vw --robot N` **no** pasa por el mapa: ahí N es la posición de radio cruda (bring-up sin visión).

**Conversión** (en `command_to_vw`, única fuente de verdad del frame, el CSV y la GUI):
```
v [mm/s] = round((vx·cos(orientation) + vy·sin(orientation)) · 1000)   // proyección al heading
w [°/s]  = round(omega · 180/π)
```
Ambos se recortan a los topes de arriba. Convención: `omega > 0` → `w > 0` → antihorario visto desde arriba (en el firmware, rueda derecha más rápida; coincide con la convención `Spin`).

**Tope por rueda del firmware:** si (v, ω) le pide más de 450 mm/s a una rueda (`MAX_WHEEL_MM_S`), el firmware escala **ambas ruedas** por el mismo factor: conserva la curva y baja la velocidad. El PC no replica ese tope, así que el robot puede ejecutar menos de lo enviado (v ≤ 0.45 m/s en recta, ω ≤ ~12 rad/s en giro puro).

**Frame serial:** `"V1,W1,V2,W2,V3,W3,V4,W4,V5,W5\n"` (enteros decimales, terminador `\n`), 115200 baud. Solo se incluyen comandos del equipo propio (`VSSL_TEAM_COLOR`). La base descarta una línea con decimales y sigue enviando el comando anterior. **No tiene timeout de serial**: si el PC deja de mandar, repite lo último; el watchdog de 200 ms del robot solo cubre la pérdida de radio.

**NaN/Inf safety:** si cualquier campo del `MotionCommand` no es finito, `command_to_vw` devuelve `(0, 0)` sin pánico.

**Bring-up directo** (sin skills): `BaseStationTransport::send_raw_vw_frame(slots)` o el binario `skill_test --mode vw --v MM_S --w DEG_S`.

---

## Integración del modelo RL

El engine está diseñado para recibir un modelo de RL con cambios mínimos. El seam de integración es el trait `Coach` en `src/coach/coach_trait.rs`.

### Cómo conectar el modelo (cuando esté listo)

**1.** Crear `src/coach/rl_coach.rs`:

```rust
use crate::coach::{Coach, Observation, SkillChoice};
use crate::skills::SkillId;

pub struct RlCoach { /* model handle */ }

impl RlCoach {
    pub fn load(path: &str) -> Self { ... }
}

impl Coach for RlCoach {
    fn decide(&mut self, obs: &Observation) -> Vec<SkillChoice> {
        let input = obs.to_flat_vec(); // 52 floats — ver contrato abajo
        // inferencia → Vec<SkillChoice { robot_id, skill_id, target }>
        // skill_id ∈ {GoTo=0, FacePoint=1, ChaseBall=2, Spin=3} (catálogo congelado)
    }
}
```

**2.** En `make_coach` (`src/main.rs`) agregar la rama `VSSL_COACH=rl` que carga `RlCoach::load(...)`. El `CoachDecider` ya envuelve cualquier `Box<dyn Coach>` con el frame-skip de 10 Hz, sin más cambios.

El contrato discreto es **`SkillId` con orden congelado**: no renumerar ni eliminar entradas, solo agregar al final.

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

## Aceptación de motion (`motion_bench`)

`motion_bench` corre casos fijos (GoTo, FacePoint, ChaseBall, Mark, Intercept, ApproachAligned, BlockLine, entradas al área y casos de skills) contra FIRASim, por el mismo `run_control_loop` que `main`. Cada caso es un guion de una o más fases (skill, target, duración) y tiene un criterio de éxito (llegar, no tocar el área, patear hacia el objetivo, no empujar de costado, arquero en su punto mirando la pelota). Imprime una línea JSON por repetición, un resumen de los 17 casos de navegación (llegadas, giro acumulado hasta llegar, ticks de escape y tiempo medio, sobre la repetición 0) y la tasa de éxito por caso. `b2_blockline_open` se informa aparte. En los casos de GoalKeep el robot de prueba es el arquero (`coach.keeper_id`). Se configura con las mismas variables que producción.

- `skills` corre los 11 casos de los bugs de skills; `all`, todos.
- `suite` corre la **suite de regresión de las 13 skills** (32 casos, al menos uno con criterio por skill) y `suite-rapida`, un caso por skill (13). Al final se imprime la tasa por caso y **por skill**. Criterios de las skills que no tenían uno: Spin (gira en el sentido pedido a ≥ 80 % de `spin_omega` sin trasladarse), Hold (comando cero y quieto), Mark (llega mirando a la pelota), Intercept con la pelota en movimiento (el teleport acepta velocidad de la pelota) y GoTo con la pelota en el camino (llega sin moverla: pendiente A4).
- `--repeat N`: N repeticiones por caso; la 0 con la pose del caso y las demás con un jitter determinista (±0.02 m, ±10°).
- `--out FILE`: agrega cada repetición a un JSONL y, al relanzar con el mismo archivo, salta las que ya tienen resultado (para reanudar si FIRASim se cae).
- Una repetición con FIRASim caído, sin datos del robot o con la física explotada (salto de pose de más de 0.3 m entre ticks) es **falla de infraestructura**: no cuenta en la tasa, se reintenta hasta 2 veces y, si no se recupera, el binario sale con código 3. El resumen informa cuántas se perdieron.
- `--csv-dir DIR` escribe un CSV por tick y por repetición (`<caso><tag>_r<N>.csv`); con `--csv-failures-only`, solo de las que fallan.
- `referee` corre los casos del árbitro (`halt_game_on`, `halt_in_area`, `stop_in_area`). El bench lanza el listener real y le manda `HALT`, `STOP` y `GAME_ON` como texto por UDP a `VSSL_REFEREE_ADDR` (o al grupo multicast por defecto). Si en la máquina no funciona el loopback de multicast, usar `VSSL_REFEREE_ADDR=127.0.0.1:10003`.

```bash
# FIRASim corriendo. Regla del equipo: nada se acepta en sim limpio, siempre con el proxy de ruido.
VSSL_VISION_NOISE=1 cargo run --release --bin motion_bench -- all
VSSL_VISION_NOISE=1 VSSL_BIDIRECTIONAL=1 cargo run --release --bin motion_bench -- all --csv-dir /tmp/mb --tag _bd1
VSSL_VISION_NOISE=1 cargo run --release --bin motion_bench -- skills --repeat 20 --out r.jsonl --csv-dir csv --csv-failures-only
cargo run --release --bin motion_bench -- --list
```

### Suite de regresión de las 13 skills (`tools/suite_skills.sh`)

Un solo comando corre la suite en los dos modos, con el proxy de ruido y 20 repeticiones por caso:

```bash
# FIRASim corriendo; sin la GUI ni otro engine (comandan los mismos robots).
tools/suite_skills.sh rapida logs/suite                      # ~50 min, para cada change
tools/suite_skills.sh completa logs/suite                    # ~2 h, para hitos (motion, torneo, sysid)
REPEAT=10 FIRASIM_BIN=~/FIRASim/bin/FIRASim tools/suite_skills.sh rapida logs/suite
```

Guarda `bd0.jsonl`/`bd1.jsonl` (una línea por repetición), `bd0.txt`/`bd1.txt` (tasa por caso y por skill) y el CSV por tick solo de las repeticiones que fallan. Si FIRASim se cae, reanuda desde la repetición pendiente (lo relanza si `FIRASIM_BIN` está definido). Se niega a correr si hay otro engine abierto.

## Tests

```bash
cargo test                       # toda la suite
cargo test --lib                 # solo lib (369 tests)
cargo test --bin motion_bench    # métricas, criterios, suites y reanudación del bench de aceptación (23 tests)
cargo test --bin scenario        # constructores de Scenario (6 tests)
cargo test --bin skill_test      # parser del CLI (19 tests)
```

**417 tests** cubriendo: UVF, motion (ley de seguimiento de heading, rampa, y una planta diferencial de test con límite de aceleración por rueda y latencia), recuperación de atasco (con la gracia al arrancar desde el reposo), rodeo de la pelota y de las áreas por la tangente, ZoneGuard (con el arco del área), PID, Environment, radio (cinemática inversa + frames + golden tests del contrato base station + ruedas de FIRASim + teleport en grados), skills (catálogo, reinicio al cambiar de skill, BlockLine, ShootPush/Clear desde la línea de empuje, SpinKick, GoalKeep, ApproachAligned), observation/coach, world, tracker, vision, control_loop (FixedSkillDecider, CoachDecider frame-skip, orden de los reflejos, HALT/STOP del árbitro y parada de emergencia, frame de la base en cero), skill_log (CsvLogger + row-builder compartido).

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
| Robots detectados en GUI pero no se mueven | `MI_ROBOT_ID` (firmware) no matchea el id que la visión asigna; o `#define MODO_BASESTATION` comentado en firmware | Confirmar IDs en GUI y en `config.h`. Probar bring-up directo: `cargo run --bin skill_test --release -- --transport base-station --mode vw --team blue --robot 1 --v 300 --w 0 --dur 2` (`--robot` = `MI_ROBOT_ID` − 1) |
| Frame `?,?,?,?,...` en consola pero robot inerte | `#define MODO_BASESTATION` está comentado → firmware compila en modo BLE/RemoteXY y no escucha ESP-NOW | Descomentar línea en `VSSL-firmware/include/config.h:10` y reflashear |
