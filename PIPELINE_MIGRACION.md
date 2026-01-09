# Pipeline de Migración: Engine C++ → RustEngine

## Estado Actual del Proyecto Rust

**Completado:**
- ✅ Módulo Vision básico (recepción UDP, parsing Protobuf)
- ✅ GUI con Iced 0.13 (visualización del campo)
- ✅ Generación de Protobufs desde `.proto`

**Pendiente:**
- ❌ Tracker con Filtros Kalman (actualmente placeholder)
- ❌ Módulo World (modelo del mundo)
- ❌ Módulo Motion (control de movimiento)
- ❌ Módulo Radio (comunicación con robots)
- ❌ GameController (referee)
- ❌ Integración completa

---

## Fase 1: Completar Módulo Vision - Tracker con Filtros Kalman

### Objetivo
Implementar el sistema de tracking con Filtros Kalman Extendidos (EKF) para estimar velocidades de robots y balón desde posiciones medidas.

### Dependencias
- Módulo Vision básico (✅ completado)
- Biblioteca de álgebra lineal para Rust

### Tareas Detalladas

#### 1.1. Seleccionar e Integrar Biblioteca de Álgebra Lineal
**Prioridad:** Crítica  
**Estimación:** 2-4 horas

**Opciones:**
- `nalgebra` (recomendado): Similar a Eigen, ampliamente usado
- `ndarray`: Más simple pero menos funcionalidades
- `cgmath`: Ligera pero limitada

**Tareas:**
- [ ] Agregar `nalgebra` a `Cargo.toml`
- [ ] Probar operaciones básicas (matrices, vectores)
- [ ] Verificar compatibilidad con tipos existentes

**Criterio de éxito:**
- Compilación exitosa con `nalgebra`
- Tests básicos de operaciones matriciales pasando

---

#### 1.2. Implementar ExtendedKalmanFilter (EKF)
**Prioridad:** Crítica  
**Estimación:** 8-12 horas  
**Referencia C++:** `src/receivers/kalman/ekf.hpp` y `ekf.cpp`

**Estructura del estado:**
```rust
// Estado: [x, y, sin(θ), cos(θ), vx, vy, ω]
type StateVector = SVector<f64, 7>;
type CovarianceMatrix = SMatrix<f64, 7, 7>;
type MeasurementVector = SVector<f64, 3>; // [x, y, θ]
```

**Tareas:**
- [ ] Crear `src/tracker/ekf.rs`
- [ ] Implementar struct `ExtendedKalmanFilter` con:
  - [ ] Estado interno: `x_`, `P_`, `Q_`, `R_`
  - [ ] Método `predict(dt: f64)`
  - [ ] Método `update(measurement: &MeasurementVector)`
  - [ ] Método `filter_pose(x, y, theta, dt) -> (x, y, theta, vx, vy, omega)`
  - [ ] Funciones auxiliares:
    - [ ] `f(x, dt)` - Modelo de transición
    - [ ] `h(x)` - Modelo de observación
    - [ ] `jacobian_f(x, dt)` - Jacobiano de transición
    - [ ] `jacobian_h(x)` - Jacobiano de observación
    - [ ] `normalize_angle(angle)` - Normalización de ángulos

**Parámetros iniciales (del código C++):**
```rust
// Covarianza inicial P
P(0,0) = 1e-7;  // x
P(1,1) = 1e-7;  // y
P(2,2) = 1e-7;  // sin(theta)
P(3,3) = 1e-7;  // cos(theta)
P(4,4) = 1.0;   // vx
P(5,5) = 1.0;   // vy
P(6,6) = 1.0;   // omega

// Ruido de proceso Q
Q(0,0) = 1e-7;  // x
Q(1,1) = 1e-7;  // y
Q(2,2) = 1e-4;  // sin(theta)
Q(3,3) = 1e-4;  // cos(theta)
Q(4,4) = 1e-4;  // vx
Q(5,5) = 1e-4;  // vy
Q(6,6) = 1e-2;  // omega

// Ruido de medición R
R(0,0) = 1e-6;  // x
R(1,1) = 1e-6;  // y
R(2,2) = 1e-6;  // theta
```

**Criterio de éxito:**
- Tests unitarios pasando para cada método
- Comparación con resultados del código C++ (mismas entradas → mismas salidas)
- Rendimiento: procesar 1000 actualizaciones en < 10ms

---

#### 1.3. Implementar Tracker
**Prioridad:** Crítica  
**Estimación:** 4-6 horas  
**Referencia C++:** `src/receivers/tracker.hpp` y `tracker.cpp`

**Tareas:**
- [ ] Crear `src/tracker/mod.rs`
- [ ] Implementar struct `Tracker` con:
  - [ ] `HashMap<(i32, i32), ExtendedKalmanFilter>` - Un filtro por (team, id)
  - [ ] Método `track(team, id, x, y, theta, dt) -> (x, y, theta, vx, vy, omega)`
  - [ ] Método `create_initial_filter() -> ExtendedKalmanFilter`
  - [ ] Manejo de robots nuevos (crear filtro automáticamente)

**Código de referencia:**
```rust
pub struct Tracker {
    filters: HashMap<(i32, i32), ExtendedKalmanFilter>,
}

impl Tracker {
    pub fn track(&mut self, team: i32, id: i32, x: f64, y: f64, 
                 theta: f64, dt: f64) -> (f64, f64, f64, f64, f64, f64) {
        let key = (team, id);
        let filter = self.filters.entry(key)
            .or_insert_with(|| self.create_initial_filter());
        filter.filter_pose(x, y, theta, dt)
    }
}
```

**Criterio de éxito:**
- Tracking funcional para robots y balón
- Velocidades estimadas razonables (no cero)
- Manejo correcto de robots que aparecen/desaparecen

---

#### 1.4. Integrar Tracker en Vision
**Prioridad:** Crítica  
**Estimación:** 2-3 horas

**Tareas:**
- [ ] Reemplazar placeholder del Tracker en `src/vision.rs`
- [ ] Crear instancia de `Tracker` en el módulo Vision
- [ ] Llamar a `tracker.track()` en `process_data()` y `process_ball()`
- [ ] Actualizar tipos de eventos para incluir velocidades reales

**Criterio de éxito:**
- Vision emite eventos con velocidades estimadas (no cero)
- GUI muestra robots moviéndose suavemente
- No hay panics ni errores en runtime

---

#### 1.5. Tests y Validación
**Prioridad:** Alta  
**Estimación:** 4-6 horas

**Tareas:**
- [ ] Tests unitarios para EKF:
  - [ ] Test de predicción con dt conocido
  - [ ] Test de actualización con medición
  - [ ] Test de normalización de ángulos
- [ ] Tests de integración:
  - [ ] Tracking de robot en movimiento constante
  - [ ] Tracking de robot con cambios de velocidad
  - [ ] Tracking de balón
- [ ] Comparación con código C++:
  - [ ] Mismas entradas → mismas salidas (tolerancia 1e-6)

**Criterio de éxito:**
- Cobertura de tests > 80%
- Todos los tests pasando
- Validación cruzada con C++ exitosa

---

### Entregables Fase 1
- [ ] `src/tracker/ekf.rs` - Filtro Kalman Extendido
- [ ] `src/tracker/mod.rs` - Tracker principal
- [ ] `src/tracker/tests.rs` - Tests unitarios
- [ ] Vision integrado con Tracker funcional
- [ ] Documentación del módulo Tracker

**Tiempo estimado total:** 20-31 horas

---

## Fase 2: Implementar Módulo World

### Objetivo
Crear el modelo del mundo que almacena y gestiona el estado de todos los robots y el balón, siendo la fuente de verdad para el resto del sistema.

### Dependencias
- Fase 1 completada (Tracker funcional)
- Vision emitiendo eventos con datos completos

### Tareas Detalladas

#### 2.1. Definir Estructuras de Datos
**Prioridad:** Crítica  
**Estimación:** 3-4 horas  
**Referencia C++:** `src/utilities/robotstate.hpp` y `ballstate.hpp`

**Tareas:**
- [ ] Crear `src/world/robot_state.rs`:
  ```rust
  pub struct RobotState {
      pub id: i32,
      pub team: i32,  // 0 = azul, 1 = amarillo
      pub position: Vec2,
      pub velocity: Vec2,
      pub orientation: f64,
      pub angular_velocity: f64,
      pub active: bool,
      pub last_update: SystemTime,
  }
  ```
- [ ] Crear `src/world/ball_state.rs`:
  ```rust
  pub struct BallState {
      pub position: Vec2,
      pub velocity: Vec2,
  }
  ```
- [ ] Implementar métodos getters/setters
- [ ] Implementar `is_moving()` para BallState

**Criterio de éxito:**
- Estructuras compilan correctamente
- Tests básicos de creación y modificación pasando

---

#### 2.2. Implementar World
**Prioridad:** Crítica  
**Estimación:** 6-8 horas  
**Referencia C++:** `src/world/world.hpp` y `world.cpp`

**Tareas:**
- [ ] Crear `src/world/mod.rs`
- [ ] Implementar struct `World`:
  ```rust
  pub struct World {
      blue_robots: HashMap<i32, RobotState>,
      yellow_robots: HashMap<i32, RobotState>,
      ball: BallState,
      blue_team_size: usize,
      yellow_team_size: usize,
  }
  ```
- [ ] Implementar métodos:
  - [ ] `new(blue_team_size, yellow_team_size) -> Self`
  - [ ] `get_robot_state(id, team) -> Option<&RobotState>`
  - [ ] `get_ball_state() -> &BallState`
  - [ ] `update_robot(id, team, position, orientation, velocity, omega)`
  - [ ] `update_ball(position, velocity)`
  - [ ] `get_blue_team_state() -> Vec<&RobotState>`
  - [ ] `get_yellow_team_state() -> Vec<&RobotState>`
  - [ ] `update()` - Marca robots inactivos si no hay actualizaciones recientes

**Lógica de inactividad:**
- Robot inactivo si `last_update` > 2 segundos

**Criterio de éxito:**
- World almacena y recupera estados correctamente
- Detección de inactividad funcionando
- Thread-safe (usar `Arc<Mutex<World>>` o `Arc<RwLock<World>>`)

---

#### 2.3. Serialización a JSON
**Prioridad:** Media  
**Estimación:** 3-4 horas

**Tareas:**
- [ ] Agregar `serde` y `serde_json` a `Cargo.toml`
- [ ] Implementar `Serialize` para `RobotState` y `BallState`
- [ ] Implementar método `to_json(&self) -> Value` en `World`
- [ ] Formato JSON compatible con el código C++:
  ```json
  {
    "robots": [
      {
        "id": 0,
        "team": "blue",
        "position": {"x": 1.5, "y": 0.3},
        "velocity": {"x": 0.5, "y": -0.2},
        "orientation": 1.57
      }
    ],
    "ball": {
      "position": {"x": 0.0, "y": 0.0},
      "velocity": {"x": 0.1, "y": 0.05}
    }
  }
  ```

**Criterio de éxito:**
- Serialización produce JSON válido
- Formato compatible con GUI y futuros módulos

---

#### 2.4. Integrar World con Vision
**Prioridad:** Crítica  
**Estimación:** 4-5 horas

**Tareas:**
- [ ] Crear instancia de `World` en `main.rs`
- [ ] Conectar eventos de Vision a World:
  - [ ] `robotReceived` → `world.update_robot()`
  - [ ] `ballReceived` → `world.update_ball()`
- [ ] Usar `Arc<RwLock<World>>` para acceso compartido
- [ ] Actualizar GUI para leer desde World

**Criterio de éxito:**
- Vision actualiza World correctamente
- GUI muestra datos desde World
- No hay race conditions

---

#### 2.5. Tests y Validación
**Prioridad:** Alta  
**Estimación:** 4-5 horas

**Tareas:**
- [ ] Tests unitarios:
  - [ ] Creación de World con tamaños de equipo
  - [ ] Actualización de robots y balón
  - [ ] Detección de inactividad
  - [ ] Serialización JSON
- [ ] Tests de integración:
  - [ ] Vision → World → GUI funciona end-to-end

**Criterio de éxito:**
- Cobertura de tests > 75%
- Todos los tests pasando

---

### Entregables Fase 2
- [ ] `src/world/robot_state.rs`
- [ ] `src/world/ball_state.rs`
- [ ] `src/world/mod.rs`
- [ ] World integrado con Vision
- [ ] Serialización JSON funcional
- [ ] Tests completos

**Tiempo estimado total:** 20-26 horas

---

## Fase 3: Implementar Módulo Motion

### Objetivo
Implementar el sistema de control de movimiento: planificación de trayectorias, controladores PID, y evasión de obstáculos.

### Dependencias
- Fase 2 completada (World funcional)
- Estructuras de datos de comandos

### Tareas Detalladas

#### 3.1. Definir Estructuras de Comandos
**Prioridad:** Crítica  
**Estimación:** 2-3 horas  
**Referencia C++:** `src/utilities/motioncommand.hpp`, `kickercommand.hpp`, `robotcommand.hpp`

**Tareas:**
- [ ] Crear `src/motion/commands.rs`:
  ```rust
  pub struct MotionCommand {
      pub id: i32,
      pub team: i32,
      pub vx: f64,
      pub vy: f64,
      pub omega: f64,
  }
  
  pub struct KickerCommand {
      pub id: i32,
      pub team: i32,
      pub kick_x: bool,
      pub kick_z: bool,
      pub dribbler: f64,  // 0.0 - 10.0
  }
  
  pub struct RobotCommand {
      pub id: i32,
      pub team: i32,
      pub motion: MotionCommand,
      pub kicker: KickerCommand,
  }
  ```

**Criterio de éxito:**
- Estructuras compilan y son fáciles de usar

---

#### 3.2. Implementar Environment (Detección de Obstáculos)
**Prioridad:** Alta  
**Estimación:** 6-8 horas  
**Referencia C++:** `src/motion/environment.hpp` y `environment.cpp`

**Tareas:**
- [ ] Crear `src/motion/environment.rs`
- [ ] Implementar struct `Environment`:
  - [ ] Representación del campo (dimensiones, áreas)
  - [ ] Lista de obstáculos (otros robots)
  - [ ] Métodos:
    - [ ] `new(world: &World) -> Self` - Construir desde World
    - [ ] `check_collision(start, end, robot_radius) -> bool`
    - [ ] `get_obstacles() -> Vec<Obstacle>`

**Criterio de éxito:**
- Detección de colisiones funcional
- Tests con casos conocidos pasando

---

#### 3.3. Implementar Path Planner
**Prioridad:** Alta  
**Estimación:** 8-12 horas  
**Referencia C++:** `src/motion/path_planner.hpp` y `path_planner.cpp`

**Tareas:**
- [ ] Crear `src/motion/path_planner.rs`
- [ ] Implementar `FastPathPlanner`:
  - [ ] Método `get_path(from, to, environment) -> Vec<Vec2>`
  - [ ] Estrategia de sub-objetivos
  - [ ] Simplificación de rutas
- [ ] Algoritmo:
  1. Verificar si ruta directa es válida
  2. Si hay colisión, buscar sub-objetivos
  3. Simplificar ruta resultante

**Criterio de éxito:**
- Planificación de rutas sin colisiones
- Rutas razonablemente cortas
- Tests con diferentes configuraciones de obstáculos

---

#### 3.4. Implementar Controladores PID
**Prioridad:** Media  
**Estimación:** 4-6 horas  
**Referencia C++:** `src/motion/pid/`

**Tareas:**
- [ ] Crear `src/motion/pid.rs`
- [ ] Implementar struct `PIDController`:
  ```rust
  pub struct PIDController {
      kp: f64,
      ki: f64,
      kd: f64,
      integral: f64,
      last_error: f64,
  }
  ```
- [ ] Método `compute(error, dt) -> f64`
- [ ] Reset de integral cuando sea necesario

**Criterio de éxito:**
- Controladores PID funcionan correctamente
- Tests con señales conocidas

---

#### 3.5. Implementar Funciones de Movimiento
**Prioridad:** Crítica  
**Estimación:** 10-14 horas  
**Referencia C++:** `src/motion/motion.hpp` y `motion.cpp`

**Tareas:**
- [ ] Crear `src/motion/mod.rs`
- [ ] Implementar struct `Motion` con métodos:
  - [ ] `move_to(robot_state, target, world) -> MotionCommand`
  - [ ] `move_direct(robot_state, target) -> MotionCommand`
  - [ ] `motion(robot_state, target, world, kp_x, ki_x, kp_y, ki_y) -> MotionCommand`
  - [ ] `face_to(robot_state, target, kp, ki, kd) -> MotionCommand`
  - [ ] `face_to_angle(robot_state, target_angle, kp, ki) -> MotionCommand`
  - [ ] `motion_with_orientation(robot_state, target, target_angle, world, ...) -> MotionCommand`
  - [ ] `normalize_angle(angle) -> f64`

**Lógica:**
- `move_to`: Usa path planner + evasión de obstáculos
- `move_direct`: Movimiento directo sin evasión
- `motion`: Control PID personalizado
- `face_to`: Orientar hacia un punto
- `motion_with_orientation`: Movimiento + orientación simultáneos

**Criterio de éxito:**
- Todas las funciones de movimiento implementadas
- Tests con diferentes escenarios
- Comportamiento similar al código C++

---

#### 3.6. Implementar Bang-Bang Control (Opcional)
**Prioridad:** Baja  
**Estimación:** 4-6 horas  
**Referencia C++:** `src/motion/bangbangcontrol/`

**Tareas:**
- [ ] Crear `src/motion/bang_bang.rs`
- [ ] Implementar perfiles de velocidad con límites de aceleración
- [ ] Integrar con path planner

**Criterio de éxito:**
- Perfiles de velocidad suaves
- Respeto de límites de aceleración

---

#### 3.7. Tests y Validación
**Prioridad:** Alta  
**Estimación:** 6-8 horas

**Tareas:**
- [ ] Tests unitarios para cada componente
- [ ] Tests de integración:
  - [ ] Robot se mueve hacia objetivo
  - [ ] Robot evita obstáculos
  - [ ] Robot se orienta correctamente
- [ ] Comparación con código C++ (mismas entradas)

**Criterio de éxito:**
- Cobertura de tests > 70%
- Todos los tests pasando
- Movimiento suave y predecible

---

### Entregables Fase 3
- [ ] `src/motion/commands.rs`
- [ ] `src/motion/environment.rs`
- [ ] `src/motion/path_planner.rs`
- [ ] `src/motion/pid.rs`
- [ ] `src/motion/mod.rs`
- [ ] `src/motion/bang_bang.rs` (opcional)
- [ ] Tests completos

**Tiempo estimado total:** 40-57 horas

---

## Fase 4: Implementar Módulo Radio

### Objetivo
Implementar el sistema de comunicación con robots reales (serial) y grSim (UDP), incluyendo serialización de comandos a Protobuf.

### Dependencias
- Fase 3 completada (Motion funcional)
- Protobufs de grSim generados

### Tareas Detalladas

#### 4.1. Implementar Serialización de Comandos
**Prioridad:** Crítica  
**Estimación:** 4-6 horas  
**Referencia C++:** `src/radio/` y protobufs `grSim_Commands.proto`

**Tareas:**
- [ ] Crear `src/radio/commands.rs`
- [ ] Implementar función `serialize_robot_command(cmd: &RobotCommand) -> Vec<u8>`
- [ ] Crear mensaje Protobuf `grSim_Commands`:
  - [ ] Robot ID y team
  - [ ] Velocidades (vx, vy, omega)
  - [ ] Comandos de kicker (kick_x, kick_z, dribbler)
- [ ] Validar formato con grSim

**Criterio de éxito:**
- Comandos serializados correctamente
- grSim acepta los comandos
- Tests de serialización pasando

---

#### 4.2. Implementar Comunicación con grSim (UDP)
**Prioridad:** Crítica  
**Estimación:** 4-6 horas

**Tareas:**
- [ ] Crear `src/radio/grsim.rs`
- [ ] Implementar struct `GrSimClient`:
  ```rust
  pub struct GrSimClient {
      socket: UdpSocket,
      address: SocketAddr,
  }
  ```
- [ ] Métodos:
  - [ ] `new(address, port) -> Result<Self>`
  - [ ] `send_command(cmd: &RobotCommand) -> Result<()>`
  - [ ] `send_commands(commands: &[RobotCommand]) -> Result<()>`
- [ ] Usar Tokio para operaciones asíncronas

**Criterio de éxito:**
- Comandos enviados a grSim correctamente
- Robots en simulador responden a comandos
- Manejo de errores robusto

---

#### 4.3. Implementar Comunicación Serial (Opcional)
**Prioridad:** Media  
**Estimación:** 6-8 horas

**Tareas:**
- [ ] Agregar `tokio-serial` a `Cargo.toml`
- [ ] Crear `src/radio/serial.rs`
- [ ] Implementar struct `SerialRadio`:
  - [ ] Conexión a puerto serial
  - [ ] Envío de comandos en formato específico del hardware
  - [ ] Configuración de baud rate
- [ ] Manejo de errores de conexión

**Nota:** Depende del protocolo específico del hardware. Puede requerir documentación adicional.

**Criterio de éxito:**
- Comandos enviados por serial correctamente
- Robots reales responden (si hay hardware disponible)

---

#### 4.4. Implementar Radio Principal
**Prioridad:** Crítica  
**Estimación:** 4-6 horas  
**Referencia C++:** `src/radio/radio.hpp` y `radio.cpp`

**Tareas:**
- [ ] Crear `src/radio/mod.rs`
- [ ] Implementar struct `Radio`:
  ```rust
  pub struct Radio {
      use_radio: bool,  // true = serial, false = grSim
      grsim_client: Option<GrSimClient>,
      serial_port: Option<SerialRadio>,
      command_map: HashMap<i32, RobotCommand>,
  }
  ```
- [ ] Métodos:
  - [ ] `new(use_radio, port_name, baud_rate) -> Result<Self>`
  - [ ] `add_motion_command(cmd: MotionCommand)`
  - [ ] `add_kicker_command(cmd: KickerCommand)`
  - [ ] `send_commands() -> Result<()>`
  - [ ] `teleport_robot(id, team, x, y, dir)` - Para debugging
  - [ ] `teleport_ball(x, y)` - Para debugging

**Criterio de éxito:**
- Radio funciona con grSim
- Radio funciona con serial (si está implementado)
- Comandos acumulados y enviados correctamente

---

#### 4.5. Integrar Radio con Motion
**Prioridad:** Crítica  
**Estimación:** 3-4 horas

**Tareas:**
- [ ] Conectar Motion → Radio en el loop principal
- [ ] Enviar comandos cada frame (60 FPS)
- [ ] Manejar errores de envío gracefully

**Criterio de éxito:**
- Comandos de Motion se envían correctamente
- Sistema funciona end-to-end: Vision → World → Motion → Radio

---

#### 4.6. Tests y Validación
**Prioridad:** Alta  
**Estimación:** 4-6 horas

**Tareas:**
- [ ] Tests unitarios:
  - [ ] Serialización de comandos
  - [ ] Envío a grSim (mock)
  - [ ] Manejo de errores
- [ ] Tests de integración:
  - [ ] Motion → Radio → grSim funciona
  - [ ] Múltiples robots simultáneos

**Criterio de éxito:**
- Todos los tests pasando
- Sistema funcional con grSim

---

### Entregables Fase 4
- [ ] `src/radio/commands.rs`
- [ ] `src/radio/grsim.rs`
- [ ] `src/radio/serial.rs` (opcional)
- [ ] `src/radio/mod.rs`
- [ ] Radio integrado con Motion
- [ ] Tests completos

**Tiempo estimado total:** 25-36 horas

---

## Fase 5: Integraciones y Optimizaciones

### Objetivo
Completar integraciones faltantes, optimizar rendimiento, y preparar el sistema para producción.

### Dependencias
- Fases 1-4 completadas

### Tareas Detalladas

#### 5.1. Implementar GameController (Referee)
**Prioridad:** Media  
**Estimación:** 6-8 horas  
**Referencia C++:** `src/receivers/game_controller_ref.hpp` y `game_state.hpp`

**Tareas:**
- [ ] Crear `src/game_controller/mod.rs`
- [ ] Implementar cliente UDP para GameController
- [ ] Parsear mensajes del referee
- [ ] Almacenar estado del juego
- [ ] Exponer estado a otros módulos

**Criterio de éxito:**
- Recepción de comandos del referee funcional
- Estado del juego disponible para estrategias

---

#### 5.2. Optimizar Rendimiento
**Prioridad:** Media  
**Estimación:** 6-10 horas

**Tareas:**
- [ ] Profiling del sistema completo
- [ ] Optimizar hot paths:
  - [ ] Vision processing
  - [ ] Tracker updates
  - [ ] Path planning
- [ ] Reducir allocations innecesarias
- [ ] Usar `#[inline]` donde sea apropiado

**Criterio de éxito:**
- Sistema funciona a 60+ FPS consistentemente
- Uso de CPU < 50% en hardware objetivo

---

#### 5.3. Mejorar Manejo de Errores
**Prioridad:** Media  
**Estimación:** 4-6 horas

**Tareas:**
- [ ] Revisar todos los `unwrap()` y `expect()`
- [ ] Implementar manejo de errores robusto
- [ ] Logging apropiado con `tracing` o `log`
- [ ] Recuperación graceful de errores

**Criterio de éxito:**
- No hay panics en casos normales
- Errores se manejan y logean apropiadamente

---

#### 5.4. Documentación
**Prioridad:** Media  
**Estimación:** 6-8 horas

**Tareas:**
- [ ] Documentar todos los módulos públicos
- [ ] Crear ejemplos de uso
- [ ] Actualizar README
- [ ] Documentar arquitectura

**Criterio de éxito:**
- Documentación completa y clara
- Ejemplos funcionando

---

#### 5.5. Tests de Integración Completa
**Prioridad:** Alta  
**Estimación:** 8-12 horas

**Tareas:**
- [ ] Tests end-to-end:
  - [ ] Vision → World → Motion → Radio
  - [ ] Múltiples robots simultáneos
  - [ ] Escenarios de juego completos
- [ ] Tests de carga (stress tests)
- [ ] Tests de regresión

**Criterio de éxito:**
- Sistema funciona correctamente en todos los escenarios
- No hay regresiones

---

### Entregables Fase 5
- [ ] GameController implementado
- [ ] Sistema optimizado
- [ ] Manejo de errores robusto
- [ ] Documentación completa
- [ ] Tests de integración completos

**Tiempo estimado total:** 30-44 horas

---

## Resumen del Pipeline

### Orden de Ejecución

1. **Fase 1:** Completar Vision (Tracker) - 20-31 horas
2. **Fase 2:** Implementar World - 20-26 horas
3. **Fase 3:** Implementar Motion - 40-57 horas
4. **Fase 4:** Implementar Radio - 25-36 horas
5. **Fase 5:** Integraciones y Optimizaciones - 30-44 horas

**Tiempo total estimado:** 135-194 horas (~3.5-5 meses a tiempo parcial)

### Dependencias entre Fases

```
Fase 1 (Tracker) ──┐
                   ├─→ Fase 2 (World)
                   └─→ Fase 3 (Motion) ──→ Fase 4 (Radio)
                                              │
                                              └─→ Fase 5 (Integraciones)
```

### Prioridades

**Crítico (Bloquea otras fases):**
- Fase 1: Tracker
- Fase 2: World
- Fase 3: Motion básico
- Fase 4: Radio con grSim

**Alto (Importante pero no bloquea):**
- Fase 3: Path planner completo
- Fase 4: Radio serial
- Fase 5: Tests de integración

**Medio/Bajo (Mejoras):**
- Fase 3: Bang-bang control
- Fase 5: Optimizaciones
- Fase 5: GameController

### Criterios de Éxito Globales

- ✅ Sistema funciona end-to-end: Vision → World → Motion → Radio
- ✅ Robots se mueven correctamente en grSim
- ✅ Rendimiento: 60+ FPS consistentes
- ✅ Tests: Cobertura > 70%
- ✅ Documentación completa
- ✅ Compatibilidad con protocolos SSL estándar

---

## Notas Importantes

1. **Estrategia (Lua) es un proyecto separado** - No forma parte de esta migración
2. **Compatibilidad con C++** - Mantener compatibilidad de protocolos y formatos
3. **Testing continuo** - Probar cada fase antes de continuar
4. **Referencias al código C++** - Usar código original como guía, no copiar ciegamente
5. **Idiomático en Rust** - Aprovechar características de Rust (ownership, pattern matching, etc.)

---

**Este pipeline proporciona una guía estructurada para completar la migración de Engine C++ a RustEngine, asegurando que cada módulo se implemente correctamente y se integre con el resto del sistema.**

