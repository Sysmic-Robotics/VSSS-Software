# Documentación del Proyecto RustEngine

## 1. Contexto General

**RustEngine** es la migración a Rust del sistema de control Engine de Sysmic Robotics para RoboCup SSL. Es el primer paso de la migración desde C++/Qt6 a Rust, enfocándose en eficiencia y conveniencia.

### Separación de Responsabilidades

Es importante entender que existen **dos proyectos separados**:

- **Engine (este proyecto)**: Lógica de control de bajo nivel
  - Recepción y procesamiento de datos de visión
  - Estimación de velocidades (tracking)
  - Modelo del mundo
  - Control de movimiento y planificación de trayectorias
  - Comunicación con robots/simulador

- **Strategy (proyecto separado)**: Lógica de estrategia de juego
  - Programado en Lua
  - Se conecta al Engine para tomar decisiones de alto nivel
  - **NO forma parte de esta migración**

## 2. Estado Actual del Proyecto

### Módulos Implementados

#### A. Módulo Vision (`src/vision.rs`)
- ✅ Recepción de paquetes UDP multicast desde SSL-Vision/grSim
- ✅ Parsing de Protocol Buffers (SSL_WrapperPacket)
- ✅ Procesamiento de robots (azul y amarillo) y balón
- ✅ Conversión de coordenadas (milímetros → metros)
- ✅ Envío de eventos a través de canales asíncronos
- ✅ Verificación de campos requeridos antes de procesar

**Estado**: Funcional, pero el Tracker es un placeholder (devuelve velocidades en cero)

#### B. Interfaz Gráfica (`src/GUI/`)
- Framework: **Iced 0.13**
- Visualización del campo con todos los detalles:
  - Líneas de penalty
  - Arcos con profundidad
  - Margen verde alrededor del campo
- Visualización de robots y balón en tiempo real
- Estadísticas de conexión y frecuencia de paquetes
- Configuración de IP y puerto de conexión

#### C. Protocol Buffers (`src/protos/`)
- Generación automática desde archivos `.proto`
- Protocolos SSL-Vision, grSim, GameController
- Generación durante la compilación con `build.rs`

### Pendiente (Crítico)

1. **Tracker con Filtros Kalman**
   - Actualmente devuelve velocidades en cero
   - Necesita implementar EKF para estimar velocidades reales
   - Estado: `[x, y, sin(θ), cos(θ), vx, vy, ω]`

2. **Modelo del Mundo (World)**
   - Estructura para almacenar estados de robots y balón
   - Actualización desde Vision
   - Serialización a JSON

3. **Módulos de Control**
   - Motion: planificación de trayectorias
   - Radio: envío de comandos a robots

## 3. Tecnologías y Dependencias

### Lenguaje y Herramientas
- **Rust** (edition 2024)
- **Cargo** (gestor de dependencias)

### Dependencias Principales (`Cargo.toml`)

```toml
[dependencies]
tokio = { version = "1", features = ["full"] }      # Runtime asíncrono
protobuf = "3.2"                                     # Protocol Buffers
glam = "0.24"                                        # Matemáticas vectoriales
iced = { version = "0.13", features = ["canvas", "tokio"] }  # GUI

[build-dependencies]
protobuf-codegen = "3"                               # Generación de código desde .proto
```

### Descripción de Dependencias

1. **Tokio**
   - Runtime asíncrono para Rust
   - Manejo de UDP sockets, canales, tareas concurrentes
   - Reemplaza a Qt6 para concurrencia

2. **Protobuf 3.2**
   - Compatible con el estilo C++ ParseFromArray
   - Serialización/deserialización de mensajes SSL

3. **Glam 0.24**
   - Biblioteca de matemáticas vectoriales
   - Reemplaza a QVector2D del código C++
   - Operaciones vectoriales eficientes

4. **Iced 0.13**
   - Framework GUI multiplataforma
   - Canvas para renderizado del campo
   - Integración con Tokio para actualizaciones asíncronas

5. **Protobuf-codegen 3**
   - Genera código Rust desde archivos `.proto`
   - Ejecutado en `build.rs` durante la compilación
   - No requiere `protoc` instalado (generación pura en Rust)

## 4. Arquitectura del Sistema

### Flujo de Datos Actual

```
grSim/SSL-Vision (UDP Multicast)
    ↓
Vision Module (src/vision.rs)
    ├─→ Parsing Protobuf
    ├─→ Procesamiento de datos
    ├─→ Tracker (placeholder)
    └─→ Canales asíncronos (mpsc)
        ├─→ VisionEvent (para futuros módulos)
        └─→ StatusUpdate (para GUI)
            ↓
GUI (src/GUI/)
    └─→ Visualización en tiempo real
```

### Estructura del Código

```
src/
├── main.rs              # Punto de entrada, orquestación
├── vision.rs            # Módulo Vision (recepción y procesamiento)
├── protos/              # Protocol Buffers generados
│   ├── ssl_vision_*.rs
│   ├── grSim_*.rs
│   └── ssl_gc_*.rs
├── GUI/                 # Interfaz gráfica
│   ├── mod.rs           # Lógica principal de GUI
│   ├── field.rs         # Renderizado del campo
│   └── vision_status.rs # Panel de estadísticas
└── tui/                 # Interfaz de terminal (no usado actualmente)
```

## 5. Características Técnicas

### Concurrencia
- **Tokio** para operaciones asíncronas
- Canales `mpsc` para comunicación entre módulos
- Thread separado para el runtime de Tokio
- GUI bloqueante (Iced) en el thread principal

### Manejo de Errores
- Verificación de campos requeridos antes de procesar
- Manejo de errores de parsing con rate limiting
- Logs de depuración para diagnóstico

### Protocolos Soportados
- **SSL-Vision**: Detección de robots y balón
- **grSim**: Simulador (puerto 10020)
- **GameController**: Referee (protobufs generados, no implementado aún)

## 6. Diferencias con el Engine C++

| Aspecto | C++ Engine | RustEngine (actual) |
|---------|------------|---------------------|
| GUI | Qt6 | Iced 0.13 |
| Concurrencia | Qt Signals/Slots | Tokio + Canales |
| Matemáticas | QVector2D | Glam Vec2 |
| Scripting | Lua (sol2) | No implementado |
| Tracker | Kalman Filters | Placeholder |
| World Model | Implementado | Pendiente |
| Motion Control | Implementado | Pendiente |
| Radio | Implementado | Pendiente |

## 7. Próximos Pasos de Migración

1. **Implementar Tracker con Filtros Kalman**
   - Filtros Kalman Extendidos (EKF)
   - Un filtro por robot/balón identificado por `(team, id)`
   - Estimación de velocidades desde posiciones medidas

2. **Crear Módulo World**
   - Estructura para almacenar estados de robots y balón
   - Actualización desde Vision
   - Serialización a JSON para visualización

3. **Migrar Módulo Motion**
   - Planificación de trayectorias
   - Controladores PID
   - Evasión de obstáculos

4. **Migrar Módulo Radio**
   - Envío de comandos a robots reales (serial)
   - Envío de comandos a grSim (UDP)
   - Serialización a Protobuf

5. **(Opcional) Integración con Lua**
   - Interfaz para scripts de estrategia
   - Exposición de funciones C++/Rust a Lua

## 8. Notas Importantes

- **Strategy (Lua) es un proyecto separado** y no forma parte de esta migración
- El Engine se enfoca en **control de bajo nivel**, no en estrategia
- La migración prioriza **funcionalidad** sobre características avanzadas
- Se mantiene **compatibilidad** con protocolos SSL estándar

## 9. Instalación y Uso

### Requisitos
- Rust (última versión estable)
- Cargo (incluido con Rust)

### Compilación
```bash
cargo build
```

### Ejecución
```bash
cargo run
```

### Formateo y Linting
```bash
cargo fmt      # Formatear código
cargo clippy   # Análisis de código
```

## 10. Referencias

- Repositorio Engine C++: https://github.com/Sysmic-Robotics/engine
- RoboCup SSL: https://ssl.robocup.org/
- grSim: https://github.com/RoboCup-SSL/grSim
- Protocolos SSL: https://github.com/RoboCup-SSL/ssl-vision

---

**Este documento contextualiza el estado actual de RustEngine para la migración desde el Engine C++. El proyecto está en fase inicial, con el módulo Vision funcional pero pendiente de completar el Tracker y otros módulos críticos.**

