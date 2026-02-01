# VSSS Software

Stack de control en Rust para **VSS (Very Small Size Soccer)** compatible con **FIRASim**. Recibe visión por multicast, mantiene un modelo del mundo (balón y robots), aplica filtrado con Kalman extendido y envía comandos de movimiento al simulador por UDP.

## Requisitos

- **Rust** (toolchain estable, 1.70 o superior recomendado)
- **FIRASim** en ejecución, con:
  - Visión por multicast en `224.0.0.1:10002`
  - Actuadores/control escuchando en `127.0.0.1:20011`

## Compilación y ejecución

```bash
cargo build --release
cargo run --release
```

La primera vez, `build.rs` genera código Rust desde los `.proto`; no hace falta tener `protoc` instalado.

## Arquitectura

El programa se organiza en módulos que se comunican por canales asíncronos (Tokio):

| Módulo | Función |
|--------|--------|
| **vision** | Socket UDP multicast para paquetes de visión (SSL/FIRA). Parsea protobuf y opcionalmente filtra poses con un EKF por robot/balón. Emite eventos (posición, velocidad) al resto del sistema. |
| **world** | Estado global: posiciones y velocidades de balón y robots (azul/amarillo, 3 por equipo). Se actualiza con los eventos de visión y marca robots inactivos tras un timeout. |
| **tracker** | Filtro de Kalman extendido por cada (team, id). Proporciona posiciones filtradas y velocidades estimadas a partir de las detecciones crudas. Se puede activar o desactivar desde la interfaz. |
| **motion** | Planificador de rutas (obstáculos), PID y generación de comandos (vx, vy, omega). Incluye `move_to`, `move_direct` y construcción de `RobotCommand`. |
| **radio** | Envío de comandos al simulador. Soporta FIRASim (SSL-Simulation, puerto 20011) y GrSim. Serializa a protobuf y envía por UDP. |
| **GUI** | Interfaz con Iced: campo 2D, estado de visión, configuración de IP/puerto multicast y toggle del tracker. |

Flujo resumido: **Visión (multicast) → Tracker (opcional) → World → Lógica de control → Motion → Radio (UDP al simulador)**. La GUI muestra el estado y permite cambiar IP/puerto y activar/desactivar el tracker.

## Configuración por defecto

- **Visión:** `224.0.0.1:10002` (multicast FIRASim).
- **Comandos:** `127.0.0.1:20011` (actuadores FIRASim).

Si los robots no se mueven, comprueba en FIRASim que el puerto de control/actuadores sea 20011 (o el que uses) y que el firewall permita tráfico local.

## Estructura del código

```
src/
├── main.rs           # Punto de entrada, canales, spawn de visión/motion/radio/GUI
├── vision.rs        # Recepción visión, parsing, integración con tracker
├── world/            # Modelo del mundo (robots, balón)
├── tracker/         # EKF por objeto (ekf.rs, mod.rs)
├── motion/          # Path planner, PID, comandos de movimiento
├── radio/           # Clientes UDP FIRASim y GrSim, serialización comandos
├── GUI/             # Interfaz Iced (campo, estado, configuración)
├── protos/          # Definiciones .proto y código Rust generado
└── tui/             # Interfaz de consola (alternativa)
```

Los mensajes de visión y control usan protobuf (FIRA, SSL-Vision, SSL-Simulation). El código de los protos se genera en compilación desde `src/protos/*.proto`.

## Formateo y análisis estático

```bash
cargo fmt
cargo clippy
```

Recomendado usar **rust-analyzer** en el editor para completado y detección de errores en tiempo real.

## Licencia y autor

Consultar el repositorio o los metadatos del proyecto.
