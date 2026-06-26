"""
Constantes físicas y de control — espejo EXACTO del motor Rust.

Fuente de verdad (Rust):
- Campo / colisión:        README.md "Campo VSS (referencia)"
- Motion:                  src/motion/mod.rs  (MAX_LINEAR_SPEED, etc.)
- Skills / PID / spin:     src/skills/config.rs (SkillConfig::default)
- Frame-skip del coach:    src/main.rs (COACH_DECISION_PERIOD)

⚠️ CONTRATO DE PARIDAD: si cambian estos valores en Rust, hay que actualizarlos
aquí y re-entrenar. `src/skills/config.rs` centraliza los gains justamente para
facilitar esta verificación (ver su docstring "Paridad Rust ↔ Python").
"""

import math

# ── Campo (m) ────────────────────────────────────────────────────────────────
FIELD_HALF_X: float = 0.75          # campo físico
FIELD_HALF_Y: float = 0.65
LOGICAL_FIELD_HALF_X: float = 0.70  # campo lógico (margen 5 cm) — clamp de targets
LOGICAL_FIELD_HALF_Y: float = 0.60
GOAL_HALF_WIDTH: float = 0.20       # medio ancho del arco (≈40 cm de boca)

ROBOT_RADIUS: float = 0.06          # radio de colisión del robot
BALL_RADIUS: float = 0.05           # radio de colisión de la pelota

# ── Motion (src/motion/mod.rs) ───────────────────────────────────────────────
MAX_LINEAR_SPEED: float = 1.2       # m/s
MAX_ANGULAR_SPEED: float = 3.0      # rad/s
BRAKE_DISTANCE: float = 0.50        # m — desde aquí se decelera al acercarse al target
ARRIVAL_THRESHOLD: float = 0.06     # m — radio de llegada

# ── Skills / PID de heading (src/skills/config.rs SkillConfig::default) ───────
CONTROL_KP: float = 3.0
CONTROL_KI: float = 0.08
CONTROL_KD: float = 0.20
SPIN_OMEGA: float = 2.0             # rad/s — magnitud del Spin

# ── Control loop / frame-skip ────────────────────────────────────────────────
CONTROL_HZ: float = 60.0
DT: float = 1.0 / CONTROL_HZ        # 16.67 ms por tick de física
COACH_DECISION_PERIOD: int = 6      # la policy decide cada 6 ticks (10 Hz)

# ── Normalización de la observación (espejo de observation.rs) ───────────────
MAX_VEL_NORM: float = 1.5
MAX_OMEGA_NORM: float = math.pi
