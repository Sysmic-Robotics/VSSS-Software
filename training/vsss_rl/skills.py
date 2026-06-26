"""
Capa de skills en Python — espejo (fidelidad β, cinemática) del catálogo
congelado del motor Rust (`src/skills/catalog.rs`, `src/skills/mod.rs`).

CONTRATO CONGELADO (no renumerar ni reordenar — rompe el modelo):
    GoTo = 0, FacePoint = 1, ChaseBall = 2, Spin = 3   (SkillId::COUNT = 4)

Cada skill se traduce a un comando del modelo uniciclo `(v_forward, omega)`:
- `v_forward`: velocidad lineal a lo largo del heading del robot (m/s).
- `omega`: velocidad angular (rad/s), signo CCW positivo (igual que Rust).

Fidelidad β (decisión de diseño, ver investigación en el informe):
NO replicamos el UVF (evasión de obstáculos) ni el PID completo del motor Rust.
Replicamos el EFECTO cinemático de cada skill con un controlador proporcional
de heading + perfil de frenado por llegada. Esta es la práctica estándar del
campo (rSoccer usa sim 2D simplificado; Bassani 2020 usa capa de abstracción +
domain adaptation para el gap). El reality gap se documenta y se mitiga con
domain randomization.

Las skills `GoTo / FacePoint / ChaseBall` usan `move_and_face`/`face_to` en Rust
(UVF+PID). `Spin` emite `omega = direction * spin_omega` con vx=vy=0.
"""

from __future__ import annotations

import math
from enum import IntEnum

from .engine_constants import (
    ARRIVAL_THRESHOLD,
    BRAKE_DISTANCE,
    MAX_ANGULAR_SPEED,
    MAX_LINEAR_SPEED,
    SPIN_OMEGA,
)


class SkillId(IntEnum):
    """Catálogo congelado — espejo de src/skills/catalog.rs::SkillId."""

    GO_TO = 0
    FACE_POINT = 1
    CHASE_BALL = 2
    SPIN = 3
    PUSH_BALL = 4   # empuje DIRIGIDO de la pelota hacia el target (tiro/pase)

    @classmethod
    def count(cls) -> int:
        return 5


# Ganancia proporcional del controlador de heading. El motor Rust usa un PID
# (kp=3.0, ki=0.08, kd=0.20) sobre el error de heading; en la aproximación β
# usamos solo el término proporcional con una ganancia que produce el mismo
# comportamiento "girar para encarar, luego avanzar". Calibrada para que un
# error de ~0.37 rad sature MAX_ANGULAR_SPEED.
KP_HEADING: float = 8.0


def _wrap_pi(angle: float) -> float:
    """Normaliza un ángulo a (-π, π]."""
    a = (angle + math.pi) % (2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def _heading_omega(theta: float, desired_angle: float) -> float:
    """Controlador proporcional de heading, saturado a MAX_ANGULAR_SPEED."""
    err = _wrap_pi(desired_angle - theta)
    omega = KP_HEADING * err
    return max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, omega))


def _go_to(
    robot_x: float, robot_y: float, theta: float, tx: float, ty: float
) -> tuple[float, float]:
    """Navegar a (tx, ty). Gira para encarar el target y avanza proporcional
    al alineamiento, decelerando dentro de BRAKE_DISTANCE. Espejo cinemático
    de `GoToSkill` / `motion.move_and_face`.
    """
    dx, dy = tx - robot_x, ty - robot_y
    dist = math.hypot(dx, dy)
    if dist < ARRIVAL_THRESHOLD:
        return 0.0, 0.0  # llegó: sin traslación ni giro

    desired_angle = math.atan2(dy, dx)
    omega = _heading_omega(theta, desired_angle)

    # Solo avanza cuando está razonablemente encarado (cos del error de heading).
    ang_err = _wrap_pi(desired_angle - theta)
    align = max(0.0, math.cos(ang_err))
    speed_cap = MAX_LINEAR_SPEED * min(1.0, dist / BRAKE_DISTANCE)
    v = speed_cap * align
    return v, omega


def _face_point(
    robot_x: float, robot_y: float, theta: float, tx: float, ty: float
) -> tuple[float, float]:
    """Rotar para mirar a (tx, ty) sin trasladarse. Espejo de `FacePointSkill`."""
    dx, dy = tx - robot_x, ty - robot_y
    if dx * dx + dy * dy < 1e-12:
        return 0.0, 0.0  # target sobre el robot → comando seguro (igual que Rust)
    desired_angle = math.atan2(dy, dx)
    return 0.0, _heading_omega(theta, desired_angle)


def _push_ball(
    robot_x: float, robot_y: float, theta: float, tx: float, ty: float, ball_x: float, ball_y: float
) -> tuple[float, float]:
    """Empuje DIRIGIDO: lleva la pelota hacia el target (tx, ty). Espejo
    cinemático de `PushBallSkill`. Si el robot está bien posicionado (detrás de
    la pelota respecto al target y cerca), conduce a través de la pelota hacia
    el target → la empuja en esa dirección. Si está mal posicionado, se detiene
    (la policy debe usar GoTo para acomodarse detrás primero).
    """
    dxg, dyg = tx - ball_x, ty - ball_y
    n = math.hypot(dxg, dyg)
    if n < 1e-6:
        return 0.0, 0.0
    ux, uy = dxg / n, dyg / n
    # ¿robot detrás de la pelota respecto al target?  dot((robot-ball),(target-ball))
    dot = (robot_x - ball_x) * ux + (robot_y - ball_y) * uy
    dist_ball = math.hypot(ball_x - robot_x, ball_y - robot_y)
    if dot > 0.05 or dist_ball > 0.25:
        return 0.0, 0.0  # mal posicionado → reposicionar con GoTo (lo decide la policy)
    # punto un poco más allá de la pelota en dirección al target (overshoot 0.12)
    push_x = ball_x + ux * 0.12
    push_y = ball_y + uy * 0.12
    return _go_to(robot_x, robot_y, theta, push_x, push_y)


def _spin(target_x: float) -> tuple[float, float]:
    """Rotar en el lugar. Sentido por el signo de target_x. Espejo de `SpinSkill`.
    target_x > 0 → CCW (omega>0); < 0 → CW; == 0 → no spin.
    """
    if target_x > 0.0:
        direction = 1.0
    elif target_x < 0.0:
        direction = -1.0
    else:
        direction = 0.0
    return 0.0, direction * SPIN_OMEGA


def dispatch_skill(
    skill_id: int,
    target_x: float,
    target_y: float,
    robot_x: float,
    robot_y: float,
    robot_theta: float,
    ball_x: float,
    ball_y: float,
) -> tuple[float, float]:
    """Traduce una SkillChoice `(skill_id, target)` al comando uniciclo
    `(v_forward, omega)`. Idéntico al despacho de `SkillCatalog::tick`.

    `target` se interpreta según la skill (igual que el catálogo Rust):
    - GoTo / FacePoint → punto destino completo.
    - ChaseBall        → ignora target, usa la pelota.
    - Spin             → solo el signo de target_x.
    """
    sid = int(skill_id)
    if sid == SkillId.GO_TO:
        return _go_to(robot_x, robot_y, robot_theta, target_x, target_y)
    if sid == SkillId.FACE_POINT:
        return _face_point(robot_x, robot_y, robot_theta, target_x, target_y)
    if sid == SkillId.CHASE_BALL:
        return _go_to(robot_x, robot_y, robot_theta, ball_x, ball_y)
    if sid == SkillId.SPIN:
        return _spin(target_x)
    if sid == SkillId.PUSH_BALL:
        return _push_ball(robot_x, robot_y, robot_theta, target_x, target_y, ball_x, ball_y)
    # id fuera de catálogo → comando seguro (el dispatcher Rust valida antes)
    return 0.0, 0.0
