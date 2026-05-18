"""
Función de recompensa Fase 1 (1v0 — atacante vs arco vacío).

Diseño:
- Recompensa terminal alta por meter gol.
- Reward shaping denso por acercar la pelota al goal rival.
- Time penalty pequeño para incentivar rapidez.
- Penalización si la pelota sale del campo.

Coordenadas: el goal rival está siempre en +X (convención: atacamos hacia +X).
La función toma estado actual + anterior para el shaping diferencial.
"""

from dataclasses import dataclass
import math


# ── Pesos de la recompensa ─────────────────────────────────────────────────
GOAL_REWARD: float = 10.0
OUT_OF_BOUNDS_PENALTY: float = -1.0
TIME_PENALTY: float = -0.01
BALL_PROGRESS_WEIGHT: float = 5.0     # × Δ(distancia pelota→goal)
ROBOT_TO_BALL_WEIGHT: float = 0.5     # × Δ(distancia robot→pelota), reward shaping
BALL_VELOCITY_TO_GOAL_WEIGHT: float = 0.1  # incentiva chutar al arco


@dataclass
class RewardComponents:
    """Desglose de la recompensa de un step. Útil para logging / debugging."""

    goal: float = 0.0
    out_of_bounds: float = 0.0
    time: float = 0.0
    ball_progress: float = 0.0
    robot_to_ball: float = 0.0
    ball_vel_to_goal: float = 0.0

    @property
    def total(self) -> float:
        return (
            self.goal
            + self.out_of_bounds
            + self.time
            + self.ball_progress
            + self.robot_to_ball
            + self.ball_vel_to_goal
        )


def compute_reward(
    *,
    goal_scored: bool,
    out_of_bounds: bool,
    ball_pos: tuple[float, float],
    prev_ball_pos: tuple[float, float],
    robot_pos: tuple[float, float],
    prev_robot_pos: tuple[float, float],
    ball_vel: tuple[float, float],
    attack_goal: tuple[float, float] = (0.75, 0.0),
) -> RewardComponents:
    """Computa la recompensa del step actual."""
    comp = RewardComponents()

    if goal_scored:
        comp.goal = GOAL_REWARD
        return comp  # terminal: no acumulamos shaping al meter gol

    if out_of_bounds:
        comp.out_of_bounds = OUT_OF_BOUNDS_PENALTY

    comp.time = TIME_PENALTY

    # Progreso de la pelota hacia el goal rival
    d_now = math.dist(ball_pos, attack_goal)
    d_prev = math.dist(prev_ball_pos, attack_goal)
    comp.ball_progress = BALL_PROGRESS_WEIGHT * (d_prev - d_now)

    # Robot acercándose a la pelota (shaping para descubrir la pelota)
    rb_now = math.dist(robot_pos, ball_pos)
    rb_prev = math.dist(prev_robot_pos, prev_ball_pos)
    comp.robot_to_ball = ROBOT_TO_BALL_WEIGHT * (rb_prev - rb_now)

    # Velocidad de la pelota en dirección al goal (incentivo a chutar)
    ball_to_goal = (attack_goal[0] - ball_pos[0], attack_goal[1] - ball_pos[1])
    norm = math.hypot(*ball_to_goal)
    if norm > 1e-6:
        ball_to_goal_unit = (ball_to_goal[0] / norm, ball_to_goal[1] / norm)
        dot = ball_vel[0] * ball_to_goal_unit[0] + ball_vel[1] * ball_to_goal_unit[1]
        comp.ball_vel_to_goal = BALL_VELOCITY_TO_GOAL_WEIGHT * max(dot, 0.0)

    return comp
