"""
Baseline rule-based — puerto Python EXACTO de src/coach/rule_based_coach.rs
(versión SkillChoice). Sirve para tres cosas:

1. Baseline A/B obligatorio contra el modelo RL (mismas skills, mismo path).
2. Oponente en Fase 2 (1v1) y Fase 3 (3v3).
3. Portero propio cuando la política de campo no lo controla.

Cada robot produce un `Choice = (robot_id, skill_id, target_x, target_y)` en
coordenadas de campo (m), idéntico a lo que emite `RuleBasedCoach::decide`.

Roles fijos (igual que Rust): robot 0 atacante, robot 1 soporte, robot 2 portero.
Parametrizable por `attack_goal` / `own_goal` para servir a cualquier equipo
(el oponente ataca hacia el lado contrario → se construye con goles espejados).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .skills import SkillId

# Constantes idénticas a rule_based_coach.rs
ATTACKER_STAGING_OFFSET = 0.16
ATTACKER_CHASE_RADIUS = 0.10
SUPPORT_BEHIND = 0.25
SUPPORT_LATERAL = 0.20
SUPPORT_CLAMP_X = 0.60
SUPPORT_CLAMP_Y = 0.55
GK_DEFEND_OFFSET = 0.12
GK_Y_CLAMP = 0.20


@dataclass
class Choice:
    robot_id: int
    skill_id: int
    target_x: float
    target_y: float


def _norm(dx: float, dy: float) -> tuple[float, float]:
    n = math.hypot(dx, dy)
    if n < 1e-9:
        return 0.0, 0.0
    return dx / n, dy / n


class RuleBasedCoach:
    """Espejo de src/coach/rule_based_coach.rs."""

    def __init__(
        self,
        attack_goal: tuple[float, float] = (0.75, 0.0),
        own_goal: tuple[float, float] = (-0.75, 0.0),
    ):
        self.attack_goal = attack_goal
        self.own_goal = own_goal

    def attacker_choice(self, ball: tuple[float, float], robot: tuple[float, float]) -> Choice:
        bgx, bgy = _norm(self.attack_goal[0] - ball[0], self.attack_goal[1] - ball[1])
        staging = (ball[0] - bgx * ATTACKER_STAGING_OFFSET, ball[1] - bgy * ATTACKER_STAGING_OFFSET)
        if math.hypot(robot[0] - staging[0], robot[1] - staging[1]) < ATTACKER_CHASE_RADIUS:
            return Choice(0, int(SkillId.CHASE_BALL), 0.0, 0.0)
        return Choice(0, int(SkillId.GO_TO), staging[0], staging[1])

    def support_choice(self, ball: tuple[float, float]) -> Choice:
        ox, oy = _norm(self.own_goal[0] - ball[0], self.own_goal[1] - ball[1])
        lat_x, lat_y = -oy * SUPPORT_LATERAL, ox * SUPPORT_LATERAL
        raw_x = ball[0] + ox * SUPPORT_BEHIND + lat_x
        raw_y = ball[1] + oy * SUPPORT_BEHIND + lat_y
        px = max(-SUPPORT_CLAMP_X, min(SUPPORT_CLAMP_X, raw_x))
        py = max(-SUPPORT_CLAMP_Y, min(SUPPORT_CLAMP_Y, raw_y))
        return Choice(1, int(SkillId.GO_TO), px, py)

    def goalkeeper_choice(self, ball: tuple[float, float]) -> Choice:
        sign = 1.0 if self.own_goal[0] < 0.0 else -1.0
        defend_x = self.own_goal[0] + sign * GK_DEFEND_OFFSET
        target_y = max(-GK_Y_CLAMP, min(GK_Y_CLAMP, ball[1]))
        return Choice(2, int(SkillId.GO_TO), defend_x, target_y)

    def decide(
        self, ball: tuple[float, float], own_positions: list[tuple[float, float]]
    ) -> list[Choice]:
        """Devuelve choices para los 3 roles. `own_positions[0]` se usa para el
        atacante (decide GoTo vs ChaseBall por proximidad al staging)."""
        return [
            self.attacker_choice(ball, own_positions[0]),
            self.support_choice(ball),
            self.goalkeeper_choice(ball),
        ]
