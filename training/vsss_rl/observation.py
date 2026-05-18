"""
Contrato de observación — espejo EXACTO de src/coach/observation.rs.

Layout (52 floats):
    [0..3]   ball:          x, y, vx, vy                       (4 floats)
    [4..11]  own_robots[0]: x, y, vx, vy, sin, cos, omega, act (8 floats)
    [12..19] own_robots[1]: ...                                 (8 floats)
    [20..27] own_robots[2]: ...                                 (8 floats)
    [28..35] opp_robots[0]: ...                                 (8 floats)
    [36..43] opp_robots[1]: ...                                 (8 floats)
    [44..51] opp_robots[2]: ...                                 (8 floats)
"""

from dataclasses import dataclass, field
import math
import numpy as np

# ── Constantes de normalización (sincronizadas con observation.rs) ───────────
FIELD_HALF_X: float = 0.75
FIELD_HALF_Y: float = 0.65
MAX_VEL_NORM: float = 1.5
MAX_OMEGA_NORM: float = math.pi

FLAT_SIZE: int = 4 + 8 * 6  # 52


@dataclass
class BallObs:
    x: float = 0.0
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0


@dataclass
class RobotObs:
    x: float = 0.0
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    sin_theta: float = 0.0
    cos_theta: float = 0.0
    omega: float = 0.0
    active: float = 0.0


@dataclass
class Observation:
    ball: BallObs = field(default_factory=BallObs)
    own_robots: list[RobotObs] = field(
        default_factory=lambda: [RobotObs() for _ in range(3)]
    )
    opp_robots: list[RobotObs] = field(
        default_factory=lambda: [RobotObs() for _ in range(3)]
    )
    own_team: int = 0  # 0 = azul, 1 = amarillo

    def to_flat(self) -> np.ndarray:
        v = np.zeros(FLAT_SIZE, dtype=np.float32)
        v[0] = self.ball.x
        v[1] = self.ball.y
        v[2] = self.ball.vx
        v[3] = self.ball.vy
        for i, r in enumerate(self.own_robots):
            base = 4 + 8 * i
            v[base + 0] = r.x
            v[base + 1] = r.y
            v[base + 2] = r.vx
            v[base + 3] = r.vy
            v[base + 4] = r.sin_theta
            v[base + 5] = r.cos_theta
            v[base + 6] = r.omega
            v[base + 7] = r.active
        for i, r in enumerate(self.opp_robots):
            base = 4 + 8 * 3 + 8 * i
            v[base + 0] = r.x
            v[base + 1] = r.y
            v[base + 2] = r.vx
            v[base + 3] = r.vy
            v[base + 4] = r.sin_theta
            v[base + 5] = r.cos_theta
            v[base + 6] = r.omega
            v[base + 7] = r.active
        return v


def normalize_ball(x: float, y: float, vx: float, vy: float) -> BallObs:
    return BallObs(
        x=x / FIELD_HALF_X,
        y=y / FIELD_HALF_Y,
        vx=vx / MAX_VEL_NORM,
        vy=vy / MAX_VEL_NORM,
    )


def normalize_robot(
    x: float,
    y: float,
    theta: float,
    vx: float,
    vy: float,
    omega: float,
    active: bool,
) -> RobotObs:
    if not active:
        return RobotObs()  # todo 0
    return RobotObs(
        x=x / FIELD_HALF_X,
        y=y / FIELD_HALF_Y,
        vx=vx / MAX_VEL_NORM,
        vy=vy / MAX_VEL_NORM,
        sin_theta=math.sin(theta),
        cos_theta=math.cos(theta),
        omega=omega / MAX_OMEGA_NORM,
        active=1.0,
    )
