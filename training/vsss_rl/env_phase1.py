"""
Entorno Fase 1 — mini-simulador VSSS propio (1v0).

Diseño:
- Mundo: campo VSSS ±0.75m × ±0.65m, arcos en x=±0.75.
- Robot: modelo diferencial — control por ruedas (v_left, v_right) ∈ [-1, 1].
  Conversión a (v, ω) usando radio de rueda y separación entre ruedas estándar VSSS.
- Pelota: integración con fricción exponencial; transferencia de momento al contacto.
- Episodio: 1 atacante azul (id 0), sin oponentes. Termina al meter gol, salir del
  campo, o 30 s (1800 steps a 60 Hz).
- Observación: 52 floats con el CONTRATO Rust (vsss_rl/observation.py).
- Recompensa: vsss_rl/reward.py (gol + shaping denso).

Esta clase es Gymnasium-compatible (gym.Env), por lo que se enchufa directo en
Stable-Baselines3 sin más adaptadores.

Refs:
- Dinámica diferencial: ver test_robot_motion_simulation en src/motion/mod.rs.
- Constantes: src/coach/observation.rs y src/motion/mod.rs.
"""

from __future__ import annotations

import math
from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from .observation import (
    FIELD_HALF_X,
    FIELD_HALF_Y,
    FLAT_SIZE,
    Observation,
    normalize_ball,
    normalize_robot,
)
from .reward import compute_reward


# ── Constantes físicas del campo (sincronizadas con el repo Rust) ────────────
ATTACK_GOAL = np.array([FIELD_HALF_X, 0.0], dtype=np.float32)
OWN_GOAL = np.array([-FIELD_HALF_X, 0.0], dtype=np.float32)

GOAL_HALF_WIDTH: float = 0.20      # arco VSSS aprox 40cm de ancho
OUT_X: float = 0.74
OUT_Y: float = 0.64

# Robot diferencial VSSS estándar
ROBOT_RADIUS: float = 0.04
BALL_RADIUS: float = 0.025
WHEEL_BASE: float = 0.075          # separación entre ruedas (~7.5 cm)
MAX_WHEEL_LINEAR_SPEED: float = 0.7  # m/s — cada rueda. Da v_max≈0.7m/s y ω_max≈18 rad/s
                                      # con WHEEL_BASE=0.075. Saturamos ω abajo para realismo.
MAX_ANGULAR_SPEED: float = 8.0     # rad/s — saturación posterior a la diferencial

# Pelota
BALL_FRICTION_PER_TICK: float = 0.94      # vel *= 0.94 cada tick (~1−0.06)
IMPULSE_TRANSFER: float = 0.7             # fracción del v del robot que pasa a la pelota

# Tiempo
DT: float = 1.0 / 60.0                    # 60 Hz como el control loop real
MAX_STEPS: int = 30 * 60                  # 30 s máximo


class VsssPhase1Env(gym.Env):
    """Mini-sim 1v0 — atacante azul vs arco rival vacío.

    Observación: Box(52,) — contrato Rust (ball + 3 own + 3 opp robots normalizados).
    Acción: Box(2,) ∈ [-1, 1] — (v_left_norm, v_right_norm) del robot azul id 0.
    """

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 60}

    def __init__(self, render_mode: str | None = None, seed: int | None = None):
        super().__init__()
        self.render_mode = render_mode

        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32
        )
        # Observación normalizada al rango [-1, 1] aprox; usamos margen 2.0.
        self.observation_space = spaces.Box(
            low=-2.0, high=2.0, shape=(FLAT_SIZE,), dtype=np.float32
        )

        # Estado interno
        self._rng = np.random.default_rng(seed)
        self._robot_pos = np.zeros(2, dtype=np.float32)
        self._robot_theta = 0.0
        self._robot_vel = np.zeros(2, dtype=np.float32)
        self._robot_omega = 0.0
        self._ball_pos = np.zeros(2, dtype=np.float32)
        self._ball_vel = np.zeros(2, dtype=np.float32)
        self._steps = 0

    # ── API gym ──────────────────────────────────────────────────────────

    def reset(
        self, *, seed: int | None = None, options: dict | None = None
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)

        # Robot: spawn aleatorio en la mitad izquierda del campo, orientación random.
        self._robot_pos = np.array(
            [
                self._rng.uniform(-0.55, -0.15),
                self._rng.uniform(-0.40, 0.40),
            ],
            dtype=np.float32,
        )
        self._robot_theta = float(self._rng.uniform(-math.pi, math.pi))
        self._robot_vel = np.zeros(2, dtype=np.float32)
        self._robot_omega = 0.0

        # Pelota: spawn aleatorio cerca del centro, ligeramente desplazada.
        self._ball_pos = np.array(
            [
                self._rng.uniform(-0.10, 0.30),
                self._rng.uniform(-0.30, 0.30),
            ],
            dtype=np.float32,
        )
        self._ball_vel = np.zeros(2, dtype=np.float32)

        self._steps = 0
        return self._build_obs(), self._info()

    def step(
        self, action: np.ndarray
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        # ── Acción ──
        action = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
        v_left = float(action[0]) * MAX_WHEEL_LINEAR_SPEED
        v_right = float(action[1]) * MAX_WHEEL_LINEAR_SPEED

        # Modelo diferencial
        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / WHEEL_BASE
        omega = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, omega))

        # ── Integración del robot ──
        prev_robot_pos = self._robot_pos.copy()
        cos_t = math.cos(self._robot_theta)
        sin_t = math.sin(self._robot_theta)
        self._robot_vel = np.array([v * cos_t, v * sin_t], dtype=np.float32)
        self._robot_pos = self._robot_pos + self._robot_vel * DT
        self._robot_theta = _wrap_pi(self._robot_theta + omega * DT)
        self._robot_omega = omega

        # Clamp del robot a límites del campo (sin rebote, solo barrera)
        self._robot_pos[0] = float(np.clip(self._robot_pos[0], -OUT_X, OUT_X))
        self._robot_pos[1] = float(np.clip(self._robot_pos[1], -OUT_Y, OUT_Y))

        # ── Integración de la pelota ──
        prev_ball_pos = self._ball_pos.copy()
        dist_rb = float(np.linalg.norm(self._ball_pos - self._robot_pos))
        contact = dist_rb < (ROBOT_RADIUS + BALL_RADIUS + 0.005)
        if contact and v > 0.0:
            # Empuje en el forward del robot, proporcional a v
            forward = np.array([cos_t, sin_t], dtype=np.float32)
            impulse = forward * (v * IMPULSE_TRANSFER)
            self._ball_vel = self._ball_vel + impulse * 5.0 * DT  # impulso instantáneo escalado
        self._ball_vel = self._ball_vel * BALL_FRICTION_PER_TICK
        self._ball_pos = self._ball_pos + self._ball_vel * DT

        # ── Detección de eventos ──
        ball_x, ball_y = float(self._ball_pos[0]), float(self._ball_pos[1])
        goal_scored = ball_x > (FIELD_HALF_X - 0.005) and abs(ball_y) < GOAL_HALF_WIDTH
        own_goal = ball_x < -(FIELD_HALF_X - 0.005) and abs(ball_y) < GOAL_HALF_WIDTH
        out_of_bounds = (
            (not goal_scored and not own_goal)
            and (abs(ball_x) > OUT_X or abs(ball_y) > OUT_Y)
        )

        # ── Recompensa ──
        comp = compute_reward(
            goal_scored=goal_scored,
            out_of_bounds=out_of_bounds or own_goal,
            ball_pos=(ball_x, ball_y),
            prev_ball_pos=(float(prev_ball_pos[0]), float(prev_ball_pos[1])),
            robot_pos=(float(self._robot_pos[0]), float(self._robot_pos[1])),
            prev_robot_pos=(float(prev_robot_pos[0]), float(prev_robot_pos[1])),
            ball_vel=(float(self._ball_vel[0]), float(self._ball_vel[1])),
            attack_goal=(float(ATTACK_GOAL[0]), float(ATTACK_GOAL[1])),
        )

        self._steps += 1
        terminated = bool(goal_scored or own_goal or out_of_bounds)
        truncated = self._steps >= MAX_STEPS

        info = self._info()
        info["goal_scored"] = goal_scored
        info["own_goal"] = own_goal
        info["out_of_bounds"] = out_of_bounds
        info["reward_components"] = {
            "goal": comp.goal,
            "out_of_bounds": comp.out_of_bounds,
            "time": comp.time,
            "ball_progress": comp.ball_progress,
            "robot_to_ball": comp.robot_to_ball,
            "ball_vel_to_goal": comp.ball_vel_to_goal,
        }

        return self._build_obs(), float(comp.total), terminated, truncated, info

    def render(self):
        # Render simple ASCII para debug — no usar en training.
        if self.render_mode != "human":
            return None
        bx, by = self._ball_pos
        rx, ry = self._robot_pos
        print(
            f"t={self._steps:4d}  robot=({rx:+.2f},{ry:+.2f}) θ={math.degrees(self._robot_theta):+5.0f}°  "
            f"ball=({bx:+.2f},{by:+.2f})  vb=({self._ball_vel[0]:+.2f},{self._ball_vel[1]:+.2f})"
        )

    def close(self):
        pass

    # ── Helpers internos ──────────────────────────────────────────────────

    def _build_obs(self) -> np.ndarray:
        obs = Observation(own_team=0)
        obs.ball = normalize_ball(
            float(self._ball_pos[0]),
            float(self._ball_pos[1]),
            float(self._ball_vel[0]),
            float(self._ball_vel[1]),
        )
        # Solo el robot 0 está activo (1v0)
        obs.own_robots[0] = normalize_robot(
            x=float(self._robot_pos[0]),
            y=float(self._robot_pos[1]),
            theta=float(self._robot_theta),
            vx=float(self._robot_vel[0]),
            vy=float(self._robot_vel[1]),
            omega=float(self._robot_omega),
            active=True,
        )
        # own_robots[1], own_robots[2], opp_robots[*] quedan en default (active=0)
        return obs.to_flat()

    def _info(self) -> dict[str, Any]:
        return {
            "steps": self._steps,
            "robot_pos": self._robot_pos.copy(),
            "ball_pos": self._ball_pos.copy(),
        }


def _wrap_pi(angle: float) -> float:
    """Normaliza ángulo a [-π, π]."""
    a = (angle + math.pi) % (2 * math.pi)
    if a < 0:
        a += 2 * math.pi
    return a - math.pi
