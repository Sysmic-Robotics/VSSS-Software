"""Wrapper de rSoccer (física VSS realista rc_robosim) con NUESTRO contrato:
obs de 52 floats + acción (v, omega) por robot de campo. Subclasea el env VSS de
rSoccer (override obs/acción/reward) y lo expone como gymnasium.Env para SB3.

Intento 3 — pivote a física realista. El modelo entrenado acá usa la MISMA obs y
acción que el deploy (RlCoach), así se despliega sin cambios.

⚠️ ESTADO: esqueleto VALIDADO (pipeline rSoccer→obs52→(v,ω)→SB3 funciona). Falta
portar: reward shaping completo, arquero rule-based, oponente rule-based/snapshot
(hoy son OU-random placeholder), y el currículo. Requiere venv dedicada:
numpy<2, protobuf<3.21, rsoccer-gym, shimmy, stable-baselines3, torch (CPU).
"""
from __future__ import annotations
import math
import numpy as np
import gymnasium
from gymnasium import spaces as gspaces

from rsoccer_gym.Entities import Robot
from rsoccer_gym.vss.env_vss.vss_gym import VSSEnv

# Contrato de normalizacion (espejo de observation.py / observation.rs)
FIELD_HALF_X = 0.75
FIELD_HALF_Y = 0.65
MAX_VEL_NORM = 1.5
MAX_OMEGA_NORM = math.pi
# Topes de accion (espejo de SIM_V_MAX / SIM_OMEGA_MAX del soccer_env)
V_MAX = 1.2
OMEGA_MAX = 3.0
WHEELBASE_L = 0.075  # m (aprox VSS); para (v,omega)->ruedas

CONTROLLED = (0, 1)  # robots de campo que controla la politica (blue 0,1); blue 2 = arquero


class RSoccerFieldEnv(VSSEnv):
    """Controla los 2 robots de campo (blue 0,1) con accion (v,omega); arquero (blue 2)
    y rivales (yellow) por ahora OU-random (placeholder hasta portar GK/oponente)."""

    def __init__(self):
        super().__init__()
        self._r = self.field.rbt_wheel_radius
        self._wheel_max = self.max_v / self._r  # rad/s
        n = len(CONTROLLED)
        self.action_space = gspaces.Box(low=-1.0, high=1.0, shape=(2 * n,), dtype=np.float32)
        self.observation_space = gspaces.Box(low=-2.0, high=2.0, shape=(52,), dtype=np.float32)

    def _vw_to_wheels(self, v, w):
        vl = (v - w * WHEELBASE_L / 2.0) / self._r
        vr = (v + w * WHEELBASE_L / 2.0) / self._r
        c = self._wheel_max
        return float(np.clip(vl, -c, c)), float(np.clip(vr, -c, c))

    def _get_commands(self, action):
        action = np.asarray(action, dtype=np.float32).flatten()
        cmds = []
        for idx, rid in enumerate(CONTROLLED):
            v = float(np.clip(action[2 * idx], -1, 1)) * V_MAX
            w = float(np.clip(action[2 * idx + 1], -1, 1)) * OMEGA_MAX
            vl, vr = self._vw_to_wheels(v, w)
            cmds.append(Robot(yellow=False, id=rid, v_wheel0=vl, v_wheel1=vr))
        for rid in range(self.n_robots_blue):
            if rid in CONTROLLED:
                continue
            a = self.ou_actions[rid].sample()
            vl, vr = self._actions_to_v_wheels(a)
            cmds.append(Robot(yellow=False, id=rid, v_wheel0=vl, v_wheel1=vr))
        for i in range(self.n_robots_yellow):
            a = self.ou_actions[self.n_robots_blue + i].sample()
            vl, vr = self._actions_to_v_wheels(a)
            cmds.append(Robot(yellow=True, id=i, v_wheel0=vl, v_wheel1=vr))
        return cmds

    def _robot_obs(self, rb):
        return [rb.x / FIELD_HALF_X, rb.y / FIELD_HALF_Y,
                rb.v_x / MAX_VEL_NORM, rb.v_y / MAX_VEL_NORM,
                math.sin(math.radians(rb.theta)), math.cos(math.radians(rb.theta)),
                math.radians(rb.v_theta) / MAX_OMEGA_NORM, 1.0]

    def _frame_to_observations(self):
        f = self.frame
        obs = [f.ball.x / FIELD_HALF_X, f.ball.y / FIELD_HALF_Y,
               f.ball.v_x / MAX_VEL_NORM, f.ball.v_y / MAX_VEL_NORM]
        for i in range(3):
            obs += self._robot_obs(f.robots_blue[i])
        for i in range(3):
            obs += self._robot_obs(f.robots_yellow[i])
        return np.array(obs, dtype=np.float32)

    def _calculate_reward_and_done(self):
        half = self.field.length / 2.0
        bx = self.frame.ball.x
        if bx > half:
            return 10.0, True       # gol (azul ataca +X)
        if bx < -half:
            return -10.0, True      # gol en contra
        # shaping minimo (placeholder): progreso de la pelota hacia +X
        reward = 0.0
        if self.last_frame is not None:
            reward += 3.0 * (self.frame.ball.x - self.last_frame.ball.x)
        return float(reward), False


class GymnasiumAdapter(gymnasium.Env):
    """Adapta el env de rSoccer (gym viejo, 4-tupla) a gymnasium (5-tupla) para SB3."""
    metadata = {"render_modes": []}

    def __init__(self):
        self.env = RSoccerFieldEnv()
        self.observation_space = self.env.observation_space
        self.action_space = self.env.action_space

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        obs = self.env.reset()
        return np.asarray(obs, dtype=np.float32), {}

    def step(self, action):
        obs, reward, done, info = self.env.step(action)
        return np.asarray(obs, dtype=np.float32), float(reward), bool(done), False, info or {}

    def close(self):
        self.env.close()


def make_field_env():
    return GymnasiumAdapter()
