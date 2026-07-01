"""Wrapper de rSoccer (fisica VSS realista rc_robosim) con NUESTRO contrato:
obs 52 floats + accion (v, omega) por robot de campo. Deploy sin cambios (RlCoach).

Setup: 2 robots de campo (blue 0,1) los controla la politica; blue 2 = arquero
rule-based; yellow 0,1,2 = oponente rule-based (2 persiguen pelota, 1 arquero).
Reward portado de soccer_env (gol/concede + progreso + robot-a-pelota + vel-a-arco).
Requiere venv dedicada (numpy<2, rsoccer-gym, SB3, gymnasium) — ver requirements-rsoccer.txt.
"""
from __future__ import annotations
import os
# rSoccer trae bindings protobuf viejas; el modo pure-python las hace compatibles
# con cualquier protobuf (ej. el que arrastra tensorboard). Debe setearse ANTES de
# importar rsoccer_gym. Impacto nulo en VSS-v0 (usa rc_robosim, no protobuf en el loop).
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
import math
import numpy as np
import gymnasium
from gymnasium import spaces as gspaces
from rsoccer_gym.Entities import Robot
from rsoccer_gym.vss.env_vss.vss_gym import VSSEnv

FIELD_HALF_X = 0.75
FIELD_HALF_Y = 0.65
MAX_VEL_NORM = 1.5
MAX_OMEGA_NORM = math.pi
V_MAX = 1.2
OMEGA_MAX = 3.0
WHEELBASE_L = 0.075
CONTROLLED = (0, 1)
GK_ID = 2
FRAME_SKIP = 4        # 4 * 0.025s = 0.1s -> decide a 10 Hz (igual que el deploy: COACH_DECISION_PERIOD=6 @ 60Hz)
MAX_DECISIONS = 300   # 300 decisiones * 0.1s = 30s por episodio (como el sim viejo)
GOAL_R, CONCEDE_R = 10.0, -10.0
BALL_PROGRESS_W, ROBOT_TO_BALL_W, BALL_VEL_W, TIME_PEN = 3.0, 0.3, 0.1, -0.001
ATTACK_GOAL = (FIELD_HALF_X, 0.0)   # blue ataca +X


class RSoccerFieldEnv(VSSEnv):
    def __init__(self):
        super().__init__()
        self._r = self.field.rbt_wheel_radius
        self._wmax = self.max_v / self._r
        self.action_space = gspaces.Box(-1.0, 1.0, (2 * len(CONTROLLED),), np.float32)
        self.observation_space = gspaces.Box(-2.0, 2.0, (52,), np.float32)
        self._prev_bg = None
        self._prev_rb = None

    def _vw_to_wheels(self, v, w):
        vl = (v - w * WHEELBASE_L / 2.0) / self._r
        vr = (v + w * WHEELBASE_L / 2.0) / self._r
        return float(np.clip(vl, -self._wmax, self._wmax)), float(np.clip(vr, -self._wmax, self._wmax))

    def _goto_wheels(self, rb, tx, ty, vfrac=0.8):
        dx, dy = tx - rb.x, ty - rb.y
        dist = math.hypot(dx, dy)
        ang_err = ((math.degrees(math.atan2(dy, dx)) - rb.theta + 180) % 360) - 180
        v = V_MAX * vfrac * max(0.0, math.cos(math.radians(ang_err)))
        if dist < 0.05:
            v = 0.0
        w = float(np.clip(math.radians(ang_err) * 4.0, -OMEGA_MAX, OMEGA_MAX))
        return self._vw_to_wheels(v, w)

    def _get_commands(self, action):
        a = np.asarray(action, dtype=np.float32).flatten()
        f = self.frame
        cmds = []
        for idx, rid in enumerate(CONTROLLED):
            v = float(np.clip(a[2 * idx], -1, 1)) * V_MAX
            w = float(np.clip(a[2 * idx + 1], -1, 1)) * OMEGA_MAX
            vl, vr = self._vw_to_wheels(v, w)
            cmds.append(Robot(yellow=False, id=rid, v_wheel0=vl, v_wheel1=vr))
        gk = f.robots_blue[GK_ID]
        vl, vr = self._goto_wheels(gk, -FIELD_HALF_X + 0.12, float(np.clip(f.ball.y, -0.2, 0.2)))
        cmds.append(Robot(yellow=False, id=GK_ID, v_wheel0=vl, v_wheel1=vr))
        for i in range(self.n_robots_yellow):
            yb = f.robots_yellow[i]
            if i == 2:
                vl, vr = self._goto_wheels(yb, FIELD_HALF_X - 0.12, float(np.clip(f.ball.y, -0.2, 0.2)))
            else:
                vl, vr = self._goto_wheels(yb, f.ball.x, f.ball.y)
            cmds.append(Robot(yellow=True, id=i, v_wheel0=vl, v_wheel1=vr))
        return cmds

    def _robot_obs(self, rb):
        return [rb.x / FIELD_HALF_X, rb.y / FIELD_HALF_Y, rb.v_x / MAX_VEL_NORM, rb.v_y / MAX_VEL_NORM,
                math.sin(math.radians(rb.theta)), math.cos(math.radians(rb.theta)),
                math.radians(rb.v_theta) / MAX_OMEGA_NORM, 1.0]

    def _frame_to_observations(self):
        f = self.frame
        obs = [f.ball.x / FIELD_HALF_X, f.ball.y / FIELD_HALF_Y, f.ball.v_x / MAX_VEL_NORM, f.ball.v_y / MAX_VEL_NORM]
        for i in range(3):
            obs += self._robot_obs(f.robots_blue[i])
        for i in range(3):
            obs += self._robot_obs(f.robots_yellow[i])
        return np.array(obs, dtype=np.float32)

    def reset(self):
        self._prev_bg = None
        self._prev_rb = None
        return super().reset()

    def _calculate_reward_and_done(self):
        f = self.frame
        half = self.field.length / 2.0
        bx, by = f.ball.x, f.ball.y
        if bx > half:
            return GOAL_R, True
        if bx < -half:
            return CONCEDE_R, True
        r = TIME_PEN
        dg = math.hypot(ATTACK_GOAL[0] - bx, ATTACK_GOAL[1] - by)
        if self._prev_bg is not None:
            r += BALL_PROGRESS_W * (self._prev_bg - dg)
        self._prev_bg = dg
        dmin = min(math.hypot(f.robots_blue[rid].x - bx, f.robots_blue[rid].y - by) for rid in CONTROLLED)
        if self._prev_rb is not None:
            r += ROBOT_TO_BALL_W * (self._prev_rb - dmin)
        self._prev_rb = dmin
        gx, gy = ATTACK_GOAL[0] - bx, ATTACK_GOAL[1] - by
        gn = math.hypot(gx, gy)
        if gn > 1e-6:
            r += BALL_VEL_W * max(0.0, (f.ball.v_x * gx + f.ball.v_y * gy) / gn)
        return float(r), False


class GymnasiumAdapter(gymnasium.Env):
    """Adapta el env de rSoccer (gym viejo, 4-tupla) a gymnasium (5-tupla) para SB3."""
    metadata = {"render_modes": []}

    def __init__(self):
        self.env = RSoccerFieldEnv()
        self.observation_space = self.env.observation_space
        self.action_space = self.env.action_space
        self._dec = 0

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._dec = 0
        return np.asarray(self.env.reset(), dtype=np.float32), {}

    def step(self, action):
        # Frame-skip: aplica la MISMA accion FRAME_SKIP pasos (decide a 10 Hz como el
        # deploy), acumulando reward (igual que el frame-skip del sim viejo).
        total_r, done, info, obs = 0.0, False, {}, None
        for _ in range(FRAME_SKIP):
            obs, r, done, info = self.env.step(action)
            total_r += r
            if done:
                break
        self._dec += 1
        truncated = (not done) and (self._dec >= MAX_DECISIONS)
        return np.asarray(obs, dtype=np.float32), float(total_r), bool(done), bool(truncated), info or {}

    def close(self):
        self.env.close()


def make_field_env():
    return GymnasiumAdapter()
