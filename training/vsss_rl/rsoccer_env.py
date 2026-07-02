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
from collections import deque
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
# WHEELBASE efectivo de rSim, MEDIDO empíricamente (sysid/measure_wheelbase_rsim.py,
# 2026-07-02): giro puro a 3 velocidades → L≈0.0776 (el nominal 0.075 respondía ~3%
# arriba). El deploy usa las constantes efectivas de FIRASim (r=0.02, L=0.085) en
# commands.rs — cada lado con las constantes verdaderas de SU sim ⇒ la misma (v,ω)
# produce la misma respuesta física en entrenamiento y en deploy (Intento 4, Fase A1).
WHEELBASE_L = 0.0776
CONTROLLED = (0, 1)
GK_ID = 2
FRAME_SKIP = 4        # 4 * 0.025s = 0.1s -> decide a 10 Hz (igual que el deploy: COACH_DECISION_PERIOD=6 @ 60Hz)
MAX_DECISIONS = 300   # 300 decisiones * 0.1s = 30s por episodio (como el sim viejo)
PHYS_DT = 0.025       # tick físico de rSim (40 Hz)
# Rampa de aceleración — espejo EXACTO de DirectVel en el deploy (skills/catalog.rs):
# la política nunca ve un cambio de (v,ω) más brusco de lo que el runtime ejecuta (A2).
DV_LIN_ACCEL = 1.2    # m/s² acelerando
DV_LIN_DECEL = 5.0    # m/s² frenando
DV_ANG_ACCEL = 30.0   # rad/s²
# Ruido de observación MEDIDO (σ del sistema de visión real, Brandão et al. IEEE
# Access 2022). Se aplica a posiciones/orientación; las VELOCIDADES de la obs se
# derivan por diferencia finita de las posiciones ruidosas a 10 Hz — imita al EKF
# del deploy (lag + ruido) en vez de exponer las velocidades perfectas del sim (A3).
OBS_SIGMA_X = 1.854e-3   # m
OBS_SIGMA_Y = 1.679e-3   # m
OBS_SIGMA_TH = 0.031     # rad (~1.79°)
FD_DT = FRAME_SKIP * PHYS_DT  # ventana de la diferencia finita = 1 decisión (0.1 s)
GOAL_R, CONCEDE_R = 10.0, -10.0
# Reward B2 (estilo Brandão IEEE Access 2022): el término DOMINANTE es la velocidad
# de la pelota hacia el arco rival (con signo) = premia el REMATE. El progreso por
# distancia queda secundario (antes 3.0 dominaba → enseñaba a "pastorear" la pelota:
# 76% de episodios en timeout). Pesos por tick calibrados para que por decisión
# (4 ticks) el término de remate llegue a ~0.7 como en Brandão.
BALL_VEL_W = 0.175      # ×4 ticks ≈ 0.7/decisión con pelota a V_MAX hacia el arco
BALL_PROGRESS_W = 1.0   # gradiente Δdist (~0.1/decisión máx) — secundario
# Cercanía a la pelota como NIVEL (Brandão r_dist = 0.1·cercanía por paso), NO como
# gradiente: el gradiente se telescopa a ~0 total y no compensa el riesgo de tocar
# la pelota → el Gate 1a (8M) aprendió a EVITARLA (diag: dmin=0.51 m, 67% de
# episodios cortados por Free Ball, 8% goles). El nivel paga sostenidamente por
# estar encima de la pelota. (0.025/tick ≈ 0.1/decisión con contacto.)
BALL_PROX_W = 0.025
BALL_PROX_DMAX = 0.75   # a esta distancia el término llega a 0
TIME_PEN = -0.001
# DEF_POS eliminado (Gate 1a): era farmeable — pagaba por estar "detrás de la pelota"
# a CUALQUIER distancia, alimentando la pasividad. La defensa vendrá del concede -10
# y del self-play (Fase C/D).
ATTACK_GOAL = (FIELD_HALF_X, 0.0)   # blue ataca +X
# Árbitro interno (B-R — reglas LARC 2026, como el "virtual referee" de Brandão):
AREA_X = 0.60           # área de meta: |x| > 0.60 (prof. 0.15 desde 0.75)
AREA_Y = 0.35           # |y| < 0.35 (ancho 0.70)
FOUL_DEF_R = -1.0       # defensa ilegal: 2+ propios en área propia CON pelota (→ penal)
FOUL_ATK_R = -0.2       # ataque ilegal: 2+ propios en área rival CON pelota (→ goal kick)
STUCK_R = -0.5          # Free Ball: pelota quieta 10 s → el árbitro corta la jugada
STUCK_TICKS = 400       # 10 s a 40 Hz
STUCK_DIST = 0.03       # umbral de pelota "quieta" (3 cm)
IDLE_TICKS = 200        # comportamiento de juego (§9.2): robot quieto > 5 s
IDLE_DIST = 0.03
IDLE_PEN = -0.0005      # por tick y robot quieto (≈ −0.02/decisión)


class RSoccerFieldEnv(VSSEnv):
    def __init__(self):
        super().__init__()
        self._r = self.field.rbt_wheel_radius
        self._wmax = self.max_v / self._r
        self.action_space = gspaces.Box(-1.0, 1.0, (2 * len(CONTROLLED),), np.float32)
        self.observation_space = gspaces.Box(-2.0, 2.0, (52,), np.float32)
        self._prev_bg = None
        # Estado de la rampa DirectVel por robot controlado: [v_cur, omega_cur] (A2).
        self._dv = {rid: [0.0, 0.0] for rid in CONTROLLED}
        # Historial de poses RUIDOSAS por entidad, para la diferencia finita a 10 Hz (A3).
        self._pose_hist = {}
        # Árbitro interno (B-R): timers de pelota estancada y robots quietos.
        self._ball_ref = None
        self._ball_still = 0
        self._idle_ref = {}
        self._idle = {rid: 0 for rid in CONTROLLED}

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
            v_t = float(np.clip(a[2 * idx], -1, 1)) * V_MAX
            w_t = float(np.clip(a[2 * idx + 1], -1, 1)) * OMEGA_MAX
            # Rampa DirectVel (espejo del deploy): integra hacia el target con los
            # mismos límites de aceleración, a la resolución del tick físico (A2).
            st = self._dv[rid]
            a_lin = DV_LIN_ACCEL if abs(v_t) >= abs(st[0]) else DV_LIN_DECEL
            st[0] += float(np.clip(v_t - st[0], -a_lin * PHYS_DT, a_lin * PHYS_DT))
            st[1] += float(np.clip(w_t - st[1], -DV_ANG_ACCEL * PHYS_DT, DV_ANG_ACCEL * PHYS_DT))
            vl, vr = self._vw_to_wheels(st[0], st[1])
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

    def _noisy_pose(self, key, x, y, th):
        """Aplica ruido de visión a la pose y la guarda en el historial. Devuelve
        (pose ruidosa actual, pose ruidosa de hace 1 decisión, historial completo?)."""
        nx = x + np.random.normal(0.0, OBS_SIGMA_X)
        ny = y + np.random.normal(0.0, OBS_SIGMA_Y)
        nth = th + np.random.normal(0.0, OBS_SIGMA_TH)
        hist = self._pose_hist.setdefault(key, deque(maxlen=FRAME_SKIP + 1))
        hist.append((nx, ny, nth))
        # Tras el append, hist[0] queda EXACTAMENTE FRAME_SKIP ticks (0.1 s) atrás.
        full = len(hist) == FRAME_SKIP + 1
        return (nx, ny, nth), hist[0], full

    def _robot_obs(self, key, rb):
        (x, y, th), (px, py, pth), full = self._noisy_pose(key, rb.x, rb.y, math.radians(rb.theta))
        if full:
            vx, vy = (x - px) / FD_DT, (y - py) / FD_DT
            dth = th - pth
            while dth > math.pi:
                dth -= 2 * math.pi
            while dth < -math.pi:
                dth += 2 * math.pi
            om = dth / FD_DT
        else:
            vx = vy = om = 0.0  # arranque: como el EKF del deploy (velocidad 0 hasta converger)
        return [x / FIELD_HALF_X, y / FIELD_HALF_Y, vx / MAX_VEL_NORM, vy / MAX_VEL_NORM,
                math.sin(th), math.cos(th), om / MAX_OMEGA_NORM, 1.0]

    def _frame_to_observations(self):
        f = self.frame
        (bx, by, _), (pbx, pby, _), full = self._noisy_pose("ball", f.ball.x, f.ball.y, 0.0)
        bvx, bvy = ((bx - pbx) / FD_DT, (by - pby) / FD_DT) if full else (0.0, 0.0)
        obs = [bx / FIELD_HALF_X, by / FIELD_HALF_Y, bvx / MAX_VEL_NORM, bvy / MAX_VEL_NORM]
        for i in range(3):
            obs += self._robot_obs(("b", i), f.robots_blue[i])
        for i in range(3):
            obs += self._robot_obs(("y", i), f.robots_yellow[i])
        return np.array(obs, dtype=np.float32)

    def reset(self):
        self._prev_bg = None
        self._dv = {rid: [0.0, 0.0] for rid in CONTROLLED}
        self._pose_hist = {}
        self._ball_ref = None
        self._ball_still = 0
        self._idle_ref = {}
        self._idle = {rid: 0 for rid in CONTROLLED}
        return super().reset()

    def _calculate_reward_and_done(self):
        f = self.frame
        half = self.field.length / 2.0
        bx, by = f.ball.x, f.ball.y
        if bx > half:
            return GOAL_R, True
        if bx < -half:
            return CONCEDE_R, True

        # ── Árbitro interno (B-R, reglas LARC 2026) ──────────────────────────
        # Faltas de área (§9.5): la condición incluye la PELOTA en el área (así
        # operacionaliza Brandão su árbitro virtual). El foul REEMPLAZA el shaping
        # de ese tick y corta la jugada (como el árbitro real).
        if abs(by) < AREA_Y:
            if bx < -AREA_X and sum(1 for i in range(3)
                    if f.robots_blue[i].x < -AREA_X and abs(f.robots_blue[i].y) < AREA_Y) >= 2:
                return FOUL_DEF_R, True   # defensa ilegal → penal en contra
            if bx > AREA_X and sum(1 for i in range(3)
                    if f.robots_blue[i].x > AREA_X and abs(f.robots_blue[i].y) < AREA_Y) >= 2:
                return FOUL_ATK_R, True   # ataque ilegal → goal kick rival
        # Free Ball (§10.4): pelota sin moverse 10 s → el árbitro corta.
        if self._ball_ref is None or math.hypot(bx - self._ball_ref[0], by - self._ball_ref[1]) > STUCK_DIST:
            self._ball_ref = (bx, by)
            self._ball_still = 0
        else:
            self._ball_still += 1
            if self._ball_still >= STUCK_TICKS:
                return STUCK_R, True

        # ── Shaping B2 ────────────────────────────────────────────────────────
        r = TIME_PEN
        gx, gy = ATTACK_GOAL[0] - bx, ATTACK_GOAL[1] - by
        gn = math.hypot(gx, gy)
        if gn > 1e-6:
            # DOMINANTE: velocidad de la pelota hacia el arco rival, CON SIGNO
            # (remate acerca → +; despeje hacia atrás/autogol → −).
            r += BALL_VEL_W * float(np.clip((f.ball.v_x * gx + f.ball.v_y * gy) / gn / V_MAX, -1.0, 1.0))
        dg = gn
        if self._prev_bg is not None:
            r += BALL_PROGRESS_W * (self._prev_bg - dg)
        self._prev_bg = dg
        # Cercanía a la pelota (NIVEL, como Brandão): paga sostenidamente por estar
        # encima de la pelota — el tirón que le faltaba a la política pasiva del 1a.
        dmin = min(math.hypot(f.robots_blue[rid].x - bx, f.robots_blue[rid].y - by) for rid in CONTROLLED)
        r += BALL_PROX_W * max(0.0, 1.0 - dmin / BALL_PROX_DMAX)
        # Comportamiento de juego (§9.2): penaliza robots de campo quietos > 5 s.
        for rid in CONTROLLED:
            rb = f.robots_blue[rid]
            ref = self._idle_ref.get(rid)
            if ref is None or math.hypot(rb.x - ref[0], rb.y - ref[1]) > IDLE_DIST:
                self._idle_ref[rid] = (rb.x, rb.y)
                self._idle[rid] = 0
            else:
                self._idle[rid] += 1
                if self._idle[rid] >= IDLE_TICKS:
                    r += IDLE_PEN
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
