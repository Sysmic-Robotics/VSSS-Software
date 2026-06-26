"""
Entorno VSSS unificado y parametrizable — base del currículo 1v0 → 1v1 → 3v3.

Contrato de acción (espejo del SkillChoice congelado del equipo): por cada
robot controlado, la política emite `[skill_sel, tx, ty]` ∈ [-1, 1]³:
- `skill_sel` se discretiza por binning en 4 → SkillId {GoTo, FacePoint, ChaseBall, Spin}.
- `(tx, ty)` se mapea a coordenadas de campo (m): `tx·FIELD_HALF_X`, `ty·FIELD_HALF_Y`.

Para N robots controlados, action = Box(3·N). Esto reproduce el tensor (N, 3)
del contrato del equipo: cada fila `[skill_id, target_x, target_y]`. El deploy
(`RlCoach`) lee exactamente esto.

Observación: 52 floats (contrato Rust, observation.py), desde el marco del
equipo controlado (ataca hacia +X).

Frame-skip K=6: un step del env = una decisión de la política = 6 ticks de
física a 60 Hz (igual que COACH_DECISION_PERIOD del motor).

La política es CENTRALIZADA (una obs → choices de todos los robots de campo
controlados), que es exactamente lo que espera `Coach::decide(obs) -> Vec<SkillChoice>`.
El portero propio y los oponentes los maneja un RuleBasedCoach (o, en self-play,
un snapshot congelado seteado vía set_opponent_policy).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from .engine_constants import (
    BALL_RADIUS,
    COACH_DECISION_PERIOD,
    DT,
    FIELD_HALF_X,
    FIELD_HALF_Y,
    GOAL_HALF_WIDTH,
    MAX_ANGULAR_SPEED,
    MAX_LINEAR_SPEED,
    ROBOT_RADIUS,
)
from .observation import FLAT_SIZE, Observation, normalize_ball, normalize_robot
from .rule_based import RuleBasedCoach
from .skills import SkillId, dispatch_skill, _wrap_pi

# ── Pesos de recompensa (time penalty reducido vs Fase 1 inicial) ────────────
GOAL_REWARD = 10.0
CONCEDE_PENALTY = -10.0
OOB_PENALTY = -0.5
TIME_PENALTY = -0.001          # por tick de física
BALL_PROGRESS_W = 3.0          # × Δ(distancia pelota→arco rival)
ROBOT_TO_BALL_W = 0.3          # × Δ(distancia robot más cercano→pelota)
BALL_VEL_TO_GOAL_W = 0.1       # × componente de v_pelota hacia el arco rival
SURVIVE_REWARD = 0.05          # defensa (arquero): premio por decisión sin conceder
DEFENSE_PROGRESS_W = 1.5       # defensa: × Δ(distancia pelota→arco propio), + si se aleja
SELFPLAY_RB_PROB = 0.2         # self-play: prob. de usar rule-based (anti-forgetting)

OUT_X = 0.74
OUT_Y = 0.64

# Pelota
BALL_FRICTION_PER_TICK = 0.94
IMPULSE_TRANSFER = 0.7
SPIN_KICK_GAIN = 4.0   # amplifica el impulso tangencial por giro (representa el
                       # canto del robot; sin esto un robot circular no "patea" girando)

# Áreas de meta (reglamento LARC: rectángulo 70x15 cm frente a cada arco).
OWN_AREA = (-FIELD_HALF_X, -FIELD_HALF_X + 0.15, -0.35, 0.35)   # arco propio (−X)
RIVAL_AREA = (FIELD_HALF_X - 0.15, FIELD_HALF_X, -0.35, 0.35)   # arco rival (+X)
ILLEGAL_AREA_PENALTY = -0.3   # por decisión: 2+ robots propios en un área de meta


@dataclass
class RobotBody:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0
    spinning: bool = False  # True si este tick ejecuta la skill Spin (para el pelotazo)


@dataclass
class EnvConfig:
    """Configuración de escenario. Constructores abajo para cada fase."""

    controlled_field_ids: tuple[int, ...] = (0,)   # own ids que controla la política
    own_gk_id: int | None = None                   # own id portero (rule-based)
    opp_active_ids: tuple[int, ...] = ()            # opp ids activos
    opponent_mode: str = "none"                     # none | rulebased | selfplay
    max_seconds: float = 30.0
    domain_randomization: bool = False
    reward_mode: str = "attack"                     # attack (campo) | defense (arquero)
    randomize_opponents: bool = False               # samplear nº de oponentes por episodio (escenario mixto)
    gk_area: tuple[float, float, float, float] | None = None  # (xmin,xmax,ymin,ymax) confina al arquero (id 2)


def phase1_config() -> EnvConfig:
    """1v0 — atacante vs arco vacío."""
    return EnvConfig(controlled_field_ids=(0,), own_gk_id=None, opp_active_ids=(), opponent_mode="none")


def phase2_config() -> EnvConfig:
    """1v1 — atacante vs portero rival rule-based."""
    return EnvConfig(controlled_field_ids=(0,), own_gk_id=None, opp_active_ids=(2,), opponent_mode="rulebased")


def phase_2v1_config() -> EnvConfig:
    """2v1 — 2 jugadores de campo controlados (cooperación) vs 1 defensor.
    Acción Box(6). Aquí 'nacen las jugadas' (pase/desmarque) antes del 3v3."""
    return EnvConfig(
        controlled_field_ids=(0, 1),
        own_gk_id=None,
        opp_active_ids=(2,),
        opponent_mode="rulebased",
        max_seconds=30.0,
    )


def phase_mixed_config(opponent_mode: str = "rulebased") -> EnvConfig:
    """Escenario MIXTO — 2 jugadores de campo controlados (Box 6) + portero
    propio rule-based, contra un nº de oponentes RANDOMIZADO (1-3) por episodio.
    Cubre 2v1/2v2/2v3 con una sola red robusta. Es el escenario del run largo."""
    return EnvConfig(
        controlled_field_ids=(0, 1),
        own_gk_id=2,
        opp_active_ids=(0, 1, 2),
        opponent_mode=opponent_mode,
        max_seconds=40.0,
        randomize_opponents=True,
        gk_area=GK_AREA,
    )


def phase3_config(opponent_mode: str = "rulebased") -> EnvConfig:
    """3v3 fijo — 2 jugadores de campo controlados + portero propio rule-based,
    contra 3 oponentes. opponent_mode: rulebased o selfplay."""
    return EnvConfig(
        controlled_field_ids=(0, 1),
        own_gk_id=2,
        opp_active_ids=(0, 1, 2),
        opponent_mode=opponent_mode,
        max_seconds=40.0,
        gk_area=GK_AREA,
    )


def goalkeeper_config() -> EnvConfig:
    """Arquero — la política CONTROLA el arquero (id 2) defendiendo el arco
    propio (−X) contra un atacante rule-based. Recompensa de DEFENSA.
    Red separada (arquitectura asimétrica). Acción Box(3)."""
    return EnvConfig(
        controlled_field_ids=(2,),
        own_gk_id=None,
        opp_active_ids=(0,),
        opponent_mode="rulebased",
        max_seconds=20.0,
        reward_mode="defense",
        gk_area=GK_AREA,
    )


# Arcos por equipo (own ataca +X)
OWN_ATTACK_GOAL = (FIELD_HALF_X, 0.0)
OWN_OWN_GOAL = (-FIELD_HALF_X, 0.0)

# Área del arquero propio (delante del arco en −X). El arquero (id 2) se clampea
# a esta caja: NO puede salir de su área (restricción dura, regla VSSS).
# ⚠️ DIMENSIONES POR CONFIRMAR contra el reglamento. Default razonable:
# 0.15 m de profundidad desde la línea de fondo, 0.70 m de ancho.
GK_AREA = (-FIELD_HALF_X, -FIELD_HALF_X + 0.15, -0.35, 0.35)


class VsssSoccerEnv(gym.Env):
    """Env unificado para el currículo. Ver docstring del módulo."""

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 60}

    def __init__(self, config: EnvConfig | None = None, render_mode: str | None = None, seed: int | None = None):
        super().__init__()
        self.cfg = config or phase1_config()
        self.render_mode = render_mode
        self._rng = np.random.default_rng(seed)

        n_ctrl = len(self.cfg.controlled_field_ids)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(3 * n_ctrl,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-2.0, high=2.0, shape=(FLAT_SIZE,), dtype=np.float32)

        self._max_ticks = int(self.cfg.max_seconds * 60)
        # Coaches rule-based: propio (para el portero) y oponente.
        self._own_coach = RuleBasedCoach(attack_goal=OWN_ATTACK_GOAL, own_goal=OWN_OWN_GOAL)
        self._opp_coach = RuleBasedCoach(attack_goal=OWN_OWN_GOAL, own_goal=OWN_ATTACK_GOAL)
        # Self-play: el oponente de campo carga un snapshot congelado desde disco
        # (compatible con SubprocVecEnv — cada worker carga su propia copia).
        self._opp_model = None
        self._opp_model_path: str | None = None
        self._use_rb_this_ep: bool = True  # por episodio: usar rule-based vs snapshot

        self._own: dict[int, RobotBody] = {}
        self._opp: dict[int, RobotBody] = {}
        self._ball = np.zeros(2, dtype=np.float32)
        self._ball_vel = np.zeros(2, dtype=np.float32)
        self._ticks = 0

    # ── API pública para self-play ───────────────────────────────────────
    def set_opponent_snapshot(self, path: str | None) -> None:
        """Apunta el oponente de campo a un snapshot congelado en disco. El
        modelo se (re)carga perezosamente. None → siempre rule-based.
        Llamado por SelfPlayCallback vía VecEnv.env_method()."""
        self._opp_model_path = path
        self._opp_model = None  # forzar recarga en el próximo uso

    # ── API gym ──────────────────────────────────────────────────────────
    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)

        self._own.clear()
        self._opp.clear()

        # Self-play: por episodio, usar rule-based (anti-forgetting) o el snapshot.
        self._use_rb_this_ep = (
            self._opp_model_path is None or float(self._rng.random()) < SELFPLAY_RB_PROB
        )

        own_ids = set(self.cfg.controlled_field_ids) | ({self.cfg.own_gk_id} if self.cfg.own_gk_id is not None else set())
        for rid in own_ids:
            self._own[rid] = self._spawn_own(rid)
        # Escenario mixto: samplear cuántos oponentes activar este episodio (1..N).
        opp_ids = self.cfg.opp_active_ids
        if self.cfg.randomize_opponents and opp_ids:
            k = int(self._rng.integers(1, len(opp_ids) + 1))
            opp_ids = tuple(opp_ids[:k])
        for rid in opp_ids:
            self._opp[rid] = self._spawn_opp(rid)

        self._ball = np.array(
            [self._rng.uniform(-0.10, 0.30), self._rng.uniform(-0.30, 0.30)], dtype=np.float32
        )
        self._ball_vel = np.zeros(2, dtype=np.float32)
        self._ticks = 0
        return self._build_obs(), self._info()

    def step(self, action: np.ndarray):
        action = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
        # Decodificar SkillChoice por robot controlado.
        ctrl_choices = self._decode_action(action)

        prev_ball_to_goal = self._dist_ball_to(OWN_ATTACK_GOAL)
        prev_ball_to_own = self._dist_ball_to(OWN_OWN_GOAL)
        prev_min_robot_ball = self._min_ctrl_dist_to_ball()
        defense = self.cfg.reward_mode == "defense"

        reward = 0.0
        comp = {"goal": 0.0, "concede": 0.0, "oob": 0.0, "time": 0.0,
                "ball_progress": 0.0, "robot_to_ball": 0.0, "ball_vel_to_goal": 0.0,
                "survive": 0.0, "defense_progress": 0.0, "illegal_area": 0.0}
        terminated = False
        goal_scored = conceded = False

        # Oponente y portero deciden 1× por decisión (frame-skip), igual que el coach.
        opp_choices = self._opponent_choices()
        gk_choice = self._own_gk_choice()

        for _ in range(COACH_DECISION_PERIOD):
            self._apply_controlled(ctrl_choices)
            self._apply_gk(gk_choice)
            self._apply_opponent(opp_choices)
            self._confine_goalkeeper()
            self._step_ball()
            # En defensa NO hay time penalty (incentivaría conceder rápido para
            # terminar el episodio); se reemplaza por survive reward abajo.
            if not defense:
                comp["time"] += TIME_PENALTY

            bx, by = float(self._ball[0]), float(self._ball[1])
            # Línea de fondo rival (+X): gol si está en la boca del arco, si no out.
            if bx >= FIELD_HALF_X:
                if abs(by) < GOAL_HALF_WIDTH:
                    goal_scored = True
                    comp["goal"] = GOAL_REWARD
                    terminated = True
                    break
                comp["oob"] += OOB_PENALTY
                self._reset_ball_center()
            # Línea de fondo propia (−X): gol en contra si está en la boca, si no out.
            elif bx <= -FIELD_HALF_X:
                if abs(by) < GOAL_HALF_WIDTH:
                    conceded = True
                    comp["concede"] = CONCEDE_PENALTY
                    terminated = True
                    break
                comp["oob"] += OOB_PENALTY
                self._reset_ball_center()
            # Líneas laterales (±Y).
            elif abs(by) >= FIELD_HALF_Y:
                comp["oob"] += OOB_PENALTY
                self._reset_ball_center()

        # Shaping (por decisión).
        if not terminated:
            if defense:
                # Arquero: premio por sobrevivir + por mantener la pelota lejos
                # del arco propio (Δ positivo si la pelota se aleja). El robot
                # más cercano a la pelota es el propio arquero.
                comp["survive"] = SURVIVE_REWARD
                now_ball_to_own = self._dist_ball_to(OWN_OWN_GOAL)
                comp["defense_progress"] = DEFENSE_PROGRESS_W * (now_ball_to_own - prev_ball_to_own)
                comp["robot_to_ball"] = ROBOT_TO_BALL_W * (
                    prev_min_robot_ball - self._min_ctrl_dist_to_ball()
                )
            else:
                now_ball_to_goal = self._dist_ball_to(OWN_ATTACK_GOAL)
                comp["ball_progress"] = BALL_PROGRESS_W * (prev_ball_to_goal - now_ball_to_goal)
                now_min_robot_ball = self._min_ctrl_dist_to_ball()
                comp["robot_to_ball"] = ROBOT_TO_BALL_W * (prev_min_robot_ball - now_min_robot_ball)
                bvx, bvy = float(self._ball_vel[0]), float(self._ball_vel[1])
                gx, gy = OWN_ATTACK_GOAL[0] - self._ball[0], OWN_ATTACK_GOAL[1] - self._ball[1]
                gn = math.hypot(gx, gy)
                if gn > 1e-6:
                    dot = (bvx * gx + bvy * gy) / gn
                    comp["ball_vel_to_goal"] = BALL_VEL_TO_GOAL_W * max(0.0, dot)

        # Regla VSSS: máx. 1 robot propio por área de meta (2+ = defensa/ataque
        # ilegal → penal). Penalizamos cada área en infracción.
        violations = (1 if self._own_in_area(OWN_AREA) >= 2 else 0) + \
                     (1 if self._own_in_area(RIVAL_AREA) >= 2 else 0)
        comp["illegal_area"] = ILLEGAL_AREA_PENALTY * violations

        reward = sum(comp.values())
        self._ticks += COACH_DECISION_PERIOD
        truncated = self._ticks >= self._max_ticks

        info = self._info()
        info["goal_scored"] = goal_scored
        info["conceded"] = conceded
        info["reward_components"] = comp
        return self._build_obs(), float(reward), bool(terminated), bool(truncated), info

    def close(self):
        pass

    # ── Spawns ───────────────────────────────────────────────────────────
    def _spawn_own(self, rid: int) -> RobotBody:
        if rid == self.cfg.own_gk_id:
            return RobotBody(x=-FIELD_HALF_X + 0.12, y=float(self._rng.uniform(-0.1, 0.1)), theta=0.0)
        return RobotBody(
            x=float(self._rng.uniform(-0.55, -0.10)),
            y=float(self._rng.uniform(-0.40, 0.40)),
            theta=float(self._rng.uniform(-math.pi, math.pi)),
        )

    def _spawn_opp(self, rid: int) -> RobotBody:
        # rid 2 = portero del oponente (defiende +X); otros, campo.
        if rid == 2:
            return RobotBody(x=FIELD_HALF_X - 0.12, y=float(self._rng.uniform(-0.1, 0.1)), theta=math.pi)
        return RobotBody(
            x=float(self._rng.uniform(0.10, 0.55)),
            y=float(self._rng.uniform(-0.40, 0.40)),
            theta=float(self._rng.uniform(-math.pi, math.pi)),
        )

    def _reset_ball_center(self) -> None:
        self._ball = np.array([self._rng.uniform(-0.05, 0.05), self._rng.uniform(-0.05, 0.05)], dtype=np.float32)
        self._ball_vel = np.zeros(2, dtype=np.float32)

    # ── Decodificación de acción ─────────────────────────────────────────
    def _decode_action(self, action: np.ndarray) -> list[tuple[int, int, float, float]]:
        out = []
        for j, rid in enumerate(self.cfg.controlled_field_ids):
            a = action[3 * j: 3 * j + 3]
            skill_id = self._bin_skill(float(a[0]))
            tx = float(a[1]) * FIELD_HALF_X
            ty = float(a[2]) * FIELD_HALF_Y
            out.append((rid, skill_id, tx, ty))
        return out

    @staticmethod
    def _bin_skill(v: float) -> int:
        idx = int((min(1.0, max(-1.0, v)) + 1.0) / 2.0 * SkillId.count())
        return min(SkillId.count() - 1, max(0, idx))

    # ── Aplicación de comandos ───────────────────────────────────────────
    def _apply_controlled(self, choices: list[tuple[int, int, float, float]]) -> None:
        bx, by = float(self._ball[0]), float(self._ball[1])
        for rid, skill_id, tx, ty in choices:
            body = self._own[rid]
            body.spinning = (skill_id == int(SkillId.SPIN))
            v, omega = dispatch_skill(skill_id, tx, ty, body.x, body.y, body.theta, bx, by)
            self._integrate(body, v, omega)

    def _apply_gk(self, gk_choice) -> None:
        if gk_choice is None or self.cfg.own_gk_id is None:
            return
        rid = self.cfg.own_gk_id
        body = self._own[rid]
        body.spinning = (gk_choice.skill_id == int(SkillId.SPIN))
        bx, by = float(self._ball[0]), float(self._ball[1])
        v, omega = dispatch_skill(gk_choice.skill_id, gk_choice.target_x, gk_choice.target_y,
                                  body.x, body.y, body.theta, bx, by)
        self._integrate(body, v, omega)

    def _apply_opponent(self, opp_choices) -> None:
        if not self.cfg.opp_active_ids:
            return
        bx, by = float(self._ball[0]), float(self._ball[1])
        for rid, (skill_id, tx, ty) in opp_choices.items():
            body = self._opp[rid]
            body.spinning = (skill_id == int(SkillId.SPIN))
            v, omega = dispatch_skill(skill_id, tx, ty, body.x, body.y, body.theta, bx, by)
            self._integrate(body, v, omega)

    def _confine_goalkeeper(self) -> None:
        """Restricción dura: el arquero propio (id 2) no puede salir de su área.
        Clampa su posición a `gk_area`. Aplica tanto al arquero entrenado
        (modo defensa) como al rule-based en campo (3v3/mixto)."""
        area = self.cfg.gk_area
        if area is None or 2 not in self._own:
            return
        xmin, xmax, ymin, ymax = area
        b = self._own[2]
        b.x = float(min(max(b.x, xmin), xmax))
        b.y = float(min(max(b.y, ymin), ymax))

    def _integrate(self, body: RobotBody, v: float, omega: float) -> None:
        omega = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, omega))
        body.theta = _wrap_pi(body.theta + omega * DT)
        ct, st = math.cos(body.theta), math.sin(body.theta)
        body.vx, body.vy, body.omega = v * ct, v * st, omega
        body.x = float(np.clip(body.x + body.vx * DT, -OUT_X, OUT_X))
        body.y = float(np.clip(body.y + body.vy * DT, -OUT_Y, OUT_Y))

    def _step_ball(self) -> None:
        # Contacto robot-pelota con resolución POSICIONAL: cuando el robot
        # solapa la pelota, la empuja hacia afuera (al borde de contacto) y le
        # transfiere su velocidad. Modela el "shove" del robot diferencial sin
        # dribbler, que es como se empuja la pelota en VSSS real.
        contact = ROBOT_RADIUS + BALL_RADIUS
        for body in list(self._own.values()) + list(self._opp.values()):
            dx, dy = float(self._ball[0]) - body.x, float(self._ball[1]) - body.y
            dist = math.hypot(dx, dy)
            if dist < contact:
                if dist > 1e-6:
                    nx, ny = dx / dist, dy / dist
                else:
                    nx, ny = math.cos(body.theta), math.sin(body.theta)
                # Empujar la pelota al borde de contacto a lo largo de robot→pelota.
                self._ball[0] = body.x + nx * contact
                self._ball[1] = body.y + ny * contact
                # (1) Empuje por TRASLACIÓN (shove) — modelo probado: transfiere
                #     la velocidad del robot a la pelota.
                # (2) SPIN-KICK: impulso TANGENCIAL por velocidad angular (ω × radio),
                #     lo que hace el "pelotazo" al girar; sin esto Spin no patea.
                tang_x, tang_y = -ny, nx  # tangente (sentido CCW para ω>0)
                # El pelotazo tangencial SOLO aplica al robot que ejecuta la skill
                # Spin — no al giro de ajuste de heading de GoTo/ChaseBall (que si
                # no, desviaría la pelota durante el empuje normal).
                spin = body.omega * ROBOT_RADIUS * SPIN_KICK_GAIN if body.spinning else 0.0
                self._ball_vel[0] = body.vx * IMPULSE_TRANSFER + tang_x * spin + self._ball_vel[0] * 0.3
                self._ball_vel[1] = body.vy * IMPULSE_TRANSFER + tang_y * spin + self._ball_vel[1] * 0.3
        self._ball_vel *= BALL_FRICTION_PER_TICK
        self._ball = self._ball + self._ball_vel * DT

    # ── Choices de portero y oponente ────────────────────────────────────
    def _own_gk_choice(self):
        if self.cfg.own_gk_id is None:
            return None
        ball = (float(self._ball[0]), float(self._ball[1]))
        return self._own_coach.goalkeeper_choice(ball)

    def _opponent_choices(self) -> dict[int, tuple[int, float, float]]:
        # Usa los oponentes realmente spawneados este episodio (el escenario
        # mixto puede activar 1-3), no la config fija.
        active = sorted(self._opp.keys())
        if not active or self.cfg.opponent_mode == "none":
            return {}
        ball = (float(self._ball[0]), float(self._ball[1]))
        out: dict[int, tuple[int, float, float]] = {}
        if self.cfg.opponent_mode == "selfplay" and not self._use_rb_this_ep:
            # Cargar el snapshot perezosamente (una vez por worker/ruta).
            if self._opp_model is None and self._opp_model_path:
                try:
                    from stable_baselines3 import PPO
                    self._opp_model = PPO.load(self._opp_model_path, device="cpu")
                except Exception:
                    self._opp_model = None
            if self._opp_model is not None:
                # El snapshot ve la obs en SU marco (espejo en X) y controla campo.
                act, _ = self._opp_model.predict(self._build_obs_opp_frame(), deterministic=True)
                act = np.asarray(act, dtype=np.float32).flatten()
                field_opp = [r for r in active if r != 2]
                for j, rid in enumerate(field_opp):
                    a = act[3 * j: 3 * j + 3]
                    sid = self._bin_skill(float(a[0]))
                    # des-espejar el target del marco opp al marco global
                    tx = -float(a[1]) * FIELD_HALF_X
                    ty = float(a[2]) * FIELD_HALF_Y
                    out[rid] = (sid, tx, ty)
                if 2 in self._opp:
                    gk = self._opp_coach.goalkeeper_choice(ball)
                    out[2] = (gk.skill_id, gk.target_x, gk.target_y)
                return out
        # rule-based: cada opp activo toma su rol (0 atacante, 1 soporte, 2 portero)
        opp_positions = [
            (self._opp[r].x, self._opp[r].y) if r in self._opp else (0.0, 0.0) for r in range(3)
        ]
        choices = self._opp_coach.decide(ball, opp_positions)
        for rid in active:
            c = choices[rid]
            out[rid] = (c.skill_id, c.target_x, c.target_y)
        return out

    # ── Observación ──────────────────────────────────────────────────────
    def _build_obs(self) -> np.ndarray:
        return self._build_obs_frame(mirror=False)

    def _build_obs_opp_frame(self) -> np.ndarray:
        return self._build_obs_frame(mirror=True)

    def _build_obs_frame(self, mirror: bool) -> np.ndarray:
        s = -1.0 if mirror else 1.0
        obs = Observation(own_team=0)
        obs.ball = normalize_ball(s * float(self._ball[0]), float(self._ball[1]),
                                  s * float(self._ball_vel[0]), float(self._ball_vel[1]))
        # En marco espejo, "own" y "opp" se intercambian.
        own_src = self._opp if mirror else self._own
        opp_src = self._own if mirror else self._opp
        for rid in range(3):
            if rid in own_src:
                b = own_src[rid]
                obs.own_robots[rid] = normalize_robot(
                    s * b.x, b.y, _wrap_pi(math.pi - b.theta) if mirror else b.theta,
                    s * b.vx, b.vy, b.omega, active=True)
        for rid in range(3):
            if rid in opp_src:
                b = opp_src[rid]
                obs.opp_robots[rid] = normalize_robot(
                    s * b.x, b.y, _wrap_pi(math.pi - b.theta) if mirror else b.theta,
                    s * b.vx, b.vy, b.omega, active=True)
        return obs.to_flat()

    # ── Helpers de recompensa ────────────────────────────────────────────
    def _dist_ball_to(self, goal: tuple[float, float]) -> float:
        return math.hypot(self._ball[0] - goal[0], self._ball[1] - goal[1])

    def _own_in_area(self, area: tuple[float, float, float, float]) -> int:
        """Cuántos robots propios están dentro de la caja `area`."""
        xmin, xmax, ymin, ymax = area
        return sum(1 for b in self._own.values()
                   if xmin <= b.x <= xmax and ymin <= b.y <= ymax)

    def _min_ctrl_dist_to_ball(self) -> float:
        ds = [math.hypot(self._own[r].x - self._ball[0], self._own[r].y - self._ball[1])
              for r in self.cfg.controlled_field_ids if r in self._own]
        return min(ds) if ds else 0.0

    def _info(self) -> dict[str, Any]:
        return {
            "ticks": self._ticks,
            "ball_pos": self._ball.copy(),
            "own": {r: (b.x, b.y, b.theta) for r, b in self._own.items()},
            "opp": {r: (b.x, b.y, b.theta) for r, b in self._opp.items()},
        }
