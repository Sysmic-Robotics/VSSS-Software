"""
Callbacks de entrenamiento: logging de componentes de recompensa a TensorBoard
y métricas de gol por episodio. Resuelve el faltante de visibilidad de la
Fase 1 inicial.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import torch
from stable_baselines3.common.callbacks import BaseCallback


class ClipLogStdCallback(BaseCallback):
    """Limita el log_std de la política tras cada rollout para que el desvío de
    las acciones (std) NO explote — fue la causa de la divergencia del self-play
    (std creció 1 → 1e14). Cap duro: std ∈ [e^min, e^max] ≈ [0.14, 2.0]."""

    def __init__(self, min_log_std: float = -2.0, max_log_std: float = 0.7, verbose: int = 0):
        super().__init__(verbose)
        self.min_log_std = min_log_std
        self.max_log_std = max_log_std

    def _on_rollout_end(self) -> None:
        ls = getattr(self.model.policy, "log_std", None)
        if ls is not None:
            with torch.no_grad():
                ls.clamp_(self.min_log_std, self.max_log_std)

    def _on_step(self) -> bool:
        return True


class RewardComponentCallback(BaseCallback):
    """Promedia y loguea los componentes de recompensa y la tasa de gol.

    Lee `info["reward_components"]` y `info["goal_scored"]` que emite el env en
    cada step, los acumula y los publica a TensorBoard cada `log_freq` steps.
    """

    def __init__(self, log_freq: int = 2048, verbose: int = 0):
        super().__init__(verbose)
        self.log_freq = log_freq
        self._acc: dict[str, float] = {}
        self._n = 0
        self._goals = 0
        self._concedes = 0
        self._episodes = 0

    def _on_step(self) -> bool:
        infos = self.locals.get("infos", [])
        dones = self.locals.get("dones", [])
        for i, info in enumerate(infos):
            comp = info.get("reward_components")
            if comp:
                for k, v in comp.items():
                    self._acc[k] = self._acc.get(k, 0.0) + float(v)
                self._n += 1
            if i < len(dones) and dones[i]:
                self._episodes += 1
                if info.get("goal_scored"):
                    self._goals += 1
                if info.get("conceded"):
                    self._concedes += 1

        if self.num_timesteps % self.log_freq < self.training_env.num_envs:
            if self._n > 0:
                for k, v in self._acc.items():
                    self.logger.record(f"reward/{k}", v / self._n)
            if self._episodes > 0:
                self.logger.record("metrics/goal_rate", self._goals / self._episodes)
                self.logger.record("metrics/concede_rate", self._concedes / self._episodes)
            self._acc.clear()
            self._n = 0
            self._goals = self._concedes = self._episodes = 0
        return True


class SelfPlayCallback(BaseCallback):
    """Self-play con pool de snapshots congelados.

    Cada `snapshot_freq` pasos guarda una copia de la política actual al pool y
    apunta a todos los envs (vía VecEnv.env_method) a un snapshot muestreado del
    pool. El env, por episodio, usa ese snapshot o rule-based (20%, anti-forgetting,
    controlado por SELFPLAY_RB_PROB en el env). Así el oponente va escalando con
    la política y mantiene diversidad de versiones pasadas.
    """

    def __init__(self, pool_dir: str, snapshot_freq: int = 300_000, seed: int = 0, verbose: int = 0):
        super().__init__(verbose)
        self.pool_dir = Path(pool_dir)
        self.snapshot_freq = snapshot_freq
        self._last_snap = 0
        self._pool: list[str] = []
        self._rng = np.random.default_rng(seed)

    def _on_training_start(self) -> None:
        self.pool_dir.mkdir(parents=True, exist_ok=True)

    def _on_step(self) -> bool:
        if self.num_timesteps - self._last_snap >= self.snapshot_freq:
            self._last_snap = self.num_timesteps
            path = self.pool_dir / f"snap_{self.num_timesteps}.zip"
            self.model.save(str(path))
            self._pool.append(str(path))
            # Muestrear un oponente del pool (sesgo a versiones recientes: 50%
            # la última, 50% una histórica uniforme).
            if len(self._pool) == 1 or self._rng.random() < 0.5:
                chosen = self._pool[-1]
            else:
                chosen = self._pool[int(self._rng.integers(0, len(self._pool)))]
            self.training_env.env_method("set_opponent_snapshot", chosen)
            if self.verbose:
                print(f"[selfplay] pool={len(self._pool)} oponente={Path(chosen).name}")
        return True
