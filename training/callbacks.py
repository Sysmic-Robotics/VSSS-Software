"""
Callbacks de entrenamiento: logging de componentes de recompensa a TensorBoard
y métricas de gol por episodio. Resuelve el faltante de visibilidad de la
Fase 1 inicial.
"""

from __future__ import annotations

import numpy as np
from stable_baselines3.common.callbacks import BaseCallback


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
