"""
Evaluación de una política entrenada.

Uso:
    python eval.py --checkpoint checkpoints/phase1_final.zip --episodes 100

Reporta:
- Tasa de goles (KPI del documento INF398)
- Tiempo medio por episodio
- Desviación estándar entre episodios (estabilidad — KPI del documento)
- Recompensa media
"""

from __future__ import annotations

import argparse
import statistics
from pathlib import Path

from stable_baselines3 import PPO

from vsss_rl.env_phase1 import VsssPhase1Env


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", required=True, type=Path)
    p.add_argument("--episodes", type=int, default=100)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--render", action="store_true")
    args = p.parse_args()

    render_mode = "human" if args.render else None
    env = VsssPhase1Env(render_mode=render_mode)
    model = PPO.load(str(args.checkpoint), env=env)

    rewards: list[float] = []
    lengths: list[int] = []
    goals: list[int] = []

    for ep in range(args.episodes):
        obs, _ = env.reset(seed=args.seed + ep)
        done = False
        total_r = 0.0
        steps = 0
        scored = 0
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, r, terminated, truncated, info = env.step(action)
            total_r += float(r)
            steps += 1
            done = terminated or truncated
            if info.get("goal_scored"):
                scored = 1
        rewards.append(total_r)
        lengths.append(steps)
        goals.append(scored)

    env.close()

    goal_rate = sum(goals) / len(goals)
    print("── Evaluación ─────────────────────────────")
    print(f"Checkpoint: {args.checkpoint}")
    print(f"Episodios: {args.episodes}")
    print(f"Tasa de goles:           {goal_rate:.1%}")
    print(f"Recompensa media:        {statistics.mean(rewards):.3f}")
    print(f"Recompensa std:          {statistics.stdev(rewards):.3f}")
    print(f"Largo medio (steps):     {statistics.mean(lengths):.1f}")
    print(f"Largo std (steps):       {statistics.stdev(lengths):.1f}")
    print(f"Tiempo medio (s @ 60Hz): {statistics.mean(lengths) / 60:.2f}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
