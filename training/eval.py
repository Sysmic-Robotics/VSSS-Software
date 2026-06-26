"""
Evaluación cuantitativa de una política entrenada sobre el env skill-based.
Reporta los KPIs del documento INF398: tasa de goles, tiempo medio, estabilidad.

Uso:
    python eval.py --checkpoint checkpoints/phase1_final.zip --phase 1 --episodes 100
    python eval.py --checkpoint checkpoints/phase3_final.zip --phase 3 --episodes 100
"""

from __future__ import annotations

import argparse
import statistics
from pathlib import Path

from stable_baselines3 import PPO

from vsss_rl.soccer_env import VsssSoccerEnv, phase1_config, phase2_config, phase3_config

PHASE_CONFIGS = {1: phase1_config, 2: phase2_config, 3: phase3_config}


def evaluate(model, env, episodes: int, seed: int) -> dict:
    rewards, lengths, goals, concedes = [], [], [], []
    for ep in range(episodes):
        obs, _ = env.reset(seed=seed + ep)
        done = False
        total_r, steps, scored, conceded = 0.0, 0, 0, 0
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, info = env.step(action)
            total_r += float(r)
            steps += 1
            if info.get("goal_scored"):
                scored = 1
            if info.get("conceded"):
                conceded = 1
            done = term or trunc
        rewards.append(total_r)
        lengths.append(steps)
        goals.append(scored)
        concedes.append(conceded)
    return {
        "goal_rate": sum(goals) / episodes,
        "concede_rate": sum(concedes) / episodes,
        "reward_mean": statistics.mean(rewards),
        "reward_std": statistics.stdev(rewards) if episodes > 1 else 0.0,
        "len_mean": statistics.mean(lengths),
        "len_std": statistics.stdev(lengths) if episodes > 1 else 0.0,
    }


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", required=True, type=Path)
    p.add_argument("--phase", type=int, choices=[1, 2, 3], default=1)
    p.add_argument("--episodes", type=int, default=100)
    p.add_argument("--seed", type=int, default=42)
    args = p.parse_args()

    env = VsssSoccerEnv(config=PHASE_CONFIGS[args.phase]())
    model = PPO.load(str(args.checkpoint), device="cpu")
    m = evaluate(model, env, args.episodes, args.seed)
    env.close()

    decision_hz = 10.0  # frame-skip K=6 a 60 Hz
    print(f"── Evaluación Fase {args.phase} ({args.episodes} episodios) ──")
    print(f"Checkpoint:          {args.checkpoint.name}")
    print(f"Tasa de goles:       {m['goal_rate']:.1%}")
    print(f"Tasa de goles recib: {m['concede_rate']:.1%}")
    print(f"Recompensa media:    {m['reward_mean']:.3f}  (std {m['reward_std']:.3f})")
    print(f"Largo medio:         {m['len_mean']:.1f} decisiones  (std {m['len_std']:.1f})")
    print(f"Tiempo medio:        {m['len_mean'] / decision_hz:.2f} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
