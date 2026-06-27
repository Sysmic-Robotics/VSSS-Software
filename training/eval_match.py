"""
Evaluación por WIN-TENDENCY contra oponentes FIJOS (no self-play).

Reemplaza el goal_rate de self-play como KPI principal. La investigación VSSS
(Brandão evalúa 100 partidos vs cada rival fijo; Coach-RL usa 30) converge en:
**baselines fijos + muchos episodios + métricas agregadas.** El goal_rate de
self-play es ruidoso y engañoso (equilibrio ~50% contra una copia de sí mismo).

Por cada oponente FIJO corre N episodios y reporta:
- scored%   : % de episodios donde NUESTRO equipo metió gol
- conceded% : % donde el rival metió
- draw%     : % sin gol (timeout)
- net       : scored% - conceded%  (proxy de win-tendency; >0 = dominamos)

Oponentes fijos diversos (recomendación: heurístico + agresivo + snapshot):
- rulebased : RuleBasedCoach (heurístico clásico).
- ballchaser: todos los rivales persiguen la pelota (agresivo).
- snapshot  : opcional, un modelo RL congelado (--opp-snapshot PATH).

Uso:
    python eval_match.py --checkpoint checkpoints/phasemixedsp_final.zip --episodes 100
    python eval_match.py --checkpoint checkpoints/phasemixedsp_final.zip --opp-snapshot checkpoints/algun_snap.zip
"""

from __future__ import annotations

import sys

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
sys.stderr.reconfigure(encoding="utf-8", errors="replace")

import argparse
from pathlib import Path

from stable_baselines3 import PPO

from vsss_rl.soccer_env import VsssSoccerEnv, phase3_config


def eval_vs(model, opponent_mode: str, episodes: int, seed: int,
            snapshot: str | None = None) -> dict:
    """Corre `episodes` episodios vs un oponente FIJO y agrega resultados."""
    cfg = phase3_config(opponent_mode)
    env = VsssSoccerEnv(config=cfg)
    if snapshot is not None:
        env.set_opponent_snapshot(snapshot)
    scored = conceded = draws = 0
    for ep in range(episodes):
        obs, _ = env.reset(seed=seed + ep)
        # Para el snapshot: forzar que use SIEMPRE el modelo (no rule-based 20%).
        if snapshot is not None:
            env._use_rb_this_ep = False
        done = False
        s = c = 0
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, _r, term, trunc, info = env.step(action)
            if info.get("goal_scored"):
                s = 1
            if info.get("conceded"):
                c = 1
            done = term or trunc
        if s:
            scored += 1
        elif c:
            conceded += 1
        else:
            draws += 1
    n = float(episodes)
    return {"scored": scored / n, "conceded": conceded / n,
            "draw": draws / n, "net": (scored - conceded) / n}


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", required=True, type=Path)
    p.add_argument("--episodes", type=int, default=100)
    p.add_argument("--seed", type=int, default=1000)
    p.add_argument("--opp-snapshot", type=str, default=None,
                   help="opcional: modelo RL congelado como 3er oponente fijo")
    args = p.parse_args()

    model = PPO.load(str(args.checkpoint), device="cpu")
    print(f"== Win-tendency vs baselines FIJOS — {args.checkpoint.name} "
          f"({args.episodes} ep c/u) ==")
    print(f"{'oponente':<12}{'scored%':>9}{'conceded%':>11}{'draw%':>8}{'net%':>8}")
    print("-" * 48)

    targets = [("rulebased", None), ("ballchaser", None)]
    if args.opp_snapshot:
        targets.append(("snapshot", args.opp_snapshot))

    for name, snap in targets:
        mode = "selfplay" if snap else name
        r = eval_vs(model, mode, args.episodes, args.seed, snapshot=snap)
        print(f"{name:<12}{r['scored']*100:>8.0f}%{r['conceded']*100:>10.0f}%"
              f"{r['draw']*100:>7.0f}%{r['net']*100:>+7.0f}")
    print("-" * 48)
    print("net% > 0 = dominamos a ese rival. Comparar entre checkpoints "
          "para ver progreso REAL (no el goal_rate de self-play).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
