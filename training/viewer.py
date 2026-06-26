"""
Visualizador del agente entrenado (env skill-based, autocontenido).

Muestra al agente jugando en el mini-sim Python — sin Rust ni FIRASim.

Uso:
    python viewer.py --phase 1 --checkpoint checkpoints/phase1_final.zip
    python viewer.py --phase 2 --checkpoint checkpoints/phase2_final.zip --episodes 5
    python viewer.py --phase 1 --random            # baseline visual sin entrenar
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from vsss_rl.engine_constants import FIELD_HALF_X, FIELD_HALF_Y, GOAL_HALF_WIDTH, ROBOT_RADIUS, BALL_RADIUS
from vsss_rl.soccer_env import VsssSoccerEnv, phase1_config, phase2_config, phase3_config

PHASE_CONFIGS = {1: phase1_config, 2: phase2_config, 3: phase3_config}


def setup_field(ax):
    ax.set_xlim(-FIELD_HALF_X - 0.05, FIELD_HALF_X + 0.05)
    ax.set_ylim(-FIELD_HALF_Y - 0.05, FIELD_HALF_Y + 0.05)
    ax.set_aspect("equal")
    ax.set_facecolor("#1f7a1f")
    field = mpatches.Rectangle((-FIELD_HALF_X, -FIELD_HALF_Y), 2 * FIELD_HALF_X, 2 * FIELD_HALF_Y,
                               linewidth=2, edgecolor="white", facecolor="none")
    ax.add_patch(field)
    ax.plot([0, 0], [-FIELD_HALF_Y, FIELD_HALF_Y], "w--", lw=1, alpha=0.6)
    # Arcos
    ax.plot([FIELD_HALF_X, FIELD_HALF_X], [-GOAL_HALF_WIDTH, GOAL_HALF_WIDTH], "-", color="yellow", lw=4)
    ax.plot([-FIELD_HALF_X, -FIELD_HALF_X], [-GOAL_HALF_WIDTH, GOAL_HALF_WIDTH], "-", color="cyan", lw=4)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--phase", type=int, choices=[1, 2, 3], default=1)
    p.add_argument("--checkpoint", type=Path)
    p.add_argument("--random", action="store_true")
    p.add_argument("--episodes", type=int, default=5)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--fps", type=int, default=30)
    args = p.parse_args()

    use_random = args.random or args.checkpoint is None
    model = None
    if not use_random:
        from stable_baselines3 import PPO
        model = PPO.load(str(args.checkpoint), device="cpu")
        print(f"Cargado: {args.checkpoint}")

    env = VsssSoccerEnv(config=PHASE_CONFIGS[args.phase]())
    state = {"ep": 0, "obs": None, "info": None, "r": 0.0, "goals": 0}

    def reset_ep():
        obs, info = env.reset(seed=args.seed + state["ep"])
        state.update(obs=obs, info=info, r=0.0)

    reset_ep()

    fig, ax = plt.subplots(figsize=(8, 7))
    fig.patch.set_facecolor("#222222")
    setup_field(ax)
    title = args.checkpoint.name if not use_random else "Agente random"
    ax.set_title(f"Fase {args.phase} — {title}", color="white")

    own_circles, own_arrows, opp_circles = [], [], []
    for _ in range(3):
        c = mpatches.Circle((0, 0), ROBOT_RADIUS, color="#3399ff", ec="white", zorder=5, visible=False)
        ax.add_patch(c); own_circles.append(c)
        arr = ax.annotate("", xy=(0, 0), xytext=(0, 0),
                          arrowprops=dict(arrowstyle="->", color="white", lw=2), zorder=6)
        own_arrows.append(arr)
        oc = mpatches.Circle((0, 0), ROBOT_RADIUS, color="#ffcc00", ec="white", zorder=5, visible=False)
        ax.add_patch(oc); opp_circles.append(oc)
    ball = mpatches.Circle((0, 0), BALL_RADIUS, color="#ff6600", ec="white", zorder=4)
    ax.add_patch(ball)
    info_txt = ax.text(-FIELD_HALF_X, FIELD_HALF_Y + 0.02, "", color="white", fontsize=9,
                       bbox=dict(facecolor="black", alpha=0.5, pad=2, edgecolor="none"))

    def frame(_):
        if state["ep"] >= args.episodes:
            return []
        if model is not None:
            action, _ = model.predict(state["obs"], deterministic=True)
        else:
            action = env.action_space.sample()
        obs, r, term, trunc, info = env.step(action)
        state.update(obs=obs, info=info)
        state["r"] += float(r)

        for i in range(3):
            if i in info["own"]:
                x, y, th = info["own"][i]
                own_circles[i].center = (x, y); own_circles[i].set_visible(True)
                own_arrows[i].set_position((x, y))
                own_arrows[i].xy = (x + 0.08 * math.cos(th), y + 0.08 * math.sin(th))
            else:
                own_circles[i].set_visible(False); own_arrows[i].set_position((0, 0)); own_arrows[i].xy = (0, 0)
            if i in info["opp"]:
                ox, oy, _ = info["opp"][i]
                opp_circles[i].center = (ox, oy); opp_circles[i].set_visible(True)
            else:
                opp_circles[i].set_visible(False)
        bx, by = info["ball_pos"]
        ball.center = (float(bx), float(by))
        info_txt.set_text(f"Ep {state['ep']+1}/{args.episodes} | reward={state['r']:+.2f} | goles={state['goals']}")

        if term or trunc:
            if info.get("goal_scored"):
                state["goals"] += 1
                print(f"Ep {state['ep']+1}: GOL  reward={state['r']:+.2f}")
            else:
                print(f"Ep {state['ep']+1}: fin (term={term} trunc={trunc}) reward={state['r']:+.2f}")
            state["ep"] += 1
            if state["ep"] < args.episodes:
                reset_ep()
            else:
                print(f"\nRESUMEN: {state['goals']}/{args.episodes} goles")
        return []

    _anim = FuncAnimation(fig, frame, interval=int(1000 / args.fps), blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
