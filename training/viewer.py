"""
Visualizador del agente entrenado.

Carga un checkpoint y muestra episodios en una ventana matplotlib animada.
Útil para mostrar visualmente el progreso del aprendizaje (presentación, debug).

Uso:
    python viewer.py --checkpoint checkpoints/phase1_final.zip
    python viewer.py --checkpoint checkpoints/phase1_100000_steps.zip --episodes 3
    python viewer.py --random          # agente random (baseline visual)
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from vsss_rl.env_phase1 import VsssPhase1Env
from vsss_rl.render import setup_field, update_field


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument(
        "--checkpoint",
        type=Path,
        help="Ruta al .zip del modelo. Si se omite, usa agente random.",
    )
    p.add_argument("--random", action="store_true", help="Forzar agente random")
    p.add_argument("--episodes", type=int, default=5)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--fps", type=int, default=60)
    args = p.parse_args()

    use_random = args.random or args.checkpoint is None
    model = None
    if not use_random:
        from stable_baselines3 import PPO
        model = PPO.load(str(args.checkpoint))
        print(f"Cargado checkpoint: {args.checkpoint}")
    else:
        print("Modo random (sin modelo)")

    env = VsssPhase1Env(seed=args.seed)

    fig, ax = plt.subplots(figsize=(8, 7))
    handles = setup_field(ax)

    state = {"episode": 0, "step": 0, "ep_reward": 0.0, "goals": 0, "obs": None, "info": None}

    def reset_episode():
        obs, info = env.reset(seed=args.seed + state["episode"])
        state["obs"] = obs
        state["info"] = info
        state["step"] = 0
        state["ep_reward"] = 0.0

    reset_episode()

    title = (
        f"Checkpoint: {args.checkpoint.name}" if not use_random else "Agente random"
    )
    ax.set_title(title, color="white", pad=10)
    fig.patch.set_facecolor("#222222")
    ax.tick_params(colors="white")
    for spine in ax.spines.values():
        spine.set_color("white")
    ax.xaxis.label.set_color("white")
    ax.yaxis.label.set_color("white")

    def step(_frame):
        if state["episode"] >= args.episodes:
            return tuple(handles.values())

        if model is not None:
            action, _ = model.predict(state["obs"], deterministic=True)
        else:
            action = env.action_space.sample()

        obs, reward, terminated, truncated, info = env.step(action)
        state["obs"] = obs
        state["info"] = info
        state["step"] += 1
        state["ep_reward"] += float(reward)

        rx, ry = info["robot_pos"]
        bx, by = info["ball_pos"]
        info_str = (
            f"Episodio {state['episode'] + 1}/{args.episodes} | "
            f"t={state['step'] / 60:.1f}s | "
            f"reward={state['ep_reward']:+.2f} | "
            f"goles={state['goals']}"
        )
        update_field(
            handles=handles,
            robot_pos=(float(rx), float(ry)),
            robot_theta=env._robot_theta,
            ball_pos=(float(bx), float(by)),
            info=info_str,
        )

        if terminated or truncated:
            if info.get("goal_scored"):
                state["goals"] += 1
                print(
                    f"Episodio {state['episode'] + 1}: GOL "
                    f"en {state['step'] / 60:.2f}s, reward={state['ep_reward']:+.2f}"
                )
            else:
                reason = (
                    "out_of_bounds" if info.get("out_of_bounds")
                    else "own_goal" if info.get("own_goal")
                    else "timeout"
                )
                print(
                    f"Episodio {state['episode'] + 1}: {reason} "
                    f"reward={state['ep_reward']:+.2f}"
                )
            state["episode"] += 1
            if state["episode"] < args.episodes:
                reset_episode()
            else:
                print(
                    f"\n── RESUMEN ──\n"
                    f"Goles: {state['goals']}/{args.episodes}  "
                    f"({100 * state['goals'] / args.episodes:.0f}%)"
                )

        return tuple(handles.values())

    # FuncAnimation: 1 frame por step del env, a fps configurable.
    anim = FuncAnimation(
        fig,
        step,
        interval=int(1000 / args.fps),
        blit=False,
        cache_frame_data=False,
    )

    plt.tight_layout()
    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
