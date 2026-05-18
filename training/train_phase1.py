"""
Entrenamiento Fase 1 — atacante vs arco (warmup) con PPO sobre rSoccer VSS env.

Uso:
    training\.venv\Scripts\python.exe train_phase1.py
    training\.venv\Scripts\python.exe train_phase1.py --total-steps 2000000 --n-envs 8

Outputs:
- TensorBoard logs en training/runs/phase1_<timestamp>/
- Checkpoint final en training/checkpoints/phase1_final.zip
- Checkpoints intermedios cada 100k steps en training/checkpoints/phase1_<steps>.zip
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

from vsss_rl.env_phase1 import VsssPhase1Env
from vsss_rl.policy import VsssActorCriticPolicy


def make_env(seed: int):
    def _init():
        env = VsssPhase1Env(seed=seed)
        env.reset(seed=seed)
        return env
    return _init


def build_run_dir(run_name: str | None) -> Path:
    repo_root = Path(__file__).resolve().parent
    runs_dir = repo_root / "runs"
    runs_dir.mkdir(exist_ok=True)
    ckpts_dir = repo_root / "checkpoints"
    ckpts_dir.mkdir(exist_ok=True)
    stamp = run_name or f"phase1_{time.strftime('%Y%m%d_%H%M%S')}"
    return runs_dir / stamp


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--total-steps", type=int, default=1_000_000)
    p.add_argument("--n-envs", type=int, default=4)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--device", default="auto")  # auto = GPU si disponible
    p.add_argument("--run-name", default=None)
    p.add_argument(
        "--learning-rate",
        type=float,
        default=3e-4,
        help="PPO learning rate (default 3e-4, estándar).",
    )
    args = p.parse_args()

    run_dir = build_run_dir(args.run_name)
    ckpt_dir = Path(__file__).resolve().parent / "checkpoints"
    print(f"[train_phase1] env           = VsssPhase1Env (mini-sim 1v0)")
    print(f"[train_phase1] total_steps   = {args.total_steps:,}")
    print(f"[train_phase1] n_envs        = {args.n_envs}")
    print(f"[train_phase1] device        = {args.device}")
    print(f"[train_phase1] tb logdir     = {run_dir}")
    print(f"[train_phase1] checkpoints   = {ckpt_dir}")

    # SubprocVecEnv para máxima velocidad (envs en procesos separados).
    # DummyVecEnv si n-envs=1 o si SubprocVec da problemas en Windows.
    env_fns = [make_env(args.seed + i) for i in range(args.n_envs)]
    if args.n_envs == 1:
        vec_env = DummyVecEnv(env_fns)
    else:
        vec_env = SubprocVecEnv(env_fns, start_method="spawn")

    model = PPO(
        policy=VsssActorCriticPolicy,  # ← nuestra red propia en PyTorch puro
        env=vec_env,
        learning_rate=args.learning_rate,
        n_steps=2048,
        batch_size=256,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
        vf_coef=0.5,
        max_grad_norm=0.5,
        verbose=1,
        tensorboard_log=str(run_dir.parent),  # logs en runs/
        device=args.device,
        seed=args.seed,
    )

    ckpt_cb = CheckpointCallback(
        save_freq=max(100_000 // args.n_envs, 1),
        save_path=str(ckpt_dir),
        name_prefix="phase1",
        save_replay_buffer=False,
        save_vecnormalize=False,
    )

    try:
        model.learn(
            total_timesteps=args.total_steps,
            callback=ckpt_cb,
            tb_log_name=run_dir.name,
            progress_bar=True,
        )
    finally:
        final_path = ckpt_dir / "phase1_final.zip"
        model.save(str(final_path))
        print(f"[train_phase1] modelo guardado: {final_path}")
        vec_env.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
