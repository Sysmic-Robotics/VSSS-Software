"""
Entrenamiento unificado del currículo VSSS (skill-based, contrato SkillChoice).

Uso:
    python train.py --phase 1 --total-steps 1500000 --device cpu
    python train.py --phase 2 --warm-start checkpoints/phase1_final.zip --total-steps 3000000
    python train.py --phase 3 --warm-start checkpoints/phase2_final.zip --total-steps 8000000

Características (endurecimiento Workstream A):
- Monitor wrapper → ep_rew_mean / ep_len_mean en TensorBoard.
- RewardComponentCallback → componentes de recompensa + goal_rate en TensorBoard.
- EvalCallback → guarda el mejor modelo según recompensa de evaluación.
- CheckpointCallback → snapshots periódicos (también sirven al pool de self-play).
- Warm-start parcial: transfiere los pesos cuya forma coincide (el tronco MLP
  52→256→256 siempre transfiere; las cabezas se reinician si cambia el nº de
  robots controlados entre fases).
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# UTF-8 a prueba de consolas Windows (cp1252) — evita UnicodeEncodeError en prints.
sys.stdout.reconfigure(encoding="utf-8", errors="replace")
sys.stderr.reconfigure(encoding="utf-8", errors="replace")

import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

from callbacks import ClipLogStdCallback, RewardComponentCallback, SelfPlayCallback
from vsss_rl.policy import VsssActorCriticPolicy
from vsss_rl.soccer_env import (
    VsssSoccerEnv,
    phase1_config,
    phase2_config,
    phase_2v1_config,
    phase_mixed_config,
    phase3_config,
    goalkeeper_config,
)

# Fases del currículo (campo) + arquero. Strings para poder nombrarlas.
PHASE_CONFIGS = {
    "1": phase1_config,         # 1v0
    "2": phase2_config,         # 1v1
    "2v1": phase_2v1_config,    # 2v1 cooperación (Box 6)
    "mixed": phase_mixed_config,  # 2vN randomizado (run largo de campo)
    "3": phase3_config,         # 3v3 fijo (oponente rule-based)
    "3sp": lambda: phase3_config("selfplay"),       # 3v3 self-play
    "mixedsp": lambda: phase_mixed_config("selfplay"),  # 2vN mixto self-play (run largo)
    "gk": goalkeeper_config,    # arquero (red separada, recompensa defensa)
}


def make_env(phase: str, seed: int):
    def _init():
        cfg = PHASE_CONFIGS[phase]()
        env = VsssSoccerEnv(config=cfg, seed=seed)
        env = Monitor(env)  # ep_rew_mean / ep_len_mean
        env.reset(seed=seed)
        return env
    return _init


def warm_start(model: PPO, ckpt_path: str, device: str) -> None:
    """Transferencia parcial de pesos: copia los parámetros cuya forma coincide."""
    old = PPO.load(ckpt_path, device=device)
    new_sd = model.policy.state_dict()
    old_sd = old.policy.state_dict()
    transferred, total = 0, 0
    for k, v in new_sd.items():
        total += 1
        if k in old_sd and old_sd[k].shape == v.shape:
            new_sd[k] = old_sd[k]
            transferred += 1
    model.policy.load_state_dict(new_sd)
    print(f"[warm-start] transferidos {transferred}/{total} tensores desde {ckpt_path}")


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--phase", type=str, choices=list(PHASE_CONFIGS.keys()), required=True)
    p.add_argument("--total-steps", type=int, default=1_500_000)
    p.add_argument("--n-envs", type=int, default=4)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--device", default="cpu")
    p.add_argument("--warm-start", default=None, help="checkpoint .zip para warm-start parcial (cross-fase)")
    p.add_argument("--resume", default=None, help="checkpoint .zip para REANUDAR esta misma fase (modelo+optimizador, continúa el contador)")
    p.add_argument("--run-name", default=None)
    p.add_argument("--learning-rate", type=float, default=3e-4)
    args = p.parse_args()

    root = Path(__file__).resolve().parent
    runs_dir = root / "runs"; runs_dir.mkdir(exist_ok=True)
    ckpt_dir = root / "checkpoints"; ckpt_dir.mkdir(exist_ok=True)
    run_name = args.run_name or f"phase{args.phase}_{time.strftime('%Y%m%d_%H%M%S')}"
    best_dir = ckpt_dir / f"{run_name}_best"

    print(f"[train] phase={args.phase} steps={args.total_steps:,} n_envs={args.n_envs} device={args.device}")
    print(f"[train] run={run_name}  warm_start={args.warm_start}")

    env_fns = [make_env(args.phase, args.seed + i) for i in range(args.n_envs)]
    vec_env = DummyVecEnv(env_fns) if args.n_envs == 1 else SubprocVecEnv(env_fns, start_method="spawn")
    eval_env = DummyVecEnv([make_env(args.phase, args.seed + 999)])

    if args.resume:
        print(f"[resume] cargando modelo completo (con optimizador) desde {args.resume}")
        model = PPO.load(args.resume, env=vec_env, device=args.device)
    else:
        model = PPO(
            policy=VsssActorCriticPolicy,
            env=vec_env,
            learning_rate=args.learning_rate,
            n_steps=2048,
            batch_size=256,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.001,  # bajado de 0.01: el bonus de entropía alto infló el std en self-play
            vf_coef=0.5,
            max_grad_norm=0.5,
            verbose=1,
            tensorboard_log=str(runs_dir),
            device=args.device,
            seed=args.seed,
        )
        if args.warm_start:
            warm_start(model, args.warm_start, args.device)

    callbacks = [
        CheckpointCallback(
            save_freq=max(100_000 // args.n_envs, 1),  # guardado frecuente (pausa/resume)
            save_path=str(ckpt_dir),
            name_prefix=f"phase{args.phase}",
        ),
        EvalCallback(
            eval_env,
            best_model_save_path=str(best_dir),
            log_path=str(best_dir),
            eval_freq=max(50_000 // args.n_envs, 1),
            n_eval_episodes=20,
            deterministic=True,
            verbose=0,
        ),
        RewardComponentCallback(log_freq=20_480),
        ClipLogStdCallback(min_log_std=-2.0, max_log_std=0.7),  # evita explosión del std
    ]

    # Self-play: si el escenario lo pide, agregar el pool de snapshots.
    if PHASE_CONFIGS[args.phase]().opponent_mode == "selfplay":
        callbacks.append(SelfPlayCallback(
            pool_dir=str(ckpt_dir / f"{run_name}_pool"),
            snapshot_freq=300_000, seed=args.seed, verbose=1,
        ))

    try:
        model.learn(total_timesteps=args.total_steps, callback=callbacks,
                    tb_log_name=run_name, progress_bar=True,
                    reset_num_timesteps=(args.resume is None))
    finally:
        final_path = ckpt_dir / f"phase{args.phase}_final.zip"
        model.save(str(final_path))
        print(f"[train] modelo guardado: {final_path}")
        vec_env.close()
        eval_env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
