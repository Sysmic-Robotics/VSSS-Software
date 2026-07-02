"""Entrenamiento PPO sobre rSoccer (fisica VSS realista) con nuestro contrato
(obs 52 + accion (v,omega)). El modelo se despliega con el RlCoach SIN cambios.

Intento 4 (Fase B): policy custom de dos torres + salvaguardas anti-colapso
(clip de log_std, ent_coef>0, target_kl — el intento 3b colapso: std 0.86->0.045,
approx_kl 0.75 a los 20.5M) + Monitor (ep_rew_mean) + eval periodico.

Uso (venv-rsoccer):  N_ENVS=8 RSOCCER_STEPS=8000000 python train_rsoccer.py
Exportar luego:      python export_onnx.py --checkpoint checkpoints_rsoccer/rsoccer_field_final.zip --out models/policy_field.onnx
"""
import os
# Debe ir ANTES de importar cualquier cosa que toque protobuf (rsoccer / tensorboard).
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pathlib import Path
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from callbacks import ClipLogStdCallback
from vsss_rl.policy import VsssActorCriticPolicy
from vsss_rl.rsoccer_env import make_field_env

N_ENVS = int(os.environ.get("N_ENVS", "8"))
STEPS = int(os.environ.get("RSOCCER_STEPS", "8000000"))
CK = Path("checkpoints_rsoccer")


def make_env():
    # Monitor registra ep_rew_mean/ep_len_mean en TensorBoard (faltaba en intento 3b).
    return Monitor(make_field_env())


def main():
    CK.mkdir(exist_ok=True)
    env = SubprocVecEnv([make_env for _ in range(N_ENVS)])
    model = PPO(
        VsssActorCriticPolicy, env, device="cpu",
        n_steps=2048, batch_size=256, learning_rate=1e-4,
        gamma=0.99, gae_lambda=0.95,
        ent_coef=0.001,   # B1: bonus de entropia (el 0.0 del intento 3b dejo colapsar la politica)
        target_kl=0.05,   # B1: corta updates divergentes (approx_kl llego a 0.75)
        tensorboard_log="tb_rsoccer", verbose=1,
    )
    callbacks = [
        CheckpointCallback(save_freq=max(100000 // N_ENVS, 1),
                           save_path=str(CK), name_prefix="rsoccer_field"),
        # B1: std acotada a [e^-2, e^0.7] ~ [0.14, 2.0] — mismo fix que salvo el self-play viejo.
        ClipLogStdCallback(min_log_std=-2.0, max_log_std=0.7),
        # Eval deterministico periodico → mean_reward en TB + guarda el best model.
        EvalCallback(Monitor(make_field_env()),
                     eval_freq=max(250000 // N_ENVS, 1), n_eval_episodes=10,
                     deterministic=True, best_model_save_path=str(CK / "best"),
                     verbose=1),
    ]
    model.learn(total_timesteps=STEPS, callback=callbacks, progress_bar=True)
    model.save(str(CK / "rsoccer_field_final"))
    print("[train_rsoccer] LISTO ->", CK / "rsoccer_field_final.zip")


if __name__ == "__main__":
    main()
