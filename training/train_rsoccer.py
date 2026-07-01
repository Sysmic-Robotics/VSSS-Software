"""Entrenamiento PPO sobre rSoccer (fisica VSS realista) con nuestro contrato
(obs 52 + accion (v,omega)). El modelo se despliega con el RlCoach SIN cambios.

Uso (venv-rsoccer):  N_ENVS=8 RSOCCER_STEPS=5000000 python train_rsoccer.py
Exportar luego:      python export_onnx.py --checkpoint checkpoints_rsoccer/rsoccer_field_final.zip --out models/policy_field.onnx
"""
import os
# Debe ir ANTES de importar cualquier cosa que toque protobuf (rsoccer / tensorboard).
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pathlib import Path
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback
from vsss_rl.rsoccer_env import make_field_env

N_ENVS = int(os.environ.get("N_ENVS", "8"))
STEPS = int(os.environ.get("RSOCCER_STEPS", "5000000"))
CK = Path("checkpoints_rsoccer")


def main():
    CK.mkdir(exist_ok=True)
    env = SubprocVecEnv([make_field_env for _ in range(N_ENVS)])
    model = PPO(
        "MlpPolicy", env, device="cpu",
        policy_kwargs=dict(net_arch=[256, 256]),
        n_steps=2048, batch_size=256, learning_rate=1e-4,
        gamma=0.99, gae_lambda=0.95, ent_coef=0.0,
        tensorboard_log="tb_rsoccer", verbose=1,
    )
    ckpt = CheckpointCallback(save_freq=max(100000 // N_ENVS, 1),
                              save_path=str(CK), name_prefix="rsoccer_field")
    model.learn(total_timesteps=STEPS, callback=ckpt, progress_bar=True)
    model.save(str(CK / "rsoccer_field_final"))
    print("[train_rsoccer] LISTO ->", CK / "rsoccer_field_final.zip")


if __name__ == "__main__":
    main()
