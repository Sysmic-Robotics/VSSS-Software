"""
Exporta una política PPO entrenada a ONNX para deploy en el motor Rust (RlCoach).

El modelo ONNX recibe la observación de 52 floats y devuelve la acción
DETERMINISTA (la media μ de la gaussiana), con forma (batch, 2·N) donde N es el
número de robots de campo controlados. Cada par es `[v, ω]` ∈ [-1,1] (Intento 3):
- `v` → velocidad lineal hacia el frente (el RlCoach la denorm con V_MAX).
- `ω` → velocidad angular (denorm con OMEGA_MAX). Sin UVF (SkillId::DirectVel).

Esto es exactamente lo que `RlCoach::decide` debe leer y convertir en
`Vec<SkillChoice>`.

Uso:
    python export_onnx.py --checkpoint checkpoints/phase3_final.zip --out models/policy.onnx
"""

from __future__ import annotations

import sys

# UTF-8 a prueba de consolas Windows (cp1252) — evita UnicodeEncodeError al
# imprimir "→" y demás (mismo fix que train.py / run_curriculum.py).
sys.stdout.reconfigure(encoding="utf-8", errors="replace")
sys.stderr.reconfigure(encoding="utf-8", errors="replace")

import argparse
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import PPO

from vsss_rl.observation import FLAT_SIZE


class OnnxablePolicy(torch.nn.Module):
    """Envuelve la policy de SB3 para exportar solo el camino obs → acción media."""

    def __init__(self, policy):
        super().__init__()
        self.policy = policy

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        features = self.policy.extract_features(obs)
        if isinstance(features, tuple):  # extractores pi/vf separados
            features = features[0]
        latent_pi = self.policy.mlp_extractor.forward_actor(features)
        return self.policy.action_net(latent_pi)  # media μ (acción determinista)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", required=True, type=Path)
    p.add_argument("--out", type=Path, default=Path("models/policy.onnx"))
    p.add_argument("--opset", type=int, default=17)
    args = p.parse_args()

    args.out.parent.mkdir(parents=True, exist_ok=True)

    model = PPO.load(str(args.checkpoint), device="cpu")
    model.policy.eval()
    onnxable = OnnxablePolicy(model.policy)
    onnxable.eval()

    action_dim = int(model.action_space.shape[0])
    dummy = torch.zeros(1, FLAT_SIZE, dtype=torch.float32)

    torch.onnx.export(
        onnxable,
        dummy,
        str(args.out),
        input_names=["obs"],
        output_names=["action"],
        dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
        opset_version=args.opset,
        # Exportador LEGACY (TorchScript): produce un .onnx self-contained con los
        # pesos embebidos, compatible con tract-onnx (igual que el modelo de skills).
        # El exportador nuevo (dynamo=True) sacaba los pesos a un archivo externo →
        # .onnx de ~1.8 KB sin pesos, que tract no puede usar.
        dynamo=False,
    )
    print(f"[onnx] exportado: {args.out}  (obs={FLAT_SIZE} → action={action_dim})")

    # ── Validación: ONNX vs PyTorch determinista ─────────────────────────
    try:
        import onnxruntime as ort

        rng = np.random.default_rng(0)
        sess = ort.InferenceSession(str(args.out), providers=["CPUExecutionProvider"])
        max_err = 0.0
        for _ in range(100):
            obs = rng.uniform(-1.0, 1.0, size=(1, FLAT_SIZE)).astype(np.float32)
            onnx_out = sess.run(["action"], {"obs": obs})[0]
            with torch.no_grad():
                torch_out = onnxable(torch.from_numpy(obs)).numpy()
            max_err = max(max_err, float(np.abs(onnx_out - torch_out).max()))
        print(f"[onnx] validación OK — error máx PyTorch↔ONNX: {max_err:.2e}")
        if max_err > 1e-4:
            print("[onnx] ⚠ error mayor al esperado; revisar export.")
    except ImportError:
        print("[onnx] onnxruntime no instalado — exportado sin validar.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
