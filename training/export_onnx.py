"""
Exporta una política PPO entrenada a ONNX para deploy en el motor Rust (RlCoach).

El modelo ONNX recibe la observación de 52 floats y devuelve la acción
DETERMINISTA (la media μ de la gaussiana), con forma (batch, 3·N) donde N es el
número de robots de campo controlados. Cada tripleta es `[skill_sel, tx, ty]`:
- `skill_sel` → SkillId por binning en 5 (mismo binning que el env).
- `(tx, ty)`  → target en coords de campo: `tx·0.75`, `ty·0.65`.

Esto es exactamente lo que `RlCoach::decide` debe leer y convertir en
`Vec<SkillChoice>`.

Uso:
    python export_onnx.py --checkpoint checkpoints/phase3_final.zip --out models/policy.onnx
"""

from __future__ import annotations

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
