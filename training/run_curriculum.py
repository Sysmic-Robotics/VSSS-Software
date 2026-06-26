"""
Orquestador del currículo AVANZADO: corre las fases en secuencia en UN solo
proceso, encadenándolas automáticamente. Resumible: si se corta y se relanza,
salta las fases ya completas y reanuda la interrumpida desde su último checkpoint.

Uso (desde training/, con el venv):
    python run_curriculum.py
"""
from __future__ import annotations

import os
import re
import subprocess
import sys
from pathlib import Path

# Forzar UTF-8 en la salida para no crashear en consolas Windows (cp1252) al
# imprimir acentos/símbolos. errors='replace' es a prueba de balas.
sys.stdout.reconfigure(encoding="utf-8", errors="replace")
sys.stderr.reconfigure(encoding="utf-8", errors="replace")

CK = Path("checkpoints")
VENV_PY = sys.executable  # corre bajo el python del venv
N_ENVS = os.environ.get("N_ENVS", "4")  # en la VM con muchos núcleos: export N_ENVS=32

# (phase_key, total_steps, warm_from_final | None)
PHASES = [
    ("1",       2_000_000,  None),
    ("2",       2_000_000,  "phase1_final.zip"),
    ("2v1",     4_000_000,  "phase2_final.zip"),
    ("mixed",  10_000_000,  "phase2v1_final.zip"),
    ("mixedsp", 25_000_000, "phasemixed_final.zip"),
    ("gk",      6_000_000,  None),
]


def latest_checkpoint(key: str):
    files = list(CK.glob(f"phase{key}_*_steps.zip"))
    if not files:
        return None, 0
    def steps(f: Path) -> int:
        m = re.search(r"_(\d+)_steps", f.name)
        return int(m.group(1)) if m else 0
    f = max(files, key=steps)
    return f, steps(f)


def main() -> int:
    print("=" * 60)
    print("CURRÍCULO AVANZADO — orquestador (encadena fases solo)")
    print("=" * 60)
    for key, total, warm in PHASES:
        final = CK / f"phase{key}_final.zip"
        if final.exists():
            print(f"[curriculum] fase '{key}' YA completa ({final.name}) → salto.")
            continue

        ck, done = latest_checkpoint(key)
        if ck is not None and done > 0:
            # train.py usa reset_num_timesteps=False al reanudar → SB3 SUMA al
            # contador. Por eso pasamos SOLO lo que falta (total - done), así el
            # contador llega exacto a `total` y no entrena de más.
            total_arg = max(total - done, 8192)
            print(f"[curriculum] REANUDAR fase '{key}' desde {ck.name} ({done:,}/{total:,}) → faltan {total_arg:,}")
            extra = ["--resume", str(ck)]
        elif warm:
            total_arg = total
            print(f"[curriculum] fase '{key}' fresca, warm-start de {warm}")
            extra = ["--warm-start", str(CK / warm)]
        else:
            total_arg = total
            print(f"[curriculum] fase '{key}' fresca (sin warm-start)")
            extra = []

        cmd = [VENV_PY, "train.py", "--phase", key, "--total-steps", str(total_arg),
               "--n-envs", N_ENVS, "--device", "cpu", "--run-name", f"adv_{key}"] + extra
        print(f"[curriculum] >>> {' '.join(cmd)}", flush=True)
        child_env = {**os.environ, "PYTHONIOENCODING": "utf-8"}
        result = subprocess.run(cmd, env=child_env)
        if result.returncode != 0:
            print(f"[curriculum] fase '{key}' devolvió código {result.returncode} → aborto.", flush=True)
            return result.returncode
        print(f"[curriculum] fase '{key}' COMPLETA.", flush=True)

    print("[curriculum] ===== CURRÍCULO COMPLETO =====", flush=True)
    print("Exportar: python export_onnx.py --checkpoint checkpoints/phasemixedsp_final.zip --out models/policy_field.onnx")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
