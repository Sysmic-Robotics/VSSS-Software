"""
Orquestador del currículo AVANZADO: corre las fases en secuencia en UN solo
proceso, encadenándolas automáticamente. Resumible: si se corta y se relanza,
salta las fases ya completas y reanuda la interrumpida desde su último checkpoint.

Uso (desde training/, con el venv):
    python run_curriculum.py
"""
from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

CK = Path("checkpoints")
VENV_PY = sys.executable  # corre bajo el python del venv

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

        cmd = [VENV_PY, "train.py", "--phase", key, "--total-steps", str(total),
               "--n-envs", "4", "--device", "cpu", "--run-name", f"adv_{key}"]

        ck, done = latest_checkpoint(key)
        if ck is not None and done > 0:
            print(f"[curriculum] REANUDAR fase '{key}' desde {ck.name} ({done:,}/{total:,})")
            cmd += ["--resume", str(ck)]
        elif warm:
            print(f"[curriculum] fase '{key}' fresca, warm-start de {warm}")
            cmd += ["--warm-start", str(CK / warm)]
        else:
            print(f"[curriculum] fase '{key}' fresca (sin warm-start)")

        print(f"[curriculum] >>> {' '.join(cmd)}", flush=True)
        result = subprocess.run(cmd)
        if result.returncode != 0:
            print(f"[curriculum] fase '{key}' devolvió código {result.returncode} → aborto.", flush=True)
            return result.returncode
        print(f"[curriculum] fase '{key}' COMPLETA.", flush=True)

    print("[curriculum] ===== CURRÍCULO COMPLETO =====", flush=True)
    print("Exportar: python export_onnx.py --checkpoint checkpoints/phasemixedsp_final.zip --out models/policy_field.onnx")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
