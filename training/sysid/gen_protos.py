"""Genera los bindings Python de los .proto FIRA que usa el harness de system-ID.

Toma los .proto del engine (`src/protos/fira_*.proto`) y genera los `*_pb2.py`
en `training/sysid/fira_proto/`. Así el harness habla EXACTAMENTE el mismo formato
de wire que el engine Rust (mismo `Packet`/`Command`/`Replacement`/`Environment`).

Requisito (una vez):
    pip install grpcio-tools

Uso:
    python gen_protos.py
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]            # training/sysid -> training -> raíz del repo
PROTO_DIR = REPO_ROOT / "src" / "protos"
OUT_DIR = HERE / "fira_proto"

# Orden no importa (protoc resuelve imports vía -I), pero listamos los 4 que usamos.
PROTOS = [
    "fira_common.proto",       # Ball, Robot, Field, Frame
    "fira_command.proto",      # Command{id, yellowteam, wheel_left, wheel_right}, Commands
    "fira_replacement.proto",  # RobotReplacement, BallReplacement, Replacement
    "fira_packet.proto",       # Packet{cmd, replace}, Environment{step, frame, ...}
]


def main() -> int:
    if not PROTO_DIR.is_dir():
        print(f"[gen_protos] No encuentro {PROTO_DIR}", file=sys.stderr)
        return 1

    OUT_DIR.mkdir(exist_ok=True)

    cmd = [
        sys.executable, "-m", "grpc_tools.protoc",
        f"-I{PROTO_DIR}",
        f"--python_out={OUT_DIR}",
        *[str(PROTO_DIR / p) for p in PROTOS],
    ]
    print("[gen_protos]", " ".join(cmd))
    try:
        subprocess.run(cmd, check=True)
    except FileNotFoundError:
        print("[gen_protos] Falta grpcio-tools. Instalalo con:\n"
              "    pip install grpcio-tools", file=sys.stderr)
        return 1
    except subprocess.CalledProcessError as e:
        print(f"[gen_protos] protoc falló (código {e.returncode})", file=sys.stderr)
        return e.returncode

    print(f"[gen_protos] OK → bindings en {OUT_DIR}")
    print("[gen_protos] Los *_pb2.py se importan agregando esa carpeta al sys.path "
          "(el harness ya lo hace).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
