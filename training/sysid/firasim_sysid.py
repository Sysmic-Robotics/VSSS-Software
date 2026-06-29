"""Harness de system-ID contra FIRASim (Fase 0 del Intento 3).

Manda comandos `(v, ω)` CONOCIDOS a un robot en FIRASim (convertidos a velocidad de
rueda con la MISMA fórmula que el engine: `wheel = v/r ± ω·L/(2r)`, ver
`src/radio/commands.rs`), teleporta con `Replacement`, y registra la respuesta que
FIRASim publica en la visión (`Environment.frame`). Genera CSVs que luego ajusta
`fit_sysid.py` para producir `sim_calibration.json`.

Objetivo: medir la dinámica REAL de FIRASim (vel. máx, aceleración, fricción, física
de pelota) para calibrar el sim de entrenamiento y los rangos de domain randomization,
en vez de usar valores adivinados.

Prerequisitos:
    1) FIRASim corriendo (visión multicast en 224.0.0.1:10002, comandos en
       127.0.0.1:20011 — los defaults del engine).
    2) python gen_protos.py   (genera los bindings protobuf, una vez)

Uso típico:
    python firasim_sysid.py                 # batería completa, equipo azul, robot 0
    python firasim_sysid.py --field-units mm # si tu build de FIRASim reporta en mm
    python firasim_sysid.py --team yellow --robot-id 0

Salida: CSVs en training/sysid/data/.  Después:  python fit_sysid.py
"""

from __future__ import annotations

import argparse
import csv
import math
import socket
import struct
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "fira_proto"))

try:
    import fira_packet_pb2          # noqa: E402  (Packet, Environment)
    import fira_command_pb2 as _cmd  # noqa: E402,F401  (cargado por fira_packet)
    import fira_common_pb2 as _common  # noqa: E402,F401
    import fira_replacement_pb2 as _rep  # noqa: E402,F401
except Exception as exc:  # pragma: no cover
    print("[sysid] No se encontraron los bindings protobuf.", file=sys.stderr)
    print("[sysid] Corré primero:  python gen_protos.py", file=sys.stderr)
    print(f"[sysid] (detalle: {exc})", file=sys.stderr)
    raise SystemExit(1)

# --- Constantes del engine (src/radio/commands.rs) ---
WHEEL_R = 0.02          # radio de rueda [m]
WHEEL_L = 0.05          # base entre ruedas [m]
WHEEL_RAD_S_MAX = 70.0  # tope de rueda [rad/s]  → v_max ≈ 1.4 m/s

# --- Red (defaults del engine) ---
CMD_PORT = 20011
VISION_GROUP = "224.0.0.1"
VISION_PORT = 10002


def vw_to_wheels(v: float, w: float) -> tuple[float, float]:
    """(v lineal m/s, ω rad/s) → (wheel_left, wheel_right) rad/s. Igual que el engine."""
    inv_r = 1.0 / WHEEL_R
    half_l_over_r = WHEEL_L / (2.0 * WHEEL_R)
    wl = v * inv_r - w * half_l_over_r
    wr = v * inv_r + w * half_l_over_r
    clamp = lambda x: max(-WHEEL_RAD_S_MAX, min(WHEEL_RAD_S_MAX, x))
    return clamp(wl), clamp(wr)


class FiraIO:
    """Envía comandos/replacement a FIRASim y lee la visión multicast."""

    def __init__(self, cmd_ip: str, team_yellow: bool, robot_id: int,
                 field_units: str):
        self.addr = (cmd_ip, CMD_PORT)
        self.team_yellow = team_yellow
        self.robot_id = robot_id
        # Escala de unidades: si FIRASim reporta/espera mm, convertimos a/desde m.
        self.u = 1000.0 if field_units == "mm" else 1.0

        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except (AttributeError, OSError):
            pass  # SO_REUSEPORT no existe en Windows; no es necesario
        self.rx.bind(("", VISION_PORT))
        mreq = struct.pack("4sl", socket.inet_aton(VISION_GROUP), socket.INADDR_ANY)
        self.rx.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        self.rx.setblocking(False)

    # --- envío ---
    def send_cmd(self, v: float, w: float) -> None:
        wl, wr = vw_to_wheels(v, w)
        pkt = fira_packet_pb2.Packet()
        c = pkt.cmd.robot_commands.add()
        c.id = self.robot_id
        c.yellowteam = self.team_yellow
        c.wheel_left = wl
        c.wheel_right = wr
        self.tx.sendto(pkt.SerializeToString(), self.addr)

    def teleport(self, x: float, y: float, theta: float,
                 ball: tuple[float, float, float, float] | None = None) -> None:
        """Teleporta el robot (en metros, se escala a unidades de FIRASim) y, opcional,
        coloca la pelota con velocidad inicial `ball=(x,y,vx,vy)` en m y m/s."""
        pkt = fira_packet_pb2.Packet()
        r = pkt.replace.robots.add()
        r.position.robot_id = self.robot_id
        r.position.x = x * self.u
        r.position.y = y * self.u
        r.position.orientation = theta
        r.yellowteam = self.team_yellow
        r.turnon = True
        if ball is not None:
            bx, by, bvx, bvy = ball
            pkt.replace.ball.x = bx * self.u
            pkt.replace.ball.y = by * self.u
            pkt.replace.ball.vx = bvx * self.u
            pkt.replace.ball.vy = bvy * self.u
        self.tx.sendto(pkt.SerializeToString(), self.addr)

    # --- recepción ---
    def latest_state(self) -> dict | None:
        """Drena el socket y devuelve el estado más reciente del robot + pelota (en m, m/s)."""
        last = None
        while True:
            try:
                data, _ = self.rx.recvfrom(65536)
            except (BlockingIOError, OSError):
                break
            last = data
        if last is None:
            return None
        env = fira_packet_pb2.Environment()
        try:
            env.ParseFromString(last)
        except Exception:
            return None
        frame = env.frame
        robots = frame.robots_yellow if self.team_yellow else frame.robots_blue
        rob = next((rr for rr in robots if rr.robot_id == self.robot_id), None)
        if rob is None:
            return None
        s = 1.0 / self.u
        return {
            "x": rob.x * s, "y": rob.y * s, "theta": rob.orientation,
            "vx": rob.vx * s, "vy": rob.vy * s, "w": rob.vorientation,
            "ball_x": frame.ball.x * s, "ball_y": frame.ball.y * s,
            "ball_vx": frame.ball.vx * s, "ball_vy": frame.ball.vy * s,
        }


def run_segment(io: FiraIO, dur: float, v: float, w: float,
                phase: str, hz: float = 60.0) -> list[dict]:
    """Comanda `(v,w)` durante `dur` s a `hz`, registrando el estado de la visión."""
    samples: list[dict] = []
    t0 = time.perf_counter()
    dt = 1.0 / hz
    next_t = t0
    while (now := time.perf_counter()) - t0 < dur:
        io.send_cmd(v, w)
        st = io.latest_state()
        if st is not None:
            st.update({"t": now - t0, "phase": phase, "cmd_v": v, "cmd_w": w})
            samples.append(st)
        next_t += dt
        sleep = next_t - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)
    return samples


_FIELDS = ["t", "phase", "cmd_v", "cmd_w", "x", "y", "theta",
           "vx", "vy", "w", "ball_x", "ball_y", "ball_vx", "ball_vy"]


def write_csv(path: Path, samples: list[dict]) -> None:
    with path.open("w", newline="") as f:
        wri = csv.DictWriter(f, fieldnames=_FIELDS, extrasaction="ignore")
        wri.writeheader()
        wri.writerows(samples)
    print(f"[sysid] {path.name}: {len(samples)} muestras")


def preflight(io: FiraIO) -> bool:
    """Verifica conectividad y unidades: teleporta a (-0.5,0) y reporta lo que ve FIRASim."""
    print("[sysid] Preflight: teleportando robot a (-0.50, 0.00) m...")
    for _ in range(20):
        io.teleport(-0.5, 0.0, 0.0)
        time.sleep(0.05)
    t0 = time.perf_counter()
    st = None
    while time.perf_counter() - t0 < 3.0:
        io.send_cmd(0.0, 0.0)
        st = io.latest_state() or st
        time.sleep(0.02)
    if st is None:
        print("[sysid] ✗ No llegan frames de visión. ¿FIRASim corriendo? "
              "¿multicast 224.0.0.1:10002? ¿firewall?", file=sys.stderr)
        return False
    print(f"[sysid] ✓ Visión OK. Robot reportado en x={st['x']:.3f}, y={st['y']:.3f} "
          f"(esperado ~ -0.5, 0.0 en metros).")
    if abs(st["x"]) > 5.0:
        print("[sysid] ⚠ x reportado es grande → tu FIRASim probablemente está en mm. "
              "Reintentá con  --field-units mm", file=sys.stderr)
        return False
    return True


def main() -> int:
    p = argparse.ArgumentParser(description="System-ID de FIRASim para calibrar el sim.")
    p.add_argument("--cmd-ip", default="127.0.0.1")
    p.add_argument("--team", choices=["blue", "yellow"], default="blue")
    p.add_argument("--robot-id", type=int, default=0)
    p.add_argument("--field-units", choices=["m", "mm"], default="m",
                   help="unidades que usa tu FIRASim (default m).")
    p.add_argument("--outdir", type=Path, default=HERE / "data")
    p.add_argument("--skip-preflight", action="store_true")
    args = p.parse_args()

    args.outdir.mkdir(parents=True, exist_ok=True)
    io = FiraIO(args.cmd_ip, args.team == "yellow", args.robot_id, args.field_units)

    if not args.skip_preflight and not preflight(io):
        return 1

    # --- 1) Steps lineales (vel máx, aceleración, fricción) ---
    # Tope físico v_max ≈ wheel_max * r = 1.4 m/s. Probamos varias fracciones.
    for v in (0.4, 0.8, 1.2, 1.4):
        print(f"[sysid] Step lineal v={v} m/s ...")
        io.teleport(-0.55, 0.0, 0.0)          # arrancar de un extremo, mirar +x
        time.sleep(0.4)
        accel = run_segment(io, dur=0.6, v=v, w=0.0, phase="accel")
        coast = run_segment(io, dur=0.5, v=0.0, w=0.0, phase="coast")
        for s in coast:
            s["t"] += 0.6                      # continuidad temporal
        write_csv(args.outdir / f"lin_step_v{v:.2f}.csv", accel + coast)

    # --- 2) Steps angulares (ω máx, aceleración angular) ---
    for w in (3.0, 6.0, 9.0):
        print(f"[sysid] Step angular w={w} rad/s ...")
        io.teleport(0.0, 0.0, 0.0)             # gira en el lugar: seguro en cualquier lado
        time.sleep(0.4)
        accel = run_segment(io, dur=0.8, v=0.0, w=w, phase="accel")
        coast = run_segment(io, dur=0.5, v=0.0, w=0.0, phase="coast")
        for s in coast:
            s["t"] += 0.8
        write_csv(args.outdir / f"ang_step_w{w:.1f}.csv", accel + coast)

    # --- 3) Fricción de la pelota ---
    print("[sysid] Fricción de pelota (vx0=1.0 m/s) ...")
    io.teleport(0.0, 0.55, math.pi / 2.0)      # robot fuera del camino de la pelota
    io.teleport(0.0, 0.55, math.pi / 2.0, ball=(-0.5, 0.0, 1.0, 0.0))
    time.sleep(0.1)
    ball = run_segment(io, dur=1.5, v=0.0, w=0.0, phase="ball")
    write_csv(args.outdir / "ball_friction.csv", ball)

    print(f"[sysid] Listo. CSVs en {args.outdir}. Ahora corré:  python fit_sysid.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
