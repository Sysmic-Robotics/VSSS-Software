#!/usr/bin/env python3
"""Árbitro de operador para el engine (cancha real o simulador sin VSSReferee).

Envía comandos de texto al puerto del árbitro del engine (VSSL_REFEREE_ADDR, por
defecto 224.5.23.2:10003). El engine acepta el mismo formato que VSSReferee (protobuf)
y este texto plano, así que en cancha un humano escribe lo que señala el árbitro.

Uso:
    python tools/referee_cli.py                    # interactivo
    python tools/referee_cli.py kickoff blue       # un comando y salir
    python tools/referee_cli.py --addr 127.0.0.1:10003 free_ball q1

Comandos: KICKOFF <BLUE|YELLOW>, FREE_KICK <BLUE|YELLOW>, PENALTY <BLUE|YELLOW>,
GOAL_KICK <BLUE|YELLOW>, FREE_BALL <Q1|Q2|Q3|Q4>, GAME_ON (o GO), STOP, HALT.
Atajos interactivos: k b / k y (kickoff), f b / f y (free kick), p b / p y (penalty),
g b / g y (goal kick), b 1..4 (free ball), go, s (stop), h (halt), q (salir).
"""

from __future__ import annotations

import socket
import sys

DEFAULT_ADDR = "224.5.23.2:10003"

SHORTCUTS = {
    "k": "KICKOFF", "f": "FREE_KICK", "p": "PENALTY", "g": "GOAL_KICK", "b": "FREE_BALL",
    "go": "GAME_ON", "s": "STOP", "h": "HALT",
}
TEAM = {"b": "BLUE", "y": "YELLOW", "a": "BLUE", "am": "YELLOW"}


def expand(line: str) -> str | None:
    words = line.strip().split()
    if not words:
        return None
    head = words[0].lower()
    cmd = SHORTCUTS.get(head, words[0].upper())
    args = []
    for w in words[1:]:
        lw = w.lower()
        if cmd == "FREE_BALL" and lw in ("1", "2", "3", "4"):
            args.append("Q" + lw)
        elif lw in TEAM:
            args.append(TEAM[lw])
        else:
            args.append(w.upper())
    return " ".join([cmd] + args)


def main(argv: list[str]) -> int:
    addr = DEFAULT_ADDR
    if "--addr" in argv:
        i = argv.index("--addr")
        addr = argv[i + 1]
        argv = argv[:i] + argv[i + 2:]
    host, port = addr.rsplit(":", 1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)

    def send(text: str) -> None:
        sock.sendto(text.encode("ascii"), (host, int(port)))
        print(f"→ {text}")

    if argv:
        cmd = expand(" ".join(argv))
        if cmd is None:
            return 2
        send(cmd)
        return 0

    print(f"árbitro de operador → {addr}. Comandos: k b|y, f b|y, p b|y, g b|y, b 1-4, go, s, h, q")
    while True:
        try:
            line = input("ref> ")
        except (EOFError, KeyboardInterrupt):
            print()
            return 0
        if line.strip().lower() in ("q", "quit", "exit"):
            return 0
        cmd = expand(line)
        if cmd:
            send(cmd)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
