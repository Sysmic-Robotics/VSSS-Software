#!/usr/bin/env python3
"""Etapa 1-2 del selector de jugadas: en qué terminó cada decisión del registro de cartas.

Lee uno o más `*.cartas.jsonl` (los escribe el coach junto al registro de partido, ver
src/coach/cartas.rs), etiqueta cada decisión de juego abierto con lo que pasó en los
`--horizonte` segundos siguientes (gol a favor, gol en contra, pérdida de la pelota, avance de
la pelota hacia el arco rival) y resume por carta elegida y por qué tan abierto estaba el arco.

    python3 tools/cartas_resultado.py logs/salita/5nov/*.cartas.jsonl
    python3 tools/cartas_resultado.py "/root/bateria/*/blue.cartas.jsonl" --horizonte 3

Un gol a favor se ve como el cambio de jugada a KickoffTheirs (saca el rival) y uno en contra
como KickoffOurs, con la pelota cerca del arco correspondiente justo antes.
"""
import argparse
import glob
import gzip
import json
from collections import defaultdict

DECISIONES_POR_S = 10.0


def leer(rutas):
    for ruta in rutas:
        lineas = []
        abrir = gzip.open if ruta.endswith(".gz") else open
        with abrir(ruta, "rt", encoding="utf-8") as f:
            for l in f:
                l = l.strip()
                if l:
                    try:
                        lineas.append(json.loads(l))
                    except json.JSONDecodeError:
                        pass  # última línea cortada al cerrar el engine
        yield ruta, lineas


def goles(lineas):
    """(t, +1 a favor / -1 en contra) por cada saque tras un gol."""
    out = []
    for l in lineas:
        if l.get("evento") != "jugada":
            continue
        x = (l.get("ultima_pelota") or [0, 0])[0]
        if l["play"] == "KickoffTheirs" and x > 0.55:
            out.append((l["t"], +1))
        elif l["play"] == "KickoffOurs" and x < -0.55:
            out.append((l["t"], -1))
    return out


def carta(l):
    e = ((l.get("eleccion") or {}).get("atacante") or {}).get("skill") or "-"
    j = l.get("jugadas") or {}
    if j.get("arrancada"):
        return "arrancada"
    if j.get("pase_y_desvio"):
        return "pase_y_desvio"
    return e


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("rutas", nargs="+", help="archivos o patrones *.cartas.jsonl")
    ap.add_argument("--horizonte", type=float, default=3.0, help="segundos después de cada decisión (3)")
    a = ap.parse_args()
    rutas = sorted({r for p in a.rutas for r in (glob.glob(p) or [p])})
    h = int(a.horizonte * DECISIONES_POR_S)
    por_carta = defaultdict(lambda: defaultdict(float))
    por_abierto = defaultdict(lambda: defaultdict(float))
    partidos = gf = gc = 0
    for _, lineas in leer(rutas):
        partidos += 1
        gs = goles(lineas)
        gf += sum(1 for _, s in gs if s > 0)
        gc += sum(1 for _, s in gs if s < 0)
        # Sin pelota a la vista el engine guarda una posición de relleno fuera de la cancha.
        def valida(l):
            p = l.get("pelota") or [None, None]
            return p[0] is not None and p[1] is not None and abs(p[0]) <= 0.85 and abs(p[1]) <= 0.70

        dec = [l for l in lineas if l.get("evento") is None and l.get("play") == "Open" and valida(l)]
        for i, l in enumerate(dec):
            t = l["t"]
            fut = [m for m in dec[i + 1:i + 1 + h] if m["t"] - t <= h]
            gol = next((s for tg, s in gs if 0 < tg - t <= h), 0)
            perdida = l.get("posesion") == "nuestra" and any(m.get("posesion") == "rival" for m in fut)
            avance = (fut[-1]["pelota"][0] - l["pelota"][0]) if fut else 0.0
            k = carta(l)
            for tabla, clave in ((por_carta, k), (por_abierto, (k, round((l.get("directo") or {}).get("abierto", 0) * 4) / 4))):
                c = tabla[clave]
                c["n"] += 1
                c["gf"] += gol > 0
                c["gc"] += gol < 0
                c["perdida"] += perdida
                c["avance"] += avance
    print(f"{partidos} partido(s), {gf} goles a favor y {gc} en contra. Horizonte {a.horizonte:g} s.")
    print(f"{'carta elegida':<16}{'decisiones':>11}{'gol a favor':>13}{'gol en contra':>15}{'pérdida':>9}{'avance m':>10}")
    for k, c in sorted(por_carta.items(), key=lambda kv: -kv[1]["n"]):
        n = c["n"]
        print(f"{k:<16}{int(n):>11}{100 * c['gf'] / n:>12.1f}%{100 * c['gc'] / n:>14.1f}%{100 * c['perdida'] / n:>8.0f}%{c['avance'] / n:>10.3f}")
    print("\nTiro (empujar o conducir) según qué tanto del arco se veía abierto:")
    for (k, ab), c in sorted(por_abierto.items()):
        if k not in ("Dribble", "ShootPush") or c["n"] < 20:
            continue
        n = c["n"]
        print(f"  {k:<10} abierto {ab:>4.2f}: {int(n):>6} decisiones, gol a favor {100 * c['gf'] / n:5.1f}%, pérdida {100 * c['perdida'] / n:4.0f}%")


if __name__ == "__main__":
    main()
