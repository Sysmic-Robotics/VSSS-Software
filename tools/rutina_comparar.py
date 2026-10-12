#!/usr/bin/env python3
"""Compara dos corridas de la misma rutina desde el CSV de partido (las columnas de
rutina): la del robot real con la de FIRASim (con la capa de actuador real), o dos
simuladas.

Uso:
  python3 tools/rutina_comparar.py A.csv B.csv [--robot-a N] [--robot-b M] [--json]

Por cada repetición presente en las dos (filas propias con `rutina_fase` = ejecutando):
  - alinea el inicio de la vuelta (t = 0 en la primera fila de esa repetición);
  - MEE: distancia media entre las posiciones en el mismo instante, en una grilla de
    20 Hz sobre el tramo común (m);
  - DTW de las trayectorias en el plano (`sysid_comun.dtw`, costo medio por paso, m).
Sin --robot-a o --robot-b, toma el primer robot con puesto de cada archivo.

Solo biblioteca estándar.
"""

import argparse
import csv
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sysid_comun as sc  # noqa: E402

DT = 0.05  # 20 Hz


def leer(path, robot=None):
    """Poses por repetición del robot: ({repetición: [(t_s, x, y), ...]}, robot)."""
    vueltas = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            if row.get("own") != "1" or row.get("rutina_fase") != "ejecutando" or not row.get("puesto"):
                continue
            rid = int(row["robot"])
            if robot is None:
                robot = rid
            if rid != robot:
                continue
            try:
                t = float(row["t_ms"]) / 1000.0
                x, y = float(row["pose_x"]), float(row["pose_y"])
                rep = int(row["repeticion"])
            except (TypeError, ValueError):
                continue
            vueltas.setdefault(rep, []).append((t, x, y))
    for poses in vueltas.values():
        poses.sort()
    return vueltas, robot


def interpolar(poses, t):
    """(x, y) en el instante t, lineal entre muestras (constante fuera del rango)."""
    if t <= poses[0][0]:
        return poses[0][1:]
    if t >= poses[-1][0]:
        return poses[-1][1:]
    lo, hi = 0, len(poses) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if poses[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    a, b = poses[lo], poses[hi]
    f = 0.0 if b[0] == a[0] else (t - a[0]) / (b[0] - a[0])
    return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]))


def grilla(poses):
    return [interpolar(poses, k * DT) for k in range(int(poses[-1][0] / DT) + 1)]


def comparar_vuelta(a, b):
    ra = [(t - a[0][0], x, y) for t, x, y in a]
    rb = [(t - b[0][0], x, y) for t, x, y in b]
    comun = min(ra[-1][0], rb[-1][0])
    dist = []
    for k in range(int(comun / DT) + 1):
        pa, pb = interpolar(ra, k * DT), interpolar(rb, k * DT)
        dist.append(math.hypot(pa[0] - pb[0], pa[1] - pb[1]))
    return {
        "mee_m": round(sum(dist) / len(dist), 4),
        "dtw_m": round(sc.dtw(grilla(ra), grilla(rb)), 4),
        "duracion_a_s": round(ra[-1][0], 3),
        "duracion_b_s": round(rb[-1][0], 3),
    }


def comparar(path_a, path_b, robot_a=None, robot_b=None):
    va, ra = leer(path_a, robot_a)
    vb, rb = leer(path_b, robot_b)
    vueltas = {
        rep: comparar_vuelta(va[rep], vb[rep])
        for rep in sorted(set(va) & set(vb))
        if len(va[rep]) >= 2 and len(vb[rep]) >= 2
    }

    def promedio(clave):
        if not vueltas:
            return None
        return round(sum(v[clave] for v in vueltas.values()) / len(vueltas), 4)

    return {"robot_a": ra, "robot_b": rb, "vueltas": vueltas, "mee_m": promedio("mee_m"), "dtw_m": promedio("dtw_m")}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("a")
    ap.add_argument("b")
    ap.add_argument("--robot-a", type=int)
    ap.add_argument("--robot-b", type=int)
    ap.add_argument("--json", action="store_true", help="salida en JSON")
    args = ap.parse_args()
    res = comparar(args.a, args.b, args.robot_a, args.robot_b)
    if args.json:
        print(json.dumps(res, indent=2))
        return 0
    if not res["vueltas"]:
        print("no hay vueltas en común (¿son CSV de partido con las columnas de rutina?)", file=sys.stderr)
        return 1
    print(f"robot {res['robot_a']} ({args.a}) contra robot {res['robot_b']} ({args.b})")
    print(f"{'vuelta':>6} {'MEE m':>7} {'DTW m':>7} {'dur. A s':>9} {'dur. B s':>9}")
    for rep, v in res["vueltas"].items():
        print(f"{rep:>6} {v['mee_m']:>7.4f} {v['dtw_m']:>7.4f} {v['duracion_a_s']:>9.3f} {v['duracion_b_s']:>9.3f}")
    print(f"{'media':>6} {res['mee_m']:>7.4f} {res['dtw_m']:>7.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
