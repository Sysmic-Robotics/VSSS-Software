#!/usr/bin/env python3
"""Dinámica observada en registros de partido (`VSSL_MATCH_LOG`), reales o del simulador, para
calibrar el simulador y el coach con la cancha de verdad.

    python3 tools/medir_salita.py logs/salita/2026-10-30/p1_azul.csv [más.csv ...]

- **Pelota libre:** tramos de ≥ 0.4 s sin ningún robot (propio o rival) a menos de 12 cm, lejos
  de las paredes y a más de 0.15 m/s al empezar. En cada tramo ajusta el frenado exponencial
  (v = v0·e^(−k·t), el modelo de `skills.intercept_ball_decay_per_s`) y uno constante
  (v = v0 − d·t), y dice cuál ajusta mejor.
- **Robots propios:** rapidez, aceleración y giro en juego (percentiles 50/95 y máximo), para
  comparar con `motion.max_linear_speed`, `motion.max_linear_accel` y el tope del firmware.

Solo la biblioteca estándar.
"""
from __future__ import annotations

import csv
import math
import statistics
import sys
from collections import defaultdict
from dataclasses import dataclass, field

NEAR_M = 0.12          # robot a menos de esto: la pelota no rueda libre
WALL_X, WALL_Y = 0.68, 0.58
MIN_SEG_S = 0.4
MIN_V0 = 0.15
DIFF_S = 0.05          # media ventana de la diferencia centrada (s)


@dataclass
class Tick:
    t: float
    ball: tuple[float, float]
    robots: list[tuple[float, float]] = field(default_factory=list)          # todos (propios y rivales)
    own: dict[str, tuple[float, float, float]] = field(default_factory=dict)  # id -> (vx, vy, ω)


def load(path: str) -> list[Tick]:
    by_tick: dict[int, Tick] = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            if not r.get("pose_x") or not r.get("ball_x"):
                continue
            try:
                k = int(r["tick"])
                tk = by_tick.get(k)
                if tk is None:
                    tk = by_tick[k] = Tick(float(r["t_ms"]) / 1000.0, (float(r["ball_x"]), float(r["ball_y"])))
                tk.robots.append((float(r["pose_x"]), float(r["pose_y"])))
                if r.get("own") == "1" and r.get("vel_x"):
                    tk.own[r["robot"]] = (float(r["vel_x"]), float(r["vel_y"] or 0), float(r.get("omega") or 0))
            except (ValueError, KeyError):
                continue  # fila truncada (registro en vivo)
    return [by_tick[k] for k in sorted(by_tick)]


def ball_speeds(ticks: list[Tick]) -> list[float | None]:
    """Rapidez de la pelota por diferencia centrada de posiciones (±DIFF_S)."""
    out: list[float | None] = []
    j0 = j1 = 0
    for i, tk in enumerate(ticks):
        while j0 < i and ticks[j0].t < tk.t - DIFF_S:
            j0 += 1
        j1 = max(j1, i)
        while j1 + 1 < len(ticks) and ticks[j1 + 1].t <= tk.t + DIFF_S:
            j1 += 1
        a, b = ticks[j0], ticks[j1]
        dt = b.t - a.t
        out.append(math.dist(a.ball, b.ball) / dt if dt > 0.02 else None)
    return out


def fit_line(xs: list[float], ys: list[float]) -> tuple[float, float, float]:
    """Mínimos cuadrados y = a + b·x: (a, b, R²)."""
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0:
        return my, 0.0, 0.0
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    a = my - b * mx
    ss = sum((y - my) ** 2 for y in ys)
    res = sum((y - a - b * x) ** 2 for x, y in zip(xs, ys))
    return a, b, (1 - res / ss) if ss > 0 else 0.0


def free_ball_segments(ticks: list[Tick]) -> list[dict]:
    speeds = ball_speeds(ticks)
    free = [
        v is not None and abs(tk.ball[0]) < WALL_X and abs(tk.ball[1]) < WALL_Y
        and all(math.dist(p, tk.ball) >= NEAR_M for p in tk.robots)
        for tk, v in zip(ticks, speeds)
    ]
    segs, i = [], 0
    while i < len(ticks):
        if not free[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(ticks) and free[j + 1] and ticks[j + 1].t - ticks[j].t < 0.1:
            j += 1
        span = ticks[j].t - ticks[i].t
        if span >= MIN_SEG_S and speeds[i] >= MIN_V0:
            ts = [ticks[k].t - ticks[i].t for k in range(i, j + 1) if speeds[k] and speeds[k] > 0.02]
            vs = [speeds[k] for k in range(i, j + 1) if speeds[k] and speeds[k] > 0.02]
            if len(ts) >= 6:
                _, slope_log, r2_exp = fit_line(ts, [math.log(v) for v in vs])
                _, slope_lin, r2_lin = fit_line(ts, vs)
                segs.append({"v0": vs[0], "dur": span, "k": -slope_log, "d": -slope_lin,
                             "r2_exp": r2_exp, "r2_lin": r2_lin})
        i = j + 1
    return segs


def robot_dynamics(ticks: list[Tick]) -> dict[str, list[float]]:
    out = defaultdict(list)
    prev: dict[str, tuple[float, float]] = {}
    for tk in ticks:
        for rid, (vx, vy, w) in tk.own.items():
            v = math.hypot(vx, vy)
            out["v"].append(v)
            out["w"].append(abs(w))
            if rid in prev:
                t0, v0 = prev[rid]
                if 0.08 <= tk.t - t0:
                    out["a"].append(abs(v - v0) / (tk.t - t0))
                    prev[rid] = (tk.t, v)
            else:
                prev[rid] = (tk.t, v)
    return out


def pct(xs: list[float], q: float) -> float:
    s = sorted(xs)
    return s[min(len(s) - 1, int(q * len(s)))] if s else float("nan")


def report(paths: list[str]) -> str:
    segs, dyn = [], defaultdict(list)
    for p in paths:
        ticks = load(p)
        segs += free_ball_segments(ticks)
        for k, v in robot_dynamics(ticks).items():
            dyn[k] += v
    lines = [f"Registros: {len(paths)}"]
    if segs:
        ks = [s["k"] for s in segs]; ds = [s["d"] for s in segs]
        better = sum(1 for s in segs if s["r2_exp"] >= s["r2_lin"])
        lines += [
            f"Pelota libre: {len(segs)} tramos (v0 mediana {statistics.median(s['v0'] for s in segs):.2f} m/s, "
            f"{statistics.median(s['dur'] for s in segs):.2f} s)",
            f"  frenado exponencial k = {statistics.median(ks):.2f} 1/s (p25 {pct(ks, .25):.2f}, p75 {pct(ks, .75):.2f})"
            f"  → skills.intercept_ball_decay_per_s",
            f"  frenado constante   d = {statistics.median(ds):.2f} m/s² (p25 {pct(ds, .25):.2f}, p75 {pct(ds, .75):.2f})",
            f"  ajusta mejor el exponencial en {better} de {len(segs)} tramos "
            f"(R² mediano exp {statistics.median(s['r2_exp'] for s in segs):.2f}, const {statistics.median(s['r2_lin'] for s in segs):.2f})",
        ]
    else:
        lines.append("Pelota libre: ningún tramo útil (¿la pelota rodó lejos de los robots y de las paredes?)")
    if dyn["v"]:
        lines += [
            f"Robots propios: rapidez p50 {pct(dyn['v'], .5):.2f}, p95 {pct(dyn['v'], .95):.2f}, máx {max(dyn['v']):.2f} m/s",
            f"  aceleración p95 {pct(dyn['a'], .95):.2f}, p99 {pct(dyn['a'], .99):.2f} m/s²; "
            f"giro p95 {pct(dyn['w'], .95):.1f}, máx {max(dyn['w']):.1f} rad/s",
        ]
    return "\n".join(lines)


def main(argv: list[str]) -> int:
    if not argv or argv[0] in ("-h", "--help", "--ayuda"):
        print(__doc__)
        return 0 if argv else 1
    print(report(argv))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
