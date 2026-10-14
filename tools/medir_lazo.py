"""Frecuencia del lazo de control a partir del registro de partido (`VSSL_MATCH_LOG`).

    python3 tools/medir_lazo.py sin_gui.csv gui_antes.csv gui_nueva.csv
    python3 tools/medir_lazo.py --base gui_antes.csv gui_nueva.csv

El registro escribe una fila por comando propio y por rival activo, todas con el
`t_ms` y el `tick` del tick. Se toma un `t_ms` por tick y se miden los intervalos
solo entre ticks consecutivos: si un tick no escribió filas (sin comandos ni
rivales), su intervalo no se inventa y se cuenta como tick faltante.

Por archivo informa ticks, frecuencia media, p50/p95/p99 y máximo del intervalo, y
el % de intervalos de más de 20 y de 33 ms. Con `--base`, además dice si cada
archivo cumple el criterio de la GUI de depuración: media >= 62.0 Hz,
p99 <= 20 ms, <= 0.5 % de intervalos > 33 ms, y no peor que la base en más de
0.3 Hz de media ni 2 ms de p99.
"""

import argparse
import csv
import math
import sys

MIN_HZ = 62.0
MAX_P99_MS = 20.0
MAX_PCT_33 = 0.5
MAX_HZ_DROP = 0.3
MAX_P99_RISE_MS = 2.0


def tiempos_por_tick(path):
    """Pares `(tick, t_ms)` ordenados por tick, uno por tick."""
    por_tick = {}
    with open(path, newline="") as f:
        for fila in csv.DictReader(f):
            try:
                tick = int(fila["tick"])
                t_ms = float(fila["t_ms"])
            except (KeyError, TypeError, ValueError):
                continue
            por_tick.setdefault(tick, t_ms)
    return sorted(por_tick.items())


def intervalos(pares):
    """Intervalos (ms) entre ticks consecutivos y cantidad de ticks faltantes."""
    out = []
    faltan = 0
    for (k0, t0), (k1, t1) in zip(pares, pares[1:]):
        if k1 == k0 + 1:
            out.append(t1 - t0)
        else:
            faltan += k1 - k0 - 1
    return out, faltan


def percentil(valores, p):
    """Percentil con interpolación lineal (como `numpy.percentile` por defecto)."""
    if not valores:
        return math.nan
    xs = sorted(valores)
    pos = (len(xs) - 1) * p / 100.0
    lo = math.floor(pos)
    hi = min(lo + 1, len(xs) - 1)
    return xs[lo] + (xs[hi] - xs[lo]) * (pos - lo)


def resumen(dts, ticks=None, faltan=0):
    """Estadística de una lista de intervalos en ms."""
    n = len(dts)
    total_s = sum(dts) / 1000.0
    return {
        "ticks": ticks if ticks is not None else n + 1,
        "faltan": faltan,
        "hz": n / total_s if total_s > 0 else math.nan,
        "p50": percentil(dts, 50),
        "p95": percentil(dts, 95),
        "p99": percentil(dts, 99),
        "max": max(dts) if dts else math.nan,
        "pct_20": 100.0 * sum(d > 20.0 for d in dts) / n if n else math.nan,
        "pct_33": 100.0 * sum(d > 33.0 for d in dts) / n if n else math.nan,
    }


def medir(path):
    pares = tiempos_por_tick(path)
    dts, faltan = intervalos(pares)
    return resumen(dts, ticks=len(pares), faltan=faltan)


def cumple(r, base=None):
    """Lista de incumplimientos del criterio (vacía si cumple)."""
    fallas = []
    if not r["hz"] >= MIN_HZ:
        fallas.append(f"media {r['hz']:.2f} Hz < {MIN_HZ}")
    if not r["p99"] <= MAX_P99_MS:
        fallas.append(f"p99 {r['p99']:.1f} ms > {MAX_P99_MS}")
    if not r["pct_33"] <= MAX_PCT_33:
        fallas.append(f"{r['pct_33']:.2f} % de intervalos > 33 ms (máximo {MAX_PCT_33} %)")
    if base is not None:
        if r["hz"] < base["hz"] - MAX_HZ_DROP:
            fallas.append(f"media {r['hz']:.2f} Hz, más de {MAX_HZ_DROP} Hz bajo la base ({base['hz']:.2f})")
        if r["p99"] > base["p99"] + MAX_P99_RISE_MS:
            fallas.append(f"p99 {r['p99']:.1f} ms, más de {MAX_P99_RISE_MS} ms sobre la base ({base['p99']:.1f})")
    return fallas


def linea(nombre, r):
    return (
        f"{nombre}: {r['ticks']} ticks ({r['faltan']} faltantes), {r['hz']:.2f} Hz, "
        f"intervalo p50 {r['p50']:.1f} / p95 {r['p95']:.1f} / p99 {r['p99']:.1f} / "
        f"máx {r['max']:.1f} ms, {r['pct_20']:.2f} % > 20 ms, {r['pct_33']:.2f} % > 33 ms"
    )


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("archivos", nargs="+", help="CSV de VSSL_MATCH_LOG")
    ap.add_argument("--base", help="CSV de la condición base (GUI anterior) para el criterio")
    args = ap.parse_args(argv)

    base = medir(args.base) if args.base else None
    if base is not None:
        print(linea(f"{args.base} (base)", base))
    ok = True
    for path in args.archivos:
        r = medir(path)
        print(linea(path, r))
        if base is not None:
            fallas = cumple(r, base)
            ok = ok and not fallas
            print("  cumple el criterio" if not fallas else "  no cumple: " + "; ".join(fallas))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
