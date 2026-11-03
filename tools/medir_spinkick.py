#!/usr/bin/env python3
"""Ángulo de salida real del SpinKick, desde registros de `skill_test --mode skill --skill spinkick
--log` (robot real o FIRASim) o CSV de `motion_bench --csv-dir`.

    python3 tools/medir_spinkick.py ~/uni/spin_r*.csv
    python3 tools/medir_spinkick.py /tmp/spin90/spinkick_side_r*.csv --target 0.75,0

Por cada registro, desde que el robot empieza a girar:
- **salida:** dirección de la pelota desde su reposo hasta 0.3 s después de que se movió 3 cm;
- **φ:** ángulo, en el sentido del giro, entre la recta centro del robot → pelota (al empezar a
  girar) y la salida. Es lo que va en `skills.spin_exit_angle_deg`: 90 si sale tangente, ~26 con
  la cara cuadrada (lo medido en FIRASim);
- **error:** salida contra la dirección pelota → objetivo (`target_x/target_y` del registro, o
  `--target`), y la rapidez de la pelota.

Solo la biblioteca estándar.
"""
import argparse
import csv
import math
import statistics
import sys

MOVED_M = 0.03
EXIT_WINDOW_S = 0.30


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def load(path):
    rows = list(csv.DictReader(open(path, newline="")))
    if not rows:
        return []
    if "t_ms" in rows[0]:  # skill_test --log
        def get(r):
            return (float(r["t_ms"]) / 1000.0, r["pose_x"], r["pose_y"], r["ball_x"], r["ball_y"],
                    r["cmd_omega"], r.get("target_x"), r.get("target_y"))
    else:  # motion_bench --csv-dir
        def get(r):
            return (float(r["t"]), r["x"], r["y"], r["ball_x"], r["ball_y"], r["cmd_omega"], None, None)
    out = []
    for r in rows:
        t, x, y, bx, by, w, tx, ty = get(r)
        if "" in (x, y, bx, by, w):
            continue
        tgt = (float(tx), float(ty)) if tx not in (None, "") and ty not in (None, "") else None
        out.append((t, float(x), float(y), float(bx), float(by), float(w), tgt))
    return out


def measure(rows, target):
    w_max = max((abs(r[5]) for r in rows), default=0.0)
    if w_max < 1.0:
        return None, "no giró"
    k0 = next(k for k, r in enumerate(rows) if abs(r[5]) >= 0.9 * w_max)
    sign = 1.0 if rows[k0][5] > 0 else -1.0
    before = rows[max(0, k0 - 5):k0 + 1]
    bx0 = statistics.median(r[3] for r in before)
    by0 = statistics.median(r[4] for r in before)
    cx, cy = rows[k0][1], rows[k0][2]
    i = next((k for k in range(k0, len(rows)) if math.hypot(rows[k][3] - bx0, rows[k][4] - by0) > MOVED_M), None)
    if i is None:
        return None, "la pelota no se movió"
    j = next((k for k in range(i, len(rows)) if rows[k][0] >= rows[i][0] + EXIT_WINDOW_S), len(rows) - 1)
    ex, ey = rows[j][3] - bx0, rows[j][4] - by0
    exit_deg = math.degrees(math.atan2(ey, ex))
    radial_deg = math.degrees(math.atan2(by0 - cy, bx0 - cx))
    phi = wrap_deg(exit_deg - radial_deg) * sign
    tgt = target or rows[k0][6]
    err = wrap_deg(exit_deg - math.degrees(math.atan2(tgt[1] - by0, tgt[0] - bx0))) if tgt else None
    speed = math.hypot(rows[j][3] - rows[i][3], rows[j][4] - rows[i][4]) / max(rows[j][0] - rows[i][0], 1e-3)
    return {"phi": phi, "err": err, "speed": speed, "contact": math.hypot(bx0 - cx, by0 - cy),
            "sense": "CCW" if sign > 0 else "CW"}, None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--target", help="x,y del objetivo si el registro no lo trae (motion_bench)")
    args = ap.parse_args()
    target = tuple(float(v) for v in args.target.split(",")) if args.target else None
    phis = []
    for path in args.csv:
        m, why = measure(load(path), target)
        if m is None:
            print(f"{path}: {why}")
            continue
        phis.append(m["phi"])
        err = f"error {m['err']:+.0f}°" if m["err"] is not None else "sin objetivo"
        print(f"{path}: φ {m['phi']:.0f}° ({m['sense']}), {err}, pelota {m['speed']:.2f} m/s, "
              f"centro-pelota {m['contact'] * 100:.1f} cm")
    if not phis:
        sys.exit(1)
    print(f"\nφ mediana {statistics.median(phis):.0f}° en {len(phis)} tiros "
          f"(mín {min(phis):.0f}°, máx {max(phis):.0f}°): skills.spin_exit_angle_deg = {statistics.median(phis):.0f}")


if __name__ == "__main__":
    main()
