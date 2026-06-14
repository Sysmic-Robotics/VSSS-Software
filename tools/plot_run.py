#!/usr/bin/env python3
"""plot_run.py — visualiza un CSV de `scenario.rs` o `skill_test`.

Convierte el log de auditoría (qué se mandó vs qué pasó) en gráficos.

Uso:
    python tools/plot_run.py logs/scenario_go_to_XXXX.csv            # abre ventana
    python tools/plot_run.py logs/run.csv --save run.png             # guarda PNG y muestra
    python tools/plot_run.py logs/run.csv --save run.png --no-show   # solo guarda (headless/WSL)

Paneles:
  1. Trayectoria (pose x,y) + target + punto de inicio  → "dónde está / a dónde va".
  2. Errores en el tiempo (err_dist [m], err_heading [rad]).
  3. Velocidades de rueda L/R [mm/s]                     → "lo que le LLEGA al robot".
  4. Comando vx, vy [m/s] y omega [rad/s]               → "lo que INTENTA hacer".

Requiere matplotlib (`pip install matplotlib`). El resumen de texto funciona sin él.
Columnas esperadas (header de skill_log): t_ms, pose_x, pose_y, pose_theta,
target_x, target_y, cmd_vx, cmd_vy, cmd_omega, wheel_L_mm_s, wheel_R_mm_s,
err_dist, err_heading, skill, robot, transport.
"""
import argparse
import csv
import sys


def fnum(s):
    """Float tolerante: '' o None → None."""
    if s is None:
        return None
    s = s.strip()
    if s == "":
        return None
    try:
        return float(s)
    except ValueError:
        return None


def load(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def col(rows, name):
    return [fnum(d.get(name)) for d in rows]


def pairs(xs, ys):
    """Filtra (x, y) donde ambos son no-None."""
    ox, oy = [], []
    for x, y in zip(xs, ys):
        if x is not None and y is not None:
            ox.append(x)
            oy.append(y)
    return ox, oy


def last_non_none(xs):
    for v in reversed(xs):
        if v is not None:
            return v
    return None


def text_summary(rows):
    n = len(rows)
    print(f"\n=== Resumen del run ({n} ticks) ===")
    if n == 0:
        print("  (CSV vacío)")
        return
    head = rows[0]
    print(f"  skill={head.get('skill')}  robot={head.get('robot')}  "
          f"transport={head.get('transport')}  vision={head.get('vision')}")

    def stat(name, unit=""):
        vals = [v for v in col(rows, name) if v is not None]
        if not vals:
            return
        print(f"  {name:<14} min={min(vals):8.3f}  max={max(vals):8.3f}  "
              f"final={vals[-1]:8.3f} {unit}")

    stat("err_dist", "m")
    stat("err_heading", "rad")
    stat("wheel_L_mm_s", "mm/s")
    stat("wheel_R_mm_s", "mm/s")

    # Alerta de saturación de rueda (±1500 mm/s).
    wl = [v for v in col(rows, "wheel_L_mm_s") if v is not None]
    wr = [v for v in col(rows, "wheel_R_mm_s") if v is not None]
    sat = sum(1 for v in wl + wr if abs(v) >= 1500)
    if sat:
        print(f"  ⚠ {sat} muestras de rueda saturadas a ±1500 mm/s "
              f"(¿velocidad/omega demasiado altos?)")
    print()


def main():
    ap = argparse.ArgumentParser(description="Grafica un CSV de run VSSS.")
    ap.add_argument("csv", help="ruta al CSV (p.ej. logs/scenario_go_to_*.csv)")
    ap.add_argument("--save", default=None, help="guarda el gráfico como PNG")
    ap.add_argument("--no-show", action="store_true",
                    help="no abre ventana (para headless/WSL sin display)")
    args = ap.parse_args()

    try:
        rows = load(args.csv)
    except FileNotFoundError:
        print(f"No existe el archivo: {args.csv}", file=sys.stderr)
        sys.exit(1)

    text_summary(rows)
    if not rows:
        return

    try:
        import matplotlib
        if args.no_show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib no está instalado; instálalo con: pip install matplotlib",
              file=sys.stderr)
        print("(El resumen de texto de arriba sí se generó.)", file=sys.stderr)
        sys.exit(0)

    t = [(v / 1000.0 if v is not None else None) for v in col(rows, "t_ms")]
    head = rows[0]
    title = (f"skill={head.get('skill')}  robot={head.get('robot')}  "
             f"transport={head.get('transport')}")

    fig, axs = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle(title, fontsize=13)

    # 1) Trayectoria
    ax = axs[0][0]
    px, py = pairs(col(rows, "pose_x"), col(rows, "pose_y"))
    if px:
        ax.plot(px, py, "-", color="tab:blue", lw=1.5, label="trayectoria")
        ax.plot(px[0], py[0], "o", color="green", ms=9, label="inicio")
        ax.plot(px[-1], py[-1], "s", color="tab:blue", ms=8, label="fin")
    tx, ty = last_non_none(col(rows, "target_x")), last_non_none(col(rows, "target_y"))
    if tx is not None and ty is not None:
        ax.plot(tx, ty, "*", color="red", ms=16, label="target")
    ax.set_title("Trayectoria (pose) + target")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    # 2) Errores
    ax = axs[0][1]
    ed_x, ed_y = pairs(t, col(rows, "err_dist"))
    plotted = False
    if ed_x:
        ax.plot(ed_x, ed_y, color="tab:red", label="err_dist [m]")
        ax.set_ylabel("err_dist [m]", color="tab:red")
        plotted = True
    eh_x, eh_y = pairs(t, col(rows, "err_heading"))
    if eh_x:
        ax2 = ax.twinx()
        ax2.plot(eh_x, eh_y, color="tab:purple", alpha=0.7, label="err_heading [rad]")
        ax2.set_ylabel("err_heading [rad]", color="tab:purple")
        plotted = True
    ax.set_title("Error vs tiempo" if plotted else "Error vs tiempo (sin datos)")
    ax.set_xlabel("t [s]")
    ax.grid(True, alpha=0.3)

    # 3) Ruedas (lo que le llega al robot)
    ax = axs[1][0]
    wl_x, wl_y = pairs(t, col(rows, "wheel_L_mm_s"))
    wr_x, wr_y = pairs(t, col(rows, "wheel_R_mm_s"))
    if wl_x:
        ax.plot(wl_x, wl_y, color="tab:orange", label="L [mm/s]")
    if wr_x:
        ax.plot(wr_x, wr_y, color="tab:green", label="R [mm/s]")
    ax.axhline(1500, color="grey", ls="--", lw=0.7, alpha=0.6)
    ax.axhline(-1500, color="grey", ls="--", lw=0.7, alpha=0.6)
    ax.set_title("Velocidades de rueda (lo que LLEGA al robot)")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("mm/s")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    # 4) Comando (lo que intenta hacer)
    ax = axs[1][1]
    vx_x, vx_y = pairs(t, col(rows, "cmd_vx"))
    vy_x, vy_y = pairs(t, col(rows, "cmd_vy"))
    if vx_x:
        ax.plot(vx_x, vx_y, color="tab:blue", label="vx [m/s]")
    if vy_x:
        ax.plot(vy_x, vy_y, color="tab:cyan", label="vy [m/s]")
    ax.set_ylabel("v [m/s]")
    om_x, om_y = pairs(t, col(rows, "cmd_omega"))
    if om_x:
        ax3 = ax.twinx()
        ax3.plot(om_x, om_y, color="tab:red", alpha=0.7, label="omega [rad/s]")
        ax3.set_ylabel("omega [rad/s]", color="tab:red")
    ax.set_title("Comando (lo que INTENTA hacer)")
    ax.set_xlabel("t [s]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="upper left")

    fig.tight_layout(rect=[0, 0, 1, 0.97])

    if args.save:
        fig.savefig(args.save, dpi=120)
        print(f"Guardado: {args.save}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
