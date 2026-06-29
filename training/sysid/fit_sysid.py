"""Ajusta los CSVs de `firasim_sysid.py` y produce `sim_calibration.json`.

Extrae de las respuestas medidas en FIRASim:
  - v_max, omega_max      : velocidades máximas alcanzadas.
  - lin_accel, ang_accel  : aceleración (de la rampa de subida, ajuste de 1er orden).
  - lin_decel, ang_decel  : deceleración al soltar el comando (fricción).
  - ball_decel            : deceleración de la pelota (fricción pelota-piso).

Esos valores se usan en la Fase 1/3 para calibrar el integrador del sim y los rangos
de domain randomization (centrados en lo MEDIDO, no adivinado).

Uso:  python fit_sysid.py
Solo requiere numpy (ya está en requirements).
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DATA = HERE / "data"
OUT = HERE / "sim_calibration.json"


def load(path: Path) -> dict[str, np.ndarray]:
    rows: dict[str, list[float]] = {}
    phases: list[str] = []
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            phases.append(row["phase"])
            for k, v in row.items():
                if k == "phase":
                    continue
                rows.setdefault(k, []).append(float(v))
    out = {k: np.asarray(v, dtype=float) for k, v in rows.items()}
    out["_phase"] = np.asarray(phases)
    return out


def fit_first_order_accel(t: np.ndarray, speed: np.ndarray, v_ss: float) -> float:
    """Ajusta speed(t) = v_ss·(1 - e^(-t/τ)) y devuelve la aceleración inicial v_ss/τ."""
    if v_ss <= 1e-6:
        return float("nan")
    mask = (speed > 0.05 * v_ss) & (speed < 0.95 * v_ss) & (t > 0)
    if mask.sum() < 3:
        # fallback: pendiente media de la subida
        if len(t) < 2:
            return float("nan")
        return float(np.max(np.diff(speed) / np.maximum(np.diff(t), 1e-6)))
    y = np.log(1.0 - speed[mask] / v_ss)       # = -t/τ
    slope = np.polyfit(t[mask], y, 1)[0]        # = -1/τ
    if slope >= 0:
        return float("nan")
    tau = -1.0 / slope
    return float(v_ss / tau)


def fit_decel(t: np.ndarray, speed: np.ndarray) -> float:
    """Deceleración (≥0) durante el coast: pendiente de speed vs t mientras se frena."""
    mask = speed > 0.05 * (speed.max() if speed.size else 1.0)
    if mask.sum() < 3:
        return float("nan")
    slope = np.polyfit(t[mask], speed[mask], 1)[0]
    return float(max(0.0, -slope))


def analyze_linear() -> dict:
    files = sorted(DATA.glob("lin_ramp_a*.csv"))
    if not files:
        return {}
    vpeak_all, accel_all, decel_all, tracked_all = [], [], [], []
    for fp in files:
        d = load(fp)
        is_ramp = d["_phase"] == "ramp"
        is_coast = d["_phase"] == "coast"
        if is_ramp.sum() < 5:
            continue
        t = d["t"][is_ramp]
        cmd = d["cmd_v"][is_ramp]
        act = np.hypot(d["vx"], d["vy"])[is_ramp]
        vpeak = float(np.percentile(act, 95))          # vel real sostenida (robusto a picos)
        vpeak_all.append(vpeak)
        # máx velocidad COMANDADA que el robot todavía seguía (act >= 60% del comando)
        tracking = act >= 0.6 * np.maximum(cmd, 1e-6)
        if tracking.any():
            tracked_all.append(float(np.max(cmd[tracking])))
        # aceleración real: pendiente de la subida (entre 5% y 80% del pico)
        rising = (act > 0.05 * vpeak) & (act < 0.80 * vpeak)
        if rising.sum() >= 3:
            slope = float(np.polyfit(t[rising], act[rising], 1)[0])
            if slope > 0:
                accel_all.append(slope)
        if is_coast.sum() >= 3:
            tc = d["t"][is_coast]
            sc = np.hypot(d["vx"], d["vy"])[is_coast]
            dd = fit_decel(tc - tc.min(), sc)
            if not math.isnan(dd):
                decel_all.append(dd)
    return {
        "v_max": max(vpeak_all) if vpeak_all else None,
        "v_tracked_max": max(tracked_all) if tracked_all else None,
        "lin_accel": float(np.median(accel_all)) if accel_all else None,
        "lin_decel": float(np.median(decel_all)) if decel_all else None,
    }


def analyze_angular() -> dict:
    files = sorted(DATA.glob("ang_step_w*.csv"))
    if not files:
        return {}
    w_ss_all, accel_all, decel_all = [], [], []
    for fp in files:
        d = load(fp)
        spd = np.abs(d["w"])
        is_acc = d["_phase"] == "accel"
        is_coast = d["_phase"] == "coast"
        if is_acc.sum() < 3:
            continue
        acc_t, acc_s = d["t"][is_acc], spd[is_acc]
        tail = acc_t >= (acc_t.min() + 0.7 * (acc_t.max() - acc_t.min()))
        w_ss = float(np.median(acc_s[tail])) if tail.any() else float(acc_s.max())
        w_ss_all.append(w_ss)
        a = fit_first_order_accel(acc_t - acc_t.min(), acc_s, w_ss)
        if not math.isnan(a):
            accel_all.append(a)
        if is_coast.sum() >= 3:
            dd = fit_decel(d["t"][is_coast] - d["t"][is_coast].min(), spd[is_coast])
            if not math.isnan(dd):
                decel_all.append(dd)
    return {
        "omega_max": max(w_ss_all) if w_ss_all else None,
        "ang_accel": float(np.median(accel_all)) if accel_all else None,
        "ang_decel": float(np.median(decel_all)) if decel_all else None,
    }


def analyze_ball() -> dict:
    fp = DATA / "ball_friction.csv"
    if not fp.exists():
        return {}
    d = load(fp)
    spd = np.hypot(d["ball_vx"], d["ball_vy"])
    dec = fit_decel(d["t"], spd)
    return {"ball_decel": dec if not math.isnan(dec) else None,
            "ball_v0": float(spd.max()) if spd.size else None}


def main() -> int:
    if not DATA.is_dir() or not any(DATA.glob("*.csv")):
        print(f"[fit] No hay CSVs en {DATA}. Corré primero firasim_sysid.py.")
        return 1

    lin = analyze_linear()
    ang = analyze_angular()
    ball = analyze_ball()

    cal = {
        "source": "firasim system-id",
        "wheel_radius_r": 0.02,
        "wheel_base_l": 0.05,
        "wheel_rad_s_max": 70.0,
        "v_max": lin.get("v_max"),
        "v_tracked_max": lin.get("v_tracked_max"),
        "omega_max": ang.get("omega_max"),
        "lin_accel": lin.get("lin_accel"),
        "ang_accel": ang.get("ang_accel"),
        "lin_decel": lin.get("lin_decel"),
        "ang_decel": ang.get("ang_decel"),
        "ball_decel": ball.get("ball_decel"),
        "dr_ranges_hint": "para domain randomization: multiplicá cada accel/decel por "
                          "U(0.8, 1.2) por episodio (rango ±20% alrededor de lo medido).",
    }

    OUT.write_text(json.dumps(cal, indent=2, ensure_ascii=False), encoding="utf-8")

    print("\n== Calibración medida en FIRASim ==")
    def show(label, key, unit):
        val = cal.get(key)
        print(f"  {label:<22}{'-- ' if val is None else f'{val:8.3f} '}{unit}")
    show("v_max (sostenible)", "v_max", "m/s")
    show("v_tracked_max (cmd)", "v_tracked_max", "m/s")
    show("omega_max", "omega_max", "rad/s")
    show("lin_accel", "lin_accel", "m/s^2")
    show("ang_accel", "ang_accel", "rad/s^2")
    show("lin_decel (fricción)", "lin_decel", "m/s^2")
    show("ang_decel (fricción)", "ang_decel", "rad/s^2")
    show("ball_decel (fricción)", "ball_decel", "m/s^2")
    print(f"\n[fit] Escrito {OUT}")
    print("[fit] Usá estos valores en soccer_env.py (Fase 1/3) y centrá el DR ahí.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
