#!/usr/bin/env python3
"""Métricas de un partido a partir del CSV de `VSSL_MATCH_LOG` (rustengine).

Uso:
    python tools/match_metrics.py logs/partido_azul.csv
    python tools/match_metrics.py logs/partido_azul.csv --json   # salida JSON
    python tools/match_metrics.py a.csv b.csv                     # varios partidos, tabla comparada

El CSV tiene una fila por robot y tick (propios con skill/comando, rivales solo pose)
y la pelota repetida en cada fila. Solo usa la biblioteca estándar.

Convención de campo (m): arcos en x = ±0.75, boca |y| ≤ 0.20, área |x| ≥ 0.60 y |y| ≤ 0.35.
El equipo propio ("own"=1) ataca hacia +x si es azul (team 0) y hacia −x si es amarillo.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import Counter, defaultdict

GOAL_X = 0.75
GOAL_HALF_Y = 0.20
WALL_BAND_Y = 0.52
WALL_BAND_X = 0.62
TOUCH_RADIUS = 0.10       # robot a menos de esto de la pelota = contacto
POSSESSION_RADIUS = 0.15  # el más cercano dentro de esto tiene la posesión
TOUCH_GAP_S = 0.5         # nuevo toque si pasó más de esto sin contacto
STUCK_CMD = 0.25          # comando (m/s) por encima del cual "debería moverse"
STUCK_VEL = 0.03          # velocidad medida (m/s) por debajo de la cual "no se mueve"
STRIKER_SKILLS = {"shoot", "approach", "intercept", "chaseball"}


def fnum(s: str):
    return float(s) if s not in ("", None) else None


def load(path: str):
    """Agrupa filas por tick → {tick: {"t": t_ms, "ball": (x,y,vx,vy), "robots": [..]}}."""
    ticks: dict[int, dict] = {}
    own_team = None
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            # Fila truncada (el engine sigue escribiendo el CSV): se ignora.
            if row.get("ball_vy") in (None, "") or row.get("tick") in (None, ""):
                continue
            tick = int(row["tick"])
            entry = ticks.setdefault(tick, {"t": int(row["t_ms"]), "robots": []})
            entry["ball"] = (
                float(row["ball_x"]), float(row["ball_y"]),
                float(row["ball_vx"]), float(row["ball_vy"]),
            )
            own = row["own"] == "1"
            if own and own_team is None:
                own_team = int(row["team"])
            entry["robots"].append({
                "team": int(row["team"]),
                "id": int(row["robot"]),
                "own": own,
                "skill": row["skill"],
                "pos": (fnum(row["pose_x"]), fnum(row["pose_y"])),
                "vel": (fnum(row["vel_x"]), fnum(row["vel_y"])),
                "cmd": (fnum(row["cmd_vx"]), fnum(row["cmd_vy"])),
            })
    return [ticks[k] for k in sorted(ticks)], own_team


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def analyze(path: str) -> dict:
    ticks, own_team = load(path)
    if not ticks or own_team is None:
        return {"archivo": path, "error": "sin filas propias"}
    attack_sign = 1.0 if own_team == 0 else -1.0
    duration_s = (ticks[-1]["t"] - ticks[0]["t"]) / 1000.0
    n = len(ticks)

    goals_for = goals_against = 0
    ball_in_goal = None  # None / "for" / "against" (histéresis para contar una vez)
    poss = Counter()      # "own" / "opp" / "none"
    touches = Counter()   # (team, id) → toques
    last_touch_t = {}     # (team, id) → t_ms del último contacto
    skill_ticks = Counter()
    striker_switches = 0
    last_striker = None
    ball_wall_ticks = 0
    stuck_ticks = Counter()   # id propio → ticks "atascado"
    cmd_ticks = Counter()     # id propio → ticks con comando de avance
    own_ball_dist_sum = 0.0
    ball_own_half_ticks = 0

    for e in ticks:
        bx, by, bvx, bvy = e["ball"]
        # Goles: pelota más allá de la línea de gol dentro de la boca.
        side = None
        if abs(by) <= GOAL_HALF_Y:
            if bx * attack_sign > GOAL_X:
                side = "for"
            elif bx * attack_sign < -GOAL_X:
                side = "against"
        if side and side != ball_in_goal:
            if side == "for":
                goals_for += 1
            else:
                goals_against += 1
        ball_in_goal = side

        if abs(by) > WALL_BAND_Y or (abs(bx) > WALL_BAND_X and abs(by) > GOAL_HALF_Y):
            ball_wall_ticks += 1
        if bx * attack_sign < 0:
            ball_own_half_ticks += 1

        nearest = None
        nearest_d = 1e9
        own_nearest_d = 1e9
        striker_now = None
        for r in e["robots"]:
            if r["pos"][0] is None:
                continue
            d = dist(r["pos"], (bx, by))
            if d < nearest_d:
                nearest, nearest_d = r, d
            if r["own"]:
                own_nearest_d = min(own_nearest_d, d)
                skill_ticks[r["skill"] or "(sin skill)"] += 1
                if r["skill"] in STRIKER_SKILLS:
                    striker_now = r["id"]
                cmd = r["cmd"]
                vel = r["vel"]
                if cmd[0] is not None and vel[0] is not None:
                    cmd_mag = math.hypot(cmd[0], cmd[1])
                    vel_mag = math.hypot(vel[0], vel[1])
                    if cmd_mag > STUCK_CMD:
                        cmd_ticks[r["id"]] += 1
                        if vel_mag < STUCK_VEL:
                            stuck_ticks[r["id"]] += 1
            if d < TOUCH_RADIUS:
                key = (r["team"], r["id"])
                if e["t"] - last_touch_t.get(key, -1e9) > TOUCH_GAP_S * 1000:
                    touches[key] += 1
                last_touch_t[key] = e["t"]
        if nearest is not None and nearest_d < POSSESSION_RADIUS:
            poss["own" if nearest["own"] else "opp"] += 1
        else:
            poss["none"] += 1
        if own_nearest_d < 1e8:
            own_ball_dist_sum += own_nearest_d
        if striker_now is not None and last_striker is not None and striker_now != last_striker:
            striker_switches += 1
        if striker_now is not None:
            last_striker = striker_now

    rules = audit_rules(ticks, own_team, attack_sign)
    own_touches = sum(v for (t, _), v in touches.items() if t == own_team)
    opp_touches = sum(v for (t, _), v in touches.items() if t != own_team)
    total_skill = sum(skill_ticks.values()) or 1
    return {
        "archivo": path,
        "equipo": "azul" if own_team == 0 else "amarillo",
        "duracion_s": round(duration_s, 1),
        "ticks": n,
        "goles_favor": goals_for,
        "goles_contra": goals_against,
        "posesion_propia_pct": round(100 * poss["own"] / n, 1),
        "posesion_rival_pct": round(100 * poss["opp"] / n, 1),
        "pelota_libre_pct": round(100 * poss["none"] / n, 1),
        "toques_propios": own_touches,
        "toques_rival": opp_touches,
        "toques_por_robot_propio": {str(i): v for (t, i), v in sorted(touches.items()) if t == own_team},
        "dist_media_al_balon_m": round(own_ball_dist_sum / n, 3),
        "pelota_en_campo_propio_pct": round(100 * ball_own_half_ticks / n, 1),
        "pelota_en_pared_pct": round(100 * ball_wall_ticks / n, 1),
        "cambios_de_striker": striker_switches,
        "cambios_de_striker_por_min": round(striker_switches / max(duration_s / 60, 1e-6), 1),
        "uso_skills_pct": {k: round(100 * v / total_skill, 1) for k, v in skill_ticks.most_common()},
        "atascado_pct_por_robot": {
            str(i): round(100 * stuck_ticks[i] / cmd_ticks[i], 1) for i in sorted(cmd_ticks) if cmd_ticks[i]
        },
        **rules,
    }


AREA_X = 0.60
AREA_HALF_Y = 0.35
RETENTION_TICKS = 600  # 10 s a 60 Hz


def in_area(p, side):
    """Centro del robot/pelota dentro del área del arco del lado `side` (signo de x)."""
    return p[0] * side >= AREA_X and abs(p[1]) <= AREA_HALF_Y


def audit_rules(ticks, own_team, attack_sign) -> dict:
    """Auditor de reglas (LARC 2026 §9.4-9.5) sobre el registro, para ambos equipos.

    Faltas de área: jugador de campo (no arquero) dentro del área propia; dos o más
    atacantes dentro del área rival; retención: pelota más de 10 s dentro del área.
    El arquero propio es el robot con skill `goalkeep` (o el más cercano al arco);
    el rival, su robot más cercano a su arco.
    """
    own_side, opp_side = -attack_sign, attack_sign
    own_goal = (own_side * 0.75, 0.0)
    opp_goal = (opp_side * 0.75, 0.0)
    stats = {
        "own_area_ticks": 0, "own_area_episodes": 0,
        "own_double_ticks": 0, "own_retention_episodes": 0,
        "opp_area_ticks": 0, "opp_area_episodes": 0,
        "opp_double_ticks": 0, "opp_retention_episodes": 0,
    }
    prev = {"own_area": False, "opp_area": False}
    ball_in_own = ball_in_opp = 0
    for e in ticks:
        own = [r for r in e["robots"] if r["own"] and r["pos"][0] is not None]
        opp = [r for r in e["robots"] if not r["own"] and r["pos"][0] is not None]
        if not own or not opp:
            continue
        keeper = next((r for r in own if r["skill"] == "goalkeep"), None) or min(own, key=lambda r: dist(r["pos"], own_goal))
        opp_keeper = min(opp, key=lambda r: dist(r["pos"], opp_goal))
        own_field_in = [r for r in own if r is not keeper and in_area(r["pos"], own_side)]
        opp_field_in = [r for r in opp if r is not opp_keeper and in_area(r["pos"], opp_side)]
        own_attackers_in = [r for r in own if in_area(r["pos"], opp_side)]
        opp_attackers_in = [r for r in opp if in_area(r["pos"], own_side)]
        for key, cond in (("own_area", bool(own_field_in)), ("opp_area", bool(opp_field_in))):
            if cond:
                stats[key + "_ticks"] += 1
                if not prev[key]:
                    stats[key + "_episodes"] += 1
            prev[key] = cond
        if len(own_attackers_in) >= 2:
            stats["own_double_ticks"] += 1
        if len(opp_attackers_in) >= 2:
            stats["opp_double_ticks"] += 1
        bx, by = e["ball"][0], e["ball"][1]
        ball_in_own = ball_in_own + 1 if in_area((bx, by), own_side) else 0
        ball_in_opp = ball_in_opp + 1 if in_area((bx, by), opp_side) else 0
        if ball_in_own == RETENTION_TICKS:
            stats["own_retention_episodes"] += 1
        if ball_in_opp == RETENTION_TICKS:
            stats["opp_retention_episodes"] += 1
    n = max(len(ticks), 1)
    return {
        "faltas_area_propia_episodios": stats["own_area_episodes"],
        "faltas_area_propia_pct": round(100 * stats["own_area_ticks"] / n, 2),
        "doble_atacante_area_rival_pct": round(100 * stats["own_double_ticks"] / n, 2),
        "retencion_10s_episodios": stats["own_retention_episodes"],
        "rival_faltas_area_propia_episodios": stats["opp_area_episodes"],
        "rival_doble_atacante_pct": round(100 * stats["opp_double_ticks"] / n, 2),
        "rival_retencion_10s_episodios": stats["opp_retention_episodes"],
    }


def print_table(results: list[dict]) -> None:
    keys = [
        "equipo", "duracion_s", "goles_favor", "goles_contra", "posesion_propia_pct",
        "posesion_rival_pct", "pelota_libre_pct", "toques_propios", "toques_rival",
        "dist_media_al_balon_m", "pelota_en_campo_propio_pct", "pelota_en_pared_pct",
        "cambios_de_striker_por_min",
        "faltas_area_propia_episodios", "faltas_area_propia_pct", "doble_atacante_area_rival_pct",
        "retencion_10s_episodios", "rival_faltas_area_propia_episodios", "rival_doble_atacante_pct",
        "rival_retencion_10s_episodios",
    ]
    width = max(len(k) for k in keys) + 2
    header = " " * width + "".join(f"{r['archivo'][-28:]:>30}" for r in results)
    print(header)
    for k in keys:
        print(f"{k:<{width}}" + "".join(f"{str(r.get(k, '-')):>30}" for r in results))
    for r in results:
        print(f"\n{r['archivo']}")
        print("  uso de skills (% de ticks propios):", r.get("uso_skills_pct"))
        print("  toques por robot propio:", r.get("toques_por_robot_propio"))
        print("  atascado (% de ticks con comando y sin movimiento):", r.get("atascado_pct_por_robot"))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+", help="CSV(s) de VSSL_MATCH_LOG")
    ap.add_argument("--json", action="store_true", help="imprimir JSON en vez de tabla")
    args = ap.parse_args(argv)
    results = [analyze(p) for p in args.csv]
    if args.json:
        json.dump(results, sys.stdout, indent=2, ensure_ascii=False)
        print()
    else:
        print_table(results)
    return 0


if __name__ == "__main__":
    sys.exit(main())
