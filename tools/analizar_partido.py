#!/usr/bin/env python3
"""Qué pasó en un partido, desde el registro de un equipo (`VSSL_MATCH_LOG`), del simulador o de
la salita, para comparar la cancha real con el simulador con los mismos números.

    python3 tools/analizar_partido.py logs/salita/2026-10-31/p1_blue.csv [más.csv ...] [--lado +1|-1]

Todo se mide en juego abierto (`play = open`) y en el marco de ataque del equipo del registro
(x positivo hacia el arco rival). El lado sale del arquero (skill `railkeeper`/`goalkeep`: vive en
nuestro arco); con `--lado` se fuerza.

- **Trabas:** pelota quieta (a < 5 cm de un punto) ≥ 4 s: zona, duelo (propio y rival a < 12 cm),
  dos propios encima, skill del propio más cercano.
- **Esquina y fondo de ataque:** episodios ≥ 1 s con la pelota en |x| > 0.60 y |y| > 0.20 del
  lado rival: dónde está la pelota 2 s después, gol en 5 s, distancia del apoyo.
- **Reanudaciones:** al volver a juego abierto tras un free ball o un saque, quién toca primero y
  en cuánto, la pelota a los 3 s y gol en 10 s.
- **Goles:** a favor y en contra: rapidez al entrar, de dónde venía 2 s antes, desfase del
  arquero propio 1 s antes y skill del jugador de campo más cercano.

Solo la biblioteca estándar.
"""
from __future__ import annotations

import argparse
import csv
import glob
import math
import statistics
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass, field

GOAL_HALF_Y = 0.20
GOAL_LINE_X = 0.76
TOUCH = 0.075
NEAR = 0.12
HZ = 60.0


@dataclass
class Frame:
    t: float
    play: str
    ball: tuple[float, float, float, float]
    own: list[dict] = field(default_factory=list)
    riv: list[tuple[float, float]] = field(default_factory=list)


def load(path: str) -> list[Frame]:
    by_tick: dict[int, Frame] = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            if not r.get("pose_x") or r.get("ball_vy") in (None, ""):
                continue
            try:
                k = int(r["tick"])
                fr = by_tick.get(k)
                if fr is None:
                    fr = by_tick[k] = Frame(
                        float(r["t_ms"]) / 1000.0, "",
                        (float(r["ball_x"]), float(r["ball_y"]), float(r["ball_vx"]), float(r["ball_vy"])),
                    )
                pos = (float(r["pose_x"]), float(r["pose_y"]))
                if r.get("own") == "1":
                    fr.play = fr.play or r.get("play") or "open"
                    fr.own.append({"id": r["robot"], "pos": pos, "skill": r.get("skill") or "", "role": r.get("role") or ""})
                else:
                    fr.riv.append(pos)
            except (ValueError, KeyError):
                continue  # fila truncada (registro en vivo)
    return [by_tick[k] for k in sorted(by_tick)]


def attack_sign(frames: list[Frame], override: float | None = None) -> float:
    """+1 si atacamos hacia +x. El arquero propio vive en nuestro arco."""
    if override:
        return 1.0 if override > 0 else -1.0
    xs = [o["pos"][0] for fr in frames for o in fr.own if o["skill"] in ("railkeeper", "goalkeep")]
    if not xs:
        xs = [o["pos"][0] for fr in frames for o in fr.own if o["id"] == "2"]
    if xs and abs(statistics.fmean(xs)) > 0.3:
        return -1.0 if statistics.fmean(xs) > 0 else 1.0
    return 1.0


def zona(x: float, y: float) -> str:
    """Zona en el marco de ataque (x hacia el arco rival)."""
    end = "ataque" if x > 0 else "defensa"
    ax, ay = abs(x), abs(y)
    if ax > 0.60 and ay > 0.45:
        return f"esquina {end}"
    if ax > 0.60 and ay > GOAL_HALF_Y:
        return f"fondo {end}"
    if ax > 0.60:
        return f"boca {end}"
    if ay > 0.50:
        return f"banda {end}"
    return f"abierto {end}"


def ball_att(fr: Frame, s: float) -> tuple[float, float]:
    return (fr.ball[0] * s, fr.ball[1])


def trabas(frames: list[Frame], s: float, min_s: float = 4.0) -> list[dict]:
    """Episodios de pelota quieta en juego abierto."""
    out, anchor, ep = [], None, []

    def close():
        if len(ep) / HZ < min_s:
            return
        bx, by = ball_att(ep[0], s)
        st = {"zona": zona(bx, by), "dur": len(ep) / HZ, "duelo": 0.0, "dos_propios": 0.0, "skill": Counter()}
        sample = ep[::15]
        for fr in sample:
            b = fr.ball[:2]
            near = sorted((math.dist(o["pos"], b), o["skill"]) for o in fr.own)
            rd = min((math.dist(p, b) for p in fr.riv), default=9.0)
            if near:
                st["skill"][near[0][1]] += 1
                st["duelo"] += near[0][0] < NEAR and rd < NEAR
                st["dos_propios"] += sum(1 for d, _ in near if d < 0.15) >= 2
        n = max(1, len(sample))
        st["duelo"] /= n
        st["dos_propios"] /= n
        out.append(st)

    for fr in frames:
        if fr.play != "open" or not fr.own:
            close(); anchor, ep = None, []
            continue
        b = fr.ball[:2]
        if anchor is None or math.dist(anchor, b) > 0.05:
            close(); anchor, ep = b, []
        ep.append(fr)
    close()
    return out


def esquinas(frames: list[Frame], s: float) -> list[dict]:
    """Episodios ≥ 1 s con la pelota en la esquina o el fondo de ataque."""
    out = []
    inz = lambda fr: fr.play == "open" and fr.own and ball_att(fr, s)[0] > 0.60 and abs(fr.ball[1]) > GOAL_HALF_Y
    i, n = 0, len(frames)
    while i < n:
        if not inz(frames[i]):
            i += 1
            continue
        j = i
        while j < n and inz(frames[j]):
            j += 1
        if (j - i) / HZ >= 1.0:
            sup = [math.dist(o["pos"], fr.ball[:2]) for fr in frames[i:j:15] for o in fr.own if o["role"] == "support"]
            after = frames[j:j + 120]
            if j < n and frames[j].play != "open":
                fin = "free ball / parada"
            elif not after:
                fin = "fin del registro"
            else:
                bx, by = ball_att(after[-1], s)
                fin = ("gol" if bx > GOAL_LINE_X and abs(by) < GOAL_HALF_Y else
                       "al frente del arco" if bx > 0.35 and abs(by) < 0.30 else
                       "vuelve al fondo/esquina" if bx > 0.60 else
                       "mitad rival, lejos del arco" if bx > 0.0 else "a nuestra mitad")
            gol5 = any(ball_att(fr, s)[0] > GOAL_LINE_X and abs(fr.ball[1]) < GOAL_HALF_Y for fr in frames[j:j + 300])
            out.append({"dur": (j - i) / HZ, "fin": fin, "gol5": gol5,
                        "apoyo": statistics.median(sup) if sup else float("nan")})
        i = j
    return out


def reanudaciones(frames: list[Frame], s: float) -> list[dict]:
    """Primer toque tras un free ball o un saque."""
    out = []
    for i in range(1, len(frames)):
        prev, cur = frames[i - 1].play, frames[i].play
        if cur != "open" or not (prev.startswith("free_ball") or prev.startswith("kickoff")):
            continue
        tipo = "free ball" if prev.startswith("free_ball") else prev
        quien, t = "nadie", float("nan")
        for j in range(i, min(i + 180, len(frames))):
            b = frames[j].ball[:2]
            if any(math.dist(o["pos"], b) < TOUCH for o in frames[j].own):
                quien, t = "propio", (j - i) / HZ
                break
            if any(math.dist(p, b) < TOUCH for p in frames[j].riv):
                quien, t = "rival", (j - i) / HZ
                break
        k = min(i + 180, len(frames) - 1)
        dx = ball_att(frames[k], s)[0] - ball_att(frames[i], s)[0] if all(f.play == "open" for f in frames[i:k + 1]) else float("nan")
        gol = ""
        for fr in frames[i:i + 600]:
            bx, by = ball_att(fr, s)
            if abs(by) < GOAL_HALF_Y and bx > GOAL_LINE_X:
                gol = "a favor"
                break
            if abs(by) < GOAL_HALF_Y and bx < -GOAL_LINE_X:
                gol = "en contra"
                break
        out.append({"tipo": tipo, "quien": quien, "t": t, "dx": dx, "gol": gol})
    return out


def goles(frames: list[Frame], s: float) -> list[dict]:
    """Pelota entera dentro de un arco en juego abierto (una vez por gol)."""
    out, dentro = [], False
    for i, fr in enumerate(frames):
        bx, by = ball_att(fr, s)
        if abs(bx) < 0.5:
            dentro = False
        if dentro or fr.play != "open" or not (abs(bx) > GOAL_LINE_X and abs(by) < GOAL_HALF_Y):
            continue
        dentro = True
        antes2 = frames[max(0, i - 120)]
        antes1 = frames[max(0, i - 60)]
        ox, oy = ball_att(antes2, s)
        v = math.hypot(fr.ball[2], fr.ball[3])
        keeper = next((o for o in antes1.own if o["skill"] in ("railkeeper", "goalkeep")), None)
        campo = [o for o in antes1.own if o is not keeper]
        cerca = min(campo, key=lambda o: math.dist(o["pos"], antes1.ball[:2]), default=None)
        out.append({
            "lado": "a favor" if bx > 0 else "en contra",
            "v": v,
            "tipo": "conducido" if v < 0.30 else "empuje/rebote",
            "origen": "área/borde" if abs(ox) > 0.55 else "tercio" if abs(ox) > 0.25 else "medio/lejos",
            "dy_arquero": abs(keeper["pos"][1] - antes1.ball[1]) if keeper and bx < 0 else float("nan"),
            "skill_campo": cerca["skill"] if cerca and bx < 0 else "",
        })
    return out


def fmt_counter(c: Counter, n: int) -> str:
    return ", ".join(f"{k} {100 * v / n:.0f} %" for k, v in c.most_common(4)) if n else "—"


def informe(paths: list[str], lado: float | None = None) -> str:
    T, E, R, G, dur_total = [], [], [], [], 0.0
    for p in paths:
        frames = load(p)
        if not frames:
            continue
        s = attack_sign(frames, lado)
        dur_total += sum(1 for fr in frames if fr.play == "open") / HZ
        T += trabas(frames, s)
        E += esquinas(frames, s)
        R += reanudaciones(frames, s)
        G += goles(frames, s)
    lines = [f"Registros: {len(paths)}; juego abierto {dur_total:.0f} s"]
    if T:
        tot = sum(t["dur"] for t in T)
        lines.append(f"Trabas (≥ 4 s): {len(T)} episodios, {tot:.0f} s ({100 * tot / max(dur_total, 1):.0f} % del juego abierto)")
        por_zona = defaultdict(list)
        for t in T:
            por_zona[t["zona"]].append(t)
        for z, v in sorted(por_zona.items(), key=lambda kv: -sum(t["dur"] for t in kv[1])):
            sk = Counter()
            for t in v:
                sk.update(t["skill"])
            lines.append(f"  {z:16s} {len(v):3d} ep. {sum(t['dur'] for t in v):5.0f} s | duelo {statistics.fmean(t['duelo'] for t in v):.0%} "
                         f"| 2 propios encima {statistics.fmean(t['dos_propios'] for t in v):.0%} | propio más cercano: {fmt_counter(sk, sum(sk.values()))}")
    else:
        lines.append("Trabas (≥ 4 s): ninguna")
    if E:
        fines = Counter(e["fin"] for e in E)
        ap = [e["apoyo"] for e in E if not math.isnan(e["apoyo"])]
        lines.append(f"Esquina/fondo de ataque (≥ 1 s): {len(E)} episodios, {sum(e['dur'] for e in E):.0f} s; gol en 5 s: {sum(e['gol5'] for e in E)}")
        lines.append("  pelota 2 s después: " + ", ".join(f"{k} {v} ({100 * v / len(E):.0f} %)" for k, v in fines.most_common()))
        if ap:
            lines.append(f"  apoyo a la pelota (mediana): {statistics.median(ap):.2f} m")
    if R:
        por_tipo = defaultdict(list)
        for r in R:
            por_tipo[r["tipo"]].append(r)
        for tipo, v in sorted(por_tipo.items()):
            ts = [r["t"] for r in v if not math.isnan(r["t"])]
            dxs = [r["dx"] for r in v if not math.isnan(r["dx"])]
            lines.append(f"{tipo}: {len(v)}; primer toque propio {sum(r['quien'] == 'propio' for r in v)} "
                         f"({100 * sum(r['quien'] == 'propio' for r in v) / len(v):.0f} %), rival {sum(r['quien'] == 'rival' for r in v)}"
                         f"; en {statistics.median(ts) if ts else float('nan'):.2f} s; pelota a los 3 s {statistics.fmean(dxs) if dxs else float('nan'):+.2f} m"
                         f"; gol en 10 s a favor {sum(r['gol'] == 'a favor' for r in v)}, en contra {sum(r['gol'] == 'en contra' for r in v)}")
    for lado_g in ("a favor", "en contra"):
        gs = [g for g in G if g["lado"] == lado_g]
        if not gs:
            lines.append(f"Goles {lado_g}: 0")
            continue
        lines.append(f"Goles {lado_g}: {len(gs)}; tipo {dict(Counter(g['tipo'] for g in gs))}; origen 2 s antes {dict(Counter(g['origen'] for g in gs))}"
                     f"; rapidez al entrar mediana {statistics.median(g['v'] for g in gs):.2f} m/s")
        if lado_g == "en contra":
            dys = [g["dy_arquero"] for g in gs if not math.isnan(g["dy_arquero"])]
            if dys:
                lines.append(f"  arquero 1 s antes: desfase en y mediana {statistics.median(dys):.2f} m; "
                             f"jugador de campo más cercano: {dict(Counter(g['skill_campo'] for g in gs))}")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="qué pasó en un partido, desde el registro de un equipo")
    ap.add_argument("csv", nargs="+", help="registros VSSL_MATCH_LOG (acepta comodines)")
    ap.add_argument("--lado", type=float, default=None, help="+1 si atacamos hacia +x, -1 si hacia -x (si no, se deduce del arquero)")
    args = ap.parse_args(argv)
    paths = [p for pat in args.csv for p in (sorted(glob.glob(pat)) or [pat])]
    print(informe(paths, args.lado))
    return 0


if __name__ == "__main__":
    sys.exit(main())
