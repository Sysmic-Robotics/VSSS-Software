#!/usr/bin/env python3
"""Informe detallado de un partido, desde el registro de un equipo (`VSSL_MATCH_LOG`, `.csv` o
`.csv.gz`, 60 Hz, una fila por robot y tick): el mismo análisis que se hizo a mano con los
partidos del 5-nov, para repetirlo después de cada partido.

    python3 tools/analizar_detalle.py logs/partidos/partido_5nov_tarde/1628_yellow.csv.gz
    python3 tools/analizar_detalle.py logs/partidos/partido_5nov/*.csv.gz --solo goles,caras

Todo se mide en el marco "atacamos hacia +x" (nuestro arco en x = -0.75). El lado sale del
arquero propio (rol `keeper`, si no la skill `railkeeper`/`goalkeep`): si vive en +x, se gira la
cancha en 180° (x, y, velocidades, comandos y objetivos cambian de signo; el heading suma π).
Con `--lado` se fuerza.

Juego abierto es solo `play = open`. En la cancha real el árbitro va a mano y `play` queda vacío o
en `hold` largos ratos; por eso los goles se detectan por la pelota cruzando la línea (una vez por
gol: se rearma cuando la pelota vuelve a |x| < 0.40; se descartan los cruces precedidos por un
salto de más de 15 cm entre cuadros, que es la visión o la pelota puesta a mano), no por el
árbitro. La orden siguiente del árbitro solo se muestra: un gol sin saque después (`→ —`) es
dudoso (la pelota pudo rebotar adentro y salir sin que el árbitro lo cobrara).

Secciones (todas por defecto; `--solo` elige algunas):

- **goles:** a favor y en contra, una línea por gol: jugada → orden siguiente del árbitro,
  portador (rectitud, giro y zigzag de su camino y del de la pelota en los últimos 3 s), y en los
  en contra el arquero 1 s antes y al gol, los nuestros rodeados, el otro rival empujando y dónde
  estaban los jugadores de campo.
- **caras:** inversiones del avance pedido por minuto y % de ida y vuelta en < 0.5 s, giro por
  minuto, cambios de cara por skill y las llegadas a la pelota (demora, giro, si llega alineada).
- **soporte:** skills y posición, salvadas y rescates por rol, rival pegado a uno de los nuestros
  y las ocasiones de gol que tuvo el soporte.
- **ataque:** pelota en la mitad y el tercio rival, tiros al arco, skills del atacante y los
  episodios con la pelota junto al arco rival.
- **saques:** cada saque inicial (el `open` que sigue a `kickoff_*`, aunque pase por `hold`): la
  pelota al empezar y a los 3 s, y gol en 10 s.
- **casi_gol:** pelota que muere a < 0.15 m de la línea rival frente al arco sin gol: quién estaba
  más cerca y si el soporte estaba en el área rival.

Solo la biblioteca estándar.
"""
from __future__ import annotations

import argparse
import csv
import glob
import gzip
import math
import os
import statistics
import sys
from collections import Counter

HZ = 60.0
GL = 0.75           # línea de gol (|x|)
GH = 0.20           # medio ancho del arco
AREA_X = 0.60       # área rival: x ≥ 0.60 ...
AREA_HY = 0.35      # ... y |y| ≤ 0.35
CONTACTO = 0.085    # centro del robot a la pelota: tocándola
PEGADO = 0.095      # centro a centro entre robots: en contacto
PORTA = 0.12        # el robot más cercano a menos de esto lleva la pelota
SALTO = 0.15        # salto de la pelota entre cuadros que no puede ser juego (visión o mano)
SALTO_CRUCE = 0.30  # en el cuadro del cruce se acepta hasta esto: pelota tapada que reaparece adentro
REARME_X = 0.40     # tras un gol, se cuenta otro cuando la pelota vuelve a |x| < 0.40
ORDENES = ("kickoff_ours", "kickoff_theirs", "penalty_ours", "penalty_theirs")
SECCIONES = ("goles", "caras", "soporte", "ataque", "saques", "casi_gol")


class Propio:
    __slots__ = ("x", "y", "th", "vx", "vy", "cvx", "cvy", "tx", "ty", "skill", "role", "face", "guard", "kmode")

    def __init__(self, x, y, th, vx, vy, cvx, cvy, tx, ty, skill, role, face, guard, kmode):
        self.x, self.y, self.th, self.vx, self.vy = x, y, th, vx, vy
        self.cvx, self.cvy, self.tx, self.ty = cvx, cvy, tx, ty
        self.skill, self.role, self.face, self.guard, self.kmode = skill, role, face, guard, kmode


class Rival:
    __slots__ = ("x", "y", "th", "vx", "vy")

    def __init__(self, x, y, th, vx, vy):
        self.x, self.y, self.th, self.vx, self.vy = x, y, th, vx, vy


class Cuadro:
    """Un tick: tiempo, orden del árbitro, pelota (x, y, vx, vy) y robots por id."""
    __slots__ = ("t", "play", "b", "own", "riv")

    def __init__(self, t, b):
        self.t, self.b, self.play = t, b, ""
        self.own: dict[int, Propio] = {}
        self.riv: dict[int, Rival] = {}


class Partido:
    def __init__(self, path: str, cuadros: list[Cuadro], s: float):
        nombre = os.path.basename(path)
        for ext in (".gz", ".csv"):
            nombre = nombre[:-len(ext)] if nombre.endswith(ext) else nombre
        self.path, self.nombre, self.cuadros, self.s = path, nombre, cuadros, s
        self.abierto_s = sum(1 for f in cuadros if f.play == "open") / HZ
        self.ids = sorted({rid for f in cuadros for rid in f.own})
        self.goles = detectar_goles(cuadros)

    def goles_en(self, lado: str, i0: int, i1: int) -> bool:
        return any(g["lado"] == lado and i0 <= g["i"] <= i1 for g in self.goles)


# ---------------------------------------------------------------- lectura y marco de ataque

def _abrir(path: str):
    with open(path, "rb") as f:
        gz = f.read(2) == b"\x1f\x8b"
    return gzip.open(path, "rt", newline="") if gz else open(path, newline="")


def _num(s: str, defecto=None):
    try:
        return float(s)
    except ValueError:
        return defecto


def _ang(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def lado_ataque(cuadros: list[Cuadro], forzado: float | None = None) -> float:
    """+1 si atacamos hacia +x en el registro. El arquero propio vive en nuestro arco."""
    if forzado:
        return 1.0 if forzado > 0 else -1.0
    xs = sorted(o.x for f in cuadros for o in f.own.values() if o.role == "keeper")
    if not xs:
        xs = sorted(o.x for f in cuadros for o in f.own.values() if o.skill in ("railkeeper", "goalkeep"))
    if not xs:
        return 1.0
    return -1.0 if xs[len(xs) // 2] > 0 else 1.0


def _girar(cuadros: list[Cuadro]) -> None:
    """Gira la cancha en 180°: el arco del arquero propio queda en -x."""
    for f in cuadros:
        bx, by, bvx, bvy = f.b
        f.b = (-bx, -by, -bvx, -bvy)
        for r in list(f.own.values()) + list(f.riv.values()):
            r.x, r.y, r.vx, r.vy = -r.x, -r.y, -r.vx, -r.vy
            r.th = _ang(r.th + math.pi)
        for r in f.own.values():
            if r.cvx is not None:
                r.cvx = -r.cvx
            if r.cvy is not None:
                r.cvy = -r.cvy
            if r.tx is not None:
                r.tx = -r.tx
            if r.ty is not None:
                r.ty = -r.ty


def cargar(path: str, lado: float | None = None) -> Partido:
    """Lee el registro (texto o gzip) y lo deja en el marco de ataque. Las filas truncadas (registro
    en vivo) o sin pose se saltan."""
    cuadros: dict[int, Cuadro] = {}
    textos: dict[str, str] = {}
    with _abrir(path) as f:
        rd = csv.reader(f)
        h = next(rd, [])
        nh = len(h)
        ix = {k: i for i, k in enumerate(h)}
        if "tick" not in ix or "pose_x" not in ix:
            return Partido(path, [], 1.0)
        c = lambda k: ix.get(k, nh)  # columna que falta: la celda vacía del relleno
        (i_t, i_tick, i_own, i_rob, i_px, i_py, i_th, i_vx, i_vy, i_cvx, i_cvy, i_bx, i_by, i_bvx, i_bvy,
         i_tx, i_ty, i_sk, i_role, i_face, i_guard, i_km, i_play) = map(c, (
            "t_ms", "tick", "own", "robot", "pose_x", "pose_y", "pose_theta", "vel_x", "vel_y", "cmd_vx", "cmd_vy",
            "ball_x", "ball_y", "ball_vx", "ball_vy", "target_x", "target_y", "skill", "role", "face", "guard",
            "keeper_mode", "play"))
        for r in rd:
            if len(r) <= nh:
                r.extend([""] * (nh + 1 - len(r)))
            try:
                k = int(r[i_tick])
                x, y = float(r[i_px]), float(r[i_py])
                rid = int(r[i_rob])
            except ValueError:
                continue
            fr = cuadros.get(k)
            if fr is None:
                try:
                    b = (float(r[i_bx]), float(r[i_by]), float(r[i_bvx] or 0), float(r[i_bvy] or 0))
                    t = float(r[i_t]) / 1000.0
                except ValueError:
                    continue
                fr = cuadros[k] = Cuadro(t, b)
            th, vx, vy = _num(r[i_th], 0.0), _num(r[i_vx], 0.0), _num(r[i_vy], 0.0)
            if r[i_own] == "1":
                if not fr.play:
                    fr.play = textos.setdefault(r[i_play], r[i_play])
                txt = textos.setdefault  # una sola copia de cada texto repetido (skill, rol, cara...)
                fr.own[rid] = Propio(x, y, th, vx, vy, _num(r[i_cvx]), _num(r[i_cvy]), _num(r[i_tx]), _num(r[i_ty]),
                                     txt(r[i_sk], r[i_sk]), txt(r[i_role], r[i_role]), txt(r[i_face], r[i_face]),
                                     txt(r[i_guard], r[i_guard]), txt(r[i_km], r[i_km]))
            else:
                fr.riv[rid] = Rival(x, y, th, vx, vy)
    lista = [cuadros[k] for k in sorted(cuadros)]
    s = lado_ataque(lista, lado)
    if s < 0:
        _girar(lista)
    return Partido(path, lista, s)


# ---------------------------------------------------------------- geometría

def _dp(r, p) -> float:
    """Robot a punto (x, y, ...)."""
    return math.hypot(r.x - p[0], r.y - p[1])


def _dr(a, b) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def _seg(px, py, ax, ay, bx, by) -> float:
    dx, dy = bx - ax, by - ay
    L = dx * dx + dy * dy
    t = 0.0 if L < 1e-12 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def arco_abierto(bx: float, by: float, tapan) -> float:
    """Fracción de 9 líneas de la pelota a la boca del arco rival que ningún robot tapa (a > 6 cm)."""
    libres = 0
    for i in range(9):
        y = -0.18 + 0.36 * i / 8
        if all(_seg(o.x, o.y, bx, by, GL, y) > 0.06 for o in tapan):
            libres += 1
    return libres / 9


def alineacion(r, b) -> float:
    """Coseno entre robot→pelota y pelota→arco rival (1: detrás de la pelota mirando al arco)."""
    tb = (b[0] - r.x, b[1] - r.y)
    tg = (GL - b[0], -b[1])
    return (tb[0] * tg[0] + tb[1] * tg[1]) / ((math.hypot(*tb) or 1) * (math.hypot(*tg) or 1))


def camino(pts: list[tuple[float, float]]) -> dict | None:
    """Puntos cada 0.1 s: rectitud (desplazamiento / largo), giro total y neto, y zigzag (cambios
    de sentido entre giros de más de 0.15 rad)."""
    if len(pts) < 3:
        return None
    L = sum(math.dist(pts[j], pts[j + 1]) for j in range(len(pts) - 1))
    angs = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(pts, pts[1:]) if math.dist(a, b) > 0.01]
    giro, signos = 0.0, []
    for a0, a1 in zip(angs, angs[1:]):
        d = _ang(a1 - a0)
        giro += abs(d)
        if abs(d) > 0.15:
            signos.append(1 if d > 0 else -1)
    return {"rect": math.dist(pts[0], pts[-1]) / L if L > 1e-6 else 1.0, "giro": math.degrees(giro),
            "neto": math.degrees(_ang(angs[-1] - angs[0])) if len(angs) >= 2 else 0.0,
            "zigzag": sum(1 for a, b in zip(signos, signos[1:]) if a != b)}


# ---------------------------------------------------------------- goles

def siguiente_orden(cuadros: list[Cuadro], i: int, ventana_s: float = 90.0) -> str:
    """Primera orden de reanudación del árbitro (saque, penal o free ball) que empieza después de i."""
    for j in range(i + 1, min(len(cuadros), i + int(ventana_s * HZ))):
        p = cuadros[j].play
        if p != cuadros[j - 1].play and (p in ORDENES or p.startswith("free_ball")):
            return p
    return ""


def detectar_goles(cuadros: list[Cuadro]) -> list[dict]:
    """La pelota entera cruza la línea dentro del arco, en cualquier jugada, y se cuenta en el
    cuadro del cruce. Una vez por gol: se rearma cuando la pelota vuelve a |x| < 0.40. Se descarta
    el cruce si en el segundo anterior la pelota no estuvo en la cancha (|x| < 0.70) o saltó más de
    15 cm entre dos cuadros (puesta a mano o visión), o si el cruce mismo es un salto de más de
    30 cm. Un salto menor en el cruce sí vale: la visión pierde la pelota tapada por el robot que la
    empuja y la reencuentra ya adentro."""
    out, armado, adentro_antes = [], True, True
    for i, f in enumerate(cuadros):
        bx, by = f.b[0], f.b[1]
        adentro = abs(by) < GH + 0.01 and abs(bx) > GL + 0.01
        cruce, adentro_antes = adentro and not adentro_antes, adentro
        if not armado:
            armado = abs(bx) < REARME_X
            continue
        if not cruce:
            continue
        antes = cuadros[max(0, i - int(HZ)):i]
        if not any(abs(p.b[0]) < 0.70 for p in antes):
            continue
        if any(math.hypot(a.b[0] - c.b[0], a.b[1] - c.b[1]) > SALTO for a, c in zip(antes, antes[1:])):
            continue
        if math.hypot(bx - antes[-1].b[0], by - antes[-1].b[1]) > SALTO_CRUCE:
            continue
        out.append({"i": i, "t": f.t, "lado": "favor" if bx > 0 else "contra", "play": f.play,
                    "siguiente": siguiente_orden(cuadros, i)})
        armado = False
    return out


def analizar_gol(p: Partido, g: dict) -> dict:
    """Los 4 s antes de un gol: portador y caminos; en los en contra, arquero, rodeados y empujones."""
    fr, i = p.cuadros, g["i"]
    w = fr[max(0, i - int(4 * HZ)):i + 1]
    ult3 = w[-int(3 * HZ):]
    favor = g["lado"] == "favor"
    equipo = (lambda f: f.own) if favor else (lambda f: f.riv)
    # portador: el robot del equipo que anota que más veces está pegado a la pelota en los últimos 3 s
    cnt = Counter()
    for f in ult3:
        robs = equipo(f)
        if robs:
            rid, r = min(robs.items(), key=lambda kv: _dp(kv[1], f.b))
            if _dp(r, f.b) < PORTA:
                cnt[rid] += 1
    port = cnt.most_common(1)[0][0] if cnt else None
    muestra = ult3[::6]
    info = {"t": g["t"], "play": g["play"], "sig": g["siguiente"], "portador": port,
            "contacto_s": cnt[port] / HZ if port is not None else 0.0,
            "pelota": camino([(f.b[0], f.b[1]) for f in muestra]),
            "robot": camino([(equipo(f)[port].x, equipo(f)[port].y) for f in muestra if port in equipo(f)]),
            "origen": (w[0].b[0], w[0].b[1])}
    f_gol = fr[i]
    f_1 = w[-int(HZ)] if len(w) > HZ else w[0]
    if favor:
        r = f_1.own.get(port) if port is not None else None
        info["anota"] = (r.role, r.skill) if r else None
        return info
    k = next((r for r in f_1.own.values() if r.role == "keeper"), None)
    kg = next((r for r in f_gol.own.values() if r.role == "keeper"), None)
    info["arquero"] = None if k is None else {
        "y": k.y, "pelota_y": f_1.b[1], "dy": abs(k.y - f_1.b[1]), "skill": k.skill, "modo": k.kmode, "guard": k.guard,
        "y_gol": kg.y if kg else None, "pelota_y_gol": f_gol.b[1]}
    # rodeados: un propio de campo entre el portador y nuestro arco que al gol quedó atrás o lejos
    rodeados = []
    for rid in p.ids:
        entre = False
        for f in ult3[:-int(0.5 * HZ)]:
            c, o = f.riv.get(port), f.own.get(rid)
            if c is None or o is None or o.role == "keeper":
                continue
            if o.x < c.x and _seg(o.x, o.y, c.x, c.y, -GL, 0.0) < 0.10:
                entre = True
                break
        o = f_gol.own.get(rid)
        if entre and o is not None and (o.x > f_gol.b[0] - 0.02 or _dp(o, f_gol.b) > 0.20):
            rodeados.append(o.role or str(rid))
    info["rodeados"] = rodeados
    # el otro rival pegado a uno de los nuestros (empujando o tapando)
    emp = sum(1 for f in ult3 if any(rid != port and any(_dr(r, o) < PEGADO for o in f.own.values())
                                     for rid, r in f.riv.items()))
    info["empuja_pct"] = 100 * emp / max(len(ult3), 1)
    info["campo"] = [(r.role, r.skill, _dp(r, f_1.b)) for r in f_1.own.values() if r.role != "keeper"]
    return info


def _f(v, fmt="+.2f", nada="—") -> str:
    return nada if v is None else format(v, fmt)


def _orden(s: str) -> str:
    return s or "—"


def seccion_goles(p: Partido) -> list[str]:
    lines = []
    for lado, titulo in (("favor", "a favor"), ("contra", "en contra")):
        gs = [analizar_gol(p, g) for g in p.goles if g["lado"] == lado]
        pen = sum(1 for g in gs if g["play"].startswith("penalty"))
        lines.append(f"Goles {titulo}: {len(gs)}" + (f" ({pen} de penal)" if pen else ""))
        for g in gs:
            pe, rb = g["pelota"] or {}, g["robot"] or {}
            cab = f"  t={g['t']:7.1f} s [{_orden(g['play'])} → {_orden(g['sig'])}]"
            if lado == "favor":
                a = g["anota"]
                quien = f"{a[0] or '—'} ({a[1] or '—'})" if a else "—"
                lines.append(f"{cab} anota {quien}, contacto {g['contacto_s']:.1f} s; pelota: rectitud {pe.get('rect', 0):.2f}, "
                             f"giro {pe.get('giro', 0):.0f}°; origen ({g['origen'][0]:+.2f}, {g['origen'][1]:+.2f})")
                continue
            port = "—" if g["portador"] is None else f"rival {g['portador']}"
            lines.append(f"{cab} portador {port}, contacto {g['contacto_s']:.1f} s; su camino: rectitud {rb.get('rect', 0):.2f}, "
                         f"giro {rb.get('giro', 0):.0f}° (neto {rb.get('neto', 0):+.0f}°), zigzag {rb.get('zigzag', 0)}; "
                         f"pelota: rectitud {pe.get('rect', 0):.2f}; origen ({g['origen'][0]:+.2f}, {g['origen'][1]:+.2f})")
            a = g["arquero"]
            arq = ("sin arquero" if a is None else
                   f"arquero 1 s antes y {a['y']:+.2f} (pelota {a['pelota_y']:+.2f}, desfase {a['dy']:.2f}) → al gol y "
                   f"{_f(a['y_gol'])} (pelota {a['pelota_y_gol']:+.2f}); {a['skill'] or '—'}/{a['modo'] or '—'}, guardia {a['guard'] or '—'}")
            campo = ", ".join(f"{r or '—'} {sk or '—'} a {d:.2f} m" for r, sk, d in sorted(g["campo"])) or "—"
            lines.append(f"      {arq}; rodeados: {', '.join(g['rodeados']) or '—'}; otro rival empujando {g['empuja_pct']:.0f} %; "
                         f"campo: {campo}")
        if lado == "contra" and gs:
            zz = [g["robot"]["zigzag"] for g in gs if g["robot"]]
            dys = [g["arquero"]["dy"] for g in gs if g["arquero"]]
            lines.append(f"  resumen: rodeando a alguno de los nuestros {sum(1 for g in gs if g['rodeados'])} de {len(gs)}; "
                         f"zigzag del portador mediana {_f(statistics.median(zz) if zz else None, '.0f')}; "
                         f"desfase del arquero 1 s antes mediana {_f(statistics.median(dys) if dys else None, '.2f')} m; "
                         f"otro rival empujando ≥ 20 % en {sum(1 for g in gs if g['empuja_pct'] >= 20)}")
    return lines


# ---------------------------------------------------------------- caras

def llegadas(p: Partido) -> list[dict]:
    """Llegadas de los jugadores de campo a la pelota en juego abierto: desde que están a 0.30-0.40 m
    hasta tocarla (se abandona si se alejan a más de 0.45 m o tardan más de 5 s)."""
    fr, eps = p.cuadros, []
    for rid in p.ids:
        ini = None
        for i, f in enumerate(fr):
            r = f.own.get(rid)
            if f.play != "open" or r is None or r.role == "keeper":
                ini = None
                continue
            d = _dp(r, f.b)
            if ini is None:
                if 0.30 < d < 0.40:
                    ini = i
                continue
            if d > 0.45 or i - ini > 5 * HZ:
                ini = None
                continue
            if d >= CONTACTO:
                continue
            seg = fr[ini:i + 1]
            r0 = seg[0].own[rid]
            rot, cambios, cara = 0.0, 0, r0.face
            for a, b in zip(seg, seg[1:]):
                ra, rb = a.own.get(rid), b.own.get(rid)
                if ra is None or rb is None:
                    continue
                rot += abs(_ang(rb.th - ra.th))
                if rb.face and rb.face != cara:
                    cambios += 1
                    cara = rb.face
            eps.append({"t": seg[0].t, "dur": seg[-1].t - seg[0].t, "ideal": max(0.0, _dp(r0, seg[0].b) - 0.06) / 0.45,
                        "rot": math.degrees(rot), "cambios": cambios, "cara": r.face, "alin": alineacion(r, f.b),
                        "skill": r.skill, "rol": r.role})
            ini = None
    return eps


def medir_caras(p: Partido) -> dict:
    """Cuentas crudas (sumables entre registros) de la elección de cara de los jugadores de campo."""
    m = {"min": p.abierto_s / 60.0, "inv": 0, "ida": 0, "rot": 0.0, "robot_min": 0.0,
         "cambios": Counter(), "con_skill": Counter(), "cambio_ida": Counter(), "llegadas": llegadas(p)}
    fr = p.cuadros
    for rid in p.ids:
        # inversiones del avance pedido (comando a lo largo del frente, |v| > 3 cm/s)
        sentido, ult, th_prev, n = 0, None, None, 0
        for i, f in enumerate(fr):
            r = f.own.get(rid)
            if f.play != "open" or r is None or r.role == "keeper" or r.cvx is None:
                sentido, ult, th_prev = 0, None, None
                continue
            n += 1
            v = r.cvx * math.cos(r.th) + (r.cvy or 0.0) * math.sin(r.th)
            d = 1 if v > 0.03 else (-1 if v < -0.03 else 0)
            if d and sentido and d != sentido:
                m["inv"] += 1
                if ult is not None and i - ult < 30:
                    m["ida"] += 1
                ult = i
            if d:
                sentido = d
            if th_prev is not None:
                m["rot"] += abs(_ang(r.th - th_prev))
            th_prev = r.th
        m["robot_min"] += n / HZ / 60.0
        # cambios de la cara elegida, por la skill de después
        cambios, prev = [], None
        for i, f in enumerate(fr):
            r = f.own.get(rid)
            if f.play != "open" or r is None or r.role == "keeper" or not r.face:
                prev = None
                continue
            if prev is not None and r.face != prev[0]:
                cambios.append((i, prev[1], r.skill))
            prev = (r.face, r.skill)
        for k, (i, a, b) in enumerate(cambios):
            m["cambios"][b] += 1
            if a != b:
                m["con_skill"][b] += 1
            if k + 1 < len(cambios) and cambios[k + 1][0] - i < 30:
                m["cambio_ida"][b] += 1
    return m


def sumar_caras(ms: list[dict]) -> dict:
    t = {"min": 0.0, "inv": 0, "ida": 0, "rot": 0.0, "robot_min": 0.0, "cambios": Counter(), "con_skill": Counter(),
         "cambio_ida": Counter(), "llegadas": []}
    for m in ms:
        for k, v in m.items():
            t[k] += v
    return t


def _llegadas(sel: list[dict], titulo: str) -> str:
    if not sel:
        return f"{titulo}: 0"
    return (f"{titulo}: {len(sel)}, demora extra mediana {statistics.median(e['dur'] - e['ideal'] for e in sel):.2f} s, "
            f"giro mediano {statistics.median(e['rot'] for e in sel):.0f}°, > 180° {100 * sum(e['rot'] > 180 for e in sel) / len(sel):.0f} %, "
            f"alineada al arco {100 * sum(e['alin'] > 0.5 for e in sel) / len(sel):.0f} %")


def texto_caras(m: dict) -> list[str]:
    mins = max(m["min"], 1e-6)
    lines = [f"Caras (jugadores de campo, {m['min']:.1f} min de juego abierto):",
             f"  inversiones del avance pedido: {m['inv'] / mins:.0f} por minuto (sumando los de campo), "
             f"{100 * m['ida'] / max(m['inv'], 1):.0f} % ida y vuelta en < 0.5 s; giro {math.degrees(m['rot']) / max(m['robot_min'], 1e-6):.0f}°/min por robot"]
    n = sum(m["cambios"].values())
    lines.append(f"  cambios de cara: {n} ({n / mins:.0f} por minuto)")
    if n:
        lines.append(f"    {'skill (después)':<16}{'cambios':>8}{'con cambio de skill':>21}{'ida y vuelta < 0.5 s':>22}")
        for k, v in m["cambios"].most_common(8):
            lines.append(f"    {k or '—':<16}{v:>8}{100 * m['con_skill'][k] / v:>20.0f} %{100 * m['cambio_ida'][k] / v:>20.0f} %")
    eps = m["llegadas"]
    lines.append("  llegadas a la pelota (de 0.30-0.40 m a tocarla; demora extra sobre ir derecho a 0.45 m/s):")
    for sel, titulo in ((eps, "todas"), ([e for e in eps if e["cambios"] == 0], "sin cambiar de cara"),
                        ([e for e in eps if e["cambios"] >= 1], "cambia de cara ≥ 1"),
                        ([e for e in eps if e["cambios"] >= 2], "cambia ≥ 2 (duda)"),
                        ([e for e in eps if e["cara"] == "front"], "toca con el frente"),
                        ([e for e in eps if e["cara"] == "back"], "toca con la espalda")):
        lines.append("    " + _llegadas(sel, titulo))
    return lines


# ---------------------------------------------------------------- soporte

def ocasiones_soporte(p: Partido) -> list[dict]:
    """Momentos (≥ 0.1 s) en que un jugador de campo que no es el atacante tiene un gol fácil:
    pelota a < 0.15 m en el tercio de ataque, detrás de ella hacia el arco y el arco abierto ≥ 50 %."""
    fr, eps, cur = p.cuadros, [], {}

    def cerrar(rid):
        e = cur.pop(rid)
        if e["i1"] - e["i0"] >= 6:
            eps.append(e)

    for i, f in enumerate(fr):
        if f.play != "open":
            for rid in list(cur):
                cerrar(rid)
            continue
        b = f.b
        for rid, r in f.own.items():
            ok = (r.role not in ("keeper", "striker") and b[0] > 0.25 and _dp(r, b) < 0.15 and alineacion(r, b) > 0.5
                  and arco_abierto(b[0], b[1], f.riv.values()) >= 0.5)
            if ok:
                e = cur.setdefault(rid, {"i0": i, "skills": Counter()})
                e["skills"][r.skill] += 1
                e["i1"] = i
            elif rid in cur:
                cerrar(rid)
    for rid in list(cur):
        cerrar(rid)
    out = []
    for e in sorted(eps, key=lambda e: e["i0"]):
        f0 = fr[e["i0"]]
        st = next((r for r in f0.own.values() if r.role == "striker"), None)
        out.append({"t": f0.t, "dur": (e["i1"] - e["i0"]) / HZ, "skills": e["skills"], "b": f0.b,
                    "gol": p.goles_en("favor", e["i0"], e["i1"] + int(3 * HZ)),
                    "d_atacante": _dp(st, f0.b) if st else None})
    return out


def seccion_soporte(p: Partido) -> list[str]:
    fr = p.cuadros
    sk, xs, salv, resc = Counter(), [], Counter(), Counter()
    ult_salv, ult_resc = {}, {}
    pegado = n_abierto = 0
    for i, f in enumerate(fr):
        if f.play != "open":
            continue
        n_abierto += 1
        b = f.b
        for rid, r in f.own.items():
            if r.role == "support":
                sk[r.skill] += 1
                xs.append(r.x)
            d = _dp(r, b)
            # salvada: toca la pelota que iba hacia nuestro arco en nuestro tercio y 0.7 s después
            # la pelota está más lejos de nuestro arco o quieta
            if b[0] < -0.20 and b[2] < -0.15 and d < CONTACTO and i - ult_salv.get(rid, -999) > 60:
                ult_salv[rid] = i
                luego = fr[min(len(fr) - 1, i + 42)]
                if luego.b[0] > b[0] + 0.03 or math.hypot(luego.b[2], luego.b[3]) < 0.05:
                    salv[r.role] += 1
            # rescate: toque en nuestra mitad tras el que la pelota se aleja ≥ 10 cm de nuestro arco en 1 s
            if b[0] <= 0.0 and d < CONTACTO + 0.01 and i - ult_resc.get(rid, -999) > 60:
                ult_resc[rid] = i
                if fr[min(len(fr) - 1, i + 60)].b[0] > b[0] + 0.10:
                    resc[r.role] += 1
        # un rival que no lleva la pelota pegado a uno de nuestros jugadores de campo
        if f.riv:
            port = min(f.riv.items(), key=lambda kv: _dp(kv[1], b))[0]
            if any(rid != port and any(_dr(r, o) < PEGADO and o.role != "keeper" for o in f.own.values())
                   for rid, r in f.riv.items()):
                pegado += 1
    n_sk = sum(sk.values())
    xs.sort()
    lines = ["Soporte:",
             "  skills en juego abierto: " + (", ".join(f"{k or '(sin skill)'} {100 * v / n_sk:.0f} %" for k, v in sk.most_common(5)) or "—")
             + (f"; x mediana {xs[len(xs) // 2]:+.2f} m" if xs else ""),
             "  salvadas (toca la pelota que venía a nuestro arco) por rol: " + _conteo(salv),
             "  rescates en nuestra mitad (la aleja ≥ 10 cm en 1 s) por rol: " + _conteo(resc),
             f"  rival sin la pelota pegado a uno de nuestros jugadores de campo: {100 * pegado / max(n_abierto, 1):.1f} % del juego abierto"]
    oc = ocasiones_soporte(p)
    lines.append(f"  ocasiones de gol del soporte (pelota a < 0.15 m, detrás de ella, arco abierto ≥ 50 %): {len(oc)}"
                 + (f", con gol en 3 s: {sum(e['gol'] for e in oc)}" if oc else ""))
    for e in oc:
        lines.append(f"    t={e['t']:7.1f} s {e['dur']:.2f} s; pelota ({e['b'][0]:+.2f}, {e['b'][1]:+.2f}), atacante a {_f(e['d_atacante'], '.2f')} m; "
                     f"skills {_conteo(e['skills'], 3)}; gol {'sí' if e['gol'] else 'no'}")
    return lines


def _conteo(c: Counter, n: int = 6) -> str:
    return ", ".join(f"{k or '—'} {v}" for k, v in c.most_common(n)) or "—"


# ---------------------------------------------------------------- ataque

def junto_al_arco(p: Partido) -> list[dict]:
    """Episodios ≥ 0.25 s con la pelota a < 0.25 m de la boca del arco rival en juego abierto."""
    fr, eps, cur = p.cuadros, [], None
    for i, f in enumerate(fr + [None]):
        b = f.b if f else None
        if f and f.play == "open" and b[0] > GL - 0.25 and abs(b[1]) < AREA_HY:
            if cur is None:
                cur = {"i0": i, "sup": 9.0, "st": 9.0, "sup_sk": Counter(), "st_sk": Counter(), "abierto": 0.0}
            cur["i1"] = i
            for r in f.own.values():
                d = _dp(r, b)
                if r.role == "support":
                    cur["sup"] = min(cur["sup"], d)
                    cur["sup_sk"][r.skill] += 1
                elif r.role == "striker":
                    cur["st"] = min(cur["st"], d)
                    cur["st_sk"][r.skill] += 1
            cur["abierto"] = max(cur["abierto"], arco_abierto(b[0], b[1], f.riv.values()))
        elif cur is not None:
            if cur["i1"] - cur["i0"] >= 15:
                cur["gol"] = p.goles_en("favor", cur["i0"], cur["i1"] + int(HZ))
                eps.append(cur)
            cur = None
    return eps


def seccion_ataque(p: Partido) -> list[str]:
    fr = p.cuadros
    ab = [f for f in fr if f.play == "open"]
    n = max(len(ab), 1)
    mitad = sum(1 for f in ab if f.b[0] > 0) / n
    tercio = sum(1 for f in ab if f.b[0] > 0.25) / n
    # tiro: pelota a > 0.25 m/s hacia el arco rival que cruzaría la línea dentro de ±0.25 en < 2 s
    tiros, ult = 0, -999
    for i, f in enumerate(fr):
        bx, by, vx, vy = f.b
        if f.play == "open" and vx > 0.25 and bx > 0.0 and i - ult > 60:
            tt = (GL - bx) / vx
            if 0 < tt < 2.0 and abs(by + vy * tt) < 0.25:
                tiros, ult = tiros + 1, i
    sk = Counter(r.skill for f in ab for r in f.own.values() if r.role == "striker")
    tot = sum(sk.values()) or 1
    lines = [f"Ataque: juego abierto {len(ab) / HZ:.0f} s; pelota en la mitad rival {100 * mitad:.0f} %, en el último tercio "
             f"{100 * tercio:.0f} %; tiros al arco {tiros}",
             "  atacante (striker): " + (", ".join(f"{k or '—'} {100 * v / tot:.0f} %" for k, v in sk.most_common(5)) or "—")]
    eps = junto_al_arco(p)
    lines.append(f"  pelota junto al arco rival (x > {GL - 0.25:.2f}, |y| < {AREA_HY:.2f}, ≥ 0.25 s): {len(eps)} episodios"
                 + (f", con gol: {sum(e['gol'] for e in eps)}" if eps else ""))
    for e in eps:
        sup = "sin soporte" if e["sup"] >= 9 else f"soporte a {e['sup']:.2f} m ({_conteo(e['sup_sk'], 2)})"
        st = "sin atacante" if e["st"] >= 9 else f"atacante a {e['st']:.2f} m ({_conteo(e['st_sk'], 2)})"
        lines.append(f"    t={fr[e['i0']].t:7.1f} s {(e['i1'] - e['i0']) / HZ:4.1f} s; arco abierto máx {e['abierto']:.2f}; {sup}; {st}; "
                     f"gol {'sí' if e['gol'] else 'no'}")
    return lines


# ---------------------------------------------------------------- saques

def saques(p: Partido) -> list[dict]:
    """Cada saque inicial: el `open` que sigue a una orden `kickoff_*` (aunque en medio haya `hold` o
    nada, como con el árbitro a mano), con la pelota a < 15 cm del centro. La pelota al empezar y a
    los 3 s, y gol a favor o en contra en 10 s."""
    fr, out, pendiente = p.cuadros, [], None
    for i in range(1, len(fr)):
        q = fr[i].play
        if q == fr[i - 1].play:
            continue
        if q.startswith("kickoff"):
            pendiente = q
        elif q == "open":
            if pendiente and math.hypot(fr[i].b[0], fr[i].b[1]) < 0.15:
                out.append({"tipo": pendiente, "t": fr[i].t, "b0": fr[i].b, "b3": fr[min(len(fr) - 1, i + int(3 * HZ))].b,
                            "favor": p.goles_en("favor", i, i + int(10 * HZ)),
                            "contra": p.goles_en("contra", i, i + int(10 * HZ))})
            pendiente = None
        elif q not in ("hold", ""):
            pendiente = None  # otra orden (penal, free ball...) reemplaza al saque
    return out


def seccion_saques(p: Partido) -> list[str]:
    ss = saques(p)
    lines = []
    for tipo, titulo in (("kickoff_ours", "Saques propios"), ("kickoff_theirs", "Saques rivales")):
        sel = [s for s in ss if s["tipo"] == tipo]
        lines.append(f"{titulo}: {len(sel)}" + (f"; gol en 10 s a favor {sum(s['favor'] for s in sel)}, en contra {sum(s['contra'] for s in sel)}" if sel else ""))
        for s in sel:
            gol = "a favor" if s["favor"] else "en contra" if s["contra"] else "no"
            lines.append(f"  t={s['t']:7.1f} s pelota ({s['b0'][0]:+.2f}, {s['b0'][1]:+.2f}) → a los 3 s ({s['b3'][0]:+.2f}, {s['b3'][1]:+.2f}); "
                         f"gol en 10 s: {gol}")
    return lines


# ---------------------------------------------------------------- casi gol

def casi_goles(p: Partido) -> list[dict]:
    """Pelota a < 0.15 m de la línea rival frente a la boca, en juego abierto. Por episodio: si hubo
    gol en 1.5 s y el propio de campo mejor puesto (más cerca, desempata la alineación al arco)."""
    fr, eps, cur = p.cuadros, [], None
    for i, f in enumerate(fr + [None]):
        b = f.b if f else None
        if f and f.play == "open" and GL - 0.15 < b[0] < GL + 0.02 and abs(b[1]) < GH + 0.03:
            cur = cur or {"i0": i, "idx": []}
            cur["i1"] = i
            cur["idx"].append(i)
        elif cur:
            eps.append(cur)
            cur = None
    out = []
    for e in eps:
        mejor, sup_area = None, False
        for i in e["idx"]:
            f = fr[i]
            for r in f.own.values():
                if r.role == "keeper":
                    continue
                if r.role == "support" and r.x >= AREA_X and abs(r.y) <= AREA_HY:
                    sup_area = True
                d, al = _dp(r, f.b), alineacion(r, f.b)
                if mejor is None or -d + 0.1 * al > mejor["score"]:
                    mejor = {"score": -d + 0.1 * al, "t": f.t, "rol": r.role, "skill": r.skill, "d": d, "al": al, "b": f.b,
                             "abierto": arco_abierto(f.b[0], f.b[1], f.riv.values())}
        out.append({"t": fr[e["i0"]].t, "dur": (e["i1"] - e["i0"]) / HZ, "gol": p.goles_en("favor", e["i0"], e["i1"] + int(1.5 * HZ)),
                    "mejor": mejor, "sup_area": sup_area})
    return out


def seccion_casi_gol(p: Partido) -> list[str]:
    eps = casi_goles(p)
    sin = [e for e in eps if not e["gol"]]
    lines = [f"Casi gol (pelota a < 0.15 m de la línea rival frente al arco): {len(eps)} episodios, {len(eps) - len(sin)} con gol, "
             f"{len(sin)} sin gol"]
    for e in sin:
        m = e["mejor"]
        quien = ("ningún jugador de campo" if m is None else
                 f"más cerca {m['rol'] or '—'} ({m['skill'] or '—'}) a {m['d']:.2f} m, alineado {m['al']:+.2f}; "
                 f"pelota ({m['b'][0]:+.2f}, {m['b'][1]:+.2f}) a ({m['b'][2]:+.2f}, {m['b'][3]:+.2f}) m/s; arco abierto {m['abierto']:.2f}")
        lines.append(f"  t={e['t']:7.1f} s {e['dur']:4.1f} s; {quien}; soporte en el área rival: {'sí' if e['sup_area'] else 'no'}")
    return lines


# ---------------------------------------------------------------- informe

SECCION = {"goles": seccion_goles, "soporte": seccion_soporte, "ataque": seccion_ataque, "saques": seccion_saques,
           "casi_gol": seccion_casi_gol}


def informe(paths: list[str], lado: float | None = None, solo: list[str] | None = None) -> str:
    secciones = [s for s in SECCIONES if not solo or s in solo]
    lines, caras_ms, favor, contra = [], [], 0, 0
    for path in paths:
        p = cargar(path, lado)
        if not p.cuadros:
            lines.append(f"===== {p.nombre}: sin datos")
            continue
        dur = p.cuadros[-1].t - p.cuadros[0].t
        marco = "atacamos hacia +x en el registro" if p.s > 0 else "atacamos hacia -x en el registro (se giró 180°)"
        lines.append(f"===== {p.nombre}: {dur:.0f} s de registro, {p.abierto_s:.0f} s de juego abierto; {marco}")
        favor += sum(1 for g in p.goles if g["lado"] == "favor")
        contra += sum(1 for g in p.goles if g["lado"] == "contra")
        for s in secciones:
            if s == "caras":
                m = medir_caras(p)
                caras_ms.append(m)
                lines += texto_caras(m)
            else:
                lines += SECCION[s](p)
    if len(paths) > 1 and ("goles" in secciones or "caras" in secciones):
        lines.append(f"===== Total ({len(paths)} registros)")
        if "goles" in secciones:
            lines.append(f"Goles a favor {favor}, en contra {contra}")
        if caras_ms:
            lines += texto_caras(sumar_caras(caras_ms))
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="informe detallado de un partido, desde el registro de un equipo")
    ap.add_argument("csv", nargs="+", help="registros VSSL_MATCH_LOG, .csv o .csv.gz (acepta comodines)")
    ap.add_argument("--lado", type=float, default=None, help="+1 si atacamos hacia +x, -1 si hacia -x (si no, se deduce del arquero)")
    ap.add_argument("--solo", action="append", default=None, metavar="SECCION",
                    help="solo esta sección (" + ", ".join(SECCIONES) + "); varias con coma o repitiendo --solo")
    args = ap.parse_args(argv)
    solo = None
    if args.solo:
        solo = [s.strip() for a in args.solo for s in a.split(",") if s.strip()]
        malas = [s for s in solo if s not in SECCIONES]
        if malas:
            ap.error(f"sección desconocida: {', '.join(malas)} (hay {', '.join(SECCIONES)})")
    paths = [p for pat in args.csv for p in (sorted(glob.glob(pat)) or [pat])]
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")  # consola de Windows sin UTF-8: "≥" y "→" no caen
    print(informe(paths, args.lado, solo))
    return 0


if __name__ == "__main__":
    sys.exit(main())
