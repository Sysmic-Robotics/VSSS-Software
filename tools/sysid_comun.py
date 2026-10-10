"""Piezas comunes del ajuste y la validación de la sysid sobre (v, ω).

Solo biblioteca estándar (sin numpy). El modelo del actuador es el mismo que aplica
`src/radio/actuator.rs`; si cambia uno, cambia el otro (lo cubre el ensayo
`tools/sysid_ensayo.sh`, que compara la planta de Rust con este modelo).
"""

import csv
import math
import os

# Paso de la grilla de simulación y de comparación (s): el loop del engine.
DT = 1.0 / 60.0

PARAMS = [
    "latency_ms",
    "tau_v_s",
    "tau_w_s",
    "max_accel_m_s2",
    "max_alpha_rad_s2",
    "deadzone_wheel_mm_s",
    "max_wheel_mm_s",
    "wheel_track_mm",
    "gain_v",
    "gain_w",
]

# Track y tope del firmware vigente (CLAUDE.md §4.3): para la fracción saturada.
FIRMWARE_TRACK_MM = 75.0
FIRMWARE_MAX_WHEEL_MM_S = 450.0


# ─────────────────────────────── Archivos ───────────────────────────────


def leer_comandos(path):
    """CSV de `skill_test` → lista de (t s, v mm/s, ω °/s), en orden."""
    with open(path, newline="") as f:
        r = csv.reader(f)
        head = next(r)
        it, iv, iw = head.index("t_ms"), head.index("v_mm_s"), head.index("w_deg_s")
        out = []
        for row in r:
            if len(row) <= max(it, iv, iw) or row[iv] == "" or row[iw] == "":
                continue
            out.append((float(row[it]) / 1000.0, int(row[iv]), int(row[iw])))
    return out


def leer_poses(path):
    """`<perfil>.pose.csv` → lista de (t s, x, y, θ desenvuelto)."""
    out = []
    with open(path, newline="") as f:
        r = csv.reader(f)
        head = next(r)
        it, ix, iy, ith = (head.index(k) for k in ("t_ms", "x", "y", "theta"))
        for row in r:
            out.append((float(row[it]) / 1000.0, float(row[ix]), float(row[iy]), float(row[ith])))
    return desenvolver(out)


def desenvolver(poses):
    out, prev, off = [], None, 0.0
    for t, x, y, th in poses:
        if prev is not None:
            d = th + off - prev
            while d > math.pi:
                off -= 2 * math.pi
                d -= 2 * math.pi
            while d < -math.pi:
                off += 2 * math.pi
                d += 2 * math.pi
        prev = th + off
        out.append((t, x, y, prev))
    return out


def perfiles_en(directorio):
    """Perfiles con CSV de comandos y de poses en el directorio, en orden alfabético."""
    nombres = []
    for f in sorted(os.listdir(directorio)):
        if f.endswith(".pose.csv"):
            stem = f[: -len(".pose.csv")]
            if os.path.exists(os.path.join(directorio, stem + ".csv")):
                nombres.append(stem)
    return nombres


# ─────────────────────────────── Modelo ───────────────────────────────


def setpoint_firmware(m, v_mm_s, w_deg_s):
    """Lo que hace el firmware con la consigna: ruedas con el track, tope por rueda que
    escala las dos y zona muerta por rueda; de vuelta a (v m/s, ω rad/s)."""
    ht = m["wheel_track_mm"] / 2000.0
    mw = m["max_wheel_mm_s"] / 1000.0
    dz = m["deadzone_wheel_mm_s"] / 1000.0
    v = v_mm_s / 1000.0
    w = math.radians(w_deg_s)
    l, r = v - w * ht, v + w * ht
    peak = max(abs(l), abs(r))
    if peak > mw:
        l *= mw / peak
        r *= mw / peak
    if abs(l) < dz:
        l = 0.0
    if abs(r) < dz:
        r = 0.0
    return (l + r) / 2.0, (r - l) / (2.0 * ht)


def simular(m, comandos, dt=DT):
    """Salida (v m/s, ω rad/s) del modelo para una consigna por tick, como
    `ActuatorState::step` con `dt` fijo."""
    d = max(0, math.ceil(m["latency_ms"] / 1000.0 / dt - 1e-9))
    gv, gw = m.get("gain_v", 1.0), m.get("gain_w", 1.0)

    def ganancia(tau):
        return 1.0 - math.exp(-dt / tau) if tau > 0 else 1.0

    av, aw = ganancia(m["tau_v_s"]), ganancia(m["tau_w_s"])
    lv, lw = m["max_accel_m_s2"] * dt, m["max_alpha_rad_s2"] * dt
    memo = {}
    sp = []
    for c in comandos:
        if c not in memo:
            sv, sw = setpoint_firmware(m, c[0], c[1])
            memo[c] = (gv * sv, gw * sw)
        sp.append(memo[c])
    v = w = 0.0
    out = []
    for n in range(len(comandos)):
        tv, tw = sp[n - d] if n >= d else (0.0, 0.0)
        dv = (tv - v) * av
        dw = (tw - w) * aw
        v += -lv if dv < -lv else (lv if dv > lv else dv)
        w += -lw if dw < -lw else (lw if dw > lw else dw)
        out.append((v, w))
    return out


def integrar(vw, dt=DT, pose=(0.0, 0.0, 0.0)):
    """Poses al inicio de cada tick (la primera es `pose`), como la planta de Rust."""
    x, y, th = pose
    out = []
    for v, w in vw:
        out.append((x, y, th))
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th += w * dt
    out.append((x, y, th))
    return out


# ─────────────────────────────── Grillas ───────────────────────────────


def grilla(comandos, t0, t1, dt=DT):
    """Tiempos k·dt desde `t0` hasta `t1` y la consigna vigente (retención) en cada uno.
    El `t_ms` del CSV está redondeado al ms: una consigna cuenta desde 1 ms antes."""
    n = int((t1 - t0) / dt) + 1
    tiempos = [t0 + k * dt for k in range(n)]
    cmds, j, cur = [], 0, (0, 0)
    for t in tiempos:
        while j < len(comandos) and comandos[j][0] <= t + 0.0011:
            cur = (comandos[j][1], comandos[j][2])
            j += 1
        cmds.append(cur)
    return tiempos, cmds


K_VENTANA = 3  # medio ancho de la ventana del derivador, en ticks de 60 Hz (±50 ms)


def velocidades_medidas(poses, centros, k=K_VENTANA, dt=DT):
    """(v, ω) medidos en cada centro: recta por mínimos cuadrados a x, y, θ con las poses
    a menos de k·dt del centro (derivador suavizado; la diferencia finita amplifica el
    ruido). v es la proyección sobre el heading. `None` si hay menos de 3 poses."""
    h = k * dt
    out = []
    lo = 0
    n = len(poses)
    for c in centros:
        while lo < n and poses[lo][0] < c - h:
            lo += 1
        hi = lo
        while hi < n and poses[hi][0] < c + h:
            hi += 1
        pts = poses[lo:hi]
        if len(pts) < 3:
            out.append(None)
            continue
        m = len(pts)
        tm = sum(p[0] for p in pts) / m
        stt = sum((p[0] - tm) ** 2 for p in pts)
        if stt <= 0:
            out.append(None)
            continue
        xm = sum(p[1] for p in pts) / m
        ym = sum(p[2] for p in pts) / m
        thm = sum(p[3] for p in pts) / m
        sx = sum((p[0] - tm) * (p[1] - xm) for p in pts) / stt
        sy = sum((p[0] - tm) * (p[2] - ym) for p in pts) / stt
        sth = sum((p[0] - tm) * (p[3] - thm) for p in pts) / stt
        th_c = thm + sth * (c - tm)
        out.append((sx * math.cos(th_c) + sy * math.sin(th_c), sth))
    return out


def nucleo(k=K_VENTANA):
    """Pesos sobre los tramos de la grilla que equivalen al derivador de
    `velocidades_medidas` (recta a 2k poses equiespaciadas centradas en el medio del
    tramo 0): el tramo i pesa la suma de los desvíos de las poses posteriores.
    Devuelve [(desplazamiento, peso)], con pesos que suman 1."""
    offs = [j - 0.5 for j in range(-k + 1, k + 1)]  # poses respecto del centro, en ticks
    pesos = []
    for i in range(-k + 1, k):
        pesos.append((i, sum(o for o in offs if o > i)))
    total = sum(p for _, p in pesos)
    return [(i, p / total) for i, p in pesos]


def suavizar(serie, kern):
    """Serie filtrada con el núcleo (los bordes repiten el primer y el último valor)."""
    n = len(serie)
    kmin = min(o for o, _ in kern)
    kmax = max(o for o, _ in kern)
    pad = [serie[0]] * (-kmin) + list(serie) + [serie[-1]] * kmax
    out = [0.0] * n
    for off, w in kern:
        s = off - kmin
        out = [a + w * b for a, b in zip(out, pad[s : s + n])]
    return out


def segmentos(cmds, cola=60):
    """Tramos en movimiento: [inicio, fin) de cada racha con consigna distinta de cero,
    separadas por pausas de al menos 1 s, más `cola` ticks de la pausa siguiente (la
    respuesta sigue después de que la consigna vuelve a cero)."""
    pausa = int(round(1.0 / DT)) - 2
    tramos, i, n = [], 0, len(cmds)
    while i < n:
        if cmds[i] == (0, 0):
            i += 1
            continue
        ini = i
        ceros = 0
        while i < n and ceros < pausa:
            ceros = ceros + 1 if cmds[i] == (0, 0) else 0
            i += 1
        fin = i - ceros
        tramos.append((ini, min(n, fin + cola)))
    return tramos


# ─────────────────────────────── Parche de visión ───────────────────────────────


def estimar_parche(perfiles):
    """Desplazamiento (dx, dy) del parche de visión respecto del eje de giro, en el marco
    del robot (m), con los ticks de giro puro: ahí el eje queda quieto y la pose medida
    describe un círculo. `perfiles` = [(poses, comandos)]. `None` sin giro puro."""
    filas = []
    for poses, comandos in perfiles:
        # Tramos de giro puro: poses mientras la consigna es (0, ω ≠ 0).
        j, cur, grupo = 0, (0, 0), []
        for t, x, y, th in poses:
            while j < len(comandos) and comandos[j][0] <= t:
                cur = (comandos[j][1], comandos[j][2])
                j += 1
            if cur[0] == 0 and cur[1] != 0:
                grupo.append((x, y, th))
            elif grupo:
                filas.append(grupo)
                grupo = []
        if grupo:
            filas.append(grupo)
    # Por grupo: x − x̄ = dx (c − c̄) − dy (s − s̄); y − ȳ = dx (s − s̄) + dy (c − c̄).
    a11 = a12 = a22 = b1 = b2 = 0.0
    for g in filas:
        if len(g) < 10:
            continue
        n = len(g)
        xm = sum(p[0] for p in g) / n
        ym = sum(p[1] for p in g) / n
        cm = sum(math.cos(p[2]) for p in g) / n
        sm = sum(math.sin(p[2]) for p in g) / n
        for x, y, th in g:
            c, s = math.cos(th) - cm, math.sin(th) - sm
            # fila x: [c, −s]; fila y: [s, c]
            a11 += c * c + s * s
            a22 += s * s + c * c
            a12 += -c * s + s * c
            b1 += c * (x - xm) + s * (y - ym)
            b2 += -s * (x - xm) + c * (y - ym)
    det = a11 * a22 - a12 * a12
    if det <= 1e-9:
        return None
    return ((b1 * a22 - b2 * a12) / det, (a11 * b2 - a12 * b1) / det)


def corregir_parche(poses, parche):
    """Poses del eje de giro a partir de las del parche."""
    if not parche:
        return poses
    dx, dy = parche
    return [
        (t, x - dx * math.cos(th) + dy * math.sin(th), y - dx * math.sin(th) - dy * math.cos(th), th)
        for t, x, y, th in poses
    ]


# ─────────────────────────────── Métricas de trayectoria ───────────────────────────────


def interpolar(poses, t):
    """Pose interpolada linealmente en `t` (poses ordenadas); `None` fuera de rango."""
    if not poses or t < poses[0][0] or t > poses[-1][0]:
        return None
    lo, hi = 0, len(poses) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if poses[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    a, b = poses[lo], poses[hi]
    if b[0] == a[0]:
        return a[1:]
    f = (t - a[0]) / (b[0] - a[0])
    return tuple(a[i] + f * (b[i] - a[i]) for i in (1, 2, 3))


def dtw(a, b):
    """DTW con distancia euclídea en el plano, normalizado: costo medio por paso del
    camino óptimo (0 si las trayectorias son iguales)."""
    n, m = len(a), len(b)
    if n == 0 or m == 0:
        return float("nan")
    inf = float("inf")
    prev = [(inf, 0)] * (m + 1)
    prev[0] = (0.0, 0)
    for i in range(1, n + 1):
        cur = [(inf, 0)] * (m + 1)
        ax, ay = a[i - 1]
        for j in range(1, m + 1):
            bx, by = b[j - 1]
            d = math.hypot(ax - bx, ay - by)
            best = min(prev[j - 1], prev[j], cur[j - 1])
            cur[j] = (best[0] + d, best[1] + 1)
        prev = cur
    total, pasos = prev[m]
    return total / pasos if pasos else float("nan")
