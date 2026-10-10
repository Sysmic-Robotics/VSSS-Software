#!/usr/bin/env python3
"""Ajuste del modelo del actuador (sysid sobre (v, ω)) y generador de datos sintéticos.

Uso:
  # Ajustar una sesión del robot real y guardarla en config/real_calibration.json:
  python3 tools/sysid_ajuste.py ajustar ~/sysid/1/full --robot 1 --mi-robot-id 2

  # Dinámica propia de FIRASim (sesión con --transport firasim, sin la capa):
  python3 tools/sysid_ajuste.py ajustar ~/sysid/firasim --firasim

  # Datos sintéticos de un modelo conocido, a partir de los CSV de comandos de una
  # sesión (por ejemplo, de skill_test --transport plant):
  python3 tools/sysid_ajuste.py sintetico DIR_COMANDOS DIR_SALIDA [--modelo k=v,...]

Lee, por perfil, `<perfil>.csv` (comandos de skill_test), `<perfil>.pose.csv` (pose
cruda de visión) y `<perfil>.meta.json`. Estima, con el mismo modelo que aplica
`src/radio/actuator.rs`:
  - la latencia de punta a punta (comando enviado → movimiento visto por el engine);
  - τ de primer orden de v y de ω, y la aceleración máxima de cada uno;
  - la ganancia en régimen de v y de ω, el tope por rueda y el track efectivo;
  - la zona muerta por rueda;
  - el desplazamiento del parche de visión respecto del eje de giro.
Ajusta por mínimos cuadrados (descenso por coordenadas) la velocidad medida (derivador
suavizado de la pose) contra la del modelo pasada por el mismo derivador, e informa el
error: RMS y R² por perfil e intervalos del 90 % por bootstrap sobre los tramos.

Solo biblioteca estándar.
"""

import argparse
import datetime
import json
import math
import os
import random
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sysid_comun as sc  # noqa: E402

CALIBRACION = "config/real_calibration.json"

# Modelo conocido del escenario de la spec (y del ensayo).
MODELO_SINTETICO = {
    "latency_ms": 120.0,
    "tau_v_s": 0.15,
    "tau_w_s": 0.10,
    "max_accel_m_s2": 2.0,
    "max_alpha_rad_s2": 60.0,
    "deadzone_wheel_mm_s": 20.0,
    "max_wheel_mm_s": 450.0,
    "wheel_track_mm": 75.0,
    "gain_v": 1.0,
    "gain_w": 1.0,
}

INICIAL = {
    "latency_ms": 100.0,
    "tau_v_s": 0.1,
    "tau_w_s": 0.1,
    "max_accel_m_s2": 5.0,
    "max_alpha_rad_s2": 100.0,
    "deadzone_wheel_mm_s": 10.0,
    "max_wheel_mm_s": 450.0,
    "wheel_track_mm": 75.0,
    "gain_v": 1.0,
    "gain_w": 1.0,
}

TAU = [0.0, 0.01, 0.02, 0.04, 0.07, 0.1, 0.15, 0.2, 0.3, 0.45, 0.7, 1.0]
GAIN = [0.5, 0.7, 0.8, 0.85, 0.9, 0.95, 1.0, 1.05, 1.1, 1.2, 1.35, 1.5]
GRILLAS = {
    "gain_v": GAIN,
    "gain_w": GAIN,
    "max_wheel_mm_s": [100, 200, 300, 350, 400, 450, 500, 600, 800, 1000, 1500, 2000],
    "wheel_track_mm": [30, 45, 55, 62, 68, 72, 75, 78, 82, 90, 110, 150, 200],
    "tau_v_s": TAU,
    "tau_w_s": TAU,
    "max_accel_m_s2": [0.2, 0.4, 0.7, 1.0, 1.5, 2.0, 3.0, 4.5, 7.0, 10.0, 15.0, 30.0],
    "max_alpha_rad_s2": [2, 5, 10, 20, 35, 50, 75, 100, 150, 250, 400, 600],
    "deadzone_wheel_mm_s": [0, 5, 10, 15, 20, 25, 30, 40, 50, 70, 100, 150],
}
ORDEN = list(GRILLAS)
MAX_LATENCIA_TICKS = 30  # 500 ms

# Unidades para el informe.
UNIDAD = {
    "latency_ms": "ms",
    "tau_v_s": "s",
    "tau_w_s": "s",
    "max_accel_m_s2": "m/s²",
    "max_alpha_rad_s2": "rad/s²",
    "deadzone_wheel_mm_s": "mm/s",
    "max_wheel_mm_s": "mm/s",
    "wheel_track_mm": "mm",
    "gain_v": "",
    "gain_w": "",
}


# ─────────────────────────────── Datos ───────────────────────────────


class Perfil:
    """Un perfil de la sesión en la grilla de 60 Hz: consigna y velocidad medida."""

    def __init__(self, nombre, comandos, poses):
        self.nombre = nombre
        t0 = comandos[0][0]
        t1 = comandos[-1][0] + 0.5
        self.tiempos, self.cmds = sc.grilla(comandos, t0, t1)
        centros = [t + sc.DT / 2 for t in self.tiempos]
        self.medido = sc.velocidades_medidas(poses, centros)
        self.validos = [i for i, m in enumerate(self.medido) if m is not None]
        self.mv = [self.medido[i][0] for i in self.validos]
        self.mw = [self.medido[i][1] for i in self.validos]
        self.tramos = sc.segmentos(self.cmds)
        # Ejes que el perfil excita (para el R²).
        self.excita = (any(c[0] for c in self.cmds), any(c[1] for c in self.cmds))


def cargar_sesion(directorio, corregir=True):
    nombres = sc.perfiles_en(directorio)
    if not nombres:
        sys.exit(f"{directorio}: no hay pares <perfil>.csv + <perfil>.pose.csv")
    crudos = []
    for n in nombres:
        cmds = sc.leer_comandos(os.path.join(directorio, n + ".csv"))
        poses = sc.leer_poses(os.path.join(directorio, n + ".pose.csv"))
        if len(cmds) < 10 or len(poses) < 10:
            print(f"  {n}: muy pocos datos ({len(cmds)} comandos, {len(poses)} poses); lo salto")
            continue
        crudos.append((n, cmds, poses))
    parche = sc.estimar_parche([(p, c) for _, c, p in crudos]) if corregir else None
    perfiles = [Perfil(n, c, sc.corregir_parche(p, parche)) for n, c, p in crudos]
    return perfiles, parche


def leer_sesion(directorio):
    """`sesion.json` de tools/sysid_sesion.sh (robot, batería, voltaje, notas), si está."""
    path = os.path.join(directorio, "sesion.json")
    if os.path.exists(path):
        with open(path) as f:
            return json.load(f)
    return {}


def leer_meta(directorio):
    for f in sorted(os.listdir(directorio)):
        if f.endswith(".meta.json"):
            with open(os.path.join(directorio, f)) as fh:
                return json.load(fh)
    return {}


def escalas_de_ruido(perfiles):
    """Desvío de la velocidad medida en reposo (consigna en cero desde hace más de 1 s):
    normaliza los residuos de v y de ω."""
    vs, ws = [], []
    for p in perfiles:
        quieto = 0
        for i, c in enumerate(p.cmds):
            quieto = quieto + 1 if c == (0, 0) else 0
            if quieto > 60 and p.medido[i] is not None:
                vs.append(p.medido[i][0])
                ws.append(p.medido[i][1])

    def desvio(xs, piso):
        if len(xs) < 10:
            return piso
        m = sum(xs) / len(xs)
        return max(piso, math.sqrt(sum((x - m) ** 2 for x in xs) / len(xs)))

    return desvio(vs, 0.005), desvio(ws, 0.05)


def chequear_signos(perfiles):
    """El robot tiene que ir hacia donde la visión dice que mira: si v medida y pedida
    se correlacionan al revés, el heading de visión está girado 180° respecto de +v."""
    for eje, nombre in ((0, "v"), (1, "ω")):
        acc = 0.0
        for p in perfiles:
            for i in p.validos:
                acc += p.cmds[i][eje] * p.medido[i][eje]
        if acc < 0:
            sys.exit(
                f"✗ {nombre} medida va al revés de la pedida. Con v: revisar "
                "vision.real_theta_offset_deg (team_params.json) o los signos de rueda del "
                "firmware; con ω: GYRO_Z_SIGN o el sentido de la visión. No se ajusta."
            )


# ─────────────────────────────── Costo y búsqueda ───────────────────────────────


class Problema:
    def __init__(self, perfiles):
        self.perfiles = perfiles
        self.sv, self.sw = escalas_de_ruido(perfiles)
        self.kern = sc.nucleo()
        self.evaluaciones = 0

    def predicho(self, m, p):
        out = sc.simular(m, p.cmds)
        return sc.suavizar([o[0] for o in out], self.kern), sc.suavizar([o[1] for o in out], self.kern)

    def costo(self, m, pesos=None):
        self.evaluaciones += 1
        total = 0.0
        kv, kw = 1.0 / self.sv**2, 1.0 / self.sw**2
        for k, p in enumerate(self.perfiles):
            vs, ws = self.predicho(m, p)
            if pesos:
                w = pesos[k]
                total += sum(
                    w[i] * ((a - vs[i]) ** 2 * kv + (b - ws[i]) ** 2 * kw)
                    for i, a, b in zip(p.validos, p.mv, p.mw)
                )
            else:
                total += sum((a - vs[i]) ** 2 for i, a in zip(p.validos, p.mv)) * kv
                total += sum((b - ws[i]) ** 2 for i, b in zip(p.validos, p.mw)) * kw
        return total


def dorada(f, a, b, iters=12):
    phi = (math.sqrt(5) - 1) / 2
    c, d = b - phi * (b - a), a + phi * (b - a)
    fc, fd = f(c), f(d)
    for _ in range(iters):
        if fc < fd:
            b, d, fd = d, c, fc
            c = b - phi * (b - a)
            fc = f(c)
        else:
            a, c, fc = c, d, fd
            d = a + phi * (b - a)
            fd = f(d)
    return (c, fc) if fc < fd else (d, fd)


def buscar_latencia(prob, m, pesos, ticks):
    mejor = (None, float("inf"))
    for d in ticks:
        if d < 0 or d > MAX_LATENCIA_TICKS:
            continue
        lat = d * sc.DT * 1000.0
        c = prob.costo(dict(m, latency_ms=lat), pesos)
        if c < mejor[1]:
            mejor = (lat, c)
    return mejor


def ajustar(prob, m, pesos=None, barridos=4, local=False):
    """Descenso por coordenadas: por parámetro, grilla gruesa y sección dorada entre los
    vecinos del mejor punto. Con `local`, solo la dorada en [x/2, 2x] (bootstrap)."""
    m = dict(m)
    actual = prob.costo(m, pesos)
    for _ in range(barridos):
        antes = actual
        d0 = round(m["latency_ms"] / 1000.0 / sc.DT)
        ticks = range(d0 - 3, d0 + 4) if local else range(0, MAX_LATENCIA_TICKS + 1)
        lat, c = buscar_latencia(prob, m, pesos, ticks)
        if c < actual:
            m["latency_ms"], actual = lat, c
        for k in ORDEN:
            grid = GRILLAS[k]
            lo_b, hi_b = grid[0], grid[-1]

            def f(x, k=k):
                return prob.costo(dict(m, **{k: x}), pesos)

            if local:
                x0 = m[k]
                a, b = max(lo_b, x0 * 0.5), min(hi_b, max(x0 * 2.0, x0 + 0.02 * (hi_b - lo_b)))
            else:
                costos = [(f(x), i) for i, x in enumerate(grid)]
                _, i = min(costos)
                a, b = grid[max(0, i - 1)], grid[min(len(grid) - 1, i + 1)]
            x, c = dorada(f, a, b)
            if c < actual:
                m[k], actual = x, c
        if antes - actual <= 1e-4 * max(antes, 1.0):
            break
    return m, actual


def bootstrap(prob, m, n, semilla):
    """Intervalos del 90 % remuestreando los tramos en movimiento con reemplazo (los
    ticks en reposo pesan siempre 1) y reajustando desde el mejor modelo."""
    rng = random.Random(semilla)
    todos = [(k, t) for k, p in enumerate(prob.perfiles) for t in p.tramos]
    muestras = {k: [] for k in sc.PARAMS}
    for b in range(n):
        pesos = [[1.0] * len(p.cmds) for p in prob.perfiles]
        for k, p in enumerate(prob.perfiles):
            for ini, fin in p.tramos:
                for i in range(ini, fin):
                    pesos[k][i] = 0.0
        for _ in range(len(todos)):
            k, (ini, fin) = rng.choice(todos)
            for i in range(ini, fin):
                pesos[k][i] += 1.0
        mb, _ = ajustar(prob, m, pesos, barridos=1, local=True)
        for k in sc.PARAMS:
            muestras[k].append(mb[k])
        print(f"  bootstrap {b + 1}/{n}", end="\r", flush=True)
    print()
    ic = {}
    for k, xs in muestras.items():
        xs.sort()
        ic[k] = [xs[int(0.05 * (len(xs) - 1))], xs[int(math.ceil(0.95 * (len(xs) - 1)))]]
    return ic


# ─────────────────────────────── Informe ───────────────────────────────


def calidad(prob, m):
    """RMS y R² de v y de ω por perfil (velocidad medida contra el modelo)."""
    filas = {}
    for p in prob.perfiles:
        vs, ws = prob.predicho(m, p)
        ev, ew, mv, mw = [], [], [], []
        for i in p.validos:
            a, b = p.medido[i]
            ev.append(a - vs[i])
            ew.append(b - ws[i])
            mv.append(a)
            mw.append(b)

        def r2(err, med):
            mu = sum(med) / len(med)
            tot = sum((x - mu) ** 2 for x in med)
            return 1.0 - sum(e * e for e in err) / tot if tot > 0 else float("nan")

        n = len(ev)
        filas[p.nombre] = {
            "rms_v_mm_s": round(1000 * math.sqrt(sum(e * e for e in ev) / n), 1),
            "rms_w_deg_s": round(math.degrees(math.sqrt(sum(e * e for e in ew) / n)), 1),
            "r2_v": round(r2(ev, mv), 3) if p.excita[0] else None,
            "r2_w": round(r2(ew, mw), 3) if p.excita[1] else None,
        }
    return filas


def regimenes(prob):
    """Velocidad medida en régimen (segunda mitad de cada consigna constante de más de
    0.5 s) contra la pedida: ganancia y curva de saturación."""
    out = []
    for p in prob.perfiles:
        i, n = 0, len(p.cmds)
        while i < n:
            j = i
            while j < n and p.cmds[j] == p.cmds[i]:
                j += 1
            c = p.cmds[i]
            if c != (0, 0) and (j - i) * sc.DT >= 0.5:
                med = [p.medido[k] for k in range((i + j) // 2, j) if p.medido[k] is not None]
                if med:
                    v = sum(x[0] for x in med) / len(med)
                    w = sum(x[1] for x in med) / len(med)
                    out.append((p.nombre, c, round(v * 1000), round(math.degrees(w))))
            i = j
    return out


def imprimir(m, ic, filas, parche, prob, reg):
    print("\nParámetros ajustados (intervalo del 90 % por bootstrap):")
    for k in sc.PARAMS:
        lo, hi = ic.get(k, [float("nan")] * 2)
        ancho = (hi - lo) / abs(m[k]) if m[k] else float("inf")
        nota = "  ← poco identificable con estos datos" if ancho > 0.5 else ""
        print(f"  {k:<20} {m[k]:9.3f} {UNIDAD[k]:<7} [{lo:.3f}, {hi:.3f}]{nota}")
    if parche:
        print(f"  parche de visión: {parche[0] * 1000:+.1f} mm adelante, {parche[1] * 1000:+.1f} mm a la izquierda del eje")
    print(f"\nRuido en reposo: v {prob.sv * 1000:.1f} mm/s, ω {math.degrees(prob.sw):.1f} °/s")
    print("\nError del modelo por perfil:")
    print(f"  {'perfil':<10} {'RMS v mm/s':>11} {'R² v':>7} {'RMS ω °/s':>10} {'R² ω':>7}")
    def r2(x):
        return "—" if x is None else x

    for n, f in filas.items():
        print(f"  {n:<10} {f['rms_v_mm_s']:>11} {r2(f['r2_v']):>7} {f['rms_w_deg_s']:>10} {r2(f['r2_w']):>7}")
    if reg:
        print("\nRégimen (pedido → medido):")
        for n, (v, w), mv, mw in reg:
            print(f"  {n:<9} v {v:>5} → {mv:>5} mm/s   ω {w:>5} → {mw:>5} °/s")


def redondeado(m):
    dec = {"latency_ms": 1, "deadzone_wheel_mm_s": 1, "max_wheel_mm_s": 1, "wheel_track_mm": 1}
    return {k: round(m[k], dec.get(k, 3)) for k in sc.PARAMS}


def escribir_calibracion(path, entrada, firasim):
    cal = {"firasim": None, "measurements": []}
    if os.path.exists(path):
        with open(path) as f:
            cal.update(json.load(f))
    if firasim:
        cal["firasim"] = entrada
    else:
        cal["measurements"] = [
            x
            for x in cal.get("measurements", [])
            if not (x["vision_id"] == entrada["vision_id"] and x["battery"] == entrada["battery"])
        ] + [entrada]
        cal["measurements"].sort(key=lambda x: (x["vision_id"], x["battery"]))
    with open(path, "w") as f:
        json.dump(cal, f, indent=2, ensure_ascii=False)
        f.write("\n")


# ─────────────────────────────── Comandos ───────────────────────────────


def cmd_ajustar(a):
    print(f"Sesión {a.directorio}")
    perfiles, parche = cargar_sesion(a.directorio, corregir=not a.sin_parche)
    print(f"  perfiles: {', '.join(p.nombre for p in perfiles)}")
    chequear_signos(perfiles)
    prob = Problema(perfiles)
    m, c = ajustar(prob, INICIAL)
    print(f"  ajuste: costo {c:.0f} en {prob.evaluaciones} evaluaciones")
    ic = bootstrap(prob, m, a.bootstrap, a.semilla) if a.bootstrap > 0 else {}
    filas = calidad(prob, m)
    reg = regimenes(prob)
    imprimir(m, ic, filas, parche, prob, reg)

    fit = {
        "per_profile": filas,
        "ci90": {k: [round(lo, 4), round(hi, 4)] for k, (lo, hi) in ic.items()},
        "bootstrap": a.bootstrap,
        "noise_rest": {"v_mm_s": round(prob.sv * 1000, 1), "w_deg_s": round(math.degrees(prob.sw), 1)},
        "patch_offset_mm": [round(parche[0] * 1000, 1), round(parche[1] * 1000, 1)] if parche else None,
        "window_ms": round(2 * sc.K_VENTANA * sc.DT * 1000),
        "session": os.path.abspath(a.directorio),
    }
    meta = leer_meta(a.directorio)
    sesion = leer_sesion(a.directorio)
    fecha = a.fecha or sesion.get("date") or (
        datetime.date.fromtimestamp(meta["unix_time_s"]).isoformat()
        if meta.get("unix_time_s")
        else datetime.date.today().isoformat()
    )
    notas = a.notas or sesion.get("notes", "")
    mr = redondeado(m)
    if a.firasim:
        entrada = {
            "latency_ms": mr["latency_ms"],
            "tau_v_s": mr["tau_v_s"],
            "tau_w_s": mr["tau_w_s"],
            "date": fecha,
            "notes": notas,
            "fit": dict(fit, model=mr),
        }
    else:

        def primero(*xs):
            return next((x for x in xs if x is not None), None)

        robot = primero(a.robot, sesion.get("vision_id"), meta.get("track"))
        mri = primero(a.mi_robot_id, sesion.get("mi_robot_id"))
        bateria = primero(a.bateria, sesion.get("battery"), meta.get("battery"))
        volts = primero(a.bateria_v, sesion.get("battery_v"), meta.get("battery_v"))
        faltan = [n for n, v in (("--robot", robot), ("--mi-robot-id", mri), ("--bateria", bateria), ("--bateria-v", volts)) if v is None]
        if faltan:
            sys.exit(f"✗ faltan {', '.join(faltan)} (no están en sesion.json ni en el .meta.json de la sesión)")
        entrada = {
            "vision_id": int(robot),
            "mi_robot_id": int(mri),
            "battery": bateria,
            "battery_v": float(volts),
            "date": fecha,
            "notes": notas,
            **mr,
            "vision_latency_ms": 0.0,
            "fit": fit,
        }
    if a.salida == "-":
        print(json.dumps(entrada, indent=2, ensure_ascii=False))
    else:
        escribir_calibracion(a.salida, entrada, a.firasim)
        print(f"\n→ {a.salida} ({'sección firasim' if a.firasim else 'robot %s, %s' % (entrada['vision_id'], entrada['battery'])})")
    if a.esperado:
        return comparar(m, parse_modelo(a.esperado), a.tolerancia)
    return 0


def parse_modelo(texto):
    m = dict(MODELO_SINTETICO)
    if texto and texto != "spec":
        for par in texto.split(","):
            k, v = par.split("=")
            if k.strip() not in sc.PARAMS:
                sys.exit(f"parámetro desconocido: {k}")
            m[k.strip()] = float(v)
    return m


def comparar(m, esperado, tol):
    """Autocontrol del ajuste con un modelo conocido: cada parámetro dentro de ±tol
    (relativo) y la latencia dentro de ±1 tick de 50 ms. Devuelve 0 si pasa."""
    print("\nComparación con el modelo conocido:")
    mal = 0
    for k in sc.PARAMS:
        e, x = esperado[k], m[k]
        if k == "latency_ms":
            ok = abs(x - e) <= 50.0
            desc = f"{x - e:+.1f} ms"
        elif e == 0:
            ok = abs(x) <= 2.0
            desc = f"{x:+.3f}"
        else:
            ok = abs(x - e) <= tol * abs(e)
            desc = f"{100 * (x - e) / e:+.1f} %"
        mal += not ok
        print(f"  {k:<20} esperado {e:8.3f}  ajustado {x:8.3f}  {desc:>9}  {'ok' if ok else 'FUERA'}")
    print("→ " + ("RECUPERA el modelo" if mal == 0 else f"{mal} parámetro(s) fuera de tolerancia"))
    return 0 if mal == 0 else 1


def cmd_sintetico(a):
    """Pose sintética de un modelo conocido, con el ruido del proxy de visión, para los
    CSV de comandos de `entrada`."""
    m = parse_modelo(a.modelo)
    rng = random.Random(a.semilla)
    dx, dy = (float(x) / 1000.0 for x in a.parche_mm.split(","))
    os.makedirs(a.salida, exist_ok=True)
    nombres = [f[:-4] for f in sorted(os.listdir(a.entrada)) if f.endswith(".csv") and not f.endswith(".pose.csv")]
    for n in nombres:
        cmds = sc.leer_comandos(os.path.join(a.entrada, n + ".csv"))
        t0, t1 = cmds[0][0], cmds[-1][0] + 0.5
        tiempos, grid = sc.grilla(cmds, t0, t1)
        poses = sc.integrar(sc.simular(m, grid))
        with open(os.path.join(a.salida, n + ".pose.csv"), "w") as f:
            f.write("t_ms,x,y,theta\n")
            for t, (x, y, th) in zip(tiempos, poses):
                if rng.random() < a.perdida:
                    continue
                px = x + dx * math.cos(th) - dy * math.sin(th) + rng.gauss(0, a.ruido_pos)
                py = y + dx * math.sin(th) + dy * math.cos(th) + rng.gauss(0, a.ruido_pos)
                pth = math.atan2(math.sin(th), math.cos(th)) + rng.gauss(0, a.ruido_theta)
                f.write(f"{t * 1000:.1f},{px:.5f},{py:.5f},{pth:.5f}\n")
        shutil.copy(os.path.join(a.entrada, n + ".csv"), os.path.join(a.salida, n + ".csv"))
        meta_in = os.path.join(a.entrada, n + ".meta.json")
        meta = json.load(open(meta_in)) if os.path.exists(meta_in) else {}
        meta["synthetic_model"] = m
        meta["synthetic_noise"] = {"pos_m": a.ruido_pos, "theta_rad": a.ruido_theta, "drop": a.perdida, "patch_mm": [dx * 1000, dy * 1000]}
        with open(os.path.join(a.salida, n + ".meta.json"), "w") as f:
            json.dump(meta, f, indent=2)
        print(f"  {n}: {len(poses)} poses")
    print(f"→ {a.salida} (modelo {m})")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    aj = sub.add_parser("ajustar", help="ajusta el modelo a una sesión")
    aj.add_argument("directorio")
    aj.add_argument("--salida", default=CALIBRACION, help=f"archivo de calibración (default {CALIBRACION}; '-' = solo imprimir)")
    aj.add_argument("--firasim", action="store_true", help="escribe la sección firasim (dinámica propia del simulador)")
    aj.add_argument("--robot", type=int, help="id de visión (default: sesion.json o el .meta.json)")
    aj.add_argument("--mi-robot-id", type=int, help="MI_ROBOT_ID del firmware (default: sesion.json)")
    aj.add_argument("--bateria", choices=["full", "half"], help="default: sesion.json o el .meta.json")
    aj.add_argument("--bateria-v", type=float, help="voltaje medido (default: sesion.json o el .meta.json)")
    aj.add_argument("--fecha", help="AAAA-MM-DD (default: la de la sesión)")
    aj.add_argument("--notas", default="", help="notas (default: las de sesion.json)")
    aj.add_argument("--bootstrap", type=int, default=20, help="remuestreos para los intervalos (0 = sin intervalos)")
    aj.add_argument("--semilla", type=int, default=1)
    aj.add_argument("--sin-parche", action="store_true", help="no estima ni corrige el desplazamiento del parche")
    aj.add_argument("--esperado", help="modelo conocido para el autocontrol ('spec' o k=v,...)")
    aj.add_argument("--tolerancia", type=float, default=0.15)

    si = sub.add_parser("sintetico", help="genera poses sintéticas de un modelo conocido")
    si.add_argument("entrada", help="directorio con los CSV de comandos")
    si.add_argument("salida")
    si.add_argument("--modelo", default="spec", help="'spec' o k=v,... sobre el modelo de la spec")
    si.add_argument("--ruido-pos", type=float, default=0.00185, help="σ de posición (m), el del proxy")
    si.add_argument("--ruido-theta", type=float, default=0.031, help="σ de orientación (rad), el del proxy")
    si.add_argument("--perdida", type=float, default=0.02, help="fracción de frames perdidos")
    si.add_argument("--parche-mm", default="0,0", help="desplazamiento del parche respecto del eje (adelante,izquierda)")
    si.add_argument("--semilla", type=int, default=7)

    a = ap.parse_args()
    sys.exit(cmd_ajustar(a) if a.cmd == "ajustar" else cmd_sintetico(a))


if __name__ == "__main__":
    main()
