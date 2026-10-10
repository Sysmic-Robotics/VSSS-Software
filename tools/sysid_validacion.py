#!/usr/bin/env python3
"""Validación sim↔real de la sysid (CLAUDE.md §9.7): compara los mismos perfiles
corridos en el robot y en el simulador (FIRASim o la planta, con la capa de actuador
real).

Uso:
  python3 tools/sysid_validacion.py ~/sysid/1/full ~/sysid/1/full-firasim [--parche-mm 8,-5]

Por perfil presente en las dos carpetas:
  - alinea el inicio en el primer comando distinto de cero de cada una y lleva las dos
    trayectorias a la misma pose de partida (traslación y rotación);
  - MEE: distancia media entre las posiciones en el mismo instante (m);
  - DTW de las trayectorias en el plano (costo medio por paso, m);
  - error medio de heading, de v y de ω medidas;
  - error de v en régimen (segunda mitad de cada consigna constante que llega a
    régimen en la referencia: los escalones cortos y rápidos son transitorio);
  - fracción de ticks con una rueda saturada por el tope del firmware (450 mm/s).
Umbrales (design D6, meta inicial, revisables después del laboratorio): MEE < 0.03 m en
v_steps y w_steps, y error de v en régimen < 10 %.

Solo biblioteca estándar.
"""

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sysid_comun as sc  # noqa: E402

UMBRAL_MEE_M = 0.03
UMBRAL_V_REGIMEN = 0.10
PERFILES_MEE = ("v_steps", "w_steps")
DT_COMPARACION = 0.05  # 20 Hz
CUADROS_MIN = 45  # cuadros de visión del simulador en 1 s por debajo de los cuales se avisa


def cargar(directorio, nombre, parche=None):
    cmds = sc.leer_comandos(os.path.join(directorio, nombre + ".csv"))
    poses = sc.corregir_parche(sc.leer_poses(os.path.join(directorio, nombre + ".pose.csv")), parche)
    inicio = next((t for t, v, w in cmds if (v, w) != (0, 0)), None)
    return cmds, poses, inicio


def relativa(poses, t0):
    """Poses con el tiempo desde `t0` y la pose de `t0` como origen."""
    p0 = sc.interpolar(poses, t0)
    if p0 is None:
        return None
    x0, y0, th0 = p0
    c, s = math.cos(-th0), math.sin(-th0)
    return [(t - t0, c * (x - x0) - s * (y - y0), s * (x - x0) + c * (y - y0), th - th0) for t, x, y, th in poses]


def cuadros_min(poses, ventana=1.0):
    """Menor cantidad de poses en una ventana de `ventana` s (la cadencia de la visión;
    en FIRASim, cuántos pasos de 16 ms avanzó la simulación en ese segundo)."""
    ts = [p[0] for p in poses]
    j, peor = 0, None
    for i, t in enumerate(ts):
        while ts[j] < t - ventana:
            j += 1
        if t - ts[0] >= ventana:
            n = i - j
            peor = n if peor is None else min(peor, n)
    return peor if peor is not None else len(ts)


def fraccion_saturada(cmds):
    moviendo = [(v, w) for _, v, w in cmds if (v, w) != (0, 0)]
    if not moviendo:
        return 0.0
    ht = sc.FIRMWARE_TRACK_MM / 2.0
    sat = sum(1 for v, w in moviendo if abs(v) + abs(math.radians(w)) * ht > sc.FIRMWARE_MAX_WHEEL_MM_S)
    return sat / len(moviendo)


def regimen_v(cmds, pr, ps):
    """Error relativo de v en régimen por cada consigna constante de v (ω = 0) de más de
    0.5 s y más de 50 mm/s medidos que llega a régimen en la referencia: v media de la
    segunda mitad, real contra sim."""
    errores = []
    for k in range(len(cmds)):
        t, v, w = cmds[k]
        if v == 0 or w != 0 or (k and (cmds[k - 1][1], cmds[k - 1][2]) == (v, w)):
            continue
        j = k
        while j + 1 < len(cmds) and (cmds[j + 1][1], cmds[j + 1][2]) == (v, w):
            j += 1
        t1 = cmds[j + 1][0] if j + 1 < len(cmds) else cmds[j][0]
        if t1 - t < 0.5:
            continue
        d = t1 - t
        mr, ms = v_media(pr, t + d / 2, t1), v_media(ps, t + d / 2, t1)
        if mr is None or ms is None or abs(mr) <= 0.05:
            continue
        # Régimen: en la referencia, el último cuarto anda igual que el tercero. Un
        # escalón corto que no llega (600 mm/s en 0.6 s con 2 m/s²) es transitorio.
        q3, q4 = v_media(pr, t + d / 2, t + 3 * d / 4), v_media(pr, t + 3 * d / 4, t1)
        if q3 is None or q4 is None or abs(q4 - q3) > 0.05 * abs(q4):
            continue
        errores.append(abs(ms - mr) / abs(mr))
    return errores


def v_media(poses, ta, tb):
    """v media entre `ta` y `tb`: el desplazamiento sobre el heading medio dividido por el
    tiempo (mucho menos ruidoso que promediar la derivada)."""
    a, b = sc.interpolar(poses, ta), sc.interpolar(poses, tb)
    if a is None or b is None or tb <= ta:
        return None
    th = (a[2] + b[2]) / 2
    return ((b[0] - a[0]) * math.cos(th) + (b[1] - a[1]) * math.sin(th)) / (tb - ta)


def comparar(dir_real, dir_sim, nombre, parche):
    cr, pr, ir = cargar(dir_real, nombre, parche)
    cs, ps, is_ = cargar(dir_sim, nombre)
    if ir is None or is_ is None:
        return None
    rr, rs = relativa(pr, ir), relativa(ps, is_)
    if not rr or not rs:
        return None
    fin = min(rr[-1][0], rs[-1][0])
    n = int(fin / DT_COMPARACION)
    a, b = [], []
    dist, dth = [], []
    for k in range(n + 1):
        t = k * DT_COMPARACION
        qa, qb = sc.interpolar(rr, t), sc.interpolar(rs, t)
        if qa is None or qb is None:
            continue
        a.append(qa[:2])
        b.append(qb[:2])
        dist.append(math.hypot(qa[0] - qb[0], qa[1] - qb[1]))
        dth.append(abs(math.atan2(math.sin(qa[2] - qb[2]), math.cos(qa[2] - qb[2]))))
    centros = [k * sc.DT for k in range(int(fin / sc.DT))]
    mr, ms = sc.velocidades_medidas(rr, centros), sc.velocidades_medidas(rs, centros)
    dv = [abs(x[0] - y[0]) for x, y in zip(mr, ms) if x and y]
    dw = [abs(x[1] - y[1]) for x, y in zip(mr, ms) if x and y]
    reg = regimen_v([(t - ir, v, w) for t, v, w in cr], rr, rs)
    media = lambda xs: sum(xs) / len(xs) if xs else float("nan")  # noqa: E731
    return {
        "cuadros_min_sim": cuadros_min(ps),
        "mee_m": round(media(dist), 4),
        "dtw_m": round(sc.dtw(a, b), 4),
        "heading_deg": round(math.degrees(media(dth)), 2),
        "v_mm_s": round(1000 * media(dv), 1),
        "w_deg_s": round(math.degrees(media(dw)), 1),
        "v_regimen_max": round(max(reg), 3) if reg else None,
        "v_regimen_medio": round(media(reg), 3) if reg else None,
        "saturada": round(fraccion_saturada(cr), 3),
    }


def veredicto(res):
    fallas = []
    for n in PERFILES_MEE:
        if n in res and res[n]["mee_m"] >= UMBRAL_MEE_M:
            fallas.append(f"{n}: MEE {res[n]['mee_m']:.3f} m ≥ {UMBRAL_MEE_M}")
    if "v_steps" in res and res["v_steps"]["v_regimen_max"] is not None:
        if res["v_steps"]["v_regimen_max"] >= UMBRAL_V_REGIMEN:
            fallas.append(f"v_steps: error de v en régimen {100 * res['v_steps']['v_regimen_max']:.1f} % ≥ {100 * UMBRAL_V_REGIMEN:.0f} %")
    return fallas


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("real", help="sesión del robot (o la referencia)")
    ap.add_argument("sim", help="la misma sesión en el simulador")
    ap.add_argument("--perfiles", nargs="*", help="default: todos los que estén en las dos")
    ap.add_argument("--parche-mm", default="0,0", help="parche de visión del robot (fit.patch_offset_mm del ajuste)")
    ap.add_argument("--json", help="guarda los resultados en este archivo")
    ap.add_argument("--estricto", action="store_true", help="código de salida 1 si no cumple los umbrales")
    a = ap.parse_args()

    dx, dy = (float(x) / 1000.0 for x in a.parche_mm.split(","))
    parche = (dx, dy) if (dx, dy) != (0.0, 0.0) else None
    comunes = sorted(set(sc.perfiles_en(a.real)) & set(sc.perfiles_en(a.sim)))
    nombres = [n for n in comunes if not a.perfiles or n in a.perfiles]
    if not nombres:
        sys.exit("no hay perfiles en común")
    res = {}
    print(f"{'perfil':<9} {'MEE m':>7} {'DTW m':>7} {'θ °':>6} {'v mm/s':>7} {'ω °/s':>6} {'v rég.':>7} {'sat.':>5}")
    for n in nombres:
        r = comparar(a.real, a.sim, n, parche)
        if r is None:
            print(f"{n:<9} sin datos suficientes")
            continue
        res[n] = r
        reg = "—" if r["v_regimen_max"] is None else f"{100 * r['v_regimen_max']:.1f}%"
        print(
            f"{n:<9} {r['mee_m']:>7.4f} {r['dtw_m']:>7.4f} {r['heading_deg']:>6.1f} "
            f"{r['v_mm_s']:>7.1f} {r['w_deg_s']:>6.1f} {reg:>7} {100 * r['saturada']:>4.0f}%"
        )
    atrasos = [f"{n} ({r['cuadros_min_sim']})" for n, r in res.items() if r["cuadros_min_sim"] < CUADROS_MIN]
    if atrasos:
        print(
            f"\n⚠ la visión del simulador bajó de {CUADROS_MIN} cuadros/s en algún segundo: {', '.join(atrasos)}. "
            "FIRASim da un paso de 16 ms por cuadro dibujado: si dibuja más lento, el tiempo simulado se "
            "atrasa contra el reloj del engine y el robot parece más lento. Repetir esos perfiles antes de "
            "sacar conclusiones."
        )
    fallas = veredicto(res)
    print("\n" + ("✓ cumple los umbrales de D6" if not fallas else "✗ no cumple: " + "; ".join(fallas)))
    if a.json:
        with open(a.json, "w") as f:
            json.dump({"perfiles": res, "fallas": fallas}, f, indent=2, ensure_ascii=False)
    sys.exit(1 if fallas and a.estricto else 0)


if __name__ == "__main__":
    main()
