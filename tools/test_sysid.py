"""Tests del ajuste y la validación de la sysid.

    python3 -m unittest discover -s tools -p 'test_sysid.py'

El modelo de Python tiene que ser el de `src/radio/actuator.rs`: los primeros tests
repiten los números de sus tests de Rust. La equivalencia completa con la planta de
Rust (mismos perfiles, sin ruido) la verifica `tools/sysid_ensayo.sh`.
"""

import math
import os
import random
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sysid_ajuste as aj  # noqa: E402
import sysid_comun as sc  # noqa: E402
import sysid_validacion as va  # noqa: E402

DT = sc.DT


def modelo(**kw):
    m = dict(aj.MODELO_SINTETICO, latency_ms=100.0, tau_v_s=0.2, tau_w_s=0.1, max_accel_m_s2=100.0, max_alpha_rad_s2=1000.0)
    m.update(kw)
    return m


class ModeloComoRust(unittest.TestCase):
    def test_retardo_y_primer_orden_63_por_ciento_a_tau(self):
        out = sc.simular(modelo(), [(300, 0)] * 60)
        self.assertTrue(all(v == 0.0 for v, _ in out[:6]))
        self.assertGreater(out[6][0], 0.0)
        self.assertAlmostEqual(out[17][0] / 0.3, 1 - math.exp(-1), places=6)

    def test_tope_por_rueda_escala_las_dos(self):
        v, w = sc.simular(modelo(), [(600, 0)] * 300)[-1]
        self.assertAlmostEqual(v, 0.45, places=6)
        sv, sw = sc.setpoint_firmware(modelo(), 400, 360)
        self.assertAlmostEqual(sv + sw * 0.0375, 0.45, places=9)
        self.assertAlmostEqual(sw / sv, math.radians(360) / 0.4, places=9)

    def test_zona_muerta(self):
        self.assertTrue(all(o == (0.0, 0.0) for o in sc.simular(modelo(), [(15, 0)] * 120)))
        self.assertTrue(all(o[1] == 0.0 for o in sc.simular(modelo(), [(0, 25)] * 120)))
        self.assertGreater(sc.simular(modelo(), [(0, 90)] * 120)[-1][1], 1.5)

    def test_aceleracion_maxima(self):
        out = sc.simular(modelo(latency_ms=0.0, max_accel_m_s2=1.0), [(400, 0)] * 30)
        self.assertAlmostEqual(out[0][0], DT, places=12)
        for a, b in zip(out, out[1:]):
            self.assertLessEqual(b[0] - a[0], DT + 1e-12)

    def test_ganancias(self):
        v, w = sc.simular(modelo(gain_v=0.9, gain_w=1.1), [(300, 90)] * 300)[-1]
        self.assertAlmostEqual(v, 0.27, places=6)
        self.assertAlmostEqual(w, 1.1 * math.radians(90), places=6)


class Derivador(unittest.TestCase):
    def test_movimiento_uniforme_da_la_velocidad_exacta(self):
        poses = [(k * DT, 0.3 * k * DT * math.cos(0.5), 0.3 * k * DT * math.sin(0.5), 0.5 + 2.0 * k * DT) for k in range(100)]
        for med in sc.velocidades_medidas(poses, [0.5, 0.8, 1.0]):
            # Solo ω: el heading gira a 2 rad/s.
            self.assertIsNotNone(med)
            self.assertAlmostEqual(med[1], 2.0, places=6)
        recta = [(k * DT, 0.3 * k * DT, 0.0, 0.0) for k in range(100)]
        v, w = sc.velocidades_medidas(recta, [0.7])[0]
        self.assertAlmostEqual(v, 0.3, places=9)
        self.assertAlmostEqual(w, 0.0, places=9)

    def test_sin_poses_suficientes_es_none(self):
        self.assertIsNone(sc.velocidades_medidas([(0.0, 0, 0, 0), (1.0, 0, 0, 0)], [0.5])[0])

    def test_nucleo_suma_uno_y_equivale_al_derivador(self):
        kern = sc.nucleo()
        self.assertAlmostEqual(sum(w for _, w in kern), 1.0)
        # Velocidad arbitraria → poses en la grilla → derivador == núcleo sobre la v.
        rng = random.Random(3)
        vs = [rng.uniform(-0.5, 0.5) for _ in range(80)]
        poses = [(k * DT, x, 0.0, 0.0) for k, (x, _, _) in enumerate(sc.integrar([(v, 0.0) for v in vs]))]
        medido = sc.velocidades_medidas(poses, [k * DT + DT / 2 for k in range(80)])
        suave = sc.suavizar(vs, kern)
        for k in range(5, 75):
            self.assertAlmostEqual(medido[k][0], suave[k], places=9)

    def test_parche_de_vision(self):
        # Giro puro con el parche 10 mm adelante y 4 mm a la derecha del eje.
        dx, dy = 0.010, -0.004
        cmds = [(0.0, 0, 0), (0.5, 0, 360)]
        poses = []
        for k in range(60, 120):
            th = (k - 60) * DT * 2 * math.pi
            poses.append((k * DT, 0.2 + dx * math.cos(th) - dy * math.sin(th), -0.1 + dx * math.sin(th) + dy * math.cos(th), th))
        px, py = sc.estimar_parche([(poses, cmds)])
        self.assertAlmostEqual(px, dx, places=6)
        self.assertAlmostEqual(py, dy, places=6)
        eje = sc.corregir_parche(poses, (px, py))
        self.assertTrue(all(abs(x - 0.2) < 1e-6 and abs(y + 0.1) < 1e-6 for _, x, y, _ in eje))

    def test_segmentos_separados_por_pausas(self):
        cmds = [(0, 0)] * 60 + [(100, 0)] * 30 + [(0, 0)] * 2 + [(-50, 0)] * 10 + [(0, 0)] * 70 + [(0, 90)] * 20 + [(0, 0)] * 60
        self.assertEqual(sc.segmentos(cmds, cola=0), [(60, 102), (172, 192)])


class Validacion(unittest.TestCase):
    def escribir(self, d, nombre, cmds, poses):
        with open(os.path.join(d, nombre + ".csv"), "w") as f:
            f.write("t_ms,v_mm_s,w_deg_s\n")
            for t, v, w in cmds:
                f.write(f"{round(t * 1000)},{v},{w}\n")
        with open(os.path.join(d, nombre + ".pose.csv"), "w") as f:
            f.write("t_ms,x,y,theta\n")
            for t, x, y, th in poses:
                f.write(f"{t * 1000:.1f},{x:.6f},{y:.6f},{th:.6f}\n")

    def sesion(self, d, m, ruido=0.0, semilla=1):
        rng = random.Random(semilla)
        # Un escalón de 2 s: con τ = 0.2 s llega a régimen (los cortos no cuentan).
        cmds = [(k * 0.05, 0, 0) for k in range(20)] + [(1 + k * 0.05, 300, 0) for k in range(40)] + [(3 + k * 0.05, 0, 0) for k in range(30)]
        tiempos, grid = sc.grilla(cmds, 0.0, 4.5)
        poses = [(t, x + rng.gauss(0, ruido), y + rng.gauss(0, ruido), th) for t, (x, y, th) in zip(tiempos, sc.integrar(sc.simular(m, grid)))]
        self.escribir(d, "v_steps", cmds, poses)

    def test_trayectorias_iguales_dan_cero(self):
        self.assertEqual(sc.dtw([(0, 0), (1, 1)], [(0, 0), (1, 1)]), 0.0)
        with tempfile.TemporaryDirectory() as d:
            self.sesion(d, modelo())
            r = va.comparar(d, d, "v_steps", None)
            self.assertEqual((r["mee_m"], r["dtw_m"], r["v_mm_s"]), (0.0, 0.0, 0.0))
            self.assertEqual(r["v_regimen_max"], 0.0)

    def test_un_escalon_que_no_llega_a_regimen_no_cuenta(self):
        with tempfile.TemporaryDirectory() as a, tempfile.TemporaryDirectory() as b:
            self.sesion(a, modelo(tau_v_s=1.0))
            self.sesion(b, modelo(tau_v_s=0.5))
            self.assertIsNone(va.comparar(a, b, "v_steps", None)["v_regimen_max"])

    def test_cuadros_por_segundo(self):
        poses = [(k / 60, 0, 0, 0) for k in range(120)] + [(2 + k / 20, 0, 0, 0) for k in range(40)]
        self.assertEqual(va.cuadros_min(poses), 20)

    def test_un_modelo_distinto_se_nota(self):
        with tempfile.TemporaryDirectory() as a, tempfile.TemporaryDirectory() as b:
            self.sesion(a, modelo())
            self.sesion(b, modelo(gain_v=0.8))
            r = va.comparar(a, b, "v_steps", None)
            self.assertGreater(r["v_regimen_max"], 0.15)
            self.assertGreater(r["mee_m"], 0.01)
            self.assertTrue(va.veredicto({"v_steps": r}))


def sesion_corta(m, semilla):
    """Comandos a 20 Hz con escalones, rampa lenta, barridos y arcos saturados (no son los
    perfiles oficiales: el ensayo usa esos) y las poses del modelo con el ruido del proxy."""
    cmds, t = [], 0.0

    def tramo(segs, f):
        nonlocal t
        for k in range(int(round(segs / 0.05))):
            cmds.append((t, *f((k + 0.5) * 0.05)))
            t += 0.05

    def pausa():
        tramo(1.0, lambda _: (0, 0))

    pausa()
    for v in (100, 300, 450, 600):
        for s in (1, -1):
            tramo(min(1.5, 0.36 / (v / 1000)), lambda _, v=v, s=s: (s * v, 0))
            pausa()
    for w in (180, 720):
        for s in (1, -1):
            tramo(1.0, lambda _, w=w, s=s: (0, s * w))
            pausa()
    for s in (1, -1):
        tramo(8.0, lambda x, s=s: (round(s * 60 * (1 - abs(x - 4) / 4)), 0))
        pausa()
        tramo(6.0, lambda x, s=s: (round(s * 250 * math.sin(2 * math.pi * (0.3 * x + 0.2 * x * x))), 0))
        pausa()
        tramo(6.0, lambda x, s=s: (0, round(s * 360 * math.sin(2 * math.pi * (0.3 * x + 0.2 * x * x)))))
        pausa()
    for v, w in ((400, 360), (300, -540)):
        for s in (1, -1):
            tramo(1.0, lambda _, v=v, w=w, s=s: (s * v, s * w))
            pausa()
    rng = random.Random(semilla)
    tiempos, grid = sc.grilla(cmds, 0.0, t + 0.5)
    poses = []
    for tt, (x, y, th) in zip(tiempos, sc.integrar(sc.simular(m, grid))):
        if rng.random() < 0.02:
            continue
        poses.append((tt, x + rng.gauss(0, 0.00185), y + rng.gauss(0, 0.00185), th + rng.gauss(0, 0.031)))
    return cmds, sc.desenvolver(poses)


class Recupera(unittest.TestCase):
    def test_recupera_el_modelo_de_la_spec(self):
        m = dict(aj.MODELO_SINTETICO)
        cmds, poses = sesion_corta(m, semilla=11)
        prob = aj.Problema([aj.Perfil("corta", cmds, poses)])
        ajustado, _ = aj.ajustar(prob, aj.INICIAL, barridos=3)
        self.assertEqual(aj.comparar(ajustado, m, 0.15), 0)


if __name__ == "__main__":
    unittest.main()
