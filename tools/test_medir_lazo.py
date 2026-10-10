"""Tests de la medición del lazo.

    python3 -m unittest discover -s tools -p 'test_medir_lazo.py'
"""

import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import medir_lazo as ml  # noqa: E402

HEADER = "t_ms,tick,team,robot,own,skill,target_x,target_y,pose_x,pose_y,pose_theta,vel_x,vel_y,omega,cmd_vx,cmd_vy,cmd_omega,ball_x,ball_y,ball_vx,ball_vy\n"


def escribir(filas_por_tick):
    """CSV de registro de partido con `filas` filas por cada `(tick, t_ms)`."""
    f = tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False)
    f.write(HEADER)
    for tick, t_ms, filas in filas_por_tick:
        for robot in range(filas):
            f.write(f"{t_ms},{tick},0,{robot},1,GoTo,,,0,0,0,0,0,0,0,0,0,0,0,0,0\n")
    f.close()
    return f.name


def ticks_con(intervalos, filas=1):
    t = 0
    out = [(0, 0, filas)]
    for k, dt in enumerate(intervalos, start=1):
        t += dt
        out.append((k, t, filas))
    return out


class Medicion(unittest.TestCase):
    def tearDown(self):
        for p in getattr(self, "_paths", []):
            os.unlink(p)

    def csv(self, filas_por_tick):
        p = escribir(filas_por_tick)
        self._paths = getattr(self, "_paths", []) + [p]
        return p

    def test_un_tick_tarde(self):
        r = ml.medir(self.csv(ticks_con([16] * 99 + [40])))
        self.assertEqual(r["ticks"], 101)
        self.assertAlmostEqual(r["hz"], 100 / 1.624, places=6)
        self.assertEqual(round(r["hz"], 1), 61.6)
        self.assertEqual(r["max"], 40)
        self.assertAlmostEqual(r["pct_20"], 1.0)
        self.assertAlmostEqual(r["pct_33"], 1.0)
        self.assertEqual(r["p50"], 16)

    def test_varias_filas_por_tick_cuentan_una_vez(self):
        r = ml.medir(self.csv(ticks_con([16] * 50, filas=6)))
        self.assertEqual(r["ticks"], 51)
        self.assertAlmostEqual(r["hz"], 62.5)

    def test_tick_faltante_no_inventa_intervalo(self):
        filas = ticks_con([16] * 10)
        del filas[5]  # el tick 5 no escribió filas
        r = ml.medir(self.csv(filas))
        self.assertEqual(r["faltan"], 1)
        self.assertEqual(r["max"], 16)

    def test_criterio(self):
        bueno = ml.resumen([16] * 1000)
        self.assertEqual(ml.cumple(bueno), [])
        lento = ml.resumen([17] * 1000)
        self.assertTrue(any("media" in f for f in ml.cumple(lento)))
        tirones = ml.resumen([16] * 990 + [40] * 10)
        self.assertTrue(any("> 33 ms" in f for f in ml.cumple(tirones)))
        base = ml.resumen([15.9] * 1000)
        peor = ml.resumen([16.0] * 990 + [21.0] * 10)
        self.assertTrue(any("base" in f for f in ml.cumple(peor, base)))

    def test_percentil_interpolado(self):
        self.assertEqual(ml.percentil([1, 2, 3, 4], 50), 2.5)
        self.assertEqual(ml.percentil([5], 99), 5)


if __name__ == "__main__":
    unittest.main()
