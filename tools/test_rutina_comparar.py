"""Tests de la comparación de corridas de rutinas.

    python3 -m unittest discover -s tools -p 'test_rutina_comparar.py'
"""

import csv
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rutina_comparar as rc  # noqa: E402

COLUMNAS = ["t_ms", "tick", "team", "robot", "own", "pose_x", "pose_y", "rutina", "rutina_fase", "puesto", "repeticion", "paso", "accion"]


def escribir(path, filas):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNAS)
        w.writeheader()
        for fila in filas:
            w.writerow({c: fila.get(c, "") for c in COLUMNAS})


def corrida(dx=0.0, t0_ms=0, robot=0, vueltas=2):
    """Dos vueltas subiendo por x = 0 a 0.2 m/s (2 s cada una), con filas de preparación y
    de un rival que no cuentan."""
    filas = [{"t_ms": t0_ms, "robot": robot, "own": 1, "pose_x": 5, "pose_y": 5, "rutina": "r", "rutina_fase": "preparando", "puesto": "A"}]
    t = t0_ms + 100
    for rep in range(1, vueltas + 1):
        for k in range(126):
            y = -0.2 + 0.4 * k / 125
            filas.append({"t_ms": t, "robot": robot, "own": 1, "pose_x": dx, "pose_y": y, "rutina": "r",
                          "rutina_fase": "ejecutando", "puesto": "A", "repeticion": rep, "paso": 1, "accion": "ir"})
            filas.append({"t_ms": t, "robot": 0, "team": 1, "own": 0, "pose_x": 9, "pose_y": 9})
            t += 16
    return filas


class Comparacion(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.dir.cleanup()

    def ruta(self, nombre, filas):
        p = os.path.join(self.dir.name, nombre)
        escribir(p, filas)
        return p

    def test_misma_corrida_da_cero(self):
        a = self.ruta("a.csv", corrida())
        res = rc.comparar(a, a)
        self.assertEqual(sorted(res["vueltas"]), [1, 2])
        self.assertEqual(res["mee_m"], 0.0)
        self.assertEqual(res["dtw_m"], 0.0)

    def test_desplazada_un_centimetro(self):
        a = self.ruta("a.csv", corrida())
        b = self.ruta("b.csv", corrida(dx=0.01, t0_ms=5000, robot=1))
        res = rc.comparar(a, b)
        self.assertEqual(res["robot_b"], 1)
        for v in res["vueltas"].values():
            self.assertAlmostEqual(v["mee_m"], 0.01, places=4)
            # El DTW alinea pasos vecinos: algo más que el desplazamiento, nunca menos.
            self.assertGreaterEqual(v["dtw_m"], 0.01)
            self.assertLess(v["dtw_m"], 0.0105)

    def test_solo_las_vueltas_en_comun(self):
        a = self.ruta("a.csv", corrida(vueltas=3))
        b = self.ruta("b.csv", corrida(vueltas=2))
        self.assertEqual(sorted(rc.comparar(a, b)["vueltas"]), [1, 2])

    def test_robot_elegido(self):
        filas = corrida(robot=0) + corrida(dx=0.3, robot=2)
        a = self.ruta("a.csv", filas)
        self.assertEqual(rc.comparar(a, a, robot_a=2, robot_b=2)["robot_a"], 2)


if __name__ == "__main__":
    unittest.main()
