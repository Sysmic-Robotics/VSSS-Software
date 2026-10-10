"""Lectura de los CSV de skill_test/scenario, nuevos (frame_str entre comillas) y viejos.

    python3 -m unittest discover -s tools -p 'test_csv.py'
"""

import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plot_run  # noqa: E402
import sysid_comun as sc  # noqa: E402

HEADER = (
    "t_ms,tick,mode,transport,vision,robot,team,skill,pose_x,pose_y,pose_theta,target_x,target_y,"
    "cmd_vx,cmd_vy,cmd_omega,v_mm_s,w_deg_s,frame_str,err_dist,err_heading,ball_x,ball_y,ball_vx,ball_vy"
)
# La misma fila con la base, como la escribe CsvLogger hoy y como la escribía antes.
NUEVA = '50,3,skill,base-station,real,1,blue,goto,0.1,0.2,0.3,0.4,0.5,0.6,0,0,300,-45,"0,0,300,-45,0,0,0,0,0,0",0.25,0.05,0.7,-0.2,0,0'
VIEJA = "50,3,skill,base-station,real,1,blue,goto,0.1,0.2,0.3,0.4,0.5,0.6,0,0,300,-45,0,0,300,-45,0,0,0,0,0,0,0.25,0.05,0.7,-0.2,0,0"
SIM = "50,3,skill,firasim,sim,1,blue,goto,0.1,0.2,0.3,0.4,0.5,0.6,0,0,300,-45,,0.25,0.05,0.7,-0.2,0,0"


def archivo(d, nombre, fila):
    path = os.path.join(d, nombre)
    with open(path, "w") as f:
        f.write(HEADER + "\n" + fila + "\n")
    return path


class LecturaCsv(unittest.TestCase):
    def test_plot_run_lee_igual_el_nuevo_y_el_viejo(self):
        with tempfile.TemporaryDirectory() as d:
            nueva = plot_run.load(archivo(d, "nueva.csv", NUEVA))[0]
            vieja = plot_run.load(archivo(d, "vieja.csv", VIEJA))[0]
            self.assertEqual(nueva, vieja)
            self.assertEqual(nueva["frame_str"], "0,0,300,-45,0,0,0,0,0,0")
            self.assertEqual((nueva["err_dist"], nueva["ball_x"], nueva["ball_vy"]), ("0.25", "0.7", "0"))
            self.assertNotIn(None, nueva)
            sim = plot_run.load(archivo(d, "sim.csv", SIM))[0]
            self.assertEqual((sim["frame_str"], sim["err_dist"]), ("", "0.25"))

    def test_sysid_lee_igual_el_nuevo_y_el_viejo(self):
        with tempfile.TemporaryDirectory() as d:
            esperado = [(0.05, 300, -45)]
            self.assertEqual(sc.leer_comandos(archivo(d, "nueva.csv", NUEVA)), esperado)
            self.assertEqual(sc.leer_comandos(archivo(d, "vieja.csv", VIEJA)), esperado)


if __name__ == "__main__":
    unittest.main()
