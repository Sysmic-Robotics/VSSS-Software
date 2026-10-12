"""Tests de tools/match_metrics.py con el CSV de partido que trae las columnas de
depuración al final (capability depuracion-tick): los resultados no cambian.

    python3 -m unittest discover -s tools -p 'test_match_metrics.py'
"""
import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import match_metrics  # noqa: E402

BASE = ("t_ms,tick,team,robot,own,skill,target_x,target_y,pose_x,pose_y,pose_theta,vel_x,vel_y,"
        "omega,cmd_vx,cmd_vy,cmd_omega,ball_x,ball_y,ball_vx,ball_vy")
DEPURACION = ("role,progress,feasible,face,nav_dest_x,nav_dest_y,nav_aim_x,nav_aim_y,nav_detour,nav_side,"
              "nav_reverse,guard,guard_proj_x,guard_proj_y,escape_dx,escape_dy,arrival_s,coach_decision,"
              "striker,score_shoot,score_clear,score_spin,score_intercept,score_approach,pick_score,pick,"
              "bonus_skill,commit_left,stall,stall_s,adapt_shot_upper,adapt_shot_center,adapt_shot_lower,"
              "adapt_duel")


def filas(con_depuracion: bool):
    """Un partido corto con guion: la pelota va y viene, dos propios y un rival."""
    out = [BASE + ("," + DEPURACION if con_depuracion else "")]
    extra_propio = ",striker,0.500,1,front,0.100,0.050,0.050,0.120,ball,1,0,brake,0.200,0.000,,,0.800,1,0,0.4,0,0,0,0.27,approach,approach,approach,2,none,,0.500,0.500,0.500,0.500"
    extra_rival = "," * 34
    for tick in range(240):
        t = tick * 16
        s = tick / 60.0
        bx, by = 0.5 * math.sin(s * 0.8), 0.3 * math.cos(s * 0.5)
        bvx, bvy = 0.4 * math.cos(s * 0.8), -0.15 * math.sin(s * 0.5)
        pelota = f"{bx:.4f},{by:.4f},{bvx:.4f},{bvy:.4f}"
        for rid, (px, py, skill) in enumerate([(bx - 0.08, by, "approach"), (-0.65, 0.0, "goalkeep")]):
            fila = f"{t},{tick},0,{rid},1,{skill},0.75,0,{px:.4f},{py:.4f},0,0.1,0,0,0.2,0,0,{pelota}"
            out.append(fila + (extra_propio if con_depuracion else ""))
        fila = f"{t},{tick},1,0,0,,,,{bx + 0.1:.4f},{by:.4f},3.14,0,0,0,,,,{pelota}"
        out.append(fila + (extra_rival if con_depuracion else ""))
    return "\n".join(out) + "\n"


class TestDebugColumns(unittest.TestCase):
    def test_results_do_not_change_with_the_debug_columns(self):
        with tempfile.TemporaryDirectory() as d:
            sin = os.path.join(d, "sin.csv")
            con = os.path.join(d, "con.csv")
            with open(sin, "w") as f:
                f.write(filas(False))
            with open(con, "w") as f:
                f.write(filas(True))
            a = match_metrics.analyze(sin)
            b = match_metrics.analyze(con)
            self.assertNotIn("error", a)
            a.pop("archivo")
            b.pop("archivo")
            self.assertEqual(a, b)


if __name__ == "__main__":
    unittest.main()
