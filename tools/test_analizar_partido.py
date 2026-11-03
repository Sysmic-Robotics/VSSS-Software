import csv
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analizar_partido as ap  # noqa: E402

HEADER = ["t_ms", "tick", "team", "robot", "own", "skill", "pose_x", "pose_y", "ball_x", "ball_y", "ball_vx", "ball_vy",
          "role", "play"]


def frame_rows(tick, play, ball, own, riv):
    t = tick / 60 * 1000
    base = {"t_ms": f"{t:.1f}", "tick": tick, "team": 0, "ball_x": ball[0], "ball_y": ball[1], "ball_vx": ball[2], "ball_vy": ball[3]}
    rows = []
    for rid, (x, y, skill, role) in own.items():
        rows.append({**base, "robot": rid, "own": 1, "skill": skill, "pose_x": x, "pose_y": y, "role": role, "play": play})
    for i, (x, y) in enumerate(riv):
        rows.append({**base, "robot": i, "own": 0, "skill": "", "pose_x": x, "pose_y": y, "role": "", "play": ""})
    return rows


def write_log(rows):
    f = tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False, newline="")
    w = csv.DictWriter(f, fieldnames=HEADER)
    w.writeheader()
    w.writerows(rows)
    f.close()
    return f.name


class AnalizarPartidoTest(unittest.TestCase):
    def test_corner_stuck_kickoff_touch_and_goal_are_reported_in_the_attack_frame(self):
        # Equipo azul atacando hacia +x: arquero (id 2) en x = -0.67. 0-1 s saque propio; 1-6 s
        # pelota trabada en la esquina rival con nuestro atacante y un rival encima; 6-7.5 s la
        # pelota va al centro del área y entra al arco rival.
        rows = []
        keeper = (-0.67, 0.0, "railkeeper", "keeper")
        for tick in range(0, 60):
            rows += frame_rows(tick, "kickoff_ours", (0.70, 0.60, 0, 0),
                               {"0": (0.64, 0.56, "mark", "striker"), "1": (0.2, -0.2, "mark", "support"), "2": keeper},
                               [(0.66, 0.64), (-0.3, 0.3), (0.72, 0.0)])
        for tick in range(60, 360):
            rows += frame_rows(tick, "open", (0.70, 0.60, 0, 0),
                               {"0": (0.64, 0.56, "spinkick", "striker"), "1": (0.2, -0.2, "mark", "support"), "2": keeper},
                               [(0.66, 0.64), (-0.3, 0.3), (0.72, 0.0)])
        for tick in range(360, 450):
            x = 0.50 + (tick - 360) * 0.004
            rows += frame_rows(tick, "open", (x, 0.0, 0.24, 0),
                               {"0": (x - 0.09, 0.0, "dribble", "striker"), "1": (0.2, -0.2, "mark", "support"), "2": keeper},
                               [(0.1, 0.5), (-0.3, 0.3), (0.72, 0.15)])
        for tick in range(450, 480):
            rows += frame_rows(tick, "open", (0.78, 0.0, 0.24, 0),
                               {"0": (0.66, 0.0, "dribble", "striker"), "1": (0.2, -0.2, "mark", "support"), "2": keeper},
                               [(0.1, 0.5), (-0.3, 0.3), (0.72, 0.15)])
        path = write_log(rows)
        try:
            frames = ap.load(path)
            s = ap.attack_sign(frames)
            self.assertEqual(s, 1.0)
            t = ap.trabas(frames, s)
            self.assertEqual(len(t), 1)
            self.assertEqual(t[0]["zona"], "esquina ataque")
            self.assertGreater(t[0]["dur"], 4.5)
            self.assertGreater(t[0]["duelo"], 0.9)
            self.assertEqual(t[0]["skill"].most_common(1)[0][0], "spinkick")
            e = ap.esquinas(frames, s)
            self.assertEqual(len(e), 1)
            # Dos segundos después de salir de la esquina la pelota ya está dentro del arco.
            self.assertEqual(e[0]["fin"], "gol")
            self.assertTrue(e[0]["gol5"])
            r = ap.reanudaciones(frames, s)
            self.assertEqual(len(r), 1)
            self.assertEqual((r[0]["tipo"], r[0]["quien"], r[0]["gol"]), ("kickoff_ours", "propio", "a favor"))
            g = ap.goles(frames, s)
            self.assertEqual([x["lado"] for x in g], ["a favor"])
            self.assertEqual(g[0]["tipo"], "conducido")
            texto = ap.informe([path])
            self.assertIn("esquina ataque", texto)
            self.assertIn("Goles a favor: 1", texto)
            # Visto desde el otro lado (el amarillo que defiende ese arco), el gol es en contra.
            g2 = ap.goles(frames, -1.0)
            self.assertEqual([x["lado"] for x in g2], ["en contra"])
        finally:
            os.unlink(path)

    def test_side_comes_from_the_keeper_or_the_override(self):
        rows = frame_rows(0, "open", (0.0, 0.0, 0, 0), {"2": (0.67, 0.0, "railkeeper", "keeper")}, [])
        path = write_log(rows)
        try:
            frames = ap.load(path)
            self.assertEqual(ap.attack_sign(frames), -1.0)
            self.assertEqual(ap.attack_sign(frames, 1.0), 1.0)
        finally:
            os.unlink(path)


if __name__ == "__main__":
    unittest.main()
