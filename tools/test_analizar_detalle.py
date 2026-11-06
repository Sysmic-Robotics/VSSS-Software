import csv
import gzip
import io
import math
import os
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analizar_detalle as ad  # noqa: E402

HEADER = ["t_ms", "tick", "team", "robot", "own", "skill", "role", "pose_x", "pose_y", "pose_theta", "vel_x", "vel_y",
          "cmd_vx", "cmd_vy", "cmd_omega", "ball_x", "ball_y", "ball_vx", "ball_vy", "face", "nav_reverse", "guard",
          "keeper_mode", "target_x", "target_y", "play"]


def frame_rows(tick, play, ball, own, riv=()):
    """own: id -> dict(x, y, role, skill, th, cvx, cvy, face, tx, ty); riv: [(x, y), ...]."""
    base = {"t_ms": f"{tick / 60 * 1000:.1f}", "tick": tick, "team": 1, "ball_x": ball[0], "ball_y": ball[1],
            "ball_vx": ball[2] if len(ball) > 2 else 0, "ball_vy": ball[3] if len(ball) > 3 else 0}
    rows = []
    for rid, o in own.items():
        rows.append({**base, "robot": rid, "own": 1, "skill": o.get("skill", ""), "role": o.get("role", ""),
                     "pose_x": o["x"], "pose_y": o["y"], "pose_theta": o.get("th", 0.0), "vel_x": 0, "vel_y": 0,
                     "cmd_vx": o.get("cvx", 0.0), "cmd_vy": o.get("cvy", 0.0), "cmd_omega": 0, "face": o.get("face", ""),
                     "target_x": o.get("tx", ""), "target_y": o.get("ty", ""), "play": play})
    for i, (x, y) in enumerate(riv):
        rows.append({**base, "robot": i, "own": 0, "pose_x": x, "pose_y": y, "pose_theta": 0, "play": ""})
    return rows


def write_log(rows, gz=False):
    f = tempfile.NamedTemporaryFile("wb", suffix=".csv.gz" if gz else ".csv", delete=False)
    f.close()
    with (gzip.open(f.name, "wt", newline="") if gz else open(f.name, "w", newline="")) as out:
        w = csv.DictWriter(out, fieldnames=HEADER)
        w.writeheader()
        w.writerows(rows)
    return f.name


KEEPER = {"x": -0.70, "y": 0.0, "role": "keeper", "skill": "railkeeper"}


def goles_rows():
    """Atacamos hacia +x (arquero en -0.70). Un gol a favor conducido que después va y vuelve sobre
    la línea; la pelota puesta a mano dentro del arco y puesta a mano cerca del arco y empujada
    adentro (no son goles); un gol en contra conducido; un gol a favor con la pelota tapada que
    la visión reencuentra adentro."""
    rows, play = [], "open"
    ball_at = {}
    for k in range(0, 60):
        ball_at[k] = (0.0, 0.0)
    for k in range(60, 100):
        ball_at[k] = (0.02 * (k - 60), 0.0)              # cruza 0.76 en el tick 99
    for k in range(100, 160):
        ball_at[k] = (0.80 if k % 2 == 0 else 0.74, 0.0)  # va y vuelve sobre la línea: un solo gol
    for k in range(160, 220):
        ball_at[k] = (0.0, 0.0)                          # al centro (rearma)
    for k in range(220, 280):
        ball_at[k] = (0.80, 0.05)                        # salto de la visión / puesta a mano adentro
    for k in range(280, 340):
        ball_at[k] = (0.0, 0.0)
    for k in range(340, 420):
        ball_at[k] = (max(-0.80, -0.02 * (k - 340)), 0.0)  # gol en contra: cruza -0.76 en el tick 379
    for k in range(420, 480):
        ball_at[k] = (0.0, 0.0)
    for k in range(480, 600):
        ball_at[k] = (min(0.80, 0.65 + 0.02 * (k - 480)), 0.0)  # puesta a mano en 0.65 y empujada adentro
    for k in range(600, 640):
        ball_at[k] = (0.0, 0.0)
    for k in range(640, 700):
        ball_at[k] = (0.62 * (k - 640) / 59, 0.0)
    for k in range(700, 718):
        ball_at[k] = (0.62, 0.0)                         # la visión la pierde (tapada): queda quieta
    for k in range(718, 760):
        ball_at[k] = (0.80, 0.0)                         # y reaparece adentro (salto de 18 cm): gol
    for k in range(760):
        if k == 130:
            play = "hold"
        elif k == 160:
            play = "kickoff_theirs"
        elif k == 200:
            play = "hold"
        elif k == 210:
            play = "open"
        bx, by = ball_at[k]
        rows += frame_rows(k, play, (bx, by, 0.0, 0.0),
                           {0: {"x": bx - 0.08, "y": by, "role": "striker", "skill": "dribble"}, 2: KEEPER},
                           [(0.5, 0.5)])
    return rows


class GolesTest(unittest.TestCase):
    def test_a_goal_counts_once_and_teleports_into_the_goal_do_not_count(self):
        path = write_log(goles_rows())
        try:
            p = ad.cargar(path)
            self.assertEqual(p.s, 1.0)
            self.assertEqual([(g["lado"], round(g["t"] * 60)) for g in p.goles],
                             [("favor", 99), ("contra", 379), ("favor", 718)])
            # La orden siguiente es la primera reanudación que empieza después del gol (no el hold).
            self.assertEqual(p.goles[0]["siguiente"], "kickoff_theirs")
            info = ad.analizar_gol(p, p.goles[0])
            self.assertEqual(info["anota"], ("striker", "dribble"))
            # El saque rival pasa por hold antes del open (árbitro a mano) y cuenta igual.
            s = ad.saques(p)
            self.assertEqual([(x["tipo"], round(x["t"] * 60)) for x in s], [("kickoff_theirs", 210)])
        finally:
            os.unlink(path)

    def test_report_reads_gzip_and_picks_sections(self):
        path = write_log(goles_rows(), gz=True)
        try:
            texto = ad.informe([path], solo=["goles"])
            self.assertIn("Goles a favor: 2", texto)
            self.assertIn("Goles en contra: 1", texto)
            self.assertNotIn("Caras", texto)
            out = io.StringIO()
            with redirect_stdout(out):
                ad.main([path, "--solo", "saques,casi_gol"])
            self.assertIn("Saques rivales: 1", out.getvalue())
            self.assertIn("Casi gol", out.getvalue())
            self.assertNotIn("Goles a favor", out.getvalue())
            with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                ad.main([path, "--solo", "penales"])
        finally:
            os.unlink(path)


class MarcoTest(unittest.TestCase):
    def test_the_field_is_turned_when_our_keeper_lives_on_plus_x(self):
        # Arquero en +0.70: atacamos hacia -x. La pelota entra al arco de -x: es gol a favor.
        rows = []
        for k in range(120):
            bx = max(-0.80, -0.02 * k)
            rows += frame_rows(k, "open", (bx, 0.05, -1.2, 0.0),
                               {0: {"x": 0.2, "y": 0.1, "th": 0.3, "cvx": 0.2, "cvy": 0.0, "tx": 0.5, "ty": 0.2,
                                    "role": "striker", "skill": "dribble"},
                                2: {"x": 0.70, "y": 0.0, "role": "keeper", "skill": "railkeeper"}},
                               [(0.4, -0.2)])
        path = write_log(rows)
        try:
            p = ad.cargar(path)
            self.assertEqual(p.s, -1.0)
            f0 = p.cuadros[0]
            r = f0.own[0]
            self.assertAlmostEqual(r.x, -0.2)
            self.assertAlmostEqual(r.y, -0.1)
            self.assertAlmostEqual(r.th, 0.3 - math.pi)
            self.assertAlmostEqual(r.cvx, -0.2)
            self.assertAlmostEqual(r.tx, -0.5)
            self.assertAlmostEqual(r.ty, -0.2)
            # El avance a lo largo del frente no cambia al girar la cancha.
            self.assertAlmostEqual(r.cvx * math.cos(r.th) + r.cvy * math.sin(r.th), 0.2 * math.cos(0.3))
            self.assertAlmostEqual(f0.riv[0].x, -0.4)
            self.assertAlmostEqual(f0.b[2], 1.2)
            self.assertAlmostEqual(f0.own[2].x, -0.70)
            self.assertEqual([g["lado"] for g in p.goles], ["favor"])
            # Con --lado se fuerza: visto como si atacáramos hacia +x, el mismo gol es en contra.
            p2 = ad.cargar(path, lado=1.0)
            self.assertEqual(p2.s, 1.0)
            self.assertEqual([g["lado"] for g in p2.goles], ["contra"])
        finally:
            os.unlink(path)


class CarasTest(unittest.TestCase):
    def test_reversals_of_the_commanded_advance_and_face_changes(self):
        # Atacante: adelante 1 s, atrás 1/6 s (ida y vuelta), adelante 1 s, quieto, adelante mirando
        # a +y (el comando va en y: sigue siendo adelante), atrás 1 s; hold (se reinicia) y adelante.
        # El arquero alterna el comando en cada tick y no cuenta.
        tramos = [(0, 60, 0.0, (0.2, 0.0), "front"), (60, 70, 0.0, (-0.2, 0.0), "back"),
                  (70, 130, 0.0, (0.2, 0.0), "front"), (130, 140, 0.0, (0.0, 0.0), "front"),
                  (140, 200, math.pi / 2, (0.0, 0.2), "front"), (200, 260, 0.0, (-0.2, 0.0), "back"),
                  (260, 300, 0.0, (0.2, 0.0), "front"), (300, 360, 0.0, (0.2, 0.0), "front")]
        rows = []
        for a, b, th, (cvx, cvy), face in tramos:
            for k in range(a, b):
                play = "hold" if 260 <= k < 300 else "open"
                rows += frame_rows(k, play, (0.5, 0.5, 0.0, 0.0),
                                   {0: {"x": 0.0, "y": 0.0, "th": th, "cvx": cvx, "cvy": cvy, "face": face,
                                        "role": "striker", "skill": "dribble"},
                                    2: {**KEEPER, "cvx": 0.2 if k % 2 else -0.2, "face": "front" if k % 2 else "back"}})
        path = write_log(rows)
        try:
            p = ad.cargar(path)
            m = ad.medir_caras(p)
            self.assertEqual((m["inv"], m["ida"]), (3, 1))
            self.assertAlmostEqual(m["min"], 320 / 3600)
            self.assertEqual(dict(m["cambios"]), {"dribble": 3})
            self.assertEqual(dict(m["cambio_ida"]), {"dribble": 1})
            texto = "\n".join(ad.texto_caras(m))
            self.assertIn(f"{3 / (320 / 3600):.0f} por minuto", texto)
            self.assertIn("33 % ida y vuelta", texto)
        finally:
            os.unlink(path)


if __name__ == "__main__":
    unittest.main()
