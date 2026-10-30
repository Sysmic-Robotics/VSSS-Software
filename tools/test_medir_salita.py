import csv
import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import medir_salita  # noqa: E402

HEADER = ["t_ms", "tick", "team", "robot", "own", "skill", "pose_x", "pose_y", "vel_x", "vel_y", "omega",
          "ball_x", "ball_y"]


def write_log(rows):
    f = tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False, newline="")
    w = csv.DictWriter(f, fieldnames=HEADER)
    w.writeheader()
    w.writerows(rows)
    f.close()
    return f.name


class MedirSalitaTest(unittest.TestCase):
    def test_free_rolling_ball_recovers_the_exponential_decay(self):
        # Pelota lanzada a 0.6 m/s con k = 0.5 1/s, robots lejos; un robot propio acelera a 1 m/s²
        # hasta 0.45 m/s.
        rows, x, v = [], -0.5, 0.6
        for i in range(120):
            t = i / 60
            x += v / 60
            v *= math.exp(-0.5 / 60)
            rv = min(0.45, 1.0 * t)
            base = {"t_ms": f"{t * 1000:.1f}", "tick": i, "team": 0, "skill": "dribble", "ball_x": x, "ball_y": 0.0}
            rows.append({**base, "robot": 0, "own": 1, "pose_x": 0.0, "pose_y": 0.4, "vel_x": rv, "vel_y": 0, "omega": 0})
            rows.append({**base, "robot": 0, "own": 0, "pose_x": 0.0, "pose_y": -0.4, "vel_x": "", "vel_y": "", "omega": ""})
        path = write_log(rows)
        try:
            ticks = medir_salita.load(path)
            segs = medir_salita.free_ball_segments(ticks)
            self.assertEqual(len(segs), 1)
            self.assertAlmostEqual(segs[0]["k"], 0.5, delta=0.05)
            self.assertGreaterEqual(segs[0]["r2_exp"], segs[0]["r2_lin"])
            dyn = medir_salita.robot_dynamics(ticks)
            self.assertAlmostEqual(max(dyn["v"]), 0.45, delta=1e-6)
            self.assertAlmostEqual(max(dyn["a"]), 1.0, delta=0.15)
            self.assertIn("k = 0.50", medir_salita.report([path]))
        finally:
            os.unlink(path)

    def test_ball_next_to_a_robot_is_not_free(self):
        rows = []
        for i in range(60):
            t = i / 60
            base = {"t_ms": f"{t * 1000:.1f}", "tick": i, "team": 0, "skill": "dribble",
                    "ball_x": 0.3 * t, "ball_y": 0.0}
            rows.append({**base, "robot": 0, "own": 1, "pose_x": 0.3 * t - 0.06, "pose_y": 0.0,
                         "vel_x": 0.3, "vel_y": 0, "omega": 0})
        path = write_log(rows)
        try:
            self.assertEqual(medir_salita.free_ball_segments(medir_salita.load(path)), [])
        finally:
            os.unlink(path)


if __name__ == "__main__":
    unittest.main()
