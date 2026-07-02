"""A1b v2: mide r y L EFECTIVOS de rSim (rSoccer) — posiciones iniciales FIJAS."""
import os
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
import math, statistics, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rsoccer_gym.Entities import Robot, Ball, Frame
from rsoccer_gym.vss.env_vss.vss_gym import VSSEnv

class RawEnv(VSSEnv):
    def __init__(self):
        super().__init__()
        self.wl = 0.0; self.wr = 0.0
        self.start_x = 0.0
    def _get_initial_positions_frame(self):
        f = Frame()
        f.ball = Ball(x=0.70, y=0.58)
        f.robots_blue = {0: Robot(x=self.start_x, y=0.0, theta=0.0),
                         1: Robot(x=-0.65, y=0.55, theta=0.0),
                         2: Robot(x=-0.65, y=-0.55, theta=0.0)}
        f.robots_yellow = {0: Robot(x=0.65, y=0.55, theta=0.0),
                           1: Robot(x=0.65, y=-0.55, theta=0.0),
                           2: Robot(x=0.65, y=0.0, theta=0.0)}
        return f
    def _get_commands(self, action):
        cmds = [Robot(yellow=False, id=0, v_wheel0=self.wl, v_wheel1=self.wr)]
        for i in range(1, 3): cmds.append(Robot(yellow=False, id=i, v_wheel0=0, v_wheel1=0))
        for i in range(3): cmds.append(Robot(yellow=True, id=i, v_wheel0=0, v_wheel1=0))
        return cmds
    def _frame_to_observations(self): return [0.0]
    def _calculate_reward_and_done(self): return 0.0, False

env = RawEnv()
r_spec = env.field.rbt_wheel_radius
print(f"specs rSoccer: rbt_wheel_radius={r_spec}  rbt_radius={env.field.rbt_radius}")
DT = 0.025

def run(wl, wr, start_x=0.0, steps=100, settle=30):
    env.start_x = start_x
    env.reset()
    env.wl, env.wr = wl, wr
    vs, oms = [], []
    prev_th = None; unw = 0.0; unw0 = None; t0 = None
    for k in range(steps):
        env.step([0.0])
        rb = env.frame.robots_blue[0]
        th = math.radians(rb.theta)
        if prev_th is not None:
            d = th - prev_th
            while d > math.pi: d -= 2*math.pi
            while d < -math.pi: d += 2*math.pi
            unw += d
        prev_th = th
        if k >= settle:
            if t0 is None: t0 = k*DT; unw0 = unw
            vs.append(math.hypot(rb.v_x, rb.v_y))
            oms.append(abs(math.radians(rb.v_theta)))
    om_slope = abs(unw-unw0)/((steps-1)*DT - t0)
    return statistics.median(vs), statistics.median(oms), om_slope, env.frame.robots_blue[0].x

# radio efectivo: recto (desde x=-0.5, 100 pasos=2.5s, settle 0.75s)
rs = []
for w in (10.0, 15.0):
    v, _, _, xf = run(w, w, start_x=-0.5, steps=80, settle=30)
    rs.append(v/w)
    print(f"[recto] w={w:5.1f} -> v_ss={v:.4f} -> r_efectivo={v/w:.5f}  (x final {xf:+.2f})")
r_meas = statistics.mean(rs)

# wheelbase efectivo: giro puro en el centro
Ls = []
for w in (10.0, 15.0, 20.0):
    _, om_v, om_s, _ = run(-w, w, start_x=0.0, steps=120, settle=40)
    om = om_v if om_v > 0.1 else om_s
    L = 2*w*r_meas/om
    Ls.append(L)
    print(f"[giro ] w=+-{w:4.1f} -> omega_vtheta={om_v:.3f} omega_slope={om_s:.3f} -> L_efectivo={L:.5f}")

print()
print(f"== RESULTADO rSim ==")
print(f"r_efectivo = {r_meas:.5f} (spec {r_spec})  |  L_efectivo = {statistics.mean(Ls):.5f} (mediana {statistics.median(Ls):.5f})")
print(f"wrapper actual usa WHEELBASE_L=0.075 y r=spec({r_spec})  |  FIRASim medido: r=0.02000, L=0.08499")
