"""A1 (Intento 4): mide r y L EFECTIVOS de FIRASim comandando RUEDAS directamente.
Test 1 (recto, wl=wr=w): r = v_medida/w.  Test 2 (giro puro, -w/+w): L = 2*w*r/omega_medida."""
import math, statistics, sys, time
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / "fira_proto"))
import fira_packet_pb2
from firasim_sysid import FiraIO

class WheelIO(FiraIO):
    def send_wheels(self, wl, wr):
        pkt = fira_packet_pb2.Packet()
        c = pkt.cmd.robot_commands.add()
        c.id = self.robot_id; c.yellowteam = self.team_yellow
        c.wheel_left = wl; c.wheel_right = wr
        self.tx.sendto(pkt.SerializeToString(), self.addr)

def park_others(io):
    pkt = fira_packet_pb2.Packet()
    for yellow, rid, x, y in [(False,1,-0.65,0.55),(False,2,-0.65,-0.55),
                              (True,0,0.65,0.55),(True,1,0.65,-0.55),(True,2,0.65,0.0)]:
        r = pkt.replace.robots.add()
        r.position.robot_id = rid; r.position.x = x; r.position.y = y
        r.position.orientation = 0.0; r.yellowteam = yellow; r.turnon = True
    pkt.replace.ball.x = 0.70; pkt.replace.ball.y = 0.58
    pkt.replace.ball.vx = 0.0; pkt.replace.ball.vy = 0.0
    io.tx.sendto(pkt.SerializeToString(), io.addr)

def run_wheels(io, dur, wl_t, wr_t, ramp_t=1.0, hz=60.0):
    out = []; t0 = time.perf_counter(); nxt = t0
    while (t := time.perf_counter() - t0) < dur:
        k = min(1.0, t / ramp_t)
        io.send_wheels(wl_t * k, wr_t * k)
        st = io.latest_state()
        if st is not None:
            if abs(st["x"]) > 0.58 or abs(st["y"]) > 0.55: break
            st["t"] = t; out.append(st)
        nxt += 1.0 / hz
        s = nxt - time.perf_counter()
        if s > 0: time.sleep(s)
    io.send_wheels(0.0, 0.0)
    return out

def med(vals): return statistics.median(vals) if vals else float("nan")

io = WheelIO("127.0.0.1", False, 0, "m")
park_others(io); time.sleep(0.3)

# --- Test 1: radio de rueda (recto) ---
rs = []
for w_cmd in (15.0, 25.0):
    io.teleport(-0.50, 0.0, 0.0); io.send_wheels(0,0); time.sleep(0.6)
    s = run_wheels(io, 2.6, w_cmd, w_cmd)
    v_ss = med([math.hypot(x["vx"], x["vy"]) for x in s if x["t"] >= 1.5])
    r = v_ss / w_cmd
    rs.append(r)
    print(f"[recto] w={w_cmd:5.1f} rad/s -> v_ss={v_ss:.4f} m/s -> r_efectivo={r:.5f} m  (n={len(s)})")
r_meas = statistics.mean(rs)

# --- Test 2: wheelbase (giro puro) ---
Ls = []
for w_cmd in (10.0, 15.0, 20.0):
    io.teleport(0.0, 0.0, 0.0); io.send_wheels(0,0); time.sleep(0.6)
    s = run_wheels(io, 3.2, -w_cmd, +w_cmd)
    win = [x for x in s if x["t"] >= 1.8]
    om_vis = med([abs(x["w"]) for x in win])
    # cross-check: pendiente de theta desenrollado
    th = [x["theta"] for x in win]; ts = [x["t"] for x in win]
    unw = [th[0]]
    for a in th[1:]:
        d = a - unw[-1]
        while d > math.pi: d -= 2*math.pi
        while d < -math.pi: d += 2*math.pi
        unw.append(unw[-1] + d)
    om_slope = abs((unw[-1]-unw[0])/(ts[-1]-ts[0])) if len(ts) > 5 else float("nan")
    om = om_vis if not math.isnan(om_vis) and om_vis > 0.1 else om_slope
    L = 2.0 * w_cmd * r_meas / om
    Ls.append(L)
    print(f"[giro ] w=+-{w_cmd:4.1f} rad/s -> omega_vis={om_vis:.3f} omega_slope={om_slope:.3f} rad/s -> L_efectivo={L:.5f} m  (n={len(s)})")

print()
print(f"== RESULTADO ==")
print(f"r_efectivo  = {r_meas:.5f} m   (commands.rs usa 0.02)")
print(f"L_efectivo  = {statistics.mean(Ls):.5f} m  (mediana {statistics.median(Ls):.5f})")
print(f"  vs commands.rs 0.05 | base_station 0.07 | rsoccer_env 0.075")
