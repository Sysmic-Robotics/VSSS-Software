"""Eval nativo en rSoccer del modelo (ONNX) 20.5M: goles/reward por episodio DENTRO
de rSoccer (mismo setup de entrenamiento: azul 0,1 = politica; azul 2 = arquero regla;
amarillo = rule-based). Decisivo: mide si el modelo aprendio a jugar en su propio sim."""
import os
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
import sys, numpy as np
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import onnxruntime as ort
from vsss_rl.rsoccer_env import make_field_env

MODEL = os.environ.get("EVAL_MODEL", str(Path(__file__).resolve().parent / "models" / "policy_field.onnx"))
N = int(os.environ.get("EVAL_EPISODES", "50"))
sess = ort.InferenceSession(MODEL, providers=["CPUExecutionProvider"])
iname = sess.get_inputs()[0].name
env = make_field_env()
goals = conceded = draws = 0
rewards, steps = [], []
for ep in range(N):
    obs, _ = env.reset(seed=1000 + ep)
    done = trunc = False
    total, k = 0.0, 0
    while not (done or trunc):
        a = sess.run(None, {iname: obs.reshape(1, -1).astype(np.float32)})[0][0]
        obs, r, done, trunc, info = env.step(a)
        total += r; k += 1
    bx = env.env.frame.ball.x
    half = env.env.field.length / 2.0
    if bx > half: goals += 1
    elif bx < -half: conceded += 1
    else: draws += 1
    rewards.append(total); steps.append(k)
print(f"== Eval NATIVO en rSoccer -- policy_field.onnx (20.5M) -- {N} episodios ==")
print(f"goles a favor : {goals:3d}  ({100*goals/N:5.1f}%)")
print(f"goles en contra: {conceded:3d}  ({100*conceded/N:5.1f}%)")
print(f"empates (30s)  : {draws:3d}  ({100*draws/N:5.1f}%)")
print(f"net%           : {100*(goals-conceded)/N:+.1f}")
print(f"reward medio/ep: {np.mean(rewards):.2f} +/- {np.std(rewards):.2f}")
print(f"pasos medio/ep : {np.mean(steps):.1f} / 300  (decisiones a 10 Hz; 300 = 30 s)")
