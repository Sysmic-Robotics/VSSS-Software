# vsss-rl — Entrenamiento RL para VSSS-Software

Trainer Python de la política neuronal que se ejecuta dentro del engine Rust
(`src/coach/rl_coach.rs`, feature `rl`). Proyecto INF398 — UTFSM.

Hay **dos stacks de entrenamiento** con el mismo contrato (obs 52 floats + acción `(v, ω)`):

| Stack | Entorno | Física | Uso |
|---|---|---|---|
| Sim propio | `vsss_rl/soccer_env.py` + `train.py` | cinemática calibrada a FIRASim | currículo + self-play (rápido) |
| **rSoccer** (actual) | `vsss_rl/rsoccer_env.py` + `train_rsoccer.py` | rSim/ODE realista | corridas principales |

## Setup

### Stack sim propio (`.venv`, cualquier SO)
```bash
python -m venv training/.venv
source training/.venv/bin/activate        # Windows: training\.venv\Scripts\Activate.ps1
pip install -r training/requirements.txt
pip install torch --index-url https://download.pytorch.org/whl/cpu   # CPU basta (red ~159k params)
```

### Stack rSoccer (`.venv-rsoccer`, Linux, Python ≤ 3.12)
rSoccer arrastra dependencias viejas (numpy<2, protobuf<3.21) → venv separada.
Si el sistema no tiene un Python compatible, instalarlo con [uv](https://github.com/astral-sh/uv):
```bash
cd training
uv venv --python 3.10 .venv-rsoccer && . .venv-rsoccer/bin/activate
uv pip install torch --index-url https://download.pytorch.org/whl/cpu
uv pip install -r requirements-rsoccer.txt
python -c "import rsoccer_gym; print('rSoccer OK')"
```

## Entrenar

### rSoccer (corrida principal)
```bash
. training/.venv-rsoccer/bin/activate
cd training
N_ENVS=8 RSOCCER_STEPS=15000000 python train_rsoccer.py
```
- Checkpoints cada 100k pasos en `checkpoints_rsoccer/` (+ `best/` del EvalCallback).
- TensorBoard: `tensorboard --logdir training/tb_rsoccer`.
- Incluye: policy custom de dos torres, clip de log_std, `ent_coef=0.001`, `target_kl=0.05`,
  ruido de observación medido, rampa del ejecutor replicada y árbitro interno (reglas LARC).

### Sim propio (currículo completo 1v0 → … → self-play)
```bash
. training/.venv/bin/activate
cd training
python run_curriculum.py            # o por fase: python train.py --phase 2v1 --warm-start ...
```

## Evaluar
```bash
# win-tendency vs baselines fijos (sim propio), n=100:
python eval_match.py --checkpoint checkpoints/phasemixedsp_final.zip --episodes 100

# eval nativo en rSoccer de un modelo YA exportado a ONNX, n=50:
EVAL_MODEL=models/policy_field.onnx EVAL_EPISODES=50 python eval_rsoccer_native.py
```

## Exportar y desplegar en FIRASim
```bash
python export_onnx.py --checkpoint checkpoints_rsoccer/rsoccer_field_final.zip --out models/policy_field.onnx
```
Luego, desde la raíz del repo (FIRASim abierto y en play):
```bash
cargo build --release --features rl
VSSL_COACH=rl VSSL_RL_MODEL=training/models/policy_field.onnx VSSL_RL_GK_MODEL=none \
  VSSL_TEAM_COLOR=blue ./target/release/rustengine
```
(`VSSL_RL_GK_MODEL=none` → arquero rule-based. `VSSL_RL_DEBUG=1` loguea obs/acciones.)

## System-ID (calibración empírica de los simuladores)
```bash
python sysid/gen_protos.py                 # una vez: bindings protobuf de FIRASim
python sysid/firasim_sysid.py              # dinámica de FIRASim → sysid/data/*.csv
python sysid/fit_sysid.py                  # ajusta → sim_calibration.json
python sysid/measure_wheelbase.py          # r y L efectivos de FIRASim (r=0.02, L=0.085)
python sysid/measure_wheelbase_rsim.py     # ídem para rSim (r=0.026, L≈0.0776)
```
Estas constantes efectivas están aplicadas en `src/radio/commands.rs` (deploy) y
`vsss_rl/rsoccer_env.py` (entrenamiento): la misma `(v, ω)` produce la misma respuesta física.

## Convención: contrato de observación

El layout está fijado por el engine Rust en
[src/coach/observation.rs](../src/coach/observation.rs) y espejado en
[vsss_rl/observation.py](vsss_rl/observation.py).

| Constante           | Valor   |
|---------------------|---------|
| `FLAT_SIZE`         | 52      |
| `FIELD_HALF_X`      | 0.75 m  |
| `FIELD_HALF_Y`      | 0.65 m  |
| `MAX_VEL_NORM`      | 1.5 m/s |
| `MAX_OMEGA_NORM`    | π rad/s |
| Acción              | `(v, ω)` ∈ [−1,1]² por robot de campo; `V_MAX=1.2 m/s`, `OMEGA_MAX=3.0 rad/s` |
| Frecuencia decisión | 10 Hz (frame-skip 4 en rSoccer / `COACH_DECISION_PERIOD=6` en el engine) |

**No cambiar sin actualizar el lado Rust en el mismo PR.**
