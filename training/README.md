# vsss-rl — Entrenamiento RL para VSSS-Software

Trainer Python para la política neuronal que se ejecuta dentro del engine Rust
(`src/coach/rl_coach.rs` — pendiente). Proyecto INF398.

## Setup (Windows, primer uso)

```powershell
# desde la raíz del repo (no entrar a training/)
python -m venv training\.venv
training\.venv\Scripts\Activate.ps1
pip install -r training\requirements.txt
# torch con CUDA se instala aparte:
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124
```

## Verificar GPU

```powershell
training\.venv\Scripts\python.exe -c "import torch; print('cuda:', torch.cuda.is_available(), torch.cuda.get_device_name(0) if torch.cuda.is_available() else '-')"
```

## Estructura

```
training/
├── vsss_rl/
│   ├── observation.py    # Mirror del contrato Rust (52 floats)
│   ├── env_phase1.py     # Wrapper rSoccer Fase 1 (1v0)
│   ├── policy.py         # Red MLP
│   └── reward.py         # Función de recompensa Fase 1
├── train_phase1.py       # Script de entrenamiento PPO
├── eval.py               # Evaluación contra RuleBasedCoach (Python port)
├── checkpoints/          # (gitignored) modelos entrenados
└── runs/                 # (gitignored) logs TensorBoard
```

## Entrenar Fase 1 (1v0)

```powershell
training\.venv\Scripts\Activate.ps1
cd training
python train_phase1.py --total-steps 1000000 --device cuda
```

TensorBoard:
```powershell
tensorboard --logdir training\runs
```

## Convención: contrato de observación

El layout de la observación está fijado por el engine Rust en
[src/coach/observation.rs](../src/coach/observation.rs).

| Constante           | Valor   |
|---------------------|---------|
| `FLAT_SIZE`         | 52      |
| `FIELD_HALF_X`      | 0.75 m  |
| `FIELD_HALF_Y`      | 0.65 m  |
| `MAX_VEL_NORM`      | 1.5 m/s |
| `MAX_OMEGA_NORM`    | π rad/s |

**No cambiar sin actualizar el lado Rust en el mismo PR.**
