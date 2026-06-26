# Entrenamiento en AWS — guía paso a paso

Objetivo: correr el currículo completo (1v0 → 1v1 → 2v1 → mixto → mixto self-play)
+ el arquero, durante 10-20 h, para un modelo de jugabilidad avanzada.

## 1. Elegir la instancia (CPU, NO GPU)

Para esta MLP chica la **CPU es más rápida que la GPU** (el cuello es la
simulación del entorno, no la red). Una GPU sería plata tirada.

| Instancia | vCPU | ~USD/h | N_ENVS | Con $40 |
|---|---|---|---|---|
| **c7i.4xlarge** | 16 | ~0.71 | 16 | ~50 h |
| **c7i.8xlarge** | 32 | ~1.36 | 32 | ~30 h |

AMI: Ubuntu Server 22.04/24.04 LTS. Disco: 30 GB gp3 sobra.

## 2. Setup en la instancia

```bash
sudo apt update && sudo apt install -y python3-venv python3-pip git
git clone https://github.com/Sysmic-Robotics/VSSS-Software.git
cd VSSS-Software && git checkout feature/rl-fase1-red-neuronal
cd training
python3 -m venv .venv && source .venv/bin/activate
pip install -U pip
pip install -r aws/requirements-aws.txt
# torch build CPU (más liviano y rápido en instancia sin GPU):
pip install torch --index-url https://download.pytorch.org/whl/cpu
# verificar
python -c "import torch, stable_baselines3, gymnasium; print('OK', torch.__version__)"
```

## 3. Correr el currículo (en sesión persistente)

Usá `tmux` para que siga corriendo si se cae la conexión SSH:

```bash
tmux new -s train
source .venv/bin/activate
N_ENVS=16 bash aws/train_curriculum.sh    # ajustá N_ENVS a los vCPU
# Ctrl+B luego D para desadjuntar. Reconectar: tmux attach -t train
```

Monitoreo (otra terminal):
```bash
source .venv/bin/activate
tensorboard --logdir runs --host 0.0.0.0 --port 6006
# abrir túnel SSH: ssh -L 6006:localhost:6006 ubuntu@<IP>  → http://localhost:6006
```

## 4. Recuperar los modelos

Al terminar, exportar a ONNX y bajarlos:
```bash
python export_onnx.py --checkpoint checkpoints/phasemixedsp_final.zip --out models/policy_field.onnx
python export_onnx.py --checkpoint checkpoints/phasegk_final.zip      --out models/policy_gk.onnx
```
Desde tu PC:
```bash
scp ubuntu@<IP>:~/VSSS-Software/training/models/policy_field.onnx .
scp ubuntu@<IP>:~/VSSS-Software/training/models/policy_gk.onnx .
```

## 5. Apagar la instancia

⚠️ **Apagá/terminá la instancia al terminar** para no quemar créditos:
```bash
sudo shutdown -h now      # o terminarla desde la consola AWS
```

## 6. Costos — control

- Mirá el gasto en la consola AWS (Billing) periódicamente.
- Una instancia c7i NO se apaga sola; ponete una alarma de presupuesto.
- Los $40 dan de sobra para 20-30 h en c7i.4xlarge. El run completo del
  currículo de arriba (~40M pasos) entra en ese presupuesto.

## Notas
- El compile del engine Rust (`cargo build --features rl`) también funciona en
  esta instancia Linux — sirve para validar `RlCoach` (lo que en Windows está
  bloqueado por falta del linker MSVC).
- Domain randomization (ruido de obs/latencia) está pendiente — agregarlo antes
  del deploy en robots reales, no es necesario para el entrenamiento.
