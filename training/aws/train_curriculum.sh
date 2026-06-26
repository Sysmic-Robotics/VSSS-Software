#!/usr/bin/env bash
# Currículo completo encadenado con warm-start — pensado para una instancia
# AWS de CPU con muchos núcleos (c7i.4xlarge/8xlarge). NO usa GPU: para esta
# MLP chica la CPU es más rápida (el cuello es la simulación, no la red).
#
# Uso:
#   N_ENVS=16 bash aws/train_curriculum.sh          # c7i.4xlarge (16 vCPU)
#   N_ENVS=32 bash aws/train_curriculum.sh          # c7i.8xlarge (32 vCPU)
#
# Ajustá los --total-steps según las horas/créditos disponibles. Los valores
# de abajo apuntan a un run "considerablemente avanzado" (~40M pasos totales).
set -euo pipefail

N=${N_ENVS:-16}
CK=checkpoints
echo "=== Currículo VSSS — N_ENVS=$N ==="

# 1) 1v0 — control individual (anotar)
python train.py --phase 1     --total-steps 2000000  --n-envs "$N" --device cpu --run-name c_p1

# 2) 1v1 — vs portero (warm-start)
python train.py --phase 2     --warm-start $CK/phase1_final.zip   --total-steps 2000000  --n-envs "$N" --device cpu --run-name c_p2

# 3) 2v1 — cooperación (entra el 2º robot, Box 6)
python train.py --phase 2v1   --warm-start $CK/phase2_final.zip   --total-steps 3000000  --n-envs "$N" --device cpu --run-name c_2v1

# 4) Mixto 2vN vs rule-based — robustez de escenario
python train.py --phase mixed --warm-start $CK/phase2v1_final.zip --total-steps 8000000  --n-envs "$N" --device cpu --run-name c_mixed

# 5) Mixto 2vN SELF-PLAY — el run largo (pool de snapshots + 20% rule-based)
python train.py --phase mixedsp --warm-start $CK/phasemixed_final.zip --total-steps 20000000 --n-envs "$N" --device cpu --run-name c_mixedsp

# 6) Arquero — RED SEPARADA, recompensa de defensa, confinado a su área.
#    (Refinamiento futuro: entrenarlo contra el atacante ya entrenado en vez de
#     rule-based, vía un env de defensa con opponent_mode=selfplay.)
python train.py --phase gk    --total-steps 5000000  --n-envs "$N" --device cpu --run-name c_gk

echo "=== Listo. Modelos finales en $CK/ ==="
echo "Exportar a ONNX:  python export_onnx.py --checkpoint $CK/phasemixedsp_final.zip --out models/policy_field.onnx"
echo "                  python export_onnx.py --checkpoint $CK/phasegk_final.zip      --out models/policy_gk.onnx"
