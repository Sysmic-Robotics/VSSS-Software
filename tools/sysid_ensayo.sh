#!/bin/bash
# Ensayo de punta a punta de la sysid, sin robot.
#
#   tools/sysid_ensayo.sh DIR [firasim]
#
# Siempre (planta offline, ~3 min):
#   1. comandos de los 6 perfiles en la planta (skill_test --transport plant);
#   2. equivalencia Rust ↔ Python: la planta con la capa cargada con el modelo de la spec
#      contra el modelo de Python sin ruido (MEE < 1 mm en todos los perfiles);
#   3. poses sintéticas del modelo de la spec con el ruido del proxy y el parche corrido
#      8 mm adelante y 5 mm a la derecha; el ajuste tiene que recuperar el modelo (±15 %,
#      latencia ±1 tick de 50 ms);
#   4. la planta con la capa AJUSTADA contra los sintéticos: MEE de v_steps < 0.02 m y
#      los umbrales de D6.
# Con `firasim` (FIRASim corriendo, sin otro engine; ~10 min más):
#   5. la sesión en FIRASim sin la capa (tools/sysid_sesion.sh) y el ajuste de su
#      dinámica propia (sección firasim);
#   6. la sesión en FIRASim con la capa (el modelo de la spec como robot, más la sección
#      firasim) contra los sintéticos sin ruido: valida el descuento de la dinámica de
#      FIRASim (design, D4).
set -u
OUT=${1:?uso: tools/sysid_ensayo.sh DIR [firasim]}
MODO=${2:-}
cd "$(dirname "$0")/.." || exit 1
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
PERFILES="v_steps v_ramp v_chirp w_steps w_chirp sat_arc"
FALLAS=0
falla() { echo "✗ $1"; FALLAS=$((FALLAS + 1)); }

cargo build --release --bin skill_test || exit 1
BIN=target/release/skill_test

planta() { # planta DIR [selector calibración]
  mkdir -p "$1"
  for p in $PERFILES; do
    VSSL_REAL_ACTUATOR=${2:-} VSSL_REAL_CALIBRATION=${3:-} "$BIN" --mode vw --transport plant \
      --profile "$p" --robot 0 --team blue --log "$1/$p.csv" 2>/dev/null || return 1
  done
}

mee_max() { # mee_max JSON → el MEE más alto de todos los perfiles
  python3 -c "import json,sys; r=json.load(open(sys.argv[1]))['perfiles']; print(max(v['mee_m'] for v in r.values()))" "$1"
}

calibracion() { # calibracion ARCHIVO VISION_ID [ARCHIVO_FIRASIM]: el modelo de la spec como medición
  python3 - "$@" <<'EOF'
import json, sys
sys.path.insert(0, "tools")
import sysid_ajuste as aj
path, vid = sys.argv[1], int(sys.argv[2])
m = dict(aj.MODELO_SINTETICO, vision_id=vid, mi_robot_id=vid + 1, battery="full", battery_v=8.2,
         date="2026-10-08", notes="modelo de la spec (ensayo)", vision_latency_ms=0.0)
cal = {"firasim": None, "measurements": [m]}
if len(sys.argv) > 3:
    cal["firasim"] = json.load(open(sys.argv[3])).get("firasim")
json.dump(cal, open(path, "w"), indent=2)
EOF
}

echo "== 1. comandos en la planta"
planta "$OUT/comandos" || { echo "skill_test --transport plant falló"; exit 1; }

echo "== 2. equivalencia Rust ↔ Python (sin ruido)"
calibracion "$OUT/cal_spec.json" 1
planta "$OUT/planta_spec" 1:full "$OUT/cal_spec.json" || exit 1
python3 tools/sysid_ajuste.py sintetico "$OUT/comandos" "$OUT/sint_limpio" --ruido-pos 0 --ruido-theta 0 --perdida 0 > /dev/null
python3 tools/sysid_validacion.py "$OUT/sint_limpio" "$OUT/planta_spec" --json "$OUT/equivalencia.json"
M=$(mee_max "$OUT/equivalencia.json")
python3 -c "import sys; sys.exit(0 if float(sys.argv[1]) < 0.001 else 1)" "$M" || falla "la planta de Rust y el modelo de Python difieren (MEE máx. $M m)"

echo "== 3. ajuste de datos sintéticos con ruido"
python3 tools/sysid_ajuste.py sintetico "$OUT/comandos" "$OUT/sint" --parche-mm 8,-5 > /dev/null
python3 tools/sysid_ajuste.py ajustar "$OUT/sint" --robot 1 --mi-robot-id 2 --bateria full --bateria-v 8.2 \
  --salida "$OUT/cal_ajustada.json" --esperado spec || falla "el ajuste no recupera el modelo de la spec"
PARCHE=$(python3 -c "import json,sys; m=json.load(open(sys.argv[1]))['measurements'][0]['fit']['patch_offset_mm']; print(f'{m[0]},{m[1]}')" "$OUT/cal_ajustada.json")

echo "== 4. planta con la capa ajustada contra los sintéticos (parche $PARCHE mm)"
planta "$OUT/planta_ajustada" 1:full "$OUT/cal_ajustada.json" || exit 1
python3 tools/sysid_validacion.py "$OUT/sint" "$OUT/planta_ajustada" --parche-mm "$PARCHE" \
  --json "$OUT/validacion_planta.json" --estricto || falla "no cumple los umbrales de D6"
python3 -c "import json,sys; sys.exit(0 if json.load(open(sys.argv[1]))['perfiles']['v_steps']['mee_m'] < 0.02 else 1)" \
  "$OUT/validacion_planta.json" || falla "MEE de v_steps ≥ 0.02 m"

if [ "$MODO" = firasim ]; then
  if ! pgrep -x FIRASim >/dev/null; then
    echo "FIRASim no está corriendo"
    exit 1
  fi
  echo "== 5. FIRASim sin la capa: su dinámica propia"
  SYSID_TRANSPORT=firasim SYSID_AUTO=1 SYSID_DIR="$OUT/firasim_sin_capa" tools/sysid_sesion.sh 0 0 full || falla "sesión en FIRASim"
  python3 tools/sysid_ajuste.py ajustar "$OUT/firasim_sin_capa/0/full" --firasim --salida "$OUT/cal_firasim.json" \
    --notas "FIRASim sin la capa (ensayo)" || falla "ajuste de FIRASim"

  echo "== 6. FIRASim con la capa (modelo de la spec, descontando FIRASim) contra los sintéticos sin ruido"
  calibracion "$OUT/cal_spec_firasim.json" 0 "$OUT/cal_firasim.json"
  VSSL_REAL_ACTUATOR=0:full VSSL_REAL_CALIBRATION="$OUT/cal_spec_firasim.json" SYSID_TRANSPORT=firasim SYSID_AUTO=1 \
    SYSID_DIR="$OUT/firasim_con_capa" tools/sysid_sesion.sh 0 0 full || falla "sesión en FIRASim con la capa"
  python3 tools/sysid_validacion.py "$OUT/sint_limpio" "$OUT/firasim_con_capa/0/full" \
    --json "$OUT/validacion_firasim.json" --estricto || falla "FIRASim con la capa no cumple los umbrales de D6"
fi

echo
if [ $FALLAS -eq 0 ]; then echo "✓ ensayo completo sin fallas ($OUT)"; else echo "✗ $FALLAS falla(s) ($OUT)"; fi
exit $FALLAS
