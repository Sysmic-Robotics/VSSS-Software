#!/bin/bash
# Sesión guiada de sysid (v, ω): corre los 6 perfiles de identificación en un robot con
# una batería, grabando comandos, pose de visión, metadatos y paquetes crudos.
#
#   tools/sysid_sesion.sh <posición de radio> <id de visión> <full|half> [--repetir PERFIL]
#   tools/sysid_sesion.sh --help        (procedimiento completo: tools/sysid_procedimiento.txt)
#
# Variables: SYSID_DIR (carpeta raíz, ~/sysid por defecto), SYSID_TEAM (blue por defecto),
# SYSID_TRANSPORT (base-station por defecto; firasim para ensayar en el simulador, que
# coloca el robot solo), SYSID_AUTO=1 (sin pausas entre perfiles: solo con firasim).
set -u
cd "$(dirname "$0")/.." || exit 1
PROC=tools/sysid_procedimiento.txt

case "${1:-}" in -h|--help) cat "$PROC"; exit 0 ;; esac
if [ $# -lt 3 ]; then
  echo "uso: tools/sysid_sesion.sh <posición de radio> <id de visión> <full|half> [--repetir PERFIL]" >&2
  echo "procedimiento completo: tools/sysid_sesion.sh --help" >&2
  exit 2
fi

POS=$1
VID=$2
BAT=$3
shift 3
SOLO=""
if [ "${1:-}" = "--repetir" ]; then
  SOLO=${2:?--repetir requiere el nombre del perfil}
fi
TEAM=${SYSID_TEAM:-blue}
TRANSPORT=${SYSID_TRANSPORT:-base-station}
AUTO=${SYSID_AUTO:-0}
ROOT=${SYSID_DIR:-$HOME/sysid}
DIR="$ROOT/$VID/$BAT"
PERFILES="v_steps v_ramp v_chirp w_steps w_chirp sat_arc"

case "$BAT" in full|half) ;; *) echo "batería: full o half (recibí '$BAT')" >&2; exit 2 ;; esac
case "$TRANSPORT" in
  base-station) VISION=real ;;
  firasim) VISION=sim ;;
  *) echo "SYSID_TRANSPORT: base-station o firasim" >&2; exit 2 ;;
esac
if [ "$AUTO" = 1 ] && [ "$TRANSPORT" != firasim ]; then
  echo "SYSID_AUTO=1 solo con SYSID_TRANSPORT=firasim: con el robot real, alguien tiene que colocarlo y vigilarlo" >&2
  exit 2
fi
if [ -n "$SOLO" ] && ! echo " $PERFILES " | grep -q " $SOLO "; then
  echo "perfil desconocido: $SOLO (perfiles: $PERFILES)" >&2
  exit 2
fi

for p in rustengine skill_test scenario motion_bench match_director; do
  if pgrep -x "$p" >/dev/null; then
    echo "hay otro engine corriendo ($p): ciérralo antes de la sesión (comandan los mismos robots)" >&2
    exit 1
  fi
done

cargo build --release --bin skill_test || exit 1
BIN=target/release/skill_test
mkdir -p "$DIR"

pregunta() { # pregunta TEXTO DEFAULT → respuesta en $R
  if [ "$AUTO" = 1 ]; then R=$2; return; fi
  read -r -p "$1 [$2]: " R
  R=${R:-$2}
}

# Datos de la sesión (se guardan una vez; --repetir los reusa).
if [ ! -f "$DIR/sesion.json" ]; then
  if [ "$TRANSPORT" = base-station ]; then
    cat <<EOF

SEGURIDAD: la base NO tiene timeout de serial. Si skill_test muere sin mandar ceros, la
base repite el último comando para siempre. Siempre tiene que haber alguien listo para
LEVANTAR EL ROBOT o DESENCHUFAR LA BASE. Para cortar un perfil: Ctrl+C (manda ceros).
Nunca cerrar la terminal ni matar el proceso.

EOF
    while :; do
      pregunta "Voltaje de la batería medido con tester (V)" ""
      [[ "$R" =~ ^[0-9]+([.][0-9]+)?$ ]] && break
      echo "  un número, por ejemplo 8.2"
    done
    VOLTS=$R
  else
    VOLTS=0
  fi
  pregunta "MI_ROBOT_ID del firmware de este robot" "$((POS + 1))"
  MRI=$R
  pregunta "Notas de la sesión (cancha, iluminación, lo que llame la atención)" ""
  NOTAS=$R
  python3 - "$DIR/sesion.json" "$VID" "$MRI" "$BAT" "$VOLTS" "$NOTAS" "$TRANSPORT" "$POS" <<'EOF'
import json, sys, time
path, vid, mri, bat, volts, notas, transport, pos = sys.argv[1:]
json.dump({"vision_id": int(vid), "mi_robot_id": int(mri), "radio_slot": int(pos), "battery": bat,
           "battery_v": float(volts) if float(volts) > 0 else None, "notes": notas,
           "transport": transport, "date": time.strftime("%Y-%m-%d")},
          open(path, "w"), indent=2, ensure_ascii=False)
EOF
fi
VOLTS=$(python3 -c "import json,sys; v=json.load(open(sys.argv[1]))['battery_v']; print(v if v else '')" "$DIR/sesion.json")

echo
echo "Sesión: robot (radio $POS, visión $VID, $TEAM), batería $BAT${VOLTS:+ ($VOLTS V)}, $TRANSPORT → $DIR"
"$BIN" --list-profiles

correr() { # correr PERFIL → código de salida de skill_test
  local p=$1
  local extra=(--track "$VID")
  if [ "$TRANSPORT" = base-station ]; then
    extra+=(--battery "$BAT")
    [ -n "$VOLTS" ] && extra+=(--battery-v "$VOLTS")
  fi
  VSSL_VISION_RECORD="$DIR/$p.vision.bin" "$BIN" --transport "$TRANSPORT" --vision "$VISION" \
    --mode vw --team "$TEAM" --robot "$POS" --profile "$p" --log "$DIR/$p.csv" "${extra[@]}"
}

for p in $PERFILES; do
  [ -n "$SOLO" ] && [ "$p" != "$SOLO" ] && continue
  while :; do
    if [ "$AUTO" != 1 ]; then
      echo
      read -r -p "[$p] Robot en la marca (centro, mirando a +x), cancha despejada. Enter = correr, s = saltar, q = salir: " R
      case "$R" in s) break ;; q) exit 0 ;; esac
    fi
    correr "$p"
    code=$?
    if [ $code -eq 0 ]; then
      echo "[$p] completo"
      break
    fi
    echo "[$p] NO terminó (código $code; el motivo está arriba y en $DIR/$p.meta.json)"
    if [ "$AUTO" = 1 ]; then
      exit 1
    fi
    read -r -p "[$p] r = repetir, s = seguir con el próximo, q = salir: " R
    case "$R" in s) break ;; q) exit 1 ;; *) ;; esac
  done
done

echo
echo "Listo. Archivos en $DIR:"
ls "$DIR"
echo
echo "Ajuste (en este PC o en otro, con la carpeta copiada):"
echo "  python3 tools/sysid_ajuste.py ajustar $DIR"
