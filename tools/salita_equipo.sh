#!/bin/bash
# Un equipo con los robots reales, para partidos de la salita con dos bases (una por equipo).
# A diferencia de tools/iniciar.sh, deja abrir un segundo programa (el otro equipo, por su
# propia base) y registra el partido. Cada equipo tiene su archivo de identidad.
#
#   tools/salita_equipo.sh blue   /dev/ttyUSB0 p1     # terminal 1
#   tools/salita_equipo.sh yellow /dev/ttyUSB1 p1     # terminal 2
#   python3 tools/referee_cli.py                      # terminal 3: árbitro de los dos
#
# Deja logs/salita/<fecha>/<etiqueta>_<equipo>.csv (VSSL_MATCH_LOG) y, solo el azul, la
# grabación de la cámara <etiqueta>_vision.bin (VSSL_VISION_RECORD). Parte con el coach en
# pausa: P o el botón Coach de su interfaz lo activa; la barra espaciadora detiene a todos.
# Identidad: config/identidad_robots.json (azul) y config/identidad_robots_yellow.json
# (amarillo), editables en la pestaña Robots ("Identificar"). Se respetan otras VSSL_*
# (VSSL_PARAMS, VSSL_SIDE, VSSL_COACH, VSSL_MULTICAST_IFACE...).
set -eu
EQUIPO=${1:-}
PUERTO=${2:-}
ETIQ=${3:-partido}
case $EQUIPO in
  blue | yellow) ;;
  *)
    echo "uso: tools/salita_equipo.sh blue|yellow /dev/ttyUSBn [etiqueta]" >&2
    exit 1
    ;;
esac
cd "$(dirname "$(readlink -f "$0")")/.."
if [ -z "$PUERTO" ] || [ ! -e "$PUERTO" ]; then
  echo "✗ Falta el puerto de la base de este equipo o no existe ('$PUERTO')." >&2
  echo "  Puertos: $(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | tr '\n' ' ')" >&2
  exit 1
fi
command -v cargo >/dev/null || PATH="$HOME/.cargo/bin:$PATH"
DIR=logs/salita/$(date +%F)
mkdir -p "$DIR"
echo "Compilando (si hace falta)..."
cargo build --release >"$DIR/compilar_$EQUIPO.log" 2>&1 || {
  tail -n 20 "$DIR/compilar_$EQUIPO.log"
  exit 1
}
if [ "$EQUIPO" = blue ]; then
  IDENT=${VSSL_IDENTIDAD:-config/identidad_robots.json}
  GRABAR=$DIR/${ETIQ}_vision.bin
else
  IDENT=${VSSL_IDENTIDAD:-config/identidad_robots_yellow.json}
  GRABAR=
fi
echo "Equipo $EQUIPO por $PUERTO · identidad $IDENT · registro $DIR/${ETIQ}_$EQUIPO.csv"
echo "⚠ Coach en pausa: P o el botón Coach para jugar. Barra espaciadora = parada de emergencia."
exec env VSSL_VISION_SOURCE=sslvision VSSL_RADIO_TARGET=basestation VSSL_BASESTATION_DEVICE="$PUERTO" \
  VSSL_TEAM_COLOR="$EQUIPO" VSSL_BIDIRECTIONAL="${VSSL_BIDIRECTIONAL:-1}" VSSL_VISION_NOISE=0 \
  VSSL_DEBUG_GUI=1 VSSL_COACH_PAUSA="${VSSL_COACH_PAUSA:-1}" VSSL_IDENTIDAD="$IDENT" \
  VSSL_MATCH_LOG="$DIR/${ETIQ}_$EQUIPO.csv" VSSL_VISION_RECORD="$GRABAR" target/release/rustengine
