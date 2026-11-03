#!/bin/bash
# Un equipo real con un estilo de juego, en un comando (día de partido).
#
#   tools/jugar.sh <estilo> <blue|yellow> </dev/ttyUSBn> [etiqueta] [left|right]
#   tools/jugar.sh defensivo blue /dev/ttyUSB0 vs_itandroids left
#
# Estilos: los archivos de config/estilos/ (equilibrado, defensivo, ofensivo, adaptativo).
# Lado: el que defendemos (left = arco en -x); sin lado, el azul defiende la izquierda y el
# amarillo la derecha. El resto es tools/salita_equipo.sh: coach en pausa hasta P o el botón
# Coach, registro en logs/salita/<fecha>/, identidad por equipo, barra espaciadora = parada.
set -eu
ESTILO=${1:-}
EQUIPO=${2:-}
PUERTO=${3:-}
ETIQ=${4:-partido}
LADO=${5:-}
cd "$(dirname "$(readlink -f "$0")")/.."
ARCHIVO=config/estilos/$ESTILO.json
if [ -z "$ESTILO" ] || [ ! -f "$ARCHIVO" ]; then
  echo "uso: tools/jugar.sh <estilo> <blue|yellow> </dev/ttyUSBn> [etiqueta] [left|right]" >&2
  echo "  estilos: $(ls config/estilos/*.json | xargs -n1 basename | sed 's/\.json$//' | tr '\n' ' ')" >&2
  exit 1
fi
case $LADO in
  "" | left | right) ;;
  *)
    echo "✗ lado '$LADO' no vale: left o right" >&2
    exit 1
    ;;
esac
echo "Estilo $ESTILO ($ARCHIVO)${LADO:+ · defendemos $LADO}"
if [ -n "$LADO" ]; then
  exec env VSSL_PARAMS="$ARCHIVO" VSSL_SIDE="$LADO" tools/salita_equipo.sh "$EQUIPO" "$PUERTO" "${ETIQ}_$ESTILO"
fi
exec env VSSL_PARAMS="$ARCHIVO" tools/salita_equipo.sh "$EQUIPO" "$PUERTO" "${ETIQ}_$ESTILO"
