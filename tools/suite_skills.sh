#!/bin/bash
# Suite de regresión de las 13 skills en FIRASim, con el proxy de ruido y en los dos modos
# (frontal y bidireccional).
#
#   tools/suite_skills.sh [rapida|completa] DIR
#
#   rapida    un caso por skill (13 casos), ~50 min con 20 repeticiones: para cada change.
#   completa  32 casos, ~2 h con 20 repeticiones: para hitos (motion, torneo, sysid).
#
# Variables: REPEAT (repeticiones, 20 por defecto), FIRASIM_BIN (ruta al binario de
# FIRASim: si está definida, el script lo relanza cuando se cae).
#
# FIRASim tiene que estar corriendo. No correr la GUI ni otro engine a la vez: comandan
# los mismos robots. Por repetición queda solo la línea JSON (DIR/bdN.jsonl) y el CSV por
# tick de las repeticiones que fallan; si FIRASim se cae, se reanuda desde la pendiente.
set -u
SUITE=${1:-rapida}
OUT=${2:?uso: tools/suite_skills.sh [rapida|completa] DIR}
REP=${REPEAT:-20}
case "$SUITE" in
  rapida) CASES=suite-rapida ;;
  completa) CASES=suite ;;
  *) echo "suite desconocida: $SUITE (rapida|completa)" >&2; exit 2 ;;
esac
cd "$(dirname "$0")/.." || exit 1

if pgrep -x rustengine >/dev/null || pgrep -x skill_test >/dev/null || pgrep -x scenario >/dev/null; then
  echo "hay otro engine corriendo (rustengine/GUI, skill_test o scenario): ciérralo antes de la suite" >&2
  exit 1
fi
cargo build --release --bin motion_bench || exit 1
mkdir -p "$OUT/csv"

firasim_up() {
  if [ -n "${FIRASIM_BIN:-}" ] && ! pgrep -x FIRASim >/dev/null; then
    (cd "$(dirname "$FIRASIM_BIN")" && nohup "./$(basename "$FIRASIM_BIN")" >> "$OUT/firasim.log" 2>&1 &)
    sleep 8
  fi
}

for bd in 0 1; do
  for intento in 1 2 3 4 5 6; do
    firasim_up
    VSSL_VISION_NOISE=1 VSSL_BIDIRECTIONAL=$bd target/release/motion_bench "$CASES" \
      --repeat "$REP" --out "$OUT/bd$bd.jsonl" --csv-dir "$OUT/csv" --tag "_bd$bd" \
      --csv-failures-only > "$OUT/bd$bd.txt" 2> >(grep -av 'comandos enviados\|✓ DETECTION\|\] Stats:' >> "$OUT/bd$bd.err")
    code=$?
    echo "$(date +%T) $SUITE bd$bd intento $intento exit=$code" >> "$OUT/progress"
    [ $code -eq 3 ] || break
    echo "FIRASim no responde (código 3); reanudo en 20 s" >&2
    if [ -n "${FIRASIM_BIN:-}" ]; then pkill -x FIRASim; fi
    sleep 20
  done
done
echo "resultados en $OUT (bd0.txt frontal, bd1.txt bidireccional: tasa por caso y por skill)"
