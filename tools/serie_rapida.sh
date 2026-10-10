#!/bin/bash
# Serie ACELERADA: FIRASim headless a FPS cuadros/s (60 = tiempo real), engines con reloj
# por frames (VSSL_FRAME_CLOCK=1) y semilla de ruido por partido, director con las pausas
# escaladas. Resultados en OUT (en D:). Los CSV por tick se escriben en ext4 durante el
# partido (sobre /mnt/d el lazo salta frames) y se copian a OUT al final.
#   serie_rapida.sh N MIN GOLES RIVAL FPS OUT
# Variables opcionales: LADO_AZUL=left|right (sin engine de detección), COACH_AZUL,
# SEED_AZUL/SEED_AMARILLO (base de semillas), ORDEN=amarillo_primero, SIN_LOG=1.
N=${1:-3}; MIN=${2:-3}; GOLES=${3:-3}; RIVAL=${4:-rule_based}; FPS=${5:-480}
REPO=/mnt/d/Proyectos/VSSS-Software
OUT=${6:-$REPO/logs/validacion/rapida}
COACH_AZUL=${COACH_AZUL:-heuristic}
SEED_AZUL=${SEED_AZUL:-1000}; SEED_AMARILLO=${SEED_AMARILLO:-2000}
if [ -n "${SIN_LOG:-}" ]; then LOGVAR=VSSL_SIN_LOG_DUMMY; else LOGVAR=VSSL_MATCH_LOG; fi
ENGINE=/root/vsss-target/release/rustengine
DIRECTOR=/root/vsss-target/release/match_director
SPEED=$((FPS / 60))
WORK=/root/rapida_work
mkdir -p "$OUT"; rm -rf "$WORK"; mkdir -p "$WORK"
cd "$REPO" || exit 1
pkill -9 -x rustengine; pkill -9 -x match_director; pkill -9 -x FIRASim; /bin/sleep 1
/root/FIRASim/bin/FIRASim --headless --fps "$FPS" > "$OUT/fira.log" 2>&1 &
/bin/sleep 6
echo "FIRASim: $(pgrep -x FIRASim | wc -l) proceso(s), fps $FPS, speed $SPEED"

if [ -n "${LADO_AZUL:-}" ]; then
  BLUE=$LADO_AZUL
  if [ "$BLUE" = left ]; then YEL=right; else YEL=left; fi
else
  VSSL_TEAM_COLOR=blue VSSL_SIDE=left VSSL_COACH=heuristic "$ENGINE" > "$OUT/side.log" 2>&1 &
  p=$!; /bin/sleep 5; kill $p; /bin/sleep 1
  if grep -q "mitad del arco RIVAL" "$OUT/side.log"; then BLUE=right; YEL=left; else BLUE=left; YEL=right; fi
fi
echo "lado del azul: $BLUE"

rm -f "$OUT/serie.jsonl"
i=1
while [ "$i" -le "$N" ]; do
  KICK=blue
  if [ $((i % 2)) -eq 0 ]; then KICK=yellow; fi
  AZUL_LOG="$LOGVAR=$WORK/serie_${i}_azul.csv"
  AMARILLO_LOG="$LOGVAR=$WORK/serie_${i}_amarillo.csv"
  # stderr del engine también a ext4: imprime una línea por envío de radio.
  if [ "${ORDEN:-}" = amarillo_primero ]; then
    VSSL_TEAM_COLOR=yellow VSSL_SIDE=$YEL VSSL_COACH=$RIVAL VSSL_BIDIRECTIONAL=1 VSSL_VISION_NOISE=1 VSSL_FRAME_CLOCK=1 \
      VSSL_NOISE_SEED=$((SEED_AMARILLO + i)) env "$AMARILLO_LOG" "$ENGINE" > "$WORK/amarillo_$i.log" 2>&1 &
    py=$!
    /bin/sleep 1
    VSSL_TEAM_COLOR=blue VSSL_SIDE=$BLUE VSSL_COACH=$COACH_AZUL VSSL_BIDIRECTIONAL=1 VSSL_VISION_NOISE=1 VSSL_FRAME_CLOCK=1 \
      VSSL_NOISE_SEED=$((SEED_AZUL + i)) env "$AZUL_LOG" "$ENGINE" > "$WORK/azul_$i.log" 2>&1 &
    pb=$!
  else
    VSSL_TEAM_COLOR=blue VSSL_SIDE=$BLUE VSSL_COACH=$COACH_AZUL VSSL_BIDIRECTIONAL=1 VSSL_VISION_NOISE=1 VSSL_FRAME_CLOCK=1 \
      VSSL_NOISE_SEED=$((SEED_AZUL + i)) env "$AZUL_LOG" "$ENGINE" > "$WORK/azul_$i.log" 2>&1 &
    pb=$!
    VSSL_TEAM_COLOR=yellow VSSL_SIDE=$YEL VSSL_COACH=$RIVAL VSSL_BIDIRECTIONAL=1 VSSL_VISION_NOISE=1 VSSL_FRAME_CLOCK=1 \
      VSSL_NOISE_SEED=$((SEED_AMARILLO + i)) env "$AMARILLO_LOG" "$ENGINE" > "$WORK/amarillo_$i.log" 2>&1 &
    py=$!
  fi
  /bin/sleep 4
  echo "=== partido $i (saque $KICK) $(date +%T) ==="
  "$DIRECTOR" --minutes "$MIN" --goals "$GOLES" --kickoff "$KICK" --speed "$SPEED" --out "$OUT/serie.jsonl" 2>&1 | grep -E "GOL|RESULTADO|free ball"
  kill $pb $py; /bin/sleep 2
  grep -h "reloj por frames: frames" "$WORK/azul_$i.log" | tail -1
  grep -h "reloj por frames: frames" "$WORK/amarillo_$i.log" | tail -1
  i=$((i + 1))
done
pkill -x FIRASim
cp "$WORK"/serie_*.csv "$WORK"/*.log "$OUT"/ 2>/dev/null; rm -rf "$WORK"
echo "=== serie terminada $(date +%T) ==="
cat "$OUT/serie.jsonl"
