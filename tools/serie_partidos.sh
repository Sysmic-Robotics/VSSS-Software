#!/bin/bash
# Serie de N partidos a TIEMPO REAL con el director: azul heurístico vs amarillo (RIVAL), ruido de cámara,
# FIRASim con ventana, saque alternado. Engines nuevos por partido (adaptación limpia).
#   tools/serie_partidos.sh [N] [MIN] [GOLES] [RIVAL]   (desde WSL, con el engine en /root/vsss-target)
# Logs: logs/serie_<n>_azul.csv, logs/serie_<n>_amarillo.csv, resultados en logs/serie.jsonl.
N=${1:-6}; MIN=${2:-3}; GOLES=${3:-3}; RIVAL=${4:-rule_based}
cd /mnt/d/Proyectos/VSSS-Software || exit 1
ENGINE=/root/vsss-target/release/rustengine
DIRECTOR=/root/vsss-target/release/match_director
pkill -9 -x rustengine; pkill -9 -x match_director; pkill -9 -x FIRASim; /bin/sleep 1
/root/FIRASim/bin/FIRASim > /tmp/fira.log 2>&1 &
/bin/sleep 6
echo "FIRASim: $(pgrep -x FIRASim | wc -l) proceso(s)"

VSSL_TEAM_COLOR=blue VSSL_SIDE=left VSSL_COACH=heuristic "$ENGINE" > /tmp/side.log 2>&1 &
p=$!; /bin/sleep 5; kill $p; /bin/sleep 1
if grep -q "mitad del arco RIVAL" /tmp/side.log; then BLUE=right; YEL=left; else BLUE=left; YEL=right; fi
echo "lado del azul: $BLUE"

rm -f logs/serie.jsonl
i=1
while [ "$i" -le "$N" ]; do
  KICK=blue
  if [ $((i % 2)) -eq 0 ]; then KICK=yellow; fi
  rm -f "logs/serie_${i}_azul.csv" "logs/serie_${i}_amarillo.csv"
  VSSL_TEAM_COLOR=blue VSSL_SIDE=$BLUE VSSL_BIDIRECTIONAL=1 VSSL_VISION_NOISE=1 \
    VSSL_MATCH_LOG="logs/serie_${i}_azul.csv" "$ENGINE" > "/tmp/serie_azul_$i.log" 2>&1 &
  pb=$!
  VSSL_TEAM_COLOR=yellow VSSL_SIDE=$YEL VSSL_COACH=$RIVAL VSSL_BIDIRECTIONAL=1 VSSL_VISION_NOISE=1 \
    VSSL_MATCH_LOG="logs/serie_${i}_amarillo.csv" "$ENGINE" > "/tmp/serie_amarillo_$i.log" 2>&1 &
  py=$!
  /bin/sleep 4
  echo "=== partido $i (saque $KICK) $(date +%T) ==="
  "$DIRECTOR" --minutes "$MIN" --goals "$GOLES" --kickoff "$KICK" --out logs/serie.jsonl 2>&1 | grep -E "GOL|RESULTADO|free ball"
  kill $pb $py; /bin/sleep 2
  i=$((i + 1))
done
pkill -x FIRASim
echo "=== serie terminada $(date +%T) ==="
cat logs/serie.jsonl
