#!/usr/bin/env bash
# Una batería corta sin ventana: FIRASim headless a 480 fps, dos engines (azul y amarillo) con
# sus JSON de parámetros y el director en modo --kickoffs N o --penalties N.
#   bash tools/bateria.sh <tag> <azul.json> <amarillo.json> <--kickoffs|--penalties> <N> <k>
# k = índice de puertos, de 0 a 8: visión 10600+10k, comandos 20600+10k, árbitro 10690+10k
# (con k >= 9 la visión choca con el árbitro de k-9). Corre como root en WSL.
# Carpeta de trabajo: /root/regresion/<tag> (se borra al empezar). Imprime
# "<tag>: <línea SAQUES o PENALES del director>"; el resultado queda en <carpeta>/res.jsonl.
set -u
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
B=/root/vsss-target/release
FIRASIM=${FIRASIM_BIN:-/root/FIRASim/bin/FIRASim}

if [ $# -ne 6 ]; then sed -n '2,8p' "$0" >&2; exit 2; fi
tag=$1; modo=$4; n=$5; k=$6
case $tag in ''|/*|*..*) echo "bateria: tag inválido: '$tag'" >&2; exit 2 ;; esac
case $modo in --kickoffs|--penalties) ;; *) echo "bateria: modo '$modo' (usar --kickoffs o --penalties)" >&2; exit 2 ;; esac
case $n in ''|*[!0-9]*) echo "bateria: N debe ser un entero: '$n'" >&2; exit 2 ;; esac
case $k in ''|*[!0-9]*) echo "bateria: k debe ser un entero: '$k'" >&2; exit 2 ;; esac
if [ "$k" -gt 8 ]; then echo "bateria: k=$k fuera de rango (0 a 8)" >&2; exit 2; fi

# Rutas de parámetros: relativas a donde se llama o, si no, al repo.
ruta() { if [ -f "$1" ]; then realpath "$1"; elif [ -f "$REPO/$1" ]; then realpath "$REPO/$1"; else return 1; fi; }
az=$(ruta "$2") || { echo "bateria: no existe $2" >&2; exit 2; }
am=$(ruta "$3") || { echo "bateria: no existe $3" >&2; exit 2; }

VP=$((10600 + 10 * k)); CP=$((20600 + 10 * k)); RP=$((10690 + 10 * k))
# Un puerto ya tomado mezclaría esta batería con otra (la visión es multicast: no da error).
for p in $VP $CP $RP; do
  if ss -Huln 2>/dev/null | awk '{print $4}' | grep -qE "[:.]$p\$"; then
    echo "bateria $tag: el puerto $p ya está en uso (¿otra batería o serie con k=$k?); usar otro k" >&2
    exit 3
  fi
done

cd "$REPO" || exit 1   # config/team_params.json es relativo al directorio de trabajo
W=/root/regresion/$tag; rm -rf "$W"; mkdir -p "$W"

fp=; pb=; py=; pd=
limpiar() {
  trap - EXIT INT TERM
  [ -n "$pd" ] && kill "$pd" 2>/dev/null
  [ -n "$pb" ] && kill "$pb" 2>/dev/null
  [ -n "$py" ] && kill "$py" 2>/dev/null
  sleep 1
  if [ -n "$fp" ]; then kill "$fp" 2>/dev/null; sleep 1; kill -9 "$fp" 2>/dev/null; fi
  fp=; pb=; py=; pd=
  # Sin CSV: pesan ~1 GB por batería y llenaron el disco el 5-nov. Solo los de esta batería:
  # otras pueden estar corriendo en paralelo.
  find "$W" -name '*.csv' -delete 2>/dev/null
}
trap limpiar EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

"$FIRASIM" --headless --fps 480 --vision-port $VP --cmd-port $CP > "$W/fira.log" 2>&1 &
fp=$!; sleep 6
if ! kill -0 "$fp" 2>/dev/null; then
  echo "$tag: ERROR: FIRASim no arrancó; últimas líneas de $W/fira.log:"; tail -5 "$W/fira.log" | sed 's/^/  /'
  fp=; exit 1
fi
eng() {  # eng <color> <side> <preset> <seed>
  VSSL_TEAM_COLOR=$1 VSSL_SIDE=$2 VSSL_COACH=heuristic VSSL_BIDIRECTIONAL=1 VSSL_VISION_NOISE=1 VSSL_FRAME_CLOCK=1 \
  VSSL_NOISE_SEED=$4 VSSL_MATCH_LOG="$W/$1.csv" VSSL_REFEREE_ADDR=224.5.23.2:$RP VSSL_VISION_PORT=$VP VSSL_CMD_PORT=$CP \
  VSSL_PARAMS="$3" $B/rustengine > "$W/$1.log" 2>&1 &
}
eng blue right "$az" $((1000 + k)); pb=$!
eng yellow left "$am" $((2000 + k)); py=$!
sleep 4
for c in blue yellow; do
  if [ "$c" = blue ]; then p=$pb; else p=$py; fi
  if ! kill -0 "$p" 2>/dev/null; then
    echo "$tag: ERROR: el engine $c terminó al arrancar; últimas líneas de $W/$c.log:"; tail -5 "$W/$c.log" | sed 's/^/  /'
    exit 1
  fi
done
# En segundo plano y con wait: así una señal corta la espera y limpia enseguida.
VSSL_REFEREE_ADDR=224.5.23.2:$RP $B/match_director $modo $n --kickoff blue --speed 8 --vision-port $VP --cmd-port $CP --out "$W/res.jsonl" > "$W/director.log" 2>&1 &
pd=$!
wait "$pd"; rc=$?; pd=
limpiar
linea=$(grep -E '^(SAQUES|PENALES)' "$W/director.log")
if [ -z "$linea" ]; then
  echo "$tag: ERROR: el director terminó sin resumen (código $rc); últimas líneas de $W/director.log:"
  tail -5 "$W/director.log" | sed 's/^/  /'
  exit 1
fi
echo "$tag: $linea"
