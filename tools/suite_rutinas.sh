#!/bin/bash
# Aceptación de los circuitos de un robot en FIRASim (change rutinas, fase 1): cada
# circuito con 10 repeticiones, teleport de la preparación y el proxy de ruido, en los dos
# modos (frontal y bidireccional). Criterio: cero abortos en las 10 corridas.
#
#   tools/suite_rutinas.sh DIR
#
# Variables: REPEAT (repeticiones, 10 por defecto), RUTINAS (ids, por defecto los cinco
# circuitos), FIRASIM_BIN (ruta al binario de FIRASim: si está definida y FIRASim no
# corre, lo lanza), GUARDAR_CSV=1 (guarda el CSV de todas las corridas, comprimido).
#
# FIRASim tiene que estar en tiempo real (FIRASim-realtime.sh). No correr la GUI ni otro
# engine a la vez: comandan los mismos robots. Por corrida quedan el resumen
# (DIR/<id>_bdN.resumen.json) y la salida filtrada (.err); el CSV de partido, comprimido,
# solo de las que fallan (o de todas con GUARDAR_CSV=1).
set -u
OUT=${1:?uso: tools/suite_rutinas.sh DIR}
REP=${REPEAT:-10}
RUTINAS=${RUTINAS:-"triangulo ida_vuelta zigzag ocho cuadrado_giros"}
cd "$(dirname "$0")/.." || exit 1

if pgrep -x rustengine >/dev/null || pgrep -x skill_test >/dev/null || pgrep -x scenario >/dev/null; then
  echo "hay otro engine corriendo (rustengine/GUI, skill_test o scenario): ciérralo antes de la suite" >&2
  exit 1
fi
cargo build --release --bin skill_test || exit 1
mkdir -p "$OUT"

if [ -n "${FIRASIM_BIN:-}" ] && ! pgrep -x FIRASim >/dev/null; then
  (cd "$(dirname "$FIRASIM_BIN")" && nohup "./$(basename "$FIRASIM_BIN")" >> "$OUT/firasim.log" 2>&1 &)
  sleep 8
fi

fallas=0
for bd in 0 1; do
  for r in $RUTINAS; do
    base="$OUT/${r}_bd$bd"
    VSSL_VISION_NOISE=1 VSSL_BIDIRECTIONAL=$bd target/release/skill_test \
      --transport firasim --vision sim --team blue --mode rutina --rutina "$r" \
      --repeticiones "$REP" --teletransportar --log "$base.csv" \
      2> >(grep -av 'comandos enviados\|✓ DETECTION\|\] Stats:' > "$base.err")
    code=$?
    if [ $code -eq 0 ] && [ "${GUARDAR_CSV:-0}" != 1 ]; then rm -f "$base.csv"; else gzip -f "$base.csv"; fi
    echo "$(date +%T) $r bd$bd exit=$code" | tee -a "$OUT/progress"
    [ $code -eq 0 ] || fallas=$((fallas + 1))
  done
done

python3 - "$OUT" <<'PY'
import glob, json, os, sys
filas = []
for p in sorted(glob.glob(os.path.join(sys.argv[1], "*.resumen.json"))):
    j = json.load(open(p))
    modo = "bidireccional" if j["corrida"]["bidireccional"] else "frontal"
    err = [vp["error_medio_m"] for v in j["vueltas"] for vp in v["puestos"].values() if vp["error_medio_m"] is not None]
    filas.append((j["rutina"], modo, j["resultado"], j["vueltas_completas"], j["duracion_media_s"],
                  sum(err) / len(err) if err else None, sum(j["escapes"].values()), j["causa"] or ""))
print(f"{'rutina':<16} {'modo':<14} {'resultado':<11} {'vueltas':>7} {'s/vuelta':>8} {'err cm':>6} {'esc':>4}  causa")
for r, m, res, n, d, e, esc, causa in filas:
    d = f"{d:8.2f}" if d is not None else "       -"
    e = f"{e * 100:6.1f}" if e is not None else "     -"
    print(f"{r:<16} {m:<14} {res:<11} {n:>7} {d} {e} {esc:>4}  {causa}")
PY
echo "$fallas corridas con falla (criterio: 0); resultados en $OUT"
[ $fallas -eq 0 ]
