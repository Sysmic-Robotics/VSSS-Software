#!/bin/bash
# Aceptación de los circuitos de un robot en FIRASim (change rutinas, fase 1): cada
# circuito con 10 repeticiones, teleport de la preparación y el proxy de ruido, en los dos
# modos (frontal y bidireccional). Criterio: cero corridas con falla.
#
#   tools/suite_rutinas.sh DIR
#
# Variables: REPEAT (repeticiones, 10 por defecto), RUTINAS (ids, por defecto los cinco
# circuitos), FIRASIM_BIN (ruta al binario de FIRASim: si está definida y FIRASim no
# corre, lo lanza), GUARDAR_CSV=1 (guarda el CSV de todas las corridas, comprimido),
# VELOCIDADES (porcentajes, p. ej. "100 60": cada rutina a cada velocidad, intercaladas, en
# frontal en ese orden y en bidireccional en el inverso; la corrida se llama <id>_bdN_vPCT.
# Sin VELOCIDADES, la velocidad por defecto de la simulación, 100 %).
#
# Falla: en los circuitos, toda corrida que no se completa. En las rutinas con pelota, todo
# aborto que no sea externo (usuario, HALT, STOP, parada, corte sin visión): las situaciones
# de juego se resuelven sin abortar. Se clasifica por el campo `tipo_aborto` del resumen, no
# por la frase. Las rutinas con pelota suman las columnas de pases (logrados/intentados),
# rescates (recuperaciones + cambios de rol), despegues y reposiciones; su aceptación se
# corre aparte:
#   RUTINAS=pase_lado_a_lado VELOCIDADES="100 60" tools/suite_rutinas.sh DIR
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

errores=0
for bd in 0 1; do
  vels=${VELOCIDADES:-}
  if [ $bd -eq 1 ] && [ -n "$vels" ]; then vels=$(echo "$vels" | tr ' ' '\n' | tac | tr '\n' ' '); fi
  for r in $RUTINAS; do
    for v in ${vels:-def}; do
      if [ "$v" = def ]; then base="$OUT/${r}_bd$bd"; vel=(); else base="$OUT/${r}_bd${bd}_v$v"; vel=(--velocidad "$v"); fi
      VSSL_VISION_NOISE=1 VSSL_BIDIRECTIONAL=$bd target/release/skill_test \
        --transport firasim --vision sim --team blue --mode rutina --rutina "$r" \
        --repeticiones "$REP" --teletransportar "${vel[@]}" --log "$base.csv" \
        2> >(grep -av 'comandos enviados\|✓ DETECTION\|\] Stats:' > "$base.err")
      code=$?
      if [ $code -eq 0 ] && [ "${GUARDAR_CSV:-0}" != 1 ]; then rm -f "$base.csv"; else gzip -f "$base.csv"; fi
      echo "$(date +%T) $(basename "$base") exit=$code" | tee -a "$OUT/progress"
      # 1 = error sin resumen (2 = abortada: la clasifica el resumen).
      [ $code -eq 1 ] && errores=$((errores + 1))
    done
  done
done

python3 - "$OUT" "$errores" <<'PY'
import glob, json, os, sys
EXTERNOS = {"usuario", "halt", "stop", "parada", "corte_sin_vision"}
filas, fallas = [], int(sys.argv[2])
for p in sorted(glob.glob(os.path.join(sys.argv[1], "*.resumen.json"))):
    j = json.load(open(p))
    modo = "bidireccional" if j["corrida"]["bidireccional"] else "frontal"
    err = [vp["error_medio_m"] for v in j["vueltas"] for vp in v["puestos"].values() if vp["error_medio_m"] is not None]
    p = j.get("pases")
    pases = f"{p['logrados']}/{p['intentados']}" if p else "-"
    if j["resultado"] in ("completada", "detenida"):
        falla = False
    elif p is not None:
        falla = j.get("tipo_aborto") not in EXTERNOS
    else:
        falla = True
    ev = j.get("eventos") or {}
    if p is not None:
        resc = str(p["recuperaciones"] + ev.get("cambios_de_rol", 0))
        desp, repo = str(ev.get("despegues", "-")), str(ev.get("reposiciones", "-"))
    else:
        resc = desp = repo = "-"
    fallas += falla
    filas.append((j["rutina"], modo, j["velocidad_pct"], j["resultado"], j["vueltas_completas"], j["duracion_media_s"],
                  sum(err) / len(err) if err else None, sum(j["escapes"].values()), pases, resc, desp, repo,
                  "sí" if falla else "", j.get("tipo_aborto") or "", j["causa"] or ""))
print(f"{'rutina':<16} {'modo':<14} {'vel':>4} {'resultado':<11} {'vueltas':>7} {'s/vuelta':>8} {'err cm':>6} {'esc':>4} "
      f"{'pases':>7} {'resc':>4} {'desp':>4} {'repo':>4} {'falla':>5}  {'tipo':<18} causa")
for r, m, vel, res, n, d, e, esc, pases, resc, desp, repo, falla, tipo, causa in filas:
    d = f"{d:8.2f}" if d is not None else "       -"
    e = f"{e * 100:6.1f}" if e is not None else "     -"
    print(f"{r:<16} {m:<14} {vel:>3}% {res:<11} {n:>7} {d} {e} {esc:>4} {pases:>7} {resc:>4} {desp:>4} {repo:>4} {falla:>5}  {tipo:<18} {causa}")
print(f"{fallas} corridas con falla (criterio: 0; con pelota, todo aborto no externo); resultados en {sys.argv[1]}")
sys.exit(1 if fallas else 0)
PY
