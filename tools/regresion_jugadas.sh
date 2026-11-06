#!/usr/bin/env bash
# No-regresión de saques y penales: ¿el candidato sigue metiendo con su saque, defendiendo el
# saque rival, convirtiendo y atajando penales como la base? Corre en paralelo 4 baterías
# (tools/bateria.sh), cada una con su FIRASim headless en el índice de puertos indicado:
#   k0    saques   candidato (azul) vs base (amarillo)
#   k0+1  saques   base (azul) vs base (amarillo): referencia
#   k0+2  penales  candidato vs base
#   k0+3  penales  base vs base: referencia
# y compara al azul de cada par. Regresión: el candidato es peor que la referencia por más de
# 2 errores estándar (z de dos proporciones, error estándar binomial con la proporción conjunta).
#   bash tools/regresion_jugadas.sh <candidato.json> [base.json] [--saques N] [--penales N] [--k0 K]
# Por defecto: base logs/annealing/real_B.json, 240 saques, 240 penales, k0 0 (puertos de visión
# 10600..10630). Como root en WSL. Resultados en /root/regresion/<candidato>_k<k0>/ (resumen.txt).
# Código de salida: 0 sin regresión, 1 regresión, 2 uso, 3 baterías incompletas.
set -u
DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$DIR/.." && pwd)
UMBRAL=2

uso() { sed -n '2,14p' "$0" >&2; exit 2; }
cand=; base=; saques=240; penales=240; k0=0
while [ $# -gt 0 ]; do
  case $1 in
    --saques|--penales|--k0)
      [ $# -ge 2 ] || uso
      case $1 in --saques) saques=$2 ;; --penales) penales=$2 ;; --k0) k0=$2 ;; esac
      shift 2 ;;
    -h|--help) uso ;;
    --*) echo "regresion: opción desconocida: $1" >&2; uso ;;
    *) if [ -z "$cand" ]; then cand=$1; elif [ -z "$base" ]; then base=$1; else uso; fi; shift ;;
  esac
done
[ -n "$cand" ] || uso
base=${base:-logs/annealing/real_B.json}
for v in "$saques" "$penales" "$k0"; do
  case $v in ''|*[!0-9]*) echo "regresion: --saques, --penales y --k0 llevan un entero" >&2; exit 2 ;; esac
done
if [ $((k0 + 3)) -gt 8 ]; then echo "regresion: --k0 de 0 a 5 (usa los índices k0..k0+3, hasta 8)" >&2; exit 2; fi
if [ "$saques" -lt 2 ] || [ "$penales" -lt 2 ]; then echo "regresion: al menos 2 saques y 2 penales" >&2; exit 2; fi
ruta() { if [ -f "$1" ]; then realpath "$1"; elif [ -f "$REPO/$1" ]; then realpath "$REPO/$1"; else return 1; fi; }
cand_abs=$(ruta "$cand") || { echo "regresion: no existe $cand" >&2; exit 2; }
base_abs=$(ruta "$base") || { echo "regresion: no existe $base" >&2; exit 2; }

# Antes de lanzar nada: los 12 puertos libres (bateria.sh también lo revisa, pero así no
# quedan 3 baterías corriendo minutos con la cuarta caída).
for k in $(seq "$k0" $((k0 + 3))); do
  for p in $((10600 + 10 * k)) $((20600 + 10 * k)) $((10690 + 10 * k)); do
    if ss -Huln 2>/dev/null | awk '{print $4}' | grep -qE "[:.]$p\$"; then
      echo "regresion: el puerto $p (k=$k) ya está en uso; elegir otro --k0" >&2; exit 2
    fi
  done
done

nombre=$(basename "$cand_abs" .json); nombre=${nombre//[^A-Za-z0-9_.-]/_}
run=${nombre}_k$k0
R=/root/regresion/$run; mkdir -p "$R"
subs=(saques_cand saques_base penales_cand penales_base)
azules=("$cand_abs" "$base_abs" "$cand_abs" "$base_abs")
modos=(--kickoffs --kickoffs --penalties --penalties)
ns=("$saques" "$saques" "$penales" "$penales")

pids=()
trap 'echo "regresion: cortando las baterías..." >&2; kill "${pids[@]}" 2>/dev/null; wait; exit 130' INT TERM
echo "Regresión de jugadas: $cand vs base $base · $saques saques y $penales penales por batería"
echo "4 baterías en paralelo (k $k0..$((k0 + 3))) desde $(date +%T); carpeta $R"
for i in 0 1 2 3; do
  bash "$DIR/bateria.sh" "$run/${subs[i]}" "${azules[i]}" "$base_abs" "${modos[i]}" "${ns[i]}" $((k0 + i)) \
    > "$R/${subs[i]}.txt" 2>&1 &
  pids+=($!)
done
wait
trap - INT TERM
for s in "${subs[@]}"; do cat "$R/$s.txt"; done

# Del res.jsonl de cada batería. Saques: [[saques, goles del que saca, goles del otro]] por
# equipo (azul, amarillo). Penales: goles y penales tirados por equipo.
saques_de() { sed -n 's/.*"kickoffs":\[\[\([0-9]*\),\([0-9]*\),\([0-9]*\)\],\[\([0-9]*\),\([0-9]*\),\([0-9]*\)\]\].*/\1 \2 \3 \4 \5 \6/p' "$R/$1/res.jsonl" 2>/dev/null | tail -1; }
penales_de() { sed -n 's/^{"blue":\([0-9]*\),"yellow":\([0-9]*\),.*"penalties":\[\([0-9]*\),\([0-9]*\)\].*/\1 \2 \3 \4/p' "$R/$1/res.jsonl" 2>/dev/null | tail -1; }
# Tiempo medio (s) entre el GAME_ON y el gol del azul con su propio saque.
tiempo_de() { grep -oE 'saque [0-9]+ de azul: gol de azul a los [0-9.]+ s' "$R/$1/director.log" 2>/dev/null \
  | awk '{s += $(NF-1); n++} END {if (n) printf "%.1f", s / n; else printf "-"}'; }

faltan=()
read -r sc_n sc_g sc_c sc_rn sc_rg sc_rc <<< "$(saques_de saques_cand)"
read -r sb_n sb_g sb_c sb_rn sb_rg sb_rc <<< "$(saques_de saques_base)"
read -r pc_g pc_rg pc_n pc_rn <<< "$(penales_de penales_cand)"
read -r pb_g pb_rg pb_n pb_rn <<< "$(penales_de penales_base)"
[ -n "${sc_n:-}" ] || faltan+=(saques_cand)
[ -n "${sb_n:-}" ] || faltan+=(saques_base)
[ -n "${pc_n:-}" ] || faltan+=(penales_cand)
[ -n "${pb_n:-}" ] || faltan+=(penales_base)

# Filas: etiqueta <TAB> +1 si más es mejor, -1 si menos es mejor <TAB> x1 n1 (candidato) x2 n2 (referencia).
{
  printf 'Goles con saque propio\t1\t%s\t%s\t%s\t%s\n' "${sc_g:-}" "${sc_n:-}" "${sb_g:-}" "${sb_n:-}"
  printf 'Goles en contra tras saque rival\t-1\t%s\t%s\t%s\t%s\n' "${sc_rg:-}" "${sc_rn:-}" "${sb_rg:-}" "${sb_rn:-}"
  printf 'Penales convertidos\t1\t%s\t%s\t%s\t%s\n' "${pc_g:-}" "${pc_n:-}" "${pb_g:-}" "${pb_n:-}"
  printf 'Penales convertidos por el rival\t-1\t%s\t%s\t%s\t%s\n' "${pc_rg:-}" "${pc_rn:-}" "${pb_rg:-}" "${pb_rn:-}"
} > "$R/filas.tsv"

{
  echo
  echo "Candidato $cand · referencia: base vs base ($base)"
  awk -F'\t' -v umbral=$UMBRAL -v t1="$(tiempo_de saques_cand)" -v t2="$(tiempo_de saques_base)" '
    function seg(t) { return t == "-" ? "sin goles" : t " s" }
    function celda(x, n) { return n > 0 ? sprintf("%3d/%-3d %5.1f%%", x, n, 100 * x / n) : sprintf("%14s", "sin datos") }
    BEGIN { printf "%-34s %14s   %14s %9s %7s\n", "", "candidato", "referencia", "dif", "z" }
    {
      x1 = $3 + 0; n1 = $4 + 0; x2 = $5 + 0; n2 = $6 + 0
      if (n1 > 0 && n2 > 0) {
        p1 = x1 / n1; p2 = x2 / n2; p = (x1 + x2) / (n1 + n2)
        se = sqrt(p * (1 - p) * (1 / n1 + 1 / n2))
        z = se > 0 ? $2 * (p2 - p1) / se : 0     # z > 0: el candidato es peor
        marca = z > umbral ? "  PEOR" : (z < -umbral ? "  mejor" : "")
        printf "%-34s %s   %s %+6.1f pp %+6.2f%s\n", $1, celda(x1, n1), celda(x2, n2), 100 * (p1 - p2), z, marca
        nombre = tolower(substr($1, 1, 1)) substr($1, 2)
        if (z > umbral) peores = peores (peores == "" ? "" : ", ") nombre sprintf(" (z = %.2f)", z)
        # La referencia ya en el peor valor posible: el candidato no puede quedar por debajo.
        if (($2 > 0 && x2 == 0) || ($2 < 0 && x2 == n2)) ciegas = ciegas (ciegas == "" ? "" : ", ") nombre
      } else printf "%-34s %s   %s\n", $1, celda(x1, n1), celda(x2, n2)
    }
    END {
      printf "%-34s %14s   %14s   (informativo)\n", "Tiempo medio al gol, saque propio", seg(t1), seg(t2)
      print "z: diferencia en errores estándar, positiva si el candidato es peor; regresión si z > " umbral "."
      if (ciegas != "") print "Ojo: en " ciegas " la referencia ya tiene el peor valor posible: ahí no se puede detectar una regresión."
      if (peores != "") { print "Veredicto: REGRESIÓN en " peores "."; exit 1 }
    }' "$R/filas.tsv"
  rc=$?
  if [ "$rc" -eq 0 ] && [ ${#faltan[@]} -gt 0 ]; then
    echo "Veredicto: incompleto, faltan baterías: ${faltan[*]} (ver $R/<batería>/director.log)"; rc=3
  elif [ "$rc" -eq 0 ]; then
    echo "Veredicto: sin regresión detectable."
  fi
  echo "Duración: $((SECONDS / 60)) min $((SECONDS % 60)) s"
  exit $rc
} | tee "$R/resumen.txt"
exit "${PIPESTATUS[0]}"
