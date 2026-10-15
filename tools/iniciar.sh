#!/bin/bash
# Lanzador para demos: abre el simulador, los robots reales o una rutina desde un menú.
# Pensado para quien nunca usó el software (README, "Cómo iniciar").
#
#   tools/iniciar.sh                           menú
#   tools/iniciar.sh sim                       FIRASim (si no está abierto) y la interfaz
#   tools/iniciar.sh real                      comprueba cámara y base, y abre la interfaz
#   tools/iniciar.sh rutina [ID] [sim|real] [PCT]
#                                              una rutina de config/rutinas/ por consola
#
# Opciones: --frontal (robots solo de frente), --pausa (espera Enter antes de cerrar la
# ventana; la usa el acceso directo de tools/instalar_acceso.sh), --ayuda.
# Las variables VSSL_* que ya estén definidas se respetan (README, "Variables de entorno").

# ── Configuración ─────────────────────────────────────────────────────────────
# FIRASim que se abre si no está corriendo (por defecto, el de tiempo real, con el parche
# tools/firasim_cli.patch). En otro PC, la ruta a su FIRASim.
FIRASIM=${FIRASIM:-$HOME/sysmic/FIRASim/bin/FIRASim-realtime.sh}
# Equipo que controlamos: blue o yellow.
EQUIPO=${VSSL_TEAM_COLOR:-blue}
# 1: robots de dos caras (avanzan de espaldas cuando conviene). 0: solo de frente.
BIDIRECCIONAL=${VSSL_BIDIRECTIONAL:-1}
# 1: ruido de cámara en el simulador, como con la cámara real.
RUIDO_SIM=${VSSL_VISION_NOISE:-1}
# Puerto de la base station. Vacío: el único /dev/ttyUSB* o /dev/ttyACM* que haya.
PUERTO_BASE=${VSSL_BASESTATION_DEVICE:-}
# Velocidad por defecto de las rutinas con los robots reales (%).
VELOCIDAD_REAL=60
# ──────────────────────────────────────────────────────────────────────────────

set -u
cd "$(dirname "$(readlink -f "$0")")/.." || exit 1
mkdir -p logs
LOG=logs/iniciar.log
PAUSA=0
QUE_HACER=()

decir() { printf '%s\n' "$@"; }
fallar() {
  printf '%s\n' "$@" >&2
  exit 1
}
al_salir() {
  if [ "$PAUSA" = 1 ]; then
    echo
    read -rp "Presiona Enter para cerrar esta ventana. " _
  fi
}
trap al_salir EXIT

ayuda() {
  decir "Uso: tools/iniciar.sh [sim | real | rutina [ID] [sim|real] [PCT]] [--frontal]" \
    "  sin argumentos: un menú." \
    "  sim      abre FIRASim (si no está abierto) y la interfaz." \
    "  real     comprueba la cámara y la base station, y abre la interfaz." \
    "  rutina   corre una rutina de config/rutinas/ por consola." \
    "  --frontal  robots solo de frente (por defecto, de dos caras)."
}

# ── Entorno ───────────────────────────────────────────────────────────────────
if grep -qi microsoft /proc/version 2>/dev/null; then
  EN_WSL=1
  # En WSLg la interfaz dibuja por CPU: con todos los núcleos le quita tiempo al lazo.
  export LP_NUM_THREADS=${LP_NUM_THREADS:-4}
else
  EN_WSL=0
fi
# El acceso directo no siempre trae el PATH de la terminal.
command -v cargo >/dev/null || PATH="$HOME/.cargo/bin:$PATH"

case $EQUIPO in
  blue | azul | 0) EQUIPO=blue ;;
  yellow | amarillo | 1) EQUIPO=yellow ;;
  *) fallar "✗ EQUIPO='$EQUIPO' no es un equipo. Usa blue o yellow." ;;
esac
case $BIDIRECCIONAL in
  1 | true | on) BIDIRECCIONAL=1 ;;
  *) BIDIRECCIONAL=0 ;;
esac

texto_modo() {
  local equipo=azul caras="de dos caras"
  [ "$EQUIPO" = yellow ] && equipo=amarillo
  [ "$BIDIRECCIONAL" = 1 ] || caras="solo de frente"
  decir "Equipo $equipo, robots $caras."
}

# ── Piezas comunes ────────────────────────────────────────────────────────────
compilar() {
  command -v cargo >/dev/null ||
    fallar "✗ No encuentro cargo (Rust)." "Instálalo desde https://rustup.rs y vuelve a intentar."
  [ -x target/release/rustengine ] || decir "La primera compilación tarda varios minutos."
  printf 'Compilando.'
  # Un punto cada 3 s mientras compila; se corta solo si el script termina (Ctrl+C).
  (while sleep 3 && kill -0 $$ 2>/dev/null; do printf '.'; done) &
  local puntos=$! codigo
  cargo build --release >logs/compilar.log 2>&1
  codigo=$?
  kill "$puntos" 2>/dev/null
  if [ $codigo -ne 0 ]; then
    echo
    sed -n '/^error/,$p' logs/compilar.log | head -n 40
    fallar "" "✗ La compilación falló. El error está arriba (completo en logs/compilar.log)." \
      "Avisa al equipo de software."
  fi
  echo " Listo."
}

# Muestra la salida de skill_test sin el detalle técnico (comandos por tick, visión, lazo),
# salvo los avisos; $1, si va, es un aviso más que se oculta. El filtro ignora Ctrl+C y el
# cierre de la ventana, y si no puede escribir sigue leyendo: skill_test nunca encuentra la
# salida cerrada mientras manda ceros.
filtrar() {
  trap '' INT HUP
  awk -v ocultar="${1:-}" 'ocultar != "" && index($0, ocultar) { next }
       /✗|⚠|error/ { print; fflush(); next }
       /^\[(Vision|control_loop|Radio|FIRASim|BaseStation|skill_test)\]/ { next }
       { print; fflush() }'
  cat >/dev/null
}

# Dos programas a la vez mandarían a los mismos robots.
sin_otro_programa() {
  local otro
  otro=$(pgrep -x 'rustengine|skill_test|scenario|motion_bench' | head -n 1)
  [ -z "$otro" ] && return
  fallar "✗ Ya hay otro programa de los robots abierto: $(ps -o comm= -p "$otro") (PID $otro)." \
    "Ciérralo antes: los dos mandarían a los mismos robots."
}

# Espera a que FIRASim mande visión con nuestros robots (lo abre si hace falta).
abrir_firasim() {
  if pgrep -x FIRASim >/dev/null; then
    decir "FIRASim ya está abierto."
  else
    [ -x "$FIRASIM" ] ||
      fallar "✗ No encuentro FIRASim en $FIRASIM." "Cambia la ruta FIRASIM al principio de tools/iniciar.sh."
    decir "Abriendo FIRASim."
    local log=$PWD/logs/firasim.log
    (cd "$(dirname "$FIRASIM")" && nohup "./$(basename "$FIRASIM")" >"$log" 2>&1 &)
  fi
  local salida codigo
  for _ in $(seq 30); do
    salida=$(VSSL_VISION_SOURCE=firasim VSSL_TEAM_COLOR=$EQUIPO target/release/comprobar_vision --segundos 1)
    codigo=$?
    [ $codigo -eq 0 ] && return
    pgrep -x FIRASim >/dev/null || fallar "✗ FIRASim se cerró al abrir. Lo que dijo está en logs/firasim.log."
  done
  decir "$salida"
  case $codigo in
    3) fallar "✗ FIRASim no manda visión." "Revisa que no esté en pausa y que su visión vaya a 224.0.0.1:10002." ;;
    4) fallar "✗ FIRASim no tiene a nuestros robots en la cancha." ;;
    *) fallar "✗ No pude revisar la visión de FIRASim." ;;
  esac
}

# Cámara (paquetes y robots) y base station. Deja el puerto en PUERTO_BASE.
comprobar_real() {
  decir "Comprobando la cámara y la base station."
  local ok=1 codigo
  VSSL_VISION_SOURCE=sslvision VSSL_TEAM_COLOR=$EQUIPO target/release/comprobar_vision
  codigo=$?
  case $codigo in
    0) ;;
    3)
      QUE_HACER+=("Abre el programa de la cámara (vsss-vision-sysmic) y revisa que la cámara esté conectada."
        "Si la cámara está en otro PC, este PC tiene que estar en la misma red."
        "Con varias redes, elige la de la cámara: VSSL_MULTICAST_IFACE=<IP de este PC>.")
      [ $EN_WSL = 1 ] && QUE_HACER+=("En WSL, la red tiene que estar en modo mirrored (networkingMode=mirrored en el .wslconfig de Windows).")
      ;;
    4)
      QUE_HACER+=("Pon nuestros robots en la cancha, con el parche de colores a la vista."
        "Si nuestros robots son del otro color, cambia EQUIPO al principio de tools/iniciar.sh.")
      ;;
    *) QUE_HACER+=("Lee el mensaje de la cámara, arriba.") ;;
  esac
  [ $codigo -eq 0 ] || ok=0

  local puertos=() p
  if [ -n "$PUERTO_BASE" ]; then
    puertos=("$PUERTO_BASE")
  else
    for p in /dev/ttyUSB* /dev/ttyACM*; do [ -e "$p" ] && puertos+=("$p"); done
  fi
  if [ ${#puertos[@]} -gt 1 ]; then
    decir "Hay varios puertos USB serie. ¿Cuál es la base station?"
    PS3="Escribe el número: "
    select p in "${puertos[@]}"; do [ -n "$p" ] && break; done
    puertos=("$p")
  fi
  p=${puertos[0]:-}
  if [ -z "$p" ]; then
    decir "✗ No encuentro la base station: no hay ningún puerto USB serie (/dev/ttyUSB*, /dev/ttyACM*)."
    QUE_HACER+=("Conecta la base station por USB.")
    [ $EN_WSL = 1 ] && QUE_HACER+=("En WSL, pásala desde Windows (PowerShell): usbipd list, y luego usbipd attach --wsl --busid <BUSID>.")
    ok=0
  elif [ ! -e "$p" ]; then
    decir "✗ No existe el puerto $p de la base station."
    QUE_HACER+=("Conecta la base station o corrige VSSL_BASESTATION_DEVICE.")
    ok=0
  elif [ ! -r "$p" ] || [ ! -w "$p" ]; then
    decir "✗ No tengo permiso para usar $p."
    QUE_HACER+=("Agrégate al grupo dialout: sudo usermod -aG dialout $USER. Después cierra la sesión y vuelve a entrar.")
    ok=0
  else
    decir "✓ La base station está en $p."
    PUERTO_BASE=$p
  fi

  [ $ok = 1 ] && return
  decir "" "Qué hacer:"
  for p in "${QUE_HACER[@]}"; do decir "  - $p"; done
  fallar "" "✗ No abro nada con los robots reales hasta que todo esté en ✓."
}

correr_interfaz() {
  env "$@" VSSL_DEBUG_GUI=1 VSSL_TEAM_COLOR="$EQUIPO" VSSL_BIDIRECTIONAL="$BIDIRECCIONAL" \
    target/release/rustengine >"$LOG" 2>&1
  local codigo=$?
  if [ $codigo -eq 0 ]; then
    decir "La interfaz se cerró."
    return
  fi
  decir "✗ La interfaz se cerró con un error (código $codigo). Lo último del registro:"
  tail -n 15 "$LOG"
  fallar "El registro completo está en $PWD/$LOG."
}

# ── Modos ─────────────────────────────────────────────────────────────────────
modo_sim() {
  sin_otro_programa
  compilar
  abrir_firasim
  decir "" "Abriendo la interfaz con el simulador. $(texto_modo)" \
    "Barra espaciadora: detiene a todos los robots (con la ventana de la interfaz activa)." \
    "Para salir, cierra la ventana."
  correr_interfaz VSSL_VISION_SOURCE=firasim VSSL_RADIO_TARGET=firasim VSSL_VISION_NOISE="$RUIDO_SIM"
}

modo_real() {
  sin_otro_programa
  compilar
  comprobar_real
  decir "" "Todo listo para los robots reales. $(texto_modo)" \
    "⚠ Los robots empiezan a jugar apenas se abre la interfaz." \
    "⚠ La barra espaciadora es la parada de emergencia: detiene a todos los robots." \
    "  La ventana de la interfaz tiene que estar activa: haz click en ella primero." \
    "  También sirve el botón «Detener todo». Para seguir: «Soltar la parada»." \
    "⚠ Si el programa se cuelga, levanta los robots o desenchufa la base station."
  read -rp "Presiona Enter para abrir la interfaz (Ctrl+C para cancelar). " _
  correr_interfaz VSSL_VISION_SOURCE=sslvision VSSL_RADIO_TARGET=basestation \
    VSSL_BASESTATION_DEVICE="$PUERTO_BASE" VSSL_VISION_NOISE=0
}

modo_rutina() {
  local id=${1:-} donde=${2:-} velocidad=${3:-} lineas=() linea
  sin_otro_programa
  compilar
  if [ -z "$id" ]; then
    mapfile -t lineas < <(target/release/skill_test --listar-rutinas 2>/dev/null | sed -n 's/^  ✓ //p')
    [ ${#lineas[@]} -gt 0 ] || fallar "✗ No encuentro rutinas válidas en config/rutinas/."
    decir "" "¿Qué rutina?"
    PS3="Escribe el número: "
    select linea in "${lineas[@]%%:*}"; do [ -n "$linea" ] && break; done
    [ -n "$linea" ] || exit 0
    id=${linea%% *}
  fi
  if [ -z "$donde" ]; then
    decir "" "¿Dónde?"
    PS3="Escribe el número: "
    select donde in "En el simulador" "Con los robots reales"; do [ -n "$donde" ] && break; done
    case $donde in
      "En el simulador") donde=sim ;;
      "Con los robots reales") donde=real ;;
      *) exit 0 ;;
    esac
  fi
  if [ "$donde" = real ] && [ -z "$velocidad" ]; then
    read -rp "Velocidad en % (Enter = $VELOCIDAD_REAL): " velocidad
    velocidad=${velocidad:-$VELOCIDAD_REAL}
  fi
  if [ -n "$velocidad" ] && [[ ! $velocidad =~ ^([2-9]0|100)$ ]]; then
    fallar "✗ La velocidad va de 20 a 100, en pasos de 10."
  fi
  local opciones=(--team "$EQUIPO" --mode rutina --rutina "$id") codigo
  [ -n "$velocidad" ] && opciones+=(--velocidad "$velocidad")
  case $donde in
    sim)
      abrir_firasim
      decir "" "Rutina $id en el simulador. $(texto_modo)" \
        "Los robots aparecen en su inicio y la rutina arranca sola. Para detenerla: Ctrl+C."
      VSSL_TEAM_COLOR=$EQUIPO VSSL_BIDIRECTIONAL=$BIDIRECCIONAL VSSL_VISION_NOISE=$RUIDO_SIM \
        target/release/skill_test --transport firasim --vision sim --teletransportar "${opciones[@]}" 2>&1 |
        filtrar "fuera de la cancha" # el teleport saca de la cancha a los robots sin puesto
      codigo=${PIPESTATUS[0]}
      ;;
    real)
      comprobar_real
      decir "" "Rutina $id con los robots reales, al ${velocidad} %. $(texto_modo)" \
        "Los robots van solos a su inicio y la rutina arranca sola." \
        "⚠ Para detenerla: Ctrl+C en esta ventana." \
        "⚠ Si el programa se cuelga, levanta los robots o desenchufa la base station."
      read -rp "Presiona Enter para empezar (Ctrl+C para cancelar). " _
      VSSL_TEAM_COLOR=$EQUIPO VSSL_BIDIRECTIONAL=$BIDIRECCIONAL VSSL_BASESTATION_DEVICE=$PUERTO_BASE \
        VSSL_VISION_NOISE=0 target/release/skill_test --transport base-station --vision real "${opciones[@]}" 2>&1 | filtrar
      codigo=${PIPESTATUS[0]}
      ;;
    *) fallar "✗ '$donde' no es un lugar: usa sim o real." ;;
  esac
  case $codigo in
    0) decir "" "✓ La rutina terminó." ;;
    2)
      decir "" "La rutina se detuvo antes de terminar. La causa está arriba."
      exit 2
      ;;
    *) fallar "" "✗ La rutina no pudo correr. El motivo está arriba." ;;
  esac
}

menu() {
  local opcion
  decir "VSSS Sysmic. $(texto_modo)" "¿Qué quieres abrir?"
  PS3="Escribe el número y presiona Enter: "
  select opcion in "Simulador" "Robots reales" "Una rutina por consola" "Salir"; do
    case $opcion in
      Simulador) modo_sim ;;
      "Robots reales") modo_real ;;
      "Una rutina por consola") modo_rutina ;;
      Salir) ;;
      *)
        decir "Ese número no está en la lista."
        continue
        ;;
    esac
    break
  done
}

# ── Argumentos ────────────────────────────────────────────────────────────────
args=()
for a in "$@"; do
  case $a in
    --frontal) BIDIRECCIONAL=0 ;;
    --pausa) PAUSA=1 ;;
    -h | --help | --ayuda)
      ayuda
      exit 0
      ;;
    -*) fallar "✗ No conozco la opción $a." "$(ayuda)" ;;
    *) args+=("$a") ;;
  esac
done

case ${args[0]:-} in
  "") menu ;;
  sim) modo_sim ;;
  real) modo_real ;;
  rutina) modo_rutina "${args[@]:1}" ;;
  *) fallar "✗ No conozco '${args[0]}'." "$(ayuda)" ;;
esac
