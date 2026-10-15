#!/bin/bash
# Crea el acceso directo al lanzador (tools/iniciar.sh) en el escritorio y en el menú de
# aplicaciones, con el ícono de assets/iconos/ (vsss.png en Linux, vsss.ico en Windows; es
# el mismo dibujo).
#
#   tools/instalar_acceso.sh            crea los accesos
#   tools/instalar_acceso.sh --quitar   los borra
#
# Ubuntu: un .desktop en el menú (~/.local/share/applications) y otro en el escritorio, que
# abren el lanzador en una terminal (Terminal=true).
# WSL (WSLg): Windows no abre archivos .desktop y WSL no trae una terminal de Linux, así que
# crea accesos de Windows (.lnk) en el escritorio y en el menú Inicio. Abren el lanzador en
# la terminal de Windows con wsl.exe, y la interfaz se ve por WSLg.
set -u
cd "$(dirname "$(readlink -f "$0")")/.." || exit 1
REPO=$PWD
NOMBRE="VSSS Sysmic"

quitar=0
case ${1:-} in
  "") ;;
  --quitar) quitar=1 ;;
  *)
    echo "Uso: tools/instalar_acceso.sh [--quitar]"
    exit 1
    ;;
esac

# ── WSL: accesos de Windows ───────────────────────────────────────────────────
if grep -qi microsoft /proc/version 2>/dev/null; then
  [ -n "${WSL_DISTRO_NAME:-}" ] || {
    echo "✗ No sé el nombre de esta distribución de WSL (WSL_DISTRO_NAME)."
    exit 1
  }
  case $REPO in *"'"*)
    echo "✗ La ruta del repositorio tiene una comilla ('): muévelo a otra carpeta."
    exit 1
    ;;
  esac
  # PowerShell desde una carpeta de Windows (desde una de Linux, avisa que no la soporta),
  # con la salida en UTF-8 para que las tildes lleguen bien.
  powershell() {
    (cd /mnt/c && powershell.exe -NoProfile -NonInteractive -Command \
      "[Console]::OutputEncoding = New-Object System.Text.UTF8Encoding \$false; $1") | tr -d '\r'
  }
  carpeta_icono=$(powershell '[Environment]::GetFolderPath("LocalApplicationData")')
  [ -n "$carpeta_icono" ] || {
    echo "✗ No pude preguntarle a Windows por sus carpetas (powershell.exe)."
    exit 1
  }
  carpeta_icono+='\VSSS-Sysmic'
  if [ $quitar = 1 ]; then
    powershell "foreach (\$d in [Environment]::GetFolderPath('Desktop'), [Environment]::GetFolderPath('Programs')) {
      \$f = Join-Path \$d '$NOMBRE.lnk'; if (Test-Path \$f) { Remove-Item \$f; \"✓ Borrado: \$f\" } }"
    rm -rf "$(wslpath -u "$carpeta_icono")"
    exit 0
  fi
  mkdir -p "$(wslpath -u "$carpeta_icono")" &&
    cp assets/iconos/vsss.ico "$(wslpath -u "$carpeta_icono")/" || exit 1
  powershell "\$sh = New-Object -ComObject WScript.Shell
    foreach (\$d in [Environment]::GetFolderPath('Desktop'), [Environment]::GetFolderPath('Programs')) {
      \$l = \$sh.CreateShortcut((Join-Path \$d '$NOMBRE.lnk'))
      \$l.TargetPath = \"\$env:SystemRoot\System32\wsl.exe\"
      \$l.Arguments = '-d $WSL_DISTRO_NAME --cd \"$REPO\" -- ./tools/iniciar.sh --pausa'
      \$l.IconLocation = '$carpeta_icono\vsss.ico,0'
      \$l.Description = 'Simulador, robots reales y rutinas'
      \$l.Save()
      \"✓ Acceso creado: \$(\$l.FullName)\"
    }" || exit 1
  echo "Ábrelo desde el escritorio o desde el menú Inicio (busca «$NOMBRE»)."
  exit 0
fi

# ── Ubuntu: archivos .desktop ─────────────────────────────────────────────────
menu=${XDG_DATA_HOME:-$HOME/.local/share}/applications/vsss-sysmic.desktop
escritorio=$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")
# Sin carpeta de escritorio, xdg-user-dir da el home.
[ "${escritorio%/}" != "$HOME" ] || escritorio=""
if [ $quitar = 1 ]; then
  for f in "$menu" ${escritorio:+"$escritorio/vsss-sysmic.desktop"}; do
    [ -e "$f" ] && rm -f "$f" && echo "✓ Borrado: $f"
  done
  exit 0
fi

mkdir -p "$(dirname "$menu")"
cat >"$menu" <<EOF
[Desktop Entry]
Type=Application
Name=$NOMBRE
Comment=Simulador, robots reales y rutinas
Exec="$REPO/tools/iniciar.sh" --pausa
Path=$REPO
Icon=$REPO/assets/iconos/vsss.png
Terminal=true
Categories=Development;Education;
EOF
chmod +x "$menu"
echo "✓ Acceso creado en el menú de aplicaciones: $menu"
if [ -n "$escritorio" ] && [ -d "$escritorio" ]; then
  cp "$menu" "$escritorio/"
  chmod +x "$escritorio/vsss-sysmic.desktop"
  # Ubuntu solo abre un .desktop del escritorio marcado como confiable.
  gio set "$escritorio/vsss-sysmic.desktop" metadata::trusted true 2>/dev/null
  echo "✓ Acceso creado en el escritorio: $escritorio/vsss-sysmic.desktop"
  echo "  Si el ícono sale con una cruz: click derecho → «Permitir ejecutar»."
else
  echo "No encontré la carpeta del escritorio: el acceso quedó solo en el menú."
fi
echo "Ábrelo desde el menú de aplicaciones (busca «$NOMBRE»)."
