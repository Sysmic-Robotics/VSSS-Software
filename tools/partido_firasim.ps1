# Lanza un partido de prueba en FIRASim (WSL) con cada componente en su propia ventana:
#   1) FIRASim  2) engine azul (coach heuristico)  3) engine amarillo (rival)  4) arbitro de operador
#
# Uso (PowerShell, desde cualquier carpeta):
#   powershell -ExecutionPolicy Bypass -File D:\Proyectos\VSSS-Software\tools\partido_firasim.ps1
#   ... -SinRuido        (sin proxy de ruido de camara)
#   ... -SinAmarillo     (solo el equipo azul)
#   ... -SinArbitro      (no abre la consola del arbitro)
#   ... -Rival heuristic (amarillo tambien con el coach nuevo; default rule_based)
#
# Cada ventana es un `wsl.exe bash -lc` independiente: cerrar una no cierra las demas.
# Los CSV quedan en D:\Proyectos\VSSS-Software\logs\ (partido_azul.csv, partido_amarillo.csv).
# (Archivo solo ASCII a proposito: PowerShell 5.1 lee los .ps1 sin BOM como ANSI.)
param(
    [switch]$SinRuido,
    [switch]$SinAmarillo,
    [switch]$SinArbitro,
    # Lado donde FIRASim puso al AZUL (mira la cancha: el arco que defiende el azul).
    # Con el lado al reves cada equipo ataca su propio arco y la fisica de FIRASim revienta.
    [switch]$AzulDerecha,
    [string]$Rival = "rule_based",
    [string]$Repo = "/mnt/d/Proyectos/VSSS-Software"
)

$ErrorActionPreference = "Stop"
$target = '$HOME/vsss-target'
$ruido = if ($SinRuido) { "0" } else { "1" }
$ladoAzul = if ($AzulDerecha) { "right" } else { "left" }
$ladoAmarillo = if ($AzulDerecha) { "left" } else { "right" }

function Start-WslWindow([string]$titulo, [string]$comando) {
    # El `read` final deja la ventana abierta para leer el log cuando el proceso termina.
    $inner = "echo '== $titulo =='; $comando; echo; echo '[$titulo termino - Enter para cerrar]'; read"
    Start-Process -FilePath "wsl.exe" -ArgumentList @("bash", "-lc", ('"' + $inner.Replace('"', '\"') + '"'))
}

Write-Host "1/4 FIRASim"
Start-WslWindow "FIRASim" '~/FIRASim/bin/FIRASim'
Start-Sleep -Seconds 4

Write-Host "2/4 engine azul (heuristic, lado $ladoAzul, ruido=$ruido, GUI, log)"
Start-WslWindow "AZUL" ("cd $Repo && CARGO_TARGET_DIR=$target VSSL_TEAM_COLOR=blue VSSL_SIDE=$ladoAzul VSSL_BIDIRECTIONAL=1 " +
    "VSSL_VISION_NOISE=$ruido VSSL_DEBUG_GUI=1 VSSL_MATCH_LOG=logs/partido_azul.csv cargo run --release")

if (-not $SinAmarillo) {
    Write-Host "3/4 engine amarillo ($Rival, lado $ladoAmarillo, log)"
    Start-WslWindow "AMARILLO" ("cd $Repo && CARGO_TARGET_DIR=$target VSSL_TEAM_COLOR=yellow VSSL_SIDE=$ladoAmarillo VSSL_COACH=$Rival " +
        "VSSL_VISION_NOISE=$ruido VSSL_MATCH_LOG=logs/partido_amarillo.csv cargo run --release")
}

if (-not $SinArbitro) {
    Write-Host "4/4 arbitro de operador (k b = kickoff azul, go = silbato, b 1 = free ball Q1, s = stop, h = halt, q = salir)"
    Start-WslWindow "ARBITRO" "cd $Repo && python3 tools/referee_cli.py"
}

Write-Host ""
Write-Host "Listo. Al terminar, metricas:"
Write-Host "  python D:\Proyectos\VSSS-Software\tools\match_metrics.py D:\Proyectos\VSSS-Software\logs\partido_azul.csv"
