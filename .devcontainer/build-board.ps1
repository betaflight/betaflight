#Requires -Version 5.1
<#
.SYNOPSIS
    Build Betaflight firmware for any board target using the devcontainer image.

.DESCRIPTION
    Requires Docker Desktop (with WSL2 backend on Windows).
    Optionally applies a local board config overlay, then runs make CONFIG=<target>
    with GPS/MAG, LED profiles, position hold, and other custom options.
    Uses the standard CONFIG build (not CLOUD_BUILD) so all core features compile.

    Overlay resolution (first match wins):
      1. -OverlayPath if provided
      2. .devcontainer\<Config>.config.h if it exists
      3. No overlay — stock config from src/config submodule only

.PARAMETER Config
    Betaflight board target name (same as CONFIG= in make), e.g. SPEEDYBEEF405V4, MATEKF405TE.

.PARAMETER OverlayPath
    Optional path to a custom config.h overlay. If omitted, looks for
    .devcontainer\<Config>.config.h next to this script.

.EXAMPLE
    .\.devcontainer\build-board.ps1 -Config SPEEDYBEEF405V4

.EXAMPLE
    .\.devcontainer\build-board.ps1 -Config MATEKF405TE -Clean

.EXAMPLE
    .\.devcontainer\build-board.ps1 -Config MATEKF405TE -OverlayPath .\.devcontainer\my-matek.config.h
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$Config,

    [string]$OverlayPath,

    [switch]$SkipImageBuild,
    [switch]$Clean,
    [string]$ImageName = "betaflight-dev"
)

$ErrorActionPreference = "Stop"

function Test-CommandExists {
    param([string]$Name)
    return [bool](Get-Command $Name -ErrorAction SilentlyContinue)
}

if (-not (Test-CommandExists "docker")) {
    Write-Host ""
    Write-Host "Docker n'est pas installe ou pas dans le PATH." -ForegroundColor Red
    Write-Host ""
    Write-Host "Sur Windows, installez dans cet ordre :" -ForegroundColor Yellow
    Write-Host "  1. WSL2   : wsl --install   (redemarrage requis)"
    Write-Host "  2. Docker Desktop : https://docs.docker.com/desktop/setup/install/windows-install/"
    Write-Host "  3. Relancez ce script depuis la racine du depot betaflight."
    Write-Host ""
    exit 1
}

try {
    docker info *> $null
} catch {
    Write-Host "Docker est installe mais le daemon ne tourne pas. Demarrez Docker Desktop puis relancez." -ForegroundColor Red
    exit 1
}

$RepoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
Set-Location $RepoRoot

Write-Host "Depot : $RepoRoot" -ForegroundColor Cyan
Write-Host "Target : $Config" -ForegroundColor Cyan

Write-Host "Initialisation du submodule config..." -ForegroundColor Cyan
git submodule update --init --depth=1 src/config
if ($LASTEXITCODE -ne 0) {
    Write-Host "Echec submodule. Essayez : git submodule update --init -- src/config" -ForegroundColor Red
    exit $LASTEXITCODE
}

$BoardConfigTarget = Join-Path $RepoRoot "src/config/configs/$Config/config.h"
if (-not (Test-Path $BoardConfigTarget)) {
    Write-Host "Target inconnu ou absent du submodule config : $Config" -ForegroundColor Red
    Write-Host "Verifiez le nom exact dans Betaflight Configurator (Onglet Firmware Flasher ou CLI status)." -ForegroundColor Yellow
    Write-Host "Configs disponibles : src/config/configs/<NOM>/config.h" -ForegroundColor Yellow
    exit 1
}

$OverlayToApply = $null
if ($OverlayPath) {
    $OverlayToApply = Resolve-Path $OverlayPath -ErrorAction SilentlyContinue
    if (-not $OverlayToApply) {
        Write-Host "Overlay introuvable : $OverlayPath" -ForegroundColor Red
        exit 1
    }
} else {
    $DefaultOverlay = Join-Path $PSScriptRoot "$Config.config.h"
    if (Test-Path $DefaultOverlay) {
        $OverlayToApply = Resolve-Path $DefaultOverlay
    }
}

if ($OverlayToApply) {
    Copy-Item -Path $OverlayToApply -Destination $BoardConfigTarget -Force
    Write-Host "Overlay applique : $($OverlayToApply.Path) -> configs/$Config/config.h" -ForegroundColor Cyan
} else {
    Write-Host "Pas d'overlay local — config stock du submodule ($Config)." -ForegroundColor DarkGray
    Write-Host "Pour personnaliser : copiez board.config.h.template vers .devcontainer\$Config.config.h" -ForegroundColor DarkGray
}

if (-not $SkipImageBuild) {
    Write-Host "Construction de l'image Docker '$ImageName' (premiere fois : ~5-10 min)..." -ForegroundColor Cyan
    docker build -t $ImageName -f .devcontainer/containerfile .devcontainer/
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
} else {
    Write-Host "Image Docker : skip (-SkipImageBuild)." -ForegroundColor DarkGray
}

$ExtraFlags = @(
    "-DUSE_GPS"
    "-DUSE_MAG"
    "-DUSE_LED_STRIP"
    "-DUSE_LED_STRIP_64"
    "-DUSE_ALTITUDE_HOLD"
    "-DUSE_POSITION_HOLD"
    "-DUSE_CHIRP"
    "-DUSE_BATTERY_CONTINUE"
    "-DUSE_EMFAT_TOOLS"
    "-DUSE_ESCSERIAL_SIMONK"
    "-DUSE_FLIGHT_PLAN"
    "-DUSE_VTX"
) -join " "

$MakeTarget = if ($Clean) {
    "make arm_sdk_install && make clean && make CONFIG=$Config EXTRA_FLAGS=`"$ExtraFlags`""
} else {
    "make arm_sdk_install && make CONFIG=$Config EXTRA_FLAGS=`"$ExtraFlags`""
}

$MakeCmd = "git config --global --add safe.directory /workspace && $MakeTarget"

Write-Host "Compilation : $MakeCmd" -ForegroundColor Cyan

$WorkspacePath = $RepoRoot.Path -replace '\\', '/'
if ($WorkspacePath -match '^([A-Za-z]):') {
    $WorkspacePath = "/$($Matches[1].ToLower())$($WorkspacePath.Substring(2))"
}

docker run --rm `
    -v "${WorkspacePath}:/workspace" `
    -w /workspace `
    $ImageName `
    bash -lc "$MakeCmd"

if ($LASTEXITCODE -ne 0) {
    Write-Host "Build echoue (code $LASTEXITCODE)." -ForegroundColor Red
    exit $LASTEXITCODE
}

Write-Host ""
Write-Host "Build reussi." -ForegroundColor Green
Get-ChildItem -Path (Join-Path $RepoRoot "obj") -Filter "*${Config}*.hex" -ErrorAction SilentlyContinue |
    Sort-Object LastWriteTime -Descending |
    Select-Object -First 3 |
    ForEach-Object { Write-Host "  HEX : $($_.FullName)" -ForegroundColor Green }
Write-Host ""
Write-Host "Flasher via Betaflight Configurator (Load Firmware [Local]) ou dfu-util depuis le conteneur."
