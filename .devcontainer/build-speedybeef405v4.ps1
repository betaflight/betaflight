#Requires -Version 5.1
<#
.SYNOPSIS
    Build Betaflight firmware for SpeedyBee F405 V4 (wrapper around build-board.ps1).

.DESCRIPTION
    Backward-compatible entry point. Delegates to build-board.ps1 with Config=SPEEDYBEEF405V4
    and overlay .devcontainer/SPEEDYBEEF405V4.config.h if present.

.EXAMPLE
    .\.devcontainer\build-speedybeef405v4.ps1

.EXAMPLE
    .\.devcontainer\build-speedybeef405v4.ps1 -SkipImageBuild -Clean
#>
[CmdletBinding()]
param(
    [switch]$SkipImageBuild,
    [switch]$Clean,
    [string]$ImageName = "betaflight-dev"
)

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$BuildBoardScript = Join-Path $ScriptDir "build-board.ps1"

if (-not (Test-Path $BuildBoardScript)) {
    Write-Host "Script introuvable : $BuildBoardScript" -ForegroundColor Red
    exit 1
}

$params = @{
    Config = "SPEEDYBEEF405V4"
}

if ($SkipImageBuild) { $params.SkipImageBuild = $true }
if ($Clean) { $params.Clean = $true }
if ($ImageName -ne "betaflight-dev") { $params.ImageName = $ImageName }

& $BuildBoardScript @params
exit $LASTEXITCODE
