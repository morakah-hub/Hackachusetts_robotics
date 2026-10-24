# Capture a screenshot of the running app with headless Microsoft Edge (used for visual checks).
#   powershell -ExecutionPolicy Bypass -File scripts\screenshot.ps1 [-Url http://127.0.0.1:8765/] [-Out runtime\ui.png]
param(
    [string]$Url = 'http://127.0.0.1:8765/',
    [string]$Out = '',
    [int]$Width = 1600,
    [int]$Height = 940,
    [int]$WaitMs = 9000
)
$Root = Split-Path -Parent $PSScriptRoot
if (-not $Out) { $Out = Join-Path $Root 'runtime\ui.png' }
$edge = @("${env:ProgramFiles(x86)}\Microsoft\Edge\Application\msedge.exe", "$env:ProgramFiles\Microsoft\Edge\Application\msedge.exe") |
    Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $edge) { Write-Error 'Microsoft Edge not found'; exit 1 }
$profileDir = Join-Path $Root 'runtime\edge-profile'
& $edge --headless=new --disable-gpu-sandbox --use-angle=swiftshader --enable-unsafe-swiftshader --hide-scrollbars `
    --user-data-dir="$profileDir" --window-size="$Width,$Height" --virtual-time-budget=$WaitMs `
    --screenshot="$Out" $Url 2>&1 | Out-Null
Write-Host "screenshot: $Out"
