# Run npm in frontend\ with the project-local Node (.tools\node), e.g.:
#   powershell -ExecutionPolicy Bypass -File scripts\npm.ps1 install
#   powershell -ExecutionPolicy Bypass -File scripts\npm.ps1 run build
$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $PSScriptRoot
$Node = Join-Path $Root '.tools\node'
if (-not (Test-Path (Join-Path $Node 'node.exe'))) {
    & (Join-Path $PSScriptRoot 'install_node.ps1')
}
$env:Path = "$Node;$env:Path"
Push-Location (Join-Path $Root 'frontend')
try {
    & (Join-Path $Node 'npm.cmd') @args
    exit $LASTEXITCODE
} finally {
    Pop-Location
}
