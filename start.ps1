# Armstrong launcher (Windows PowerShell).  Usage:
#   powershell -ExecutionPolicy Bypass -File .\start.ps1            # set up if needed, build, serve
#   powershell -ExecutionPolicy Bypass -File .\start.ps1 -Port 8770 # other port
#   powershell -ExecutionPolicy Bypass -File .\start.ps1 -Rebuild   # force a frontend rebuild
# Everything is resolved relative to this script; dependencies stay inside the project folder.
param([int]$Port = 8765, [switch]$Rebuild)
$ErrorActionPreference = 'Stop'
$Root = $PSScriptRoot
$Py = Join-Path $Root '.venv\Scripts\python.exe'
$Node = Join-Path $Root '.tools\node\node.exe'
$Dist = Join-Path $Root 'frontend\dist\index.html'

function Fail($msg) { Write-Host "BLOCKER: $msg" -ForegroundColor Red; exit 1 }

# 1. Python environment (.venv)
if (-not (Test-Path $Py)) {
    $sysPy = Get-Command python -ErrorAction SilentlyContinue
    if (-not $sysPy) { Fail 'Python 3.11+ is not on PATH. Install it from https://www.python.org/downloads/ and rerun.' }
    Write-Host 'Creating project virtual environment (.venv)...'
    & $sysPy.Source -m venv (Join-Path $Root '.venv')
    if ($LASTEXITCODE -ne 0) { Fail 'python -m venv failed' }
}
& $Py -c "import mujoco, fastapi, uvicorn, scipy, PIL" 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Host 'Installing Python dependencies (requirements.txt)...'
    & $Py -m pip install --disable-pip-version-check -q -r (Join-Path $Root 'requirements.txt')
    if ($LASTEXITCODE -ne 0) { Fail 'pip install -r requirements.txt failed (network or Python version issue)' }
}

# 2. Frontend build (portable Node in .tools\node, only needed when dist is missing or stale)
$needBuild = $Rebuild -or -not (Test-Path $Dist)
if (-not $needBuild) {
    $distTime = (Get-Item $Dist).LastWriteTime
    $newest = Get-ChildItem (Join-Path $Root 'frontend\src'), (Join-Path $Root 'frontend\index.html') -Recurse -File |
        Sort-Object LastWriteTime -Descending | Select-Object -First 1
    if ($newest.LastWriteTime -gt $distTime) { $needBuild = $true }
}
if ($needBuild) {
    if (-not (Test-Path $Node)) {
        Write-Host 'Downloading portable Node.js LTS into .tools\node ...'
        & (Join-Path $Root 'scripts\install_node.ps1')
        if (-not (Test-Path $Node)) { Fail 'Node.js download failed; the backend can still run if frontend\dist exists.' }
    }
    $env:Path = "$(Split-Path $Node);$env:Path"
    Push-Location (Join-Path $Root 'frontend')
    try {
        if (-not (Test-Path 'node_modules')) {
            Write-Host 'Installing frontend dependencies...'
            & (Join-Path (Split-Path $Node) 'npm.cmd') ci --no-fund --no-audit
            if ($LASTEXITCODE -ne 0) { Fail 'npm ci failed' }
        }
        Write-Host 'Building frontend...'
        & (Join-Path (Split-Path $Node) 'npm.cmd') run build
        if ($LASTEXITCODE -ne 0) { Fail 'frontend build failed' }
    } finally { Pop-Location }
}

# 3. Port check
$busy = Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue
if ($busy) { Fail "port $Port is already in use (another Armstrong instance?). Stop it or use -Port <n>." }

# 4. Serve (simulation + API + built frontend, bound to 127.0.0.1)
Write-Host ''
Write-Host "Armstrong: open http://127.0.0.1:$Port/ in your browser (Ctrl+C here to stop)" -ForegroundColor Cyan
Push-Location (Join-Path $Root 'backend')
try { & $Py -u -m armstrong.server --port $Port } finally { Pop-Location }
