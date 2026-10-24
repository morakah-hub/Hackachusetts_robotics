# Armstrong launcher (Windows PowerShell 5.1+). Run from anywhere; everything resolves relative to this file.
#   powershell -ExecutionPolicy Bypass -File .\start.ps1               # set up if needed, then serve
#   powershell -ExecutionPolicy Bypass -File .\start.ps1 -Port 8770    # preferred port (next free one is used if busy)
#   powershell -ExecutionPolicy Bypass -File .\start.ps1 -Rebuild      # force a frontend rebuild (needs Node)
#   powershell -ExecutionPolicy Bypass -File .\start.ps1 -SetupOnly    # install/check only, do not serve
#   powershell -ExecutionPolicy Bypass -File .\start.ps1 -AnyPython    # also accept untested Python 3.12 / 3.14
# Or double-click start.bat. First run needs internet for the Python packages (and Node only if the frontend must
# be rebuilt); after that everything runs locally. Log of every setup command: runtime\setup.log.
param([int]$Port = 8765, [switch]$Rebuild, [switch]$SetupOnly, [switch]$AnyPython)
# Native tools (pip, npm) write warnings to stderr; Windows PowerShell 5.1 would turn those into terminating errors
# under 'Stop'. Every step below checks exit codes explicitly instead.
$ErrorActionPreference = 'Continue'
$Root = $PSScriptRoot
$Venv = Join-Path $Root '.venv'
$Py = Join-Path $Venv 'Scripts\python.exe'
$Req = Join-Path $Root 'requirements.txt'
$NodeDir = Join-Path $Root '.tools\node'
$Log = Join-Path $Root 'runtime\setup.log'
New-Item -ItemType Directory -Force (Join-Path $Root 'runtime') | Out-Null
"==== start.ps1 $(Get-Date -Format s) in $Root" | Out-File -FilePath $Log -Append -Encoding utf8
$SupportedPython = @('3.13')            # tested: 3.13.7 (see README)
if ($AnyPython) {                        # pinned wheels also exist for 3.12 and 3.14, but those are untested
    $SupportedPython = @('3.13', '3.12', '3.14')
    Write-Host 'Note: -AnyPython allows Python 3.12/3.14, which are not tested with Armstrong.' -ForegroundColor Yellow
}

function Fail($msg) {
    Write-Host ''
    Write-Host "BLOCKER: $msg" -ForegroundColor Red
    Write-Host "Setup log: $Log"
    exit 1
}
function Step($msg) { Write-Host "-> $msg" -ForegroundColor Cyan }
function Invoke-Logged([string]$Exe, [string[]]$ArgList) {
    "> $Exe $($ArgList -join ' ')" | Out-File -FilePath $Log -Append -Encoding utf8
    & $Exe @ArgList 2>&1 | ForEach-Object { "$_" } | Out-File -FilePath $Log -Append -Encoding utf8
    return $LASTEXITCODE
}
function Get-PyVersion([string]$Exe) {
    $v = & $Exe -c "import sys; print('%d.%d' % sys.version_info[:2])" 2>&1
    if ($LASTEXITCODE -ne 0) { return '' }
    return "$v".Trim()
}

# 1. Python ------------------------------------------------------------------------------------------------
function Find-Python {
    $cands = @()
    $launcher = Get-Command py -ErrorAction SilentlyContinue
    foreach ($v in $SupportedPython) {
        if ($launcher) {
            $exe = & $launcher.Source "-$v" -c "import sys; print(sys.executable)" 2>&1
            if ($LASTEXITCODE -eq 0 -and $exe) { $cands += "$exe".Trim() }
        }
    }
    foreach ($name in 'python', 'python3') {
        $c = Get-Command $name -ErrorAction SilentlyContinue
        if ($c -and $c.Source -notlike '*WindowsApps*') { $cands += $c.Source }
    }
    foreach ($exe in $cands) {
        if ((Test-Path $exe) -and ($SupportedPython -contains (Get-PyVersion $exe))) { return $exe }
    }
    return $null
}

if (Test-Path $Py) {
    $ver = Get-PyVersion $Py
    if (-not ($SupportedPython -contains $ver)) {
        Fail "the existing .venv uses Python '$ver', but Armstrong is tested with Python $($SupportedPython -join ', '). Delete the .venv folder and run start.ps1 again."
    }
} else {
    $sys = Find-Python
    if (-not $sys) {
        Fail "Python $($SupportedPython -join '/') was not found. Install Python 3.13 (64-bit) from https://www.python.org/downloads/ (tick 'Add python.exe to PATH'), then run start.ps1 again."
    }
    Step "Creating the project virtual environment (.venv) with $sys"
    if ((Invoke-Logged $sys @('-m', 'venv', $Venv)) -ne 0 -or -not (Test-Path $Py)) { Fail 'python -m venv failed' }
}

# 2. Python packages: reinstall whenever requirements.txt changes -----------------------------------------------
$stampFile = Join-Path $Venv 'armstrong-requirements.sha256'
$reqHash = (Get-FileHash $Req -Algorithm SHA256).Hash
$have = if (Test-Path $stampFile) { (Get-Content $stampFile -Raw).Trim() } else { '' }
if ($have -ne $reqHash) {
    Step 'Installing Python packages from requirements.txt (first run downloads about 120 MB)...'
    if ((Invoke-Logged $Py @('-m', 'pip', 'install', '--disable-pip-version-check', '--no-input', '-r', $Req)) -ne 0) {
        Fail 'pip install -r requirements.txt failed (internet connection? see the log)'
    }
    if ((Invoke-Logged $Py @('-c', 'import mujoco, numpy, scipy, fastapi, uvicorn, PIL')) -ne 0) {
        Fail 'Python packages installed but cannot be imported (see the log)'
    }
    Set-Content -Path $stampFile -Value $reqHash -NoNewline
}

# 3. Frontend: the release ships a built frontend\dist; rebuild only if the sources changed ----------------------
$stampScript = Join-Path $Root 'scripts\frontend_stamp.py'
& $Py $stampScript check
$needBuild = $Rebuild -or ($LASTEXITCODE -ne 0)
if ($needBuild) {
    if (-not (Test-Path (Join-Path $NodeDir 'node.exe'))) {
        Step 'Frontend needs a build: downloading portable Node.js LTS into .tools\node (SHA-256 verified)...'
        Invoke-Logged 'powershell' @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', (Join-Path $Root 'scripts\install_node.ps1')) | Out-Null
        if (-not (Test-Path (Join-Path $NodeDir 'node.exe'))) { Fail 'Node.js download failed (needed only to rebuild the frontend; see the log)' }
    }
    $env:Path = "$NodeDir;$env:Path"
    $npm = Join-Path $NodeDir 'npm.cmd'
    $front = Join-Path $Root 'frontend'
    Push-Location $front
    try {
        $lockStamp = Join-Path $front 'node_modules\.armstrong-lock.sha256'
        $lockHash = (Get-FileHash (Join-Path $front 'package-lock.json') -Algorithm SHA256).Hash
        if (-not (Test-Path $lockStamp) -or ((Get-Content $lockStamp -Raw).Trim() -ne $lockHash)) {
            Step 'Installing frontend packages (npm ci)...'
            if ((Invoke-Logged $npm @('ci', '--no-fund', '--no-audit')) -ne 0) { Fail 'npm ci failed (see the log)' }
            Set-Content -Path $lockStamp -Value $lockHash -NoNewline
        }
        Step 'Building the frontend...'
        if ((Invoke-Logged $npm @('run', 'build')) -ne 0) { Fail 'frontend build failed (see the log)' }
    } finally { Pop-Location }
    & $Py $stampScript write
}
if ($SetupOnly) { Write-Host 'Setup complete.' -ForegroundColor Green; exit 0 }

# 4. Port: never stop another program; use the next free port instead -----------------------------------------
function Test-PortFree([int]$p) {
    try {
        $l = [System.Net.Sockets.TcpListener]::new([System.Net.IPAddress]::Loopback, $p)
        $l.Start(); $l.Stop(); return $true
    } catch { return $false }
}
$chosen = $null
foreach ($p in $Port..($Port + 10)) { if (Test-PortFree $p) { $chosen = $p; break } }
if (-not $chosen) { Fail "ports $Port-$($Port + 10) are all in use; pass -Port <number>" }
if ($chosen -ne $Port) { Write-Host "Port $Port is in use by another program; using $chosen instead." -ForegroundColor Yellow }

# 5. Serve (simulation + API + built frontend, bound to 127.0.0.1 only) ----------------------------------------
Write-Host ''
Write-Host "Armstrong: open http://127.0.0.1:$chosen/ in your browser  (Ctrl+C in this window stops it)" -ForegroundColor Green
Push-Location (Join-Path $Root 'backend')
try { & $Py -u -m armstrong.server --port $chosen } finally { Pop-Location }
