# Run the backend test suite with the project venv:  powershell -ExecutionPolicy Bypass -File scripts\test.ps1
$Root = Split-Path -Parent $PSScriptRoot
Push-Location (Join-Path $Root 'backend')
try {
    & (Join-Path $Root '.venv\Scripts\python.exe') -m pytest @args
    exit $LASTEXITCODE
} finally { Pop-Location }
