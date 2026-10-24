# Development helper: which CPython versions have Windows x64 wheels for every pinned binary dependency?
$Root = Split-Path -Parent $PSScriptRoot
$Py = Join-Path $Root '.venv\Scripts\python.exe'
$tmp = Join-Path $env:TEMP 'armstrong-wheelcheck'
foreach ($v in '3.11', '3.12', '3.13', '3.14') {
    Remove-Item -Recurse -Force $tmp -ErrorAction SilentlyContinue
    & $Py -m pip download --quiet --disable-pip-version-check --only-binary=:all: --platform win_amd64 --python-version $v `
        --no-deps -d $tmp mujoco==3.17.0 numpy==2.5.4 scipy==1.18.1 osqp==1.1.3 pillow==13.0.0 2>&1 | Out-Null
    $ok = $LASTEXITCODE -eq 0
    "python $v : $(if ($ok) { 'all wheels available' } else { 'MISSING wheels' })"
}
Remove-Item -Recurse -Force $tmp -ErrorAction SilentlyContinue
