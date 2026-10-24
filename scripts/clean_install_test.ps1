# Clean-install test of a release ZIP: extract into a fresh folder whose path contains spaces, set up from scratch
# (new .venv, no .tools/node_modules), serve on another port, run the Two-part kit over the API, stop our server.
#   powershell -ExecutionPolicy Bypass -File scripts\clean_install_test.ps1 -Zip release\Armstrong-0.2.0.zip
param([Parameter(Mandatory = $true)][string]$Zip, [int]$Port = 8790)
$ErrorActionPreference = 'Continue'
$Zip = (Resolve-Path $Zip).Path
$Base = Join-Path $env:TEMP ("Armstrong judge test " + (Get-Date -Format 'HHmmss'))
New-Item -ItemType Directory -Force $Base | Out-Null
"extract to: $Base"
Expand-Archive -LiteralPath $Zip -DestinationPath $Base
$App = (Get-ChildItem -Directory $Base | Select-Object -First 1).FullName
"app folder: $App"
foreach ($bad in '.venv', '.tools', 'frontend\node_modules', 'handoff', 'user_data', 'runtime', 'release') {
    if (Test-Path (Join-Path $App $bad)) { "FAIL: release contains $bad"; exit 1 }
}
"release tree is clean (no .venv/.tools/node_modules/handoff/user_data/runtime)"
$t0 = Get-Date
& powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $App 'start.ps1') -SetupOnly
"setup exit $LASTEXITCODE after $([int]((Get-Date) - $t0).TotalSeconds) s; node downloaded: $(Test-Path (Join-Path $App '.tools\node\node.exe'))"
if ($LASTEXITCODE -ne 0) { Get-Content (Join-Path $App 'runtime\setup.log') -Tail 30; exit 1 }
$proc = Start-Process powershell -PassThru -WindowStyle Hidden -ArgumentList @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', "`"$(Join-Path $App 'start.ps1')`"", '-Port', $Port)
$url = "http://127.0.0.1:$Port"
$ready = $false
for ($i = 0; $i -lt 90; $i++) {
    try { if ((Invoke-RestMethod "$url/api/health" -TimeoutSec 2).ready) { $ready = $true; break } } catch {}
    Start-Sleep -Seconds 1
}
"server ready: $ready at $url"
if ($ready) {
    $html = (Invoke-WebRequest "$url/" -UseBasicParsing).Content
    "index served: $($html -match 'Armstrong')"
    $null = Invoke-RestMethod -Method Post "$url/api/projects/open" -ContentType 'application/json' -Body '{"source":"example","id":"two_part_kit"}'
    Start-Sleep -Seconds 1
    $s = Invoke-RestMethod "$url/api/state"
    $body = @{ type = 'workflow'; perception_mode = 'camera_rgbd'; scene_revision = $s.scene_revision } | ConvertTo-Json
    $run = Invoke-RestMethod -Method Post "$url/api/run" -ContentType 'application/json' -Body $body
    "run $($run.run_id) started"
    for ($i = 0; $i -lt 120; $i++) {
        $s = Invoke-RestMethod "$url/api/state"
        if ($s.status -in 'completed', 'failed', 'stopped') { break }
        Start-Sleep -Seconds 1
    }
    "result: $($s.status) / $($s.result.outcome) - $($s.result.reason)"
    foreach ($st in $s.result.steps) { "  step $($st.index) $($st.part_name) -> $($st.dest_name): $($st.outcome), placement $([math]::Round(1000 * $st.metrics.placement_error_xy_m, 2)) mm" }
}
# stop only the server this script started (its python child runs from the extracted folder)
Get-CimInstance Win32_Process -Filter "Name='python.exe'" | Where-Object { $_.CommandLine -like '*armstrong.server*' -and $_.ExecutablePath -like "$App*" } |
    ForEach-Object { Stop-Process -Id $_.ProcessId -Force; "stopped test server pid $($_.ProcessId)" }
Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
"done; test folder kept for inspection: $App"
