# Development helper: open an example over the API, screenshot, start a run, screenshot mid-run, report the result.
param([string]$Base = 'http://127.0.0.1:8765', [string]$Project = 'two_part_kit', [int]$MidRunSeconds = 26)
$Root = Split-Path -Parent $PSScriptRoot
$shot = Join-Path $PSScriptRoot 'screenshot.ps1'
function PostJson($path, $obj) { Invoke-RestMethod -Method Post -Uri "$Base$path" -ContentType 'application/json' -Body ($obj | ConvertTo-Json -Depth 8) }
$r = PostJson '/api/projects/open' @{ source = 'example'; id = $Project }
"opened $Project -> revision $($r.scene_revision)"
Start-Sleep -Seconds 2
& powershell -NoProfile -ExecutionPolicy Bypass -File $shot -Url "$Base/" -Out (Join-Path $Root 'runtime\ui_kit_idle.png') -WaitMs 8000 | Out-Null
$s = Invoke-RestMethod "$Base/api/state"
$run = PostJson '/api/run' @{ type = 'workflow'; perception_mode = 'camera_rgbd'; scene_revision = $s.scene_revision }
"run $($run.run_id)"
Start-Sleep -Seconds $MidRunSeconds
& powershell -NoProfile -ExecutionPolicy Bypass -File $shot -Url "$Base/" -Out (Join-Path $Root 'runtime\ui_kit_run.png') -WaitMs 5000 | Out-Null
for ($i = 0; $i -lt 90; $i++) {
    $s = Invoke-RestMethod "$Base/api/state"
    if ($s.status -in 'completed', 'failed', 'stopped') { break }
    Start-Sleep -Seconds 1
}
"final: $($s.status) $($s.result.outcome) - $($s.result.reason)"
foreach ($st in $s.result.steps) { "  step $($st.index) $($st.part_name) -> $($st.dest_name): $($st.outcome); placed err $([math]::Round(1000 * $st.metrics.placement_error_xy_m, 2)) mm; seen err $([math]::Round(1000 * $st.metrics.estimate_error_xy_m, 2)) mm" }
"sim $([math]::Round($s.result.sim_duration_s, 1)) s, wall $([math]::Round($s.result.wall_duration_s, 1)) s"
& powershell -NoProfile -ExecutionPolicy Bypass -File $shot -Url "$Base/" -Out (Join-Path $Root 'runtime\ui_kit_done.png') -WaitMs 6000 | Out-Null
"screenshots in runtime\ui_kit_*.png"
