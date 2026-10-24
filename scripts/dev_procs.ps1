# Development helper (read-only): list Armstrong-related processes and listeners on ports 8765-8795.
"now $(Get-Date -Format 'HH:mm:ss')"
Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -and $_.CommandLine -match 'armstrong\.server|-m pytest|ui_e2e|ui_check|run_project|playwright_' } |
    ForEach-Object { '{0,6} {1,-12} {2:HH:mm:ss} {3}' -f $_.ProcessId, $_.Name, $_.CreationDate, $_.CommandLine.Substring(0, [Math]::Min(170, $_.CommandLine.Length)) }
Get-NetTCPConnection -State Listen -ErrorAction SilentlyContinue | Where-Object { $_.LocalPort -ge 8765 -and $_.LocalPort -le 8795 } |
    ForEach-Object { "listening on $($_.LocalAddress):$($_.LocalPort) pid $($_.OwningProcess)" }
"end"
