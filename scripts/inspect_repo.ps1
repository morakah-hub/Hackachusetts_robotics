# Development helper: print time, git state and tracked-file summary (read-only).
$Root = Split-Path -Parent $PSScriptRoot
Set-Location $Root
"now: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss zzz')"
"tz:  $((Get-TimeZone).Id)"
if (Test-Path .git) {
    "--- git rev-parse"; git rev-parse --show-toplevel 2>&1; git branch --show-current 2>&1
    "--- remotes"; git remote -v 2>&1
    "--- log"; git log --oneline -n 10 2>&1
    "--- status"; git status --short 2>&1 | Select-Object -First 80
    "--- tracked top-level"; git ls-files 2>&1 | ForEach-Object { ($_ -split '/')[0] } | Group-Object | Select-Object Name, Count | Format-Table -AutoSize | Out-String
    "--- tracked count"; (git ls-files | Measure-Object).Count
    "--- tracked suspicious"; git ls-files 2>&1 | Where-Object { $_ -match '^(\.venv|\.tools|frontend/node_modules|frontend/dist|runtime|handoff)/' } | Select-Object -First 20
} else { "no .git folder in $Root" }
"--- root listing"; Get-ChildItem -Force | Select-Object Mode, Name, Length | Format-Table -AutoSize | Out-String
"--- md files"; Get-ChildItem -File -Filter *.md | Select-Object Name, LastWriteTime | Format-Table -AutoSize | Out-String
"--- recent source changes"; Get-ChildItem -Recurse -File backend\armstrong, frontend\src, scripts, start.ps1 -ErrorAction SilentlyContinue |
    Sort-Object LastWriteTime -Descending | Select-Object -First 15 FullName, LastWriteTime | Format-Table -AutoSize | Out-String -Width 200
