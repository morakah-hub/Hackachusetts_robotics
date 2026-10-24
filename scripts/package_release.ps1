# Build a clean judge ZIP: committed/release-intended source + examples + docs + a freshly built frontend\dist.
#   powershell -ExecutionPolicy Bypass -File scripts\package_release.ps1 [-Version 0.2.0] [-SkipBuild]
# Output: release\Armstrong-<version>.zip (release\ is git-ignored). Never includes .venv, .tools, node_modules,
# runtime output, user_data (private projects/uploads) or the historical handoff reference.
param([string]$Version = '0.2.0', [switch]$SkipBuild)
$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $PSScriptRoot
Set-Location $Root
$Py = Join-Path $Root '.venv\Scripts\python.exe'
$Name = "Armstrong-$Version"
$Out = Join-Path $Root 'release'
$Stage = Join-Path $Out $Name
$Zip = Join-Path $Out "$Name.zip"

if (-not $SkipBuild) {
    Write-Host '-> building frontend\dist'
    & powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $Root 'scripts\npm.ps1') run build
    if ($LASTEXITCODE -ne 0) { throw 'frontend build failed' }
    & $Py (Join-Path $Root 'scripts\frontend_stamp.py') write
}
& $Py (Join-Path $Root 'scripts\frontend_stamp.py') check
if ($LASTEXITCODE -ne 0) { throw 'frontend\dist does not match the sources; run without -SkipBuild' }

# Files intended for source control: tracked + untracked-but-not-ignored. (Commit before publishing.)
$files = git -c core.quotepath=off ls-files --cached --others --exclude-standard | Where-Object { $_ -and (Test-Path $_ -PathType Leaf) }
$blocked = '^(\.venv|\.tools|frontend/node_modules|runtime|user_data|release|handoff)/'
$bad = $files | Where-Object { $_ -match $blocked }
if ($bad) { throw "refusing to package machine-specific files: $($bad | Select-Object -First 5)" }
$dist = Get-ChildItem -Recurse -File (Join-Path $Root 'frontend\dist') | ForEach-Object { $_.FullName.Substring($Root.Length + 1).Replace('\', '/') }

Remove-Item -Recurse -Force $Stage -ErrorAction SilentlyContinue
Remove-Item -Force $Zip -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Force $Stage | Out-Null
foreach ($f in @($files) + @($dist)) {
    $dst = Join-Path $Stage $f
    New-Item -ItemType Directory -Force (Split-Path $dst) | Out-Null
    Copy-Item -LiteralPath (Join-Path $Root $f) -Destination $dst
}
foreach ($must in 'start.ps1', 'start.bat', 'requirements.txt', 'README.md', 'docs/JUDGE_QUICKSTART.md',
                  'backend/armstrong/server.py', 'backend/assets/robots/franka_emika_panda/LICENSE',
                  'frontend/dist/index.html', 'frontend/package-lock.json', 'examples/projects/two_part_kit.json', '.kiro/steering/armstrong.md') {
    if (-not (Test-Path (Join-Path $Stage $must))) { throw "release is missing $must" }
}
Compress-Archive -Path $Stage -DestinationPath $Zip -CompressionLevel Optimal
$n = (Get-ChildItem -Recurse -File $Stage).Count
$mb = [math]::Round((Get-Item $Zip).Length / 1MB, 1)
Write-Host "release: $Zip ($n files, $mb MB)"
