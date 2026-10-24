# Downloads the official portable Node.js LTS (Windows x64) into .tools\node.
# Project-local only: nothing is installed globally and PATH is not changed permanently.
$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $PSScriptRoot
$Version = 'v24.22.1'
$Tools = Join-Path $Root '.tools'
$NodeDir = Join-Path $Tools 'node'
if (Test-Path (Join-Path $NodeDir 'node.exe')) { Write-Host "Node already present: $NodeDir"; exit 0 }
New-Item -ItemType Directory -Force $Tools | Out-Null
$Zip = Join-Path $Tools 'node.zip'
$Sums = Join-Path $Tools 'SHASUMS256.txt'
$ProgressPreference = 'SilentlyContinue'
$Name = "node-$Version-win-x64"
if (-not (Test-Path $Zip)) { Invoke-WebRequest "https://nodejs.org/dist/$Version/$Name.zip" -OutFile $Zip }
Invoke-WebRequest "https://nodejs.org/dist/$Version/SHASUMS256.txt" -OutFile $Sums
$expected = ((Select-String -Path $Sums -Pattern "$Name.zip").Line -split '\s+')[0]
$actual = (Get-FileHash $Zip -Algorithm SHA256).Hash.ToLower()
if ($expected -ne $actual) { Remove-Item $Zip; throw "Node checksum mismatch (expected $expected, got $actual)" }
Expand-Archive $Zip -DestinationPath $Tools -Force
Move-Item (Join-Path $Tools $Name) $NodeDir
Remove-Item $Zip
Write-Host "Installed Node $Version to $NodeDir (sha256 verified)"
