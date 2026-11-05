# Simwright installer for Windows (PowerShell 5.1 or 7). No admin needed.
#   & ([scriptblock]::Create((irm '<url>/install.ps1'))) -Package '<url>/simwright-0.1.0-py3-none-any.whl'
param(
  [string]$Package = "",
  [string]$Version = "0.1.0",
  [string]$Workspace = ""
)
$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"
Write-Host "Simwright $Version installer" -ForegroundColor Cyan

# 1. uv (manages Python and the simwright tool environment)
if (-not (Get-Command uv -ErrorAction SilentlyContinue)) {
  Write-Host "Installing uv..."
  Invoke-RestMethod https://astral.sh/uv/install.ps1 | Invoke-Expression
  $env:Path = "$env:USERPROFILE\.local\bin;$env:Path"
}

# 2. the simwright package
$dist = Join-Path $env:USERPROFILE ".simwright\dist"
New-Item -ItemType Directory -Force -Path $dist | Out-Null
if ($Package -match '^https?://') {
  $whl = Join-Path $dist "simwright-$Version-py3-none-any.whl"
  Write-Host "Downloading package..."
  Invoke-WebRequest -Uri $Package -OutFile $whl -UseBasicParsing
} elseif ($Package) { $whl = (Resolve-Path $Package).Path } else { $whl = "simwright" }
uv tool install --force --python 3.12 "$whl"
uv tool update-shell | Out-Null
$sw = Join-Path $env:USERPROFILE ".local\bin\simwright.exe"

# 3. environment report
Write-Host ""
& $sw doctor
Write-Host ""
Write-Host "Next steps" -ForegroundColor Cyan
Write-Host "  cd <your robot workspace>; simwright init        # writes SIMWRIGHT.md"
Write-Host "  simwright                                         # interactive copilot (set SIMWRIGHT_API_KEY for standalone use)"
Write-Host "  simwright isaac launch                            # Isaac Sim + python_server + Simwright panel"
Write-Host "  claude mcp add simwright -- simwright-mcp --root <workspace>   # use from Claude Code / Cursor"
Write-Host "  In Mana: pair this PC with Mana Bridge, then type 'mana' in your workspace to drive Simwright from chat."
if ($Workspace) { Push-Location $Workspace; & $sw init; Pop-Location }
