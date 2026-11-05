# Simwright installer for Windows (PowerShell 5.1 or 7). No admin rights needed.
#   irm https://raw.githubusercontent.com/pateldhruv1672/simwright/main/install/install.ps1 | iex
#   or, with a specific wheel:  & ([scriptblock]::Create((irm '<url>/install.ps1'))) -Package '<url>/simwright-X.Y.Z-py3-none-any.whl'
param(
  [string]$Package = "",
  [string]$Ref = "main",
  [string]$Workspace = ""
)
$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"
Write-Host "Simwright installer" -ForegroundColor Cyan

# 1. uv (manages Python and the simwright tool environment)
if (-not (Get-Command uv -ErrorAction SilentlyContinue)) {
  Write-Host "Installing uv..."
  Invoke-RestMethod https://astral.sh/uv/install.ps1 | Invoke-Expression
  $env:Path = "$env:USERPROFILE\.local\bin;$env:Path"
}

# 2. stop a running `simwright serve` (Windows cannot replace simwright.exe while it runs)
$p = Get-NetTCPConnection -LocalPort 8787 -State Listen -ErrorAction SilentlyContinue | Select-Object -ExpandProperty OwningProcess -Unique
if ($p) { Write-Host "Stopping simwright serve (pid $p) for the upgrade"; Stop-Process -Id $p -Force; Start-Sleep 1 }

# 3. the simwright package: a given wheel, else straight from GitHub
if ($Package -match '^https?://') {
  $dist = Join-Path $env:USERPROFILE ".simwright\dist"
  New-Item -ItemType Directory -Force -Path $dist | Out-Null
  $whl = Join-Path $dist ([IO.Path]::GetFileName(([Uri]$Package).AbsolutePath))
  if ($whl -notmatch '\.whl$') { $whl = Join-Path $dist "simwright-0-py3-none-any.whl" }
  Write-Host "Downloading package..."
  Invoke-WebRequest -Uri $Package -OutFile $whl -UseBasicParsing
  $src = $whl
} elseif ($Package) { $src = (Resolve-Path $Package).Path }
else { $src = "simwright @ git+https://github.com/pateldhruv1672/simwright@$Ref" }
uv tool install --force --python 3.12 "$src"
uv tool update-shell | Out-Null
$sw = Join-Path $env:USERPROFILE ".local\bin\simwright.exe"

# 4. environment report
Write-Host ""
& $sw doctor
Write-Host ""
Write-Host "Next steps" -ForegroundColor Cyan
Write-Host "  cd <your robot workspace>; simwright init       # scans the workspace, writes SIMWRIGHT.md"
Write-Host "  simwright connect claude                         # or: codex | cursor | vscode | claude-desktop | chatgpt"
Write-Host "  simwright isaac launch                           # Isaac Sim + python_server + Simwright panel"
Write-Host "  simwright                                        # built-in terminal copilot (needs SIMWRIGHT_API_KEY)"
if ($Workspace) { Push-Location $Workspace; & $sw init; Pop-Location }
