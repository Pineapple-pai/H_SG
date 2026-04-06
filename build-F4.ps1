param(
    [ValidateSet("Debug", "Release")]
    [string]$Config = "Debug"
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$scriptPath = Join-Path $repoRoot "tools\build-$($Config.ToLower()).ps1"

if (-not (Test-Path $scriptPath)) {
    throw "Build script not found: $scriptPath"
}

& powershell -ExecutionPolicy Bypass -File $scriptPath

if ($LASTEXITCODE -ne 0) {
    throw "Build failed"
}
