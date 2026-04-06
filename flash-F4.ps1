param(
    [ValidateSet("Debug", "Release")]
    [string]$Config = "Debug",

    [string]$OpenOcd = "C:\Users\25444\.eide\tools\openocd_7a1adfbec_mingw32\bin\openocd.exe"
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$scriptPath = Join-Path $repoRoot "tools\flash-openocd.ps1"

if (-not (Test-Path $scriptPath)) {
    throw "Flash script not found: $scriptPath"
}

$argsList = @(
    "-ExecutionPolicy", "Bypass",
    "-File", $scriptPath,
    "-Config", $Config
)

if ($OpenOcd) {
    $argsList += @("-OpenOcd", $OpenOcd)
}

& powershell @argsList

if ($LASTEXITCODE -ne 0) {
    throw "Flash failed"
}
