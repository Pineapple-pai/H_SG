param(
    [ValidateSet("Debug", "Release")]
    [string]$Config = "Debug",

    [string]$OpenOcd = "C:\Users\25444\.eide\tools\openocd_7a1adfbec_mingw32\bin\openocd.exe"
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$buildScript = Join-Path $repoRoot "build-F4.ps1"
$flashScript = Join-Path $repoRoot "flash-F4.ps1"

& powershell -ExecutionPolicy Bypass -File $buildScript -Config $Config

if ($LASTEXITCODE -ne 0) {
    throw "Build failed"
}

$argsList = @(
    "-ExecutionPolicy", "Bypass",
    "-File", $flashScript,
    "-Config", $Config
)

if ($OpenOcd) {
    $argsList += @("-OpenOcd", $OpenOcd)
}

& powershell @argsList

if ($LASTEXITCODE -ne 0) {
    throw "Flash failed"
}
