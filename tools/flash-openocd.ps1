param(
    [ValidateSet("Debug", "Release")]
    [string]$Config = "Debug",

    [string]$OpenOcd = $env:OPENOCD,

    [string]$InterfaceCfg = "interface/stlink.cfg",

    [string]$TargetCfg = "target/stm32f4x.cfg"
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$elfPath = Join-Path $repoRoot "build\$Config\SG_CHASSIS_C_2024_11_5.elf"

if (-not (Test-Path $elfPath)) {
    throw "ELF not found: $elfPath`nBuild first with .\tools\build-$($Config.ToLower()).ps1"
}

if (-not $OpenOcd) {
    $candidates = @(
        "openocd",
        "D:\OpenOCD\bin\openocd.exe",
        "C:\OpenOCD\bin\openocd.exe",
        "C:\Program Files\OpenOCD\bin\openocd.exe",
        "D:\xpack-openocd\bin\openocd.exe"
    )

    foreach ($candidate in $candidates) {
        if ($candidate -eq "openocd") {
            $cmd = Get-Command openocd -ErrorAction SilentlyContinue
            if ($cmd) {
                $OpenOcd = $cmd.Source
                break
            }
        } elseif (Test-Path $candidate) {
            $OpenOcd = $candidate
            break
        }
    }
}

if (-not $OpenOcd) {
    throw "openocd.exe not found. Set OPENOCD or pass -OpenOcd <path>."
}

& $OpenOcd `
    -f $InterfaceCfg `
    -f $TargetCfg `
    -c "program $elfPath verify reset exit"

if ($LASTEXITCODE -ne 0) {
    throw "OpenOCD flash failed"
}
