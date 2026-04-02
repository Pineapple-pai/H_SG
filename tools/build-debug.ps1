$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$cmake = "D:\Clion\CLion 2025.1.3\bin\cmake\win\x64\bin\cmake.exe"
$ninja = "D:\Clion\CLion 2025.1.3\bin\ninja\win\x64\ninja.exe"

if (-not (Test-Path $cmake)) {
    throw "cmake not found: $cmake"
}

if (-not (Test-Path $ninja)) {
    throw "ninja not found: $ninja"
}

& $cmake -S $repoRoot -B "$repoRoot\build\Debug" -G Ninja `
    -D CMAKE_BUILD_TYPE=Debug `
    -D CMAKE_TOOLCHAIN_FILE="$repoRoot\cmake\gcc-arm-none-eabi.cmake" `
    -D CMAKE_MAKE_PROGRAM="$ninja"

if ($LASTEXITCODE -ne 0) {
    throw "CMake configure failed"
}

& $cmake --build "$repoRoot\build\Debug"

if ($LASTEXITCODE -ne 0) {
    throw "CMake build failed"
}
