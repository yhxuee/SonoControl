param(
    [string]$QtRoot = "E:\Qt\6.11.0\mingw_64",
    [string]$MingwBin = "E:\Qt\Tools\mingw1310_64\bin"
)

$ErrorActionPreference = "Stop"
$env:Path = "$QtRoot\bin;$MingwBin;$env:Path"

cmake -S . -B build-debug-mingw -G Ninja `
  -DCMAKE_BUILD_TYPE=Debug `
  -DCMAKE_C_COMPILER="$MingwBin\gcc.exe" `
  -DCMAKE_CXX_COMPILER="$MingwBin\g++.exe" `
  -DCMAKE_PREFIX_PATH="$QtRoot\lib\cmake"

cmake --build build-debug-mingw
Write-Host "Built. GUI path: .\build-debug-mingw\bin\sonocontrol_gui.exe"
