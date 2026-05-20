$ErrorActionPreference = "Stop"
$Root = Resolve-Path (Join-Path $PSScriptRoot "..")
Set-Location $Root

if (-not (Get-Command cmake -ErrorAction SilentlyContinue)) {
  Write-Host "CMake was not found in PATH. Install CMake first, then restart PowerShell. Example:" -ForegroundColor Red
  Write-Host "  winget install Kitware.CMake"
  Write-Host "Optional Ninja generator:"
  Write-Host "  winget install Ninja-build.Ninja"
  exit 1
}

$Ninja = Get-Command ninja -ErrorAction SilentlyContinue
if ($Ninja) {
  cmake -S . -B build-debug -G Ninja -DCMAKE_BUILD_TYPE=Debug
  cmake --build build-debug
} else {
  cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug
  cmake --build build-debug --config Debug
}

$Exe = Join-Path $Root "build-debug\bin\sonocontrol.exe"
if (Test-Path $Exe) {
  Write-Host "Built: $Exe" -ForegroundColor Green
  Write-Host "Simulation run example:"
  Write-Host "  .\build-debug\bin\sonocontrol.exe --total-s 3 --sim-temp --sim-us"
} else {
  Write-Host "Build finished, but executable was not found at $Exe. Check the build output above." -ForegroundColor Yellow
}
