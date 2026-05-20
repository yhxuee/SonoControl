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
  cmake -S . -B build-release -G Ninja -DCMAKE_BUILD_TYPE=Release
  cmake --build build-release
} else {
  cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
  cmake --build build-release --config Release
}

$Exe = Join-Path $Root "build-release\bin\sonocontrol.exe"
if (Test-Path $Exe) {
  Write-Host "Built: $Exe" -ForegroundColor Green
  Write-Host "Run example:"
  Write-Host "  .\build-release\bin\sonocontrol.exe --total-s 3 --com3 COM3 --com11 COM11"
} else {
  Write-Host "Build finished, but executable was not found at $Exe. Check the build output above." -ForegroundColor Yellow
}
