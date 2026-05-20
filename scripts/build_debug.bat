@echo off
setlocal
cd /d "%~dp0.."
where cmake >nul 2>nul
if errorlevel 1 (
  echo CMake was not found in PATH.
  echo Install CMake, then restart Command Prompt or PowerShell.
  echo Example: winget install Kitware.CMake
  exit /b 1
)
where ninja >nul 2>nul
if errorlevel 1 (
  cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug || exit /b 1
  cmake --build build-debug --config Debug || exit /b 1
) else (
  cmake -S . -B build-debug -G Ninja -DCMAKE_BUILD_TYPE=Debug || exit /b 1
  cmake --build build-debug || exit /b 1
)
if exist build-debug\bin\sonocontrol.exe (
  echo Built: build-debug\bin\sonocontrol.exe
  echo Run: .\build-debug\bin\sonocontrol.exe --total-s 3 --sim-temp --sim-us
) else (
  echo Build finished, but build-debug\bin\sonocontrol.exe was not found.
)
