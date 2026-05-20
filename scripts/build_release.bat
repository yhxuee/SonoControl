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
  cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release || exit /b 1
  cmake --build build-release --config Release || exit /b 1
) else (
  cmake -S . -B build-release -G Ninja -DCMAKE_BUILD_TYPE=Release || exit /b 1
  cmake --build build-release || exit /b 1
)
if exist build-release\bin\sonocontrol.exe (
  echo Built: build-release\bin\sonocontrol.exe
  echo Run: .\build-release\bin\sonocontrol.exe --total-s 3 --com3 COM3 --com11 COM11
) else (
  echo Build finished, but build-release\bin\sonocontrol.exe was not found.
)
