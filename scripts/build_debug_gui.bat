@echo off
set QT_CMAKE_PATH=%~1
if "%QT_CMAKE_PATH%"=="" (
  cmake -S . -B build-debug -G Ninja -DCMAKE_BUILD_TYPE=Debug
) else (
  cmake -S . -B build-debug -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_PREFIX_PATH="%QT_CMAKE_PATH%"
)
if errorlevel 1 exit /b 1
cmake --build build-debug
