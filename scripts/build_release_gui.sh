#!/usr/bin/env bash
set -euo pipefail
QT_CMAKE_PATH="${1:-}"
args=(-S . -B build-release -G Ninja -DCMAKE_BUILD_TYPE=Release)
if [[ -n "$QT_CMAKE_PATH" ]]; then
  args+=("-DCMAKE_PREFIX_PATH=$QT_CMAKE_PATH")
fi
cmake "${args[@]}"
cmake --build build-release
