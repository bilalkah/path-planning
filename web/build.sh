#!/usr/bin/env bash
# Build the planners to WebAssembly and assemble the project page in site/.
#
# Needs the Emscripten SDK on PATH (source emsdk_env.sh). Afterwards, serve
# the page with:
#   python3 -m http.server --directory site
set -euo pipefail

cd "$(dirname "$0")/.."

emcmake cmake -S . -B build/web -DCMAKE_BUILD_TYPE=Release -Wno-dev
cmake --build build/web --target path_planning_web --parallel

# Generated files; .gitignore keeps them out of the repository.
mkdir -p site/wasm site/maps
cp build/web/web/path_planning.js build/web/web/path_planning.wasm site/wasm/
cp maps/*/*.map site/maps/

echo "Built site/wasm and site/maps."
