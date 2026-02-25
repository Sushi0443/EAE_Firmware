#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

cmake -S . -B build
cmake --build build -j

./build/cooling_loop \
  --target 40 \
  --high 45 \
  --critical 65 \
  --kp 4.0 \
  --ki 0.2 \
  --kd 0.8 \
  --steps 8 \
  --sleep 1
