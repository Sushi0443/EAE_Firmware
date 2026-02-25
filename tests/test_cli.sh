#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

echo "[CLI TEST] Configure + build"
if [[ "${CLI_TEST_SKIP_BUILD:-0}" != "1" ]]; then
  cmake -S . -B build >/dev/null
  cmake --build build -j >/dev/null
else
  echo "[CLI TEST] Skipping build (CLI_TEST_SKIP_BUILD=1)"
fi

echo "[CLI TEST] 1) Default run"
DEFAULT_OUT="$(./build/cooling_loop --steps 1 --sleep 0)"
echo "$DEFAULT_OUT" | grep -q "Section 7.1 Firmware"

echo "[CLI TEST] 2) Full argument run"
FULL_OUT="$(./build/cooling_loop --target 41 --high 46 --critical 66 --kp 4.5 --ki 0.2 --kd 0.8 --steps 4 --sleep 0)"
echo "$FULL_OUT" | grep -q "Setpoints -> target=41.0C high=46.0C critical=66.0C"
echo "$FULL_OUT" | grep -q "PID gains -> kp=4.50 ki=0.20 kd=0.80"

echo "[CLI TEST] 3) Unknown argument should fail"
set +e
BAD_OUT="$(./build/cooling_loop --badflag 2>&1)"
BAD_RC=$?
set -e
if [[ $BAD_RC -eq 0 ]]; then
  echo "Expected non-zero exit for unknown argument"
  exit 1
fi
echo "$BAD_OUT" | grep -q "Unknown argument"
echo "$BAD_OUT" | grep -q "Usage: ./cooling_loop"

echo "[CLI TEST] 4) Sanitization checks"
SANITIZE_OUT="$(./build/cooling_loop --steps 0 --sleep -1)"
echo "$SANITIZE_OUT" | grep -q -- "--- Time Step 0 ---"

echo "[CLI TEST] 5) CAN + CLI interaction"
CAN_OUT="$(./build/cooling_loop --target 41 --steps 4 --sleep 0)"
echo "$CAN_OUT" | grep -q "Applied CAN setpoint update"

echo "All CLI checks passed."
