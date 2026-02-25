EAE Cooling Control Project
===========================

Last updated: 2026-02-25


1) Overview
-----------
This project implements a safety-first cooling controller for an ECU-style firmware loop.
It includes:
- State-machine cooling logic (OFF / NORMAL / MAX / FAULT)
- Simulated ADC/GPIO/CAN HAL drivers
- Actuator/sensor component wrappers
- Runtime CLI configuration for setpoints and PID gains
- Unit tests (GoogleTest) and CLI integration checks


2) Repository Layout
--------------------
- app/
  - main simulation loop and CLI handling
  - core control logic (`evaluate_cooling_loop`, `evaluate_system`)
- components/
  - actuators: pump/fan command clamping and PWM writes
  - sensors: temperature and coolant level interpretation
  - ui: ignition decode helper
- hal/
  - simulated ADC, GPIO, PWM, CAN interfaces
- tests/
  - `test_cooling.cpp` (GTest unit tests)
  - `test_cli.sh` (CLI behavior checks)


3) Control Logic Summary
------------------------
Main logic is in `app/cooling_logic.c`.

Inputs:
- ignition_on
- coolant_temperature
- coolant_level_low
- coolant_temp_sensor_valid

Outputs:
- state: STATE_OFF, STATE_NORMAL_COOLING, STATE_MAX_COOLING, STATE_FAULT
- pump/fan on-off intent
- pump/fan PWM percentage

Behavior:
- Ignition OFF -> STATE_OFF, pump/fan OFF
- Invalid temp sensor OR low coolant OR critical temperature -> STATE_FAULT
  - Pump OFF, Fan 100%
- Temperature >= high threshold -> STATE_MAX_COOLING
  - Pump 100%, Fan 100%
- Otherwise -> STATE_NORMAL_COOLING
  - Pump ON (base command), fan OFF

Runtime path (`evaluate_system`) also adds:
- temperature plausibility checks (valid range)
- stuck-sensor monitor
- PID reset when ignition is OFF


4) Tunable Parameters
---------------------
Compile-time defaults in `app/include/cooling_logic.h`:
- TARGET_TEMP (default target setpoint)
- TEMP_HIGH_THRESHOLD
- TEMP_CRITICAL_THRESHOLD
- TEMP_STUCK_DELTA_C
- TEMP_STUCK_CYCLES

Runtime overrides from CLI:
- `--target`, `--high`, `--critical`
- `--kp`, `--ki`, `--kd`
- `--steps`, `--sleep`


5) Build
--------
From repository root:

1. Configure:
   cmake -S . -B build

2. Build:
   cmake --build build -j

Main executable:
- `build/cooling_loop`

Test executable:
- `build/run_tests`


6) Run Firmware Simulation
--------------------------
Default:
- ./build/cooling_loop

With explicit runtime settings:
- ./build/cooling_loop --target 41 --high 46 --critical 66 --kp 4.5 --ki 0.2 --kd 0.8 --steps 8 --sleep 0

Convenience script:
- ./run_firmware.sh

The app prints:
- simulated CAN RX/TX frames
- input snapshot (ignition/temp/level)
- output snapshot (state + PWM)


7) Test
-------
Prerequisite:
- GoogleTest installed and discoverable by CMake (`find_package(GTest REQUIRED)`).

Build (if needed):
- cmake -S . -B build
- cmake --build build -j

Run all tests through CTest:
- ctest --test-dir build --output-on-failure

Run unit tests directly:
- ./build/run_tests

Run CLI checks directly:
- bash ./tests/test_cli.sh

CTest includes:
- discovered GTest cases from `run_tests`
- `CLIIntegrationTest` (runs `tests/test_cli.sh` with `CLI_TEST_SKIP_BUILD=1`)


8) CLI Validation Checklist
---------------------------
1. Default run:
- ./build/cooling_loop --steps 1 --sleep 0

2. Full argument parsing:
- ./build/cooling_loop --target 41 --high 46 --critical 66 --kp 4.5 --ki 0.2 --kd 0.8 --steps 4 --sleep 0

3. Unknown argument rejection (non-zero exit):
- ./build/cooling_loop --badflag

4. Argument sanitization:
- ./build/cooling_loop --steps 0 --sleep -1
  (internally clamps to minimum safe values)

5. CAN + CLI interaction path:
- ./build/cooling_loop --target 41 --steps 4 --sleep 0
  (look for "Applied CAN setpoint update")


9) Notes
--------
- HAL and component layers are intentionally separated for portability.
- Safety behavior favors predictable fail-safe outputs during invalid/fault inputs.
- Control setpoints are sanitized to preserve ordering:
  target < high < critical.