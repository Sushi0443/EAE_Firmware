cmake_minimum_required(VERSION 3.14)

# We need both C (for the PLC logic) and CXX (C++ for Gtest)
project(EAE_Coding VERSION 1.0 LANGUAGES C CXX)

# --- 1. Define the Main Program ---
# This builds your standard executable using your C files
add_executable(cooling_loop
	app/main.c
	app/cooling_logic.c
	components/actuators/spal_fan.c
	components/actuators/wp_pump.c
	components/sensors/hwtms_temp.c
	components/sensors/lmc100_level.c
	components/ui/f_series_sw.c
	hal/adc.c
	hal/can.c
	hal/gpio.c
	hal/pwm.c
)
target_include_directories(cooling_loop PRIVATE
	app/include
	components/actuators/include
	components/sensors/include
	components/ui/include
	hal/include
)

# --- 2. Define the Unit Tests ---
# Find the system-installed GTest (fulfills the "do not ship dependencies" rule)
find_package(GTest REQUIRED)

# Create a separate executable just for running tests
add_executable(run_tests
	tests/test_cooling.cpp
	app/cooling_logic.c
	components/actuators/spal_fan.c
	components/actuators/wp_pump.c
	hal/pwm.c
)
target_include_directories(run_tests PRIVATE
	app/include
	components/actuators/include
	hal/include
)

# Link the Google Test libraries to your test executable
target_link_libraries(run_tests GTest::gtest GTest::gtest_main pthread)

# Enable VS Code's test explorer to discover these tests
include(CTest)
gtest_discover_tests(run_tests)

# CLI integration tests
add_test(
	NAME CLIIntegrationTest
	COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_cli.sh
)
set_tests_properties(CLIIntegrationTest PROPERTIES
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	ENVIRONMENT "CLI_TEST_SKIP_BUILD=1"
)