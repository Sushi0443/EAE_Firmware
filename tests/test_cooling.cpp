#include <gtest/gtest.h>

// Tell the C++ compiler to treat this header as pure C
extern "C" {
    #include "../app/include/cooling_logic.h"
}

// --- Test 1: System Should Be Off When Ignition Is Off ---
TEST(CoolingLogicTest, SystemOffWhenIgnitionOff) {
    reset_cooling_safety_monitors();

    SystemInputs inputs = {
        .ignition_on = false,
        .coolant_temperature = 40.0f,
        .coolant_level_low = false,
        .coolant_temp_sensor_valid = true
    };
    SystemOutputs outputs = { .pump_on = true, .fan_on = true, .current_state = STATE_FAULT }; // Start with garbage data

    evaluate_cooling_loop(&inputs, &outputs);

    EXPECT_EQ(outputs.current_state, STATE_OFF);
    EXPECT_FALSE(outputs.pump_on);
    EXPECT_FALSE(outputs.fan_on);
}

// --- Test 2: Normal Cooling (Temp below high threshold) ---
TEST(CoolingLogicTest, NormalCooling) {
    reset_cooling_safety_monitors();

    SystemInputs inputs = {
        .ignition_on = true,
        .coolant_temperature = 35.0f,
        .coolant_level_low = false,
        .coolant_temp_sensor_valid = true
    };
    SystemOutputs outputs;

    evaluate_cooling_loop(&inputs, &outputs);

    EXPECT_EQ(outputs.current_state, STATE_NORMAL_COOLING);
    EXPECT_TRUE(outputs.pump_on);
    EXPECT_FALSE(outputs.fan_on);
}

// --- Test 3: Max Cooling (Temp above high threshold) ---
TEST(CoolingLogicTest, MaxCoolingWhenHot) {
    reset_cooling_safety_monitors();

    SystemInputs inputs = {
        .ignition_on = true,
        .coolant_temperature = 50.0f,
        .coolant_level_low = false,
        .coolant_temp_sensor_valid = true
    };
    SystemOutputs outputs;

    evaluate_cooling_loop(&inputs, &outputs);

    EXPECT_EQ(outputs.current_state, STATE_MAX_COOLING);
    EXPECT_TRUE(outputs.pump_on);
    EXPECT_TRUE(outputs.fan_on);
}

// --- Test 4: Safety Fault (Over-Temperature) ---
TEST(CoolingLogicTest, FaultOnCriticalTemp) {
    reset_cooling_safety_monitors();

    SystemInputs inputs = {
        .ignition_on = true,
        .coolant_temperature = 70.0f,
        .coolant_level_low = false,
        .coolant_temp_sensor_valid = true
    };
    SystemOutputs outputs;

    evaluate_cooling_loop(&inputs, &outputs);

    EXPECT_EQ(outputs.current_state, STATE_FAULT);
    EXPECT_FALSE(outputs.pump_on); // Pump off to protect system (based on our logic)
    EXPECT_TRUE(outputs.fan_on);   // Fan stays on
}

// --- Test 5: Safety Fault (Low Coolant Level) ---
TEST(CoolingLogicTest, FaultOnLowCoolantLevel) {
    reset_cooling_safety_monitors();

    SystemInputs inputs = {
        .ignition_on = true,
        .coolant_temperature = 30.0f,
        .coolant_level_low = true,
        .coolant_temp_sensor_valid = true
    };
    SystemOutputs outputs;

    evaluate_cooling_loop(&inputs, &outputs);

    EXPECT_EQ(outputs.current_state, STATE_FAULT);
    EXPECT_FALSE(outputs.pump_on); // Prevent dry running
    EXPECT_TRUE(outputs.fan_on);
}

TEST(CoolingLogicTest, FaultOnDisconnectedTemperatureSensor) {
    reset_cooling_safety_monitors();

    SystemInputs inputs = {
        .ignition_on = true,
        .coolant_temperature = 255.0f,
        .coolant_level_low = false,
        .coolant_temp_sensor_valid = false
    };
    SystemOutputs outputs;

    evaluate_cooling_loop(&inputs, &outputs);

    EXPECT_EQ(outputs.current_state, STATE_FAULT);
    EXPECT_FALSE(outputs.pump_on);
    EXPECT_TRUE(outputs.fan_on);
}

TEST(CoolingLogicTest, FaultOnStuckTemperatureSensorAfterThreshold) {
    reset_cooling_safety_monitors();

    uint8_t pump_pwm = 0U;
    uint8_t fan_pwm = 0U;
    CoolingState state = STATE_OFF;

    for (uint16_t cycle = 0U; cycle < TEMP_STUCK_CYCLES + 1U; cycle++) {
        evaluate_system(NULL, true, 50.0f, false, &pump_pwm, &fan_pwm, &state);
    }

    EXPECT_EQ(state, STATE_FAULT);
    EXPECT_EQ(pump_pwm, 0U);
    EXPECT_EQ(fan_pwm, 100U);
}

TEST(CoolingLogicTest, SetpointsAreAppliedWithValidOrdering) {
    ControlSetpoints requested = {
        .target_temp_c = 42.0f,
        .high_temp_threshold_c = 40.0f,
        .critical_temp_threshold_c = 39.0f
    };

    set_control_setpoints(requested);
    ControlSetpoints applied = get_control_setpoints();

    EXPECT_FLOAT_EQ(applied.target_temp_c, 42.0f);
    EXPECT_GT(applied.high_temp_threshold_c, applied.target_temp_c);
    EXPECT_GT(applied.critical_temp_threshold_c, applied.high_temp_threshold_c);
}

TEST(CoolingLogicTest, PidStepProducesPositiveOutputWhenAboveSetpoint) {
    PID_Controller pid;
    init_pid(&pid, 4.0f, 0.2f, 0.8f);

    float output = pid_step(&pid, 40.0f, 50.0f, 1.0f);
    EXPECT_GT(output, 0.0f);
}

TEST(CoolingLogicTest, CanHostSetpointCommandUpdatesTarget) {
    ControlSetpoints baseline = {
        .target_temp_c = 40.0f,
        .high_temp_threshold_c = 45.0f,
        .critical_temp_threshold_c = 65.0f
    };
    set_control_setpoints(baseline);

    CAN_Frame rx = {
        .id = 0x300U,
        .dlc = 2,
        .data = {0}
    };

    uint16_t target_tenths = 385U;
    rx.data[0] = (uint8_t)(target_tenths & 0xFFU);
    rx.data[1] = (uint8_t)((target_tenths >> 8) & 0xFFU);

    EXPECT_TRUE(apply_host_setpoint_can_command(&rx));
    EXPECT_FLOAT_EQ(get_control_setpoints().target_temp_c, 38.5f);
}

TEST(CoolingLogicTest, CanHostSetpointCommandRejectsInvalidFrames) {
    ControlSetpoints baseline = {
        .target_temp_c = 40.0f,
        .high_temp_threshold_c = 45.0f,
        .critical_temp_threshold_c = 65.0f
    };
    set_control_setpoints(baseline);

    CAN_Frame wrong_id = {
        .id = 0x301U,
        .dlc = 2,
        .data = {0x90U, 0x01U}
    };
    CAN_Frame short_dlc = {
        .id = 0x300U,
        .dlc = 1,
        .data = {0x90U}
    };

    EXPECT_FALSE(apply_host_setpoint_can_command(&wrong_id));
    EXPECT_FALSE(apply_host_setpoint_can_command(&short_dlc));
    EXPECT_FLOAT_EQ(get_control_setpoints().target_temp_c, 40.0f);
}