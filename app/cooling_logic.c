#include "cooling_logic.h"
#include "../components/actuators/include/spal_fan.h"
#include "../components/actuators/include/wp_pump.h"
#include <stddef.h>

#define COOLING_CAN_ID_STATUS       0x100U
#define COOLING_CAN_ID_COMMAND      0x200U
#define COOLING_CAN_ID_SETPOINT     0x300U
#define COOLING_CAN_TEMP_INVALID    0xFFU

static float s_prev_temp_c = 0.0f;
static bool s_prev_temp_valid = false;
static uint16_t s_stable_temp_cycles = 0U;
static ControlSetpoints s_setpoints = {
    .target_temp_c = TARGET_TEMP,
    .high_temp_threshold_c = TEMP_HIGH_THRESHOLD,
    .critical_temp_threshold_c = TEMP_CRITICAL_THRESHOLD
};

// --- PID Initialization ---
void init_pid(PID_Controller* pid, float p, float i, float d) {
    if (pid == NULL) {
        return;
    }

    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float pid_step(PID_Controller* pid, float setpoint, float measured, float dt) {
    float error;
    float derivative;
    float output;

    if ((pid == NULL) || (dt <= 0.0f)) {
        return 0.0f;
    }

    error = measured - setpoint;
    pid->integral += error * dt;
    derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    return output;
}

void set_control_setpoints(ControlSetpoints setpoints) {
    if (setpoints.high_temp_threshold_c <= setpoints.target_temp_c) {
        setpoints.high_temp_threshold_c = setpoints.target_temp_c + 1.0f;
    }

    if (setpoints.critical_temp_threshold_c <= setpoints.high_temp_threshold_c) {
        setpoints.critical_temp_threshold_c = setpoints.high_temp_threshold_c + 1.0f;
    }

    s_setpoints = setpoints;
}

ControlSetpoints get_control_setpoints(void) {
    return s_setpoints;
}

bool apply_host_setpoint_can_command(const CAN_Frame* rx_frame) {
    uint16_t target_tenths;
    ControlSetpoints updated_setpoints;

    if ((rx_frame == NULL) || (rx_frame->id != COOLING_CAN_ID_SETPOINT) || (rx_frame->dlc < 2U)) {
        return false;
    }

    target_tenths = (uint16_t)rx_frame->data[0] | ((uint16_t)rx_frame->data[1] << 8);
    updated_setpoints = get_control_setpoints();
    updated_setpoints.target_temp_c = ((float)target_tenths) / 10.0f;
    set_control_setpoints(updated_setpoints);
    return true;
}

bool is_temperature_sensor_valid(float temp_c) {
    return temp_c >= 0.0f && temp_c <= 160.0f;
}

void reset_cooling_safety_monitors(void) {
    s_prev_temp_c = 0.0f;
    s_prev_temp_valid = false;
    s_stable_temp_cycles = 0U;
}

static bool is_temperature_sensor_stuck(bool ignition_on, bool temp_sensor_valid, float current_temp_c) {
    float delta_c;

    if (!ignition_on || !temp_sensor_valid) {
        reset_cooling_safety_monitors();
        return false;
    }

    if (!s_prev_temp_valid) {
        s_prev_temp_c = current_temp_c;
        s_prev_temp_valid = true;
        s_stable_temp_cycles = 0U;
        return false;
    }

    delta_c = current_temp_c - s_prev_temp_c;
    if (delta_c < 0.0f) {
        delta_c = -delta_c;
    }

    if (delta_c <= TEMP_STUCK_DELTA_C) {
        if (s_stable_temp_cycles < 0xFFFFU) {
            s_stable_temp_cycles++;
        }
    } else {
        s_stable_temp_cycles = 0U;
    }

    s_prev_temp_c = current_temp_c;

    return s_stable_temp_cycles >= TEMP_STUCK_CYCLES;
}

// --- CAN Parsing (Simulating receiving data from the vehicle) ---
void decode_can_message(const CAN_Frame* rx_frame, bool* ignition, float* temp, bool* level_low) {
    if ((ignition == NULL) || (temp == NULL) || (level_low == NULL)) {
        return;
    }

    *ignition = false;
    *temp = (float)COOLING_CAN_TEMP_INVALID;
    *level_low = true;

    if ((rx_frame == NULL) || (rx_frame->id != COOLING_CAN_ID_STATUS) || (rx_frame->dlc < 3U)) {
        return;
    }

    *ignition = (rx_frame->data[0] != 0U);
    if (rx_frame->data[1] == COOLING_CAN_TEMP_INVALID) {
        *temp = (float)COOLING_CAN_TEMP_INVALID;
    } else {
        *temp = (float)rx_frame->data[1];
    }
    *level_low = (rx_frame->data[2] != 0U);
}

// --- CAN Packing (Simulating sending commands to the pump/fan) ---
void encode_can_message(CAN_Frame* tx_frame, uint8_t pump_pwm, uint8_t fan_pwm, CoolingState state) {
    tx_frame->id = COOLING_CAN_ID_COMMAND; // Assume 0x200 is the Cooling Command Message
    tx_frame->dlc = 3;
    tx_frame->data[0] = pump_pwm;
    tx_frame->data[1] = fan_pwm;
    tx_frame->data[2] = (uint8_t)state;
}

void evaluate_cooling_loop(const SystemInputs* inputs, SystemOutputs* outputs) {
    if ((inputs == NULL) || (outputs == NULL)) {
        return;
    }

    outputs->current_state = STATE_OFF;
    outputs->pump_on = false;
    outputs->fan_on = false;
    outputs->pump_pwm_pct = 0U;
    outputs->fan_pwm_pct = 0U;

    if (!inputs->ignition_on) {
        return;
    }

    if (!inputs->coolant_temp_sensor_valid) {
        outputs->current_state = STATE_FAULT;
        outputs->pump_on = false;
        outputs->fan_on = true;
        outputs->pump_pwm_pct = wp_pump_command(false, 0U);
        outputs->fan_pwm_pct = spal_fan_command(true, 100U);
        return;
    }

    if (inputs->coolant_level_low || inputs->coolant_temperature >= s_setpoints.critical_temp_threshold_c) {
        outputs->current_state = STATE_FAULT;
        outputs->pump_on = false;
        outputs->fan_on = true;
        outputs->pump_pwm_pct = wp_pump_command(false, 0U);
        outputs->fan_pwm_pct = spal_fan_command(true, 100U);
        return;
    }

    if (inputs->coolant_temperature >= s_setpoints.high_temp_threshold_c) {
        outputs->current_state = STATE_MAX_COOLING;
        outputs->pump_on = true;
        outputs->fan_on = true;
        outputs->pump_pwm_pct = wp_pump_command(true, 100U);
        outputs->fan_pwm_pct = spal_fan_command(true, 100U);
        return;
    }

    outputs->current_state = STATE_NORMAL_COOLING;
    outputs->pump_on = true;
    outputs->fan_on = false;
    outputs->pump_pwm_pct = wp_pump_command(true, 60U);
    outputs->fan_pwm_pct = spal_fan_command(false, 0U);
}

// --- Legacy compatibility entry point used by firmware task ---
void evaluate_system(PID_Controller* pid, bool ignition, float temp, bool level_low, uint8_t* pump_pwm, uint8_t* fan_pwm, CoolingState* state) {
    bool sensor_valid;
    bool sensor_stuck;

    if ((pump_pwm == NULL) || (fan_pwm == NULL) || (state == NULL)) {
        return;
    }

    sensor_valid = is_temperature_sensor_valid(temp);
    sensor_stuck = is_temperature_sensor_stuck(ignition, sensor_valid, temp);

    SystemInputs inputs = {
        .ignition_on = ignition,
        .coolant_temperature = temp,
        .coolant_level_low = level_low,
        .coolant_temp_sensor_valid = sensor_valid && !sensor_stuck
    };

    SystemOutputs outputs;
    evaluate_cooling_loop(&inputs, &outputs);

    *pump_pwm = outputs.pump_pwm_pct;
    *fan_pwm = outputs.fan_pwm_pct;
    *state = outputs.current_state;

    if ((pid != NULL) && !ignition) {
        pid->integral = 0.0f;
        pid->prev_error = 0.0f;
    }

    if (!ignition) {
        reset_cooling_safety_monitors();
    }
}