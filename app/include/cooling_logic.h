#ifndef COOLING_LOGIC_H
#define COOLING_LOGIC_H

#include <stdbool.h>
#include <stdint.h> // Required for fixed-width integers like uint8_t

#define TEMP_CRITICAL_THRESHOLD 65.0f
#define TEMP_HIGH_THRESHOLD 45.0f
#define TEMP_STUCK_DELTA_C 0.5f
#define TEMP_STUCK_CYCLES 300U
#define TARGET_TEMP 40.0f  // Legacy PID setpoint
#define DT 1.0f            // Time step (1 second per loop)

typedef enum {
    STATE_OFF,
    STATE_NORMAL_COOLING,
    STATE_MAX_COOLING,
    STATE_FAULT
} CoolingState;

typedef struct {
    bool ignition_on;
    float coolant_temperature;
    bool coolant_level_low;
    bool coolant_temp_sensor_valid;
} SystemInputs;

typedef struct {
    bool pump_on;
    bool fan_on;
    uint8_t pump_pwm_pct;
    uint8_t fan_pwm_pct;
    CoolingState current_state;
} SystemOutputs;

// --- CAN Bus Structures ---
typedef struct {
    uint32_t id;       // CAN Message ID
    uint8_t dlc;       // Data Length Code (0-8)
    uint8_t data[8];   // Payload
} CAN_Frame;

// --- PID Controller State ---
typedef struct {
    float kp, ki, kd;  // Tuning parameters
    float integral;    // Accumulated error
    float prev_error;  // Previous error for derivative
} PID_Controller;

typedef struct {
    float target_temp_c;
    float high_temp_threshold_c;
    float critical_temp_threshold_c;
} ControlSetpoints;

// Function prototypes
void init_pid(PID_Controller* pid, float p, float i, float d);
void evaluate_cooling_loop(const SystemInputs* inputs, SystemOutputs* outputs);
void decode_can_message(const CAN_Frame* rx_frame, bool* ignition, float* temp, bool* level_low);
void encode_can_message(CAN_Frame* tx_frame, uint8_t pump_pwm, uint8_t fan_pwm, CoolingState state);
void evaluate_system(PID_Controller* pid, bool ignition, float temp, bool level_low, uint8_t* pump_pwm, uint8_t* fan_pwm, CoolingState* state);
bool is_temperature_sensor_valid(float temp_c);
void reset_cooling_safety_monitors(void);
void set_control_setpoints(ControlSetpoints setpoints);
ControlSetpoints get_control_setpoints(void);
float pid_step(PID_Controller* pid, float setpoint, float measured, float dt);
bool apply_host_setpoint_can_command(const CAN_Frame* rx_frame);

#endif // COOLING_LOGIC_H