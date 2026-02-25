#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "cooling_logic.h"
#include "../components/sensors/include/hwtms_temp.h"
#include "../components/sensors/include/lmc100_level.h"
#include "../components/ui/include/f_series_sw.h"
#include "../components/actuators/include/wp_pump.h"
#include "../hal/include/adc.h"
#include "../hal/include/gpio.h"
#include "../hal/include/pwm.h"
#include "../hal/include/can.h"

#define CAN_ID_HOST_SETPOINT 0x300U

typedef struct {
    ControlSetpoints setpoints;
    float kp;
    float ki;
    float kd;
    int steps;
    int sleep_s;
} RuntimeConfig;

static void print_can_message(const CanMessage* message, const char* direction) {
    printf("%s [ID: 0x%X] Data: ", direction, message->id);
    for (int i = 0; i < message->dlc; i++) {
        printf("%02X ", message->data[i]);
    }
    printf("\n");
}

static RuntimeConfig get_default_config(void) {
    RuntimeConfig cfg;
    cfg.setpoints = get_control_setpoints();
    cfg.kp = 4.0f;
    cfg.ki = 0.2f;
    cfg.kd = 0.8f;
    cfg.steps = 8;
    cfg.sleep_s = 1;
    return cfg;
}

static bool parse_cli_args(int argc, char* argv[], RuntimeConfig* cfg) {
    for (int i = 1; i < argc; i++) {
        if ((strcmp(argv[i], "--target") == 0) && (i + 1 < argc)) {
            cfg->setpoints.target_temp_c = (float)atof(argv[++i]);
        } else if ((strcmp(argv[i], "--high") == 0) && (i + 1 < argc)) {
            cfg->setpoints.high_temp_threshold_c = (float)atof(argv[++i]);
        } else if ((strcmp(argv[i], "--critical") == 0) && (i + 1 < argc)) {
            cfg->setpoints.critical_temp_threshold_c = (float)atof(argv[++i]);
        } else if ((strcmp(argv[i], "--kp") == 0) && (i + 1 < argc)) {
            cfg->kp = (float)atof(argv[++i]);
        } else if ((strcmp(argv[i], "--ki") == 0) && (i + 1 < argc)) {
            cfg->ki = (float)atof(argv[++i]);
        } else if ((strcmp(argv[i], "--kd") == 0) && (i + 1 < argc)) {
            cfg->kd = (float)atof(argv[++i]);
        } else if ((strcmp(argv[i], "--steps") == 0) && (i + 1 < argc)) {
            cfg->steps = atoi(argv[++i]);
        } else if ((strcmp(argv[i], "--sleep") == 0) && (i + 1 < argc)) {
            cfg->sleep_s = atoi(argv[++i]);
        } else {
            printf("Unknown argument: %s\n", argv[i]);
            return false;
        }
    }

    if (cfg->steps < 1) {
        cfg->steps = 1;
    }
    if (cfg->sleep_s < 0) {
        cfg->sleep_s = 0;
    }

    return true;
}

static void system_init(void) {
    adc_init();
    gpio_init();
    pwm_init();
    can_init();
    reset_cooling_safety_monitors();
}

static void simulate_host_setpoint_command(float target_temp_c) {
    CanMessage rx = {0};
    uint16_t target_tenths = (uint16_t)(target_temp_c * 10.0f);

    rx.id = CAN_ID_HOST_SETPOINT;
    rx.dlc = 2;
    rx.data[0] = (uint8_t)(target_tenths & 0xFFU);
    rx.data[1] = (uint8_t)((target_tenths >> 8) & 0xFFU);
    (void)can_simulate_receive(&rx);
}

static void handle_can_rx_commands(void) {
    CanMessage rx;
    while (can_receive(&rx)) {
        CAN_Frame control_rx = {0};

        print_can_message(&rx, "RX");

        control_rx.id = rx.id;
        control_rx.dlc = rx.dlc;
        for (int byte_idx = 0; byte_idx < rx.dlc; byte_idx++) {
            control_rx.data[byte_idx] = rx.data[byte_idx];
        }

        if (apply_host_setpoint_can_command(&control_rx)) {
            printf("Applied CAN setpoint update -> target: %.1fC\n", get_control_setpoints().target_temp_c);
        }
    }
}

int main(int argc, char* argv[]) {
    RuntimeConfig cfg = get_default_config();
    PID_Controller pid;

    if (!parse_cli_args(argc, argv, &cfg)) {
        printf("Usage: ./cooling_loop [--target C] [--high C] [--critical C] [--kp N] [--ki N] [--kd N] [--steps N] [--sleep S]\n");
        return 1;
    }

    set_control_setpoints(cfg.setpoints);
    init_pid(&pid, cfg.kp, cfg.ki, cfg.kd);

    printf("--- EAE Cooling Control System (Section 7.1 Firmware) ---\n");
    system_init();

    // Simulation variables
    float simulated_ignition_v[] = {0.0f, 12.0f, 12.0f, 12.0f, 12.0f, 12.0f, 12.0f, 12.0f};
    float simulated_temp_ohms[] = {3457.0f, 2830.0f, 1443.0f, 992.0f, 660.0f, 475.0f, 500000.0f, 992.0f};
    bool simulated_level_no[] = {true, true, true, true, true, false, true, true};
    bool simulated_level_nc[] = {false, false, false, false, false, true, false, false};

    int available_steps = (int)(sizeof(simulated_temp_ohms) / sizeof(simulated_temp_ohms[0]));

    printf("Setpoints -> target=%.1fC high=%.1fC critical=%.1fC\n",
           cfg.setpoints.target_temp_c,
           cfg.setpoints.high_temp_threshold_c,
           cfg.setpoints.critical_temp_threshold_c);
    printf("PID gains -> kp=%.2f ki=%.2f kd=%.2f\n", cfg.kp, cfg.ki, cfg.kd);

    for (int i = 0; i < cfg.steps; i++) {
        int sample_index = i % available_steps;

        printf("\n--- Time Step %d ---\n", i);

        // 1. Simulate ADC/GPIO inputs
        adc_set_simulated_value(ADC_CH_IGNITION_VOLTAGE, simulated_ignition_v[sample_index]);
        adc_set_simulated_value(ADC_CH_TEMP_SENSOR_OHMS, simulated_temp_ohms[sample_index]);
        gpio_set_simulated_input(GPIO_PIN_LEVEL_NO, simulated_level_no[sample_index]);
        gpio_set_simulated_input(GPIO_PIN_LEVEL_NC, simulated_level_nc[sample_index]);

        // 1a. Simulate receiving setpoint command from host over CANBUS
        if (i == 2) {
            simulate_host_setpoint_command(cfg.setpoints.target_temp_c - 3.0f);
        }
        handle_can_rx_commands();

        // 2. Read and decode physical inputs
        float ignition_voltage_v;
        bool ignition_on = false;
        float current_temp;
        bool sensor_valid;
        bool level_no;
        bool level_nc;
        bool level_low = true;

        if (adc_read_channel(ADC_CH_IGNITION_VOLTAGE, &ignition_voltage_v)) {
            ignition_on = f_series_is_ignition_on_from_adc(ignition_voltage_v);
        }

        if (!hwtms_read_temp_c(&current_temp, &sensor_valid)) {
            current_temp = 0.0f;
            sensor_valid = false;
        }

        if (gpio_read_pin(GPIO_PIN_LEVEL_NO, &level_no) && gpio_read_pin(GPIO_PIN_LEVEL_NC, &level_nc)) {
            Lmc100Signals level_signals = {
                .output_no = level_no,
                .output_nc = level_nc
            };
            level_low = lmc100_read_level(level_signals);
        }

        // 3. Run the Control Logic
        uint8_t pump_speed, fan_speed;
        CoolingState current_state;
        evaluate_system(&pid, ignition_on, current_temp, level_low, &pump_speed, &fan_speed, &current_state);

        // 3a. Apply PID trim in non-fault active cooling states
        if (ignition_on && sensor_valid && !level_low && (current_state != STATE_FAULT)) {
            float pid_out = pid_step(&pid, get_control_setpoints().target_temp_c, current_temp, DT);
            int pid_pwm = 50 + (int)pid_out;
            if (pid_pwm < 0) {
                pid_pwm = 0;
            }
            if (pid_pwm > 100) {
                pid_pwm = 100;
            }

            if (current_state == STATE_NORMAL_COOLING) {
                pump_speed = wp_pump_command(true, (uint8_t)pid_pwm);
            }
        }

        // 4. Read PWM outputs commanded by actuator modules
        uint8_t pump_pwm_out = 0U;
        uint8_t fan_pwm_out = 0U;
        (void)pwm_get_duty(PWM_CH_PUMP, &pump_pwm_out);
        (void)pwm_get_duty(PWM_CH_FAN, &fan_pwm_out);

        // 5. Simulate CAN status transmit
        CAN_Frame status_frame;
        CanMessage tx = {0};
        encode_can_message(&status_frame, pump_pwm_out, fan_pwm_out, current_state);
        tx.id = status_frame.id;
        tx.dlc = status_frame.dlc;
        for (int byte_idx = 0; byte_idx < status_frame.dlc; byte_idx++) {
            tx.data[byte_idx] = status_frame.data[byte_idx];
        }
        (void)can_transmit(&tx);
        print_can_message(&tx, "TX");

        if ((current_state == STATE_FAULT) && (!sensor_valid)) {
            printf("Inputs -> Ign: %.1fV | Temp: INVALID | LevelLow: %s\n",
                   ignition_voltage_v,
                   level_low ? "YES" : "NO");
            printf("Outputs -> Pump PWM: %u%% | Fan PWM: %u%%\n", pump_pwm_out, fan_pwm_out);
        } else {
            printf("Inputs -> Ign: %.1fV | Temp: %.1fC | LevelLow: %s\n",
                   ignition_voltage_v,
                   current_temp,
                   level_low ? "YES" : "NO");
            printf("Outputs -> State: %u | Pump PWM: %u%% | Fan PWM: %u%%\n",
                   (unsigned)current_state,
                   pump_pwm_out,
                   fan_pwm_out);
        }

        sleep((unsigned int)cfg.sleep_s);
    }

    return 0;
}