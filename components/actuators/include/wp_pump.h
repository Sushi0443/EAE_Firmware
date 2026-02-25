#ifndef WP_PUMP_H
#define WP_PUMP_H

#include <stdbool.h>
#include <stdint.h>

#define WP_PUMP_MIN_START_DUTY_PCT 20U
#define WP_PUMP_MAX_DUTY_PCT       100U

uint8_t wp_pump_command(bool enable, uint8_t requested_pwm_pct);

#endif
