#ifndef SPAL_FAN_H
#define SPAL_FAN_H

#include <stdbool.h>
#include <stdint.h>

#define SPAL_FAN_MIN_RUN_DUTY_PCT 15U
#define SPAL_FAN_MAX_DUTY_PCT     100U

uint8_t spal_fan_command(bool enable, uint8_t requested_pwm_pct);

#endif
