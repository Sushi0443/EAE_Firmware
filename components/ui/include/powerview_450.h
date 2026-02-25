#ifndef POWERVIEW_450_H
#define POWERVIEW_450_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
	float coolant_temp_c;
	uint8_t pump_pwm_pct;
	uint8_t fan_pwm_pct;
	bool level_low;
	uint8_t state_code;
} Powerview450Status;

void powerview_450_publish(Powerview450Status status);

#endif
