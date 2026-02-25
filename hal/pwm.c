#include "include/pwm.h"

static uint8_t s_pwm_duty[PWM_CHANNEL_COUNT];

void pwm_init(void) {
	for (uint8_t channel = 0U; channel < (uint8_t)PWM_CHANNEL_COUNT; channel++) {
		s_pwm_duty[channel] = 0U;
	}
}

bool pwm_set_duty(PwmChannel channel, uint8_t duty_percent) {
	if (channel >= PWM_CHANNEL_COUNT) {
		return false;
	}

	if (duty_percent > 100U) {
		duty_percent = 100U;
	}

	s_pwm_duty[channel] = duty_percent;
	return true;
}

bool pwm_get_duty(PwmChannel channel, uint8_t* duty_percent) {
	if ((channel >= PWM_CHANNEL_COUNT) || (duty_percent == 0)) {
		return false;
	}

	*duty_percent = s_pwm_duty[channel];
	return true;
}

