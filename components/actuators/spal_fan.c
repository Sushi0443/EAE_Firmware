#include "include/spal_fan.h"
#include "../../hal/include/pwm.h"

uint8_t spal_fan_command(bool enable, uint8_t requested_pwm_pct) {
	uint8_t commanded_pwm;

	if (!enable) {
		pwm_set_duty(PWM_CH_FAN, 0U);
		return 0U;
	}

	if (requested_pwm_pct > SPAL_FAN_MAX_DUTY_PCT) {
		requested_pwm_pct = SPAL_FAN_MAX_DUTY_PCT;
	}

	if (requested_pwm_pct < SPAL_FAN_MIN_RUN_DUTY_PCT) {
		commanded_pwm = SPAL_FAN_MIN_RUN_DUTY_PCT;
		pwm_set_duty(PWM_CH_FAN, commanded_pwm);
		return commanded_pwm;
	}

	commanded_pwm = requested_pwm_pct;
	pwm_set_duty(PWM_CH_FAN, commanded_pwm);
	return commanded_pwm;
}
