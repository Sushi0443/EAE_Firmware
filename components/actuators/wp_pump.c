#include "include/wp_pump.h"
#include "../../hal/include/pwm.h"

uint8_t wp_pump_command(bool enable, uint8_t requested_pwm_pct) {
	uint8_t commanded_pwm;

	if (!enable) {
		pwm_set_duty(PWM_CH_PUMP, 0U);
		return 0U;
	}

	if (requested_pwm_pct > WP_PUMP_MAX_DUTY_PCT) {
		requested_pwm_pct = WP_PUMP_MAX_DUTY_PCT;
	}

	if (requested_pwm_pct < WP_PUMP_MIN_START_DUTY_PCT) {
		commanded_pwm = WP_PUMP_MIN_START_DUTY_PCT;
		pwm_set_duty(PWM_CH_PUMP, commanded_pwm);
		return commanded_pwm;
	}

	commanded_pwm = requested_pwm_pct;
	pwm_set_duty(PWM_CH_PUMP, commanded_pwm);
	return commanded_pwm;
}
