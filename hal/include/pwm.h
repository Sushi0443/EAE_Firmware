#ifndef HAL_PWM_H
#define HAL_PWM_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	PWM_CH_PUMP = 0,
	PWM_CH_FAN,
	PWM_CHANNEL_COUNT
} PwmChannel;

void pwm_init(void);
bool pwm_set_duty(PwmChannel channel, uint8_t duty_percent);
bool pwm_get_duty(PwmChannel channel, uint8_t* duty_percent);

#endif

