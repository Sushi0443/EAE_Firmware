#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	GPIO_PIN_LEVEL_NO = 0,
	GPIO_PIN_LEVEL_NC,
	GPIO_PIN_COUNT
} GpioPin;

void gpio_init(void);
void gpio_set_simulated_input(GpioPin pin, bool level_high);
bool gpio_read_pin(GpioPin pin, bool* level_high);

#endif

