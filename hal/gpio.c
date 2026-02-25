#include "include/gpio.h"

static bool s_gpio_levels[GPIO_PIN_COUNT];

void gpio_init(void) {
	for (uint8_t pin = 0U; pin < (uint8_t)GPIO_PIN_COUNT; pin++) {
		s_gpio_levels[pin] = false;
	}
}

void gpio_set_simulated_input(GpioPin pin, bool level_high) {
	if (pin >= GPIO_PIN_COUNT) {
		return;
	}

	s_gpio_levels[pin] = level_high;
}

bool gpio_read_pin(GpioPin pin, bool* level_high) {
	if ((pin >= GPIO_PIN_COUNT) || (level_high == 0)) {
		return false;
	}

	*level_high = s_gpio_levels[pin];
	return true;
}

