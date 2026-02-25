#include "include/adc.h"

static float s_adc_values[ADC_CHANNEL_COUNT];

void adc_init(void) {
	for (uint8_t channel = 0U; channel < (uint8_t)ADC_CHANNEL_COUNT; channel++) {
		s_adc_values[channel] = 0.0f;
	}
}

void adc_set_simulated_value(AdcChannel channel, float value) {
	if (channel >= ADC_CHANNEL_COUNT) {
		return;
	}

	s_adc_values[channel] = value;
}

bool adc_read_channel(AdcChannel channel, float* value) {
	if ((channel >= ADC_CHANNEL_COUNT) || (value == 0)) {
		return false;
	}

	*value = s_adc_values[channel];
	return true;
}

