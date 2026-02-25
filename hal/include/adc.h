#ifndef HAL_ADC_H
#define HAL_ADC_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	ADC_CH_IGNITION_VOLTAGE = 0,
	ADC_CH_TEMP_SENSOR_OHMS,
	ADC_CHANNEL_COUNT
} AdcChannel;

void adc_init(void);
void adc_set_simulated_value(AdcChannel channel, float value);
bool adc_read_channel(AdcChannel channel, float* value);

#endif

