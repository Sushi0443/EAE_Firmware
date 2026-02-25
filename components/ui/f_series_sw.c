#include "include/f_series_sw.h"

#define F_SERIES_IGNITION_ON_THRESHOLD_V 6.0f

bool f_series_is_ignition_on(bool raw_switch_input) {
	return raw_switch_input;
}

bool f_series_is_ignition_on_from_adc(float adc_voltage_v) {
	return adc_voltage_v >= F_SERIES_IGNITION_ON_THRESHOLD_V;
}
