#ifndef F_SERIES_SW_H
#define F_SERIES_SW_H

#include <stdbool.h>

bool f_series_is_ignition_on(bool raw_switch_input);
bool f_series_is_ignition_on_from_adc(float adc_voltage_v);

#endif
