#ifndef HWTMS_TEMP_H
#define HWTMS_TEMP_H

#include <stdbool.h>

#include "../../../hal/include/adc.h"

#define HWTMS_MIN_TEMP_C   (-20.0f)
#define HWTMS_MAX_TEMP_C   (160.0f)
#define HWTMS_MIN_OHMS     (47.0f)
#define HWTMS_MAX_OHMS     (28146.0f)

bool hwtms_is_resistance_valid(float resistance_ohm);
float hwtms_resistance_to_temp_c(float resistance_ohm);
bool hwtms_read_temp_c(float* temp_c, bool* sensor_valid);

#endif
