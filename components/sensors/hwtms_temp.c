#include "include/hwtms_temp.h"

typedef struct {
	float resistance_ohm;
	float temp_c;
} HwtmsTableEntry;

static const HwtmsTableEntry k_hwtms_table[] = {
	{28146.0f, -20.0f},
	{15873.0f, -10.0f},
	{9256.0f, 0.0f},
	{5572.0f, 10.0f},
	{3457.0f, 20.0f},
	{2830.0f, 30.0f},
	{1443.0f, 40.0f},
	{992.0f, 50.0f},
	{660.0f, 60.0f},
	{475.0f, 70.0f},
	{329.0f, 80.0f},
	{244.0f, 90.0f},
	{175.0f, 100.0f},
	{134.0f, 110.0f},
	{99.0f, 120.0f},
	{60.0f, 140.0f},
	{47.0f, 160.0f}
};

bool hwtms_is_resistance_valid(float resistance_ohm) {
	return resistance_ohm >= HWTMS_MIN_OHMS && resistance_ohm <= HWTMS_MAX_OHMS;
}

float hwtms_resistance_to_temp_c(float resistance_ohm) {
	const int table_size = (int)(sizeof(k_hwtms_table) / sizeof(k_hwtms_table[0]));

	if (resistance_ohm >= k_hwtms_table[0].resistance_ohm) {
		return k_hwtms_table[0].temp_c;
	}

	if (resistance_ohm <= k_hwtms_table[table_size - 1].resistance_ohm) {
		return k_hwtms_table[table_size - 1].temp_c;
	}

	for (int i = 0; i < table_size - 1; i++) {
		const HwtmsTableEntry* high = &k_hwtms_table[i];
		const HwtmsTableEntry* low = &k_hwtms_table[i + 1];

		if (resistance_ohm <= high->resistance_ohm && resistance_ohm >= low->resistance_ohm) {
			const float ratio = (resistance_ohm - low->resistance_ohm) /
								(high->resistance_ohm - low->resistance_ohm);
			return low->temp_c + ratio * (high->temp_c - low->temp_c);
		}
	}

	return HWTMS_MAX_TEMP_C;
}

bool hwtms_read_temp_c(float* temp_c, bool* sensor_valid) {
	float resistance_ohm;

	if ((temp_c == 0) || (sensor_valid == 0)) {
		return false;
	}

	if (!adc_read_channel(ADC_CH_TEMP_SENSOR_OHMS, &resistance_ohm)) {
		*temp_c = 0.0f;
		*sensor_valid = false;
		return false;
	}

	*sensor_valid = hwtms_is_resistance_valid(resistance_ohm);
	if (!(*sensor_valid)) {
		*temp_c = HWTMS_MIN_TEMP_C - 100.0f;
		return true;
	}

	*temp_c = hwtms_resistance_to_temp_c(resistance_ohm);
	return true;
}
