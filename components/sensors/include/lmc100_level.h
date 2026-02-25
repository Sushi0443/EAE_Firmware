#ifndef LMC100_LEVEL_H
#define LMC100_LEVEL_H

#include <stdbool.h>

typedef struct {
	bool output_no;
	bool output_nc;
} Lmc100Signals;

bool lmc100_read_level(Lmc100Signals signals);

#endif
