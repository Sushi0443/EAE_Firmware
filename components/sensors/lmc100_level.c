#include "include/lmc100_level.h"

bool lmc100_read_level(Lmc100Signals signals) {
	return (!signals.output_no) || signals.output_nc;
}
