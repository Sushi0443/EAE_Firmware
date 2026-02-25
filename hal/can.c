#include "include/can.h"

static CanMessage s_rx_message;
static bool s_rx_pending = false;

static CanMessage s_last_tx_message;
static bool s_tx_valid = false;

void can_init(void) {
	s_rx_pending = false;
	s_tx_valid = false;
}

bool can_simulate_receive(const CanMessage* message) {
	if (message == 0) {
		return false;
	}

	s_rx_message = *message;
	s_rx_pending = true;
	return true;
}

bool can_receive(CanMessage* message) {
	if ((message == 0) || (!s_rx_pending)) {
		return false;
	}

	*message = s_rx_message;
	s_rx_pending = false;
	return true;
}

bool can_transmit(const CanMessage* message) {
	if (message == 0) {
		return false;
	}

	s_last_tx_message = *message;
	s_tx_valid = true;
	return true;
}

bool can_get_last_tx(CanMessage* message) {
	if ((message == 0) || (!s_tx_valid)) {
		return false;
	}

	*message = s_last_tx_message;
	return true;
}

