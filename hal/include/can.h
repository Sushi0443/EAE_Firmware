#ifndef HAL_CAN_H
#define HAL_CAN_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
	uint32_t id;
	uint8_t dlc;
	uint8_t data[8];
} CanMessage;

void can_init(void);
bool can_simulate_receive(const CanMessage* message);
bool can_receive(CanMessage* message);
bool can_transmit(const CanMessage* message);
bool can_get_last_tx(CanMessage* message);

#endif

