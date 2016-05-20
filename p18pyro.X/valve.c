#include "valve.h"

struct valvetype valves;

void valve_config(void)
{
	valves.buf.ld = 0;
	valve_set(&valves);
}

int8_t valve_interlock(void)
{
	return 0;
}

int8_t valve_set(struct valvetype * valves)
{
	if (SPI_Daq_Update(valves->buf.bd[0], SHIFT_565_0_7, 0))
		return -1;
	return SPI_Daq_Update(valves->buf.bd[1], SHIFT_565_8_15, 0);
}

int8_t valve_switch(const uint8_t valve, uint8_t device, const uint8_t state)
{
	device &= 0x01;
	if (state) {
		valves.buf.bd[device] |= 0x01 << valve;
		valves.buf.bd[device] |= 0x01 << valve;
	} else {
		valves.buf.bd[device] &= ~(0x01 << valve);
		valves.buf.bd[device] &= ~(0x01 << valve);
	}

	return valve_set(&valves);
}