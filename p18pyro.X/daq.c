/* Read analog voltage and current inputs */
#include "daq.h"

/* switch to ADC chan 8 (shorted to ground) to reset ADC measurement cap to zero before next measurement */
void ADC_zero(void)
{
	ClrWdt(); // reset the WDT timer
	SetChanADC(ADC_CH8); // F3 grounded input
	Delay10TCYx(ADC_CHAN_DELAY);
	ConvertADC();
	while (BusyADC());
}

void ADC_Update(uint16_t adc_val, uint8_t chan)
{
	if (chan >= ADC_INDEX)
		return;
	L.adc_raw[chan] = adc_val;
	L.adc_val[chan] = (adc_val * (ADC_5V_MV - ADC_NULL + adc_cal[chan])) / 100; //      voltage correction factor;
}

int8_t SPI_Out_Update(uint16_t data, uint8_t device, uint8_t ab)
{
	static union spi_buf_type spi_buf = {0};
	static union bytes2 upper_lower = {0};
	static union mcp4822_buf_type mcp4822_buf = {0};

	if (ringBufS_full(spi_link.tx1b)) return(-1);

	DLED_6 = HIGH;
	switch (device) {
	case 0:
	case 1:
		/*
		 * setup the mcp4822 register
		 */
		mcp4822_buf.buf = data & 0x0fff;
		mcp4822_buf.map.ab = ab;
		mcp4822_buf.map.ga = 0;
		mcp4822_buf.map.shdn = 1;
		upper_lower.ld = mcp4822_buf.buf; // load HL selector var
		/*
		 * setup ring-buffer for transfer in two parts
		 */
		spi_buf.map.buf = upper_lower.bd[1]; // load high byte
		spi_buf.map.select = device;
		spi_buf.map.load = 0;
		spi_buf.map.cs = 0;
		ringBufS_put(spi_link.tx1b, spi_buf.buf); // send data/control data to SPI devices (DAC)
		spi_buf.map.buf = upper_lower.bd[0]; // load low byte
		spi_buf.map.cs = 1;
		if (ringBufS_full(spi_link.tx1b)) return(-2); // second byte failed
		ringBufS_put(spi_link.tx1b, spi_buf.buf); // send data/control data to SPI devices (DAC)
		break;
	case 10: // retry second byte
		if (ringBufS_full(spi_link.tx1b)) return(-2);
		ringBufS_put(spi_link.tx1b, spi_buf.buf); // send data/control data to SPI devices (DAC)
		break;
	default:
		break;
		return(-1);
	}
	DLED_6 = LOW;
	return 0;
}