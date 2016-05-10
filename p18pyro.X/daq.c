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
	union spi_buf_type spi_buf = {0};
	union bytes2 upper_lower = {0};
	union mcp4822_buf_type mcp4822_buf = {0};

	DLED_6 = HIGH;
	switch (device) {
	case 0:
	case 1:
		/*
		 * setup the mcp4822 register
		 */
		mcp4822_buf.buf = data;
		mcp4822_buf.map.ab = ab;
		mcp4822_buf.map.ga = 1;
		mcp4822_buf.map.shdn = 1;
		upper_lower.ld = mcp4822_buf.buf;
		/*
		 * setup ring-buffer for transfer in two parts
		 */
		spi_buf.map.buf = upper_lower.bd[1];
		spi_buf.map.bits16 = 1;
		spi_buf.map.upper = 1;
		spi_buf.map.select = device;
		spi_buf.map.load=0;
		spi_buf.map.cs = 0;
		ringBufS_put(spi_link.tx1b, spi_buf.buf); // send data/control data to SPI devices (DAC)
		spi_buf.map.buf = upper_lower.bd[0];
		spi_buf.map.upper = 0;
		spi_buf.map.cs = 1;
		ringBufS_put(spi_link.tx1b, spi_buf.buf); // send data/control data to SPI devices (DAC)
		break;
	default:
		break;
		return(-1);
	}
	DLED_6 = LOW;
	return 0;
}