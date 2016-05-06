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
