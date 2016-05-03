/* Read analog voltage and current inputs */
#include "daq.h"

static int change_count = 0;

/* switch to ADC chan 8 (shorted to ground) to reset ADC measurement cap to zero before next measurement */
void ADC_zero(void)
{
	ClrWdt(); // reset the WDT timer
	SetChanADC(ADC_CH8); // F3 grounded input
	Delay10TCYx(ADC_CHAN_DELAY);
	ConvertADC();
	while (BusyADC());
}

void Reset_Change_Count(void) // counts for ADC value change while motor/resistor is moving
{
	change_count = 0;
	R.change_x = R.pos_x;
	R.change_y = R.pos_y;
	R.change_z = R.pos_z;
	R.stable_x = FALSE;
	R.stable_y = FALSE;
	R.stable_z = FALSE;
}

BYTE Change_Count(void) // Set the stable flag for non-moving motors or inactive motors
{
	return FALSE; // wait for CHANGE_COUNT times
}

void ADC_read(void) // update all voltage/current readings and set load current in 'currentload' variable
{ // ADC is opened and config'd in main
	static uint16_t i, z, change = 0; // used for fast and slow sample loops >256
	static int32_t a10_x_t, a10_y_t, a10_z_t;

	ClrWdt(); // reset the WDT timer
	ADC_zero();
	SetChanADC(ADC_CH0); // A0 system
	Delay10TCYx(ADC_CHAN_DELAY);
	Vin = 0;
	for (i = 0; i < ADC_SAMP_F; i++) {
		ConvertADC();
		while (BusyADC());
		Vin += (uint16_t) ReadADC();
	}
	Vin /= ADC_SAMP_F;
	vbatol_t = Vin * (ADC0_MV - ADC_NULL + adc_cal[0]); //      voltage correction factor
	vbatol_t = vbatol_t / 100;


	ADC_zero();
	SetChanADC(ADC_CH1); // A1 motor
	Delay10TCYx(ADC_CHAN_DELAY);
	Vin = 0;
	for (i = 0; i < ADC_SAMP_F; i++) {
		ConvertADC();
		while (BusyADC());
		Vin += (uint16_t) ReadADC();
	}
	Vin /= ADC_SAMP_F;
	solar_t = Vin * (ADC1_MV - ADC_NULL + adc_cal[1]);
	solar_t = solar_t / 100;


	ADC_zero();
	SetChanADC(ADC_CH2); // A2 current_x
	Delay10TCYx(ADC_CHAN_DELAY);
	Vin = 0;
	for (i = 0; i < ADC_SAMP_S; i++) {
		ConvertADC();
		while (BusyADC());
		Vin += (uint16_t) ReadADC();
	}
	Vin /= ADC_SAMP_S;
	a10_x = Vin; // raw ADC value


	ADC_zero();
	SetChanADC(ADC_CH11); // A11 current_y, switch from 3 so we can use external VREF
	Delay10TCYx(ADC_CHAN_DELAY);
	Vin = 0;
	for (i = 0; i < ADC_SAMP_S; i++) {
		ConvertADC();
		while (BusyADC());
		Vin += (uint16_t) ReadADC();
	}
	Vin /= ADC_SAMP_S; //
	a10_y = Vin; // raw ADC value


	ADC_zero();
	SetChanADC(ADC_CH4); // A5 current_z
	Delay10TCYx(ADC_CHAN_DELAY);
	Vin = 0;
	for (i = 0; i < ADC_SAMP_S; i++) {
		ConvertADC();
		while (BusyADC());
		Vin += (uint16_t) ReadADC();
	}
	Vin /= ADC_SAMP_S;
	a10_z = Vin;


	ADC_zero();
	SetChanADC(ADC_CH5); // F0 pot x
	Delay10TCYx(ADC_CHAN_DELAY);
	Vin = 0;
	for (i = 0; i < ADC_SAMP_F; i++) {
		ConvertADC();
		while (BusyADC());
		Vin += (uint16_t) ReadADC();
	}
	Vin /= ADC_SAMP_F;
	rawp[0] = Vin;

	ADC_zero();
	SetChanADC(ADC_CH6); // F1 pot y
	Delay10TCYx(ADC_CHAN_DELAY);
	Vin = 0;
	for (i = 0; i < ADC_SAMP_F; i++) {
		ConvertADC();
		while (BusyADC());
		Vin += (uint16_t) ReadADC();
	}
	Vin /= ADC_SAMP_F;
	rawp[1] = Vin;

	ADC_zero();
	SetChanADC(ADC_CH7); // F2 pot z
	Delay10TCYx(ADC_CHAN_DELAY);
	Vin = 0;
	for (i = 0; i < ADC_SAMP_F; i++) {
		ConvertADC();
		while (BusyADC());
		Vin += (uint16_t) ReadADC();
	}
	Vin /= ADC_SAMP_F;
	rawp[2] = Vin;

	ADC_zero();
	SetChanADC(ADC_CH8); // F0 pot max
	Delay10TCYx(ADC_CHAN_DELAY);
	Vin = 0;
	for (i = 0; i < ADC_SAMP_F; i++) {
		ConvertADC();
		while (BusyADC());
		Vin += (uint16_t) ReadADC();
	}
	Vin /= ADC_SAMP_F;
	rawa[0] = Vin;

	ADC_zero();
	SetChanADC(ADC_CH9); // F1 pot max
	Delay10TCYx(ADC_CHAN_DELAY);
	Vin = 0;
	for (i = 0; i < ADC_SAMP_F; i++) {
		ConvertADC();
		while (BusyADC());
		Vin += (uint16_t) ReadADC();
	}
	Vin /= ADC_SAMP_F;
	rawa[1] = Vin;

	ADC_zero();
	SetChanADC(ADC_CH10); // F2 pot max
	Delay10TCYx(ADC_CHAN_DELAY);
	Vin = 0;
	for (i = 0; i < ADC_SAMP_F; i++) {
		ConvertADC();
		while (BusyADC());
		Vin += (uint16_t) ReadADC();
	}
	Vin /= ADC_SAMP_F;
	rawa[2] = Vin;

	R.motorvoltage = solar_t;
	R.systemvoltage = vbatol_t;

	if (SYSTEM_STABLE) {
		s_crit(HL);
		R.current_z = a10_z_t;
		R.current_y = a10_y_t;
		R.current_x = a10_x_t;

		e_crit();

	}

	ADC_zero(); // ground ADC input
	ClrWdt(); // reset the WDT timer
}

void zero_amploc(void) // set zero current setpoint from ADC reading from a10_x a10_z a10_y, write to EEPROM
{
	static uint8_t z;

	adc_cal[11] = adc_cal[12] = adc_cal[13] = adc_cal[14] = ADC_NULL; // reset offsets to zero (ADC_NULL)
}
