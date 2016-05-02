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
	if (change_count++ >= CHANGE_COUNT) {
		if ((ABSL(R.pos_x - R.change_x) < MIN_CHANGE) || !motordata[0].active) R.stable_x = TRUE;
		if ((ABSL(R.pos_y - R.change_y) < MIN_CHANGE) || !motordata[1].active) R.stable_y = TRUE;
		if ((ABSL(R.pos_z - R.change_z) < MIN_CHANGE) || !motordata[1].active) R.stable_z = TRUE;
		change_count = CHANGE_COUNT;
		return TRUE;
	}
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

	Vin = Vin - (AMP10_ZERO - ADC_NULL + adc_cal[11]); // set zero NULL0 point
	a10_x_t = (long) Vin * (long) (ADC2_MV - ADC_NULL + adc_cal[2]);
	if (ABSL(a10_x_t) < AMPZ)
		a10_x_t = 0; // zero bit noise
	a10_x_t = (long) ((float) a10_x_t / (float) AMP10_SEN_H);
	a10_x_t = (long) lp_filter((float) a10_x_t, LP_CURRENT_X, FALSE); // use digital filter
	if ((a10_x == NULL0) || (a10_x_t < 0.0)) a10_x_t = 0; // if sensor disconnected read zero

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

	Vin = Vin - (AMP10_ZERO - ADC_NULL + adc_cal[12]);
	a10_y_t = (long) Vin * (long) (ADC3_MV - ADC_NULL + adc_cal[3]);
	if (ABSL(a10_y_t) < AMPZ)
		a10_y_t = 0; // zero bit noise
	a10_y_t = (long) ((float) a10_y_t) / ((float) AMP10_SEN_L);
	a10_y_t = (long) lp_filter((float) a10_y_t, LP_CURRENT_Y, FALSE); //      use digital filter
	if ((a10_y == NULL0) || (a10_y_t < 0.0)) a10_y_t = 0; // if sensor disconnected (zero) read zero current

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
	Vin = Vin - (AMP10_ZERO - ADC_NULL + adc_cal[13]); // set zero NULL0 point
	a10_z_t = (long) Vin * (long) (ADC4_MV - ADC_NULL + adc_cal[4]);
	if (ABSL(a10_z_t) < AMPZ)
		a10_z_t = 0; // zero bit noise
	a10_z_t = (long) ((float) a10_z_t / (float) AMP10_SEN_L);
	a10_z_t = (long) lp_filter((float) a10_z_t, LP_CURRENT_Z, FALSE);
	if ((a10_z == NULL0) || (a10_z_t < 0.0)) a10_z_t = 0; // if sensor disconnected read zero

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
		R.pos_x = rawp[XAXIS];
		R.pos_y = rawp[YAXIS];
		R.pos_z = rawp[ZAXIS];
		R.max_x = rawa[XAXIS];
		R.max_y = rawa[YAXIS];
		R.max_z = rawa[ZAXIS];
		e_crit();
		for (z = 0; z < MAX_POT; z++) {
			motordata[z].pot.pos_actual = rawp[z];
			if (ABSI(motordata[z].pot.pos_actual - motordata[z].pot.pos_actual_prev) > motordata[z].pot.pos_change) {
				motordata[z].pot.pos_change = ABSI(motordata[z].pot.pos_actual - motordata[z].pot.pos_actual_prev);
			}
			if (motordata[z].active && (motordata[z].pot.pos_change > motordata[z].pot.limit_change)) {
				if (mode.change) {
					term_time();
					sprintf(bootstr2, " Pot %i Change too high %i\r\n", z, motordata[z].pot.pos_change);
					puts2USART(bootstr2);
					motordata[z].pot.pos_change = motordata[z].pot.limit_change; // after one message stop and set it to the limit.
				} else {
					motordata[z].pot.pos_actual = rawp[z]; // set to current pot reading
					motordata[z].pot.pos_actual_prev = rawp[z];
					motordata[z].pot.pos_change = 0; // pot position change mag
				}
			}
			// Check for POT Dead-Spot readings
			if ((motordata[z].pot.pos_change >= POT_MAX_CHANGE) && !mode.qei) motordata[z].pot.cal_failed = TRUE;
			if (!motordata[z].active) motordata[z].pot.cal_failed = FALSE; // don't fail inactive motors

			motordata[z].pot.pos_actual_prev = motordata[z].pot.pos_actual;
			if (motordata[z].pot.pos_actual > motordata[z].pot.high) motordata[z].pot.high = motordata[z].pot.pos_actual; // set adc limits of values X,Y,Z
			if (motordata[z].pot.pos_actual < motordata[z].pot.low) motordata[z].pot.low = motordata[z].pot.pos_actual;
			if (mode.operate == VIISION_MS) { // use the preset o-100 resistance limits for scaling
				motordata[z].pot.high = VIISION_MS_RES_HIGH;
				motordata[z].pot.low = VIISION_MS_RES_LOW;
			}
			motordata[z].pot.offset = motordata[z].pot.low;
			motordata[z].pot.span = motordata[z].pot.high - motordata[z].pot.low;
			if (motordata[z].pot.span < 0) motordata[z].pot.span = 0;
			motordata[z].pot.scale_out = SCALED_FLOAT / motordata[z].pot.span;
			motordata[z].pot.scale_in = motordata[z].pot.span / SCALED_FLOAT;
			motordata[z].pot.scaled_actual = (int) ((float) (motordata[z].pot.pos_actual - motordata[z].pot.offset) * motordata[z].pot.scale_out);
			if (motordata[z].pot.scaled_actual > SCALED) motordata[z].pot.scaled_actual = SCALED;
			motordata[z].pot.pos_set = (int) (((float) motordata[z].pot.scaled_set * motordata[z].pot.scale_in) + motordata[z].pot.offset);
		}
		if (mode.emo) {
			term_time();
			sprintf(bootstr2, " EMO flag tripped, possible short circuit in the assy wiring.\r\n");
			puts2USART(bootstr2);
			term_time();
			sprintf(bootstr2, " EMO DUMP Current %li, %li, %li : Position %li, %li, %li\r\n", emodump.emo[0], emodump.emo[1], emodump.emo[2], emodump.emo[3], emodump.emo[4], emodump.emo[5]);
			puts2USART(bootstr2);
			voice2_ticks(40);
			while (TRUE) {
				emo_display();
				buzzer_ticks(200);
				ClrWdt(); // reset the WDT timer
			}
		}
	}

	ADC_zero(); // ground ADC input
	ClrWdt(); // reset the WDT timer
}

void zero_amploc(void) // set zero current setpoint from ADC reading from a10_x a10_z a10_y, write to EEPROM
{
	static uint8_t z;

	adc_cal[11] = adc_cal[12] = adc_cal[13] = adc_cal[14] = ADC_NULL; // reset offsets to zero (ADC_NULL)
	// read adc data for zero setpoint
	ADC_read(); // a10_x=adc_[11], a10_z=adc_cal[12], a10_y=adc_cal[13]
	adc_cal[11] = (uint8_t) (ADC_NULL + (int) ((int) a10_x - (int) AMP10_ZERO));
	adc_cal[12] = (uint8_t) (ADC_NULL + (int) ((int) a10_z - (int) AMP10_ZERO));
	adc_cal[13] = (uint8_t) (ADC_NULL + (int) ((int) a10_y - (int) AMP10_ZERO));

	if (checktime_eep(EEP_UPDATE, FALSE)) {
		term_time();
		putrs2USART(zero0);
		sprintf(bootstr2, "\n\r Zero offset %i, X current %i, AMP10_ZERO %i", (int) adc_cal[11], a10_x, AMP10_ZERO);
		puts2USART(bootstr2);
		sprintf(bootstr2, "\n\r Zero offset %i, Y current %i, AMP10_ZERO %i", (int) adc_cal[12], a10_z, AMP10_ZERO);
		puts2USART(bootstr2);
		sprintf(bootstr2, "\n\r Zero offset %i, Z current %i, AMP10_ZERO %i", (int) adc_cal[13], a10_y, AMP10_ZERO);
		puts2USART(bootstr2);
		write_data_eeprom(adc_cal[z], ADC_SLOTS, z, 8);
		checktime_eep(EEP_UPDATE, TRUE);
		putrs2USART(zero1);
	}
}
