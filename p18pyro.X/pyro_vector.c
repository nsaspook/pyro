/* High and Low ISR codes, some glue routines */
#include "pyro_vector.h"

#pragma tmpdata ISRHtmpdata
#pragma interrupt tick_handler nosave=section (".tmpdata")

void tick_handler(void) // This is the high priority ISR routine
{
	static uint8_t c1, c2, emo_bumps = 0, HID_IDLE_FLAG = TRUE, b_read = 0;
	static uint8_t ZAP = FALSE;
	static uint8_t worker_timer = WORKSEC, fast_ticks = FT20;
	static int16_t rx_tmp = 0;
	static union Timers timer;

	_asm nop _endasm // asm code to disable compiler optimizations
	V.highint_count++; // high int counter

	/*
	 * ~1khz state machine sequencer timer
	 */
	if (PIE3bits.TMR4IE && PIR3bits.TMR4IF) {
		PIR3bits.TMR4IF = LOW;
		PR4 = 0x11;
		V.pwm4int_count++;
		DLED_0 = !DLED_0;
	}

	if (PIE1bits.SSP1IE && PIR1bits.SSP1IF) { // send data to SPI bus 1
		spi_link.int_count++;
		PIR1bits.SSP1IF = LOW;
	}

	if (PIE3bits.SSP2IE && PIR3bits.SSP2IF) { // send data to SPI bus 2
		spi_link.int_count++;
		PIR3bits.SSP2IF = LOW;
	}

	if (INTCON3bits.INT3IF) { // motor QEI input
		INTCON3bits.INT3IF = LOW;
		V.b3++;
	}

	if (INTCONbits.RBIF) { // PORT B int handler for Qencoder inputs A/B
		INTCONbits.RBIF = LOW;
		b_read = EXTIO;
		V.buttonint_count++;
		D_UPDATE = TRUE;

		// modified version from www.piclist.com qenc-dk.htm
		if (OldEncoder.OldPortB[0] != (OldEncoder.byTemp[0] = (b_read & QENC1BITS))) { // read encoder 1
			knob1.ticks = 0;
			OldEncoder.OldPortB[0] = OldEncoder.byTemp[0];
			if (ENC_A1 == OldEncoder.A1) {
				if (ENC_B1 == ENC_A1) {
					++knob1.c;
					knob1.cw = TRUE;
					knob1.ccw = FALSE;
					knob1.movement = CW;
				} else {
					--knob1.c;
					knob1.cw = FALSE;
					knob1.ccw = TRUE;
					knob1.movement = CCW;
				}
			} else {
				OldEncoder.A1 = ENC_A1;
			}
		}

		if (OldEncoder.OldPortB[1] != (OldEncoder.byTemp[1] = (b_read & QENC2BITS))) { // read encoder 2
			knob2.ticks = 0;
			OldEncoder.OldPortB[1] = OldEncoder.byTemp[1];
			if (ENC_A2 == OldEncoder.A2) {
				if (ENC_B2 == ENC_A2) {
					++knob2.c;
					knob2.cw = TRUE;
					knob2.ccw = FALSE;
					knob2.movement = CW;
				} else {
					--knob2.c;
					knob2.cw = FALSE;
					knob2.ccw = TRUE;
					knob2.movement = CCW;
				}
			} else {
				OldEncoder.A2 = ENC_A2;
			}
		}
		HID_IDLE_FLAG = FALSE;
	}

	if (PIR2bits.EEIF) { // EEPROM write complete flag
		V.eeprom_count++; // just keep count
		MPULED = !MPULED; //  flash led
		PIR2bits.EEIF = LOW;
	}

	if (PIE1bits.ADIE && PIR1bits.ADIF) { // ADC conversion complete flag
		V.adc_count++; // just keep count    							// set output bit to zero
		PIR1bits.ADIF = LOW;
	}

	if (INTCONbits.TMR0IF) { // check timer0 irq 1 second timer int handler
		INTCONbits.TMR0IF = LOW; //clear interrupt flag
		//check for TMR0 overflow

		timer.lt = TIMEROFFSET; // Copy timer value into union
		TMR0H = timer.bt[HIGH]; // Write high byte to Timer0
		TMR0L = timer.bt[LOW]; // Write low byte to Timer0

		TIMERFLAG = TRUE;

		MPULED = !MPULED; //  flash led
		V.timerint_count++; // set 1 second clock counter.
	}

	// Serial port communications to the host and terminal
	//

	/* Get the character received from the USART */
	if (PIR1bits.RC1IF) { // is data from network  port
		V.c1rx_int++; // total count
		rx_tmp++; // count for 1 second
		if (RCSTA1bits.OERR) {
			RCSTA1bits.CREN = LOW; // clear overrun
			RCSTA1bits.CREN = HIGH; // re-enable
		}

		/* clear com1 interrupt flag */
		// a read clears the flag
		c1 = RCREG1; // read data from port1 and clear PIR1bits.RC1IF
	}

	/* User terminal comm routines */
	if (PIR3bits.RC2IF) { // is data from user command/dump terminal port
		/* clear com2 interrupt flag */
		// a read clears the flag
		V.c2_int++;
		if (RCSTA2bits.OERR) {
			RCSTA2bits.CREN = LOW; //      clear overrun
			RCSTA2bits.CREN = HIGH; // re-enable
		}

		c2 = RCREG2; // read from host port2 and clear PIR3bits.RC2IF
		C2RAW = c2; // set terminal input char.
	}

	/* Control button routines */
	if (INTCONbits.INT0IF) {
		INTCONbits.INT0IF = LOW;
		V.b0++;
		if (SYSTEM_STABLE && (PB0 == 0u)) {
			button.B0 = 1;
		}
		HID_IDLE_FLAG = FALSE;
	}

	if (INTCON3bits.INT1IF) {
		INTCON3bits.INT1IF = LOW;
		V.b1++;
		if (SYSTEM_STABLE && (PB1 == 0u)) {
			button.B1 = 1;
		}
		HID_IDLE_FLAG = FALSE;
	}

	if (INTCON3bits.INT2IF) {
		INTCON3bits.INT2IF = LOW;
		V.b2++;
		if (SYSTEM_STABLE && (PB2 == 0u)) {
			button.B2 = 1;
		}
		HID_IDLE_FLAG = FALSE;
	}

}
#pragma	tmpdata

#pragma interruptlow work_handler

void work_handler(void) // This is the low priority ISR routine, the high ISR routine will be called during this code section
{ // projector lamp scan converter
	static union Timers timerl;

	if (PIR2bits.TMR3IF) { //      Timer3 int handler
		PIR2bits.TMR3IF = LOW; // clear int flag
		timerl.lt = TIMER3REG; // Save the 16-bit value in local
		TMR3H = timerl.bt[HIGH]; // Write high byte to Timer3 High byte
		TMR3L = timerl.bt[LOW]; // Write low byte to Timer3 Low byte
		V.clock20++;
		DLED_1 = !DLED_1;
		V.lowint_count++; // low int trigger entropy counter
	}

}

void idle_loop(void) // idle processe to allow for better isr triggers and background networking
{
	IDLEFLAG = TRUE;
	ClrWdt();
	IDLEFLAG = FALSE;
}

void P1wait(void)
{
	while (!TXSTA1bits.TRMT) {
	};
}

void P2wait(void)
{
	while (!TXSTA2bits.TRMT) {
	};
}

void Clear_All_Buttons(void)
{
	button.B0 = 0;
	button.B1 = 0;
	button.B2 = 0;
}
