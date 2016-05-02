/* High and Low ISR codes, some glue routines */
#include "mandm_vector.h"

#pragma tmpdata ISRHtmpdata
#pragma interrupt tick_handler nosave=section (".tmpdata")

void tick_handler(void) // This is the high priority ISR routine
{
	static uint8_t c1, c2, emo_bumps = 0, HID_IDLE_FLAG = TRUE, b_read = 0;
	static uint8_t ZAP = FALSE, vc_count = NULL0;
	static uint8_t worker_timer = WORKSEC, real_ticks = NULL0, fast_ticks = FT20;
	static int16_t rx_tmp = 0;
	static union Timers timer, timerl;
	static int32_t band_max = 0, hid_idle = 0;

	_asm nop _endasm // asm code to disable compiler optimizations
	V.highint_count++; // high int counter

	if (INTCON3bits.INT3IF) { // motor QEI input
		INTCON3bits.INT3IF = LOW;
		if (TMR4 > V.qei_counts) V.qei_counts = TMR4; // if timer4 is not 0 then update
		TMR4 = 0; // reset the timer to zero
		T4CONbits.TMR4ON = 1; // start the timer if stopped
		V.b3++;
		// not a very good position counter, relies on a 1/16 A channel hardware divider and a B channel logic level latch
		LATJ = LATJ | 0b00001111; // clear diag 0..3 leds on PORTJ
		DLED_3 = ON;
		qei1.ticks = 0;
		if (QEI1_B) {
			--qei1.c;
			qei1.cw = TRUE;
			DLED_1 = ON;
		} else {
			++qei1.c;
			qei1.cw = FALSE;
			DLED_0 = ON;
		}
		if (QHOME) qei1.home = TRUE;
		if (QBACK) qei1.back_stop = TRUE;
		if (QFORWARD) qei1.forward_stop = TRUE;
	}

	if (INTCONbits.RBIF) { // PORT B int handler for Qencoder inputs A/B
		INTCONbits.RBIF = LOW;
		b_read = EXTIO;
		V.buttonint_count++;
		D_UPDATE = TRUE;
		V.display_count = NULL0;
		LATJ = LATJ | 0b00001111; // clear diag 0..3 leds on PORTJ

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
					DLED_1 = ON;
				} else {
					--knob1.c;
					knob1.cw = FALSE;
					knob1.ccw = TRUE;
					knob1.movement = CCW;
					DLED_0 = ON;
				}
			} else {
				OldEncoder.A1 = ENC_A1;
				/*                         if (ENC_B1 == ENC_A1) {
							       --knob1.c;
							       knob1.cw = FALSE;
							       knob1.ccw = TRUE;
							       knob1.movement = CCW;
							   } else {
							       ++knob1.c;
							       knob1.cw = TRUE;
							       knob1.ccw = FALSE;
							       knob1.movement = CW;
							   }  */
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
					DLED_3 = ON;
				} else {
					--knob2.c;
					knob2.cw = FALSE;
					knob2.ccw = TRUE;
					knob2.movement = CCW;
					DLED_2 = ON;
				}
			} else {
				OldEncoder.A2 = ENC_A2;
				/*                  if (ENC_B2 == ENC_A2) {
						       --knob2.c;
						       knob2.cw = FALSE;
						       knob2.ccw = TRUE;
						       knob2.movement = CCW;
						   } else {
						       ++knob2.c;
						       knob2.cw = TRUE;
						       knob2.ccw = FALSE;
						       knob2.movement = CW;
						   } */
			}
		}
		HID_IDLE_FLAG = FALSE;
	}

	if (!mode.v24 && !mode.qei) DRIVE_V24 = LOW; //fixed // 24vdc off if flag not set

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

		emo_bumps = 0; // reset this every second to stop random EMO stops
		TIMERFLAG = TRUE;
		if (fast_ticks > 10) fast_ticks = real_ticks; // measure low ints per second
		real_ticks = NULL0; // reset low int counter

		MPULED = !MPULED; //  flash led
		V.timerint_count++; // set 1 second clock counter.

		V.display_count++; // display VC screen  display timer
		if (V.display_count > DISPSEC) { // turn off LCD after DISPSEC seconds
			//					D_UPDATE=FALSE;
		}

		if (DIPSW3 == HIGH) { // cycle LCD display
			DISPLAY_MODE = TRUE;
		} else {
			DISPLAY_MODE = FALSE;
			vc_count = NULL0;
		}

		/* cycled LCD display screens */
		if (worker_timer-- <= 1) { // check worker thread go flag
			WORKERFLAG = TRUE;
			worker_timer = WORKSEC;

			if (DISPLAY_MODE) {
				vc_count++;
				if (vc_count >= VC_MAX) vc_count = NULL0;
				switch (vc_count) {
				case VS0:
					dsi = VC0;
					break;
				case VS1:
					dsi = VC1;
					break;
				case VS2:
					dsi = VC2;
					break;
				default:
					dsi = VC0;
					break;
				}
			}
		}
		// check the LED blink flags
		if (V.blink & 0b00000001) LED_0 = !LED_0;
		if (V.blink & 0b00000010) LED_1 = !LED_1;
		if (V.blink & 0b00000100) LED_2 = !LED_2;
		if (V.blink & 0b00001000) LED_3 = !LED_3;
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

		if (c2 == '^') { // set AMPLOC sensor zero flag
			RESET_ZEROS = TRUE;
		}
		if (c2 == 'D') { // set debug logging flag
			SET_TLOG = TRUE;
		}
		if (c2 == 'T') { // Tweak charge run parameters
			TWEAK = TRUE;
		}
		if (c2 == '!') { // Hold monitor processing
			HOLD_PROC = TRUE;
		}
		if (c2 == '?') { // Show help screen
			SYS_HELP = TRUE;
		}
		if (c2 == '#') { // Display system data on rs-232 terminal
			SYS_DATA = TRUE;
		}

		if ((c2 == 'Y') || (c2 == 'y')) { // set key flag
			YNKEY = YES_M;
		}
		if ((c2 == 'N') || (c2 == 'n')) { // set key flag
			YNKEY = NO_M;
		}
		if ((c2 >= 0x30) && (c2 <= 0x39)) { // return number
			KEYNUM = c2;
		}
		if (c2 == 'K') { // lockup controller in loop force WDT timeout
			while (TRUE);
		}
		if (c2 == 'Z' || ZAP) { // clear command flags
			ZAP = FALSE;
		}
	}

	/* Control button routines */
	if (INTCONbits.INT0IF) {
		INTCONbits.INT0IF = LOW;
		V.b0++;
		if (SYSTEM_STABLE && (PB0 == 0u)) {
			DLED_0 = ON;
			button.B0 = 1;
		}
		HID_IDLE_FLAG = FALSE;
	}

	if (INTCON3bits.INT1IF) {
		INTCON3bits.INT1IF = LOW;
		V.b1++;
		if (SYSTEM_STABLE && (PB1 == 0u)) {
			DLED_1 = ON;
			button.B1 = 1;
		}
		HID_IDLE_FLAG = FALSE;
	}

	if (INTCON3bits.INT2IF) {
		INTCON3bits.INT2IF = LOW;
		V.b2++;
		if (SYSTEM_STABLE && (PB2 == 0u)) {
			DLED_2 = ON;
			ALARMOUT = LOW;
			button.B2 = 1;
			if (!mode.emo) {
				if (!mode.cal) {
					LED_1 = HIGH;
					mode.cal = TRUE;
				}
			} else {
				LED_1 = LOW;
				mode.cal = FALSE;
				ERELAYS = R_ALL_OFF;
			}
			if (emo_bumps++ > MAX_EMO) { // reset emo_bumps to zero every second to stop random EMO stops
				LED_3 = HIGH;
				LED_1 = LOW;
				mode.emo = TRUE;
				mode.cal = FALSE;
				ERELAYS = R_ALL_OFF;
				emo_bumps = 0;
			}
		}
		HID_IDLE_FLAG = FALSE;
	}

	// PWM4 ISR routines, ONLY FOR TESTING
	if (PIE3bits.TMR4IE && PIR3bits.TMR4IF) { // PWM4 post int
		PIR3bits.TMR4IF = LOW;
		T4CONbits.TMR4ON = 0; // timeout, stop timer
		TMR4 = 0;
		V.pwm4int_count++;
	}

	// moved from the low isr to here
	if (PIR2bits.TMR3IF) { //      Timer3 int handler
		PIR2bits.TMR3IF = LOW; // clear int flag
		timerl.lt = TIMER3REG; // Save the 16-bit value in local
		TMR3H = timerl.bt[HIGH]; // Write high byte to Timer3 High byte
		TMR3L = timerl.bt[LOW]; // Write low byte to Timer3 Low byte
		V.clock20++;
		real_ticks++;
		if (slow_timer > 0) {
			slow_timer--;
			if (slow_timer <= 0) {
				if (mode.qei) POWER_S = SLOW_STATUS; // restore original status.
			}
		}
		V.lowint_count++; // low int trigger entropy counter
		//FIXME watchout for int uint conversions that make negative number high
		// check for currrent overloads using the current sensors
		if (FALSE && (((int32_t) R.current_x > (int32_t) MAX_MOTOR_CURRENT_H) || ((int32_t) R.current_y > (int32_t) MAX_MOTOR_CURRENT_L) || ((int32_t) R.current_z > (int32_t) MAX_MOTOR_CURRENT_L))) { // over-current errors
			LED_3 = HIGH;
			LED_1 = LOW;
			mode.emo = TRUE;
			mode.cal = FALSE;
			ERELAYS = R_ALL_OFF;
			emo_bumps = MAX_EMO + 1;
			emodump.emo[0] = R.current_x;
			emodump.emo[1] = R.current_y;
			emodump.emo[2] = R.current_z;
			emodump.emo[3] = R.pos_x;
			emodump.emo[4] = R.pos_y;
			emodump.emo[5] = R.pos_z;
		}

		if (V.buzzertime == 0u) {
			ALARMOUT = LOW;
		} else {
			V.buzzertime--;
		}

		if (V.voice1time == 0u) {
			VOICE1_TRIS = HIGH; // back to input on vox button
		} else {
			V.voice1time--;
		}
		if (V.voice2time == 0u) {
			VOICE2_TRIS = HIGH; // back to input on vox button
		} else {
			V.voice2time--;
		}

		if (++qei1.ticks >= QEITICKS) { // check for stopped motor encoders
			qei1.movement = STOP;
			qei1.ticks = QEITICKS;
			qei1.last_c = qei1.c; // remember where we stopped at
			qei1.band = 0;
		} else {
			qei1.band = qei1.c - qei1.last_c; // relative movement
		}

		if (++knob1.ticks >= KNOBTICKS) { // check for stopped encoders
			knob1.movement = STOP;
			knob1.ticks = KNOBTICKS;
			knob1.last_c = knob1.c; // remember where we stopped at
			knob1.band = 0;
			DLED_4 = OFF;
		} else {
			knob1.band = knob1.c - knob1.last_c; // relative movement
			DLED_4 = ON;
		}

		if (++knob2.ticks >= KNOBTICKS) {
			knob2.movement = STOP;
			knob2.ticks = KNOBTICKS;
			knob2.last_c = knob2.c; // remember where we stopped at
			knob2.band = 0;
			DLED_5 = OFF;
		} else {
			if (knob2.band != (knob2.c - knob2.last_c)) { // move pot setting with constant knob movement
				band_max = knob2.band / POT_BAND_RATIO;
				if (band_max > POT_BAND_MAX_P) band_max = POT_BAND_MAX_P;
				if (band_max < POT_BAND_MAX_N) band_max = POT_BAND_MAX_N;
				motordata[knob_to_pot].pot.scaled_set += band_max;
			}
			knob2.band = knob2.c - knob2.last_c; // relative movement
			if (knob2.band > EMO_BAND) {
				mode.emo = FALSE;
				LED_3 = LOW;
			}
			DLED_5 = ON;
		}

		// constrain set limits
		if (motordata[knob_to_pot].pot.pos_set < 0) motordata[knob_to_pot].pot.pos_set = 0;
		if (motordata[knob_to_pot].pot.pos_set > ACTUAL) motordata[knob_to_pot].pot.pos_set = ACTUAL;
		if (motordata[knob_to_pot].pot.scaled_set < 0) motordata[knob_to_pot].pot.scaled_set = 0;
		if (motordata[knob_to_pot].pot.scaled_set > SCALED) motordata[knob_to_pot].pot.scaled_set = SCALED;
		if (!mode.qei) { // use ADC position feedback
			motordata[knob_to_pot].pot.error = motordata[knob_to_pot].pot.pos_set - motordata[knob_to_pot].pot.pos_actual;
		} else { // use digital encoder position feedback
			motordata[knob_to_pot].pot.error = knob2.c - qei1.c; // find the raw encoder/encoder diff
		}

		// slowing resistor logic
		if (!mode.slow_bypass) {
			if ((motordata[knob_to_pot].pot.error < TRACK_DB_H_S) && (motordata[knob_to_pot].pot.error > TRACK_DB_L_S)) {
				if (mode.v24) motordata[knob_to_pot].slow = TRUE;
			} else {
				motordata[knob_to_pot].slow = FALSE;
			}
			if (slow_timer == 0) {
				if ((mode.free || mode.cal || mode.on_off_only) && (!motordata[knob_to_pot].slow_only)) {
					POWER_S = R_ON; // free is fast with 5vdc drive or slow with higher voltage.
				} else {
					if (motordata[knob_to_pot].slow) {
						POWER_S = R_OFF;
					} else {
						if (!motordata[knob_to_pot].slow_only) POWER_S = R_ON;
					}
				}
			}
		}
		// House keeping fuctions
		// mode.idle FLAG updates
		if (HID_IDLE_FLAG) {
			if (hid_idle++ >= MAX_IDLE) {
				mode.idle = TRUE; // after MAX_IDLE counts without movement we are idle
				DLED_7 = LOW;
				hid_idle = MAX_IDLE;
			}
		}
		if (!HID_IDLE_FLAG) {
			HID_IDLE_FLAG = TRUE;
			mode.idle = FALSE; // something moved so we are not idle
			DLED_7 = HIGH;
			hid_idle = 0;
		}
		// Alarm checks
		if (almctr > MAXALM - 2) almctr = MAXALM - 2; // check for alarm message overrun.
		// WORKSEC time events
		if (WORKERFLAG && SYSTEM_STABLE) { // WORKSEC time,
			WORKERFLAG = FALSE;
			V.worker_count++;
		} // end of worksec functions
	}
}
#pragma	tmpdata

#pragma interruptlow work_handler

void work_handler(void) // This is the low priority ISR routine, the high ISR routine will be called during this code section
{
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
	ALARMOUT = LOW;
	VOICE1_TRIS = HIGH;
	VOICE2_TRIS = HIGH;
	mode.cal = FALSE;
}

void buzzer_ticks(uint8_t length)
{
	ALARMOUT = HIGH;
	V.buzzertime = length;
}

void voice1_ticks(uint8_t length)
{
	VOICE1_TRIS = LOW; // send low (button press) to vox board
	VOICE1 = LOW;
	V.voice1time = length;
}

void voice2_ticks(uint8_t length)
{
	VOICE2_TRIS = LOW; // send low (button press) to vox board
	VOICE2 = LOW;
	V.voice2time = length;
}

void slow_timer_start(void)
{
	slow_timer = 10;
}