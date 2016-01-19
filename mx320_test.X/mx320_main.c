
// PIC32MX320F128H Configuration Bit Settings

#include <p32xxxx.h>

// DEVCFG3
// USERID = No Setting

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 1)

// DEVCFG1
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = ON             // Secondary Oscillator Enable (Enabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = XT             // Primary Oscillator Configuration (XT osc mode)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_8           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1024           // Watchdog Timer Postscaler (1:1024)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))

// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (ICE EMUC2/EMUD2 pins shared with PGC2/PGD2)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)


/* 
 * File:   mx320_main.c
 * Author: root
 *
 * Created on December 23, 2013, 3:22 PM
 */

#include <plib.h>
#include <stdio.h>
#include <stdlib.h>

volatile int a, b;

/*
 * 
 */
int main(int argc, char** argv)
{
	unsigned long int i, j;
	unsigned char cylon = 0xff;
	long alive_led = 0;
	unsigned char LED_UP = TRUE;

	PORTSetPinsDigitalOut(IOPORT_F, BIT_0); // spare digital output
	mPORTFSetBits(BIT_0); //
	PORTSetPinsDigitalOut(IOPORT_G, BIT_7 | BIT_8); // spare digital output
	mPORTGSetBits(BIT_7 | BIT_8); //
	PORTSetPinsDigitalOut(IOPORT_E, 0xFF); // spare digital output
	mPORTESetBits(0xFF); //

	while (1) {
		for (i = 0; i <= 30000; i++) {
			a = b;
		}

		if (j++ >= 1) { // delay a bit ok
			if (0) { // screen status feedback
				PORTE = ~cylon; // roll leds cylon style
			} else {
				PORTE = cylon; // roll leds cylon style (inverted)
			}

			if (LED_UP && (alive_led != 0)) {
				alive_led = alive_led * 2;
				cylon = cylon << 1;
			} else {
				if (alive_led != 0) alive_led = alive_led / 2;
				cylon = cylon >> 1;
			}
			if (alive_led < 2) {
				alive_led = 2;
				LED_UP = TRUE;
				mPORTFToggleBits(BIT_0);
				mPORTGToggleBits(BIT_7 | BIT_8);
			} else {
				if (alive_led > 128) {
					alive_led = 128;
					LED_UP = FALSE;
					mPORTFToggleBits(BIT_0);
					mPORTGToggleBits(BIT_7 | BIT_8);
				}
			}
			j = 0;
		}
	}

	return(EXIT_SUCCESS);
}

