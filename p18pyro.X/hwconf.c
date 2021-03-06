#include "hwconf.h"

void config_pic(uint16_t hw_config)
{
	static char z;
	if (hw_config == 8722u) {
		_asm nop _endasm // asm code to disable compiler optimizations
		if (RCONbits.TO == (uint8_t) LOW) WDT_TO = TRUE;
		if (EECON1bits.WRERR && (EECON1bits.EEPGD == (uint8_t) LOW)) EEP_ER = TRUE;
		/* Configure all PORT  pins  */

		TRISAbits.TRISA4 = LOW; // output
		TRISAbits.TRISA0 = HIGH; // an0 mfc0 R
		TRISAbits.TRISA1 = HIGH; // an1 mfc1 R
		TRISAbits.TRISA2 = HIGH; // an2 mfc0 S
		TRISAbits.TRISA3 = HIGH; // an3 mfc1 S
		TRISAbits.TRISA5 = HIGH; // an4 

		TRISB = 0x00; // all outputs first
		LATB = 0xFF;
		TRISBbits.TRISB0 = HIGH; // switch 1
		TRISBbits.TRISB1 = HIGH; // 2
		TRISBbits.TRISB2 = HIGH; // 3
		TRISBbits.TRISB3 = HIGH; // motor QEI1 channel A
		TRISBbits.TRISB4 = HIGH; // quad1 a
		TRISBbits.TRISB5 = HIGH; // quad1 b
		TRISBbits.TRISB6 = HIGH; // quad2 a
		TRISBbits.TRISB7 = HIGH; // quad2 b
		TRISC = 0x00; // SPI outputs
		LATC = 0xff; // all devices disabled/HIGH

		TRISD = 0xff; // dip switch inputs

		TRISE = LOW; // motor relay outputs
		LATE = R_ALL_ON;

		TRISF = 0xff; // all inputs for ADC inputs

		LATGbits.LATG0 = HIGH; // output latch to DIAG jumper signal
		LATGbits.LATG3 = LOW; // output latch to zero Voice1 signal
		LATGbits.LATG4 = LOW; // output latch to zero Voice2 signal
		TRISGbits.TRISG4 = HIGH; //
		TRISGbits.TRISG3 = HIGH; // set to input for OFF and output for ON
		TRISGbits.TRISG0 = LOW; // output 
		TRISH = LOW; // mpuled and LCD
		LATH = 0xff;

		TRISJ = 0; // internal diag LEDS
		LATJ = 0xff;

		/* SPI pins setup */
		TRISCbits.TRISC3 = LOW; // SCK 
		TRISCbits.TRISC4 = HIGH; // SDI
		TRISCbits.TRISC5 = LOW; // SDO

		OpenSPI1(SPI_FOSC_4, MODE_00, SMPEND); // 1MHz
		SSP1CON1 |= SPI_FOSC_4;

		/* clear SPI module possible flags */
		PIE1bits.SSP1IE = HIGH;
		IPR1bits.SSP1IP = HIGH;
		PIR1bits.SSP1IF = LOW;

		/*
		 * Open the USARTs configured as
		 * 8N1, 38400,38400 baud, in send and receive INT mode
		 */
		BAUDCON2 |= 0x08; // 16 bit mode speed register

		Open2USART(USART_TX_INT_OFF & USART_RX_INT_ON & USART_ASYNCH_MODE & USART_EIGHT_BIT & USART_CONT_RX & USART_BRGH_LOW, 64); // 40mhz osc HS         38.4 kbaud, 16bit divider
		SPBRGH2 = 0x00;
		SPBRG2 = 0x40;

		while (DataRdy2USART()) { // dump 2 rx data
			z = Read2USART();
		};
		RCSTA1bits.SPEN = 0; // disable port 1

		OpenADC(ADC_FOSC_RC & ADC_RIGHT_JUST & ADC_20_TAD, ADC_CH0 & ADC_REF_VDD_VSS & ADC_INT_ON, ADC_11ANA); // open ADC channels for current and voltage readings
		ADCON1 = 0x03; // adc [0..11] enable, the OpenADC ADC_11ANA sets this also
		PIE1bits.ADIE = HIGH; // the ADC interrupt enable bit
		IPR1bits.ADIP = HIGH; // ADC use high pri

		OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_256);
		OpenTimer3(TIMER_INT_ON & T1_16BIT_RW & T1_SOURCE_INT & T1_PS_1_8 &
			T1_OSC1EN_OFF & T1_SYNC_EXT_OFF); // for lamp scanner
		V.t4_match = TIMER4_NORM;
		PR4 = V.t4_match;
		OpenTimer4(TIMER_INT_ON & T4_PS_1_16 & T4_POST_1_16); // state machine timer
		IPR3bits.TMR4IP = HIGH; // make it high pri level
		PIE3bits.TMR4IE = HIGH; // TIMER4 int enable bit

		INTCONbits.RBIE = HIGH; // enable PORTB interrupts 1=enable
		INTCON2bits.RBIP = HIGH; // Set the PORTB interrupt-on-change as a high priority interrupt

		INTCONbits.INT0IE = HIGH; //
		INTCON2bits.INTEDG0 = LOW; // falling edge
		INTCONbits.INT0IF = LOW; // clean possible flag

		INTCON3bits.INT1IE = HIGH; //
		INTCON2bits.INTEDG1 = LOW; // falling edge
		INTCON3bits.INT1IF = LOW; // clean possible flag

		INTCON3bits.INT2IE = HIGH; //
		INTCON2bits.INTEDG2 = LOW; // falling edge
		INTCON3bits.INT2IF = LOW; // clean possible flag

		INTCON3bits.INT3IE = HIGH; //
		INTCON2bits.INTEDG3 = LOW; // falling edge
		INTCON3bits.INT3IF = LOW; // clean possible flag

		z = PORTB; // dummy read to clear possible b irq
		INTCONbits.RBIF = LOW; // reset B flag
		INTCON2bits.RBPU = LOW; // enable input pullup. 0=on

		WriteTimer0(TIMEROFFSET); // start timer0 at 1 second ticks
		WriteTimer3(TIMER3REG); // worker thread timer about 20hz

		/*      work int thread setup */
		IPR2bits.TMR3IP = LOW; // make it low pri level
		PIE2bits.TMR3IE = HIGH; // enable int

		/* EEPROM write int enable */
		PIE2bits.EEIE = HIGH;

		/* Enable interrupt priority */
		RCONbits.IPEN = HIGH;
		PIR1 = LOW; // clear int flags
		PIR2 = LOW;
		PIR3 = LOW;

		/* Make receive interrupt high priority */
		IPR3bits.RC2IP = HIGH;

		/* Enable all high/low priority interrupts */
		INTCONbits.GIEH = HIGH;
		INTCONbits.GIEL = HIGH;
		DLED_1 = LOW;
	}
}

void start_pic(uint16_t hw_config)
{
	if (hw_config == 8722u) {
		IORELAYS = 0xff; // set control relays to off at powerup/reset
	}
}

void start_workerthread(void)
{
	//	T2CONbits.TMR2ON = HIGH;
}