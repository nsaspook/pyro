/* 
 * File:   pyro_defs.h
 * Author: root
 *
 * Created on May 2, 2016, 3:36 PM
 */

#ifndef PYRO_DEFS_H
#define	PYRO_DEFS_H

#ifdef	__cplusplus
extern "C" {
#endif

#define	PIC_8722		8722
#define CHECKMARK		0x59		// EEPROM checkmark
#define CHECKMARK_CRC	0x59595959ul		// 32 bit crc checkmark

#define MESG_W          80			// message string buffer
#define CRIT_8          8               // 8 levels
#define CRITC1          1               // The first CRITC level
#define MAX_MFC         3u               // number of MFC that will control
#if defined(__18CXX)
#define NULL0           (uint8_t)0
#endif
#define	Hpri		1
#define	Lpri		2
#define	HL		3
#define	TIMEROFFSET	26474           // timer0 16bit counter value for 1 second to overflow
#define	TIMER3REG	3048            // timer3 value for 20Hz clock, orig value 3048
#define TIMER4_NORM	0x07		// 0x11 for slower SPS
#define TIMER4_FAST	0x07
#define LCD_SLOW	8		// timer ticks for slow commands
#define	TIMEOFF		60      	// seconds in 1 min
#define	WORKSEC		10      	// run every this time seconds
#define FT20		20              // default value for fast ticks
#define SD_18		18
#define WORK_TICKS_S	20*60

#define LOW		(uint8_t)0        // digital output state levels, sink
#define	HIGH            (uint8_t)1        // digital output state levels, source
#define	ON		LOW       		//
#define OFF		HIGH			//
#define	S_ON            LOW       		// low select/on for chip/led
#define S_OFF           HIGH			// high deselect/off chip/led
#define	R_ON            HIGH       		// control relay states, relay is on when output gate is high, uln2803,omron relays need the CPU at 5.5vdc to drive 
#define R_OFF           LOW			// control relay states
#define R_ALL_OFF       0x00
#define R_ALL_ON	0xff
#define NO		LOW
#define YES		HIGH
#define LOW_VECTOR      0x18                // ISR low address
#define HIGH_VECTOR     0x8                 // ISR high address

	/* spi select pins */
#define DAC_0_CS	LATCbits.LATC0
#define DAC_1_CS	LATCbits.LATC1
#define SHF_2_CS	LATCbits.LATC2
#define SHF_3_CS	LATCbits.LATC6
#define SPI_LOAD	LATCbits.LATC7

#define DIPSW		PORTD
#define	PURGE_PSW	PORTDbits.RD0
#define	DIPSW2		PORTDbits.RD1
#define	DIPSW3		PORTDbits.RD2
#define	DIPSW4		PORTDbits.RD3
#define	DIPSW5		PORTDbits.RD4
#define	DIPSW6		PORTDbits.RD5
#define	DIPSW7		PORTDbits.RD6
#define	DIPSW8		PORTDbits.RD7

#define	DLED_0		LATJbits.LATJ0
#define	DLED_1		LATJbits.LATJ1
#define DLED_2 		LATJbits.LATJ2
#define DLED_3 		LATJbits.LATJ3 
#define DLED_4		LATJbits.LATJ4 
#define DLED_5		LATJbits.LATJ5
#define DLED_6		LATJbits.LATJ6
#define DLED_7 		LATJbits.LATJ7

#define ERELAYS		LATE                // HIGH is on
#define POWER_X		LATEbits.LATE0
#define POWER_Y		LATEbits.LATE1
#define POWER_Z		LATEbits.LATE2
#define POWER_S		LATEbits.LATE3
#define	DIR_X		LATEbits.LATE4
#define	DIR_Y		LATEbits.LATE5
#define	DIR_Z		LATEbits.LATE6
#define	DRIVE_V24	LATEbits.LATE7

#define MPULED		LATHbits.LATH0
#define VOICE1     	LATGbits.LATG3
#define VOICE1_TRIS	TRISGbits.TRISG3
#define VOICE2     	LATGbits.LATG4
#define VOICE2_TRIS	TRISGbits.TRISG4
#define ALARMOUT	LATGbits.LATG0

#define PB0        	PORTBbits.RB0	// switch1
#define PB1        	PORTBbits.RB1	// switch2
#define PB2        	PORTBbits.RB2	// switch3
#define QEI1_A		PORTBbits.RB3	// motor QEI A channel input
#define	ENC_A1		PORTBbits.RB4	// quad encoder input 1 a,b
#define	ENC_B1		PORTBbits.RB5
#define	ENC_A2		PORTBbits.RB6	// quad encoder input 2 a,b
#define	ENC_B2		PORTBbits.RB7

#define QFORWARD        PORTFbits.RF4
#define FINPUT_4        PORTFbits.RF4
#define QHOME           PORTFbits.RF5
#define FINPUT_5        PORTFbits.RF5
#define QBACK           PORTFbits.RF6
#define FINPUT_6        PORTFbits.RF6
#define QEI1_B        	PORTFbits.RF7       // channel B from the QEI encoder
#define FINPUT_7        PORTFbits.RF7

#define QENC1BITS	0b00110000	// PORTB 4..5
#define QENC2BITS	0b11000000	// PORTB 6..7

	/*	ADC gain offsets	mV per bit step defines */
#define ADC_5V_MV 	488UL             // 5vdc 
#define ADC_24V_MV 	2350UL            // 24vdc
#define	ADC_SLOTS	15		// 14 ADC data slots + 1 checksum at the end
#define ADC_INDEX	8		// max adc array select number
#define ADC_NULL	127             // zero offset value
#define ADC_MASK	0x03ff		// 10 bits of real adc data
#define ADC_CHAN_MASK	0x07		// number os scanned adc channels
	/* The number of samples must be high to sample several complete PWM cycles to get stable voltage and current measurements at lower ranges. */
#define ADC_SAMP_F	16
#define ADC_SAMP_S	64
#define ADC_CHAN_DELAY	100		// delay in 10X chip cycles after switching ADC channels

#define	LPCHANC		8               // digital filter channels
#define MAXSECONDS      315576000ul     // 10 years
#define	LL1		0x00                    // LCD line addresses
#define	LL2		0x40
#define LL3		0x14
#define	LL4		0x54

#define AIR_MFC		0
#define GAS_MFC		1
#define COLOR1_MFC	2
#define COLOR2_MFC	3


#define BANK0		0
#define BANK1		1

#define	PURGE		0
#define AIR		1
#define GAS		2 
#define COLOR1		3
#define COLOR2		4 
#define V5		5 
#define V6		6 
#define V7		7


#define MFC_INTEG	1200UL // sample updates per Min
#define MFC_VOLTS	5000.0 // millivolts

#define SHIFT_565_0_7	2
#define SHIFT_565_8_15	3	

#ifdef	__cplusplus
}
#endif

#endif	/* PYRO_DEFS_H */

