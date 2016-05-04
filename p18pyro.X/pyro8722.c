
// PIC18F8722 Configuration Bit Settings

#include <p18f8722.h>

// CONFIG1H
#pragma config OSC = HSPLL      // Oscillator Selection bits (HS oscillator, PLL enabled (Clock Frequency = 4 x FOSC1))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Two-Speed Start-up disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits 
#pragma config BORV = 3         // Brown-out Voltage bits (Max setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer (WDT enabled)
#pragma config WDTPS = 1024     // Watchdog Timer Postscale Select bits (1:1024)

// CONFIG3L
#pragma config MODE = MC        // Processor Data Memory Mode Select bits (Microcontroller mode)
#pragma config ADDRBW = ADDR20BIT// Address Bus Width Select bits (20-bit Address Bus)
#pragma config DATABW = DATA16BIT// Data Bus Width Select bit (16-bit External Bus mode)
#pragma config WAIT = OFF       // External Bus Data Wait Enable bit (Wait selections are unavailable for table reads and table writes)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (ECCP2 input/output is multiplexed with RC1)
#pragma config ECCPMX = PORTE   // ECCP MUX bit (ECCP1/3 (P1B/P1C/P3B/P3C) are multiplexed onto RE6, RE5, RE4 and RE3 respectively)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RG5 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config BBSIZ = BB2K     // Boot Block Size Select bits (1K word (2 Kbytes) Boot Block size)
#pragma config XINST = ON      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit Block 0 (Block 0 (000800, 001000 or 002000-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit Block 1 (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit Block 2 (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit Block 3 (Block 3 (00C000-00FFFFh) not code-protected)
#pragma config CP4 = OFF        // Code Protection bit Block 4 (Block 4 (010000-013FFFh) not code-protected)
#pragma config CP5 = OFF        // Code Protection bit Block 5 (Block 5 (014000-017FFFh) not code-protected)
#pragma config CP6 = OFF        // Code Protection bit Block 6 (Block 6 (01BFFF-018000h) not code-protected)
#pragma config CP7 = OFF        // Code Protection bit Block 7 (Block 7 (01C000-01FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot Block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit Block 0 (Block 0 (000800, 001000 or 002000-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit Block 1 (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit Block 2 (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit Block 3 (Block 3 (00C000-00FFFFh) not write-protected)
#pragma config WRT4 = OFF       // Write Protection bit Block 4 (Block 4 (010000-013FFFh) not write-protected)
#pragma config WRT5 = OFF       // Write Protection bit Block 5 (Block 5 (014000-017FFFh) not write-protected)
#pragma config WRT6 = OFF       // Write Protection bit Block 6 (Block 6 (01BFFF-018000h) not write-protected)
#pragma config WRT7 = OFF       // Write Protection bit Block 7 (Block 7 (01C000-01FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-007FFF, 000FFF or 001FFFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit Block 0 (Block 0 (000800, 001000 or 002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR4 = OFF      // Table Read Protection bit Block 4 (Block 4 (010000-013FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR5 = OFF      // Table Read Protection bit Block 5 (Block 5 (014000-017FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR6 = OFF      // Table Read Protection bit Block 6 (Block 6 (018000-01BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR7 = OFF      // Table Read Protection bit Block 7 (Block 7 (01C000-01FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-007FFF, 000FFF or 001FFFh) not protected from table reads executed in other blocks)

#define	__MANDM_C			//	This is the main program

/*
 *
 * standard program units:
 * Voltage  in (uint32_t/uint16_t) millivolts,
 * Current in (int32_t) hundredths of amps
 * Watts Power in (uint32_t)
 * 
 *
 * R: structure, real values from measurements
 * C: structure, calculated values from measurements or programs
 * B: structure
 * V: structure, Volatile varables modified in the ISR in a possible non-atomic fashion
 *
 * USART1		Data Link channel 38400
 * USART2 		is the host comm port 38400
 * Timer0		1 second clock
 * TImer1		Not used
 * Timer2		Not used
 * Timer3		work thread , background I/O clock ~20HZ
 * TImer4		State machine Period clock ~1khz

 * 0..8 analog channels are active
 * PORTA		analog inputs
 * adc0	systemvoltage	PIC Controller 5vdc supply voltage
 * adc1	motorvoltage	24vdv PS monitor from relay
 * PORTB		HID Qencoder and switch inputs
 * PORTC		HID leds
 * PORTD		configuration switch input
 * PORTE		motor control relays
 * PORTF		analog inputs

 * adc8 Ground REF	zero adc charge cap RF3
 * adc_cal[11-14]	current sensors zero offset stored in eeprom 11=x, 12=y, 13=z, 14=future
 * cal table with checksum as last data item in adc_cal[]
 * PORTH0		run flasher led onboard, 4x20 LCD status panel
 * PORTJ		alarm and diag leds
 * PORTG		Alarm and Voice outputs
 *
 *
 */


/*
 *
 * This application is designed for use with the
 * ET-BASE PIC8722 board, 4*20 LCD display
 *
 * RS-232 host commands 38400baud 8n1

 * K		lockup controller causing WDT timer to reboot
 * Z		reset command parser
 * ?		Send help menu to rs-232 terminal
 * $		fuse blown error. (NOT DONE YET)
 * ^		reset current sensor zero calibration
 * !		hold program in present date
 * #		System Status
 *
 */


//	***
//	0.5     mandm software.
//	1.5     Find source of LCD glitching, might be ISR related.
//	1.8     First production release
//	1.9     Change relay drive from low on to HIGH on for uln2803 buffers. OMRON relays also rewired.
//	2.0     On V2 hardware with DPDT relays add shorting motor function for brakes and EMI reduction
//	2.1     Motor QEI isr code and inputs. Use B3 input for A input isr trigger and QEI1_B for B input, use cable codes 8+
//              buzzer and voice box functions to G0 and G3
//	2.5     Bug fixes
//	3.0     Fix bug in free movement CCW drive
//      3.1     Change to use extern VREF for ADC, move input from adc3 to adc11, connect 5vdc vref TI REF02AP chip to input adc3
//		Use BOREN to reset cpu when using the external VREF
//      3.2	adjust timeouts for slower motors (RMS)
//      3.3     Mostly bug fixes and motor preset moves after calibration.

//	***
//  dipswitch settings PORTD
//  1       Clear controller data
//  2       on=RS-232 logging DEBUG mode,// log debug text to comm2 port at 38400
//  3       on=FAIL-SAFE MODE, just turn on all motors.	    Cycle LCD display to debug screens.
//  4
//  5       0 selection bit
//  6       1
//  7       2
//  8       3
//	***

#include <p18cxxx.h>
#include "xlcd.h"
#include <delays.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "pyro_defs.h"
#include "pyro.h"
#include "crit.h"
#include "pyro_msg.h"
#include "hwconf.h"
#include "pyro_vector.h"
#include "pyro_shared.h"
#include "daq.h"
#include "ringbufs.h"

uint8_t spinners(uint8_t, uint8_t);

#pragma udata gpr13
far int8_t bootstr2[MESG_W + 1];


#pragma udata gpr1

volatile struct ringBufS_t ring_buf3, ring_buf4;
uint8_t HCRIT[CRIT_8], LCRIT[CRIT_8];
float smooth[LPCHANC];
#pragma udata gpr2
volatile struct L_data L;
volatile struct spi_link_type spi_link;

volatile struct ringBufS_t ring_buf5, ring_buf6;
#pragma udata gpr3

int16_t a10_x, a10_y, a10_z, worktick;
volatile uint8_t critc_level = 0, KEYNUM = 0, C2RAW, glitch_count, cdelay, SLOW_STATUS;
volatile uint8_t TIMERFLAG = FALSE, PRIPOWEROK = TRUE, FORCEOUT = FALSE, WORKERFLAG = FALSE,
	FAILSAFE = FALSE, SYSTEM_STABLE = FALSE, HOLD_PROC = FALSE,
	DISPLAY_MODE = FALSE, D_UPDATE = TRUE, GLITCH_CHECK = TRUE, COOLING = FALSE,
	UPDATE_EEP = FALSE, RESET_ZEROS = FALSE, SYS_DATA = FALSE, MOD_DATA = FALSE, SYS_HELP = FALSE, SET_TLOG = FALSE,
	WDT_TO = FALSE, EEP_ER = FALSE, TWEAK = FALSE, TEST_SPINNERS = FALSE;
#pragma udata gpr4
volatile uint8_t almctr, RS232_DEBUG = FALSE;
volatile uint32_t critc_count = 0;
volatile struct modetype mode;
#pragma udata gpr5

int8_t sign = ' ';
float lp_speed = 0.0, lp_x = 0.0;
struct R_data R;
struct C_data C;
struct V_data V;
const rom int8_t *build_date = __DATE__, *build_time = __TIME__;
#pragma udata gpr6

volatile uint8_t dsi = 0, msi = 0, help_pos = 0; //      LCD display string index to console 0
float t1 = 0.0, t2 = 0.0, t3 = 0.0, t4 = 0.0, t5 = 0.0, t6 = 0.0, t7 = 0.0, t_time = 0.0;
float voltfrak = 0.0;
float ahfrak = 0.0;
#pragma udata gpr7
uint32_t Vin = 0, chrg_v = 0, vbatol_t = 0, solar_t = 0, rawp[MAX_MFC], rawa[MAX_MFC];
volatile uint8_t IDLEFLAG = FALSE, knob_to_pot = 0;
int32_t iw = 0, ip = 0;
#pragma udata gpr8
volatile struct ringBufS_t ring_buf1, ring_buf2;
#pragma udata gpr9
volatile struct knobtype knob1, knob2;

/* ADC voltage/current default calibration values , adjusted with D command */
// adc_cal[11-14]				current sensors zero offset stored in eeprom 11=x, 12=y, 13=z, 14=future
uint8_t adc_cal[] = {127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 0};
uint8_t CRITC = 0, LCD_OK = FALSE, cable_type = 0x07, TLOG = FALSE;

volatile enum answer_t {
	WAIT_M, YES_M, NO_M
} YNKEY;

volatile struct QuadEncoderType OldEncoder;

#pragma idata gpr10
uint8_t lcd18 = 200;
volatile struct qeitype qei1;
volatile int32_t slow_timer = 0;
volatile struct buttontype button;

/* ISR vectors */
#pragma code tick_interrupt = HIGH_VECTOR

void tick_int(void)
{
	_asm goto tick_handler _endasm // high
}
#pragma code

#pragma code work_interrupt = LOW_VECTOR

void work_int(void)
{
	_asm goto work_handler _endasm // low
}
#pragma code

void DelayFor18TCY(void)
{
	static uint8_t n;
	_asm nop _endasm // asm code to disable compiler optimizations
	for (n = 0; n < lcd18; n++) Nop(); // works at 200 (slow white) or 24 (fast blue)
	//    lcdhits_18tcy++;
}

//------------------------------------------

void DelayPORXLCD(void) // works with 15
{
	Delay10KTCYx(15); // Delay of 15ms
	return;
}

//------------------------------------------

void DelayXLCD(void) // works with 5
{
	Delay10KTCYx(5); // Delay of 5ms
	return;
}

void wdtdelay(uint32_t delay)
{
	static uint32_t dcount;
	for (dcount = 0; dcount <= delay; dcount++) { // delay a bit
		Nop();
		ClrWdt(); // reset the WDT timer
	};
}

void wdttime(uint32_t delay) // delay = ~ .05 seconds
{
	static uint32_t dcount, timetemp, clocks_hz;
	s_crit(HL);
	dcount = V.clock20;
	e_crit();
	clocks_hz = dcount + delay;

	do { // wait until delay
		s_crit(HL);
		timetemp = V.clock20;
		e_crit();
		ClrWdt();
	} while (timetemp < clocks_hz);
}

/* Misc ACSII spinner character generator, stores position for each shape */
uint8_t spinners(uint8_t shape, uint8_t reset)
{
	static uint8_t s[MAX_SHAPES], last_shape = 0;
	uint8_t c;

	if (shape > (MAX_SHAPES - 1)) shape = 0;
	if (reset) s[shape] = 0;
	last_shape = shape;
	c = spin[shape][s[shape]];
	if (++s[shape] >= strlenpgm(spin[shape])) s[shape] = 0;
	return c;
}

void term_time(void)
{

}

/*	lp_filters
	0
	1
	2
	3	current
	4	currentin
	5	currentcharger
	6
	7
	8
 *	9
 */
float lp_filter(float new, int16_t bn, int16_t slow) // low pass filter, slow rate of change for new, LPCHANC channels, slow/fast select (1) to zero channel
{
	// smooth,lp_speed,lp_x is a udata global array to save stack space

	if (bn > LPCHANC)
		return new;
	if (slow) {
		lp_speed = 0.06;
	} else {
		lp_speed = 0.250;
	}
	lp_x = ((smooth[bn]*100.0) + (((new * 100.0)-(smooth[bn]*100.0)) * lp_speed)) / 100.0;
	smooth[bn] = lp_x;
	if (slow == (-1)) { // reset and return zero
		lp_x = 0.0;
		smooth[bn] = 0.0;
	}
	return lp_x;
}

void init_lcd(void)
{
	lcd18 = 200;
	wdtdelay(10000); // delay for power related LCD setup glitch
	OpenXLCD(FOUR_BIT & LINES_5X7);
	while (BusyXLCD());
	wdtdelay(10000); // delay for power related LCD setup glitch
	OpenXLCD(FOUR_BIT & LINES_5X7);
	while (BusyXLCD());
	WriteCmdXLCD(0xc); // blink, cursor off
	while (BusyXLCD());
	WriteCmdXLCD(0x1); // clear screen
	wdtdelay(10000);
	LCD_OK = TRUE;
	lcd18 = 24;
}

void main(void) // Lets Party
{
	static uint8_t eep_char = 0;
	static uint8_t z = 0;
	static uint16_t adc_result;


#ifdef	__18F8722
	config_pic(PIC_8722); // configure all controller hardware to the correct settings and ports
#endif

#ifdef	__18F8722
	start_pic(PIC_8722); // configure external hardware to the correct startup conditions
#endif

	init_lcd();
	putrsXLCD("Pyro Control");

#ifdef	__18F8722
	if (STKPTRbits.STKFUL) {
		STKPTRbits.STKFUL = 0;
		putrs2USART("\r\n");
		term_time();
		putrs2USART("\x1b[7m Restart from Stack Full. \x1b[0m\r\n");
	}
	if (STKPTRbits.STKUNF) {
		STKPTRbits.STKUNF = 0;
		putrs2USART("\r\n");
		term_time();
		putrs2USART("\x1b[7m Restart from Stack Underflow. \x1b[0m\r\n");
	}
#endif

	if (WDT_TO) {
		putrs2USART("\r\n");
		term_time();
		putrs2USART("\x1b[7m Restart from Watchdog Timer. \x1b[0m\r\n");
	}

	if (EEP_ER) {
		putrs2USART("\r\n");
		term_time();
		putrs2USART("\x1b[7m Possible EEPROM write error. \x1b[0m\r\n");
	}

	/* state machine setups */
	L.adc_chan = 0;

	/* setup the link buffers first */
	L.rx1b = &ring_buf1;
	L.tx1b = &ring_buf2;
	L.rx2b = &ring_buf3;
	L.tx2b = &ring_buf4;
	spi_link.tx1b = &ring_buf5;
	spi_link.rx1b = &ring_buf6;

	ringBufS_init(L.rx1b);
	ringBufS_init(L.tx1b);
	ringBufS_init(L.rx2b);
	ringBufS_init(L.tx2b);
	ringBufS_init(spi_link.tx1b);
	ringBufS_init(spi_link.rx1b);

	/*      Work thread start */
	start_workerthread();

	SYSTEM_STABLE = TRUE;
	srand((uint16_t) R.systemvoltage); // set random seed

	/* Loop forever */
	while (TRUE) {
		ClrWdt(); // reset the WDT timer
		if (ringBufS_empty(L.rx1b)) {
			DLED_2 = LOW;
		} else {
			DLED_2 = HIGH;
			adc_result = ringBufS_get(L.rx1b); // get the analog voltages
			// do something
			ringBufS_put(spi_link.tx1b, (adc_result>>13)); // send control data to SPI devices (DAC)
		}

		if (SSPCON1bits.WCOL || SSPCON1bits.SSPOV) { // check for overruns/collisions
			SSPCON1bits.WCOL = (SSPCON1bits.SSPOV = LOW);
			ringBufS_flush(spi_link.tx1b, 1); // dump the spi buffers
			ringBufS_flush(spi_link.rx1b, 1);
		}
	}
}

