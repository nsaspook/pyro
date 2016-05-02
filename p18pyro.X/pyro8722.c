
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
 * TImer4		PWM Period clock

 * 0..8 analog channels are active
 * PORTA		analog inputs
 * adc0	systemvoltage	PIC Controller 5vdc supply voltage
 * adc1	motorvoltage	24vdv PS monitor from relay
 * adc2	current_x
 * adc11 current_y
 * adc4	current_z
 * PORTB		HID Qencoder and switch inputs
 * PORTC		HID leds
 * PORTD		configuration switch input
 * PORTE		motor control relays
 * PORTF		analog inputs
 * adc5	rawp1 X pot RF0
 * adc6 rawp2 Y pot RF1
 * adc7 rawp3 Z pot RF2
 * adc8 Ground REF	zero adc charge cap RF3
 * adc3 VREF from 5vdc reference chip REF02AP
 * adc_cal[11-14]	current sensors zero offset stored in eeprom 11=x, 12=y, 13=z, 14=future
 * cal table with checksum as last data item in adc_cal[]
 * PORTH0		run flasher led onboard, 4x20 LCD status panel
 * PORTJ		alarm and diag leds
 * PORTG		Alarm and Voice outputs
 *
 *
 * frederick.brooks@microchip.com Copyright 2014
 * Microchip Gresham, Oregon
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
#include <delays.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "pyro_defs.h"
#include "xlcd.h"
#include "pyro.h"
#include "crit.h"
#include "mandm_msg.h"
#include "hwconf.h"
#include "pyro_vector.h"
#include "pyro_shared.h"
#include "daq.h"

void clr_lcd(void);
int8_t *ahfp(int32_t, int8_t *);
int8_t *voltfp(uint32_t, int8_t *);
int8_t *voltfps(uint32_t, int8_t *);
int8_t *voltfpi(int32_t, int8_t *);
void MputsXLCD(int8_t *);

void fail_safe(void);
int8_t *hms(uint32_t);
int8_t *hm(uint32_t);
float lp_filter(float, int16_t, int16_t);
void putc2(uint8_t c);
void hold_process(void); // hold monitor progrsm
void system_data(void); // send system variables to rs-232 terminal
void system_help(void); // send system help menu to rs-232 terminal
void exerciser(void); // test calibrated assy
int16_t e220_qei_exer(uint8_t);
int16_t nonqei_exer(uint8_t);
uint8_t check_alarm(uint8_t, const rom int8_t*);
int32_t ABSL(int32_t);
int16_t ABSI(int16_t);
void LCD_VC_puts(uint8_t, uint8_t, uint8_t);
void wdttime(uint32_t);
uint8_t checktime(uint32_t, uint8_t);
uint8_t checktime_select(uint32_t, uint8_t);
uint8_t checktime_motor(uint32_t, uint8_t);
uint8_t checktime_cal(uint32_t, uint8_t);
uint8_t checktime_eep(uint32_t, uint8_t);
uint8_t checktime_track(uint32_t, uint8_t);
void wdtdelay(uint32_t);
void update_hist(void);
void term_time(void);
void viision_m_display(void);
void help_m_display(void);
void viision_ms_display(void);
void v810_ms_display(void);
void varian_v_display(void);
void e220_m_display(void);
void e220_qei_display(void);
void gsd_m_display(void);
void default_display(void);
void Set_Cursor(void);
void run_demos(uint8_t);
void run_cal(void);
void display_cal(void);
void init_motordata(uint8_t);
int16_t track_motor(void);
void init_lcd(void);
uint8_t check_cable(uint8_t *);
void fail_all_motors(uint8_t);
void nav_menu(void);
uint8_t spinners(uint8_t, uint8_t);

#pragma udata gpr13
far int8_t bootstr2[MESG_W + 1];
#pragma udata gpr1
int8_t p1[C_TEMP16], p2[C_TEMP16];
int8_t termstr[32];
uint8_t csd[SD_18], cid[SD_18], HCRIT[CRIT_8], LCRIT[CRIT_8];
far int8_t f1[C_TEMP7], f2[C_TEMP7], f3[C_TEMP7], f4[C_TEMP7];
int8_t p3[C_TEMP16];
volatile struct almbuffertype alarm_buffer[MAXALM];
float smooth[LPCHANC];
#pragma udata gpr2
struct lcdb ds[VS_SLOTS]; // , ms[VS_SLOTS]; //  LCD/MENU display strings, name
#pragma udata gpr3
far int8_t hms_string[16];
int16_t a10_x, a10_y, a10_z, worktick;
volatile uint8_t critc_level = 0, KEYNUM = 0, C2RAW, glitch_count, cdelay, SLOW_STATUS;
volatile uint8_t TIMERFLAG = FALSE, PRIPOWEROK = TRUE, FORCEOUT = FALSE, WORKERFLAG = FALSE,
	FAILSAFE = FALSE, SYSTEM_STABLE = FALSE, HOLD_PROC = FALSE,
	DISPLAY_MODE = FALSE, D_UPDATE = TRUE, GLITCH_CHECK = TRUE, COOLING = FALSE,
	UPDATE_EEP = FALSE, RESET_ZEROS = FALSE, SYS_DATA = FALSE, MOD_DATA = FALSE, SYS_HELP = FALSE, SET_TLOG = FALSE,
	WDT_TO = FALSE, EEP_ER = FALSE, TWEAK = FALSE, TEST_SPINNERS = FALSE;
#pragma udata gpr4
volatile struct almtype alarm_codes = {FALSE, 0};
volatile struct modetype mode = {FALSE, TRUE, TRUE, HELP_M, TRUE, TRUE, FALSE, FALSE, FALSE, FALSE, TRUE, TRUE, FALSE, FALSE, FALSE};
volatile uint8_t almctr, RS232_DEBUG = FALSE;
volatile uint32_t critc_count = 0;
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
uint32_t Vin = 0, chrg_v = 0, vbatol_t = 0, solar_t = 0, rawp[MAX_POT], rawa[MAX_POT];
volatile uint8_t IDLEFLAG = FALSE, knob_to_pot = XAXIS;
int32_t iw = 0, ip = 0;
#pragma udata gpr8
volatile struct motortype motordata[MAX_MOTOR], *motor_ptr;
#pragma udata gpr9
volatile struct knobtype knob1, knob2;

/* ADC voltage/current default calibration values , adjusted with D command */
// adc_cal[11-14]				current sensors zero offset stored in eeprom 11=x, 12=y, 13=z, 14=future
uint8_t adc_cal[] = {127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 0};
uint8_t CRITC = 0, LCD_OK = FALSE, Cursor[MAX_POT], cable_type = 0x07, TLOG = FALSE;

volatile enum answer_t {
	WAIT_M, YES_M, NO_M
} YNKEY;

volatile struct QuadEncoderType OldEncoder;

#pragma idata gpr10
uint8_t lcd18 = 200;
volatile struct qeitype qei1;
volatile int32_t slow_timer = 0;
struct emodefaulttype emodump;
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

void LCD_VC_puts(uint8_t console, uint8_t line, uint8_t COPY) // VCx,DSx, [TRUE..FALSE} copy data from bootstr2 string
{ // into the LCD display buffer
	static uint8_t ib = 0;

	if (!LCD_OK) return;
	//    lcdhits++;
	if (COPY) {
		ib = console + line; // set to string index to store data in LCD message array ds[x].b
		strncpypgm2ram(ds[ib].b, "                        ", LCD_W); // write 20 space chars
		strncpy(ds[ib].b, bootstr2, LCD_W); // move data from static buffer in lcd message array
		ds[ib].b[LCD_W] = NULL0; // make sure we have a string terminator
	}
	switch (line) {
	case DS0:
		SetDDRamAddr(LL1); // move to  line
		break;
	case DS1:
		SetDDRamAddr(LL2); // move to  line
		break;
	case DS2:
		SetDDRamAddr(LL3); // move to  line
		break;
	case DS3:
		SetDDRamAddr(LL4); // move to  line
		break;
	default:
		SetDDRamAddr(LL1); // move to  line 1 of out of range
		break;
	}
	ib = dsi + line; // set to string index to display on LCD, dsi is the current VC being displayed
	if (!D_UPDATE) return;
	while (BusyXLCD());
	putsXLCD(ds[ib].b);
	while (BusyXLCD());
	LATH &= 0b00000001;
}

void update_hist(void) // compute the runtime data from current data
{

	// check FLAGS from the tick ISR and run subroutine if needed
	if (RESET_ZEROS) { // zero the current sensors
		zero_amploc();
		RESET_ZEROS = FALSE;
	}
	if (SET_TLOG) { // Tracking RS-232 LOGGING
		SET_TLOG = FALSE;
		TLOG = !TLOG;
	}
	if (TWEAK) { // Adjust runtime data
		exerciser();
		TWEAK = FALSE;
	}
	if (HOLD_PROC) { // Hold processing
		hold_process();
		HOLD_PROC = FALSE;
	}
	if (SYS_HELP) { // Command Help message
		system_help();
		SYS_HELP = FALSE;
	}
	if (SYS_DATA) { // display System data on rs-232 port
		system_data();
		SYS_DATA = FALSE;
	}
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

uint8_t checktime(uint32_t delay, uint8_t set) // delay = ~ .05 seconds
{
	static uint32_t dcount, timetemp, clocks_hz;

	if (set) {
		s_crit(HL);
		dcount = V.clock20;
		e_crit();
		clocks_hz = dcount + delay;
	}

	s_crit(HL);
	timetemp = V.clock20;
	e_crit();
	if (timetemp < clocks_hz) return FALSE;
	return TRUE;
}

uint8_t checktime_select(uint32_t delay, uint8_t set) // delay = ~ .05 seconds
{
	static uint32_t dcount, timetemp, clocks_hz;

	if (set) {
		s_crit(HL);
		dcount = V.clock20;
		e_crit();
		clocks_hz = dcount + delay;
	}

	s_crit(HL);
	timetemp = V.clock20;
	e_crit();
	if (timetemp < clocks_hz) return FALSE;
	return TRUE;
}

uint8_t checktime_motor(uint32_t delay, uint8_t set) // delay = ~ .05 seconds
{
	static uint32_t dcount, timetemp, clocks_hz;

	if (set) {
		s_crit(HL);
		dcount = V.clock20;
		e_crit();
		clocks_hz = dcount + delay;
	}

	s_crit(HL);
	timetemp = V.clock20;
	e_crit();
	if (timetemp < clocks_hz) return FALSE;
	return TRUE;
}

uint8_t checktime_cal(uint32_t delay, uint8_t set) // delay = ~ .05 seconds
{
	static uint32_t dcount, timetemp, clocks_hz;

	if (set) {
		s_crit(HL);
		dcount = V.clock20;
		e_crit();
		clocks_hz = dcount + delay;
	}

	s_crit(HL);
	timetemp = V.clock20;
	e_crit();
	if (timetemp < clocks_hz) return FALSE;
	return TRUE;
}

uint8_t checktime_eep(uint32_t delay, uint8_t set) // delay = ~ .05 seconds
{
	static uint32_t dcount, timetemp, clocks_hz;

	if (set) {
		s_crit(HL);
		dcount = V.clock20;
		e_crit();
		clocks_hz = dcount + delay;
	}

	s_crit(HL);
	timetemp = V.clock20;
	e_crit();
	if (timetemp < clocks_hz) return FALSE;
	return TRUE;
}

uint8_t checktime_track(uint32_t delay, uint8_t set) // delay = ~ .05 seconds
{
	static uint32_t dcount, timetemp, clocks_hz;

	if (set) {
		s_crit(HL);
		dcount = V.clock20;
		e_crit();
		clocks_hz = dcount + delay;
	}

	s_crit(HL);
	timetemp = V.clock20;
	e_crit();
	if (timetemp < clocks_hz) return FALSE;
	return TRUE;
}

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

void putc2(uint8_t c)
{
	while (Busy2USART()) {
	}; // wait until the usart is clear
	putc2USART(c);
}

void start_delay(void)
{
	wdttime(BATRUNF); // wait for .5 seconds.
}

void clr_lcd()
{
	if (!D_UPDATE) return;
	while (BusyXLCD());
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

void term_time(void) // sent internal time to rs-232 terminal
{
	ClrWdt(); // reset the WDT timer
	sprintf(termstr, " CT-%s", hms(V.timerint_count));
	puts2USART(termstr);
}

void hold_process(void) // hold the monitor in the current status until released.
{
	term_time();
	putrs2USART(hold0);
	YNKEY = WAIT_M;
	while (YNKEY == WAIT_M) {
		wdtdelay(10);
		ADC_read();
		idle_loop();
	}
	if (YNKEY == NO_M) {
		term_time();
		putrs2USART(hold1);
		return;
	}
	term_time();
	putrs2USART(hold1);
}

void system_data(void) // display system data on terminal
{
	static uint8_t z;
	ClrWdt(); // reset the WDT timer
	sprintf(bootstr2, "\r\n");
	puts2USART(bootstr2);
	term_time();
	putrs2USART(" FW ");
	strcpypgm2ram(bootstr2, build_time);
	puts2USART(bootstr2);
	putrs2USART(" ");
	strcpypgm2ram(bootstr2, build_date);
	puts2USART(bootstr2);
	strcpypgm2ram(bootstr2, MANDM_VERSION);
	puts2USART(bootstr2);
	sprintf(bootstr2,
		"\r\n Power status 1=OK %u, Config DIPSW %u%u%u%u%u%u%u%u, Temp Sensor %lu,Temp Comp %i\r\n Highint %lu, Lowint %lu, Critc Levels %i, HID Idle %i, ",
		PRIPOWEROK, DIPSW1, DIPSW2, DIPSW3, DIPSW4, DIPSW5, DIPSW6, DIPSW7, DIPSW8, R.thermo_batt, C.temp_drate, V.highint_count, V.lowint_count, (int16_t) critc_level, (int16_t) mode.idle);
	puts2USART(bootstr2);
	sprintf(bootstr2,
		"Timerint %lu, Workerint %lu, \r\n LCD display counts %lu, LCD 18tcy counts %lu, Com2int %lu, Bint %lu, Eint %lu, Aint %lu, Lowclocks %lu, Lowruns/S %lu, Timer4int %lu, TMR4 %i, qei_counts %i\r\n",
		V.timerint_count, V.worker_count, V.lcdhits, V.lcdhits_18tcy, V.c2_int, V.buttonint_count, V.eeprom_count, V.adc_count, V.clock20, V.clock20 / V.timerint_count, V.pwm4int_count, (int16_t) TMR4, (int16_t) V.qei_counts);
	puts2USART(bootstr2);

	sprintf(bootstr2,
		" Button int counts B3 %lu, B2 %lu, B1 %lu, B0 %lu : Motor Hunt Counts %lu, QEI position %li \r\n",
		V.b3, V.b2, V.b1, V.b0, V.hunt_count, qei1.c);
	puts2USART(bootstr2);

	for (z = 0; z < MAX_MOTOR; z++) {
		if (motordata[z].active) {
			sprintf(bootstr2,
				" Motor #%i, MPos %4i MSet %4i ,Merror %4i , MHigh %4i MLow %4i MChange %4i , SPos %4i SSet %4i , SOffset %4i SSpan %4i ",
				(int16_t) z, motordata[z].pot.pos_actual, motordata[z].pot.pos_set, motordata[z].pot.error, motordata[z].pot.high, motordata[z].pot.low, motordata[z].pot.pos_change,
				motordata[z].pot.scaled_actual, motordata[z].pot.scaled_set, motordata[z].pot.offset, motordata[z].pot.span);
			puts2USART(bootstr2);
			sprintf(bootstr2,
				" SScale In %4i SScale out %4i \r\n",
				(int16_t) (motordata[z].pot.scale_in * 1000.0), (int16_t) (motordata[z].pot.scale_out * 1000.0));
			puts2USART(bootstr2);
		}
	}
	if (TLOG) putrs2USART(" Extra Debug Enabled.\r\n");
	ClrWdt(); // reset the WDT timer
}

void system_help(void) // display system help on terminal
{
	putrs2USART(keycmds0);
	putrs2USART(keycmds1);
	if (TLOG) putrs2USART(" Extra Debug Enabled.\r\n");
}

int8_t* hms(uint32_t sec) // convert int32_t (seconds) to time string
{
	static uint32_t h = 0, m = 0, s = 0;

	if (sec > MAXSECONDS) sec = MAXSECONDS; // max time in seconds
	s = sec;
	h = (s / 3600);
	s = s - (h * 3600);
	m = (s / 60);
	s = s - (m * 60);
	sprintf(hms_string, "%01lu:%02lu:%02lu", h, m, s);
	return hms_string;
}

int8_t* hm(uint32_t sec) // convert int32_t (seconds) to time string
{
	static uint32_t h = 0, m = 0, s = 0;
	if (sec > MAXSECONDS) sec = MAXSECONDS; // max time in seconds
	s = sec;
	h = (s / 3600);
	s = s - (h * 3600);
	m = (s / 60);
	s = s - (m * 60);
	sprintf(hms_string, "%01lu:%02lu", h, m);
	return hms_string;
}

int16_t ABSI(int16_t i)
{
	if (i < 0)
		return -i;
	else
		return i;
}

int32_t ABSL(int32_t i)
{
	if (i < 0)
		return -i;
	else
		return i;
}

int8_t* voltfp(uint32_t millvolts, int8_t *strprt) // convert uint32_t (millvolts) to voltage string
{
	voltfrak = (float) millvolts / 1000;
	iw = (int32_t) ((float) voltfrak);
	ip = (int32_t) ((float) voltfrak * 100) - iw * 100;
	sprintf(strprt, "%d.%02d", (int16_t) iw, (int16_t) ip);
	return strprt;
}

int8_t* voltfps(uint32_t millvolts, int8_t *strprt) // convert uint32_t (millvolts) to voltage string (short)
{
	voltfrak = (float) millvolts / 1000;
	iw = (int32_t) ((float) voltfrak);
	ip = (int32_t) ((float) voltfrak * 10) - iw * 10;
	sprintf(strprt, "%d.%01d", (int16_t) iw, (int16_t) ip);
	return strprt;
}

int8_t* voltfpi(int32_t millvolts, int8_t *strprt) // convert int32_t (mill volts/watts/current) to string (integer with Kilo)
{
	sign = ' '; // init sign
	if (millvolts < 0)
		sign = '-';
	voltfrak = (float) millvolts / 1000;
	iw = (int32_t) ((float) voltfrak);
	ip = (int32_t) ((float) voltfrak * 10) - iw * 10;
	if (ABSL(iw) < 1500) {
		sprintf(strprt, "%c%li", sign, ABSL(iw));
	} else {
		voltfrak = voltfrak / 1000;
		iw = (int32_t) ((float) voltfrak);
		ip = (int32_t) ((float) voltfrak * 100) - iw * 100;
		sprintf(strprt, "%c%li.%02lik", sign, ABSL(iw), ABSL(ip));
	}
	return strprt;
}

int8_t* ahfp(int32_t millah, int8_t *strprt) // convert int32_t (.1 of Ah) to Ah string
{
	sign = ' '; // init sign
	if (millah < 0)
		sign = '-';
	ahfrak = (float) millah / 10;
	iw = (int32_t) ((float) ahfrak);
	ip = (int32_t) ((float) ahfrak * 10) - iw * 10;
	sprintf(strprt, "%c%d.%01d", sign, ABSI(iw), ABSI(ip));
	return strprt;
}

/*  prints messages from the alarm array, need a rework as it's buggy */
uint8_t check_alarm(uint8_t bn, const rom int8_t* where) // print alarm codes to terminal or just the battery number with code 251
{ // or set flag with code 253 or clear flag code 252
	static uint8_t i = 0, alm_num = 255; // if bn is 255 return state of alarm_codes.alm_flag flag TRUE/FALSE, 254 return current alarm code
	s_crit(HL);

	if (bn > 250) {
		if (bn == 255) {
			if (alarm_codes.alm_flag) {
				e_crit();
				return TRUE;
			} else {
				e_crit();
				return FALSE;
			}
		}
		if (bn == 254) {
			e_crit();
			return alarm_buffer[almctr].alm_num;
		}
		if (bn == 253) {
			alarm_codes.alm_flag = TRUE;
			e_crit();
			return 253;
		}
		if (bn == 252) {
			alarm_codes.alm_flag = FALSE;
			e_crit();
			return 252;
		}
		if (bn == 251) {
			alarm_codes.alm_flag = FALSE;
			e_crit();
			return alarm_buffer[almctr].bn;
		}
	}

	if (alarm_codes.alm_flag == NULL0) { // nothing, lets go
		e_crit();
		return FALSE;
	}

	while (almctr > NULL0) {
		alm_num = alarm_buffer[--almctr].alm_num; // move down the alarm stack
		term_time();
		putrs2USART(where);
		sprintf(bootstr2, " Q%u, Battery %u,", almctr, alarm_buffer[almctr].bn); // use the message battery number if set
		puts2USART(bootstr2);
		i = 0; // set alarm index to zero
		if (alm_num == i++) putrs2USART(almcode0); //0
		if (alm_num == i++) putrs2USART(almcode1); //1
		if (alm_num == i++) putrs2USART(almcode2); //2
		if (alm_num == i++) putrs2USART(almcode3); //3
		if (alm_num == i++) putrs2USART(almcode4); //4
		if (alm_num == i++) putrs2USART(almcode5); //5
		if (alm_num == i++) putrs2USART(almcode6); //6
		if (alm_num == i++) putrs2USART(almcode7); //7
		if (alm_num == i++) putrs2USART(almcode8); //8
		if (alm_num == i++) putrs2USART(almcode9); //9
		if (alm_num == i++) putrs2USART(divert0); //10
		if (alm_num == i++) putrs2USART(divert1); //11
		if (alm_num == i++) putrs2USART(battbuffer0); //12
		if (alm_num == i++) putrs2USART(almcode10); //13
		if (alm_num == i++) putrs2USART(almcode11); //14
		if (alm_num == i++) putrs2USART(almcode12); //15
		if (alm_num == i++) putrs2USART(almcode13); //16
		if (alm_num == i++) putrs2USART(almcode14); //17
		if (alm_num == i++) putrs2USART(almcode15); //18
		if (alm_num == i++) putrs2USART(almcode16); //19
		if (alm_num == i++) putrs2USART(charger2); //20
		if (alm_num == i++) putrs2USART(charger3); //21
		if (alm_num == i++) putrs2USART(charger4); //22
		if (alm_num == i++) putrs2USART(almcode17); //23
		if ((alm_num > i) && (alm_num < 250)) { // if greater than last defined alarm code
			sprintf(bootstr2, " UNKNOWN Alarm code %u \r\n", alm_num);
			puts2USART(bootstr2);
		}
		if (almctr == NULL0) {
			alarm_codes.alm_flag = FALSE;
		}
		alarm_buffer[almctr].bn = 0; // set default battery number to zero
		alarm_buffer[almctr].time = 0; // set alarm time to zero
	};
	e_crit();
	return NULL0;
}

void fail_safe(void)
{
	FAILSAFE = TRUE;
	while (DIPSW3 == HIGH) { // wait until switch is clear
		ADC_read();
		sprintf(bootstr2, "FS T%s,SV%lu             ", hms(V.timerint_count), R.systemvoltage / 100);
		LCD_VC_puts(VC0, DS0, YES);

		//	sprintf(bootstr2, "PV%lu,PI%li,CV%lu           ", R.inputvoltage / 100, R.currentin / 10, R.ccvoltage / 100);
		LCD_VC_puts(VC0, DS1, YES);

		//	sprintf(bootstr2, "B1V%lu,B2V%lu,LI%li                 ", R.primarypower[B1] / 100, R.primarypower[B2] / 100, C.currentload / 10);
		LCD_VC_puts(VC0, DS2, YES);

		sprintf(bootstr2, "%lu, %lu, %lu                  ", V.timerint_count, V.clock20, V.clock20 / V.timerint_count);
		LCD_VC_puts(VC0, DS3, YES);

		wdtdelay(5000);
		idle_loop();
	}
	FAILSAFE = FALSE;

}

void ansidraw(int16_t mode) // tek 410x/ansi drawing stuff
{
	putc2(ESC);
	putc2('['); // clear screen
	putc2('2');
	putc2('J');
	putrs2USART(hello1);
	putrs2USART(hello0);
}

void update_lcd_menu(uint8_t menu_text_pos)
{
	static uint8_t slow;
	static uint8_t position = 0;

	if (!mode.move || (menu_text_pos == HELP_M)) { // lock the top line of the help screen
		if (mode.free) {
			strncpypgm2ram(bootstr2, menuselect_free[menu_text_pos], LCD_W); // show the selected menu text.
		} else {
			strncpypgm2ram(bootstr2, menuselect_track[menu_text_pos], LCD_W); // show the selected menu text.
		}
		mode.operate = menu_text_pos;
	} else {
		if (position != menu_text_pos) {
			mode.cal = FALSE;
			emotor_power_off();
			LED_1 = LOW;
			for (slow = 0; slow <= MENU_SPEED; slow++) {
				strncpypgm2ram(bootstr2, menutext + (slow + (menu_text_pos * 10)), LCD_W); // show the menu ribbon.
				LCD_VC_puts(dsi, DS0, YES);
				wdtdelay(10000);
			}
			position = menu_text_pos;
		} else {
			strncpypgm2ram(bootstr2, menutext + (10 + (menu_text_pos * 10)), LCD_W); // show the menu ribbon.
		}
	}
	LCD_VC_puts(dsi, DS0, YES);
	LCD_VC_puts(dsi, DS1, NO);
	LCD_VC_puts(dsi, DS2, NO);
	LCD_VC_puts(dsi, DS3, NO);
}

void Set_Cursor(void)
{
	static uint8_t z, shape = 0, s_delay;

	for (z = 0; z < MAX_POT; z++) {
		Cursor[z] = 0b11111110; // blank
	}
	if (motordata[knob_to_pot].run || TEST_SPINNERS) {
		if (TEST_SPINNERS) {
			if (s_delay++ > 50) {
				if (++shape > MAX_SHAPES - 1) shape = 0;
				s_delay = 0;
			}
		} else {
			shape = knob_to_pot + 3;
		}
		Cursor[knob_to_pot] = spinners(shape, FALSE);
	} else {
		Cursor[knob_to_pot] = 0b11111111; // solid box
	}
}

int16_t track_motor(void)
{ // move motor feedback pot/encoder to setpoint
	static float deadband = 0.0;
	static int16_t was_running = FALSE, old_error = 0;
	uint8_t track_knob_to_pot, PERFECT = FALSE;
	uint8_t menu_pos;

	if (mode.free || mode.on_off_only || (mode.info_only)) return MERR_INV;

	track_knob_to_pot = knob_to_pot; // set the local version
	motordata[track_knob_to_pot].hunt_count = 0;
	checktime_track(TRACK_TIMEOUT, TRUE);
	if (motordata[track_knob_to_pot].pot.cal_failed) {
		if (!is_led_blinking(TEST_LED)) blink_led(TEST_LED, TRUE);
	} else {
		if (is_led_blinking(TEST_LED)) blink_led(TEST_LED, FALSE);
	}

	do { // move the assy/motor into the setting position
		check_cable(&menu_pos); // check DIPSW and set the menu_pos if needed
		if (menu_pos == cable_type) { // exit do loop if control cable is disconnected
			LED_3 = LOW;
			mode.free = TRUE;
			update_lcd_menu(menu_pos);
			return 2;
		}
		ADC_read();
		nav_menu();

		if (motordata[track_knob_to_pot].hunt_count > HUNT_MAX) { // adjust position deadband
			deadband = DEADB_HUNT;
		} else {
			if (motordata[track_knob_to_pot].run) {
				deadband = DEADB_RUN; // narrow deadband  when motor running
			} else {
				deadband = DEADB_STOP; // normal deadband when stopped
				if (knob2.movement != STOP) {
					deadband = DEADB_KNOB;
				}
			}
		}

		if (mode.qei) { // shift gears in encoder mode
			if (ABSL(motordata[track_knob_to_pot].pot.error) > QEI_FAST) {
				if (DRIVE_V24 == LOW) {
					POWER_X = LOW;
					wdttime(RELAYRUNF); // wait for .5 seconds.
					motordata[track_knob_to_pot].v24 = TRUE;
					DRIVE_V24 = HIGH;
					mode.v24 = TRUE;
					if (TLOG) putrs2USART("\x1b[7m 24 volt drive \x1b[0m\r\n");
					POWER_X = HIGH;
				}
			} else {
				if (DRIVE_V24 == HIGH) {
					POWER_X = LOW;
					wdttime(RELAYRUNF); // wait for .5 seconds.
					motordata[track_knob_to_pot].v24 = FALSE;
					DRIVE_V24 = LOW;
					mode.v24 = FALSE;
					if (TLOG) putrs2USART("\x1b[7m 5 volt drive \x1b[0m\r\n");
					POWER_X = HIGH;
				}
			}
			if (ABSL(motordata[track_knob_to_pot].pot.error) > QEI_VERY_FAST) {
				if (POWER_S == LOW) {
					if (DRIVE_V24 == HIGH) {
						mode.slow_bypass = TRUE;
						if (TLOG) putrs2USART("\x1b[7m Slowing resistor is off \x1b[0m\r\n");
						POWER_S = HIGH;
					}
				}
			} else {
				if (POWER_S == HIGH) {
					if (DRIVE_V24 == HIGH) {
						mode.slow_bypass = FALSE;
						if (TLOG) putrs2USART("\x1b[7m Slowing resistor is on \x1b[0m\r\n");
						POWER_S = LOW;
					}
				}
			}
		}

		if ((motordata[track_knob_to_pot].pot.error > (int16_t) ((float) TRACK_DB_L / deadband)) && (motordata[track_knob_to_pot].pot.error < (int16_t) ((float) TRACK_DB_H / deadband))) {
			if (PERFECT && mode.qei) {
				if (was_running) {
					deadband = DEADB_STOP; // normal deadband when stopped
					if (TLOG) {
						sprintf(bootstr2, " In QEI deadband, Error %3i, SLOW Flag %1i, SLOW Relay %i, Total Hunts %i \r\n", motordata[track_knob_to_pot].pot.error, motordata[track_knob_to_pot].slow, (int16_t) POWER_S, motordata[track_knob_to_pot].hunt_count);
						puts2USART(bootstr2);
					}
				}
				motor_control(&motordata[track_knob_to_pot]);
				if (qei1.movement == STOP) {
					motordata[track_knob_to_pot].run = FALSE;
					if (was_running) {
						deadband = DEADB_STOP; // normal deadband when stopped, so adjust it now
						if (knob2.movement != STOP) {
							deadband = DEADB_KNOB;
						}
						if (TLOG) {
							sprintf(bootstr2, " QEI stopped, Error %3i, SLOW Flag %1i, SLOW Relay &i, Total Hunts %i ", motordata[track_knob_to_pot].pot.error, motordata[track_knob_to_pot].slow, (int16_t) POWER_S, motordata[track_knob_to_pot].hunt_count);
							puts2USART(bootstr2);
							putrs2USART("\x1b[7m Motor Stopped. \x1b[0m\r\n");
						}
						was_running = FALSE;
						LED_3 = LOW;
					}
				}
			} else {
				motordata[track_knob_to_pot].run = FALSE;
				motor_control(&motordata[track_knob_to_pot]);
				if (was_running) {
					deadband = DEADB_STOP; // normal deadband when stopped, so adjust it now
					if (knob2.movement != STOP) {
						deadband = DEADB_KNOB;
					}
					if (TLOG) {
						sprintf(bootstr2, " %i Error %3i, SLOW Flag %1i, SLOW Relay %i, Total Hunts %i ", track_knob_to_pot, motordata[track_knob_to_pot].pot.error, motordata[track_knob_to_pot].slow, (int16_t) POWER_S, motordata[track_knob_to_pot].hunt_count);
						puts2USART(bootstr2);
						putrs2USART("\x1b[7m Motor Stopped. \x1b[0m\r\n");
					}
					was_running = FALSE;
					LED_3 = LOW;
				}
			}
		}

		if (motordata[track_knob_to_pot].pot.error > (int16_t) ((float) TRACK_DB_H / deadband)) { // check knob position
			motordata[track_knob_to_pot].run = TRUE;
			was_running = TRUE;
			if (!motordata[track_knob_to_pot].cw) {
				motordata[track_knob_to_pot].hunt_count++;
				if (motordata[track_knob_to_pot].hunt_count > 1) V.hunt_count++;
			}
			motordata[track_knob_to_pot].cw = TRUE;
			LED_3 = HIGH;
			motor_control(&motordata[track_knob_to_pot]);
			if ((old_error != motordata[track_knob_to_pot].pot.error) && (ABSL(motordata[track_knob_to_pot].pot.error) > TRACK_DISPLAY)) {
				if (TLOG) {
					sprintf(bootstr2, " %i Error %3i, SLOW Flag %1i, SLOW Relay %i, Hunts %i ", track_knob_to_pot, motordata[track_knob_to_pot].pot.error, motordata[track_knob_to_pot].slow, POWER_S, motordata[track_knob_to_pot].hunt_count);
					puts2USART(bootstr2);
					putrs2USART("\x1b[7m Motor CW. \x1b[0m\r\n");
				}
				old_error = motordata[track_knob_to_pot].pot.error;
			}
		}
		if (motordata[track_knob_to_pot].pot.error < (int16_t) ((float) TRACK_DB_L / deadband)) { // check knob position
			motordata[track_knob_to_pot].run = TRUE;
			was_running = TRUE;
			motordata[track_knob_to_pot].cw = FALSE;
			LED_3 = HIGH;
			motor_control(&motordata[track_knob_to_pot]);
			if ((old_error != motordata[track_knob_to_pot].pot.error) && (ABSL(motordata[track_knob_to_pot].pot.error) > TRACK_DISPLAY)) {
				if (TLOG) {
					sprintf(bootstr2, " %i Error %3i, SLOW Flag %1i, SLOW Relay %i, Hunts %i ", track_knob_to_pot, motordata[track_knob_to_pot].pot.error, motordata[track_knob_to_pot].slow, POWER_S, motordata[track_knob_to_pot].hunt_count);
					puts2USART(bootstr2);
					putrs2USART("\x1b[7m Motor CCW. \x1b[0m\r\n");
				}
				old_error = motordata[track_knob_to_pot].pot.error;
			}
		}
		if (motordata[track_knob_to_pot].hunt_count > HUNT_LOW) motordata[track_knob_to_pot].slow = TRUE;
		if (motordata[track_knob_to_pot].hunt_count > HUNT_HIGH) motordata[track_knob_to_pot].slow_only = TRUE;
	} while (motordata[track_knob_to_pot].run && !button.B0 && !button.B1 && !button.B2 && !checktime_track(TRACK_TIMEOUT, FALSE));
	if (mode.qei) {
		mode.slow_bypass = FALSE;
		DRIVE_V24 = LOW;
		mode.v24 = FALSE;
	}
	if (checktime_track(TRACK_TIMEOUT, FALSE)) {
		if (TLOG) putrs2USART("\x1b[7m Motor movement TIMED OUT, setting knob position to motor position. \x1b[0m\r\n");
		if (mode.qei) knob2.c = qei1.c; // sync stopped position for timeout
		buzzer_ticks(1);
		return 4;
	}
	if (button.B0 || button.B1 || button.B2) {
		if (TLOG) putrs2USART("\x1b[7m Motor movement Button Interrupt, setting knob position to motor position. \x1b[0m\r\n");
		if (mode.qei) knob2.c = qei1.c; // sync stopped position for INTERRUPT
		if (button.B0) {
			button.B0 = LOW;
			mode.free = TRUE;
			LED_3 = LOW;
		}
		//        Clear_All_Buttons();
		return MERR_INT;
	}
	return MOK;
}

//FIX-ME rewrite all the stupid returns.

uint8_t check_cable(uint8_t *menu)
{ //adapter cables codes
	static uint8_t old_menu = 0xff, new_menu = 0, check_menu = 0, cable_return = 0;

	new_menu = ((DIPSW >> 4) & cable_type);
#ifdef 	HOUSE_DEMO
	new_menu = ((E220E500_E >> 4) & cable_type);
#endif
	if (new_menu == cable_type) { // check for disconnected cable code 0x07
		if (old_menu != cable_type) wdttime(4); // wait before recheck if it's a change from old_menu
		check_menu = ((DIPSW >> 4) & cable_type);
		if (new_menu == check_menu) { // double check
			if (old_menu != cable_type) { // jumper disconnected
				*menu = HELP_M; // select the help menu first after disconnect
				mode.operate = *menu;
				help_pos = 0;
				term_time();
				sprintf(bootstr2, " Cable Disconnected %i \r\n", (int16_t) check_menu);
				puts2USART(bootstr2);
				old_menu = cable_type;
			}
			if (mode.locked) { // note change
				term_time();
				sprintf(bootstr2, " Menu selection is UNLOCKED \r\n");
				puts2USART(bootstr2);
				old_menu = cable_type;
				*menu = HELP_M; // always update to help on true disconnect when unlocking
				mode.move = TRUE;
				mode.change = FALSE;
			}
			mode.locked = FALSE; // we can change the equipment mode
			init_motordata(mode.operate);
			update_lcd_menu(mode.operate);
			cable_return = cable_type; // no cable connected OK to modify *menu
		} else {
			term_time();
			sprintf(bootstr2, " Disconnect Glitch!! Prev Cable Selection Code %i, New Cable Selection Code %i \r\n", (int16_t) old_menu, (int16_t) check_menu);
			puts2USART(bootstr2);
			*menu = old_menu;
			cable_return = 0;
		}
		return cable_return;
	}

	if (old_menu != new_menu) {
		buzzer_ticks(3);
		wdttime(4); // wait and recheck
		check_menu = ((DIPSW >> 4) & cable_type);
		if (new_menu == check_menu) {
			*menu = new_menu; // after double check modify the menu selection
			mode.operate = *menu;
			help_pos = 0;
			term_time();
			sprintf(bootstr2, " Old Cable Selection Code %i, New Cable Selection Code %i \r\n", (int16_t) old_menu, (int16_t) * menu);
			puts2USART(bootstr2);
			old_menu = *menu;
			if (!mode.locked) { // note change
				term_time();
				sprintf(bootstr2, " Menu selection is LOCKED \r\n");
				puts2USART(bootstr2);
				mode.change = FALSE;
			}
			mode.locked = TRUE; // equipment mode is locked, we can change the info screen
			init_motordata(mode.operate);
			mode.move = FALSE;
			update_lcd_menu(mode.operate);
			return 0;
		} else {
			term_time();
			sprintf(bootstr2, " Selection Glitch!! Prev Cable Selection Code %i, New Cable Selection Code %i \r\n", (int16_t) old_menu, (int16_t) check_menu);
			puts2USART(bootstr2);
		}
	}
	*menu = old_menu;
	return 0;
}

/* basic user interface */
uint8_t get_hid(void) // mode is global
{
	static uint8_t menu_pos = HELP_M;
	static uint8_t get_out;

	if (!motordata[knob_to_pot].active) {
		get_out = 0;
		do {
			knob_to_pot++;
			if (knob_to_pot >= MAX_POT) knob_to_pot = 0;
			if (++get_out > MAX_POT + MAX_POT) continue; //    trouble in rivercity
		} while (!motordata[knob_to_pot].active);
		Set_Cursor();
	}

	if (button.B1) {
		LED_2 = HIGH;
		get_out = 0;
		do { // find a active motor for the motor control knob
			knob_to_pot++;
			if (knob_to_pot >= MAX_POT) knob_to_pot = 0;
			if (++get_out > MAX_POT + MAX_POT) continue; //    trouble in rivercity
		} while (!motordata[knob_to_pot].active);

		Set_Cursor();
		term_time();
		sprintf(bootstr2, " knob_to_pot  %u ", (int16_t) knob_to_pot);
		puts2USART(bootstr2);
		putrs2USART("\x1b[7m Mode Button 2. \x1b[0m\r\n");
		LED_2 = LOW;
		button.B1 = FALSE;
	}

	// check DIPSW and set the menu_pos if needed
	if (button.B0 || (checktime_select(SELECT_ACTION, FALSE) && mode.move)) {
		if (!mode.move) mode.free = !mode.free; // toggle the motor tracking mode
		term_time();
		sprintf(bootstr2, " menu_pos  %u ", (int16_t) menu_pos);
		puts2USART(bootstr2);
		putrs2USART("\x1b[7m Mode Button 1. \x1b[0m\r\n");
		mode.move = FALSE; // switch modes with button 1 presses
		button.B0 = FALSE;
		LED_0 = LOW;
	} else {
		if (knob1.movement != STOP) { // do we have knob motion
			if (checktime(MENU_ACTION, FALSE)) { // is knob still moving and not stopped
				checktime_select(SELECT_ACTION, TRUE); // keep resetting the select timer
				mode.move = TRUE;
				LED_0 = HIGH;
				if ((knob1.movement == CW) && (ABSL(knob1.band) > MENU_BAND)) { // turning clockwise
					mode.move = TRUE;
					LED_0 = LOW;
					if (!mode.locked) {
						menu_pos++; // if the cable is open allow changes
						checktime(MENU_ACTION, TRUE); // reset the movement timer
						if (menu_pos >= MAX_MENU) menu_pos = 0;
						init_motordata(menu_pos);
						buzzer_ticks(1);
						term_time();
						sprintf(bootstr2, " menu_pos  %u ", (int16_t) menu_pos);
						puts2USART(bootstr2);
						putrs2USART("\x1b[7m Menu Right. \x1b[0m\r\n");
					} else {
						help_pos++; // if the cable is open allow changes
						checktime(MENU_ACTION, TRUE); // reset the movement timer
						if (help_pos >= MAX_HELP) help_pos = 0;
						//                        init_motordata(menu_pos);
						term_time();
						sprintf(bootstr2, " help_pos  %u ", (int16_t) help_pos);
						puts2USART(bootstr2);
						putrs2USART("\x1b[7m Help Right. \x1b[0m\r\n");
					}
				} else { // CCW
					if (ABSL(knob1.band) > MENU_BAND) { // turning counter-clockwise
						mode.move = TRUE;
						LED_0 = LOW;
						if (!mode.locked) {
							menu_pos--; // allow changes
							checktime(MENU_ACTION, TRUE); // reset the movement timer
							if (menu_pos >= MAX_MENU) menu_pos = MAX_MENU - 1;
							init_motordata(menu_pos);
							buzzer_ticks(1);
							term_time();
							sprintf(bootstr2, " menu_pos  %u ", (int16_t) menu_pos);
							puts2USART(bootstr2);
							putrs2USART("\x1b[7m Menu Left. \x1b[0m\r\n");
						} else {
							help_pos++; // allow changes
							checktime(MENU_ACTION, TRUE); // reset the movement timer
							if (help_pos >= MAX_HELP) help_pos = 0;
							//                           init_motordata(menu_pos);
							term_time();
							sprintf(bootstr2, " help_pos  %u ", (int16_t) help_pos);
							puts2USART(bootstr2);
							putrs2USART("\x1b[7m Help Left. \x1b[0m\r\n");
						}
					}
				}
				LED_0 = HIGH;
			}
		} else {
			checktime(MENU_ACTION, TRUE); // reset the movement timer
		}
	}
	if (mode.move) {
		LED_0 = HIGH; // update mode led
	} else {
		LED_0 = LOW; // update mode led
	}

	if (!mode.info_only) {
		if (mode.free || mode.on_off_only) {
			if (is_led_blinking(TEST_LED)) blink_led(TEST_LED, FALSE);
			if (knob2.movement != STOP) { // do we have knob motion
				if (checktime_motor(MOTOR_ACTION, FALSE)) { // is knob still moving and not stopped
					LED_2 = HIGH;
					if ((knob2.movement == CW) && (knob2.band > MOTOR_BAND)) { // turning clockwise
						motordata[knob_to_pot].run = TRUE;
						motordata[knob_to_pot].cw = TRUE;
						motor_control(&motordata[knob_to_pot]);
						checktime_motor(MOTOR_ACTION, TRUE); // reset the movement timer
						term_time();
						sprintf(bootstr2, " Motor Number  %u ", (int16_t) knob_to_pot);
						puts2USART(bootstr2);
						putrs2USART("\x1b[7m Motor CW. \x1b[0m\r\n");
					} else { // CCW
						if ((knob2.movement == CCW) && (ABSL(knob2.band) > MOTOR_BAND)) { // turning counter-clockwise
							motordata[knob_to_pot].run = TRUE;
							motordata[knob_to_pot].cw = FALSE;
							motor_control(&motordata[knob_to_pot]);
							checktime_motor(MOTOR_ACTION, TRUE); // reset the movement timer
							term_time();
							sprintf(bootstr2, " Motor Number  %u ", (int16_t) knob_to_pot);
							puts2USART(bootstr2);
							putrs2USART("\x1b[7m Motor CCW. \x1b[0m\r\n");
						}
					}
				}
			} else {
				if (!mode.on_off_only) { // stop motion with no POT movement
					motordata[knob_to_pot].run = FALSE;
					motordata[knob_to_pot].cw = TRUE;
					motor_control(&motordata[knob_to_pot]);
				} else {
					motordata[knob_to_pot].run = FALSE;
				}
				checktime_motor(MOTOR_ACTION, TRUE); // reset the movement timer
				LED_2 = LOW;
			}
		} else { // track motor to postion pot
			track_motor();
		}
	}

	if (mode.free) {
		if (!is_led_blinking(MENU_LED)) blink_led(MENU_LED, TRUE);
	} else {
		if (is_led_blinking(MENU_LED)) blink_led(MENU_LED, FALSE);
	}
	return menu_pos;
}

/* assembly selection */
void nav_menu(void) // call the correct screen display function
{
	Set_Cursor();
	mode.display();

}

int16_t e220_qei_exer(uint8_t times)
{
	uint8_t z;
	int32_t y;
	int16_t ret = 0;
	emotor_power_off(); // turn off the power first
	if (mode.free || motordata[0].pot.cal_failed) return 3; // only use when tracking is enabled.
	mode.v24 = FALSE;
	if (qei1.max == 0) return 2;
	for (z = 0; z <= times; z++) {
		term_time();
		sprintf(bootstr2, " Motor encoder exerciser loop # %i of %i\r\n", (int16_t) z + 1, (int16_t) times);
		puts2USART(bootstr2);
		knob2.c = (int32_t) ((float) qei1.max * 0.01);
		if (track_motor()) break;
		for (y = 0; y < 300; y += 3) {
			knob2.c = y + (int32_t) ((float) qei1.max * 0.666);
			motordata[knob_to_pot].pot.error = knob2.c - qei1.c; // find the raw encoder/encoder diff
			if (track_motor()) break;
		}

		for (y = 0; y < 300; y += 3) {
			knob2.c = y + (int32_t) ((float) qei1.max * 0.333);
			motordata[knob_to_pot].pot.error = knob2.c - qei1.c; // find the raw encoder/encoder diff
			if (track_motor()) break;
		}
		knob2.c = (int32_t) ((float) qei1.max * 0.9);
		motordata[knob_to_pot].pot.error = knob2.c - qei1.c; // find the raw encoder/encoder diff
		if (track_motor()) break;
		knob2.c = (int32_t) ((float) qei1.max * 0.1);
		motordata[knob_to_pot].pot.error = knob2.c - qei1.c; // find the raw encoder/encoder diff
		if (track_motor()) break;
		knob2.c = (int32_t) ((float) qei1.max * 0.5);
		motordata[knob_to_pot].pot.error = knob2.c - qei1.c; // find the raw encoder/encoder diff
		if (track_motor()) break;
		if (button.B0 || button.B1 || button.B2) {
			ret = 1;
			break;
		}
	}
	Clear_All_Buttons();
	mode.v24 = FALSE;
	emotor_power_off(); // turn off the power first
	wdttime(RELAYRUNF); // wait for .5 seconds.
	return ret;
}

int16_t nonqei_exer(uint8_t times)
{
	uint8_t z, m;
	int32_t y;
	int16_t ret = 0;
	emotor_power_off(); // turn off the power first
	if (mode.free || motordata[0].pot.cal_failed) return 3; // only use when tracking is enabled.
	mode.v24 = FALSE;
	for (z = 0; z < times; z++) {
		for (m = 0; m < MAX_MOTOR; m++) {
			knob_to_pot = m;
			if (motordata[knob_to_pot].active) {
				term_time();
				sprintf(bootstr2, " Motor encoder exerciser loop # %i of %i\r\n", (int16_t) z + 1, (int16_t) times);
				puts2USART(bootstr2);
				motordata[knob_to_pot].pot.scaled_set = 100;
				if (track_motor()) break;
				for (y = 0; y < 300; y += 3) {
					motordata[knob_to_pot].pot.scaled_set = y + 666;
					motordata[knob_to_pot].pot.error = motordata[knob_to_pot].pot.pos_set - motordata[knob_to_pot].pot.pos_actual;
					if (track_motor()) break;
				}

				for (y = 0; y < 300; y += 3) {
					motordata[knob_to_pot].pot.scaled_set = y + 333;
					motordata[knob_to_pot].pot.error = motordata[knob_to_pot].pot.pos_set - motordata[knob_to_pot].pot.pos_actual;
					if (track_motor()) break;
				}
				motordata[knob_to_pot].pot.scaled_set = 900;
				motordata[knob_to_pot].pot.error = motordata[knob_to_pot].pot.pos_set - motordata[knob_to_pot].pot.pos_actual;
				track_motor();
				motordata[knob_to_pot].pot.scaled_set = 100;
				motordata[knob_to_pot].pot.error = motordata[knob_to_pot].pot.pos_set - motordata[knob_to_pot].pot.pos_actual;
				track_motor();
				motordata[knob_to_pot].pot.scaled_set = motordata[z].cal_pos; // move to install position;
				motordata[knob_to_pot].pot.error = motordata[knob_to_pot].pot.pos_set - motordata[knob_to_pot].pot.pos_actual;
				track_motor();
				if (button.B0 || button.B1 || button.B2) {
					ret = 1;
					break;
				}
			}
		}
	}
	Clear_All_Buttons();
	mode.v24 = FALSE;
	emotor_power_off(); // turn off the power first
	wdttime(RELAYRUNF); // wait for .5 seconds.
	return ret;
}

/* assembly selection */
void exerciser(void)
{
	Set_Cursor();
	switch (mode.operate) {
	case VIISION_M:
		nonqei_exer(2);
		break;
	case E220E500_M:
		nonqei_exer(2);
		break;
	case E220E500_E:
		e220_qei_exer(3);
		break;
	case GSD_M:
		nonqei_exer(2);
		break;
	case VIISION_MS:
		nonqei_exer(2);
		break;
	case V810_MS:
	case EV810_MS:
		nonqei_exer(2);
		break;
	case VARIAN_V:
		nonqei_exer(4);
		break;
	case HELP_M:
		mode.display();
		break;
	default:
		mode.display();
		break;
	}
}

/* main HID loop */
void hid_menu(void)
{
	static uint8_t menu_pos = HELP_M;

	menu_pos = get_hid();
	check_cable(&menu_pos);
	update_lcd_menu(menu_pos);
	nav_menu();
}

/* disconnect help screen */
void help_m_display(void)
{
	if (help_pos == 0) {
		sprintf(bootstr2, "%cConnect Adapter to ", Cursor[0]);
		LCD_VC_puts(VC0, DS1, YES);
		sprintf(bootstr2, "%cModule. Press Test ", Cursor[1]);
		LCD_VC_puts(VC0, DS2, YES);
		sprintf(bootstr2, "%cButton to Calibrate", Cursor[2]);
		LCD_VC_puts(VC0, DS3, YES);
	} else {
		sprintf(bootstr2, "                     ");
		LCD_VC_puts(VC0, DS1, YES);
		sprintf(bootstr2, "                     ");
		LCD_VC_puts(VC0, DS2, YES);
		sprintf(bootstr2, "                     ");
		LCD_VC_puts(VC0, DS3, YES);
	}
}

void gsd_m_display(void)
{
	if (help_pos == 0) {
		sprintf(bootstr2, "%cA%3i Set%3i V%3li I%2li               ", Cursor[0], motordata[0].pot.scaled_actual, motordata[0].pot.scaled_set, R.pos_x, R.current_x);
		LCD_VC_puts(VC0, DS1, YES);
		sprintf(bootstr2, "Move Cable to Axis              ");
		LCD_VC_puts(VC0, DS2, YES);
		sprintf(bootstr2, "Press Cal Button            ");
		LCD_VC_puts(VC0, DS3, YES);
	} else {
		sprintf(bootstr2, "                     ");
		if (motordata[0].active) sprintf(bootstr2, "A Pot%3i O%2i S%2i C%2i               ", motordata[0].pot.pos_actual, motordata[0].pot.offset / 10, motordata[0].pot.span / 10, motordata[0].pot.pos_change);
		LCD_VC_puts(VC0, DS1, YES);
		puts2USART(bootstr2);
		putrs2USART("\r\n");
		sprintf(bootstr2, "                         ");
		LCD_VC_puts(VC0, DS2, YES);
		sprintf(bootstr2, "                         ");
		LCD_VC_puts(VC0, DS3, YES);
	}
}

void e220_m_display(void)
{
	if (help_pos == 0) {
		sprintf(bootstr2, "%cX%3i Set%3i V%3li I%2li               ", Cursor[0], motordata[0].pot.scaled_actual, motordata[0].pot.scaled_set, R.pos_x, R.current_x);
		LCD_VC_puts(VC0, DS1, YES);
		sprintf(bootstr2, "%cY%3i Set%3i V%3li I%2li               ", Cursor[1], motordata[1].pot.scaled_actual, motordata[1].pot.scaled_set, R.pos_y, R.current_y);
		LCD_VC_puts(VC0, DS2, YES);
		sprintf(bootstr2, "%cZ%3i Set%3i V%3li I%2li               ", Cursor[2], motordata[2].pot.scaled_actual, motordata[2].pot.scaled_set, R.pos_z, R.current_z);
		LCD_VC_puts(VC0, DS3, YES);
	} else {
		sprintf(bootstr2, "                     ");
		if (motordata[0].active) sprintf(bootstr2, "X Pot%3i O%2i S%2i C%2i               ", motordata[0].pot.pos_actual, motordata[0].pot.offset / 10, motordata[0].pot.span / 10, motordata[0].pot.pos_change);
		LCD_VC_puts(VC0, DS1, YES);
		puts2USART(bootstr2);
		putrs2USART("\r\n");
		sprintf(bootstr2, "                     ");
		if (motordata[1].active) sprintf(bootstr2, "Y Pot%3i O%2i S%2i C%2i               ", motordata[1].pot.pos_actual, motordata[1].pot.offset / 10, motordata[1].pot.span / 10, motordata[1].pot.pos_change);
		LCD_VC_puts(VC0, DS2, YES);
		puts2USART(bootstr2);
		putrs2USART("\r\n");
		sprintf(bootstr2, "                     ");
		if (motordata[2].active) sprintf(bootstr2, "Z Pot%3i O%2i S%2i C%2i               ", motordata[2].pot.pos_actual, motordata[2].pot.offset / 10, motordata[2].pot.span / 10, motordata[2].pot.pos_change);
		LCD_VC_puts(VC0, DS3, YES);
		puts2USART(bootstr2);
		putrs2USART("\r\n");
	}
}

void viision_m_display(void)
{
	if (help_pos == 0) {
		sprintf(bootstr2, "%cX%2i Set%2i V%3li I%2li               ", Cursor[0], motordata[0].pot.scaled_actual / 10, motordata[0].pot.scaled_set / 10, R.pos_x, R.current_x);
		LCD_VC_puts(VC0, DS1, YES);
		sprintf(bootstr2, "%cNO Y or TILT AXIS    ", Cursor[1]); // info display data
		LCD_VC_puts(VC0, DS2, YES);
		sprintf(bootstr2, "%cZ%2i Set%2i V%3li I%2li               ", Cursor[2], motordata[2].pot.scaled_actual / 10, motordata[2].pot.scaled_set / 10, R.pos_z, R.current_z);
		LCD_VC_puts(VC0, DS3, YES);
	} else {
		sprintf(bootstr2, "                     ");
		if (motordata[0].active) sprintf(bootstr2, "X Pot%3i O%2i S%2i C%2i               ", motordata[0].pot.pos_actual, motordata[0].pot.offset / 10, motordata[0].pot.span / 10, motordata[0].pot.pos_change);
		LCD_VC_puts(VC0, DS1, YES);
		puts2USART(bootstr2);
		putrs2USART("\r\n");
		sprintf(bootstr2, "                     ");
		if (motordata[1].active) sprintf(bootstr2, "Y Pot%3i O%2i S%2i C%2i               ", motordata[1].pot.pos_actual, motordata[1].pot.offset / 10, motordata[1].pot.span / 10, motordata[1].pot.pos_change);
		LCD_VC_puts(VC0, DS2, YES);
		puts2USART(bootstr2);
		putrs2USART("\r\n");
		sprintf(bootstr2, "                     ");
		if (motordata[2].active) sprintf(bootstr2, "Z Pot%3i O%2i S%2i C%2i               ", motordata[2].pot.pos_actual, motordata[2].pot.offset / 10, motordata[2].pot.span / 10, motordata[2].pot.pos_change);
		LCD_VC_puts(VC0, DS3, YES);
		puts2USART(bootstr2);
		putrs2USART("\r\n");
	}
}

void viision_ms_display(void)
{
	if (help_pos == 0) {
		sprintf(bootstr2, "                    "); // info display data
		LCD_VC_puts(VC0, DS1, YES);
		sprintf(bootstr2, "%cR%2i Set%2i V%3li I%2li               ", Cursor[1], motordata[1].pot.scaled_actual / 10, motordata[1].pot.scaled_set / 10, R.pos_y, R.current_y);
		LCD_VC_puts(VC0, DS2, YES);
		sprintf(bootstr2, "                    "); // info display data
		LCD_VC_puts(VC0, DS3, YES);
	} else {
		sprintf(bootstr2, "                     ");
		LCD_VC_puts(VC0, DS1, YES);
		sprintf(bootstr2, "                     ");
		if (motordata[1].active) sprintf(bootstr2, "Y Pot%3i O%2i S%2i C%2i               ", motordata[1].pot.pos_actual, motordata[1].pot.offset / 10, motordata[1].pot.span / 10, motordata[1].pot.pos_change);
		LCD_VC_puts(VC0, DS2, YES);
		puts2USART(bootstr2);
		putrs2USART("\r\n");
		sprintf(bootstr2, "                     ");
		LCD_VC_puts(VC0, DS3, YES);
	}
}

void v810_ms_display(void)
{
	if (help_pos == 0) {
		sprintf(bootstr2, "                    "); // info display data
		LCD_VC_puts(VC0, DS1, YES);
		sprintf(bootstr2, "%cR%2i Set%2i V%3li I%2li               ", Cursor[0], motordata[0].pot.scaled_actual / 10, motordata[0].pot.scaled_set / 10, R.pos_x, R.current_x);
		LCD_VC_puts(VC0, DS2, YES);
		sprintf(bootstr2, "                    "); // info display data
		LCD_VC_puts(VC0, DS3, YES);
	} else {
		sprintf(bootstr2, "                     ");
		LCD_VC_puts(VC0, DS1, YES);
		sprintf(bootstr2, "                     ");
		if (motordata[0].active) sprintf(bootstr2, "X Pot%3i O%2i S%2i C%2i               ", motordata[0].pot.pos_actual, motordata[0].pot.offset / 10, motordata[0].pot.span / 10, motordata[0].pot.pos_change);
		LCD_VC_puts(VC0, DS2, YES);
		puts2USART(bootstr2);
		putrs2USART("\r\n");
		sprintf(bootstr2, "                     ");
		LCD_VC_puts(VC0, DS3, YES);
	}
}

void e220_qei_display(void)
{
	if (help_pos == 0) {
		sprintf(bootstr2, "%cR%2li S%2li E%2i               ", Cursor[0], qei1.c, knob2.c, motordata[0].pot.error);
		LCD_VC_puts(VC0, DS1, YES);
		if (motordata[0].run) {
			sprintf(bootstr2, " Motor direction %i                   ", (int16_t) motordata[0].cw); // info display data
			LCD_VC_puts(VC0, DS2, YES);
			sprintf(bootstr2, " Movement %li                   ", qei1.band); // info display data
			LCD_VC_puts(VC0, DS3, YES);
		} else {
			sprintf(bootstr2, "                    "); // info display data
			LCD_VC_puts(VC0, DS2, YES);
			sprintf(bootstr2, "                    "); // info display data
			LCD_VC_puts(VC0, DS3, YES);
		}
	} else {
		sprintf(bootstr2, "                     ");
		if (motordata[0].active) sprintf(bootstr2, "X Pot%2i O%2i S%2i C%2i               ", motordata[0].pot.pos_actual / 10, motordata[0].pot.offset / 10, motordata[0].pot.span / 10, motordata[0].pot.pos_change);
		LCD_VC_puts(VC0, DS1, YES);
		puts2USART(bootstr2);
		putrs2USART("\r\n");
		sprintf(bootstr2, "                     ");
		if (motordata[1].active) sprintf(bootstr2, "Y Pot%2i O%2i S%2i C%2i               ", motordata[1].pot.pos_actual / 10, motordata[1].pot.offset / 10, motordata[1].pot.span / 10, motordata[1].pot.pos_change);
		LCD_VC_puts(VC0, DS2, YES);
		puts2USART(bootstr2);
		putrs2USART("\r\n");
		sprintf(bootstr2, "                     ");
		if (motordata[2].active) sprintf(bootstr2, "Z Pot%2i O%2i S%2i C%2i               ", motordata[2].pot.pos_actual / 10, motordata[2].pot.offset / 10, motordata[2].pot.span / 10, motordata[2].pot.pos_change);
		LCD_VC_puts(VC0, DS3, YES);
		puts2USART(bootstr2);
		putrs2USART("\r\n");
	}
}

void default_display(void)
{
	if (help_pos == 0) {
		sprintf(bootstr2, " Nothing to see here ");
		LCD_VC_puts(VC0, DS1, YES);
		sprintf(bootstr2, " Move                "); // info display data
		LCD_VC_puts(VC0, DS2, YES);
		sprintf(bootstr2, " Along               "); // info display data
		LCD_VC_puts(VC0, DS3, YES);
	} else {
		sprintf(bootstr2, "                     ");
		if (motordata[0].active) sprintf(bootstr2, "X Pot%3i O%2i S%2i C%2i               ", motordata[0].pot.pos_actual, motordata[0].pot.offset / 10, motordata[0].pot.span / 10, motordata[0].pot.pos_change);
		LCD_VC_puts(VC0, DS1, YES);
		puts2USART(bootstr2);
		putrs2USART("\r\n");
		sprintf(bootstr2, "                     ");
		if (motordata[1].active) sprintf(bootstr2, "Y Pot%3i O%2i S%2i C%2i               ", motordata[1].pot.pos_actual, motordata[1].pot.offset / 10, motordata[1].pot.span / 10, motordata[1].pot.pos_change);
		LCD_VC_puts(VC0, DS2, YES);
		puts2USART(bootstr2);
		putrs2USART("\r\n");
		sprintf(bootstr2, "                     ");
		if (motordata[2].active) sprintf(bootstr2, "Z Pot%3i O%2i S%2i C%2i               ", motordata[2].pot.pos_actual, motordata[2].pot.offset / 10, motordata[2].pot.span / 10, motordata[2].pot.pos_change);
		LCD_VC_puts(VC0, DS3, YES);
		puts2USART(bootstr2);
		putrs2USART("\r\n");
	}
}

void emo_display(void)
{ // System has detected a condition that caused it to stop in a save comdition
	sprintf(bootstr2, " EMO EMO EMO EMO      ");
	LCD_VC_puts(VC0, DS1, YES);
	sprintf(bootstr2, " Power Cycle/Reset    "); // info display data
	LCD_VC_puts(VC0, DS2, YES);
	sprintf(bootstr2, " Assy Wiring Short?   "); // info display data
	LCD_VC_puts(VC0, DS3, YES);
}

void varian_v_display(void)
{ // Open SW on Z pot, Closed SW on X POT, Y POT is not used
	if (help_pos == 0) {
		sprintf(bootstr2, "                     "); // info display data
		if (motordata[2].pot.pos_actual < 100) sprintf(bootstr2, " Open  SW =%2i MADE           ", motordata[2].pot.pos_actual / 10);
		LCD_VC_puts(VC0, DS1, YES);
		if (!POWER_Y) {
			sprintf(bootstr2, "%c OPEN  SW %2i               ", Cursor[1], motordata[2].pot.pos_actual / 10);
		} else {
			sprintf(bootstr2, "%c CLOSE SW %2i               ", Cursor[1], motordata[0].pot.pos_actual / 10);
		}
		LCD_VC_puts(VC0, DS2, YES);
		sprintf(bootstr2, "                     "); // info display data
		if (motordata[0].pot.pos_actual < 100) sprintf(bootstr2, " Close SW =%2i MADE          ", motordata[0].pot.pos_actual / 10);
		LCD_VC_puts(VC0, DS3, YES);
	} else {
		sprintf(bootstr2, "                     ");
		if (motordata[0].active) sprintf(bootstr2, "X Pot%2i O%2i S%2i C%2i               ", motordata[0].pot.pos_actual / 10, motordata[0].pot.offset / 10, motordata[0].pot.span / 10, motordata[0].pot.pos_change);
		LCD_VC_puts(VC0, DS1, YES);
		puts2USART(bootstr2);
		putrs2USART("\r\n");
		sprintf(bootstr2, "                     ");
		if (motordata[1].active) sprintf(bootstr2, "Y Pot%2i O%2i S%2i C%2i               ", motordata[1].pot.pos_actual / 10, motordata[1].pot.offset / 10, motordata[1].pot.span / 10, motordata[1].pot.pos_change);
		LCD_VC_puts(VC0, DS2, YES);
		puts2USART(bootstr2);
		putrs2USART("\r\n");
		sprintf(bootstr2, "                     ");
		if (motordata[2].active) sprintf(bootstr2, "Z Pot%2i O%2i S%2i C%2i               ", motordata[2].pot.pos_actual / 10, motordata[2].pot.offset / 10, motordata[2].pot.span / 10, motordata[2].pot.pos_change);
		LCD_VC_puts(VC0, DS3, YES);
		puts2USART(bootstr2);
		putrs2USART("\r\n");
	}
}

void fail_all_motors(uint8_t fail)
{
	int16_t z;
	for (z = 0; z < MAX_MOTOR; z++) { // set or clear the motor cal failed flag
		motordata[z].pot.cal_failed = fail;
	}
}

void set_all_motors(void)
{
	static uint8_t knob_to_pot_save;

	knob_to_pot_save = knob_to_pot;
	knob_to_pot = 0;
	do {
		if (motordata[knob_to_pot].active) {
			track_motor();
		}
	} while (++knob_to_pot < MAX_POT);
	emotor_power_off();
	knob_to_pot = knob_to_pot_save;
	wdttime(RELAYRUNF); // wait for .5 seconds.
}

/* assembly calibration and test routines */
void run_cal(void) // routines to test and set position data for assy motors or valves
{
	uint32_t z, motor_counts = 1500;
	int32_t qei1_tmp = 0;
	int8_t p = 'X';
	int16_t test_counts, STOPPED = FALSE;

	mode.cal = FALSE;
	ADC_read();
	ADC_read();
	mode.change = TRUE;
	button.B2 = 0;
	help_pos = 0;
	init_motordata(mode.operate);
	if (mode.info_only) {
		LED_1 = LOW;
		mode.free = FALSE;
		mode.v24 = FALSE;
		return;
	}
	term_time();
	putrs2USART("\x1b[7m Calibrate/Test Assy(s). \x1b[0m\r\n");
	if (mode.operate == GSD_M) motor_counts = 2000;
	if (mode.operate == VIISION_MS) motor_counts = 4000;
	mode.free = TRUE;
	if (motordata[0].v24 == TRUE) mode.v24 = TRUE;
	emotor_power_off();
	wdttime(RELAYRUNF); // wait for .5 seconds.
	if (mode.operate == VIISION_M) motor_counts *= 1;
	if (mode.operate == VIISION_MS) motor_counts *= 2;
	if (mode.operate == (EV810_MS || V810_MS)) motor_counts *= 2;
	fail_all_motors(FALSE); // clear the failed flag on all motors
	emotor_power_on();
	checktime_cal(motor_counts, TRUE);
	z = 0;

	if (!mode.qei) { // analog and switch cals
		/* valve/ON/CLOSED with switch feedback tests */
		if (mode.operate == VARIAN_V) {
			for (test_counts = 0; test_counts <= 1; test_counts++) {
				motor_counts = 100;
				checktime_cal(motor_counts, TRUE);
				emotor_power_on();
				z = 0;
				do {
					emotor_v24(mode.v24);
					if (z % 2000 == 0) {
						if (button.B2 == 1) {
							emotor_power_off();
							fail_all_motors(TRUE);
							Clear_All_Buttons();
							wdttime(RELAYRUN); // wait for .5 seconds.
							LED_1 = LOW;
							mode.free = TRUE;
							mode.v24 = FALSE;
							emotor_v24(mode.v24);
							init_motordata(mode.operate);
							term_time();
							putrs2USART("\x1b[7m Calibrate/Test Completed, Failed. \x1b[0m\r\n");
							return;
						}
						display_cal();
						sprintf(bootstr2, "Cal: Valve Open %i         ", test_counts); // info display data
						LCD_VC_puts(dsi, DS0, YES);
						sprintf(bootstr2, " Open SW =%2i                             ", motordata[2].pot.pos_actual / 10);
						if (motordata[2].pot.pos_actual < 100) sprintf(bootstr2, " Open SW =%2i  MADE           ", motordata[2].pot.pos_actual / 10);
						LCD_VC_puts(VC0, DS1, YES);
						sprintf(bootstr2, " Valve Testing ON                      ");
						LCD_VC_puts(VC0, DS2, YES);
						sprintf(bootstr2, " Close SW =%2i                              ", motordata[0].pot.pos_actual / 10);
						if (motordata[0].pot.pos_actual < 100) sprintf(bootstr2, " Close SW =%2i MADE          ", motordata[0].pot.pos_actual / 10);
						LCD_VC_puts(VC0, DS3, YES);
					}
					z++;
				} while (!checktime_cal(motor_counts, FALSE));
				if (motordata[2].pot.pos_actual < 100 && motordata[0].pot.pos_actual > 400) motordata[1].pot.cal_high = TRUE;
				if (motordata[0].pot.pos_actual < 100) motordata[1].pot.cal_low = FALSE;
				checktime_cal(motor_counts, TRUE);
				emotor_power_off(); // turn off the power first
				z = 0;
				do {
					emotor_v24(mode.v24);
					if (z % 2000 == 0) {
						if (button.B2 == 1) {
							emotor_power_off();
							fail_all_motors(TRUE);
							Clear_All_Buttons();
							wdttime(RELAYRUN); // wait for .5 seconds.
							LED_1 = LOW;
							mode.free = TRUE;
							mode.v24 = FALSE;
							emotor_v24(mode.v24);
							init_motordata(mode.operate);
							term_time();
							putrs2USART("\x1b[7m Calibrate/Test Completed, Failed. \x1b[0m\r\n");
							return;
						}
						display_cal();
						sprintf(bootstr2, "Cal: Valve Closed %i      ", test_counts); // info display data
						LCD_VC_puts(dsi, DS0, YES);
						sprintf(bootstr2, " Open SW =%2i                             ", motordata[2].pot.pos_actual / 10);
						if (motordata[2].pot.pos_actual < 100) sprintf(bootstr2, " Open SW =%2i  MADE           ", motordata[2].pot.pos_actual / 10);
						LCD_VC_puts(VC0, DS1, YES);
						sprintf(bootstr2, " Valve Testing OFF                        ");
						LCD_VC_puts(VC0, DS2, YES);
						sprintf(bootstr2, " Close SW =%2i                              ", motordata[0].pot.pos_actual / 10);
						if (motordata[0].pot.pos_actual < 100) sprintf(bootstr2, " Close SW =%2i MADE          ", motordata[0].pot.pos_actual / 10);
						LCD_VC_puts(VC0, DS3, YES);
					}
					z++;
				} while (!checktime_cal(motor_counts, FALSE));
				if (motordata[0].pot.pos_actual < 100 && motordata[2].pot.pos_actual > 400) motordata[1].pot.cal_low = TRUE;
				if (motordata[2].pot.pos_actual < 100) motordata[1].pot.cal_high = FALSE;
			}
			LED_1 = LOW;
			mode.free = FALSE;
			mode.v24 = FALSE;
			if (!(motordata[1].pot.cal_low && motordata[1].pot.cal_high)) {
				if (motordata[1].active) {
					button.B2 = FALSE;
					p = 'X' + (int8_t) 1;
					sprintf(bootstr2, "Valve failed calibration %c    ", p); // info display data
					LCD_VC_puts(dsi, DS0, YES);
					puts2USART(bootstr2);
					putrs2USART("\r\n");
					sprintf(bootstr2, "Open %i Close %i      ", motordata[1].pot.cal_high, motordata[1].pot.cal_low);
					LCD_VC_puts(VC0, DS1, YES);
					puts2USART(bootstr2);
					putrs2USART("\r\n");
					sprintf(bootstr2, "                      ");
					if (!motordata[1].pot.cal_high) sprintf(bootstr2, "Open Switch FAILED         ");
					LCD_VC_puts(VC0, DS2, YES);
					puts2USART(bootstr2);
					putrs2USART("\r\n");
					sprintf(bootstr2, "                      ");
					if (!motordata[1].pot.cal_low) sprintf(bootstr2, "Closed Switch FAILED         ");
					LCD_VC_puts(VC0, DS3, YES);
					puts2USART(bootstr2);
					putrs2USART("\r\n");
					blink_led(TEST_LED, TRUE);
					do {
						buzzer_ticks(2);
						ClrWdt();
					} while (button.B2 == FALSE);
					blink_led(TEST_LED, FALSE);
					emotor_power_off();
					fail_all_motors(TRUE);
					Clear_All_Buttons();
					wdttime(RELAYRUN); // wait for .5 seconds.
					LED_1 = LOW;
					mode.free = TRUE;
					mode.v24 = FALSE;
					emotor_v24(mode.v24);
					init_motordata(mode.operate);
				}
			}
			term_time();
			putrs2USART("\x1b[7m Calibrate/Test Completed. \x1b[0m\r\n");
			return;
		} // end of valve calibration tests

		/* normal motor tests */
		emotor_power_off(); // turn off the power first
		wdttime(RELAYRUN); // wait for .5 seconds.
		emotor_switch_ccw(); // switch to CCW
		wdttime(RELAYRUNF); // wait for .5 seconds.
		STOPPED = FALSE;
		emotor_power_on();
		checktime_cal(motor_counts, TRUE);
		if (mode.operate == VIISION_MS) {
			emotor_v24(mode.v24);
			wdttime(20); // wait for a long time
		}
		Reset_Change_Count();
		do {
			emotor_v24(mode.v24);
			if (z % 1000 == 0) {
				if (Change_Count()) {
					term_time();
					sprintf(bootstr2, " X Y Z change %li,%li,%li \r\n", ABSL(R.pos_x - R.change_x), ABSL(R.pos_y - R.change_y), ABSL(R.pos_z - R.change_z));
					puts2USART(bootstr2);
					if (R.stable_x && R.stable_y && R.stable_z) {
						if (mode.operate != VIISION_M) STOPPED = TRUE;
						term_time();
						sprintf(bootstr2, " NO ADC VOLTAGE CHANGE DETECTED \r\n\r\n");
						puts2USART(bootstr2);
					}
					Reset_Change_Count();
				}
				if (button.B2 == 1) {
					emotor_power_off();
					fail_all_motors(TRUE);
					Clear_All_Buttons();
					wdttime(RELAYRUN); // wait for .5 seconds.
					LED_1 = LOW;
					mode.free = TRUE;
					mode.v24 = FALSE;
					emotor_v24(mode.v24);
					init_motordata(mode.operate);
					term_time();
					putrs2USART("\x1b[7m Calibrate/Test Completed, Failed. \x1b[0m\r\n");
					return;
				}
				display_cal();
				sprintf(bootstr2, "Calibrate CCW %lu      ", z); // info display data
				LCD_VC_puts(dsi, DS0, YES);
				if (mode.operate != GSD_M) {
					sprintf(bootstr2, "                     ");
					if (motordata[0].active) sprintf(bootstr2, "X Pot%3i D%2i S%2i I%2li               ", motordata[0].pot.pos_actual, motordata[0].pot.scaled_actual / 10, motordata[0].pot.span / 10, R.current_x);
				} else {
					if (motordata[0].active) sprintf(bootstr2, "A Pot%3i D%2i S%2i I%2li               ", motordata[0].pot.pos_actual, motordata[0].pot.scaled_actual / 10, motordata[0].pot.span / 10, R.current_x);
				}
				LCD_VC_puts(VC0, DS1, YES);
				sprintf(bootstr2, "                     ");
				if (motordata[1].active) sprintf(bootstr2, "Y Pot%3i D%2i S%2i I%2li               ", motordata[1].pot.pos_actual, motordata[1].pot.scaled_actual / 10, motordata[1].pot.span / 10, R.current_y);
				LCD_VC_puts(VC0, DS2, YES);
				sprintf(bootstr2, "                     ");
				if (motordata[2].active) sprintf(bootstr2, "Z Pot%3i D%2i S%2i I%2li               ", motordata[2].pot.pos_actual, motordata[2].pot.scaled_actual / 10, motordata[2].pot.span / 10, R.current_z);
				LCD_VC_puts(VC0, DS3, YES);
			}
			z++;
		} while (!checktime_cal(motor_counts, FALSE) && !STOPPED);

		emotor_power_off(); // turn off the power first
		wdttime(RELAYRUN); // wait for .5 seconds.
		emotor_switch_cw(); // switch to CW
		wdttime(RELAYRUNF); // wait for .5 seconds.
		STOPPED = FALSE;

		if (mode.operate != VIISION_M) {
			emotor_power_on();
			checktime_cal(motor_counts, TRUE);
			z = 0;
			if (mode.operate == VIISION_MS) {
				emotor_v24(mode.v24);
				wdttime(20); // wait for a long time
			}
			Reset_Change_Count();
			do {
				emotor_v24(mode.v24);
				if (z % 1000 == 0) {
					if (Change_Count()) {
						term_time();
						sprintf(bootstr2, " X Y Z change %li,%li,%li \r\n", ABSL(R.pos_x - R.change_x), ABSL(R.pos_y - R.change_y), ABSL(R.pos_z - R.change_z));
						puts2USART(bootstr2);
						if (R.stable_x && R.stable_y && R.stable_z) {
							if (mode.operate != VIISION_M) STOPPED = TRUE;
							term_time();
							sprintf(bootstr2, " NO ADC VOLTAGE CHANGE DETECTED \r\n\r\n");
							puts2USART(bootstr2);
						}
						Reset_Change_Count();
					}
					if (button.B2 == 1) {
						emotor_power_off();
						fail_all_motors(TRUE);
						Clear_All_Buttons();
						wdttime(RELAYRUN); // wait for .5 seconds.
						LED_1 = LOW;
						mode.free = TRUE;
						mode.v24 = FALSE;
						emotor_v24(mode.v24);
						init_motordata(mode.operate);
						term_time();
						putrs2USART("\x1b[7m Calibrate/Test Completed, Failed. \x1b[0m\r\n");
						return;
					}
					display_cal();
					sprintf(bootstr2, "Calibrate CW %lu      ", z); // info display data
					LCD_VC_puts(dsi, DS0, YES);
					sprintf(bootstr2, "                     ");
					if (mode.operate != GSD_M) {
						if (motordata[0].active) sprintf(bootstr2, "X Pot%3i D%2i S%2i I%2li               ", motordata[0].pot.pos_actual, motordata[0].pot.scaled_actual / 10, motordata[0].pot.span / 10, R.current_x);
					} else {
						if (motordata[0].active) sprintf(bootstr2, "A Pot%3i D%2i S%2i I%2li               ", motordata[0].pot.pos_actual, motordata[0].pot.scaled_actual / 10, motordata[0].pot.span / 10, R.current_x);
					}
					LCD_VC_puts(VC0, DS1, YES);
					sprintf(bootstr2, "                     ");
					if (motordata[1].active) sprintf(bootstr2, "Y Pot%3i D%2i S%2i I%2li               ", motordata[1].pot.pos_actual, motordata[1].pot.scaled_actual / 10, motordata[1].pot.span / 10, R.current_y);
					LCD_VC_puts(VC0, DS2, YES);
					sprintf(bootstr2, "                     ");
					if (motordata[2].active) sprintf(bootstr2, "Z Pot%3i D%2i S%2i I%2li               ", motordata[2].pot.pos_actual, motordata[2].pot.scaled_actual / 10, motordata[2].pot.span / 10, R.current_z);
					LCD_VC_puts(VC0, DS3, YES);
				}
				z++;
			} while (!checktime_cal(motor_counts, FALSE) && !STOPPED);
		}
	} else { // qei mode cals
		qei1.c = 0;
		qei1_tmp = qei1.c;
		emotor_power_on();
		wdttime(20); // wait
		for (z = 0; z < 990; z++) {
			sprintf(bootstr2, "Calibrate CW %lu      ", z); // info display data
			LCD_VC_puts(dsi, DS0, YES);
			sprintf(bootstr2, "                     ");
			if (motordata[0].active) sprintf(bootstr2, " Motor counts %2li               ", qei1.c);
			LCD_VC_puts(VC0, DS1, YES);
			sprintf(bootstr2, "                     ");
			LCD_VC_puts(VC0, DS2, YES);
			LCD_VC_puts(VC0, DS3, YES);
			wdttime(1); // wait
			if (z % 25 == 0) {
				if (qei1_tmp == qei1.c) z = 1000;
				qei1_tmp = qei1.c;
			}
			if (button.B2) {
				emotor_power_off();
				fail_all_motors(TRUE);
				Clear_All_Buttons();
				wdttime(RELAYRUN); // wait for .5 seconds.
				LED_1 = LOW;
				mode.free = TRUE;
				mode.v24 = FALSE;
				emotor_v24(mode.v24);
				init_motordata(mode.operate);
				term_time();
				putrs2USART("\x1b[7m Calibrate/Test Completed, Failed. \x1b[0m\r\n");
				return;
			}
		}
		emotor_power_off(); // turn off the power first
		wdttime(RELAYRUN); // wait for .5 seconds.
		emotor_switch_ccw(); // switch to CCW
		wdttime(RELAYRUNF); // wait for .5 seconds.
		qei1.c = 0;
		qei1_tmp = qei1.c;
		emotor_power_on();
		wdttime(20); // wait
		for (z = 0; z < 990; z++) {
			sprintf(bootstr2, "Calibrate CCW %lu      ", z); // info display data
			LCD_VC_puts(dsi, DS0, YES);
			sprintf(bootstr2, "                     ");
			if (motordata[0].active) sprintf(bootstr2, " Motor counts %2li               ", qei1.c);
			LCD_VC_puts(VC0, DS1, YES);
			sprintf(bootstr2, "                     ");
			LCD_VC_puts(VC0, DS2, YES);
			LCD_VC_puts(VC0, DS3, YES);
			wdttime(1); // wait
			if (z % 25 == 0) {
				if (qei1_tmp == qei1.c) z = 1000;
				qei1_tmp = qei1.c;
			}
			if (button.B2) {
				emotor_power_off();
				fail_all_motors(TRUE);
				Clear_All_Buttons();
				wdttime(RELAYRUN); // wait for .5 seconds.
				LED_1 = LOW;
				mode.free = TRUE;
				mode.v24 = FALSE;
				emotor_v24(mode.v24);
				init_motordata(mode.operate);
				term_time();
				putrs2USART("\x1b[7m Calibrate/Test Completed, Failed. \x1b[0m\r\n");
				return;
			}
		}
		qei1.max = qei1.c;
		emotor_power_off(); // turn off the power first
		mode.free = FALSE; // setup flags so the tracking can work.
		mode.v24 = FALSE;
		knob2.c = (int32_t) ((float) qei1.max * 0.01);
		track_motor();
		knob2.c = (int32_t) ((float) qei1.max * 0.666);
		motordata[knob_to_pot].pot.error = knob2.c - qei1.c; // find the raw encoder/encoder diff
		track_motor();
		knob2.c = (int32_t) ((float) qei1.max * 0.333);
		motordata[knob_to_pot].pot.error = knob2.c - qei1.c; // find the raw encoder/encoder diff
		track_motor();
		knob2.c = (int32_t) ((float) qei1.max * 0.9);
		motordata[knob_to_pot].pot.error = knob2.c - qei1.c; // find the raw encoder/encoder diff
		track_motor();
		knob2.c = (int32_t) ((float) qei1.max * 0.1);
		motordata[knob_to_pot].pot.error = knob2.c - qei1.c; // find the raw encoder/encoder diff
		track_motor();
		knob2.c = (int32_t) ((float) qei1.max * 0.5);
		motordata[knob_to_pot].pot.error = knob2.c - qei1.c; // find the raw encoder/encoder diff
		track_motor();
		for (z = 0; z < MAX_MOTOR; z++) { // set defaults to pass
			motordata[z].pot.cal_failed = FALSE;
		}
	}

	for (z = 0; z < MAX_MOTOR; z++) { // Lets look at the results of the calibration.

		if (motordata[z].active) { // check for failing results
			if ((motordata[z].pot.pos_change > motordata[z].pot.limit_change)) motordata[z].pot.cal_failed = TRUE;
			if ((motordata[z].pot.span < motordata[z].pot.limit_span)) motordata[z].pot.cal_failed = TRUE;
			if (motordata[z].pot.offset > motordata[z].pot.limit_offset_h) {
				motordata[z].pot.cal_failed = TRUE;
			}
			if (motordata[z].pot.offset < motordata[z].pot.limit_offset_l) {
				motordata[z].pot.cal_warn = TRUE;
			}
		}

		if (!motordata[z].pot.cal_failed) {
			motordata[z].pot.cal_low = TRUE;
			motordata[z].pot.cal_high = TRUE;
			motordata[z].pot.scaled_set = motordata[z].cal_pos; // move to install position
			if (motordata[z].active) {
				if (mode.operate != GSD_M) {
					p = 'X' + (int8_t) z;
				} else {
					p = 'A';
				}
				term_time();
				if (!motordata[z].pot.cal_warn) {
					sprintf(bootstr2, "\x1b[7m Calibrate/Test motor %c PASSED. \x1b[0m\r\n", p);
				} else {
					sprintf(bootstr2, "\x1b[7m Calibrate/Test motor %c PASSED with WARNING. \x1b[0m\r\n", p);
				}
				puts2USART(bootstr2);
				sprintf(bootstr2, " If Dead %i < %i      ", motordata[z].pot.pos_change, motordata[z].pot.limit_change);
				puts2USART(bootstr2);
				putrs2USART("\r\n");
				sprintf(bootstr2, " If Span %i > %i      ", motordata[z].pot.span, motordata[z].pot.limit_span);
				puts2USART(bootstr2);
				putrs2USART("\r\n");
				sprintf(bootstr2, " Offset %i< %i >%i    ", motordata[z].pot.limit_offset_h, motordata[z].pot.offset, motordata[z].pot.limit_offset_l);
				puts2USART(bootstr2);
				putrs2USART("\r\n");
			}
		} else {
			if (motordata[z].active) {
				button.B2 = FALSE;
				if (mode.operate != GSD_M) {
					p = 'X' + (int8_t) z;
				} else {
					p = 'A';
				}
				term_time();
				putrs2USART(" ");
				sprintf(bootstr2, "Motor %c FAILED cal  ", p); // info display data
				LCD_VC_puts(dsi, DS0, YES);
				puts2USART(bootstr2);
				putrs2USART("\r\n");
				sprintf(bootstr2, " If Dead %i > %i      ", motordata[z].pot.pos_change, motordata[z].pot.limit_change);
				LCD_VC_puts(VC0, DS1, YES);
				puts2USART(bootstr2);
				putrs2USART("\r\n");
				sprintf(bootstr2, " If Span %i < %i      ", motordata[z].pot.span, motordata[z].pot.limit_span);
				LCD_VC_puts(VC0, DS2, YES);
				puts2USART(bootstr2);
				putrs2USART("\r\n");
				sprintf(bootstr2, " Offset %i< %i >%i    ", motordata[z].pot.limit_offset_h, motordata[z].pot.offset, motordata[z].pot.limit_offset_l);
				LCD_VC_puts(VC0, DS3, YES);
				puts2USART(bootstr2);
				putrs2USART("\r\n");
				blink_led(TEST_LED, TRUE);
				if (mode.operate != GSD_M) {
					p = 'X' + (int8_t) z;
				} else {
					p = 'A';
				}
				term_time();
				sprintf(bootstr2, "\x1b[7m Calibrate/Test motor %c FAILED. \x1b[0m\r\n", p);
				puts2USART(bootstr2);
				do {
					buzzer_ticks(2);
					ClrWdt();
				} while (button.B2 == FALSE);
				blink_led(TEST_LED, FALSE);
				Clear_All_Buttons();
			}
		}
	}
	emotor_power_off(); // turn off the power first
	wdttime(RELAYRUN); // wait for .5 seconds.
	LED_1 = LOW;
	mode.free = FALSE;
	mode.v24 = FALSE;
	Clear_All_Buttons();
	if (!mode.qei) {
		z = 0;
		buzzer_ticks(5);
		do {
			sprintf(bootstr2, "Press Cal for EXER  "); // info display data
			LCD_VC_puts(dsi, DS0, YES);
			if (button.B2 == TRUE) {
				ALARMOUT = HIGH;
				term_time();
				sprintf(bootstr2, "  EXER running             \r\n"); // info display data
				puts2USART(bootstr2);
				LCD_VC_puts(dsi, DS0, YES);
				wdttime(RELAYRUN); // wait for .5 seconds.
				Clear_All_Buttons();
				sprintf(bootstr2, "                     ");
				LCD_VC_puts(VC0, DS1, YES);
				sprintf(bootstr2, "                     ");
				LCD_VC_puts(VC0, DS2, YES);
				sprintf(bootstr2, "                     ");
				LCD_VC_puts(VC0, DS3, YES);	
				set_all_motors();
				nonqei_exer(2);
				term_time();
				sprintf(bootstr2, "  EXER completed.         \r\n"); // info display data
				puts2USART(bootstr2);
				LCD_VC_puts(dsi, DS0, YES);
				wdttime(RELAYRUN); // wait for .5 seconds.
				z = 100000;
			}
			ClrWdt();
		} while ((button.B2 == FALSE) && (z++ < 1000));
		sprintf(bootstr2, " Completed Calibration       ");
		LCD_VC_puts(VC0, DS0, YES);
		sprintf(bootstr2, "                     ");
		LCD_VC_puts(VC0, DS1, YES);
		sprintf(bootstr2, "                     ");
		LCD_VC_puts(VC0, DS2, YES);
		sprintf(bootstr2, "                     ");
		LCD_VC_puts(VC0, DS3, YES);
	} else {
	}
	mode.free = FALSE;
	mode.v24 = FALSE;
	wdttime(RELAYRUN); // wait for .5 seconds.
	Clear_All_Buttons();
	set_all_motors();
	term_time();
	putrs2USART("\x1b[7m Calibrate/Test Completed. \x1b[0m\r\n");
}

void display_cal(void)
{
	ADC_read();
	if (RS232_DEBUG) {
		sprintf(bootstr2, "Current: X%3li Y%3li Z%3li Raw: ADC X%3i Y%3i Z%3i\r\n", R.current_x, R.current_y, R.current_z, a10_x, a10_y, a10_z);
		puts2USART(bootstr2);
		sprintf(bootstr2, "Position ADC:  X%3li Y%3li Z%3li\r\n", R.pos_x, R.pos_y, R.pos_z);
		puts2USART(bootstr2);
		sprintf(bootstr2, "Pot: Xa%3i Xs%3i Xc%3i Ya%3i Ys%3i Yc%3i Za%3i Zs%3i Zc%3i\r\n\n",
			motordata[0].pot.pos_actual, motordata[0].pot.pos_set, motordata[0].pot.pos_change,
			motordata[1].pot.pos_actual, motordata[1].pot.pos_set, motordata[1].pot.pos_change,
			motordata[2].pot.pos_actual, motordata[2].pot.pos_set, motordata[2].pot.pos_change);
		puts2USART(bootstr2);
	}
}

void init_lcd(void)
{
	lcd18 = 200;
	wdtdelay(10000); // delay for power related LCD setup glitch
	if (BusyXLCD()) {
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
}

void main(void) // Lets Party
{
	static uint8_t eep_char = NULL0;
	static uint8_t z = 0;
	static uint8_t menu_pos;

	mode.v24 = FALSE;

#ifdef	__18F8722
	config_pic(PIC_8722); // configure all controller hardware to the correct settings and ports
#endif

	sprintf(bootstr2, "********************"); // blank display data
	LCD_VC_puts(VC0, DS0, YES);
	LCD_VC_puts(VC0, DS1, YES);
	LCD_VC_puts(VC0, DS2, YES);
	LCD_VC_puts(VC0, DS3, YES);

#ifdef	__18F8722
	start_pic(PIC_8722); // configure external hardware to the correct startup conditions
#endif
	ALARMOUT = LOW;
	init_lcd();
	LEDS = R_ALL_ON;

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

	strncpypgm2ram(bootstr2, START1, LCD_W);
	LCD_VC_puts(VC0, DS0, YES);

	putrs2USART(" FW ");
	strncpypgm2ram(bootstr2, build_time, LCD_W);
	LCD_VC_puts(VC0, DS1, YES);
	puts2USART(bootstr2);
	putrs2USART(" ");

	strncpypgm2ram(bootstr2, build_date, LCD_W);
	puts2USART(bootstr2);
	LCD_VC_puts(VC0, DS2, YES);

	strncpypgm2ram(bootstr2, MANDM_VERSION, LCD_W);
	puts2USART(bootstr2);
	putrs2USART("\r\n");
	LCD_VC_puts(VC0, DS3, YES);

	// leds from outputs to ground via resistor.


	term_time();
	putrs2USART(" Configure operational data \r\n");

	if (DIPSW2 == HIGH) { // enable RS232 extra debug
		RS232_DEBUG = TRUE;
		putrs2USART(" RS232 DEBUG Enabled \r\n");
	}

	if (DIPSW1 == HIGH) { // clear data
		term_time();
		putrs2USART(" Init Controller Data \r\n");
		write_data_eeprom(0, 1, 0, 0); // data, number of data slots, postion, offset from eeprom address 0

		term_time();
		putrs2USART(" Init ADC Cal EEPROM Data \r\n");
		for (z = 0; z < ADC_SLOTS; z++) {
			write_data_eeprom(ADC_NULL, ADC_SLOTS, z, 8); // data, number of data slots, postion, offset from eeprom address 0
		}
	}

	term_time();
	putrs2USART(" Read Controller eeprom data \r\n");
	if (read_data_eeprom(0, 0) == CHECKMARK) { // load Controller types from EEPROM
		eep_char = read_data_eeprom(2, 0);
	} else {
		term_time();
		putrs2USART(" Invalid Controller eeprom data \r\n"); // init eeprom data
		write_data_eeprom(0, 1, 0, 0); // data, number of data slots, postion, offset from eeprom address 0
		term_time();
		putrs2USART(" Controller eeprom data saved\r\n"); // init eeprom data
	}

	term_time();
	putrs2USART(" Read ADC eeprom data \r\n");
	if (read_data_eeprom(0, 8) == CHECKMARK) { // load ADC cal data from EEPROM
		for (z = 0; z < ADC_SLOTS; z++) {
			adc_cal[z] = read_data_eeprom(10, z);
		}
	} else {
		term_time();
		putrs2USART(" Invalid ADC eeprom data \r\n"); // init eeprom data
		for (z = 0; z < ADC_SLOTS; z++) {
			write_data_eeprom(adc_cal[z], ADC_SLOTS, z, 8);
		}
		term_time();
		putrs2USART(" ADC eeprom data saved\r\n"); // init eeprom data
	}

	if (DIPSW3 == HIGH) { // failsafe mode
		fail_safe();
		while (TRUE); // Don't  continue
	}

	zero_amploc(); // zero input current sensor
	term_time();
	putrs2USART(" Read ADC data inputs \r\n");
	wdttime(BATRUNF);
	ADC_read();

	if (DIPSW1 == HIGH) { // set default hist and SD data
		term_time();
		putrs2USART(" Init History Data \r\n");
	}

	if (DIPSW4 == HIGH) { // Set select to 8 bits
		term_time();
		putrs2USART(" Use 4 bit selection codes\r\n");
		cable_type = 0x0f;
	}

	voltfps(R.systemvoltage, f1);
	voltfps(R.motorvoltage, f2);
	sprintf(bootstr2, "Sys %sV,Mot %sV         ", f1, f2); // display Power info
	LCD_VC_puts(VC0, DS0, YES);

	sprintf(bootstr2, " DIPSW %i%i%i%i%i%i%i%i LSB           \r\n", DIPSW8, DIPSW7,
		DIPSW6, DIPSW5, DIPSW4, DIPSW3, DIPSW2, DIPSW1);
	LCD_VC_puts(VC0, DS1, YES);
	term_time();
	puts2USART(bootstr2);

	start_delay();
	INTCONbits.GIEH = LOW;
	INTCONbits.GIEL = LOW;
	while (!PB2);
	INTCONbits.GIEH = HIGH;
	INTCONbits.GIEL = HIGH;

	wdttime(BATRUNF); // read battery and charging system voltages
	ADC_read();
	srand((uint16_t) R.systemvoltage); // set random seed

	/*      Work thread start */
	start_workerthread();
	LEDS = R_ALL_OFF;

	mode.locked = FALSE;
	mode.change = FALSE;
	mode.move = FALSE;
	check_cable((uint8_t*) mode.operate);
	init_motordata(mode.operate);

	SYSTEM_STABLE = TRUE;
	voltfps(R.systemvoltage, f1);
	voltfps(R.motorvoltage, f2);
	sprintf(bootstr2, " S%sV,M%sV,Q%5li %5li         \r\n", f1, f2, knob1.c, knob2.c);
	LCD_VC_puts(VC0, DS0, YES);
	term_time();
	puts2USART(bootstr2);
	start_delay();
	voice1_ticks(40);
	wdttime(BATRUNF);
	Set_Cursor();
	checktime_eep(1, TRUE);

	/* Loop forever */
	while (TRUE) {
		ADC_read();
		if (emotor_is_all_off() && WORKERFLAG) zero_amploc();
		hid_menu();
		ClrWdt(); // reset the WDT timer
		if (mode.cal) {
			run_cal();
			menu_pos = get_hid();
			check_cable(&menu_pos);
			update_lcd_menu(menu_pos);
		}
		ClrWdt(); // reset the WDT timer
		update_hist();
		track_motor();
		if (mode.idle) {
			help_pos = 0;
		}
	}
}

