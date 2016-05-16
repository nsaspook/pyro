
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

#define	__PYRO_C			//	This is the main program

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
 * V: structure, Volatile variables modified in the ISR in a possible non-atomic fashion
 * A: structure ADC buffer data
 * S: structure SPI buffer data 
 * D: structure LCD buffer data
 *
 * USART2 		is the host comm port 38400
 * Timer0		1 second clock
 * TImer1		Not used
 * Timer2		Not used
 * Timer3		work thread , background I/O clock ~20HZ
 * TImer4		State machine Period clock ~1khz

 * 0..11 analog channels are active
 * PORTA		analog inputs
 * PORTB		HID Qencoder and switch inputs
 * PORTC		SPI master and load, select outputs
 * PORTD		configuration switch input
 * PORTE		relays
 * PORTF		analog inputs

 * adc8 Ground REF	zero adc charge cap
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
 */

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

#pragma udata gpr13
far int8_t bootstr2[MESG_W + 1], f1[MESG_W], f2[MESG_W];
#pragma udata gpr1
volatile struct ringBufS_t ring_buf3;
uint8_t HCRIT[CRIT_8], LCRIT[CRIT_8];
float smooth[LPCHANC];
#pragma udata gpr2
volatile struct L_data L;
volatile struct spi_link_type spi_link = {0};
union mcp4822_buf_type mcp4822;
volatile struct ringBufS_t ring_buf5;
#pragma udata gpr3
volatile struct ringBufS_t ring_buf6;
far int8_t hms_string[16], f3[MESG_W];
volatile uint8_t critc_level = 0;
volatile uint8_t TIMERFLAG = FALSE, SYSTEM_STABLE = FALSE, D_UPDATE = FALSE, WDT_TO = FALSE, EEP_ER = FALSE;
#pragma udata gpr4
volatile uint32_t critc_count = 0;
volatile struct ringBufS_t ring_buf4;
#pragma udata gpr5
int8_t sign = ' ';
float lp_speed = 0.0, lp_x = 0.0;
struct V_data V = {0};
const rom int8_t *build_date = __DATE__, *build_time = __TIME__;
#pragma udata gpr6
float t1 = 0.0, t2 = 0.0, t3 = 0.0, t4 = 0.0, t5 = 0.0, t6 = 0.0, t7 = 0.0, t_time = 0.0;
float voltfrak = 0.0;
float ahfrak = 0.0;
#pragma udata gpr7
volatile uint8_t IDLEFLAG = FALSE, knob_to_pot = 0;
int32_t iw = 0, ip = 0;
#pragma udata gpr8
volatile struct ringBufS_t ring_buf1;
#pragma udata gpr9
volatile struct ringBufS_t ring_buf2;
volatile struct knobtype knob1, knob2;

/* ADC voltage default calibration values  */
uint8_t adc_cal[] = {128, 128, 128, 128, 128, 128, 128, 128, 128, 127, 127, 127, 127, 127, 127, 0};
uint8_t CRITC = 0, LCD_OK = FALSE;

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
int8_t spinners(uint8_t shape, uint8_t reset)
{
	static uint8_t s[MAX_SHAPES], last_shape = 0;
	int8_t c;

	if (shape > (MAX_SHAPES - 1)) shape = 0;
	if (reset) s[shape] = 0;
	last_shape = shape;
	c = spin[shape][s[shape]];
	if (++s[shape] >= strlenpgm(spin[shape])) s[shape] = 0;
	return c;
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
	wdtdelay(50000); // delay for power related LCD setup glitch
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
}

/*
 * bit set/reset
 */
void bitmap_e(uint8_t slot, uint8_t state)
{
	if (state)
		LATE |= 0x01 << slot;
	else
		LATE &= ~(0x01 << slot);
}

/*
 * LEDS are ON with zero, ON=0, OFF=1
 */
void fade_up_leds(void)
{
	uint8_t numbers[] = {0, 1, 3, 7, 15, 31, 63, 127, 255}, l;
	int16_t i, j, k;


	for (l = 0; l <= 7; l++) {
		k = 200;
		j = 0;
		while (j < 300 && k > 0) {
			for (i = 0; i < j; i++) {
				bitmap_e(l, ON);
				wdtdelay(2); // short delay function
			}
			for (i = 0; i < k; i++) {
				bitmap_e(l, OFF);
				wdtdelay(2);
			}
			j++;
			k--;
		}
		LATE = ~numbers[l + 1]; // flip the bits for my setup
	}
}

void main(void) // Lets Party
{

	static uint32_t z;
	static union adc_buf_type adc_buf;
	uint16_t dac1, dac2 = 5000;
	uint32_t dtime1, dtime2;


#ifdef	__18F8722
	config_pic(PIC_8722); // configure all controller hardware to the correct settings and ports
#endif

#ifdef	__18F8722
	start_pic(PIC_8722); // configure external hardware to the correct startup conditions
#endif

	init_lcd();
	fade_up_leds();

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

	ringBufS_init(L.rx1b); // adc rx buffer
	ringBufS_init(L.tx1b); // lcd tx buffer
	ringBufS_init(L.rx2b); // rs232 rx buffer
	ringBufS_init(L.tx2b); // rs232 tx buffer
	ringBufS_init(spi_link.tx1b); // spi tx buffer
	ringBufS_init(spi_link.rx1b); // spi rx buffer

	/*      Work thread start */
	start_workerthread();

	SYSTEM_STABLE = TRUE;
	srand((uint16_t) 1957); // set random seed
	putrsXLCD("Pyro MFC Control");

	dtime1 = V.clock20;
	dtime2 = V.clock20;
	/* Loop forever */
	while (TRUE) {
		ClrWdt(); // reset the WDT timer
		if (ringBufS_empty(L.rx1b)) {
			if (V.clock20 > dtime1 + 10) { // ~5hz display update freq
				voltfp(L.adc_val[0], f1); // analog control input 0
				voltfp(L.adc_val[2], f2); // mfc readback 0
				voltfp(L.adc_val[3], f3); // mfc setpoint 0
				sprintf(bootstr2, "%sV %sV %sV                        ", f1, f3, f2);
				bootstr2[20] = NULL0; // limit the string to 20 chars
				DLED_4 = HIGH;
				S_WriteCmdXLCD(0b10000000 | LL2); // SetDDRamAddr
				putsXLCD(bootstr2);
				sprintf(bootstr2, "%u %u %u %d                          ", L.adc_raw[0], L.adc_raw[3], L.adc_raw[2], (int16_t) (L.adc_raw[3] - L.adc_raw[2]));
				bootstr2[20] = NULL0; // limit the string to 20 chars
				S_WriteCmdXLCD(0b10000000 | LL3); // SetDDRamAddr
				putsXLCD(bootstr2);
				//sprintf(f1, "SPI int  %lu %c                          ", spi_link.count, spinners(5, 0));
				//f1[20] = NULL0; // limit the string to 20 chars
				//S_WriteCmdXLCD(0b10000000 | LL3); // SetDDRamAddr
				//putsXLCD(f1);
				//sprintf(f1, "LCD char %lu %lu                         ", V.lcd_count, z);
				//f1[20] = NULL0; // limit the string to 20 chars
				//S_WriteCmdXLCD(0b10000000 | LL4); // SetDDRamAddr
				//putsXLCD(f1);
				DLED_4 = LOW;
				dtime1 = V.clock20;
			}
		} else {
			z = 0;
			while (!ringBufS_empty(L.rx1b)) {
				adc_buf.buf = ringBufS_get(L.rx1b); // get the analog voltages			
				ADC_Update(adc_buf.buf & ADC_MASK, adc_buf.map.index);
				z++;
			}


			if (V.clock20 > dtime2 + 1) { // ~10hz DAC update rate
				dac1 += 3;
				dac2 -= 3;
				DLED_2 = HIGH;
				// do something
				if (SPI_Out_Update(dac1, 0, 0)) DLED_6 = LOW;
				if (SPI_Out_Update(dac2, 0, 1)) DLED_6 = LOW;
				if (SPI_Out_Update(rand(), 1, 0)) DLED_6 = LOW;
				if (SPI_Out_Update(rand(), 1, 1)) DLED_6 = LOW;
				if (SPI_Out_Update(0b00000000, 2, 0)) DLED_6 = LOW;

				dtime2 = V.clock20;
				DLED_2 = LOW;
			}
		}

		if (SSPCON1bits.WCOL || SSPCON1bits.SSPOV) { // check for overruns/collisions
			SSPCON1bits.WCOL = (SSPCON1bits.SSPOV = LOW);
			ringBufS_flush(spi_link.tx1b, 1); // dump the spi buffers
			ringBufS_flush(spi_link.rx1b, 1);
		}
	}
}

