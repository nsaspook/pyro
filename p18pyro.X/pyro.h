/* 
 * File:   pyro.h
 * Author: root
 *
 * Created on May 2, 2016, 3:35 PM
 */

#ifndef PYRO_H
#define	PYRO_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "pyro_defs.h"
#include <GenericTypeDefs.h>
#include  "ringbufs.h"

#ifdef INTTYPES
#include <stdint.h>
#else
#define INTTYPES
	/*unsigned types*/
	typedef unsigned char uint8_t;
	typedef unsigned int uint16_t;
	typedef unsigned long uint32_t;
	typedef unsigned long long uint64_t;
	/*signed types*/
	typedef signed char int8_t;
	typedef signed int int16_t;
	typedef signed long int32_t;
	typedef signed long long int64_t;
#endif

	struct QuadEncoderType {
		int16_t unused : 4; // RB[0:3]
		int16_t A1 : 1; // RB4
		int16_t B1 : 1; // RB5
		int16_t A2 : 1; // RB6
		int16_t B2 : 1; // RB7
		uint8_t OldPortB[2], byTemp[2];
	};

	struct buttontype {
		int16_t B0 : 1; // RB0
		int16_t B1 : 1; // RB1
		int16_t B2 : 1; // RB2
		int16_t B3 : 1; // RB3
		int16_t unused : 4; // RB[4:7]
	};

	struct blinktype {
		int16_t B0 : 1; // LED0
		int16_t B1 : 1; // LED1
		int16_t B2 : 1; // LED2
		int16_t B3 : 1; // LED3
		int16_t S0 : 1; // LED0 saved
		int16_t S1 : 1; // LED1 saved
		int16_t S2 : 1; // LED2 saved
		int16_t S3 : 1; // LED3 saved
	};

	typedef struct pottype {

		enum movement_t {
			CW, STOP, CCW
		} movement;

		int16_t pos_actual, pos_set, error, pos_actual_prev, pos_change; // in ADC counts
		int16_t limit_change, limit_span, limit_offset, limit_offset_l, limit_offset_h; // AXIS limits for error checking
		int16_t low, high, offset, span, cal_low, cal_high, cal_failed, cal_warn; // end of travel ADC count values
		float scale_out, scale_in; // scaling factor from actual to scaled and back
		int16_t scaled_actual, scaled_set, scaled_error; // 0..1023 value of pot for LCD readback
	} volatile pottype;

	typedef struct motortype {
		uint8_t type, run, cw, axis, free, slow, active, reversed, v24, slow_only, on_off_only;
		int16_t hunt_count, cal_pos;
		struct pottype pot;
	} volatile motortype;

	typedef struct knobtype {
		int32_t c, last_c, band; // count
		uint8_t cw, ccw, ticks; // direction

		enum movement_t {
			CW, STOP, CCW
		} movement;
	} volatile knobtype;

	typedef struct qeitype {
		int32_t c, last_c, band, max; // count
		uint8_t cw, ccw, ticks, home, back_stop, forward_stop; // directions

		enum movement_t {
			CW, STOP, CCW
		} movement;
	} volatile qeitype;

#if defined(__18CXX)
#define BCRELAYS	PORTE
#define IORELAYS	PORTJ
#define	DIPSW		PORTD
#define	EXTIO		PORTB
#endif

	typedef struct V_data { // ISR used, mainly for non-atomic mod problems
		volatile uint32_t buttonint_count, timerint_count, eeprom_count, highint_count, lowint_count, c1_int, c2_int;
		volatile uint32_t pwm4int_count, worker_count, b0, b1, b2, b3, display_count, lcdhits, lcdhits_18tcy, adc_count;
		volatile uint32_t clock20, commint_count, status_count, c1rx_int, lcd_count;
	} V_data;

	// LCD structs

	typedef struct D_data { // bitmap
		uint16_t buf : 8; // data/cmd
		uint16_t slow : 1; // long delay for clear and home
		uint16_t cmd : 1; // LCD control bit
		uint16_t state : 3; // state machine sequence
		uint16_t skip : 1; // slow cmd repeating flag
	} D_data;

	/* used to hold 16-bit LCD buffer and control bits */
	union lcd_buf_type {
		uint16_t buf;
		struct D_data map;
	};

	// ADC structs

	typedef struct A_data {
		uint16_t dummy8 : 8; // dummy space for adc data
		uint16_t dummy4 : 4; // C18 limits bitmaps to 8
		uint16_t config : 1; // adc control bit
		uint16_t index : 3; //adc channel select
	} A_data;

	/* used to hold 16-bit adc buffer, index and control bits */
	union adc_buf_type {
		uint16_t buf;
		struct A_data map;
	};

	// SPI structs, sends 8 bit data with upper/lower selects for 16 bit data

	typedef struct S_data {
		uint16_t buf : 8; // spi data
		uint16_t cs : 1; // set CS to disabled at end of command
		uint16_t load : 1; // spi device load bit
		uint16_t select : 2; //spi device select
	} S_data;

	/* used to hold 8 bit spi buffer, index and control bits */
	union spi_buf_type {
		uint16_t buf;
		struct S_data map;
	};

	/* upper/lower bytes to 16 bit word for ADC/DAC, etc ... */
	union bytes2 {
		uint16_t ld;
		uint8_t bd[2];
	};

	// mcp4822 structs

	typedef struct mcp4822_data {
		uint16_t buf : 8; // spi data
		uint16_t dummy4 : 4; // C18 limits bitmaps to 8
		uint16_t shdn : 1; // power control 1= power on
		uint16_t ga : 1; // gain select, 0=2x
		uint16_t dont_care : 1;
		uint16_t ab : 1; // DAC select, 0=DAC A
	} mcp4822_data;

	/* used to hold 12-bit spi dac word and control bits */
	union mcp4822_buf_type {
		uint16_t buf;
		struct mcp4822_data map;
	};

	/* DAC chip A/B addressing */
	typedef struct mcp4822_adr {
		uint8_t device : 1;
		uint8_t cs : 1;
	} mcp4822_adr;

	union mcp4822_adr_type {
		uint8_t buf;
		struct mcp4822_adr map;
	};

	/* tpic6b595 shift register bit addressing */
	typedef struct tpic6b595_adr {
		uint8_t o0 : 1;
		uint8_t o1 : 1;

		enum valve_t {
			PURGE, AIR, GAS, COLOR1, COLOR2
		} valve_t;
	} tpic6b595_adr;

	extern enum valve_t {
		PURGE, AIR, GAS, COLOR1, COLOR2
	} valve_t;

	union tpic6b595_adr_type {
		uint8_t buf;
		struct tpic6b595_adr map;
	};

	/* SPI devices link state */
	typedef struct L_data { // link state data
		uint8_t boot_code : 1;
		uint16_t adc_chan; // must be 16 bit value
		uint32_t adc_val[ADC_INDEX];
		uint16_t adc_raw[ADC_INDEX];
		struct ringBufS_t *rx1b, *tx1b, *rx2b, *tx2b;
	} L_data;

	struct spi_link_type { // internal state table
		uint16_t delay;
		uint8_t config;
		struct ringBufS_t *tx1b, *rx1b;
		uint32_t count;
	};

	typedef struct mfctype {
		uint8_t id;
		uint8_t done : 1;
		uint8_t timeout : 1;
		uint8_t measure : 1;

		enum gas_t {
			SHUT, FLOW, MASS
		} gas_t;
		uint16_t mfc_flow_size, mfc_actual, mfc_set, error, mfc_actual_prev, mfc_change; // in ADC counts
		float scale_out, scale_in; // scaling factor from actual to scaled and back
		uint32_t mfc_integ_total_mass, mfc_integ_current_mass, mfc_integ_target_mass,
		flow_time_total, flow_time_left;
	} volatile mfctype;

	extern enum gas_t {
		SHUT, FLOW, MASS
	} gas_t;

	int8_t spinners(uint8_t, uint8_t);
	float lp_filter(float, int16_t, int16_t);
	int8_t* ahfp(int32_t, int8_t *);
	int8_t* voltfpi(int32_t, int8_t *);
	int8_t* voltfps(uint32_t, int8_t *);
	int8_t* voltfp(uint32_t, int8_t *);

#ifdef	__cplusplus
}
#endif

#endif	/* PYRO_H */

