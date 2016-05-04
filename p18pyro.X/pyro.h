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

typedef void (*display_func)(void);

struct modetype {
	volatile uint8_t demos, move, free, operate, demos_init, slow, emo, v24, cal, on_off_only, idle, locked, info_only, qei, slow_bypass, change;
	display_func display;
};

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

struct almtype {
	uint8_t alm_flag, alm_count;
};

struct almbuffertype {
	uint32_t time;
	uint8_t alm_num, bn;
};

struct datadefaulttype {
	uint32_t data_default;
};

struct emodefaulttype {
	int32_t emo[6];
	int32_t peak[6];
};


struct timeruntype {
	uint32_t hour, day, weeks;
};

#if defined(__18CXX)
#define BCRELAYS	PORTE
#define IORELAYS	PORTJ
#define	DIPSW		PORTD
#define	EXTIO		PORTB
#endif

typedef struct R_data { // set only in adc_read
	int32_t thermo_batt;
	uint32_t systemvoltage, motorvoltage, pos_x, pos_y, pos_z, change_x, change_y, change_z, max_x, max_y, max_z;
	int32_t current_x, current_y, current_z;
	uint8_t stable_x, stable_y, stable_z;
} R_data;

typedef struct C_data { // set only in adc_read
	int32_t currentload;
	int16_t temp_drate;
	float t_comp;
} C_data;

typedef struct V_data { // ISR used, mainly for non-atomic mod problems
	volatile uint32_t buttonint_count, timerint_count, eeprom_count, highint_count, lowint_count, c1_int, c2_int, hunt_count;
	volatile uint32_t pwm4int_count, worker_count, b0, b1, b2, b3, display_count, lcdhits, lcdhits_18tcy, adc_count;
	volatile uint32_t clock20, commint_count, status_count, c1rx_int;
	volatile uint8_t blink, buzzertime, voice1time, voice2time, qei_counts;
} V_data;

typedef struct L_data { // link state data
	uint8_t boot_code : 1;
	uint16_t adc_chan;
	uint16_t tx1_dac, tx2_dac;
	struct ringBufS_t *rx1b, *tx1b, *rx2b, *tx2b;
} L_data;

struct spi_link_type { // internal state table
	uint16_t delay;
	uint8_t config;
	struct ringBufS_t *tx1b, *rx1b;
	int32_t int_count;
};

#ifdef	__cplusplus
}
#endif

#endif	/* PYRO_H */

