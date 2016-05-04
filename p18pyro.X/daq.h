#ifndef DAQ_H_INCLUDED
#define DAQ_H_INCLUDED
#include <adc.h>
#include <delays.h>
#include <usart.h>
#include <stdio.h>
#include "pyro.h"
#include "pyro_shared.h"
#include "crit.h"
#include "pyro_vector.h"

void ADC_zero(void);
void ADC_read(void);
void ADC_Update(uint16_t, uint8_t);
void do_sim(void);

extern const rom int8_t adcg0[], adcg1[], adcg2[], adcg3[], adcg4[], zero0[], zero1[], zero2[], zero3[];
extern uint32_t Vin, rawp[MAX_MFC], vbatol_t, solar_t, rawa[MAX_MFC];
extern uint8_t adc_cal[];
extern volatile uint8_t C2RAW, PRIPOWEROK, KEYNUM, SYSTEM_STABLE;
extern int16_t a10_x, a10_y, a10_z;
extern struct R_data R;
extern struct C_data C;
extern volatile struct L_data L;
extern struct emodefaulttype emodump;
extern volatile struct modetype mode;

extern volatile enum answer_t {
	WAIT_M, YES_M, NO_M
} YNKEY;
extern far int8_t bootstr2[MESG_W + 1];

extern int32_t ABSL(int32_t);
extern int16_t ABSI(int16_t);
extern float lp_filter(float, int16_t, int16_t);
extern void do_sim(void);
extern void display_system(void);
extern void wdtdelay(uint32_t);
extern void wdttime(uint32_t);
extern void term_time(void);
extern uint8_t checktime_eep(uint32_t, uint8_t);
extern void buzzer_ticks(uint8_t);
extern void emo_display(void);


#endif /* DAQ_H_INCLUDED */