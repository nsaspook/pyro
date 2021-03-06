/*
 * This is the high and low ISR vector interface dependency header
 */
//FIXME there is way too much stuff in globals

#ifndef PYRO_VECTOR_H_INCLUDED
#define PYRO_VECTOR_H_INCLUDED
#include <p18cxxx.h>
#include "pyro.h"
#include "pyro_shared.h"
#include <usart.h>
#include <timers.h>
#include <adc.h>
#include "ringbufs.h"
#include "xlcd.h"
#include "mfc.h"

void tick_handler(void);
void work_handler(void);
void Clear_All_Buttons(void);

/* BEGIN HIGH ISR */

extern volatile uint8_t TIMERFLAG, SYSTEM_STABLE, COOLING,
        almctr, WORKERFLAG, DISPLAY_MODE, dsi, C2RAW, FAILSAFE, SYS_HELP, SYS_DATA,
        HOLD_PROC, RESET_ZEROS, SET_TLOG, TWEAK, KEYNUM, SET_BATT, D_UPDATE, SLOW_STATUS;
extern volatile uint8_t GLITCH_CHECK, cdelay, knob_to_pot;
extern volatile int32_t slow_timer;

extern struct V_data V;
extern volatile struct L_data L;
extern volatile struct knobtype knob1, knob2;
extern volatile struct qeitype qei1;
extern volatile struct buttontype button;
extern volatile struct QuadEncoderType OldEncoder;
extern volatile struct spi_link_type spi_link;

extern volatile enum movement_t {
    CW, STOP, CCW
} movement;

extern volatile enum answer_t {
    WAIT_M, YES_M, NO_M
} YNKEY;

/* BEGIN LOW ISR */
extern struct mfctype mfc[4], *mfcptr;

/* BEGIN IDLE_LOOP EXTRAS */
extern volatile uint8_t IDLEFLAG;

/* END IDLE_LOOP EXTRAS */
#endif /* PYRO_VECTOR_H_INCLUDED */

