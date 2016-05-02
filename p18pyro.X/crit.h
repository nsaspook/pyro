#ifndef CRIT_H_INCLUDED
#define CRIT_H_INCLUDED
#include <p18cxxx.h>
#include "mandm.h"
#include "mandm_defs.h"
#include <timers.h>
#include <EEP.h>

volatile void s_crit(uint8_t);
volatile void e_crit(void);
volatile void clear_crit(void);
void write_data_eeprom(uint8_t, uint8_t, uint16_t, uint16_t);
uint8_t read_data_eeprom(uint16_t, uint16_t);

extern uint8_t CRITC, HCRIT[CRIT_8], LCRIT[CRIT_8];
extern volatile uint32_t critc_count;
extern volatile uint8_t critc_level;
extern volatile uint8_t IDLEFLAG;

extern void idle_loop(void);

#endif /* CRIT_H_INCLUDED */

