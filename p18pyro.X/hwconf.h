#ifndef HWCONF_H_INCLUDED
#define HWCONF_H_INCLUDED
#include <p18cxxx.h>
#include <spi.h>
#include "pyro.h"
#include <usart.h>
#include <timers.h>
#include <adc.h>
#include <pwm.h>

void config_pic(uint16_t);
void start_pic(uint16_t);
void start_workerthread(void);

extern volatile uint8_t WDT_TO, EEP_ER;

#endif /* HWCONF_H_INCLUDED */

