#ifndef DAQ_H_INCLUDED
#define DAQ_H_INCLUDED
#include <adc.h>
#include <delays.h>
#include "pyro.h"
#include "pyro_shared.h"
#include "crit.h"
#include "pyro_vector.h"

void ADC_zero(void);
void ADC_Update(const uint16_t, const uint8_t);
int8_t SPI_Daq_Update(const uint16_t, const uint8_t, const uint8_t);

extern uint8_t adc_cal[];
extern volatile struct L_data L;

#endif /* DAQ_H_INCLUDED */