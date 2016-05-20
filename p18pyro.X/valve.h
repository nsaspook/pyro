/* 
 * File:   valve.h
 * Author: root
 *
 * Created on May 19, 2016, 11:33 AM
 */

#ifndef VALVE_H
#define	VALVE_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "pyro.h"
#include "daq.h"

	void valve_config(void);
	int8_t valve_interlock(void);
	int8_t valve_set(struct valvetype *);
	int8_t valve_switch(const uint8_t, uint8_t, const uint8_t);


#ifdef	__cplusplus
}
#endif

#endif	/* VALVE_H */

