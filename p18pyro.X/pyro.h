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


#ifdef	__cplusplus
}
#endif

#endif	/* PYRO_H */

