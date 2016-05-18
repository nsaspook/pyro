/* 
 * File:   mfc.h
 * Author: root
 *
 * Created on May 15, 2016, 8:38 PM
 */

#ifndef MFC_H
#define	MFC_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "pyro.h"
#include "daq.h"

	void mfc_config(void);
	int8_t mfc_set(struct mfctype *);

#ifdef	__cplusplus
}
#endif

#endif	/* MFC_H */

