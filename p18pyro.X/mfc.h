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
	int8_t mfc_done(struct mfctype *);
	int8_t mfc_shut(struct mfctype *);
	int8_t mfc_flow(struct mfctype *, uint16_t);
	int8_t mfc_mass(struct mfctype *, uint16_t, uint32_t);

#ifdef	__cplusplus
}
#endif

#endif	/* MFC_H */

