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

	void mfc_config(void);
	
	typedef struct mfctype {

		enum gas_t {
			SHUT, FLOW, MASS
		} gas_t;
		uint16_t mfc_flow_size, mfc_actual, mfc_set, error, mfc_actual_prev, mfc_change; // in ADC counts
		float scale_out, scale_in; // scaling factor from actual to scaled and back
		uint32_t mfc_integ_total_mass, mfc_integ_current_mass;
	} volatile mfctype;

	extern enum gas_t {
		FLOW, SHUT, MASS
	} gas_t;
#ifdef	__cplusplus
}
#endif

#endif	/* MFC_H */

