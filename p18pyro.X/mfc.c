/*
 * the MFC state machine control routines
 * needs to return the integrated flow value since the last reset command for each unit in the structure pointer
 * with a status value returned from the function
 */

#include "mfc.h"

struct mfctype mfc[4] = {0}, *mfcptr;

void mfc_config(void)
{
	uint8_t i;

	mfc[AIR_MFC].mfc_flow_size = 5000; // 50SLM
	mfc[GAS_MFC].mfc_flow_size = 1500;
	mfc[COLOR1_MFC].mfc_flow_size = 100;
	mfc[COLOR2_MFC].mfc_flow_size = 100;

	for (i = 0; i <= 3; i++) {
		mfc[i].gas_t = SHUT;
	}
}