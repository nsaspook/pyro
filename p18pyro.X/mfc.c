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
		mfc[i].id = i;
		mfc[i].gas_t = SHUT;
		mfc[i].mfc_integ_total_mass = 1;
		mfc[i].mfc_integ_current_mass = 1;
		mfc[i].mfc_integ_target_mass = 1;
	}
}

int8_t mfc_set(struct mfctype * mfc)
{
	int8_t ret = -1;
	uint8_t mfc_id = mfc->id;
	union mcp4822_adr_type mfc_dac_select;

	switch (mfc->gas_t) {
	case MASS:
	case FLOW:
		if (!mfc->measure) {
			mfc->measure = HIGH;
			mfc_dac_select.buf = mfc_id;
			ret = SPI_Out_Update(mfc->mfc_set, mfc_dac_select.map.cs, mfc_dac_select.map.device);
		}
		break;
	case SHUT:
		mfc->done = LOW;
		mfc->measure = LOW;
	default:
		mfc->mfc_set = 0;
		mfc_dac_select.buf = mfc_id;
		ret = SPI_Out_Update(mfc->mfc_set, mfc_dac_select.map.cs, mfc_dac_select.map.device);
		ret += -10; // add invalid gas_t error code
		break;
	}

	return ret;
}
