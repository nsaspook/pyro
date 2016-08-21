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

	mfc[AIR_MFC].mfc_flow_size = 500; // 0.1 units 50SLM for 5 volts
	mfc[GAS_MFC].mfc_flow_size = 150;
	mfc[COLOR1_MFC].mfc_flow_size = 500;
	mfc[COLOR2_MFC].mfc_flow_size = 10;

	/*
	 * setup defaults
	 */
	for (i = 0; i <= 3; i++) {
		mfc[i].id = i;
		mfc[i].gas_t = SHUT;
		mfc[i].mfc_integ_total_mass = 0;
		mfc[i].mfc_integ_current_mass = 0;
		mfc[i].mfc_integ_target_mass = 0;
		mfc[i].scale_out = ((float) mfc[i].mfc_flow_size) / MFC_VOLTS;
		mfc[i].flow_time_max = WORK_TICKS_S;
	}
}

int8_t mfc_set(struct mfctype * mfc)
{
	int8_t ret = -1;
	uint8_t mfc_id = mfc->id;
	union mcp4822_adr_type mfc_dac_select;

	mfcptr = mfc;

	switch (mfc->gas_t) {
	case MASS:
		ret = 0;
		if (!mfc->measure) {
			mfc->measure = HIGH;
			mfc_dac_select.buf = mfc_id;
			mfc->mfc_integ_current_mass = 0;
			mfc->flow_time = 0;
			mfc->flow_time_max = WORK_TICKS_S;
			mfc->timeout = FALSE;
			ret = SPI_Daq_Update(mfc->mfc_set, mfc_dac_select.map.cs, mfc_dac_select.map.device);
		}
		break;
	case FLOW:
		mfc->measure = HIGH;
		mfc_dac_select.buf = mfc_id;
		ret = SPI_Daq_Update(mfc->mfc_set, mfc_dac_select.map.cs, mfc_dac_select.map.device);
		break;
	case SHUT:
		mfc->done = LOW;
		mfc->measure = LOW;
	default:
		mfc->mfc_set = 0;
		mfc_dac_select.buf = mfc_id;
		ret = SPI_Daq_Update(mfc->mfc_set, mfc_dac_select.map.cs, mfc_dac_select.map.device);
		break;
	}
	if (ret)
		DLED_6 = ON;

	return ret;
}

int8_t mfc_done(struct mfctype * mfc)
{
	if (mfc->done)
		return 1;
	else
		return 0;
}

/*
 * return MFC timeout status and clear flag if TRUE
 */
int8_t mfc_timeout(struct mfctype * mfc)
{
	uint8_t ret = FALSE;

	if (mfc->timeout) {
		mfc->timeout = FALSE;
		ret = TRUE;
	}
	return ret;
}

int8_t mfc_shut(struct mfctype * mfc)
{
	mfc->gas_t = SHUT;
	return mfc_set(mfc);
}

int8_t mfc_flow(struct mfctype * mfc, const uint16_t flow)
{
	mfc->mfc_set = flow;
	mfc->gas_t = FLOW;
	return mfc_set(mfc);
}

int8_t mfc_mass(struct mfctype * mfc, const uint16_t flow, const uint32_t mass)
{
	mfc->mfc_set = flow;
	mfc->mfc_integ_target_mass = mass * MFC_INTEG; // total volts
	mfc->gas_t = MASS;
	return mfc_set(mfc);
}
