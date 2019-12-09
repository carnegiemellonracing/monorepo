/**
 * @file dsp.c
 * @brief Digital Signal Processing functions
 *
 * @author Carnegie Mellon Racing
 */

#include <stdlib.h>
#include "dsp.h"

void cmr_dspFilterInit(cmr_dspFilter_t *filter, cmr_dspFilterSelection_t sel) {
	filter->instance.numStages = CMR_DSP_FILTER_NUM_STAGES;
	filter->instance.pState = (float*) calloc(sizeof(float), 2 * CMR_DSP_FILTER_NUM_STAGES);
	filter->instance.pCoeffs = (float*) &cmr_dspFilterCoeffs[sel];
}

void cmr_dspFilterProcess(cmr_dspFilter_t *filter, float *in, float *out){
	arm_biquad_cascade_df2T_f32(&filter->instance, in, out, 1);
}

uint32_t cmr_dspFilterProcessFixed(cmr_dspFilter_t *filter, uint32_t in){
	float in_f = (float) in;
	float out_f;
	cmr_dspFilterProcess(filter, &in_f, &out_f);
	return (uint32_t) out_f;
}

void cmr_dspFilterFree(cmr_dspFilter_t *filter){
	free(filter->instance.pState);
}
