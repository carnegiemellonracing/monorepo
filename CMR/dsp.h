/**
 * @file dsp.h
 * @brief Digital Signal Processing functions
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_DSP_H
#define CMR_DSP_H

#include "platform.h"
#include "arm_math.h"

/** @brief Signals voodoo, just trust it */
#define CMR_DSP_FILTER_NUM_STAGES	4

/** @brief Predefined filter modes */
typedef enum {
    CMR_DSP_FILTER_1HZ = 0,     /**< @brief Low pass filter with corner @ 1Hz */
    CMR_DSP_FILTER_5HZ,         /**< @brief Low pass filter with corner @ 5Hz */
    CMR_DSP_FILTER_10HZ,        /**< @brief Low pass filter with corner @ 10Hz */
    CMR_DSP_FILTER_25HZ,        /**< @brief Low pass filter with corner @ 25Hz */
    CMR_DSP_FILTER_100HZ,       /**< @brief Low pass filter with corner @ 100Hz */
    CMR_DSP_FILTER_250HZ,       /**< @brief Low pass filter with corner @ 250Hz */
    CMR_DSP_FILTER_500HZ,       /**< @brief Low pass filter with corner @ 500Hz */
    CMR_DSP_FILTER_1000HZ,      /**< @brief Low pass filter with corner @ 1000Hz */
    CMR_DSP_FILTER_NUM          /**< @brief Number of filter frequencies */
} cmr_dspFilterSelection_t;

/** @brief Type for CMSIS land, don't worry about this
 * (needs to be allocated along with cmr_dspFilter_t) */
typedef float32_t cmr_dsp_cmsis_buffer_t[2 * CMR_DSP_FILTER_NUM_STAGES];

/** @brief Wrapper for cmsis types */
typedef struct {
    /** @brief Not important, this is private anyways */
    arm_biquad_cascade_df2T_instance_f32 _instance;
    /** @brief Also not important */
    cmr_dsp_cmsis_buffer_t _buf;
} cmr_dspFilter_t;

void cmr_dspFilterInit(cmr_dspFilter_t *filter, cmr_dspFilterSelection_t sel);
void cmr_dspFilterProcess(cmr_dspFilter_t *filter, float32_t *in, float32_t *out);
uint32_t cmr_dspFilterProcessFixed(cmr_dspFilter_t *filter, uint32_t in);

#endif /* CMR_DSP_H */
