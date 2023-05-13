/**
 * @file fir_filter.h
 * @brief Naive implementation of FIR (finite impulse response) filters
 * 
 * @warning The user should statically allocate buffers for samples and
 * filter coefficients, and ensure that their lengths are correct
 */

#ifndef _FIR_FILTER_H_
#define _FIR_FILTER_H_

#include <stddef.h>

/** @brief State of an FIR filter */
typedef struct {
    /** @brief User-provided buffer for the FIR filter, must have length len */
    float *buf;
    /** @brief User-provided FIR filter coefficients, must have length len */
    const float *coefs;
    /** @brief Length of buf and coefs */
    size_t len;
} cmr_fir_filter_state_t;

/** @brief Pre-designed filter coefficients*/
extern const float FIR_COEFFICIENTS_6_100_10[6];
extern const float FIR_COEFFICIENTS_11_100_5[11];
extern const float FIR_COEFFICIENTS_9_80_5[9];

void cmr_fir_filter_init(
    cmr_fir_filter_state_t *filter_state,
    float *buf,
    const float *coefs,
    size_t len,
    float initial_value
);
void cmr_fir_filter_reset(
    cmr_fir_filter_state_t *filter_state,
    float initial_value
);
float cmr_fir_filter_update(
    cmr_fir_filter_state_t *filter_state,
    float new_sample
);
float cmr_fir_filter_peak(const cmr_fir_filter_state_t *filter_state);

#endif /* _FIR_FILTER_H_ */
