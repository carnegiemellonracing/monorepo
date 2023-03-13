/**
 * @file fir_filter.h
 * @brief Naive implementation of FIR (finite impulse response) filters
 * 
 * @warning The user should statically allocate buffers for samples and
 * filter coefficients, and ensure that their lengths are correct
 */

#ifndef _FIR_FILTER_H_
#define _FIR_FILTER_H_

#include <inttypes.h>
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

/**
 * ------- Pre-designed filters ------- 
 * @brief Pre-designed filter coefficients, useful for testing and debugging
 * @note Please design your own filter for your particular use case
 */

// window length:       6 samples
// sample rate:         100Hz
// cutoff frequency:    10Hz
// group delay:         (1 / 100Hz) * (6 - 1) / 2 = 25ms
const float FIR_COEFFICIENTS_6_100_10[6] = {
    0.091148595327006,
    0.179031716037403,
    0.229819688635591,
    0.229819688635591,
    0.179031716037403,
    0.091148595327006
};

// window length:       11 samples
// sample rate:         100Hz
// cutoff frequency:    5Hz
// group delay:         (1 / 100Hz) * (11 - 1) / 2 = 50ms
const float FIR_COEFFICIENTS_11_100_5[11] = {
    0.033083518941111,
    0.065270358225107,
    0.093043719495299,
    0.114433034257841,
    0.127911899644417,
    0.132514938872449,
    0.127911899644417,
    0.114433034257841,
    0.093043719495299,
    0.065270358225107,
    0.033083518941111
};

// window length:       9 samples
// sample rate:         80Hz
// cutoff frequency:    5Hz
// group delay:         (1 / 80Hz) * (9 - 1) / 2 = 50ms
const float FIR_COEFFICIENTS_9_80_5[9] = {
    0.047747067652875,
    0.092418281872636,
    0.128356570723088,
    0.151633628385033,
    0.159688902732734,
    0.151633628385033,
    0.128356570723088,
    0.092418281872636,
    0.047747067652875
};

/* ------- End of pre-designed filters ------- */

void cmr_fir_filter_init(
    cmr_fir_filter_state_t *filter_state,
    float *buf,
    const float *coefs,
    size_t len
);
void cmr_fir_filter_reset(cmr_fir_filter_state_t *filter_state);
float cmr_fir_filter_update(
    cmr_fir_filter_state_t *filter_state,
    float new_sample
);
float cmr_fir_filter_peak(const cmr_fir_filter_state_t *filter_state);

#endif /* _FIR_FILTER_H_ */
