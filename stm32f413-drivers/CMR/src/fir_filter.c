/**
 * @file fir_filter.c
 * @brief Naive implementation of FIR (finite impulse response) filters
 *
 * @warning The user should statically allocate buffers for samples and
 * filter coefficients, and ensure that their lengths are correct
 */

#include <FreeRTOSConfig.h>  // for configASSERT
#include <stddef.h>          // for size_t
#include "CMR/fir_filter.h"  // for cmr_fir_filter_state_t, cmr_fir_filter_init

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

// window length:       5 samples
// sample rate:         80Hz
// cutoff frequency:    10Hz
// group delay:         (1 / 80Hz) * (5 - 1) / 2 = 25ms
const float FIR_COEFFICIENTS_5_80_10[5] = {
    0.144424263033140,
    0.226592535371819,
    0.257966403190083,
    0.226592535371819,
    0.144424263033140
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

/**
 * @brief Initialize the filter state
 * @param filter_state Pointer to the filter state to be initialized
 * @param buf Buffer for the FIR filter, must have length len
 * @param coefs FIR filter coefficients, must have length len
 * @param len The length of buf and coefs
 * @param initial_value The initial value of the filter
 */
void cmr_fir_filter_init(
    cmr_fir_filter_state_t *filter_state,
    float *buf,
    const float *coefs,
    size_t len,
    float initial_value
) {
    configASSERT(filter_state != NULL);
    configASSERT(buf != NULL);
    configASSERT(coefs != NULL);

    // fill in the fields
    filter_state->buf = buf;
    filter_state->coefs = coefs;
    filter_state->len = len;

    // reset the filter state
    cmr_fir_filter_reset(filter_state, initial_value);
}

/**
 * @brief Resets the filter state by filling the buffer with zeros
 * @param filter_state Pointer to the filter state
 * @param initial_value The initial value of the filter
 */
void cmr_fir_filter_reset(
    cmr_fir_filter_state_t *filter_state,
    float initial_value
) {
    configASSERT(filter_state != NULL);
    configASSERT(filter_state->buf != NULL);

    // fill buf with zeros
    for (size_t i = 0; i < filter_state->len; i++) {
        filter_state->buf[i] = initial_value;
    }
}

/**
 * @brief Push a new sample into the buffer and compute the filtered value
 * @note This function should be called at the filter's sampling rate
 * @param filter_state Pointer to the filter state
 * @param new_sample The new sample to be inserted into the buffer
 * @return The dot product of the updated buffer and the filter coefficients
 */
float cmr_fir_filter_update(
    cmr_fir_filter_state_t *filter_state,
    float new_sample
) {
    configASSERT(filter_state != NULL);
    configASSERT(filter_state->buf != NULL);
    configASSERT(filter_state->coefs != NULL);

    float dot_product = 0.0f;

    // right-shift the buffer by one element and insert new_sample at buf[0]
    // and compute the dot product
    for (size_t i = 0; i < filter_state->len; i++) {
        // swap buf[i] and new_sample
        float tmp = filter_state->buf[i];
        filter_state->buf[i] = new_sample;
        new_sample = tmp;

        // update dot_product
        dot_product += filter_state->buf[i] * filter_state->coefs[i];
    }

    return dot_product;
}

/**
 * @brief Compute te filtered value without updating the buffer
 * @param filter_state Pointer to the filter state
 * @return The dot product of the updated buffer and the filter coefficients
 */
float cmr_fir_filter_peak(
    const cmr_fir_filter_state_t *filter_state
) {
    configASSERT(filter_state != NULL);
    configASSERT(filter_state->buf != NULL);
    configASSERT(filter_state->coefs != NULL);

    float dot_product = 0.0f;

    // compute the dot product
    for (size_t i = 0; i < filter_state->len; i++) {
        // update dot_product
        dot_product += filter_state->buf[i] * filter_state->coefs[i];
    }

    return dot_product;
}
