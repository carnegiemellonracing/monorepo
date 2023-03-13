/**
 * @file fir_filter.c
 * @brief Naive implementation of FIR (finite impulse response) filters
 * 
 * @warning The user should statically allocate buffers for samples and
 * filter coefficients, and ensure that their lengths are correct
 */

#include <stddef.h>     // size_t
#include <FreeRTOS.h>   // configASSERT()
#include "fir_filter.h"

/**
 * @brief Initialize the filter state
 * @param filter_state Pointer to the filter state to be initialized
 * @param buf buffer for the FIR filter, must have length len
 * @param coefs FIR filter coefficients, must have length len
 * @param len The length of buf and coefs
 */
void cmr_fir_filter_init(
    cmr_fir_filter_state_t *filter_state,
    float *buf,
    const float *coefs,
    size_t len
) {
    configASSERT(filter_state != NULL);
    configASSERT(buf != NULL);
    configASSERT(coefs != NULL);

    // fill in the fields
    filter_state->buf = buf;
    filter_state->coefs = coefs;
    filter_state->len = len;

    // reset the filter state
    cmr_fir_filter_reset(filter_state);
}

/**
 * @brief Resets the filter state by filling the buffer with zeros
 * @param filter_state Pointer to the filter state
 */
void cmr_fir_filter_reset(cmr_fir_filter_state_t *filter_state) {
    configASSERT(filter_state != NULL);
    configASSERT(filter_state->buf != NULL);

    // fill buf with zeros
    for (size_t i = 0; i < filter_state->len; i++) {
        filter_state->buf[i] = 0.0f;
    }
}

/** 
 * @brief Push a new sample into the buffer and compute the filtered value
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
float cmr_fir_filter_peak(const cmr_fir_filter_state_t *filter_state) {
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
