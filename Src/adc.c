/**
 * @file adc.c
 * @brief Board-specific ADC implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "adc.h"  // Interface to implement

/**
 * @brief Board-specific ADC channel configuration.
 *
 * Replace/add more ADC channel configurations here as appropriate. Each
 * enumeration value of `adcChannel_t` should get a configuration.
 *
 *
 *
 * @see `CMR/adc.h` for various initialization values.
 */
static cmr_adcChannel_t adcChannels[ADC_LEN] = {
    [ADC_TPOS_L] = {
        .channel = ADC_CHANNEL_1,
        .port = GPIOA,
        .pin = GPIO_PIN_1,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0 },
    [ADC_TPOS_R] = { .channel = ADC_CHANNEL_0, .port = GPIOA, .pin = GPIO_PIN_0, .samplingTime = ADC_SAMPLETIME_15CYCLES, .value = 0 },
    [ADC_BPRES] = { .channel = ADC_CHANNEL_3, .port = GPIOA, .pin = GPIO_PIN_3, .samplingTime = ADC_SAMPLETIME_15CYCLES, .value = 0 },
    [ADC_SWANGLE] = { .channel = ADC_CHANNEL_2, .port = GPIOA, .pin = GPIO_PIN_2, .samplingTime = ADC_SAMPLETIME_15CYCLES, .value = 0 }
};

/** @brief Primary ADC. */
static cmr_adc_t adc;

/**
 * @brief Initializes the ADC interface.
 */
void adcInit(void) {
    // ADC initialization and channel configuration.
    cmr_adcInit(
        &adc, ADC1,
        adcChannels, sizeof(adcChannels) / sizeof(adcChannels[0]),
        10);
}

/**
 * @brief Reads the given ADC channel's latest value.
 *
 * @param channel The channel.
 *
 * @return The read value.
 */
uint32_t adcRead(adcChannel_t channel) {
    return cmr_adcRead(&adc, channel);
}
