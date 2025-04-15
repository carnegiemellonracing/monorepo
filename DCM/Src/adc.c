/**
 * @file adc.c
 * @brief Board-specific ADC implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "adc.h"    // Interface to implement

/**
 * @brief Board-specific ADC channel configuration.
 *
 * Replace/add more ADC channel configurations here as appropriate. Each
 * enumeration value of `adcChannel_t` should get a configuration.
 *
 * @see `CMR/adc.h` for various initialization values.
 */
static cmr_adcChannel_t adcChannels[ADC_LEN] = {
    [ADC_LINPOT1] = {
        .channel = ADC_CHANNEL_8,
        .port = GPIOF,
        .pin = GPIO_PIN_6,
        .samplingTime = ADC_SAMPLETIME_1CYCLE_5,
        .value = 0
    },
    [ADC_LINPOT2] = {
        .channel = ADC_CHANNEL_3,
        .port = GPIOF,
        .pin = GPIO_PIN_7,
        .samplingTime = ADC_SAMPLETIME_1CYCLE_5,
        .value = 0
    }
};

/** @brief Primary ADC. */
static cmr_adc_t adc;

/**
 * @brief Initializes the ADC interface.
 */
void adcInit(void) {
    // ADC initialization and channel configuration.
    cmr_adcInit(
        &adc, ADC3,
        adcChannels, sizeof(adcChannels) / sizeof(adcChannels[0]),
        10
    );
}

/**
 * @brief Returns the current value of an ADC channel.
 */
uint32_t adcRead(adcChannel_t ch) {
    return adcChannels[ch].value;
}

