/**
 * @file adc.c
 * @brief Board-specific ADC implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "adc.h"

/**
 * @brief Board-specific ADC channel configuration.
 *
 * Replace/add more ADC channel configurations here as appropriate. Each
 * enumeration value of `adcChannel_t` should get a configuration.
 *
 * @see `CMR/adc.h` for various initialization values.
 */
static cmr_adcChannel_t adcChannels[ADC_LEN] = {
    [ADC_THERM_1] = {
        .channel = ADC_CHANNEL_1,
        .port = GPIOA,
        .pin = GPIO_PIN_1,
        .samplingTime = ADC_SAMPLETIME_480CYCLES,
    },
    [ADC_THERM_2] = {
        .channel = ADC_CHANNEL_0,
        .port = GPIOA,
        .pin = GPIO_PIN_0,
        .samplingTime = ADC_SAMPLETIME_480CYCLES,
    },
};

/** @brief Primary ADC. */
static cmr_adc_t adc;

/**
 * @brief Initializes the ADC interface.
 */
void adcInit(void) {
    // ADC initialization and channel configuration.
    static const TickType_t sampleTime_ms = 10;
    cmr_adcInit(
        &adc, ADC1,
        adcChannels, sizeof(adcChannels) / sizeof(adcChannels[0]),
        sampleTime_ms
    );
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
