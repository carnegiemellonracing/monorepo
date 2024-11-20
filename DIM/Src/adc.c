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
 // ADC_LEN is the number of ADC CHANNELS
static cmr_adcChannel_t adcChannels[ADC_LEN] = {
    [ADC_TPOS_L] = {
        .channel = ADC_CHANNEL_1,
        .port = GPIOA,
        .pin = GPIO_PIN_1,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0 },
    [ADC_TPOS_R] = { .channel = ADC_CHANNEL_0, .port = GPIOA, .pin = GPIO_PIN_0, .samplingTime = ADC_SAMPLETIME_15CYCLES, .value = 0 },
    [ADC_BPRES] = { .channel = ADC_CHANNEL_3, .port = GPIOA, .pin = GPIO_PIN_3, .samplingTime = ADC_SAMPLETIME_15CYCLES, .value = 0 },
    [ADC_SWANGLE] = { .channel = ADC_CHANNEL_2, .port = GPIOA, .pin = GPIO_PIN_2, .samplingTime = ADC_SAMPLETIME_15CYCLES, .value = 0 },
    [ADC_X] = { .channel = ADC_CHANNEL_4, .port = GPIOA, .pin = GPIO_PIN_7, .samplingTime = ADC_SAMPLETIME_15CYCLES, .value = 0 },
    [ADC_Y] = { .channel = ADC_CHANNEL_5, .port = GPIOA, .pin = GPIO_PIN_8, .samplingTime = ADC_SAMPLETIME_15CYCLES, .value = 0 },
	[ADC_PADDLE] = { .channel = ADC_CHANNEL_6, .port = GPIOA, .pin = GPIO_PIN_12, .samplingTime = ADC_SAMPLETIME_15CYCLES, .value = 0 }
};
//Above is the 4 ADCs
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
/*
void cmr_adcInit(
    cmr_adc_t *adc, ADC_TypeDef *instance,
    cmr_adcChannel_t *channels, const size_t channelsLen,
    TickType_t samplePeriod_ms
 */

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
