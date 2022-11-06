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
    [ADC_VSENSE] = {
        .channel = ADC_CHANNEL_0,
        .port = GPIOA,
        .pin = GPIO_PIN_0,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_ISENSE] = {
        .channel = ADC_CHANNEL_1,
        .port = GPIOA,
        .pin = GPIO_PIN_1,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_CH1] = {
        .channel = ADC_CHANNEL_8, // TODO: No ADC channel
        .port = GPIOA,
        .pin = GPIO_PIN_8,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_CH2] = {
        .channel = ADC_CHANNEL_15, // TODO: No ADC channel
        .port = GPIOC,
        .pin = GPIO_PIN_10,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_CH3] = {
        .channel = ADC_CHANNEL_14, // TODO: No ADC channel
        .port = GPIOC,
        .pin = GPIO_PIN_11,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_CH4] = {
        .channel = ADC_CHANNEL_5,
        .port = GPIOA,
        .pin = GPIO_PIN_5,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_CH5] = {
        .channel = ADC_CHANNEL_4,
        .port = GPIOA,
        .pin = GPIO_PIN_4,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_SS_MODULE] = {
        .channel = ADC_CHANNEL_10, // TODO: No ADC channel
        .port = GPIOA,
        .pin = GPIO_PIN_12,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_SS_COCKPIT] = {
        .channel = ADC_CHANNEL_11, // TODO: No ADC channel
        .port = GPIOA,
        .pin = GPIO_PIN_11,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_SS_FRHUB] = {
        .channel = ADC_CHANNEL_12,  // TODO: No ADC channel
        .port = GPIOA,
        .pin = GPIO_PIN_10,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_SS_INERTIA] = {
        .channel = ADC_CHANNEL_13, // TODO: no ADC channel
        .port = GPIOA,
        .pin = GPIO_PIN_9,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_SS_FLHUB] = {
        .channel = ADC_CHANNEL_2,
        .port = GPIOA,
        .pin = GPIO_PIN_2,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_SS_BOTS] = {
        .channel = ADC_CHANNEL_3,
        .port = GPIOA,
        .pin = GPIO_PIN_3,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
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
        &adc, ADC1,
        adcChannels, sizeof(adcChannels) / sizeof(adcChannels[0])
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

