/**
 * @file adc.c
 * @brief ADC implementation for 26x BMB HITL.
 *
 * @author Ayush Garg
 * modified by Yi-An Liao
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
    [ADC_CELL_1] = {
        .channel = ADC_CHANNEL_1,
        .port = GPIOA,
        .pin = GPIO_PIN_0,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_CELL_2] = {
        .channel = ADC_CHANNEL_1,
        .port = GPIOA,
        .pin = GPIO_PIN_1,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_CELL_3] = {
        .channel = ADC_CHANNEL_3,
        .port = GPIOF,
        .pin = GPIO_PIN_6,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_CELL_4] = {
        .channel = ADC_CHANNEL_3,
        .port = GPIOF,
        .pin = GPIO_PIN_7,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_CELL_5] = {
        .channel = ADC_CHANNEL_3,
        .port = GPIOF,
        .pin = GPIO_PIN_8,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_CELL_6] = {
        .channel = ADC_CHANNEL_3,
        .port = GPIOF,
        .pin = GPIO_PIN_9,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_CELL_7] = {
        .channel = ADC_CHANNEL_3,
        .port = GPIOF,
        .pin = GPIO_PIN_10,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_CELL_8] = {
        .channel = ADC_CHANNEL_123,
        .port = GPIOC,
        .pin = GPIO_PIN_0,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_CELL_9] = {
        .channel = ADC_CHANNEL_123,
        .port = GPIOC,
        .pin = GPIO_PIN_1,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_CELL_10] = {
        .channel = ADC_CHANNEL_12,
        .port = GPIOA,
        .pin = GPIO_PIN_5,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_CELL_11] = {
        .channel = ADC_CHANNEL_12,
        .port = GPIOA,
        .pin = GPIO_PIN_6,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_CELL_12] = {
        .channel = ADC_CHANNEL_12,
        .port = GPIOA,
        .pin = GPIO_PIN_7,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_CELL_13] = {
        .channel = ADC_CHANNEL_12,
        .port = GPIOC,
        .pin = GPIO_PIN_4,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_CELL_14] = {
        .channel = ADC_CHANNEL_12,
        .port = GPIOC,
        .pin = GPIO_PIN_5,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_CELL_15] = {
        .channel = ADC_CHANNEL_12,
        .port = GPIOB,
        .pin = GPIO_PIN_0,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_CELL_16] = {
        .channel = ADC_CHANNEL_12,
        .port = GPIOB,
        .pin = GPIO_PIN_1,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_CELL_17] = {
        .channel = ADC_CHANNEL_2,
        .port = GPIOF,
        .pin = GPIO_PIN_14,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_CELL_18] = {
        .channel = ADC_CHANNEL_1,
        .port = GPIOF,
        .pin = GPIO_PIN_11,
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
        adcChannels, sizeof(adcChannels) / sizeof(adcChannels[0]),
        5
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