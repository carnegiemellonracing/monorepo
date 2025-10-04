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
    [ADC_HALL_EFFECT] = {
        .channel = ADC_CHANNEL_1,
        .port = GPIOA,
        .pin = GPIO_PIN_1,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_REAR_BRAKE_PRES] = {
        .channel = ADC_CHANNEL_2,
        .port = GPIOA,
        .pin = GPIO_PIN_2,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_VSENSE] = {
        .channel = ADC_CHANNEL_15,
        .port = GPIOC,
        .pin = GPIO_PIN_5,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_ISENSE] = { /** @todo remove? */
        .channel = ADC_CHANNEL_8,
        .port = GPIOB,
        .pin = GPIO_PIN_0,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_SSIN] = { /** @todo verify right channel */
        .channel = ADC_CHANNEL_17,
        .port = GPIOD,
        .pin = GPIO_PIN_7,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_SSOUT] = { /** @todo verify right channel */
        .channel = ADC_CHANNEL_18,
        .port = GPIOE,
        .pin = GPIO_PIN_4,
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
        20
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

