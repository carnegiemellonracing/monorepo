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
    // XXX edit me to match your pin configuration
    [ADC_POWER_VSENSE] = {
        .channel = ADC_CHANNEL_13,
        .port = GPIOC,
        .pin = GPIO_PIN_3,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_POWER_ISENSE] = {
        .channel = ADC_CHANNEL_12,
        .port = GPIOC,
        .pin = GPIO_PIN_2,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_LOGIC_VSENSE] = {
        .channel = ADC_CHANNEL_10,
        .port = GPIOC,
        .pin = GPIO_PIN_0,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_BOARD_THERM_1] = {
        .channel = ADC_CHANNEL_0,
        .port = GPIOA,
        .pin = GPIO_PIN_0,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_BOARD_THERM_2] = {
        .channel = ADC_CHANNEL_1,
        .port = GPIOA,
        .pin = GPIO_PIN_1,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
	[ADC_BOARD_THERM_3] = {
		.channel = ADC_CHANNEL_2,
		.port = GPIOA,
		.pin = GPIO_PIN_2,
		.samplingTime = ADC_SAMPLETIME_15CYCLES,
		.value = 0
	},
	[ADC_BOARD_THERM_4] = {
		.channel = ADC_CHANNEL_3,
		.port = GPIOA,
		.pin = GPIO_PIN_3,
		.samplingTime = ADC_SAMPLETIME_15CYCLES,
		.value = 0
	},
	[ADC_BOARD_THERM_5] = {
		.channel = ADC_CHANNEL_4,
		.port = GPIOA,
		.pin = GPIO_PIN_4,
		.samplingTime = ADC_SAMPLETIME_15CYCLES,
		.value = 0
	},
	[ADC_BOARD_THERM_6] = {
		.channel = ADC_CHANNEL_5,
		.port = GPIOA,
		.pin = GPIO_PIN_5,
		.samplingTime = ADC_SAMPLETIME_15CYCLES,
		.value = 0
	},
};

/** @brief Primary ADC. */
static cmr_adc_t adc;

/**
 * @brief Initializes the ADC interface.
 */
void adcInit(void) {
    // ADC initialization and channel configuration.
    // XXX edit me to match your pin configuration
    cmr_adcInit(
        &adc, ADC1,
        adcChannels, sizeof(adcChannels) / sizeof(adcChannels[0]),
		100
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

