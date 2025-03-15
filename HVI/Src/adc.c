/**
 * @file adc.c
 * @brief Board-specific ADC implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h>  // HAL interface
#include "adc.h"    // Interface to implement

/**
 * @brief Board-specific ADC channel configuration.
 *
 * Replace/add more ADC channel configurations here as appropriate. Each
 * enumeration value of `adcChannel_t` should get a configuration.
 *
 * @see `CMR/adc.h` for various initialization values.
 */
cmr_adcChannel_t adcChannels[ADC_LEN] = {
    [ADC_VSENSE] = {
        .channel = ADC_CHANNEL_10,
        .port = GPIOC,
        .pin = GPIO_PIN_0,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_ISENSE] = {
        .channel = ADC_CHANNEL_11,
        .port = GPIOC,
        .pin = GPIO_PIN_1,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
	[ADC_VREF] = {
		 .channel = ADC_CHANNEL_12,
		 .port = GPIOC,
		 .pin = GPIO_PIN_2,
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
	static TickType_t cmr_adcSample_period_ms = 10;
    cmr_adcInit(
        &adc, ADC1,
        adcChannels, sizeof(adcChannels) / sizeof(adcChannels[0]),
		cmr_adcSample_period_ms
    );
}

/**
 * @brief Returns the current value of an ADC channel.
 */
uint32_t adcRead(adcChannels_t ch) {
    return cmr_adcRead(&adc, ch);
}

