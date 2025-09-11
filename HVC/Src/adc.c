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
cmr_adcChannel_t adcChannels[ADC_LEN] = {
    [ADC_V24V] = {
        .channel = ADC_CHANNEL_12,
        .port = GPIOC,
        .pin = GPIO_PIN_2,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    //adc_vref copied from hvi NEEDS TO BE UNCOMMENTED DONT DELETE 
    // [ADC_VREF] = { 
	// 	 .channel = ADC_CHANNEL_12,
	// 	 .port = GPIOC,
	// 	 .pin = GPIO_PIN_2,
	// 	 .samplingTime = ADC_SAMPLETIME_15CYCLES,
	// 	 .value = 0
	// }, 
    [ADC_AIR_POWER] = {
        .channel = ADC_CHANNEL_11,
        .port = GPIOC,
        .pin = GPIO_PIN_1,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_SAFETY] = {
        .channel = ADC_CHANNEL_13,
        .port = GPIOC,
        .pin = GPIO_PIN_3,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_VSENSE] = {
        .channel = ADC_CHANNEL_1,
        .port = GPIOA,
        .pin = GPIO_PIN_1,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [ADC_ISENSE] = {
        .channel = ADC_CHANNEL_2,
        .port = GPIOA,
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
    return adcChannels[ch].value;
}