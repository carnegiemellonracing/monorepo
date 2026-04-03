/**
 * @file adc.c
 * @brief Board-specific ADC implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h> // HAL interface
# include <stm32f4xx_hal_adc.h>
#include "adc.h"    // Interface to implement

static const TickType_t adc_period_ms = 5;

/**
 * @brief Board-specific ADC channel configuration.
 *
 * Replace/add more ADC channel configurations here as appropriate. Each
 * enumeration value of `adcChannel_t` should get a configuration.
 *
 * @see `CMR/adc.h` for various initialization values.
 */
static cmr_adcChannel_t adcChannels[ADC_NUM] = {
	[ADC_HALL_EFFECT] = {
		.channel = ADC_CHANNEL_13,
		.port = GPIOC,
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
    cmr_adcInit(&adc, ADC1, adcChannels, ADC_NUM, adc_period_ms);
}

/**
 * @brief Reads the given ADC channel's latest value.
 *
 * @param channel The channel.
 *
 * @return The read value.
 */
uint32_t adcRead(adcChannel_t ch) {
	return cmr_adcRead(&adc, ch);
}
