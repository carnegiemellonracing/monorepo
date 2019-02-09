/**
 * @file adc.c
 * @brief Board-specific ADC implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <FreeRTOS.h>       // FreeRTOS interface
#include <task.h>           // xTaskCreate()

#include "adc.h"    // Interface to implement

cmr_adcChannel_t adcChannels[ADC_LEN] = {
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
	}
};

/** @brief Primary ADC. */
static cmr_adc_t adc;

/**
 * @brief Initializes the ADC interface.
 */
void adcInit(void) {
    // ADC initialization and channel configuration.
	// XXX edit me to match your pin configuration
    cmr_adcInit(&adc, ADC1, adcChannels, ADC_LEN);
}

