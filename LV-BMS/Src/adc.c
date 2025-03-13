/**
 * @file adc.c
 * @brief Board-specific ADC implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h> // HAL interface
# include <stm32f4xx_hal_adc.h>
#include <adc.h>    // Interface to implement

/**
 * @brief Board-specific ADC channel configuration.
 *
 * Replace/add more ADC channel configurations here as appropriate. Each
 * enumeration value of `adcChannel_t` should get a configuration.
 *
 * @see `CMR/adc.h` for various initialization values.
 */
static cmr_adcChannel_t adc_channels[ADC_NUM] = {
	[ADC_VTHERM_PIN1] = {
		.channel = ADC_CHANNEL_6,
		.port = GPIOA,
		.pin = GPIO_PIN_1,
		.samplingTime = ADC_SAMPLETIME_2CYCLES_5,
		.value = 0
	},
    [ADC_VTHERM_PIN2] = {
        .channel = ADC_CHANNEL_5,
        .port = GPIOA,
        .pin = GPIO_PIN_0,
        .samplingTime = ADC_SAMPLETIME_2CYCLES_5,
        .value = 0
    },
	[ADC_AFE_VCOUT] = {
		.channel = ADC_CHANNEL_3,
		.port = GPIOC,
		.pin = GPIO_PIN_2,
		.samplingTime = ADC_SAMPLETIME_2CYCLES_5,
		.value = 0
	},
	[ADC_AFE_VIOUT] = {
		.channel = ADC_CHANNEL_4,
		.port = GPIOC,
		.pin = GPIO_PIN_3,
		.samplingTime = ADC_SAMPLETIME_2CYCLES_5,
		.value = 0
	}
};

/** @brief Primary ADC. */
static cmr_adc_t adc;

/**
 * @brief Initializes the ADC interface.
 */
void adc_init(void) {
    cmr_adcInit(&adc, ADC1, adc_channels, ADC_NUM, adc_period_ms);
}

/**
 * @brief Reads the given ADC channel's latest value.
 *
 * @param channel The channel.
 *
 * @return The read value.
 */
uint32_t adc_read(adc_channel_t ch) {
	return cmr_adcRead(&adc, ch);
}
