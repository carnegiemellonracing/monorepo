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
	// XXX edit me to match your pin configuration
    [V24V] = {
        .channel = ADC_CHANNEL_0,
        .port = GPIOA,
        .pin = GPIO_PIN_0,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [HV_PLUS] = {
        .channel = ADC_CHANNEL_1,
        .port = GPIOA,
        .pin = GPIO_PIN_1,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [HV_MINUS] = {
        .channel = ADC_CHANNEL_2,
        .port = GPIOA,
        .pin = GPIO_PIN_2,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [BATT_PLUS] = {
        .channel = ADC_CHANNEL_3,
        .port = GPIOA,
        .pin = GPIO_PIN_3,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [BATT_MINUS] = {
        .channel = ADC_CHANNEL_4,
        .port = GPIOA,
        .pin = GPIO_PIN_4,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [SHUNT_LV] = {
        .channel = ADC_CHANNEL_5,
        .port = GPIOA,
        .pin = GPIO_PIN_5,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [AIR_POWER] = {
        .channel = ADC_CHANNEL_6,
        .port = GPIOA,
        .pin = GPIO_PIN_6,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [DCDC_CURR] = {
        .channel = ADC_CHANNEL_7,
        .port = GPIOA,
        .pin = GPIO_PIN_7,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
        .value = 0
    },
    [LV_CURR] = {
        .channel = ADC_CHANNEL_8,
        .port = GPIOB,
        .pin = GPIO_PIN_0,
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
        adcChannels, sizeof(adcChannels) / sizeof(adcChannels[0])
    );
}

