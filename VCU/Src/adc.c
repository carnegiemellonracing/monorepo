/**
 * @file adc.c
 * @brief Board-specific ADC implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "adc.h"    // Interface to implement

/**
 * TODO: Verify the ADC peripheral, channel, GPIO port/pin, and sampling time
 * for every channel against the new combined board schematic.
 *
 * The mappings below are carried over from the previous VSM and DCM boards.
 * The VSM channels previously used ADC1, while the DCM thermistor channels
 * previously used ADC3.
 */

/**
 * @brief Board-specific ADC channel configuration.
 *
 * Replace/add more ADC channel configurations here as appropriate. Each
 * enumeration value of `adcChannel_t` should get a configuration.
 *
 * @see `CMR/adc.h` for various initialization values.
 */
static cmr_adcChannel_t adcChannels[ADC_LEN] = {

    /* VSM channels */
    [ADC_HALL_EFFECT] = {
        .channel = ADC_CHANNEL_1,
        .port = GPIOA,
        .pin = GPIO_PIN_1,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
    },

    [ADC_REAR_BRAKE_PRES] = {
        .channel = ADC_CHANNEL_2,
        .port = GPIOA,
        .pin = GPIO_PIN_2,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
    },

    [ADC_VSENSE] = {
        .channel = ADC_CHANNEL_15,
        .port = GPIOC,
        .pin = GPIO_PIN_5,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
    },

    [ADC_SSIN] = {
        .channel = ADC_CHANNEL_7,
        .port = GPIOA,
        .pin = GPIO_PIN_7,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
    },

    [ADC_SSOUT] = {
        .channel = ADC_CHANNEL_14,
        .port = GPIOC,
        .pin = GPIO_PIN_4,
        .samplingTime = ADC_SAMPLETIME_15CYCLES,
    },

    /* DCM channels */
    [ADC_THERM1] = {
        .channel = ADC_CHANNEL_2,
        .port = GPIOF,
        .pin = GPIO_PIN_9,
        .samplingTime = ADC3_SAMPLETIME_12CYCLES_5,
    },

    [ADC_THERM2] = {
        .channel = ADC_CHANNEL_7,
        .port = GPIOF,
        .pin = GPIO_PIN_8,
        .samplingTime = ADC3_SAMPLETIME_12CYCLES_5,
    },

    [ADC_THERM3] = {
        .channel = ADC_CHANNEL_3,
        .port = GPIOF,
        .pin = GPIO_PIN_7,
        .samplingTime = ADC3_SAMPLETIME_12CYCLES_5,
    },

    [ADC_THERM4] = {
        .channel = ADC_CHANNEL_8,
        .port = GPIOF,
        .pin = GPIO_PIN_6,
        .samplingTime = ADC3_SAMPLETIME_12CYCLES_5,
    }
};

/** @brief Primary ADC. */
static cmr_adc_t adc;

/**
 * @brief Reads the given ADC channel's latest value.
 *
 * @param channel The channel.
 *
 * @return The read value.
 */

 //TODO: Once board is finalized, implement this
uint32_t adcRead(adcChannel_t channel) {
    if (channel is VSM) {
        return cmr_adcRead(&adc, channel);
    }
    else if (channel is DCM) {
        return adcChannels[ch].value;
    }
}


/**
 * @brief Initializes the ADC interface.
 */
void adcInit(void) {
    // ADC initialization and channel configuration.
    static const TickType_t sampleTime_ms = 5;
    cmr_adcInit(
        &adc, ADC1,
        adcChannels, sizeof(adcChannels) / sizeof(adcChannels[0]),
        sampleTime_ms
    );
}

void adcInit(void) {
    // ADC initialization and channel configuration.
    cmr_adcInit(
        &adc, ADC3,
        adcChannels, sizeof(adcChannels) / sizeof(adcChannels[0]),
        10
    );
}