/**
 * @file adc.c
 * @brief Board-specific ADC implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "adc.h"  // Interface to implement

/**
 * @brief Board-specific ADC channel configuration.
 *
 * Replace/add more ADC channel configurations here as appropriate. Each
 * enumeration value of `adcChannel_t` should get a configuration.
 *
 *
 *
 * @see `CMR/adc.h` for various initialization values.
 */
/**
 * @brief Initializes the ADC interface.
 */
void adcInit(void) {
}
/*
void cmr_adcInit(
    cmr_adc_t *adc, ADC_TypeDef *instance,
    cmr_adcChannel_t *channels, const size_t channelsLen,
    TickType_t samplePeriod_ms
 */

/**
 * @brief Reads the given ADC channel's latest value.
 *
 * @param channel The channel.
 *
 * @return The read value.
 */
uint32_t adcRead(adcChannel_t channel) {
    (void)channel;
    return 0;
}
