/**
 * @file adc.h
 * @brief Board-specific ADC interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef ADC_H
#define ADC_H

#include <CMR/adc.h>    // ADC interface

/**
 * @brief Represents an ADC channel.
 *
 * @warning New channels MUST be added before `ADC_LEN`.
 */
typedef enum {
    ADC_THERM_1 = 0,    /**< @brief Thermistor 1 sense. */
    ADC_THERM_2,        /**< @brief Thermistor 2 sense. */
    ADC_LEN             /**< @brief Total ADC channels. */
} adcChannel_t;

void adcInit(void);
uint32_t adcRead(adcChannel_t channel);

#endif /* ADC_H */
