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
    ADC_AVG_OUT = 0,        /**< @brief Pilot circuit duty cycle */
    ADC_PEAK_OUT,       /**< @brief Pilot circuit amplitude detection */
    ADC_THERM_1,
    ADC_THERM_2,
    ADC_SAFETY,
    ADC_LEN     /**< @brief Total ADC channels. */
} adcChannel_t;

void adcInit(void);
uint32_t adcRead(adcChannel_t channel);

#endif /* ADC_H */

