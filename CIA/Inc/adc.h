/**
 * @file adc.h
 * @brief Board-specific ADC interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef ADC_H
#define ADC_H

#include <CMR/adc.h>  // ADC interface

/**
 * @brief Represents an ADC channel.
 *
 * @warning New channels MUST be added before `ADC_LEN`.
 */
typedef enum {
    ADC_AMP_IN_1 = 0,
    ADC_AMP_IN_2,
    ADC_AMP_IN_3,
    ADC_AMP_IN_4,
    ADC_AMP_IN_5,
    ADC_NTC_IN,
    ADC_LEN         /**< @brief Total ADC channels. */
} adcChannel_t;

void adcInit(void);
uint32_t adcRead(adcChannel_t channel);

#endif /* ADC_H */
