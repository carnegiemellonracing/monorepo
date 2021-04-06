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
    ADC_VSENSE = 0,     /**< @brief Board voltage sense. */
    ADC_ISENSE,         /**< @brief Board current sense. */
    ADC_LEN     /**< @brief Total ADC channels. */
} adcChannels_t;

void adcInit(void);

uint16_t adcRead(adcChannels_t channel);

#endif /* ADC_H */

