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
	ADC_HALL_EFFECT = 0,    /**< @brief Hall effect sense. */
	ADC_REAR_BRAKE_PRES,    /**< @brief Rear brake pressure sense. */
    ADC_VSENSE,             /**< @brief Board voltage sense. */
    ADC_ISENSE,             /**< @brief Board current sense. */
	ADC_LEN     /**< @brief Total ADC channels. */
} adcChannel_t;

void adcInit(void);
uint32_t adcRead(adcChannel_t channel);

#endif /* ADC_H */

