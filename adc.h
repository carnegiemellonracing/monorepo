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
    ADC_VSENSE = 0, /**< @brief Board voltage sense. */
    ADC_ISENSE,     /**< @brief Board current sense. */
    ADC_CH1,        /**< @brief Sensor channel 1. */
    ADC_CH2,        /**< @brief Sensor channel 2. */
    ADC_CH3,        /**< @brief Sensor channel 3. */
    ADC_CH4,        /**< @brief Sensor channel 4. */
    ADC_CH5,        /**< @brief Sensor channel 5. */
    ADC_LEN         /**< @brief Total ADC channels. */
} adcChannel_t;

void adcInit(void);
uint32_t adcRead(adcChannel_t channel);

#endif /* ADC_H */

