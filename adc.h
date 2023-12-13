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
    ADC_TPOS_L = 0,     /**< @brief Sensor channel 1. - TPOS L */
    ADC_TPOS_R,     /**< @brief Sensor channel 2. - TPOS R */
    ADC_BPRES,      /**< @brief Sensor channel 3. - BPRES */
    ADC_SWANGLE,    /**< @brief Sensor channel 4. - SWANGLE */
    ADC_LEN         /**< @brief Total ADC channels. */
} adcChannel_t;

void adcInit(void);
uint32_t adcRead(adcChannel_t channel);

#endif /* ADC_H */

