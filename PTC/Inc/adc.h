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
    ADC_POWER_VSENSE = 0,   /**< @brief Power-side voltage sense.        */
    ADC_POWER_ISENSE,       /**< @brief Power-side current sense.        */
    ADC_LOGIC_VSENSE,       /**< @brief Logic-side voltage sense.        */
    /* Logic-side current sense has been omitted from the pcb */
    ADC_BOARD_THERM_1,      /**< @brief On-board thermistor 1.      */
    ADC_BOARD_THERM_2,      /**< @brief On-board thermistor 2.      */
	ADC_BOARD_THERM_3,
	ADC_BOARD_THERM_4,
	ADC_BOARD_THERM_5,
	ADC_BOARD_THERM_6,
    ADC_LEN     /**< @brief Total ADC channels. */
} adcChannel_t;

void adcInit(void);
uint32_t adcRead(adcChannel_t channel);

#endif /* ADC_H */

