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
	ADC_V24V       = 0,     
	ADC_HV_PLUS    = 1,
	ADC_HV_MINUS   = 2,
	ADC_BATT_PLUS  = 3,
    ADC_BATT_MINUS = 4,
	ADC_SHUNT_LV   = 5,
	ADC_AIR_POWER  = 6,
	ADC_DCDC_CURR  = 7,
	ADC_LV_CURR    = 8,
	ADC_LEN     /**< @brief Total ADC channels. */
} adcChannels_t;

extern cmr_adcChannel_t adcChannels[ADC_LEN];

void adcInit(void);

#endif /* ADC_H */

