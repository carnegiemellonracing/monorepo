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
	V24V       = 0,     
	HV_PLUS    = 1,
	HV_MINUS   = 2,
	BATT_PLUS  = 3,
    BATT_MINUS = 4,
	SHUNT_LV   = 5,
	AIR_POWER  = 6,
	DCDC_CURR  = 7,
	LV_CURR    = 8,
	ADC_LEN     /**< @brief Total ADC channels. */
} adcChannels_t;

extern cmr_adcChannel_t adcChannels[ADC_LEN];

void adcInit(void);

#endif /* ADC_H */

