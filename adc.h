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
	ADC_AIR_POWER  = 1,
	ADC_SAFETY     = 2,
	ADC_HV         = 3,
	ADC_BATT       = 4,
    ADC_SHUNT_P    = 5,
	ADC_SHUNT_N    = 6,
	ADC_LV_CURR    = 7,
	ADC_REF1V65    = 8,
	ADC_LEN     /**< @brief Total ADC channels. */
} adcChannels_t;

static int16_t ADCChannelPolarity[ADC_LEN] = {1, 1, 1, -1, -1, 1, -1, 1, 1};

extern cmr_adcChannel_t adcChannels[ADC_LEN];

void adcInit(void);

#endif /* ADC_H */

