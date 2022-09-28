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
	ADC_V24V            = 0,
	ADC_AIR_POWER       = 1,
	ADC_SAFETY          = 2,
	ADC_LEN     /**< @brief Total ADC channels. */
} adcChannels_t;


extern cmr_adcChannel_t adcChannels[ADC_LEN];

void adcInit(void);

uint32_t adcRead(adcChannels_t ch);

#endif /* ADC_H */

