/**
 * @file adc.h
 * @brief Board-specific ADC interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef ADC_H
#define ADC_H

#include <CMR/adc.h>    // ADC interface

typedef enum {
	ADC_VSENSE = 0,
	ADC_ISENSE,
	ADC_LEN
} adcChannels_t;

cmr_adcChannel_t adcChannels[ADC_LEN];

void adcInit(void);

#endif /* ADC_H */

