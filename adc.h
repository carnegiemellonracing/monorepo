/**
 * @file adc.h
 * @brief Board-specific ADC interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef ADC_H
#define ADC_H

#include <CMR/adc.h>    // ADC interface

extern const cmr_adcChannel_t *adcVSense;
extern const cmr_adcChannel_t *adcISense;

void adcInit(void);

#endif /* ADC_H */

