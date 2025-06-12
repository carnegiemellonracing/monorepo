/**
 * @file adc.h
 * @brief Board-specific ADC interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef ADC_H
#define ADC_H

#include <CMR/adc.h> // ADC interface

/**
 * @brief Represents an ADC channel.
 *
 * @warning New channels MUST be added before `ADC_LEN`.
 */
typedef enum {
  ADC_LINPOT1,
  ADC_LINPOT2,
  ADC_THERM1,
  ADC_THERM2,
  ADC_LEN         /**< @brief Total ADC channels. */
} adcChannel_t;

void adcInit(void);
uint32_t adcRead(adcChannel_t ch);

#endif /* ADC_H */
