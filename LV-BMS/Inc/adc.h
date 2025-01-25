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
	ADC_VTHERM_PIN1 = 0,
	ADC_VTHERM_PIN2,
	ADC_AFE_VCOUT,
	ADC_AFE_VIOUT,
	ADC_NUM     /**< @brief Total ADC channels. */
} adc_channel_t;

static const uint32_t VTHERM_NUM = 16;

static const TickType_t adc_period_ms = 50;

void adcInit(void);
uint32_t adcRead(adc_channel_t ch);

#endif /* ADC_H */

