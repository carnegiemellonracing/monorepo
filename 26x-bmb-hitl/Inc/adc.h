/**
 * @file adc.h
 * @brief ADC implementation for 26x BMB HITL.
 *
 * @author Ayush Garg
 * modified by Yi-An Liao
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
	ADC_CELL_1,
    ADC_CELL_2,
    ADC_CELL_3,
    ADC_CELL_4,
    ADC_CELL_5,
    ADC_CELL_6,
    ADC_CELL_7,
    ADC_CELL_8,
    ADC_CELL_9,
    ADC_CELL_10,
    ADC_CELL_11,
    ADC_CELL_12,
    ADC_CELL_13,
    ADC_CELL_14,
    ADC_CELL_15,
    ADC_CELL_16,
    ADC_CELL_17,
    ADC_CELL_18,
	ADC_LEN                 /**< @brief Total ADC channels. */
} adcChannel_t;

void adcInit(void);
uint32_t adcRead(adcChannel_t channel);

#endif /* ADC_H */