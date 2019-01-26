/**
 * @file adc.h
 * @brief Analog-to-digital converter interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_ADC_H
#define CMR_ADC_H

#include <stddef.h>     // size_t
#include <stdint.h>     // uint32_t
#include <stm32f4xx_hal.h>  // ADC_HandleTypeDef, ADC_TypeDef, GPIO_TypeDef

/** @brief Number of channels per ADC. */
#define CMR_ADC_CHANNELS 16

/**
 * Represents an ADC channel.
 */
typedef struct {
    volatile uint32_t value;    /**< @brief The most recently-sampled value. */
} cmr_adcChannel_t;

/**
 * @brief Represents an ADC.
 *
 * @note The contents of this struct are opaque to the library consumer.
 */
typedef struct {
    ADC_HandleTypeDef handle;   /**< @brief HAL ADC handle. */
    size_t channelsUsed;        /**< @brief Number of configured channels. */

    /** @brief Channels. */
    cmr_adcChannel_t channels[CMR_ADC_CHANNELS];
} cmr_adc_t;

void cmr_adcInit(cmr_adc_t *adc, ADC_TypeDef *instance);

const cmr_adcChannel_t *cmr_adcAddChannel(cmr_adc_t *adc, uint32_t channel,
                                          uint32_t samplingTime,
                                          GPIO_TypeDef *port, uint16_t pin);

void cmr_adcSample(cmr_adc_t *adc);

#endif /* CMR_ADC_H */

