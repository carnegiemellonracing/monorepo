/**
 * @file adc.h
 * @brief Analog-to-digital converter interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_ADC_H
#define CMR_ADC_H

#include <stm32f4xx_hal.h>  // HAL_ADC_MODULE_ENABLED, ADC_HandleTypeDef,
                            // ADC_TypeDef, GPIO_TypeDef

#ifdef HAL_ADC_MODULE_ENABLED

#include <stddef.h>     // size_t
#include <stdint.h>     // uint32_t

/** @brief Number of channels per ADC. */
#define CMR_ADC_CHANNELS 16

/** @brief Maximum 12-bit ADC value. */
#define CMR_ADC_MAX (uint32_t)((1 << 12) - 1)

/** @brief Represents an ADC channel.  */
typedef struct {
    /**
     * @brief The ADC channel to configure.
     *
     * One of `ADC_CHANNEL_x` from `stm32f4xx_hal_adc.h`, 0 <= x <= 15.
     */
    const uint32_t channel;

    /** @brief Channel's GPIO port (`GPIOx` from `stm32f413xx.h`). */
    GPIO_TypeDef *const port;

    /** @brief Channel's GPIO pin (`GPIO_PIN_x` from `stm32f4xx_hal_gpio.h`). */
    const uint16_t pin;

    /**
     * @brief Sampling time for ADC channel
     *
     * One of `ADC_SAMPLETIME_xCYCLES`, from `stm32f4xx_hal_adc.h`.
     * @see RM0430 13.7 for minimum sample times.
     */
    const uint32_t samplingTime;

    /** @brief The most recently-sampled value. */
    volatile uint32_t value;
} cmr_adcChannel_t;

/**
 * @brief Represents an ADC.
 *
 * @note The contents of this struct are opaque to the library consumer.
 */
typedef struct {
    ADC_HandleTypeDef handle;       /**< @brief HAL ADC handle. */
    cmr_adcChannel_t *channels;     /**< @brief Configured ADC channels. */
    size_t channelsLen;             /**< @brief Number of ADC channels. */
} cmr_adc_t;

void cmr_adcInit(
    cmr_adc_t *adc, ADC_TypeDef *instance,
    cmr_adcChannel_t *channels, const size_t channelsLen
);

#endif /* HAL_ADC_MODULE_ENABLED */

#endif /* CMR_ADC_H */

