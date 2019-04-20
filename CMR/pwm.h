/**
 * @file pwm.h
 * @brief Pulse Width Modulation port.
 * 
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_PWM_H
#define CMR_PWM_H

#include <stm32f4xx_hal.h>  // HAL_TIM_MODULE_ENABLED, GPIO_TypeDef

#ifdef HAL_TIM_MODULE_ENABLED

/** @brief Represents a single PWM pin. */
typedef struct {
    /** @brief Pin's GPIO port (`GPIOx` from `stm32f413xx.h`). */
    GPIO_TypeDef *port;

    /** @brief Channel's GPIO pin (`GPIO_PIN_x` from `stm32f4xx_hal_gpio.h`). */
    uint16_t pin;
} cmr_pwmPin_t;

/**
 * @brief Represents a single PWM pin.
 *
 * @note The contents of this struct are opaque to the library consumer. 
 */
typedef struct {
    TIM_HandleTypeDef handle;   /**< @brief HAL TIM handle. */
    uint32_t channel;           /**< @brief HAL TIM output compare channel. */
} cmr_pwmChannel_t;

void cmr_pwmInit(const cmr_pwmPin_t *pin,
                 cmr_pwmChannel_t *pwmChannel,
                 TIM_TypeDef *timer,
                 const uint32_t channel,
                 const uint16_t presc,
                 const uint16_t period_ticks);

void cmr_pwmSetDutyCycle(cmr_pwmChannel_t *pwmChannel, uint32_t dutyCycle_pcnt);

#endif /* HAL_TIM_MODULE_ENABLED */

#endif /* CMR_PWM_H */
