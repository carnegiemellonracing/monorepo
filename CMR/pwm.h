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

    /** @brief The capture-compare channel used for the given pin.
     *   Must be a value of `TIM_CHANNEL_x` from stm32f4xx_hal_tim.h. */
    uint32_t channel;

    /** @brief How much to divide the timer's peripheral clock by. Maximum 65536 (2^16). */
    uint32_t presc;

    /** @brief PWM period in timer ticks (after clock division). Maximum 65536 (2^16). */
    uint32_t period_ticks;

    /** @brief The timer used for the given pin. Must be a value of`TIMx` from stm32f413xx.h. */
    TIM_TypeDef *timer;
} cmr_pwmPinConfig_t;

/**
 * @brief Represents a single PWM pin.
 *
 * @note The contents of this struct are opaque to the library consumer. 
 */
typedef struct {
    TIM_HandleTypeDef handle;   /**< @brief HAL TIM handle. */
    uint32_t channel;           /**< @brief HAL TIM output compare channel. */
} cmr_pwmChannel_t;

void cmr_pwmInit(const cmr_pwmPinConfig_t *pinConfig,
                 cmr_pwmChannel_t *pwmChannel);

void cmr_pwmSetDutyCycle(cmr_pwmChannel_t *pwmChannel, uint32_t dutyCycle_pcnt);

#endif /* HAL_TIM_MODULE_ENABLED */

#endif /* CMR_PWM_H */
