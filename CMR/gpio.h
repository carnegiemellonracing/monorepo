/**
 * @file gpio.h
 * @brief General purpose input/output interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_GPIO_H
#define CMR_GPIO_H

#include <stm32f4xx_hal.h>  // HAL_GPIO_MODULE_ENABLED, GPIO_TypeDef

#ifdef HAL_GPIO_MODULE_ENABLED

/** @brief GPIO pin configuration. */
typedef struct {
    /** @brief HAL GPIO port (`GPIOx` from `stm32f413xx.h`). */
    GPIO_TypeDef *port;

    /** @brief HAL GPIO pin initialization. */
    GPIO_InitTypeDef init;
} cmr_gpioPinConfig_t;

void cmr_gpioPinInit(
    const cmr_gpioPinConfig_t *pinConfigs, size_t pinConfigsLen
);

#endif /* HAL_GPIO_MODULE_ENABLED */

#endif /* CMR_GPIO_H */

