/**
 * @file gpio.c
 * @brief General purpose input/output wrapper implementation.
 */

#include "gpio.h"   // Interface to implement

#ifdef HAL_GPIO_MODULE_ENABLED

#include "rcc.h"    // cmr_rccGPIOClockEnable()

/**
 * @brief Configures the specified GPIO pin(s).
 *
 * @param pinConfigs The pin configuration(s).
 * @param pinConfigsLen The number of pin configurations.
 */
void cmr_gpioPinInit(
    const cmr_gpioPinConfig_t *pinConfigs, size_t pinConfigsLen
) {
    for (size_t i = 0; i < pinConfigsLen; i++) {
        const cmr_gpioPinConfig_t *pinConfig = pinConfigs + i;
        cmr_rccGPIOClockEnable(pinConfig->port);

        // The HAL GPIO driver doesn't actually declare the initialization
        // struct as `const`, but it doesn't modify it either.
        HAL_GPIO_Init(
            pinConfig->port,
            (GPIO_InitTypeDef *) &pinConfig->init
        );
    }
}

#endif /* HAL_GPIO_MODULE_ENABLED */

