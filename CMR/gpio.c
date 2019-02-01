/**
 * @file gpio.c
 * @brief General purpose input/output wrapper implementation.
 */

#include "gpio.h"   // Interface to implement

#ifdef HAL_GPIO_MODULE_ENABLED

#include "rcc.h"    // cmr_rccGPIOClockEnable()
#include "FreeRTOSConfig.h" // configASSERT

static const cmr_gpioPinConfig_t *gpioPinConfigs;
static size_t gpioPinConfigsLen;

/**
 * @brief Configures the specified GPIO pin(s).
 *
 * @param pinConfigs The pin configuration(s).
 * @param pinConfigsLen The number of pin configurations.
 */
void cmr_gpioPinInit(const cmr_gpioPinConfig_t *pinConfigs, const size_t pinConfigsLen) {
    gpioPinConfigs = pinConfigs;
    gpioPinConfigsLen = pinConfigsLen;

    for (size_t i = 0; i < gpioPinConfigsLen; i++) {
        const cmr_gpioPinConfig_t *pinConfig = &gpioPinConfigs[i];
        cmr_rccGPIOClockEnable(pinConfig->port);

        // The HAL GPIO driver doesn't actually declare the initialization
        // struct as `const`, but it doesn't modify it either.
        HAL_GPIO_Init(
            pinConfig->port,
            (GPIO_InitTypeDef *) &pinConfig->init
        );
    }
}

/**
 * @brief Writes a value to an output GPIO pin.
 *
 * @param pin The pin to write to.
 * @param value The value to write (zero for off; non-zero for on).
 */
void cmr_gpioWrite(size_t pin, int value) {
    configASSERT(pin < gpioPinConfigsLen);

    const cmr_gpioPinConfig_t *pinConfig = &gpioPinConfigs[pin];
    configASSERT(
        (pinConfig->init.Mode == GPIO_MODE_OUTPUT_PP) ||
        (pinConfig->init.Mode == GPIO_MODE_OUTPUT_OD)
    );

    HAL_GPIO_WritePin(
        pinConfig->port, pinConfig->init.Pin,
        value ? GPIO_PIN_SET : GPIO_PIN_RESET
    );
}

/**
 * @brief Toggles an output GPIO pin's value.
 *
 * @param pin The pin to toggle.
 */
void cmr_gpioToggle(size_t pin) {
    configASSERT(pin < gpioPinConfigsLen);

    const cmr_gpioPinConfig_t *pinConfig = &gpioPinConfigs[pin];
    configASSERT(
        (pinConfig->init.Mode == GPIO_MODE_OUTPUT_PP) ||
        (pinConfig->init.Mode == GPIO_MODE_OUTPUT_OD)
    );

    HAL_GPIO_TogglePin(pinConfig->port, pinConfig->init.Pin);
}

/**
 * @brief Reads a value from a GPIO pin.
 *
 * @return 0 if the pin was off; otherwise 1.
 */
int cmr_gpioRead(size_t pin) {
    configASSERT(pin < gpioPinConfigsLen);

    const cmr_gpioPinConfig_t *pinConfig = &gpioPinConfigs[pin];
    GPIO_PinState state = HAL_GPIO_ReadPin(
        pinConfig->port, pinConfig->init.Pin
    );
    if (state == GPIO_PIN_RESET) {
        return 0;
    }

    return 1;
}

/**
 * @brief Atomically sets and resets the specified pins on the given port.
 *
 * @note See STM32F413 RM0430 7.4.7 for more implementation details.
 *
 * @param port GPIO port to modify (i.e. `GPIOx` from `stm32f413xx.h`);
 * @param set Bitwise-OR of `GPIO_PIN_x` (from `stm32f4xx_hal_gpio.h`) to set.
 * @param reset Bitwise-OR of `GPIO_PIN_x` to reset.
 */
void cmr_gpioBSRR(GPIO_TypeDef *port, uint16_t set, uint16_t reset) {
    configASSERT(IS_GPIO_ALL_INSTANCE(port));

    port->BSRR = (((uint32_t) reset) << 16) | (uint32_t) set;
}


#endif /* HAL_GPIO_MODULE_ENABLED */

