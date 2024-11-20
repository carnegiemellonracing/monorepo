/**
 * @file gpio.c
 * @brief General purpose input/output wrapper implementation.
 *
 * The interrupt handler definitions override the default handlers for each
 * interrupt, which spins forever.
 *
 * The default handlers are (weakly) defined in `CMSIS/startup_stm32f413xx.s`.
 *
 * @author Carnegie Mellon Racing
 */

#include <CMR/gpio.h>   // Interface to implement

#ifdef HAL_GPIO_MODULE_ENABLED

#include <CMR/rcc.h>    // cmr_rccGPIOClockEnable()
#include <FreeRTOSConfig.h> // configASSERT

static const cmr_gpioPinConfig_t *cmr_gpioPinConfigs;
static size_t cmr_gpioPinConfigsLen;

/**
 * @brief Defines the EXTI IRQ handler for the given name and pins.
 *
 * @param name The EXTI name.
 * @param pins The relevant GPIO pins.
 */
#define EXTI_IRQHandler(name, pins) \
    void EXTI##name##_IRQHandler(void) { \
        HAL_GPIO_EXTI_IRQHandler(pins); \
    }
EXTI_IRQHandler(0, GPIO_PIN_0)
EXTI_IRQHandler(1, GPIO_PIN_1)
EXTI_IRQHandler(2, GPIO_PIN_2)
EXTI_IRQHandler(3, GPIO_PIN_3)
EXTI_IRQHandler(4, GPIO_PIN_4)
EXTI_IRQHandler(
    9_5,
    GPIO_PIN_9 | GPIO_PIN_8 | GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5
)
EXTI_IRQHandler(
    15_10,
    GPIO_PIN_15 | GPIO_PIN_14 | GPIO_PIN_13 |
    GPIO_PIN_12 | GPIO_PIN_11 | GPIO_PIN_10
)
#undef EXTI_IRQHandler

/**
 * @brief Configures the specified GPIO pin(s).
 *
 * @param pinConfigs The pin configuration(s).
 * @param pinConfigsLen The number of pin configurations.
 */
void cmr_gpioPinInit(const cmr_gpioPinConfig_t *pinConfigs, const size_t pinConfigsLen) {
    cmr_gpioPinConfigs = pinConfigs;
    cmr_gpioPinConfigsLen = pinConfigsLen;

    for (size_t i = 0; i < cmr_gpioPinConfigsLen; i++) {
        const cmr_gpioPinConfig_t *pinConfig = &cmr_gpioPinConfigs[i];
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
    configASSERT(pin < cmr_gpioPinConfigsLen);

    const cmr_gpioPinConfig_t *pinConfig = &cmr_gpioPinConfigs[pin];
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
    configASSERT(pin < cmr_gpioPinConfigsLen);

    const cmr_gpioPinConfig_t *pinConfig = &cmr_gpioPinConfigs[pin];
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
    configASSERT(pin < cmr_gpioPinConfigsLen);

    const cmr_gpioPinConfig_t *pinConfig = &cmr_gpioPinConfigs[pin];
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

