/**
 * @file gpio.c
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h>  // HAL interface
#include <FreeRTOS.h>   // configASSERT()

#include <CMR/gpio.h>   // GPIO interface

#include "gpio.h"   // Interface to implement

/** @brief Board-specific pin configuration. */
static const cmr_gpioPinConfig_t gpioPinConfigs[] = {
    [GPIO_LED_STATUS] = {
        .port = GPIOB,
        .init = {
            .Pin = GPIO_PIN_6,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    // XXX
    [GPIO_LED_BSPD] = {
        .port = GPIOA,
        .init = {
            .Pin = GPIO_PIN_6,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    // XXX
    [GPIO_LED_AMS] = {
        .port = GPIOA,
        .init = {
            .Pin = GPIO_PIN_7,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    // XXX
    [GPIO_LED_IMD] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_10,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    }
};

/**
 * @brief Initializes the GPIO interface.
 */
void gpioInit(void) {
    cmr_gpioPinInit(
        gpioPinConfigs, sizeof(gpioPinConfigs) / sizeof(gpioPinConfigs[0])
    );
}

/**
 * @brief Writes a value to an output GPIO pin.
 *
 * @param pin The pin to write to.
 * @param value The value to write (zero for off; non-zero for on).
 */
void gpioWrite(gpio_t pin, int value) {
    configASSERT(pin < GPIO_LEN);

    const cmr_gpioPinConfig_t *pinConfig = gpioPinConfigs + pin;
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
void gpioToggle(gpio_t pin) {
    configASSERT(pin < GPIO_LEN);

    const cmr_gpioPinConfig_t *pinConfig = gpioPinConfigs + pin;
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
int gpioRead(gpio_t pin) {
    configASSERT(pin < GPIO_LEN);

    const cmr_gpioPinConfig_t *pinConfig = gpioPinConfigs + pin;
    GPIO_PinState state = HAL_GPIO_ReadPin(
        pinConfig->port, pinConfig->init.Pin
    );
    if (state == GPIO_PIN_RESET) {
        return 0;
    }

    return 1;
}

