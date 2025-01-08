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

/**
 * @brief Board-specific pin configuration.
 *
 * Replace/add more pin configurations here as appropriate. Each enumeration
 * value of `gpio_t` should get a configuration.
 *
 * @see `stm32f4xx_hal_gpio.h` for various initialization values.
 */
static const cmr_gpioPinConfig_t gpioPinConfigs[GPIO_LEN] = {
    [GPIO_OUT_LED_STATUS] = {
        .port = GPIOB,
        .init = {
            .Pin = GPIO_PIN_4,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },

    [GPIO_OUT_DCDC_EN] = {
        .port = GPIOA,
        .init = {
            .Pin = GPIO_PIN_4,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },

    [GPIO_OUT_SOFTWARE_ERR] = {
        .port = GPIOA,
        .init = {
            .Pin = GPIO_PIN_6,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },

    [GPIO_OUT_RTD_SIGNAL] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_6,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },

    [GPIO_IN_SOFTWARE_ERR] = {
        .port = GPIOB,
        .init = {
            .Pin = GPIO_PIN_5,
            .Mode = GPIO_MODE_INPUT,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },

    [GPIO_IN_BSPD_ERR] = {
        .port = GPIOB,
        .init = {
            .Pin = GPIO_PIN_7,
            .Mode = GPIO_MODE_INPUT,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },

    [GPIO_IN_IMD_ERR] = {
        .port = GPIOB,
        .init = {
            .Pin = GPIO_PIN_6,
            .Mode = GPIO_MODE_INPUT,
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

    cmr_gpioWrite(GPIO_OUT_DCDC_EN, 0);
    cmr_gpioWrite(GPIO_OUT_RTD_SIGNAL, 0);
    cmr_gpioWrite(GPIO_OUT_SOFTWARE_ERR, 0);
}

