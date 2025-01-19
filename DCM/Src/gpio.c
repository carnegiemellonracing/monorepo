/**
 * @file gpio.c
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32h7xx_hal.h>  // HAL interface
#include <FreeRTOS.h>       // configASSERT()

#include <CMR/gpio.h>       // GPIO interface

#include "gpio.h"           // Interface to implement

/**
 * @brief Board-specific pin configuration.
 *
 * Replace/add more pin configurations here as appropriate. Each enumeration
 * value of `gpio_t` should get a configuration.
 *
 * @see `stm32f4xx_hal_gpio.h` for various initialization values.
 */
static const cmr_gpioPinConfig_t gpioPinConfigs[GPIO_LEN] = {
    [GPIO_LED_STATUS] = {
        .port = GPIOA,
        .init = {
            .Pin = GPIO_PIN_0,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_FRAM_WP] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_3,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_PULLDOWN,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_MTR_CTRL_ENABLE] = {
        .port GPIOC,
        .init = {
            .Pin = GPIO_PIN_10,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_PULLDOWN,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_MC_EFUSE_AUTO] = {
        .port GPIOC,
        .init = {
            .Pin = GPIO_PIN_10,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_PULLDOWN,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_BRKLT_ENABLE] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_12,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_PULLDOWN,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    }
};

// mcPower = c10, brakelight = c12

/**
 * @brief Initializes the GPIO interface.
 */
void gpioInit(void) {
    cmr_gpioPinInit(
        gpioPinConfigs, sizeof(gpioPinConfigs) / sizeof(gpioPinConfigs[0])
    );
}

