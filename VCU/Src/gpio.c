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
 * TODO: Verify all GPIO ports, pins, modes, pulls, and speeds against the
 * new combined VCU board schematic. Some VSM and DCM pins overlap and will
 * need to be reconciled.
 *
 * @see `stm32h7xx_hal_gpio.h` for various initialization values.
 */
static const cmr_gpioPinConfig_t gpioPinConfigs[GPIO_LEN] = {

    //DCM GPIOs

    [GPIO_LED_STATUS] = {
        .port = GPIOA,
        .init = {
            .Pin = GPIO_PIN_2,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },

    [GPIO_BRKLT_ENABLE] = {
        .port = GPIOG,
        .init = {
            .Pin = GPIO_PIN_11,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },

    [GPIO_FAN_ON] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_12,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },

    [GPIO_FAN_1] = {
        .port = GPIOB,
        .init = {
            .Pin = GPIO_PIN_5,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },

    [GPIO_FAN_2] = {
        .port = GPIOG,
        .init = {
            .Pin = GPIO_PIN_14,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },

    [GPIO_PUMP_LEFT] = {
        .port = GPIOB,
        .init = {
            .Pin = GPIO_PIN_6,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },

    [GPIO_PUMP_RIGHT] = {
        .port = GPIOG,
        .init = {
            .Pin = GPIO_PIN_13,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },

    [GPIO_PUMP_ON] = {
        .port = GPIOA,
        .init = {
            .Pin = GPIO_PIN_0,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },

    [GPIO_AUXILIARY_ENABLE] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_7,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },

    [GPIO_MTR_CTRL_ENABLE] = {
        .port = GPIOG,
        .init = {
            .Pin = GPIO_PIN_10,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },


    //VSM GPIOS

    [GPIO_OUT_LED_STATUS] = {
        .port = GPIOB,
        .init = {
            .Pin = GPIO_PIN_4,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },

    [GPIO_OUT_SOFTWARE_ERR_N] = {
        .port = GPIOB,
        .init = {
            .Pin = GPIO_PIN_5,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },

    [GPIO_OUT_AMS_ERR_N] = {
        .port = GPIOB,
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

    [GPIO_IN_SOFTWARE_ERR_N] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_12,
            .Mode = GPIO_MODE_INPUT,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },

    [GPIO_IN_BSPD_ERR_N] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_10,
            .Mode = GPIO_MODE_INPUT,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },

    [GPIO_IN_IMD_ERR_N] = {
        .port = GPIOB,
        .init = {
            .Pin = GPIO_PIN_7,
            .Mode = GPIO_MODE_INPUT,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },

    [GPIO_IN_IMD_ERR_COND_N] = {
        .port = GPIOB,
        .init = {
            .Pin = GPIO_PIN_8,
            .Mode = GPIO_MODE_INPUT,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },

    [GPIO_IN_BSPD_ERR_UNLATCH] = {
        .port = GPIOB,
        .init = {
            .Pin = GPIO_PIN_9,
            .Mode = GPIO_MODE_INPUT,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },

    [GPIO_IN_EAB] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_7,
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
        gpioPinConfigs,
        sizeof(gpioPinConfigs) / sizeof(gpioPinConfigs[0])
    );

    /* DCM initialization */
    cmr_gpioWrite(GPIO_AUXILIARY_ENABLE, 0);

    /* VSM initialization */
    cmr_gpioWrite(GPIO_OUT_RTD_SIGNAL, 0);
    cmr_gpioWrite(GPIO_OUT_SOFTWARE_ERR_N, 1);
}
