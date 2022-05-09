/**
 * @file gpio.c
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h>  // HAL interface
#include <FreeRTOS.h>   // configASSERT()

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
    [GPIO_MCU_LED] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_10,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_HV_ACTIVE_TSAL_LIGHT] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_5,
            .Mode = GPIO_MODE_INPUT,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_BMB_POWER_ENABLE_L] = {
        .port = GPIOB,
        .init = {
            .Pin = GPIO_PIN_9,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_PULLUP,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_BMB_WAKE_PIN] = {
        .port = GPIOB,
        .init = {
            .Pin = GPIO_PIN_14,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_BMB_FAULT_L] = {
        .port = GPIOB,
        .init = {
            .Pin = GPIO_PIN_10,
            .Mode = GPIO_MODE_INPUT,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_CLEAR_FAULT_L] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_9,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_GP2_24V_EN] = { // Not used now but can be used later
        .port = GPIOA,
        .init = {
            .Pin = GPIO_PIN_9,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_GP1_24V_EN] = { // Not used now but can be used later
        .port = GPIOA,
        .init = {
            .Pin = GPIO_PIN_10,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_AIR_FAULT_L] = {
        .port = GPIOB,
        .init = {
            .Pin = GPIO_PIN_15,
            .Mode = GPIO_MODE_INPUT,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_DISCHARGE_EN] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_8,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_PRECHARGE_EN] = {
        .port = GPIOA,
        .init = {
            .Pin = GPIO_PIN_8,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_AIR_POSITIVE_EN] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_7,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_AIR_NEGATIVE_EN] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_6,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_HVSENSE_DRDY_L] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_4,
            .Mode = GPIO_MODE_INPUT,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
	[SAFETY_BINARY] = {
	        .port = GPIOC,
	        .init = {
	            .Pin = GPIO_PIN_1,
	            .Mode = GPIO_MODE_INPUT,
	            .Pull = GPIO_PULLUP,
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
    cmr_gpioWrite(GPIO_BMB_POWER_ENABLE_L, 1);

}

