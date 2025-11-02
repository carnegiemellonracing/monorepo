/**
 * @file gpio.c
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h>  // HAL interface
#include <FreeRTOS.h>   // configASSERT()

#include <CMR/gpio.h>   // GPIO interface
// #include <CMR/pwm.h>

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

    // [GPIO_OUT_LED_GREEN] = {
    //     .port = GPIOA,
    //     .init = {
    //         .Pin = GPIO_PIN_10,
    //         .Mode = GPIO_MODE_OUTPUT_PP,
    //         .Pull = GPIO_NOPULL,
    //         .Speed = GPIO_SPEED_FREQ_LOW
    //     }
    // },

    // [GPIO_OUT_LED_FLASH_RED] = {
    //     .port = GPIOA,
    //     .init = {
    //         .Pin = GPIO_PIN_9,
    //         .Mode = GPIO_MODE_OUTPUT_PP,
    //         .Pull = GPIO_NOPULL,
    //         .Speed = GPIO_SPEED_FREQ_LOW
    //     }
    // },

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
            .Pin = GPIO_PIN_8,
            .Mode = GPIO_MODE_INPUT,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },

    [GPIO_IN_EBS_ACTIVATED] = {
        .port = GPIOB,
        .init = {
            .Pin = GPIO_PIN_12, /** @todo implement */
            .Mode = GPIO_MODE_INPUT,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    }
};

cmr_pwmPinConfig_t pwmPinConfig1 = {
    .port = GPIOA,
    .pin = GPIO_PIN_9,
    .channel = TIM_CHANNEL_2,
    .presc = 24,
    .period_ticks = 1000,
    .timer = TIM1
};

cmr_pwmPinConfig_t pwmPinConfig2 = {
    .port = GPIOA,
    .pin = GPIO_PIN_10,
    .channel = TIM_CHANNEL_3,
    .presc = 24,
    .period_ticks = 1000,
    .timer = TIM1
};

/**
 * @brief Initializes the GPIO interface.
 */
void gpioInit(void) {
    cmr_gpioPinInit(
        gpioPinConfigs, sizeof(gpioPinConfigs) / sizeof(gpioPinConfigs[0])
    );

    cmr_pwmInit(&LED_Red, &pwmPinConfig2);
    cmr_pwmInit(&LED_Green, &pwmPinConfig1);
    cmr_pwmSetDutyCycle(&LED_Red, 0);
    cmr_pwmSetDutyCycle(&LED_Green, 0);

    cmr_gpioWrite(GPIO_OUT_RTD_SIGNAL, 0);
    cmr_gpioWrite(GPIO_OUT_SOFTWARE_ERR, 0);
}

