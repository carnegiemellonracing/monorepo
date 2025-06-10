

#include <stm32f4xx_hal.h>  // HAL interface
#include <FreeRTOS.h>   // configASSERT()

#include "gpio.h"   // Interface to implement


static const cmr_gpioPinConfig_t gpioPinConfigs[GPIO_LEN] = {
    [DIGITAL_IN_24V_MCU1_MC] = {
        .port = GPIOA,
        .init = {
            .Pin = GPIO_PIN_0,
            .Mode = GPIO_MODE_INPUT,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [DIGITAL_IN_24V_MCU2_BRAKELIGHT] = {
        .port = GPIOA,
        .init = {
            .Pin = GPIO_PIN_1,
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
}
