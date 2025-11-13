//initializes gpio pins


/**
 * @file gpio.c
 * @brief Board-specific GPIO interface.
 *
 * @author Jasmine Li
 */

#include <stm32f4xx_hal.h>  // HAL interface
#include <FreeRTOS.h>   // configASSERT() ---> real time operating system for embedded systems
 
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

    [GPIO_LED_STATUS] = { //standard ouptut
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_1,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    }
};
 
/**
 * @brief Initializes the GPIO interface.
 */
void gpioInit() {
    cmr_gpioPinInit(
        gpioPinConfigs, sizeof(gpioPinConfigs) / sizeof(gpioPinConfigs[0])
    );
}
