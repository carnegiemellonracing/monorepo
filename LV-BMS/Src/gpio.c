/**
 * @file gpio.c
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h> // HAL interface
#include <CMR/gpio.h> // GPIO interface
#include "gpio.h"

/**
 * @brief Board-specific pin configuration.
 *
 * Replace/add more pin configurations here as appropriate. Each enumeration
 * value of `gpio_t` should get a configuration.
 *
 * @see `stm32f4xx_hal_gpio.h` for various initialization values.
 */
static const cmr_gpioPinConfig_t gpioPinConfigs[GPIO_LEN] = {
	[GPIO_LED] = {
		.port = GPIOB,
		.init = {
			.Pin = GPIO_PIN_4,
			.Mode = GPIO_MODE_OUTPUT_PP,
			.Pull = GPIO_NOPULL,
			.Speed = GPIO_SPEED_FREQ_LOW
		}
	},
	[GPIO_BMS_ERROR] = {
		.port = GPIOC,
		.init = {
			.Pin = GPIO_PIN_4,
			.Mode = GPIO_MODE_OUTPUT_PP,
			.Pull = GPIO_NOPULL,
			.Speed = GPIO_SPEED_FREQ_LOW
			}
    },
	[RX_TURNON] = {
		.port = GPIOA,
		.init = {
			.Pin = GPIO_PIN_9,
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
	cmr_gpioPinInit(gpioPinConfigs, GPIO_LEN);
}
