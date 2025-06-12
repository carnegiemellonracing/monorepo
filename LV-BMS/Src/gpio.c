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
		.port = GPIOC,
		.init = {
			.Pin = GPIO_PIN_12,
			.Mode = GPIO_MODE_OUTPUT_PP,
			.Pull = GPIO_NOPULL,
			.Speed = GPIO_SPEED_FREQ_LOW
		}
	},
	[GPIO_VTHERM_SEL0] = {
		.port = GPIOB,
		.init = {
			.Pin = GPIO_PIN_15,
			.Mode = GPIO_MODE_OUTPUT_PP,
			.Pull = GPIO_NOPULL,
			.Speed = GPIO_SPEED_FREQ_LOW
		}
	},
	[GPIO_VTHERM_SEL1] = {
		.port = GPIOB,
		.init = {
			.Pin = GPIO_PIN_14,
			.Mode = GPIO_MODE_OUTPUT_PP,
			.Pull = GPIO_NOPULL,
			.Speed = GPIO_SPEED_FREQ_LOW
		}
	},
	[GPIO_VTHERM_SEL2] = {
		.port = GPIOB,
		.init = {
			.Pin = GPIO_PIN_13,
			.Mode = GPIO_MODE_OUTPUT_PP,
			.Pull = GPIO_NOPULL,
			.Speed = GPIO_SPEED_FREQ_LOW
		}
	},
	[GPIO_VTHERM_IN] = {
		.port = GPIOB,
		.init = {
			.Pin = GPIO_PIN_11,
			.Mode = GPIO_MODE_OUTPUT_PP,
			.Pull = GPIO_NOPULL,
			.Speed = GPIO_SPEED_FREQ_LOW
		}
	},
	[GPIO_POST_MS] = {
		.port = GPIOC,
		.init = {
			.Pin = GPIO_PIN_13,
			.Mode = GPIO_MODE_INPUT,
			.Pull = GPIO_NOPULL,
			.Speed = GPIO_SPEED_FREQ_LOW
		}
	}
};

/**
 * @brief Initializes the GPIO interface.
 */
void gpio_init() {
	cmr_gpioPinInit(gpioPinConfigs, GPIO_LEN);
	cmr_gpioWrite(GPIO_LED, 0);
	cmr_gpioWrite(GPIO_VTHERM_SEL0, 0);
	cmr_gpioWrite(GPIO_VTHERM_SEL1, 0);
	cmr_gpioWrite(GPIO_VTHERM_SEL2, 0);
}
