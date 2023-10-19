/**
 * @file gpio.h
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef GPIO_H
#define GPIO_H

#include <stdbool.h>    // bool
#include <CMR/gpio.h>   // GPIO interface

/**
 * @brief Represents a GPIO pin.
 */
typedef enum {
    GPIO_LED_0 = 0,
	GPIO_LED_1,
	GPIO_LED_2,
	GPIO_LED_AMS,
	GPIO_LED_IMD,
	GPIO_LED_BSPD,
	GPIO_PTT_N,
    GPIO_LEN    /**< @brief Total GPIO pins. */
} gpio_t;

void gpioInit(void);
