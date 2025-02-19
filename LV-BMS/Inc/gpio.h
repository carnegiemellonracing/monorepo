/**
 * @file gpio.h
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef GPIO_H_
#define GPIO_H_

#include <CMR/gpio.h> // GPIO interface

/**
 * @brief Represents a GPIO pin.
 */
typedef enum {
	GPIO_LED = 0,
	GPIO_VTHERM_SEL0,
	GPIO_VTHERM_SEL1,
	GPIO_VTHERM_SEL2,
	GPIO_VTHERM_IN,
	GPIO_POST_MS,
	GPIO_LEN
} gpio_t;

/**
 * @brief initializes gpio pins
 */
void gpio_init();

#endif /* GPIO_H_ */
