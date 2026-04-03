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
	GPIO_LED_STATUS = 0,
	GPIO_BMS_ERROR, 
	RX_TURNON,
	GPIO_LEN
} gpio_t;

/**
 * @brief initializes gpio pins
 */
void gpioInit();

#endif /* GPIO_H_ */
