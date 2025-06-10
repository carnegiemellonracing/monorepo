/**
  * @file gpio.h
  * @brief Header for HITL's GPIO interface
  * @author Carnegie Mellon Racing
**/

#ifndef GPIO_H
#define GPIO_H

#include <CMR/gpio.h>

/**
 * @brief Represents a GPIO pin.
 * @note All boards should at least have a status LED (`GPIO_LED_STATUS`).
 * @warning New pins MUST be added between `GPIO_LED_STATUS` and `GPIO_LEN`.
 */

typedef enum { // to be changed per what the pins are
	DIGITAL_IN_24V_MCU1_MC,
	DIGITAL_IN_24V_MCU2_BRAKELIGHT,
    GPIO_LEN    /**< @brief Total GPIO pins. */
} gpio_t;

void gpioInit(void);

#endif /* GPIO_H */
