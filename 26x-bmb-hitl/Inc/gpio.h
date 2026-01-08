/**
 * @file gpio.h
 * @brief Board-specific GPIO interface.
 *
 * @author Ayush Garg
 */

#ifndef GPIO_H
#define GPIO_H

#include <stdbool.h>
#include <CMR/gpio.h>


/**
 * @brief Represents a GPIO pin.
 *
 * @note All boards should at least have a status LED (`GPIO_LED_STATUS`).
 * @warning New pins MUST be added between `GPIO_LED_STATUS` and `GPIO_LEN`.
 */
typedef enum {
	GPIO_MCU_STATUS,
	GPIO_LEN                /**< @brief Total GPIO pins. */
} gpio_t;

void gpioInit(void);
void openSafetyCircut(void);
void closeSafetyCircut(void);

bool getASMS(void);
bool getEAB(void);

#endif /* GPIO_H */