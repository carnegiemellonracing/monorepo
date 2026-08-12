/**
 * @file gpio.h
 * @brief Board-specific GPIO interface.
 *
 */

#ifndef GPIO_H
#define GPIO_H

#include <CMR/gpio.h>   // GPIO interface
#include <stdbool.h>

/**
 * @brief Represents a GPIO pin.
 *
 * @note All boards should at least have a status LED (`GPIO_LED_STATUS`) and a push button (`GPIO_PUSH_BUTTON`).
 */
typedef enum {
    GPIO_LED_STATUS = 0,    /**< @brief Status LED. */
    GPIO_LEN  				/**< @brief Total GPIO pins. */
} gpio_t;

void gpioInit(void);
void gpioDeinit(void);
void timedLedToggle(void);

#define LED_TOGGLE_TIME_MS 1000


#endif /* GPIO_H */

