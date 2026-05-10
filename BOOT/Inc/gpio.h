/**
 * @file gpio.h
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
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
    GPIO_PUSH_BUTTON,         /**< @brief Push button. */
    GPIO_LEN  				/**< @brief Total GPIO pins. */
} gpio_t;

void gpioInit(void);
void gpioDeinit(void);
void timedLedToggle(void);
bool getPushButton(void);

#define LED_TOGGLE_TIME_MS 500


#endif /* GPIO_H */

