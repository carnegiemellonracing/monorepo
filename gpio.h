/**
 * @file gpio.h
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef GPIO_H
#define GPIO_H

/** @brief Represents a GPIO pin. */
typedef enum {
    GPIO_LED_STATUS = 0,    /**< @brief Status LED. */
    GPIO_LED_ERROR,         /**< @brief Error LED. */
    GPIO_LEN    /**< @brief Total GPIO pins. */
} gpio_t;

void gpioInit(void);

void gpioWrite(gpio_t pin, int value);
void gpioToggle(gpio_t pin);
int gpioRead(gpio_t pin);

#endif /* GPIO_H */

