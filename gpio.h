/**
 * @file gpio.h
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef GPIO_H
#define GPIO_H

#include <stdbool.h>    // bool
#include "main.h"   // DIM-specific definitions

/**
 * @brief Represents a GPIO pin.
 *
 * @note All boards should at least have a status LED (`GPIO_LED_STATUS`).
 * @warning New pins MUST be added between `GPIO_LED_STATUS` and `GPIO_LEN`.
 */
typedef enum {
    GPIO_LED_STATUS = 0,    /**< @brief Status LED. */
    GPIO_BUTTON_0,  /**< @brief Button 0. */
    GPIO_BUTTON_1,  /**< @brief Button 1. */
    GPIO_BUTTON_2,  /**< @brief Button 2. */
    GPIO_BUTTON_3,  /**< @brief Button 3. */
    GPIO_BUTTON_4,  /**< @brief Button 4. */
    GPIO_BEEPER,    /**< @brief Beeper. */
    GPIO_LEN    /**< @brief Total GPIO pins. */
} gpio_t;

typedef struct {
    gpio_t pin;     /**< @brief The triggering pin. */
    bool pressed;   /**< @brief `true` for pressed; `false` for released. */
} buttonEvent_t;

void gpioInit(void);

#endif /* GPIO_H */
