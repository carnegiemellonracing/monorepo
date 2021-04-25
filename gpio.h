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
 *
 * @note All boards should at least have a status LED (`GPIO_LED_STATUS`).
 * @warning New pins MUST be added between `GPIO_LED_STATUS` and `GPIO_LEN`.
 */
typedef enum {
    GPIO_LED_STATUS = 0,    /**< @brief Status LED. */
    GPIO_LED_IMD,   /**< @brief IMD Error LED. */
    GPIO_LED_AMS,   /**< @brief AMD Error LED. */
    GPIO_LED_BSPD,  /**< @brief BSPD Error LED. */
    GPIO_BUTTON_0,  /**< @brief Action Button. */
    GPIO_BUTTON_1,  /**< @brief State Up Button. */
    GPIO_BUTTON_2,  /**< @brief State Down Button. */
    GPIO_BUTTON_3,  /**< @brief Gear Up Button. */
    GPIO_BUTTON_4,  /**< @brief Gear Down Button. */
    GPIO_PD_N,      /**< @brief Screen Power Down. */
    GPIO_LEN    /**< @brief Total GPIO pins. */
} gpio_t;

/** @brief Data required to processes a button press. */
typedef struct {
    gpio_t pin;     /**< @brief The triggering pin. */
    bool pressed;   /**< @brief `true` for pressed; `false` for released. */
} buttonEvent_t;

void gpioInit(void);

/** @brief Action button value */
bool actionButtonPressed;

#endif /* GPIO_H */
