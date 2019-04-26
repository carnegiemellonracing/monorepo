/**
 * @file gpio.h
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef GPIO_H
#define GPIO_H

/**
 * @brief Represents a GPIO pin.
 *
 * @note All boards should at least have a status LED (`GPIO_LED_STATUS`).
 * @warning New pins MUST be added between `GPIO_LED_STATUS` and `GPIO_LEN`.
 */
typedef enum {
    GPIO_LED_STATUS = 0,    /**< @brief Status LED. */
    GPIO_BRAKELIGHT,        /**< @brief Brake light enable. */
    GPIO_FAN_ENABLE,        /**< @brief Fan drive. */
    GPIO_PUMP,              /**< @brief Pump drive. */
    GPIO_BRAKE_DISCON,      /**< @brief Brake solenoid */
    GPIO_LEN                /**< @brief Total GPIO pins. */
} gpio_t;

void gpioInit(void);

#endif /* GPIO_H */

