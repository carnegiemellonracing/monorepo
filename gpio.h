/**
 * @file gpio.h
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef GPIO_H
#define GPIO_H

#include <CMR/gpio.h>   // GPIO interface

/**
 * @brief Represents a GPIO pin.
 *
 * @note All boards should at least have a status LED (`GPIO_LED_STATUS`).
 * @warning New pins MUST be added between `GPIO_LED_STATUS` and `GPIO_LEN`.
 */
typedef enum {
    GPIO_LED_STATUS = 0,    /**< @brief Status LED. */
    GPIO_BMB_WAKE_PIN,
    GPIO_BMB_POWER_PIN,
    GPIO_RELAY_PIN_AIR_POS,
    GPIO_RELAY_PIN_AIR_NEG,
    GPIO_RELAY_PIN_PRECHARGE,
    GPIO_RELAY_PIN_DISCHARGE,
    GPIO_RELAY_PIN_POWER_FAULT_L,
    GPIO_DCDC_ENABLE_PIN,
    GPIO_CLEAR_ERROR_PIN,
    //GPIO_OVERCURRENT_FAULT_PIN,
    GPIO_BMB_FAULT_L_PIN,
    GPIO_LEN    /**< @brief Total GPIO pins. */
} gpio_t;

void gpioInit(void);

#endif /* GPIO_H */

