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
    DIGITAL_OUT_5V_MCU = 0,    /**< @brief Status LED. */
    DIGITAL_OUT_24V_MCU,
    GPIO_MCU1,
    GPIO_MCU2,
    GPIO_MCU3,
    GPIO_MCU4,
    GPIO_MCU5,
    GPIO_MCU6,
    GPIO_MCU7,
    GPIO_MCU8,
    DIGITAL_IN_3V3_MCU1,
    DIGITAL_IN_3V3_MCU2,
    DIGITAL_IN_3V3_MCU3,
    DIGITAL_IN_3V3_MCU4,
    DIGITAL_IN_3V3_MCU5,
    DIGITAL_IN_24V_MCU1,
    DIGITAL_IN_24V_MCU2,
    DIGITAL_IN_24V_MCU3,
    DIGITAL_IN_24V_MCU4,
    DIGITAL_IN_24V_MCU5,
    GPIO_LEN    /**< @brief Total GPIO pins. */
} gpio_t;

void gpioInit(void);

#endif /* GPIO_H */