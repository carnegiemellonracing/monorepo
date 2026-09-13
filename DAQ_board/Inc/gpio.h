/**
 * @file gpio.h
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef GPIO_H
#define GPIO_H

#include <CMR/gpio.h>

/**
 * @brief Represents a GPIO pin.
 *
 * @warning New pins MUST be added before `GPIO_LEN`.
 */
typedef enum {
    GPIO_OUT_MCU_STATUS = 0,    /**< @brief Status LED. */
    GPIO_OUT_MCU_ERR,           /**< @brief Error output driver. */
    GPIO_LEN                    /**< @brief Total GPIO pins. */
} gpio_t;

void gpioInit(void);

#endif /* GPIO_H */
