/**
 * @file statusLED.h
 * @brief statusLED interface
 * @author Ayush Garg
 */

#pragma once

#include <CMR/gpio.h>   // GPIO interface

/**
 * @brief Represents a GPIO pin.
 *
 * @note All boards should at least have a status LED (`GPIO_LED_STATUS`).
 * @warning New pins MUST be added between `GPIO_LED_STATUS` and `GPIO_LEN`.
 */
typedef enum {
    GPIO_LED_STATUS = 0,    /**< @brief Status LED. */
    GPIO_LEN    /**< @brief Total GPIO pins. */
} gpio_t;

void statusLEDInit(void);

static const cmr_gpioPinConfig_t gpioPinConfigs[GPIO_LEN];