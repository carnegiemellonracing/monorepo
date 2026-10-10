/**
 * @file gpio.h
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef GPIO_H
#define GPIO_H

#include <CMR/pwm.h>
#include <CMR/gpio.h>

#pragma once

/**
 * @brief Represents a GPIO pin.
 *
 * @note All boards should at least have a status LED (`GPIO_LED_STATUS`).
 * @warning New pins MUST be added between `GPIO_FRAM_WP` and `GPIO_LEN`.
 */
typedef enum {
    GPIO_FRAM_WP,           /**< @brief FRAM Write Protect Pin */
	GPIO_BRKLT_ENABLE,      /**< @brief Brakelight Enable. */
	GPIO_FAN_ON,            /**< @brief Fan On LED. */
	GPIO_FAN_1,
	GPIO_FAN_2,
	GPIO_PUMP_LEFT,
	GPIO_PUMP_RIGHT,
	GPIO_MTR_CTRL_ENABLE,   /**< @brief Motor Controller Power Enable */

	GPIO_MCU_STATUS,    /**< @brief Status LED. */
    GPIO_OUT_SOFTWARE_ERR_N,    /**< @brief Software error Driver. */
    GPIO_OUT_RTD_SIGNAL,        /**< @brief Ready-to-drive signal. */
    GPIO_IN_SOFTWARE_ERR_N,     /**< @brief Software error latch input signal. */
    GPIO_IN_EAB,                /**< @brief EAB Signal Input */
    GPIO_LEN                    /**< @brief Total GPIO pins. */
} gpio_t;

void mcCtrlOff();
void mcCtrlOn();

void gpioInit(void);

#endif /* GPIO_H */

