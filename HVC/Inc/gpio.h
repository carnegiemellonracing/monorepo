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
    GPIO_MCU_LED = 0,    /**< @brief Status LED. */
    GPIO_CLEAR_FAULT_L,
    GPIO_AIR_FAULT_L,
    GPIO_DISCHARGE_EN,
    GPIO_PRECHARGE_EN,
    GPIO_AIR_POSITIVE_EN,
    GPIO_AIR_NEGATIVE_EN,
    GPIO_TO_WATCHDOG,
	SAFETY_BINARY, 
	RX_TURNON,
    GPIO_IN_BSPD_ERR_N,         /**< @brief BSPD latch input signal. */
    GPIO_IN_IMD_ERR_N,          /**< @brief IMD latch input signal. */
    GPIO_IN_IMD_ERR_COND_N,     /**< @brief IMD un-latch input signal. */
    GPIO_OUT_AMS_ERR_N,         /**< @brief AMS error Driver. */
    GPIO_LEN,    /**< @brief Total GPIO pins. */
} gpio_t;

void gpioInit(void);
void RXTurnOnInit(void);

#endif /* GPIO_H */

