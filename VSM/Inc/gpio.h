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

/**
 * @brief Represents a GPIO pin.
 *
 * @note All boards should at least have a status LED (`GPIO_LED_STATUS`).
 * @warning New pins MUST be added between `GPIO_LED_STATUS` and `GPIO_LEN`.
 */
typedef enum {
    GPIO_OUT_LED_STATUS = 0,    /**< @brief Status LED. */
    GPIO_OUT_SOFTWARE_ERR_N,    /**< @brief Software error Driver. */
    GPIO_OUT_AMS_ERR_N,         /**< @brief AMS error Driver. */
    GPIO_OUT_RTD_SIGNAL,        /**< @brief Ready-to-drive signal. */
    GPIO_IN_SOFTWARE_ERR_N,     /**< @brief Software error latch input signal. */
    GPIO_IN_BSPD_ERR_N,         /**< @brief BSPD latch input signal. */
    GPIO_IN_IMD_ERR_N,          /**< @brief IMD latch input signal. */
    GPIO_IN_IMD_ERR_COND_N,     /**< @brief IMD un-latch input signal. */
    GPIO_IN_BSPD_ERR_UNLATCH,   /**< @brief BSPD un-latch input signal. */
    GPIO_IN_EAB,                /**< @brief EAB Signal Input */
    GPIO_LEN                    /**< @brief Total GPIO pins. */
} gpio_t;

static const cmr_gpioPinConfig_t gpioPinConfigs[GPIO_LEN];
void gpioInit(void);

#endif /* GPIO_H */

