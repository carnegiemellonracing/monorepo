/**
 * @file gpio.h
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef GPIO_H
#define GPIO_H

#include <CMR/pwm.h>

/**
 * @brief Represents a GPIO pin.
 *
 * @note All boards should at least have a status LED (`GPIO_LED_STATUS`).
 * @warning New pins MUST be added between `GPIO_LED_STATUS` and `GPIO_LEN`.
 */
typedef enum {
    GPIO_LED_STATUS = 0,    /**< @brief Status LED. */
    // GPIO_OUT_LED_GREEN,
    // GPIO_OUT_LED_FLASH_RED,
    GPIO_OUT_DCDC_EN,           /**< @brief DCDC converter enable signal. */
    GPIO_OUT_SOFTWARE_ERR,      /**< @brief Software error indicator. */
    GPIO_OUT_RTD_SIGNAL,        /**< @brief Ready-to-drive signal. */
    GPIO_IN_SOFTWARE_ERR,       /**< @brief Software error latch input signal. */
    GPIO_IN_BSPD_ERR,           /**< @brief BSPD latch input signal. */
    GPIO_IN_IMD_ERR,            /**< @brief IMD latch input signal. */
    GPIO_LEN                    /**< @brief Total GPIO pins. */
} gpio_t;

static const cmr_gpioPinConfig_t gpioPinConfigs[GPIO_LEN];

static cmr_pwm_t LED_Red;   /**< @brief Status LED when BMS or IMD Error. */
static cmr_pwm_t LED_Green; /**< @brief Status LED when everything works. */

extern cmr_pwmPinConfig_t pwmPinConfig1;
extern cmr_pwmPinConfig_t pwmPinConfig2;
void gpioInit(void);

#endif /* GPIO_H */

