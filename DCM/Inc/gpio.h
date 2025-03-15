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
    GPIO_FRAM_WP,           /**< @brief FRAM Write Protect Pin */
	GPIO_BRKLT_ENABLE,      /**< @brief Brakelight Enable. */
	GPIO_FAN_ON,            /**< @brief Fan On LED. */
	GPIO_PUMP_ON,           /**< @brief Pump On LED. */
    GPIO_AUXILIARY_ENABLE,  /**< @brief Auxiliary Enable. */
	GPIO_MTR_CTRL_ENABLE,   /**< @brief Motor Controller Power Enable */
	GPIO_MC_EFUSE_AUTO,     /**< @brief Motor Controller eFuse Auto Reset
	                                             (high=latching, low=autoreset*/
    GPIO_LEN  				/**< @brief Total GPIO pins. */
} gpio_t;

void gpioInit(void);

#endif /* GPIO_H */

