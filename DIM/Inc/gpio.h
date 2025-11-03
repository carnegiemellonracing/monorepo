/**
 * @file gpio.h
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef GPIO_H
#define GPIO_H

#include <CMR/gpio.h>  // GPIO interface
#include <FreeRTOS.h>  // FreeRTOS API
#include <stdbool.h>

#define NUM_BUTTONS 6

/**
 * @brief Represents a GPIO pin.
 */
typedef enum {
    GPIO_BUTTON_UP = 0, /**< @brief Dashboard Up Button */
    GPIO_BUTTON_DOWN, /**< @brief Dashboard Down Button */
    GPIO_BUTTON_LEFT, /**< @brief Dashboard Left Button */
    GPIO_BUTTON_RIGHT, /**< @brief Dashboard Right Button */
    GPIO_CTRL_SWITCH, /**< @brief Dashboard Switch */
    GPIO_BUTTON_SW_LEFT, /**< @brief Steering Wheel Left Button */
    GPIO_BUTTON_SW_RIGHT, /**< @brief Steering Wheel Right Button */
	GPIO_LED_AMS, /**< @brief AMS Error LED */
    GPIO_LED_IMD, /**< @brief IMD Error LED */
    GPIO_LED_BSPD, /**< @brief BSPD Error LED */
    GPIO_ASMS_ON, /**< @brief AS Master Switch On/Off */
	GPIO_LED_STATUS, /**< @brief Status LED */
    GPIO_PD_N,
    GPIO_LEN /**< @brief Total GPIO pins. */
} gpio_t;


//gpio button indices
typedef enum {
    UP = 0,
    DOWN,
    LEFT,
    RIGHT,
    SW_LEFT,
    SW_RIGHT
} cmr_gpio_button_index;

extern bool gpioButtonStates[NUM_BUTTONS];

extern bool gpioLRUDStates[4];

void canLRUDDetect(void);

/**
 * @brief initializes gpio pins
 */
void gpioInit(void);

bool getASMS(void);
bool getEAB(void);

#endif /* GPIO_H */
