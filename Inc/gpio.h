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

#include "expanders.h"  // GPIO expanders interface

#define NUM_BUTTONS 7 //number of buttons

/**
 * @brief Represents a GPIO pin.
 */
typedef enum {
    GPIO_LED_AMS,
    GPIO_LED_IMD,
    GPIO_LED_BSPD,
    //new pins
    GPIO_BUTTON_L = 0, 
    GPIO_BUTTON_R = 1, 
    GPIO_BUTTON_U = 2, 
    GPIO_BUTTON_D = 3, 
    GPIO_BUTTON_SW1 = 4, 
    GPIO_BUTTON_SW2 = 5, 
    GPIO_BUTTON_PUSH = 6,
    GPIO_LEN /**< @brief Total GPIO pins. */
} gpio_t;

//gpio button indices
typedef enum {
    L = 0, 
    R = 1, 
    U = 2, 
    D = 3, 
    SW1 = 4, 
    SW2 = 5, 
    PUSH = 6
} cmr_gpio_button_index;

extern bool gpioButtonStates[NUM_BUTTONS];

//does it matter which .h file this global variable is in??
//can button indices
typedef enum {
    L = 0, 
    R = 1, 
    U = 2, 
    D = 3, 
    SW1 = 4, 
    SW2 = 5, 
    PUSH = 6
} cmr_can_button_index;

extern bool canButtonStates[NUM_BUTTONS];


/**
 * @brief initializes gpio pins
 */
void gpioInit(void);

#endif /* GPIO_H */
