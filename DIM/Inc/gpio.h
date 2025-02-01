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

#define NUM_BUTTONS 8 //number of buttons
#define NUM_ROTARY_POSITION 14 // number of rotary positions
#define PUSH_BUTTONS 4

/**
 * @brief Represents a GPIO pin.
 */

//Testing
extern float sensorX;
extern float sensorY;

//TODO Fix gpio
typedef enum {
    //new pins
    GPIO_BUTTON_A = 0,
    GPIO_BUTTON_B,
    GPIO_BUTTON_SW1,
    GPIO_BUTTON_SW2,
    GPIO_BUTTON_PUSH,
	GPIO_LED_AMS,
    GPIO_LED_IMD,
    GPIO_LED_BSPD,
	GPIO_LED_STATUS,
    GPIO_LEN /**< @brief Total GPIO pins. */
} gpio_t;


//gpio button indices
typedef enum {
    A = 0,
    B,
    SW1,
    SW2,
    PUSH,
} cmr_gpio_button_index;

extern bool gpioButtonStates[NUM_BUTTONS];

//does it matter which .h file this global variable is in??
//can button indices

extern bool canButtonStates[NUM_BUTTONS];

//Left right up down indexes
typedef enum {
	LEFT = 0,
	RIGHT,
	UP,
	DOWN,
	LRUDLen,
} cmr_LRUD_index;

extern bool gpioLRUDStates[4];
extern bool canLRUDStates[4];

void canLRUDDetect(void);

/**
 * @brief initializes gpio pins
 */
void gpioInit(void);


/**
* @brief gets rotary position
*/
int getRotaryPosition(void);

/**
* @brief gets past rotation position
*/
int getPastRotaryPosition(void);

#endif /* GPIO_H */
