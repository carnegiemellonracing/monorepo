/**
 * @file gpio.h
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef GPIO_H
#define GPIO_H

#include <CMR/gpio.h>   // GPIO interface
#include "expanders.h"   // GPIO expanders interface

/** @brief Macros for regen */
#define REGEN_MAX 50
#define REGEN_MIN 0
#define REGEN_STEP_NUM 10
#define REGEN_STEP ((REGEN_MAX - REGEN_MIN) / REGEN_STEP_NUM)



/**
 * @brief Represents a GPIO pin.
 */
typedef enum {
    GPIO_LED_0 = 0,
	GPIO_LED_1,
	GPIO_LED_2,
	GPIO_LED_AMS,
	GPIO_LED_IMD,
	GPIO_LED_BSPD,
	GPIO_PTT_N,
    GPIO_PD_N,
    GPIO_LEN    /**< @brief Total GPIO pins. */
} gpio_t;

void gpioInit(void);

typedef void (*action_f) (bool);

typedef struct {
    bool buttonState;
    action_f setAction;
    TickType_t lastPressed;
    TickType_t debounce;
} expanderButtonEvent_t;

typedef void (*rotaryAction_f) (expanderRotaryPosition_t);
typedef struct {
    expanderRotaryPosition_t position;
    rotaryAction_f setAction;
} expanderRotaryEvent_t;

/** @brief Current regen step */
extern unsigned int regenStep;

#endif /* GPIO_H */
