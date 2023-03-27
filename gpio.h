/**
 * @file gpio.h
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef GPIO_H
#define GPIO_H

#include <stdbool.h>    // bool
#include <CMR/gpio.h>   // GPIO interface
#include "expanders.h"   // GPIO expanders interface

/** @brief Macros for regen */
#define REGEN_MAX 50
#define REGEN_MIN 0
#define REGEN_STEP_NUM 10
#define REGEN_STEP ((REGEN_MAX - REGEN_MIN) / REGEN_STEP_NUM)

/**
 * @brief Represents a GPIO pin.
 *
 * @note All boards should at least have a status LED (`GPIO_LED_STATUS`).
 * @warning New pins MUST be added between `GPIO_LED_STATUS` and `GPIO_LEN`.
 */
typedef enum {
    GPIO_LED_STATUS = 0,    /**< @brief Status LED. */
    GPIO_LED_IMD,   /**< @brief IMD Error LED. */
    GPIO_LED_AMS,   /**< @brief AMD Error LED. */
    GPIO_LED_BSPD,  /**< @brief BSPD Error LED. */
    GPIO_SS_MODULE,
    GPIO_SS_COCKPIT,
    GPIO_SS_FRHUB,
    GPIO_SS_INERTIA,
    GPIO_SS_FLHUB,
    GPIO_SS_BOTS,
    GPIO_PD_N,      /**< @brief Screen Power Down. */
    GPIO_LEN    /**< @brief Total GPIO pins. */
} gpio_t;

void gpioInit(void);

/** Function pointer type for button actions, input is whether the button is pressed or not*/
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
