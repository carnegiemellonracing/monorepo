 /**
 * @file expanders.h
 * @brief GPIO expanders interface
 *
 * @author Carnegie Mellon Racing
 */

#ifndef EXPANDERS_H
#define EXPANDERS_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    EXP_DASH_BUTTON_0 = 0,
    EXP_DASH_BUTTON_1,
    EXP_DASH_BUTTON_2,
    EXP_DASH_BUTTON_3,
    EXP_WHEEL_BUTTON_0,
    EXP_WHEEL_BUTTON_1,
    EXP_WHEEL_BUTTON_2,
    EXP_WHEEL_BUTTON_3,
    EXP_BUTTON_LEN
} expanderButton_t;

typedef enum {
    EXP_ROTARY_0 = 0,
    EXP_ROTARY_1,
    EXP_ROTARY_2,
    EXP_ROTARY_LEN
} expanderRotary_t;

// typedef enum { // Don't think this is needed for 24e
//     ROTARY_POS_1 = 0,
//     ROTARY_POS_2,
//     ROTARY_POS_3,
//     ROTARY_POS_4,
//     ROTARY_POS_5,
//     ROTARY_POS_6,
//     ROTARY_POS_7,
//     ROTARY_POS_8,
//     ROTARY_POS_LEN,
//     ROTARY_POS_INVALID  // If none of the pins read high
// } expanderRotaryPosition_t;

typedef enum {
    EXP_CLUTCH_1 = 0,
    EXP_CLUTCH_2,
    EXP_CLUTCH_LEN
} expanderClutch_t;

// typedef enum {
//     EXP_LED_1 = 0,
//     EXP_LED_2,
//     EXP_LED_LEN
// } expanderLED_t;

void expandersInit(void);
bool expanderGetButtonPressed(expanderButton_t button);
// expanderRotaryPosition_t expanderGetRotary(expanderRotary_t rotary);
uint32_t expanderGetClutch(expanderClutch_t clutch);
// void expanderSetLED(expanderLED_t led, bool isOn);

#endif /* EXPANDERS_H */

