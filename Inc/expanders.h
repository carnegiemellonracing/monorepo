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
    ROTARY_POS_0 = 0,
    ROTARY_POS_1,
    ROTARY_POS_2,
    ROTARY_POS_3,
    ROTARY_POS_4,
    ROTARY_POS_5,
    ROTARY_POS_6,
    ROTARY_POS_7,
    ROTARY_POS_LEN,
    ROTARY_POS_INVALID  // If none of the pins read high
} expanderRotaryPosition_t;

typedef struct {
    uint8_t pin;
} rotaryPinConfig_t;

typedef enum {
    EXP_CLUTCH_LEFT = 0,
    EXP_CLUTCH_RIGHT,
    EXP_CLUTCH_LEN
} expanderClutch_t;

typedef struct {
    uint16_t expanderAddress;
    uint8_t port;
    uint8_t pin;
} expanderPinConfig_t;

void expandersInit(void);
bool expanderGetButtonPressed(expanderButton_t button);
expanderRotaryPosition_t expanderGetRotary(bool select);
uint32_t expanderGetClutch(expanderClutch_t clutch);

#endif /* EXPANDERS_H */
