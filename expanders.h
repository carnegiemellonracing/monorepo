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
    ROTARY_POS_0 = 0b111,
    ROTARY_POS_1 = 0b011,
    ROTARY_POS_2 = 0b101,
    ROTARY_POS_3 = 0b001,
    ROTARY_POS_4 = 0b110,
    ROTARY_POS_5 = 0b010,
    ROTARY_POS_6 = 0b100,
    ROTARY_POS_7 = 0b000,
    ROTARY_POS_LEN = 8,
    ROTARY_POS_INVALID = 9  // If none of the pins read high
} expanderRotaryPosition_t;

typedef enum {
    EXP_CLUTCH_1 = 0,
    EXP_CLUTCH_2,
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
