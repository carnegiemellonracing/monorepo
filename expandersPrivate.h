/**
 * @file expandersPrivate.h
 * @brief Private GPIO/ADC expanders interface
 *
 * @author Carnegie Mellon Racing
 */

#ifndef EXPANDERS_PRIVATE_H
#define EXPANDERS_PRIVATE_H

#include "expanders.h"


typedef struct {
    uint16_t expanderAddress;
    uint8_t port;
    uint8_t pin;
} expanderPinConfig_t;

typedef struct {
    expanderPinConfig_t pins[ROTARY_POS_LEN];
} expanderRotaryConfig_t;

#endif /* EXPANDERS_H */

