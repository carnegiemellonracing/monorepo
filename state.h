/**
 * @file gpio.h
 * @brief DIM state request handling logic.interface
 *
 * @author Carnegie Mellon Racing
 */

#ifndef STATE_H
#define STATE_H

#include <stdbool.h>    // bool
#include "can.h"    // Interface to implement

bool is_valid_state_request(cmr_canState_t state);
void state_up_button(bool pressed);
void state_down_button(bool pressed);

#endif /* STATE_H */
