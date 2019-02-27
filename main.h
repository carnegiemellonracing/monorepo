/**
 * @file main.h
 * @brief DIM-specific definitions
 *
 * @author Carnegie Mellon Racing
 */

#include "can.h"    // Board-specific CAN interface

extern volatile cmr_canState_t VSM_state;
extern volatile cmr_canState_t DIM_requested_state;
