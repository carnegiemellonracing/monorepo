/**
 * @file main.h
 * @brief DIM-specific definitions
 *
 * @author Carnegie Mellon Racing
 */

#ifndef MAIN_H
#define MAIN_H

#include "can.h"    // Board-specific CAN interface

extern volatile cmr_canState_t VSM_state;
extern volatile cmr_canState_t DIM_requested_state;
extern volatile int32_t HVC_pack_voltage;
extern volatile cmr_canGear_t DIM_gear;
extern volatile cmr_canGear_t DIM_newGear;
extern volatile cmr_canGear_t DIM_oldGear;

#endif /* MAIN_H */
