/**
 * @file newState.h
 * @brief DIM state interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_STATE_H
#define CMR_STATE_H

#include <stdbool.h>    // bool
#include "expanders.h"
#include "can.h"        // Board-specific CAN interface

//states
typedef enum {
    INIT = 0, 
    START = 1, 
    NORMAL = 2, 
    CONFIG = 3, 
    dimStateERROR = 4,
    RACING = 5
    //SAFETY
} cmr_state;

void reqVSM(void);

void reqGear(void);
int getRequestedGear(void);

int getMaxMotorTemp(void);
int getACTemp(void);
int getMCTemp(void);
bool getDoorsState(void);

// Declare the global variable
extern cmr_state currState;
extern cmr_state nextState;

void stateMachineInit(void);

#endif /* CMR_CAN_TYPES_H */