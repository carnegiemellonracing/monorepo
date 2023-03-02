/**
 * @file state.h
 * @brief DIM state interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef STATE_H
#define STATE_H

#include <stdbool.h>    // bool

#include "can.h"        // Board-specific CAN interface

cmr_canState_t stateGetVSM(void);
cmr_canState_t stateGetVSMReq(void);

cmr_canGear_t stateGetGear(void);
cmr_canGear_t stateGetGearReq(void);

void actionOneButton(bool pressed);
void actionTwoButton(bool pressed);

void regenDownButton(bool pressed);
void regenUpButton(bool pressed);

void exitConfigScreen();

void stateVSMUpButton(bool pressed);
void stateVSMDownButton(bool pressed);

void stateGearUpButton(bool pressed);
void stateGearDownButton(bool pressed);
void stateGearUpdate(void);

int32_t getAverageWheelRPM(void);
bool stateVSMReqIsValid(cmr_canState_t vsm, cmr_canState_t vsmReq);

void updateReq(void);

bool inConfigScreen();

#endif /* STATE_H */

