/**
 * @file state.h
 * @brief DIM state interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef STATE_H
#define STATE_H

#include <stdbool.h>    // bool
#include "expanders.h"

#include "can.h"        // Board-specific CAN interface

// TODO: Add documentation
extern volatile bool config_increment_up_requested;
extern volatile bool config_increment_down_requested;
extern volatile int8_t config_move_request;
#define CONFIG_SCREEN_NUM_COLS 4

cmr_canState_t stateGetVSM(void);
cmr_canState_t stateGetVSMReq(void);

cmr_canGear_t stateGetGear(void);
cmr_canGear_t stateGetGearReq(void);

cmr_canDRSMode_t stateGetDrs(void);
cmr_canDRSMode_t stateGetDrsReq(void);

/** @brief AE/DRS button value */
extern bool drsButtonPressed;
/** @brief Action 1 button value */
extern bool action1ButtonPressed;
/** @brief Action 2 button value */
extern bool action2ButtonPressed;

void actionOneButton(bool pressed);
void actionTwoButton(bool pressed);
void drsButton(bool pressed);

void upButton(bool pressed);
void downButton(bool pressed);
void leftButton(bool pressed);
void rightButton(bool pressed);

void exitConfigScreen();

void stateGearSwitch(expanderRotaryPosition_t pos);
void stateGearUpdate(void);

void stateDrsModeSwitch(expanderRotaryPosition_t pos);
void stateDrsUpdate(void);

void stateRotary2Switch(expanderRotaryPosition_t pos);

int32_t getAverageWheelRPM(void);
bool stateVSMReqIsValid(cmr_canState_t vsm, cmr_canState_t vsmReq);

void updateReq(void);

bool inConfigScreen();

#endif /* STATE_H */

