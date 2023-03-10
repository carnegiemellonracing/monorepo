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
extern volatile uint8_t config_paddle_left_request;
extern volatile uint8_t config_paddle_right_request;
extern volatile int8_t config_move_request;
#define CONFIG_SCREEN_NUM_COLS 4
#define MIN_PADDLE_VAL 50
#define MAX_PADDLE_VAL 255

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

int32_t getAverageWheelRPM(void);
bool stateVSMReqIsValid(cmr_canState_t vsm, cmr_canState_t vsmReq);

uint8_t getLeftPaddleState(void);
uint8_t getRightPaddleState(void);

void updateReq(void);

bool inConfigScreen(void);

float getSpeedKmh(void);

float getOdometer(void);

void odometerInit(void);

#endif /* STATE_H */

