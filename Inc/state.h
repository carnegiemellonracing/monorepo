/**
 * @file state.h
 * @brief DIM state interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef STATE_H
#define STATE_H

#include <stdbool.h>  // bool

#include "can.h"  // Board-specific CAN interface
#include "expanders.h"

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

cmr_canDrsMode_t stateGetDrs(void);
cmr_canDrsMode_t stateGetDrsReq(void);

/** @brief AE/DRS button value */
extern bool pttButtonPressed;
/** @brief Action 0 button value */
extern bool actionButtonPressed;
/** @brief Action 1 button value */
extern bool ackButtonPressed;
/** @brief Action 2 button value */
extern bool screenButtonPressed;
/** @brief Up button value */
extern bool actionUpButtonPressed;
/** @brief Down button value*/
extern bool actionDownButtonPressed;
/** @brief Left button value */
extern bool actionLeftButtonPressed;
/** @brief Right button value */
extern bool actionRightButtonPressed;
/** @brief rotatory values {ROT_SEL,ROT_2,ROT_1,ROT_0} */
extern uint8_t rotaryPos;
extern uint8_t switchValues;
void actionButton(bool pressed);
void ackButton(bool pressed);
void screenButton(bool pressed);
void pttButton(bool pressed);

uint32_t get_test_message_id();
void new_test_message_id();

void upButton(bool pressed);
void downButton(bool pressed);
void leftButton(bool pressed);
void rightButton(bool pressed);

void rotaries(bool select, uint8_t pos);

void exitConfigScreen();

void stateGearSwitch(expanderRotaryPosition_t pos);
void stateGearUpdate(void);

void stateDrsModeSwitch(expanderRotaryPosition_t pos);
void stateDrsUpdate(void);

bool getAcknowledgeButton(void);

int32_t getAverageWheelRPM(void);
bool stateVSMReqIsValid(cmr_canState_t vsm, cmr_canState_t vsmReq);

void StateVSMUp();
void StateVSMDown();

uint8_t getPaddleState(expanderClutch_t clutch);
uint8_t getPos(expanderClutch_t clutch);

void updateReq(void);

bool inConfigScreen(void);

bool inRacingScreen(void);

float getSpeedKmh(void);

float getOdometer(void);

void odometerInit(void);

typedef enum {
    LV_LIFEPO = 0,
    LV_LIPO,
} lv_battery_type_t;

uint8_t getLVSoC(float voltage, lv_battery_type_t battery_type);

//enum Gear(0="UNKNOWN", 1="SLOW", 2="FAST", 3="ENDURANCE", 4="AUTO-X",
//  5="SKIDPAD", 6="ACCEL", 7="TEST", 8="REVERSE")


#endif /* STATE_H */
