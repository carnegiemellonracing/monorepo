/**
 * @file state.h
 * @brief DIM state interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_STATE_H
#define CMR_STATE_H

#include <stdbool.h>    // bool
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

/** @brief declaration of config screen variables */
extern volatile int8_t config_move_request;
extern volatile bool config_increment_up_requested;
extern volatile bool config_increment_down_requested;
extern volatile uint8_t config_paddle_left_request;
extern volatile uint8_t config_paddle_right_request;
#define CONFIG_SCREEN_NUM_COLS 4
#define MIN_PADDLE_VAL 50
#define MAX_PADDLE_VAL 255

#define TFT_STARTUP_MS 3000

void reqVSM(void);

void reqGear(void);
int getRequestedGear(void);

int getMaxMotorTemp(void);
int getACTemp(void);
int getMCTemp(void);
bool DRSOpen(void);
void exitConfigScreen();
void stateGearUpdate(void);
void stateDrsUpdate(void);
cmr_canState_t stateGetVSM(void);
cmr_canState_t stateGetVSMReq(void);

cmr_canGear_t stateGetGear(void);
cmr_canGear_t stateGetGearReq(void);

cmr_canDrsMode_t stateGetDrs(void);
cmr_canDrsMode_t stateGetDrsReq(void);

uint32_t get_test_message_id();

float getSpeedKmh(void);

float getOdometer(void);

typedef enum {
	LV_LIFEPO = 0,
	LV_LIPO,
} lv_battery_type_t;

uint8_t getLVSoC(float voltage, lv_battery_type_t battery_type);

bool getAcknowledgeButton(void);

int32_t getAverageWheelRPM(void);

void stateVSMUp(void);
void stateVSMDown(void);
bool stateVSMReqIsValid(cmr_canState_t vsm, cmr_canState_t vsmReq);


// Declare the global variable
extern cmr_state currState;
extern cmr_state nextState;
//testing
// extern cmr_canGear_t currGear;
// extern cmr_canGear_t reqGear;

void stateMachineInit(void);

#endif // CMR_STATE_H