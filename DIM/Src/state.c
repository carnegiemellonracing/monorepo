/**
 * @file newState.c
 * @brief Firmware entry point.
 *
 * @author Carnegie Mellon Racing
 */

#include "state.h"

#include <CMR/adc.h>  // ADC interface
#include <CMR/can.h>  // CAN interface
#include <CMR/can_types.h>
#include <CMR/gpio.h>   // GPIO interface
#include <CMR/panic.h>  // cmr_panic()
#include <stdlib.h>

#include "can.h"   // Board-specific CAN interface
#include "gpio.h"  // Board-specific GPIO interface
#include "tft.h"   // TFT display interface.
#include "tftDL.h"
#include "adc.h"

static const uint32_t stateMachine_priority = 4;

/** @brief Button input task period (milliseconds). */
static const TickType_t stateMachine_period = 1000;

/** @brief Button input task task. */
static cmr_task_t stateMachine_task;

cmr_state nextState;

cmr_state currState;

volatile int8_t config_move_request;


#define max(a,b) ((a) > (b) ? (a) : (b))

#define min(a, b) __extension__\
({ __typeof__ (a) _a = (a); \
__typeof__ (b) _b = (b); \
_a < _b ? _a : _b; })

#define NUM_DV_MODES 3
#define POLE_PAIRS 4
#define MOTOR_LEN 4

/** @brief declaration of config screen variables */
extern volatile bool flush_config_screen_to_cdc;
/** @brief declaration of config screen variables */
volatile bool config_increment_up_requested = false;
/** @brief declaration of config screen variables */
volatile bool config_increment_down_requested = false;
/** @brief declaration of config screen variables */
volatile uint8_t config_paddle_left_request = 0;
/** @brief declaration of config screen variables */
volatile uint8_t config_paddle_right_request = 0;
/** @brief declaration of what screen mode one is in */
volatile bool in_config_screen = false;
/** @brief declaration of what screen mode one is in */
volatile bool in_racing_screen = false;
/** @brief declaration of if the DIM is waiting for a new driver config */
volatile bool waiting_for_cdc_new_driver_config;
/** @brief declaration of if the DIM is waiting for a new driver config */
volatile bool exit_config_request = false;
/** @brief Checks to see if the screen has been setup before and if not will appropriately draw it */
volatile bool dim_first_time_config_screen;
/** @brief Checks to see if the screen needs to be redrawn after getting new driver profiles */
volatile bool redraw_new_driver_profiles;

/** @brief Acknowledge button value */
bool ackButtonPressed;

uint8_t switchValues;


/** @brief DIM state. */
static volatile struct {
	cmr_canState_t vsmReq;    /**< @brief Requested VSM state. */
	cmr_canGear_t gear;       /**< @brief Current gear. */
	cmr_canGear_t gearReq;    /**< @brief Requested gear. */
	cmr_canDrsMode_t drsMode; /**< @brief Current DRS Mode. */
	cmr_canDrsMode_t drsReq;  /**< @brief Requested DRS Mode. */
    cmr_canDVMode_t dvCtrlMode;
    cmr_canDVMode_t dvCtrlReq;
} state = {
	.vsmReq = CMR_CAN_GLV_ON,
	.gear = CMR_CAN_GEAR_SLOW,
	.gearReq = CMR_CAN_GEAR_SLOW,
	.drsMode = CMR_CAN_DRSM_CLOSED,
	.drsReq = CMR_CAN_DRSM_CLOSED
};

void exitConfigScreen() {
	// the first time the user presses the exit button, it'll flush the memory to the cdc
	// the second time it'll exit the config screen because it'll be dependent having
	// recieved the message from CDC
	if (!flush_config_screen_to_cdc) {
		flush_config_screen_to_cdc = true;
		waiting_for_cdc_to_confirm_config = true;
		return;
	}
	if (!waiting_for_cdc_to_confirm_config) {
		in_config_screen = false;
		exit_config_request = false;
		return;
	}
}


/**
 * @brief Gets the VSM state.
 *
 * @note VSM state is maintained in the received CAN heartbeat.
 *
 * @return The VSM state.
 */
cmr_canState_t stateGetVSM(void) {
	cmr_canHeartbeat_t *payload = getPayload(CANRX_HEARTBEAT_VSM);
	return payload->state;
}


/**
 * @brief Gets the requested VSM state.
 *
 * @return The requested VSM state.
 */
cmr_canState_t stateGetVSMReq(void) {
	return state.vsmReq;
}

/**
 * @brief Gets the current gear.
 *
 * @return The current gear.
 */
cmr_canGear_t stateGetGear(void) {
	return state.gear;
}
/**
 * @brief Gets the requested gear.
 *
 * @return The requested gear.
 */
cmr_canGear_t stateGetGearReq(void) {
	return state.gearReq;
}

cmr_canDrsMode_t stateGetDrs(void) {
	return state.drsMode;
}

cmr_canDrsMode_t stateGetDrsReq(void) {
	return state.drsReq;
}

cmr_canDVMode_t stateGetDVMode(void) {
	return state.dvCtrlMode;
}

cmr_canDVMode_t stateGetDVReq(void) {
	return state.dvCtrlReq;
}

static uint32_t test_message_id = 0;

uint32_t get_test_message_id() {
	return test_message_id;
}

/**
 * @brief Returns the current car's speed in km/h
 */
float getSpeedKmh() {
	int32_t avgWheelRPM = getAverageWheelRPM();

	/* Wheel Speed to Vehicle Speed Conversion
	 *      (x rotations / 1min) * (18" * PI) *  (2.54*10^-5km/inch)
	 *      (60min / 1hr) * (1/15.1 gear ratio)
	 *      = x * 0.0057072960048526627892388896218624717297547517194475432371                                 */
	return (float)avgWheelRPM * 0.0057073f;
}

/**
 * @brief Returns the current car's odometry in km
 */
float getOdometer() {
	volatile cmr_canCDCOdometer_t *odometer = (volatile cmr_canCDCOdometer_t *)getPayload(CANRX_CDC_ODOMETER);
	return odometer->odometer_km;
}



/**
 * @brief Gets the average wheel speed reported by the inverters.
 *
 * @param none
 *
 * @return average wheel speed
 */
int32_t getAverageWheelRPM(void) {
	/* Get CAN data */

    volatile cmr_canDTI_TX_Erpm_t *dtiERPM_FL = getPayload(CANRX_DTI_FL_ERPM);
    volatile cmr_canDTI_TX_Erpm_t *dtiERPM_FR = getPayload(CANRX_DTI_FR_ERPM);
    volatile cmr_canDTI_TX_Erpm_t *dtiERPM_RL = getPayload(CANRX_DTI_RL_ERPM);
    volatile cmr_canDTI_TX_Erpm_t *dtiERPM_RR = getPayload(CANRX_DTI_RR_ERPM);

    const int32_t avgMotorSpeed_RPM = (
        + (int32_t)(dtiERPM_FL->erpm / POLE_PAIRS)
        + (int32_t)(dtiERPM_FR->erpm / POLE_PAIRS)
        + (int32_t)(dtiERPM_RL->erpm / POLE_PAIRS)
        + (int32_t)(dtiERPM_RR->erpm / POLE_PAIRS)
    ) / MOTOR_LEN;
	
    return avgMotorSpeed_RPM;
}



bool getAcknowledgeButton(void) {
	return ackButtonPressed;
}


/**
 * @brief Gets the highest motor temperature.
 *
 * @param none
 *
 * @return highest motor temperature in celsius, rounded to integer
 */
int getMaxMotorTemp(void){
	/* Get CAN data */
	// Front Left
	cmr_canDTI_TX_TempFault_t *canDTI_FL_temp = getPayload(CANRX_DTI_FL_TEMPFAULT);

	// Front Right
	cmr_canDTI_TX_TempFault_t *canDTI_FR_temp = getPayload(CANRX_DTI_FR_TEMPFAULT);

	// Rear Left
	cmr_canDTI_TX_TempFault_t *canDTI_RL_temp = getPayload(CANRX_DTI_RL_TEMPFAULT);

	// Rear Right
	cmr_canDTI_TX_TempFault_t *canDTI_RR_temp = getPayload(CANRX_DTI_RR_TEMPFAULT);

	/* Extract motor temperatures */
    //TODO: does this need to be int32_t or int16_t?? and what is multiplied by 10?
	int32_t frontLeftTemp = canDTI_FL_temp->motor_temp;
	int32_t frontRightTemp = canDTI_FR_temp->motor_temp;
	int32_t rearLeftTemp = canDTI_RL_temp->motor_temp;
	int32_t rearRightTemp = canDTI_RR_temp->motor_temp;

	/* Return highest motor temperature*/
	int32_t maxTemp = frontLeftTemp;

	maxTemp = max(max(max(maxTemp, frontRightTemp), rearLeftTemp), rearRightTemp);
/* conversion from dC to C*/
	return maxTemp / 10;

}

/**
 * @brief Gets the ac temperature.
 *
 * @param none
 *
 * @return ac temperature in celsius
 */
int getACTemp(void)
{
	volatile cmr_canBMSMinMaxCellTemperature_t *canHVCPackTemps = getPayload(CANRX_HVC_PACK_TEMPS);
	int32_t acTemp_C = (canHVCPackTemps->maxCellTemp_C) / 10;
	return acTemp_C;
}
/**
 * @brief Gets the mc temperature.
 *
 * @param none
 *
 * @return mc temperature in celsius
 */
int getMCTemp(void)
{
	/* Get CAN data */
	// Front Left
	cmr_canDTI_TX_TempFault_t *canDTI_FL_temp = getPayload(CANRX_DTI_FL_TEMPFAULT);

	// Front Right
	cmr_canDTI_TX_TempFault_t *canDTI_FR_temp = getPayload(CANRX_DTI_FR_TEMPFAULT);

	// Rear Left
	cmr_canDTI_TX_TempFault_t *canDTI_RL_temp = getPayload(CANRX_DTI_RL_TEMPFAULT);

	// Rear Right
	cmr_canDTI_TX_TempFault_t *canDTI_RR_temp = getPayload(CANRX_DTI_RR_TEMPFAULT);

    //TODO: does this need to be int32_t or int16_t?? and what is multiplied by 10?
    // is this controller temp?
	int32_t frontLeftMCTemp = canDTI_FL_temp->ctlr_temp;
	int32_t frontRightMCTemp = canDTI_FR_temp->ctlr_temp;
	int32_t rearLeftMCTemp = canDTI_RL_temp->ctlr_temp;
	int32_t rearRightMCTemp = canDTI_RR_temp->ctlr_temp;

	/* Return highest motor temperature*/
	int32_t maxTemp = frontLeftMCTemp;

	maxTemp = max(max(max(maxTemp, frontRightMCTemp), rearLeftMCTemp), rearRightMCTemp);
	return maxTemp / 10;
}

/**
 * @brief Gets the door state
 * true = closed
 * false = open or other
 *
 * @param none
 *
 * @return door state as integer
 */
bool DRSOpen(void)
{
	volatile cmr_canCDCDRSStates_t *drsState = (volatile cmr_canCDCDRSStates_t *)getPayload(CANRX_DRS_STATE);
    return drsState->state == CMR_CAN_DRS_STATE_OPEN;
}



static cmr_state getNextState(void) {
    if(stateGetVSM() == CMR_CAN_ERROR){
    	nextState = dimStateERROR;
    	return nextState;
    }
    switch (currState) {
        case INIT:
    		nextState = START;
            break;
        case START:
            if(state.vsmReq == CMR_CAN_GLV_ON) {
                nextState = NORMAL;
            }
            else {
                nextState = START;
            }
            break;
        case NORMAL:
            if(getASMS()) {
                nextState = AUTON;
            }
            else if(!cmr_gpioRead(GPIO_CTRL_SWITCH) && true/*(stateGetVSM() == CMR_CAN_GLV_ON || stateGetVSM() == CMR_CAN_HV_EN)*/) {
                nextState = CONFIG;
                flush_config_screen_to_cdc = false;
            }
            else if(buttonStates[RIGHT].isPressed && stateGetVSM() == CMR_CAN_RTD) {
                nextState = RACING;
                buttonStates[RIGHT].isPressed = false; 
            }
            else {
                nextState = NORMAL;
            }
            break;
        case CONFIG:
            if(cmr_gpioRead(GPIO_CTRL_SWITCH)) {
                nextState = NORMAL;
                flush_config_screen_to_cdc = true;
            }
            else if(buttonStates[LEFT].isPressed) {
                //move left on screen
                config_move_request = -1;
                nextState = CONFIG;
                buttonStates[LEFT].isPressed = false; 
            }
            else if(buttonStates[RIGHT].isPressed) {
                //move right on screen
                config_move_request = 1;
                nextState = CONFIG;
                buttonStates[RIGHT].isPressed = false; 
            }
            else if(buttonStates[UP].isPressed) {
                //move up on screen
                config_move_request = -CONFIG_SCREEN_NUM_COLS;
                nextState = CONFIG;
                buttonStates[UP].isPressed = false; 
            }
            else if(buttonStates[DOWN].isPressed) {
                //move down on screen
                config_move_request = CONFIG_SCREEN_NUM_COLS;
                nextState = CONFIG;
                buttonStates[DOWN].isPressed = false; 
            }
            else{
                nextState = CONFIG;
            }
            break;
        case RACING:
            if(buttonStates[LEFT].isPressed && stateGetVSM() == CMR_CAN_RTD) {
                nextState = NORMAL;
                buttonStates[LEFT].isPressed = false; 
            }
            else if(stateGetVSM() != CMR_CAN_RTD) {
                nextState = NORMAL;
            }
            else {
                nextState = RACING;
            }
            break;
        case AUTON:
            if(!getASMS()) {
                if(!(stateGetVSM() == CMR_CAN_AS_READY || stateGetVSM() == CMR_CAN_AS_DRIVING || 
                stateGetVSM() == CMR_CAN_AS_FINISHED || stateGetVSM() == CMR_CAN_AS_EMERGENCY)) {
                    nextState = NORMAL;
                }
                else {
                    nextState = AUTON;
                }
            }
            else {
                nextState = AUTON;
            }
            break;
        case dimStateERROR:
            nextState = INIT;
            break;
        default:
            nextState = INIT;
    }
	return nextState;
}

/**
 * @brief Checks if the requested VSM state is allowed.
 *
 * @param vsm The current VSM state.
 * @param vsmReq The requested VSM state.
 */
bool stateVSMReqIsValid(cmr_canState_t vsm, cmr_canState_t vsmReq) {
	switch (vsm) {
		case CMR_CAN_UNKNOWN:
			return (vsmReq == CMR_CAN_GLV_ON);
		case CMR_CAN_GLV_ON:
			return (vsmReq == CMR_CAN_GLV_ON) ||
				   (vsmReq == CMR_CAN_HV_EN);
		case CMR_CAN_HV_EN:
			return (vsmReq == CMR_CAN_GLV_ON) ||
				   (vsmReq == CMR_CAN_HV_EN) ||
				   (vsmReq == CMR_CAN_RTD);
		case CMR_CAN_RTD:
			return ((vsmReq == CMR_CAN_HV_EN)) ||
				   (vsmReq == CMR_CAN_RTD);
		case CMR_CAN_ERROR:
			return (vsmReq == CMR_CAN_GLV_ON);
		case CMR_CAN_CLEAR_ERROR:
			return (vsmReq == CMR_CAN_GLV_ON);
		default:
			break;
	}

	return false;
}


void EABStateUp() {
	cmr_canVSMState_t vsmState = stateGetVSM();
	if(getEAB() && getASMS() && vsmState == CMR_CAN_GLV_ON) {
		state.vsmReq = CMR_CAN_AS_READY;
	}
}

/**
 * @brief Handles VSM state up.
 */
void stateVSMUp() {

    if(getASMS()) {
        return;
    }

	cmr_canState_t vsmState = stateGetVSM();
	if (state.vsmReq < vsmState) {
		// Cancel state-down request.
		state.vsmReq = vsmState;
		return;
	}

    cmr_canState_t vsmReq = ((vsmState == CMR_CAN_UNKNOWN) || (vsmState == CMR_CAN_ERROR))
                                ? CMR_CAN_GLV_ON
                                : vsmState + 1;

	if (!stateVSMReqIsValid(vsmState, vsmReq)) {
		return;  // Invalid requested state.
	}
	state.vsmReq = vsmReq;
}

/**
 * @brief Handles VSM state down request.
 */
void stateVSMDown() {

    if(getASMS()) {
        return;
    }

	cmr_canState_t vsmState = stateGetVSM();
        if (state.vsmReq > vsmState) {
            // Cancel state-up request.
            state.vsmReq = vsmState;
            return;
        }

        if (state.vsmReq == CMR_CAN_RTD && getAverageWheelRPM() > 5) {
            // Only exit RTD when motor is basically stopped.
            return;
        }
        cmr_canState_t vsmReq = vsmState - 1;  // Decrement state.
	// Valid State
	if (stateVSMReqIsValid(vsmState, vsmReq)) {
		state.vsmReq = vsmReq;
		return;
	}
}



void reqVSM(void) {

    if(getASMS() && stateGetVSM() == CMR_CAN_GLV_ON) {
        EABStateUp();
        return;
    }
    
    if(stateGetVSM() == CMR_CAN_ERROR || stateGetVSM == CMR_CAN_CLEAR_ERROR) {
        state.vsmReq = CMR_CAN_GLV_ON;
        return;
    }
    if (getCurrState() != CONFIG) {
        if (buttonStates[UP].isPressed) {
            stateVSMUp();
            buttonStates[UP].isPressed = false; 
        } else if (buttonStates[DOWN].isPressed) {
            stateVSMDown();
            buttonStates[DOWN].isPressed = false; 
        }
        return;
    } 
}

//keeps track of requested gear
static volatile int requestedGear;

/**
* @brief Request Gear Change
*
* Clockwise is negative, counterclockwise is positive
* Turn clockwise is Gearup, counterclockwise is Geardown
* Request gear change if necessary
*
*/
void reqGear(void) {
    bool canChangeGear = ((stateGetVSM() == CMR_CAN_GLV_ON) 
                       || (stateGetVSM() == CMR_CAN_HV_EN));
    if(getASMS() && cmr_gpioRead(GPIO_CTRL_SWITCH)) {
        if(canChangeGear && buttonStates[RIGHT].isPressed) {
            if(state.gearReq == CMR_CAN_GEAR_DV_MISSION_MAX - 1) {
                state.gearReq = CMR_CAN_GEAR_DV_MISSION_MIN + 1;
                buttonStates[RIGHT].isPressed = false; 
            }
            else state.gearReq++;
        }
        else if(canChangeGear && buttonStates[LEFT].isPressed) {
            if(state.gearReq == CMR_CAN_GEAR_DV_MISSION_MIN + 1) {
                state.gearReq = CMR_CAN_GEAR_DV_MISSION_MAX - 1;
                buttonStates[LEFT].isPressed = false; 
            }
            else state.gearReq--;
        }
    }
    else if (!getASMS()) {
        if(canChangeGear && buttonStates[RIGHT].isPressed) {
            if(state.gearReq == CMR_CAN_GEAR_MAX - 1) {
                state.gearReq = CMR_CAN_GEAR_MIN + 1;
            }
            else state.gearReq++;
            buttonStates[RIGHT].isPressed = false; 
        }
        else if(canChangeGear && buttonStates[LEFT].isPressed) {
            if(state.gearReq == CMR_CAN_GEAR_MIN + 1) {
                state.gearReq = CMR_CAN_GEAR_MAX - 1;
            }
            else state.gearReq--;
            buttonStates[LEFT].isPressed = false; 
        }
    }
}

void reqDRS(void) {
    if(buttonStates[SW_RIGHT].isPressed) {
        state.drsReq = CMR_CAN_DRS_STATE_OPEN;
        buttonStates[SW_RIGHT].isPressed = false; 
    }
    else {
        state.drsReq = CMR_CAN_DRS_STATE_CLOSED;
    }
}

void reqDVCtrl(void) {
    if(cmr_gpioRead(GPIO_CTRL_SWITCH)) {
        if(buttonStates[RIGHT].isPressed) {
            state.dvCtrlReq = (state.dvCtrlMode + 1) % NUM_DV_MODES;
            buttonStates[RIGHT].isPressed = false; 
        }
        else if(buttonStates[LEFT].isPressed) {
            state.dvCtrlReq = (state.dvCtrlMode - 1) % NUM_DV_MODES;
            buttonStates[LEFT].isPressed = false; 
        }
    }
}

/**
 * @brief Updates the gear to be the requested gear.
 */
void stateGearUpdate(void) {
	state.gear = state.gearReq;
}

void stateDrsUpdate(void) {
	state.drsMode = state.drsReq;
}

void stateDVCtrlUpdate(void) {
    state.dvCtrlMode = state.dvCtrlReq;
}

cmr_state getCurrState() {
    return currState;
}

/**
 * @brief Struct for voltage SoC lookup table
 */
typedef struct {
    float voltage;
    uint8_t SoC;
} voltage_SoC_t;

#define LV_GRAPOW_LUT_NUM_ITEMS 11
#define S_COUNT 7.0f


/**
 * @brief Look up table for 26x 7s Grapow LV Battery (Semi-Solid State)
 *
 * Must be sorted in descending order
 */
static voltage_SoC_t LV_grapow_SoC_lookup[LV_GRAPOW_LUT_NUM_ITEMS] = {
    { 4.20f * S_COUNT, 100 }, 
    { 3.94f * S_COUNT,  90 },
    { 3.88f * S_COUNT,  80 }, 
    { 3.78f * S_COUNT,  70 }, 
    { 3.65f * S_COUNT,  60 }, 
    { 3.50f * S_COUNT,  50 }, 
    { 3.38f * S_COUNT,  40 }, 
    { 3.25f * S_COUNT,  30 }, 
    { 3.08f * S_COUNT,  20 }, 
    { 2.91f * S_COUNT,  10 }, 
    { 2.50f * S_COUNT,   0 }  
};


/**
 * @brief Function for getting Low Voltage SoC
 *
 * @param voltage the current LV Voltage.
 *
 * @return the state of charge % between 0 and 99.
 */
uint8_t getLVSoC(float voltage) {
    voltage_SoC_t *lut = LV_grapow_SoC_lookup;
    size_t num_items = LV_GRAPOW_LUT_NUM_ITEMS;

    for (size_t i = 0; i < num_items; i++) {
        if (lut[i].voltage == voltage) {
            return lut[i].SoC;
        }

        if (lut[i].voltage < voltage) {
            // if voltage > voltage from lut, we have passed correct value
            if (i == 0) {
                // if i == 0, then it must be higher than highest voltage
                return 99;
            }
            // otherwise we do some linear extrapolation!
            float result =
                (float)lut[i].SoC + ((voltage - lut[i].voltage) /
                                     (lut[i - 1].voltage - lut[i].voltage)) *
                                        ((float)(lut[i - 1].SoC - lut[i].SoC));
            return min(99, ((uint8_t)result));
        }
    }
    // if we get to end of loop, voltage is less than lowest voltage in lut
    return 0;
}

cmr_canState_t vsmStateGlobal;
cmr_canState_t vsmStateGlobalReq;


static void stateMachine(void *pvParameters){
    (void)pvParameters;
    TickType_t lastWakeTime = xTaskGetTickCount();
    currState = INIT;
    uint32_t test;
    uint32_t space1 = 0;
    while (1) {
        // taskENTER_CRITICAL();
        currState = getNextState();
        // if(getASMS()) {
        //     // TODO: checks for brakes, dv system, etc
        //     if(stateGetVSM() == CMR_CAN_GLV_ON) {
        //         cmr_gpioWrite(GPIO_AS_RELAY, 0);
        //     }
        //     else {
        //         cmr_gpioWrite(GPIO_AS_RELAY, 1);
        //     }
        // }
        // tftRead(&tft, TFT_ADDR_CMD_READ, sizeof(test), &test);
        // test = test & 0x00000FFF;
        // tftRead(&tft, TFT_ADDR_CMDB_SPACE, sizeof(space1), &space1);
		/* for testing
		vsmStateGlobal = stateGetVSM();
		vsmStateGlobalReq = stateGetVSMReq();
		*/
        // taskEXIT_CRITICAL();
		vTaskDelayUntil(&lastWakeTime, stateMachine_period);
    }
}

/**
 * @brief Initializes the state machine interface.
 */
void stateMachineInit(void) {
    cmr_taskInit(
        &stateMachine_task,
        "stateMachine",
        stateMachine_priority,
        stateMachine,
        NULL);
}

