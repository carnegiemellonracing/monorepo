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

// cmr_canGear_t reqGear;
// cmr_canGear_t currGear;

volatile int8_t config_move_request;


#define max(a,b) ((a) > (b) ? (a) : (b))

#define min(a, b) __extension__\
({ __typeof__ (a) _a = (a); \
__typeof__ (b) _b = (b); \
_a < _b ? _a : _b; })

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
	// Front Left
	cmr_canAMKActualValues1_t *canAMK_FL_Act1 = getPayload(CANRX_AMK_FL_ACT_1);

	// Front Right
	cmr_canAMKActualValues1_t *canAMK_FR_Act1 = getPayload(CANRX_AMK_FR_ACT_1);

	// Rear Left
	cmr_canAMKActualValues1_t *canAMK_RL_Act1 = getPayload(CANRX_AMK_RL_ACT_1);

	// Rear Right
	cmr_canAMKActualValues1_t *canAMK_RR_Act1 = getPayload(CANRX_AMK_RL_ACT_1);

	/* Extract wheel speeds */
	int32_t frontLeftRPM = (canAMK_FL_Act1->velocity_rpm);  // Motor direction reversed on left side
	int32_t frontRightRPM = canAMK_FR_Act1->velocity_rpm;
	int32_t rearLeftRPM = (canAMK_RL_Act1->velocity_rpm);  // Motor direction reversed on left side
	int32_t rearRightRPM = canAMK_RR_Act1->velocity_rpm;

	/* Compute average */
	int32_t average = (frontLeftRPM + frontRightRPM + rearLeftRPM + rearRightRPM) / 4;

	return average;
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
	cmr_canAMKActualValues2_t *canAMK_FL_Act2 = getPayload(CANRX_AMK_FL_ACT_2);

	// Front Right
	cmr_canAMKActualValues2_t *canAMK_FR_Act2 = getPayload(CANRX_AMK_FR_ACT_2);

	// Rear Left
	cmr_canAMKActualValues2_t *canAMK_RL_Act2 = getPayload(CANRX_AMK_RL_ACT_2);

	// Rear Right
	cmr_canAMKActualValues2_t *canAMK_RR_Act2 = getPayload(CANRX_AMK_RR_ACT_2);

	/* Extract motor temperatures */
	int32_t frontLeftTemp = canAMK_FL_Act2->motorTemp_dC;
	int32_t frontRightTemp = canAMK_FR_Act2->motorTemp_dC;
	int32_t rearLeftTemp = canAMK_RL_Act2->motorTemp_dC;
	int32_t rearRightTemp = canAMK_RR_Act2->motorTemp_dC;

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
	volatile cmr_canHVCPackMinMaxCellTemps_t *canHVCPackTemps = getPayload(CANRX_HVC_PACK_TEMPS);
	int32_t acTemp_C = (canHVCPackTemps->maxCellTemp_dC) / 10;
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
	cmr_canAMKActualValues2_t *canAMK_FL_Act2 = getPayload(CANRX_AMK_FL_ACT_2);
	// Front Right
	cmr_canAMKActualValues2_t *canAMK_FR_Act2 = getPayload(CANRX_AMK_FR_ACT_2);
	// Rear Left
	cmr_canAMKActualValues2_t *canAMK_RL_Act2 = getPayload(CANRX_AMK_RL_ACT_2);
	// Rear Right
	cmr_canAMKActualValues2_t *canAMK_RR_Act2 = getPayload(CANRX_AMK_RR_ACT_2);
	int32_t frontLeftMCTemp = canAMK_FL_Act2->motorTemp_dC;
	int32_t frontRightMCTemp = canAMK_FR_Act2->motorTemp_dC;
	int32_t rearLeftMCTemp = canAMK_RL_Act2->motorTemp_dC;
	int32_t rearRightMCTemp = canAMK_RR_Act2->motorTemp_dC;

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


static cmr_state getReqScreen(void) {
    if(stateGetVSM() == CMR_CAN_ERROR){
    	nextState = dimStateERROR;
    	return nextState;
    }
    /*case if we use safety screen
    if(stateGetVSM() == CMR_CAN_ERROR) {
        if(stateGetVSMReq() == CMR_CAN_HV_EN) return SAFETY;
        return dimStateERROR;
    }
    */
    switch (currState) {
        case INIT:
        	//initializes tft screen
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
            if(canLRUDStates[LEFT]) {
                nextState = CONFIG;
                flush_config_screen_to_cdc = false;
				//gpioLRUDStates[LEFT] = false;
            }
            else if(canLRUDStates[RIGHT]) {
                nextState = RACING;
                //canLRUDStates[RIGHT] = false;
            }
            else {
                nextState = NORMAL;
            }
            break;
        case CONFIG:
            //look into how button move on screen on campus
            if(canLRUDStates[LEFT]) {
                //move left on screen
                config_move_request = -1;
                nextState = CONFIG;
            }
            else if(canLRUDStates[RIGHT]) {
                //move right on screen
                config_move_request = 1;
                nextState = CONFIG;
            }
            else if(canLRUDStates[UP]) {
                //move up on screen
                config_move_request = -CONFIG_SCREEN_NUM_COLS;
                nextState = CONFIG;
            }
            else if(canLRUDStates[DOWN]) {
                //move down on screen
                config_move_request = CONFIG_SCREEN_NUM_COLS;
                nextState = CONFIG;
            }
    	//TODO: WHAT THE HELL IS THIS??
            else if(!cmr_gpioRead(GPIO_BUTTON_SW1)) {
            // else if(canLRUDStates[LEFT]) {
                nextState = NORMAL;
                flush_config_screen_to_cdc = true;
                // exitConfigScreen();

                //gpioButtonStates[SW1] = 0;
                //nextState = CONFIG;
            }
            else if(false) {
                flush_config_screen_to_cdc = true;
                // exitConfigScreen();
                nextState = RACING;
                //gpioButtonStates[SW2] = 0;
            }
            else{
                nextState = CONFIG;
            }
            break;
        case dimStateERROR:
            nextState = INIT;
            break;
        case RACING:
            if(canLRUDStates[LEFT] && state.vsmReq == CMR_CAN_GLV_ON) {
                nextState = CONFIG;
                flush_config_screen_to_cdc = false;
                //canLRUDStates[LEFT] = false;
            }
            else if(canLRUDStates[RIGHT]) {
                nextState = NORMAL;
                //canLRUDStates[RIGHT] = false;
            }
            else {
                nextState = RACING;
            }
            break;
        default:
            nextState = INIT;
    }
    //change all can states to false to deregister buttons
    for(int i = 0; i < LRUD_LEN; i++){
        canLRUDStates[i] = false;
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


/**
 * @brief Handles VSM state up.
 */
void stateVSMUp() {
	cmr_canState_t vsmState = stateGetVSM();
	if (state.vsmReq < vsmState) {
		// Cancel state-down request.
		state.vsmReq = vsmState;
		return;
	}

	cmr_canState_t vsmReq = ((vsmState == CMR_CAN_UNKNOWN) || (vsmState == CMR_CAN_ERROR))
								? (CMR_CAN_GLV_ON)  // Unknown state; request GLV_ON.
								: (vsmState + 1);   // Increment state.
	if (!stateVSMReqIsValid(vsmState, vsmReq)) {
		return;  // Invalid requested state.
	}

	state.vsmReq = vsmReq;
}

/**
 * @brief Handles VSM state down request.
 */
void stateVSMDown() {
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
	//So check state of the car and the pressing of up or down button, then based on the output
	//update VstateVSMReq
    if(stateGetVSM() == CMR_CAN_ERROR || stateGetVSM == CMR_CAN_CLEAR_ERROR) {
        state.vsmReq = CMR_CAN_GLV_ON;
    }
    else {
        if (((stateGetVSM() == CMR_CAN_GLV_ON) || (stateGetVSM() == CMR_CAN_HV_EN) || (stateGetVSM() == CMR_CAN_UNKNOWN)) && (canLRUDStates[UP])){
            if(getCurrState() != CONFIG){
                stateVSMUp();
            }
        }
        if (((stateGetVSM() == CMR_CAN_RTD) || (stateGetVSM() == CMR_CAN_HV_EN)) && (canLRUDStates[DOWN])){
            if(getCurrState() != CONFIG){
                stateVSMDown();
            }
        }
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
	int pastRotary = getPastRotaryPosition();
	int currentRotary = getRotaryPosition();

	// bool canChangeGear = ((stateGetVSM() == CMR_CAN_GLV_ON) || (stateGetVSM() == CMR_CAN_HV_EN));
    // bool canChangeGear = true;
	// if(canChangeGear && (currentRotary!=state.gear)){
	// 	if((currentRotary < pastRotary) || (currentRotary == 7 && pastRotary==0)){
	// 		//turned clockwise so gearup
	// 		state.gear++;
	// 		}
	// 	} else {
	// 		state.gear--;
	// }
    bool canChangeGear = true;
    if(canChangeGear && (pastRotary != currentRotary)) {
        if(currState == CONFIG) config_increment_up_requested = true;
        if(state.gearReq == 8) state.gearReq = 1;
        else state.gearReq++;
    }
}

//returns requested gear

int getRequestedGear(void){
	return requestedGear;
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

cmr_state getCurrState() {
    return currState;
}

static void stateOutput() {
    //output
    // switch(currState) {
    //     case INIT:
    //         //initialize buttons to 0
	// 		//also initializes all LRUD buttons to 0
    //         for(int i=0; i<NUM_BUTTONS; i++){
    //             //is it necessary to initialize the can buttons to 0 if they are just reading pins??
    //             canButtonStates[i] = 0;
    //             gpioButtonStates[i] = 0;
    //         }
    //          /* Restarting the Display. */
    //         TickType_t lastWakeTime = xTaskGetTickCount();
    // 		//change pin of screen
    //         /* Initialize the display. */
    //         // tftInitSequence();
    //         tftUpdate();
    //         break;
    //     case START:
    //         /* Display Startup Screen for fixed time */
    //         //tftDLWrite(&tft, &tftDL_startup);
    //         //drawConfigScreen();
    //         //vTaskDelayUntil(&lastWakeTime, TFT_STARTUP_MS);
    //         break;
    //     case NORMAL:
    //         drawRTDScreen(); //from somethingP
    //         //tftDLWrite(&tft, &tftDL_startup);
    //         //vTaskDelayUntil(&lastWakeTime, TFT_STARTUP_MS);
    //         break;
    //     case CONFIG:
    //         drawConfigScreen();
    //         vTaskDelayUntil(&lastWakeTime, TFT_STARTUP_MS);
    //         break;
    //     case dimStateERROR:
    //         drawErrorScreen();
    //         vTaskDelayUntil(&lastWakeTime, TFT_STARTUP_MS);
    //         break;
    //     case RACING:
    //         drawRacingScreen();
    //         vTaskDelayUntil(&lastWakeTime, TFT_STARTUP_MS);
    //         break;
    // }
	//TODO: Why is this called again?
    currState = getReqScreen();
}



/**
 * @brief Struct for voltage SoC lookup table
 */
typedef struct {
    float voltage;
    uint8_t SoC;
} voltage_SoC_t;

#define LV_LIFEPO_LUT_NUM_ITEMS 11
#define LV_LIPO_LUT_NUM_ITEMS 6

/**
 * @brief Look up table for 24v lifepo4 battery State of Charge
 *
 * Must be sorted in descending order
 */
static voltage_SoC_t LV_LiFePo_SoC_lookup[LV_LIFEPO_LUT_NUM_ITEMS] = {
    { 27.2f, 100 },
    { 26.8f, 90 },
    { 26.6f, 80 },
    { 26.1f, 70 },
    { 26.4f, 60 },
    { 26.1f, 50 },
    { 26.0f, 40 },
    { 25.8f, 30 },
    { 25.6f, 20 },
    { 24.0f, 10 },
    { 20.0f, 0 }
};

static voltage_SoC_t LV_LiPo_SoC_lookup[LV_LIPO_LUT_NUM_ITEMS] = {
    { 25.2f, 100 },
    { 24.5f, 90 },
    { 23.0f, 80 },
    { 21.0f, 20 },
    { 20.0f, 10 },
    { 18.0f, 0 }
};

/**
 * @brief Function for getting Low Voltage SoC
 *
 * @param voltage the current LV Voltage.
 *
 * @return the state of charge % between 0 and 99.
 */
uint8_t getLVSoC(float voltage, lv_battery_type_t battery_type) {
    voltage_SoC_t *lut;
    size_t num_items;

    if (battery_type == LV_LIFEPO) {
        lut = LV_LiFePo_SoC_lookup;
        num_items = LV_LIFEPO_LUT_NUM_ITEMS;
    } else if (battery_type == LV_LIPO) {
        lut = LV_LiPo_SoC_lookup;
        num_items = LV_LIPO_LUT_NUM_ITEMS;
    } else {
        // unknown battery type - return 0%
		cmr_panic("Unknown battery type");
        return 0;
    }

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
        // stateOutput();
        currState = getReqScreen();
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

