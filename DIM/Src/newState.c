/**
 * @file newState.c
 * @brief Firmware entry point.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h>  // HAL interface

#include <CMR/panic.h>  // cmr_panic()
#include <CMR/can.h>    // CAN interface
#include <CMR/adc.h>    // ADC interface
#include <CMR/gpio.h>   // GPIO interface
#include <CMR/can_types.h>

#include "gpio.h"       // Board-specific GPIO interface
#include "can.h"        // Board-specific CAN interface
#include "tft.h"        // TFT display interface.
#include "newState.h"
#include <stdlib.h>
#include <stdio.h>
#include <tftDL.h>

static const uint32_t stateMachine_priority = 4;

/** @brief Button input task period (milliseconds). */
static const TickType_t stateMachine_period = 10;

/** @brief Button input task task. */
static cmr_task_t stateMachine_task;

cmr_state nextState;

cmr_state currState;

volatile int8_t config_move_request;




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
	cmr_canRXMeta_t *heartbeatVSMMeta = canRXMeta + CANRX_HEARTBEAT_VSM;
	volatile cmr_canHeartbeat_t *heartbeatVSM =
		(void *)heartbeatVSMMeta->payload;

	return heartbeatVSM->state;
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
	float vehicleSpeed = (float)avgWheelRPM * 0.0057073f;

	return vehicleSpeed;
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
	cmr_canRXMeta_t *metaAMK_FL_Act1 = canRXMeta + CANRX_AMK_FL_ACT_1;
	volatile cmr_canAMKActualValues1_t *canAMK_FL_Act1 =
		(void *)metaAMK_FL_Act1->payload;
	// Front Right
	cmr_canRXMeta_t *metaAMK_FR_Act1 = canRXMeta + CANRX_AMK_FR_ACT_1;
	volatile cmr_canAMKActualValues1_t *canAMK_FR_Act1 =
		(void *)metaAMK_FR_Act1->payload;
	// Rear Left
	cmr_canRXMeta_t *metaAMK_RL_Act1 = canRXMeta + CANRX_AMK_RL_ACT_1;
	volatile cmr_canAMKActualValues1_t *canAMK_RL_Act1 =
		(void *)metaAMK_RL_Act1->payload;
	// Rear Right
	cmr_canRXMeta_t *metaAMK_RR_Act1 = canRXMeta + CANRX_AMK_RR_ACT_1;
	volatile cmr_canAMKActualValues1_t *canAMK_RR_Act1 =
		(void *)metaAMK_RR_Act1->payload;

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
	cmr_canRXMeta_t *metaAMK_FL_Act2 = canRXMeta + CANRX_AMK_FL_ACT_2;
	volatile cmr_canAMKActualValues2_t *canAMK_FL_Act2 =
		(void *)metaAMK_FL_Act2->payload;
	// Front Right
	cmr_canRXMeta_t *metaAMK_FR_Act2 = canRXMeta + CANRX_AMK_FR_ACT_2;
	volatile cmr_canAMKActualValues2_t *canAMK_FR_Act2 =
		(void *)metaAMK_FR_Act2->payload;
	// Rear Left
	cmr_canRXMeta_t *metaAMK_RL_Act2 = canRXMeta + CANRX_AMK_RL_ACT_2;
	volatile cmr_canAMKActualValues2_t *canAMK_RL_Act2 =
		(void *)metaAMK_RL_Act2->payload;
	// Rear Right
	cmr_canRXMeta_t *metaAMK_RR_Act2 = canRXMeta + CANRX_AMK_RR_ACT_2;
	volatile cmr_canAMKActualValues2_t *canAMK_RR_Act2 =
		(void *)metaAMK_RR_Act2->payload;

	/* Extract motor temperatures */
	int32_t frontLeftTemp = canAMK_FL_Act2->motorTemp_dC;
	int32_t frontRightTemp = canAMK_FR_Act2->motorTemp_dC;
	int32_t rearLeftTemp = canAMK_RL_Act2->motorTemp_dC;
	int32_t rearRightTemp = canAMK_RR_Act2->motorTemp_dC;

	/* Return highest motor temperature*/
	int32_t maxTemp = frontLeftTemp;

	if( maxTemp < frontRightTemp ){
		maxTemp = frontRightTemp;
	}
	if (maxTemp < rearLeftTemp )
	{
		maxTemp = rearLeftTemp;
	}
	if (maxTemp < rearRightTemp )
	{
		maxTemp = rearRightTemp;
	}
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
	cmr_canRXMeta_t *metaHVCPackTemps = canRXMeta + CANRX_HVC_PACK_TEMPS;
	volatile cmr_canHVCPackMinMaxCellTemps_t *canHVCPackTemps =
		(void *)metaHVCPackTemps->payload;
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
	cmr_canRXMeta_t *metaAMK_FL_Act2 = canRXMeta + CANRX_AMK_FL_ACT_2;
	volatile cmr_canAMKActualValues2_t *canAMK_FL_Act2 =
		(void *)metaAMK_FL_Act2->payload;
	// Front Right
	cmr_canRXMeta_t *metaAMK_FR_Act2 = canRXMeta + CANRX_AMK_FR_ACT_2;
	volatile cmr_canAMKActualValues2_t *canAMK_FR_Act2 =
		(void *)metaAMK_FR_Act2->payload;
	// Rear Left
	cmr_canRXMeta_t *metaAMK_RL_Act2 = canRXMeta + CANRX_AMK_RL_ACT_2;
	volatile cmr_canAMKActualValues2_t *canAMK_RL_Act2 =
		(void *)metaAMK_RL_Act2->payload;
	// Rear Right
	cmr_canRXMeta_t *metaAMK_RR_Act2 = canRXMeta + CANRX_AMK_RR_ACT_2;
	volatile cmr_canAMKActualValues2_t *canAMK_RR_Act2 =
		(void *)metaAMK_RR_Act2->payload;
	int32_t frontLeftMCTemp = canAMK_FL_Act2->motorTemp_dC;
	int32_t frontRightMCTemp = canAMK_FR_Act2->motorTemp_dC;
	int32_t rearLeftMCTemp = canAMK_RL_Act2->motorTemp_dC;
	int32_t rearRightMCTemp = canAMK_RR_Act2->motorTemp_dC;

	/* Return highest motor temperature*/
	int32_t maxTemp = frontLeftMCTemp;

	if( maxTemp < frontRightMCTemp ){
		maxTemp = frontRightMCTemp;
	}
	if (maxTemp < rearLeftMCTemp )
	{
		maxTemp = rearLeftMCTemp;
	}
	if (maxTemp < rearRightMCTemp )
	{
		maxTemp = rearRightMCTemp;
	}
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
bool getDoorsState(void)
{
	cmr_canRXMeta_t *meta_canCDCDRSStates_t = canRXMeta + CANRX_DRS_STATE;
	volatile cmr_canCDCDRSStates_t *canCDCDRSState =
		(void *) meta_canCDCDRSStates_t->payload;

	int16_t doorState = canCDCDRSState->state;
	if (doorState == 0)
	{
		return true;
	}else{
		return false;
	}
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
        	tftUpdate(&tft);
    		nextState = START;

            break;
        case START:
            if(stateGetVSMReq() == CMR_CAN_GLV_ON) {
                nextState = NORMAL;
            }
            else nextState = START;
            break;
        case NORMAL:
            if(canLRUDStates[LEFT]) {
                nextState = CONFIG;
				//gpioLRUDStates[LEFT] = false;
            }
            else if(canLRUDStates[RIGHT]) {
                nextState = RACING;
                //canLRUDStates[RIGHT] = false;
            }
            else nextState = NORMAL;
            break;
        case CONFIG:
            //look into how button move on screen on campus
            if(canLRUDStates[LEFT]) {
                //move left on screen
                //canLRUDStates[LEFT] = 0;
                nextState = CONFIG;
            }
            else if(canLRUDStates[RIGHT]) {
                //move right on screen
                //canLRUDStates[RIGHT] = 0;
                nextState = CONFIG;
            }
            else if(canLRUDStates[UP]) {
                //move up on screen
                //canLRUDStates[UP] = 0;
                nextState = CONFIG;
            }
            else if(canLRUDStates[DOWN]) {
                //move down on screen
                //canLRUDStates[DOWN] = 0;
                nextState = CONFIG;
            }
    	//TODO: WHAT THE HELL IS THIS??
            else if(gpioButtonStates[SW1]) {
                nextState = NORMAL;
                //gpioButtonStates[SW1] = 0;
                nextState = CONFIG;
            }
            else if(gpioButtonStates[SW2]) {
                nextState = RACING;
                //gpioButtonStates[SW2] = 0;
            }
            else nextState = CONFIG;
            break;
        case dimStateERROR:
            nextState = INIT;
            break;
        case RACING:
            if(canLRUDStates[LEFT] && stateGetVSMReq() == CMR_CAN_GLV_ON) {
                nextState = CONFIG;
                //canLRUDStates[LEFT] = false;
            }
            else if(canLRUDStates[RIGHT]) {
                nextState = NORMAL;
                //canLRUDStates[RIGHT] = false;
            }
            else nextState = RACING;
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

	if (
		state.vsmReq == CMR_CAN_RTD &&
		getAverageWheelRPM() > 5) {
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
	if (((stateGetVSM() == CMR_CAN_GLV_ON) || (stateGetVSM() == CMR_CAN_HV_EN)) && (gpioLRUDStates[UP])){
		stateVSMUp();
	}
	if (((stateGetVSM() == CMR_CAN_GLV_ON) || (stateGetVSM() == CMR_CAN_HV_EN)) && (gpioLRUDStates[DOWN])){
		stateVSMDown();
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

	bool canChangeGear = ((stateGetVSM() == CMR_CAN_GLV_ON) || (stateGetVSM() == CMR_CAN_HV_EN));
	if(canChangeGear && (currentRotary!=stateGetGear())){
		if((currentRotary < pastRotary) || (currentRotary == 7 && pastRotary==0)){
			//turned clockwise so gearup
			requestedGear = stateGetGear() + 1;
			}
		} else {
				requestedGear = stateGetGear() - 1;

		state.gearReq = requestedGear;
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

static void stateOutput() { 
    //output
    switch(currState) {
        case INIT:
            //initialize buttons to 0
			//also initializes all LRUD buttons to 0
            for(int i=0; i<NUM_BUTTONS; i++){
                //is it necessary to initialize the can buttons to 0 if they are just reading pins??
                canButtonStates[i] = 0;
                gpioButtonStates[i] = 0;
            }
    		for (int i=0; i<LRUDLen; i++) {
    			canButtonStates[i] = 0;
    			gpioButtonStates[i] = 0;
    		}
             /* Restarting the Display. */
            TickType_t lastWakeTime = xTaskGetTickCount();
    		//change pin of screen
            cmr_gpioWrite(GPIO_PD_N, 0);  // TODO figure out pin
            vTaskDelayUntil(&lastWakeTime, TFT_RESET_MS);
            cmr_gpioWrite(GPIO_PD_N, 1);
            vTaskDelayUntil(&lastWakeTime, TFT_RESET_MS);

            /* Initialize the display. */
            tftCmd(&tft, TFT_CMD_CLKEXT, 0x00);
            tftCmd(&tft, TFT_CMD_ACTIVE, 0x00);
            tftCmd(&tft, TFT_CMD_ACTIVE, 0x00);
            break;
        case START:
            /* Display Startup Screen for fixed time */
            tftDLContentLoad(&tft, &tftDL_startup);
            tftDLWrite(&tft, &tftDL_startup);
            vTaskDelayUntil(&lastWakeTime, TFT_STARTUP_MS);
            break;
        case NORMAL:
            drawRTDScreen(); //from something
            break;
        case CONFIG:
            drawConfigScreen();
            break;
        case dimStateERROR:
            drawErrorScreen();
            break;
        case RACING:
            drawRacingScreen();
            break;
    }
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
        return 0;
    }

    for (size_t i = 0; i < num_items; i++) {
        if (lut[i].voltage == voltage) {
            // if voltage equals voltage from lut, return soc
            return lut[i].SoC;
        } else if (lut[i].voltage < voltage) {
            // if voltage > voltage from lut, we have passed correct value
            if (i == 0) {
                // if i == 0, then it must be higher than highest voltage
                return 99;
            } else {
                // otherwise we do some linear extrapolation!
                float result = (float)lut[i].SoC + ((voltage - lut[i].voltage) / (lut[i - 1].voltage - lut[i].voltage)) * ((float)(lut[i - 1].SoC - lut[i].SoC));
                return min(99, ((uint8_t)result));
            }
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
    while (1) {
        //taskENTER_CRITICAL();
        getReqScreen();
        stateOutput();
		/* for testing
		vsmStateGlobal = stateGetVSM();
		vsmStateGlobalReq = stateGetVSMReq();
		*/
        //taskEXIT_CRITICAL();
		vTaskDelayUntil(&lastWakeTime, stateMachine_period);
    }
}
//want to pack into cmr driver 

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

