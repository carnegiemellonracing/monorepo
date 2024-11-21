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
            if(gpioLRUDStates[LEFT]) {
                nextState = CONFIG;
                gpioLRUDStates[LEFT] = false;
            }
            else if(gpioLRUDStates[RIGHT]) {
                nextState = RACING;
                gpioLRUDStates[RIGHT] = false;
            }
            else nextState = NORMAL;
            break;
        case CONFIG:
            //look into how button move on screen on campus
            if(gpioLRUDStates[LEFT]) {
                //move left on screen
                gpioLRUDStates[LEFT] = 0;
                nextState = CONFIG;
            }
            else if(gpioLRUDStates[RIGHT]) {
                //move right on screen
                gpioLRUDStates[RIGHT] = 0;
                nextState = CONFIG;
            }
            else if(gpioLRUDStates[UP]) {
                //move up on screen
                gpioLRUDStates[UP] = 0;
                nextState = CONFIG;
            }
            else if(gpioLRUDStates[DOWN]) {
                //move down on screen
                gpioLRUDStates[DOWN] = 0;
                nextState = CONFIG;
            }
    	//TODO: WHAT THE HELL IS THIS??
            else if(gpioButtonStates[SW1]) {
                nextState = NORMAL;
                gpioButtonStates[SW1] = 0;
                nextState = CONFIG;
            }
            else if(gpioButtonStates[SW2]) {
                nextState = RACING;
                gpioButtonStates[SW2] = 0;
            }
            else nextState = CONFIG;
            break;
        case dimStateERROR:
            nextState = INIT;
            break;
        case RACING:
            if(gpioLRUDStates[LEFT] && stateGetVSMReq() == CMR_CAN_GLV_ON) {
                nextState = CONFIG;
                gpioLRUDStates[UP] = false;
            }
            else if(gpioLRUDStates[RIGHT]) {
                nextState = NORMAL;
                gpioLRUDStates[RIGHT] = false;
            }
            else nextState = RACING;
            break;
        default:
            nextState = INIT;
    }
	return nextState;
}

void reqVSM(void) {
	//So check state of the car and the pressing of up or down button, then based on the output
	//update VstateVSMReq
	if (((stateGetVSM() == CMR_CAN_GLV_ON) || (stateGetVSM() == CMR_CAN_HV_EN)) && (gpioLRUDStates[UP])){
		StateVSMUp();
	}
	if (((stateGetVSM() == CMR_CAN_GLV_ON) || (stateGetVSM() == CMR_CAN_HV_EN)) && (gpioLRUDStates[DOWN])){
		StateVSMDown();
	}
}

//keeps track of requested gear
volatile static int requestedGear;

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
	int currGear = stateGetGear();

	bool canChangeGear = ((stateGetVSM() == CMR_CAN_GLV_ON) || (stateGetVSM() == CMR_CAN_HV_EN));
	if(canChangeGear && (currentRotary!=stateGetGear())){
		if((currentRotary < pastRotary) || (currentRotary == 7 && pastRotary==0)){
			//turned clockwise so gearup
			requestedGear = stateGetGear() + 1;
			}
		} else {
				requestedGear = stateGetGear() - 1;


	}
}

//returns requested gear
int getRequestedGear(void){
	return requestedGear;
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
            cmr_gpioWrite(0, 0);  // TODO figure out pin
            vTaskDelayUntil(&lastWakeTime, TFT_RESET_MS);
            cmr_gpioWrite(0, 1);
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
            //    vTaskDelayUntil(&lastWakeTime, TFT_STARTUP_MS);
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
//TODO: I think the best idea is to deleted getReqScreen() from the task and change tft.c to read the current state from dim instead of can
static void stateMachine(void *pvParameters){
    (void)pvParameters;
    TickType_t lastWakeTime = xTaskGetTickCount();
    currState = INIT;
    while (1) {
    	//TODO: WHAT IS CRITICAL?
        taskENTER_CRITICAL();
    	//TODO: CALLED TWICE getReqScreen()?
        getReqScreen();
        stateOutput();
        taskEXIT_CRITICAL();
    }
    vTaskDelayUntil(&lastWakeTime, stateMachine_period);
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

