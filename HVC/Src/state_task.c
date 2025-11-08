/*
 * state_task.c
 *
 *  Created on: Jun 8, 2020
 *      Author: vamsi
 */
#include "state_task.h"
#include "errors.h"
#include <stdlib.h>

static cmr_canHVCState_t currentState = CMR_CAN_HVC_STATE_ERROR;

#define PRECHARGE_THRESH 57000

static bool cellBalancing = false; 

/*
 * External Accessor Functions
 */

cmr_canHVCState_t getState() {
    return currentState;
}

/*
 * Helper Functions
 */

static cmr_canHVCState_t getNextState(cmr_canHVCError_t currentError){

    //Default to unknown state if no paths are satisfied.
    cmr_canHVCState_t nextState = CMR_CAN_HVC_STATE_UNKNOWN;
    static TickType_t lastPrechargeTime = 0;// = xTaskGetTickCount();


    // initialize min/max cell voltage variables for next state logic
    uint16_t packMinCellVoltage;
    uint16_t packMaxCellVoltage;

    if (currentError != CMR_CAN_HVC_ERROR_NONE) {
        // An error condition is active, stay in ERROR state
        return CMR_CAN_HVC_STATE_ERROR;
    }

    // Getting HVC Command
    volatile cmr_canHVCCommand_t *HVCCommand = getPayload(CANRX_HVC_COMMAND);

    //Getting BMB Min Max voltage data
    volatile cmr_canBMSMinMaxCellVoltage_t *HVBMSMinMaxVolt = getPayload(CANRX_HVBMS_MINMAX_VOLTAGE); 

    //Getting BMB pack voltage 
    volatile cmr_canHVBMSPackVoltage_t *HVBMSPackVoltage = getPayload(CANRX_HVBMS_PACKVOLT); 

    switch (currentState) {
        case CMR_CAN_HVC_STATE_DISCHARGE: // S1
        	// TODO: WAIT FOR HV VOLTAGE TO GO DOWN
            if(cellBalancing){
                stopCellBalancing(); 
                cellBalancing = false; 
            }
            nextState = CMR_CAN_HVC_STATE_STANDBY;
            break;
        case CMR_CAN_HVC_STATE_STANDBY: // S23
            if(cellBalancing){
                stopCellBalancing(); 
                cellBalancing = false; 
            }
            if (HVCCommand->modeRequest == CMR_CAN_HVC_MODE_START) {
                //T1: START mode requested
            		nextState = CMR_CAN_HVC_STATE_DRIVE_PRECHARGE;
            		lastPrechargeTime = xTaskGetTickCount();
            } else if (HVCCommand->modeRequest == CMR_CAN_HVC_MODE_CHARGE) {
                //T9: CHARGE mode requested
                    nextState = CMR_CAN_HVC_STATE_CHARGE_PRECHARGE;
                    lastPrechargeTime = xTaskGetTickCount();
            } else {
                nextState = CMR_CAN_HVC_STATE_STANDBY; 
            }
            break;
        case CMR_CAN_HVC_STATE_DRIVE_PRECHARGE: // S3
            if (!(HVCCommand->modeRequest == CMR_CAN_HVC_MODE_START ||
                  HVCCommand->modeRequest == CMR_CAN_HVC_MODE_RUN)) {
                //T6: Mode requested is neither START nor RUN
                nextState = CMR_CAN_HVC_STATE_DISCHARGE;
            } else if (abs((HVBMSPackVoltage->battVoltage_mV) - (getHVmillivolts()) < 30000)) {
                //T2: HV rails are precharged to within 30000mV
                nextState = CMR_CAN_HVC_STATE_DRIVE_PRECHARGE_COMPLETE;
                lastPrechargeTime = xTaskGetTickCount(); 
            } else {
                nextState = CMR_CAN_HVC_STATE_DRIVE_PRECHARGE;
            }
            break;
        case CMR_CAN_HVC_STATE_DRIVE_PRECHARGE_COMPLETE: {// S4
            if (!(HVCCommand->modeRequest == CMR_CAN_HVC_MODE_START ||
                  HVCCommand->modeRequest == CMR_CAN_HVC_MODE_RUN)) {
                //T7: Mode requested is neither START nor RUN
                nextState = CMR_CAN_HVC_STATE_DISCHARGE;
            } else if (HVCCommand->modeRequest == CMR_CAN_HVC_MODE_RUN) {
//                        abs(getBattMillivolts() - getHVmillivolts()) < 30000) {
                // T3: Contactors are closed and RUN mode is requested
                nextState = CMR_CAN_HVC_STATE_DRIVE; 
            } else {
                nextState = CMR_CAN_HVC_STATE_DRIVE_PRECHARGE_COMPLETE;
            }
            break;
        }
        case CMR_CAN_HVC_STATE_DRIVE: // S5
            if (HVCCommand->modeRequest != CMR_CAN_HVC_MODE_RUN) {
                // T8: Mode requested is not RUN
                nextState = CMR_CAN_HVC_STATE_DISCHARGE;
            } else {
                nextState = CMR_CAN_HVC_STATE_DRIVE;
            }
            break;
        case CMR_CAN_HVC_STATE_CHARGE_PRECHARGE: // S6
            if (HVCCommand->modeRequest != CMR_CAN_HVC_MODE_CHARGE) {
                //T18: Mode requested is not CHARGE
                nextState = CMR_CAN_HVC_STATE_DISCHARGE; 
            } else if (abs((HVBMSPackVoltage->battVoltage_mV) - ((uint32_t)getHVmillivolts()) < 30000)) { 
            	lastPrechargeTime = xTaskGetTickCount();
                //T10: HV rails are precharged
                nextState = CMR_CAN_HVC_STATE_CHARGE_PRECHARGE_COMPLETE; 
                //should only enable once even without wrapping 
                if(!cellBalancing){
                    enableCellBalancing();
                    cellBalancing = true; 
                } 
            } else {
                nextState = CMR_CAN_HVC_STATE_CHARGE_PRECHARGE; 
                cellBalancing = false; 
            }
            break;
        case CMR_CAN_HVC_STATE_CHARGE_PRECHARGE_COMPLETE: {// S7
            if (HVCCommand->modeRequest != CMR_CAN_HVC_MODE_CHARGE) {
                // T17: Mode requested is not CHARGE
                nextState = CMR_CAN_HVC_STATE_DISCHARGE; 
            } else if (true || abs(HVBMSPackVoltage->battVoltage_mV - getHVmillivolts()) < 5000) {
                // T11: Contactors are closed
                nextState = CMR_CAN_HVC_STATE_CHARGE_TRICKLE;
            } else {
                nextState = CMR_CAN_HVC_STATE_CHARGE_PRECHARGE_COMPLETE;
            }
            break;
        }
        case CMR_CAN_HVC_STATE_CHARGE_TRICKLE: // S8
            // find lowest cell voltage among all BMBs
            packMinCellVoltage = HVBMSMinMaxVolt->minCellVoltage_mV; 

            if (HVCCommand->modeRequest != CMR_CAN_HVC_MODE_CHARGE) {
                // T16: Mode requested is not CHARGE
                nextState = CMR_CAN_HVC_STATE_DISCHARGE;
            } else if (packMinCellVoltage >= 3000) {
                // T12: Minimum cell voltage > 3V, begin full charging
                nextState = CMR_CAN_HVC_STATE_CHARGE_CONSTANT_CURRENT;
            } else {
                nextState = CMR_CAN_HVC_STATE_CHARGE_TRICKLE;
            }
            break;
        case CMR_CAN_HVC_STATE_CHARGE_CONSTANT_CURRENT: // S9
            // find highest cell voltage among all BMBs
            packMaxCellVoltage = HVBMSMinMaxVolt->maxCellVoltage_mV; 

            if (HVCCommand->modeRequest != CMR_CAN_HVC_MODE_CHARGE) {
                // T15: Mode requested is not CHARGE
                nextState = CMR_CAN_HVC_STATE_DISCHARGE;
            } else if (packMaxCellVoltage >= 4280) {
                // T13: Maximum cell voltage > 4.15V, reached max cell voltage cannot continue charging 
                nextState = CMR_CAN_HVC_STATE_DISCHARGE; //discharge 
            } else {
                nextState = CMR_CAN_HVC_STATE_CHARGE_CONSTANT_CURRENT;
            }
            break;
        case CMR_CAN_HVC_STATE_CHARGE_CONSTANT_VOLTAGE: // S10
            // find lowest cell voltage among all BMBs
            packMinCellVoltage = HVBMSMinMaxVolt->minCellVoltage_mV; 

            if (HVCCommand->modeRequest != CMR_CAN_HVC_MODE_CHARGE || packMinCellVoltage >= 4145) {
                //T14: Mode requested is not CHARGE or all cells fully charged
                nextState = CMR_CAN_HVC_STATE_DISCHARGE;
            } else {
                nextState = CMR_CAN_HVC_STATE_CHARGE_CONSTANT_VOLTAGE;
            }
            break;
        case CMR_CAN_HVC_STATE_ERROR: { // S0
            if(cellBalancing){
                stopCellBalancing(); 
                cellBalancing = false; 
            }
            if (HVCCommand->modeRequest == CMR_CAN_HVC_MODE_ERROR) {
                //T19: GLV acknowledged error, move to clear error
                nextState = CMR_CAN_HVC_STATE_CLEAR_ERROR;
            } else {
                nextState = CMR_CAN_HVC_STATE_ERROR;
            }
            break;
        }
        case CMR_CAN_HVC_STATE_CLEAR_ERROR: // S11
            if ((HVCCommand->modeRequest == CMR_CAN_HVC_MODE_IDLE) || getHVmillivolts() < 5000) {
                //T4: GLV requesting idle and rails discharged
                nextState = CMR_CAN_HVC_STATE_STANDBY;
            } else {
                nextState = CMR_CAN_HVC_STATE_CLEAR_ERROR;
            }
            break;
        case CMR_CAN_HVC_STATE_UNKNOWN:
        default:
            nextState = CMR_CAN_HVC_STATE_UNKNOWN;
            break;
    }

    // Return the result of next state logic
    return nextState;
}

static void clearHardwareFault(bool assertClear) {
    // Set GPIO pin low (asserted) if
    // assertClear, high (deasserted) otherwise
    if (assertClear) {
        cmr_gpioWrite(GPIO_CLEAR_FAULT_L, 0);
    } else {
        cmr_gpioWrite(GPIO_CLEAR_FAULT_L, 1);
    }
}

static cmr_canHVCState_t setStateOutput(){

    //Note: For relay action, set all opens before closes to avoid shorts
    switch (currentState) {
        case CMR_CAN_HVC_STATE_DISCHARGE: // S1
            setRelay(AIR_POS_RELAY, OPEN);
            setRelay(AIR_NEG_RELAY, OPEN);
            setRelay(PRECHARGE_RELAY, OPEN);
            setRelay(DISCHARGE_RELAY, CLOSED);
            clearHardwareFault(false);
            break;
        case CMR_CAN_HVC_STATE_STANDBY: // S2
            setRelay(AIR_POS_RELAY, OPEN);
            setRelay(AIR_NEG_RELAY, OPEN);
            setRelay(PRECHARGE_RELAY, OPEN);
            setRelay(DISCHARGE_RELAY, CLOSED);
            clearHardwareFault(false);
            break;
        case CMR_CAN_HVC_STATE_DRIVE_PRECHARGE: // S3
            setRelay(DISCHARGE_RELAY, OPEN);
            setRelay(AIR_POS_RELAY, OPEN);
            setRelay(AIR_NEG_RELAY, CLOSED);
            setRelay(PRECHARGE_RELAY, CLOSED);
            clearHardwareFault(false);
            break;
        case CMR_CAN_HVC_STATE_DRIVE_PRECHARGE_COMPLETE: // S4
            setRelay(DISCHARGE_RELAY, OPEN);
            setRelay(AIR_POS_RELAY, CLOSED);
            setRelay(AIR_NEG_RELAY, CLOSED);
            setRelay(PRECHARGE_RELAY, CLOSED);
            clearHardwareFault(false);
            break;
        case CMR_CAN_HVC_STATE_DRIVE: // S5
            setRelay(DISCHARGE_RELAY, OPEN);
            setRelay(PRECHARGE_RELAY, OPEN);
            setRelay(AIR_POS_RELAY, CLOSED);
            setRelay(AIR_NEG_RELAY, CLOSED);
            clearHardwareFault(false);
            break;
        case CMR_CAN_HVC_STATE_CHARGE_PRECHARGE: // S6
            setRelay(DISCHARGE_RELAY, OPEN);
            setRelay(AIR_POS_RELAY, OPEN);
            setRelay(AIR_NEG_RELAY, CLOSED);
            setRelay(PRECHARGE_RELAY, CLOSED);
            clearHardwareFault(false);
            break;
        case CMR_CAN_HVC_STATE_CHARGE_PRECHARGE_COMPLETE: // S7
            setRelay(DISCHARGE_RELAY, OPEN);
            setRelay(AIR_POS_RELAY, CLOSED);
            setRelay(AIR_NEG_RELAY, CLOSED);
            setRelay(PRECHARGE_RELAY, CLOSED);
            clearHardwareFault(false);
            break;
        case CMR_CAN_HVC_STATE_CHARGE_TRICKLE: // S8
            setRelay(DISCHARGE_RELAY, OPEN);
            setRelay(PRECHARGE_RELAY, OPEN);
            setRelay(AIR_POS_RELAY, CLOSED);
            setRelay(AIR_NEG_RELAY, CLOSED);
            clearHardwareFault(false);
            break;
        case CMR_CAN_HVC_STATE_CHARGE_CONSTANT_CURRENT: // S9
            setRelay(DISCHARGE_RELAY, OPEN);
            setRelay(PRECHARGE_RELAY, OPEN);
            setRelay(AIR_POS_RELAY, CLOSED);
            setRelay(AIR_NEG_RELAY, CLOSED);
            clearHardwareFault(false);
            break;
        case CMR_CAN_HVC_STATE_CHARGE_CONSTANT_VOLTAGE: // S10
            setRelay(DISCHARGE_RELAY, OPEN);
            setRelay(PRECHARGE_RELAY, OPEN);
            setRelay(AIR_POS_RELAY, CLOSED);
            setRelay(AIR_NEG_RELAY, CLOSED);
            clearHardwareFault(false);
            break;
        case CMR_CAN_HVC_STATE_ERROR: // S0
            // Note, relays will not powered, as
            // the fault cuts their supply
            setRelay(AIR_POS_RELAY, OPEN);
            setRelay(AIR_NEG_RELAY, OPEN);
            setRelay(PRECHARGE_RELAY, OPEN);
            setRelay(DISCHARGE_RELAY, CLOSED);
            clearHardwareFault(false);
            // Its possible that in some error cases we might want to
            // open the contactors without cutting their power.
            // For now, take the more definitive approach
            break;
        case CMR_CAN_HVC_STATE_CLEAR_ERROR: // S11
            setRelay(AIR_POS_RELAY, OPEN);
            setRelay(AIR_NEG_RELAY, OPEN);
            setRelay(PRECHARGE_RELAY, OPEN);
            setRelay(DISCHARGE_RELAY, CLOSED);
            clearHVCErrorReg();
            clearHardwareFault(true);
            break;
        case CMR_CAN_HVC_STATE_UNKNOWN:
        default:
            setRelay(AIR_POS_RELAY, OPEN);
            setRelay(AIR_NEG_RELAY, OPEN);
            setRelay(PRECHARGE_RELAY, OPEN);
            setRelay(DISCHARGE_RELAY, CLOSED);
            clearHardwareFault(false);
            break;
    }

    return currentState;
}

/*
 * Task function
 */

void vSetStateTask(void *pvParameters) {
    // Make compiler happy
    (void) pvParameters;

    // Previous wake time pointer, initialized to current tick count.
    // This gets updated by vTaskDelayUntil every time it is called
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Period
    const TickType_t xPeriod = 10;        // In ticks (ms)

    cmr_canHVCState_t nextState;
    cmr_canHVCError_t currentError = CMR_CAN_HVC_ERROR_NONE;

    cmr_gpioWrite(GPIO_CLEAR_FAULT_L, 0);
    cmr_gpioWrite(GPIO_BMB_FAULT_L, 1);
    cmr_gpioWrite(GPIO_AIR_POSITIVE_EN, 1);
    cmr_gpioWrite(GPIO_AIR_NEGATIVE_EN, 1);

    // See https://drive.google.com/file/d/1xey3It43X-4tRBvnWUMSgUpSeBYo-Vj6/view?usp=sharing
    // Executes infinitely with defined period using vTaskDelayUntil
    for (;;) {

        // Ask Deepak ab getting rid of this
        //Critical block so that the contents of the heartbeat are consistent
        //taskENTER_CRITICAL();
        setStateOutput();
        // HVCHeartbeat->errorStatus = __REVSH(currentError);
        // HVCHeartbeat->state = currentState;
        // HVCHeartbeat->contactorStatus = getRelayStatus();
        //taskEXIT_CRITICAL();


        currentError = checkHVCErrors(currentState);
        nextState = getNextState(currentError);

        currentState = nextState;

        // Delay until next period
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

/** @brief CAN 100 Hz TX period (milliseconds). */
static const TickType_t canTX100Hz_period_ms = 10;

void enableCellBalancing(void) {
    cmr_canBMSMinMaxCellVoltage_t *voltagedata = getPayload(CANRX_HVBMS_MINMAX_VOLTAGE); 

    cmr_canHVCBalanceCommand_t balance = {
        .balanceRequest = true, 
        .threshold = voltagedata->minCellVoltage_mV, //placeholder 
    }; 

    canTX(CMR_CANID_CELL_BALANCE_ENABLE, &balance, sizeof(balance), canTX100Hz_period_ms); 
} 

void stopCellBalancing(void) {
    cmr_canHVCBalanceCommand_t balance = {
        .balanceRequest = false, 
        .threshold = 0, //placeholder 
    }; 

    canTX(CMR_CANID_CELL_BALANCE_ENABLE, &balance, sizeof(balance), canTX100Hz_period_ms); 
}

