/*
 * bms_error.c
 *
 *  Created on: Jun 14, 2020
 *      Author: vamsi
 */

#include "bms_error.h"

static bool checkCommandTimeout();

// Persistent value for storing the error type. Will be useful if
// error checking becomes its own task
static cmr_canHVCError_t errorRegister = CMR_CAN_HVC_STATE_ERROR;

cmr_canHVCError_t checkErrors(cmr_canHVCState_t currentState){
    cmr_canHVCError_t errorFlags = CMR_CAN_HVC_ERROR_NONE;
    if(checkCommandTimeout()) {
        // TODO E1 check the timeout field of the command message meta data
        errorFlags |= CMR_CAN_HVC_ERROR_CAN_TIMEOUT;
    }
    if(false) {
        // TODO E2 devise a UART monitor system
        errorFlags |= CMR_CAN_HVC_ERROR_BMB_TIMEOUT; /**< @brief BMB has timed out. */
;
    }
    if(getPackMaxCellTemp() > 590) { // Temp limit of 59C
        // TODO E3 create structures for cell temp data and stats (min/max)
        errorFlags |= CMR_CAN_HVC_ERROR_CELL_OVERTEMP;
    }
    if(getPackMaxCellVoltage() > 4175) { // Cell voltage limit of 4175
        // TODO E4 create structures for cell voltage data and stats (min/max)
        errorFlags |= CMR_CAN_HVC_ERROR_CELL_OVERVOLT;
    }
    if(getPackMinCellVoltage() < 2650) {
        // TODO E5 create structures for cell voltage data and stats (min/max)
        errorFlags |= CMR_CAN_HVC_ERROR_CELL_UNDERVOLT;
    }
    if((((int32_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_BATT_PLUS)) - ((int32_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_BATT_MINUS))) > maxPackVoltageMV) {
        // E6
        errorFlags |= CMR_CAN_HVC_ERROR_PACK_OVERVOLT;
    }
    if((((int32_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_BATT_PLUS)) - ((int32_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_BATT_MINUS))) < minPackVoltageMV) {
        // E7
        errorFlags |= CMR_CAN_HVC_ERROR_PACK_UNDERVOLT;
    }
    if(((int32_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_SHUNT_LV)) > maxPackCurrentInstantMA) {
        // E8
        errorFlags |= CMR_CAN_HVC_ERROR_PACK_OVERCURRENT;
    }
    if(false){//getCurrentAverage() > maxPackCurrentAverageMA) {
        // E9 Average is not reliable, takes to long to compute
        errorFlags |= CMR_CAN_HVC_ERROR_PACK_OVERCURRENT;
    }
    if(checkRelayPowerFault() && (getState() != CMR_CAN_HVC_STATE_ERROR && getState() != CMR_CAN_HVC_STATE_CLEAR_ERROR)) {//(getRelayStatus() & 0xAA) != 0xAA) {
        // TODO look into the AIR_Fault_L signal, it might be necessary to confirm this is not active
        // before looking at relay status, otherwise we could be in dead lock trying to clear errors.
        //errorFlags |= BMS_ERROR_CODE_RELAY;
    }

    if((currentState == CMR_CAN_HVC_STATE_DRIVE_PRECHARGE ||
        currentState == CMR_CAN_HVC_STATE_DRIVE_PRECHARGE_COMPLETE ||
        currentState == CMR_CAN_HVC_STATE_DRIVE ||
        currentState == CMR_CAN_HVC_STATE_CHARGE_PRECHARGE ||
        currentState == CMR_CAN_HVC_STATE_CHARGE_PRECHARGE_COMPLETE ||
        currentState == CMR_CAN_HVC_STATE_CHARGE_TRICKLE ||
        currentState == CMR_CAN_HVC_STATE_CHARGE_CONSTANT_CURRENT ||
        currentState == CMR_CAN_HVC_STATE_CHARGE_CONSTANT_VOLTAGE) &&
       (((int32_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_AIR_POWER)) < minShutdownCiruitVoltageMV)) {
        // E11
        // If SC voltage is below 8v while we're trying to drive relays, throw an error.
        errorFlags |= CMR_CAN_HVC_ERROR_LV_UNDERVOLT;
    }

    if(cmr_gpioRead(GPIO_BMB_FAULT_L_PIN) == false) {
        // E2
        // BMB fault pin is asserted (low), could indicate BMB disconnected
        // or hardware UV/OV condition.
        errorFlags |= CMR_CAN_HVC_ERROR_BMB_FAULT;
    }

    // Cut relay power if we have an error
    if (errorFlags != CMR_CAN_HVC_ERROR_NONE) {
        setOvercurrentFault(true);
    }
    errorRegister = errorFlags;
    
    return errorFlags;
}

void clearHardwareFault(bool assertClear) {
    // Set GPIO pin low (asserted) if
    // assertClear, high (deasserted) otherwise
    if (assertClear) {
        cmr_gpioWrite(GPIO_CLEAR_ERROR_PIN, 0);
    } else {
        cmr_gpioWrite(GPIO_CLEAR_ERROR_PIN, 1);
    }
}

void setOvercurrentFault(bool assertFault) {
    // Set GPIO pin high (asserted) if
    // assertClear, low (deasserted) otherwise
    if (assertFault) {
        //cmr_gpioWrite(GPIO_OVERCURRENT_FAULT_PIN, 1);

    } else {
        //cmr_gpioWrite(GPIO_OVERCURRENT_FAULT_PIN, 0);
    }
}


void clearErrorReg() {
    errorRegister = CMR_CAN_HVC_ERROR_NONE;
}

void setErrorReg(cmr_canHVCError_t errorCode){
    errorRegister = errorCode;
}

cmr_canHVCError_t getErrorReg(){
    return errorRegister;
}

/*

static bool checkCommandTimeout() {
    //This function must be run in a task with a 100Hz rate.

	bool inError = false;

	// Command Message Stale Check
	if(BMSCommandReceiveMeta->staleFlag && !BMSCommandReceiveMeta.timeoutFlag) {
		// Only increment miss count if stale and not timed out
		BMSCommandReceiveMeta.missCount++;
		} else if (!BMSCommandReceiveMeta.staleFlag) {
		// If not stale, reset miss count and set back to stale
		BMSCommandReceiveMeta.missCount = 0;
		BMSCommandReceiveMeta.staleFlag = 1;
	}
    // Command Message Timeout
	if(BMSCommandReceiveMeta.missCount > HEARTBEAT_TIMEOUT) {
		// Set timeout if above timeout threshold
		BMSCommandReceiveMeta.timeoutFlag = 1;
		} else {
		// Reset timeout flag if not timed out
		BMSCommandReceiveMeta.timeoutFlag = 0;
	}

    if (BMSCommandReceiveMeta.timeoutFlag) {
        inError = true;
    }

	return inError;
}

*/
