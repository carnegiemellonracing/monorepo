/*
 * bms_error.c
 *
 *  Created on: Jun 14, 2020
 *      Author: vamsi
 */

#include "bms_error.h"
#include "slave_uart.h"

static bool checkCommandTimeout();
static bool checkBMBTimeout();

volatile int BMBTimeoutCount[NUM_BMBS] = { 0 };

// Persistent value for storing the error type. Will be useful if
// error checking becomes its own task
static cmr_canHVCError_t errorRegister = CMR_CAN_HVC_STATE_ERROR;

cmr_canHVCError_t checkErrors(cmr_canHVCState_t currentState){
    cmr_canHVCError_t errorFlags = CMR_CAN_HVC_ERROR_NONE;
    if(checkCommandTimeout()) {
        // TODO E1 check the timeout field of the command message meta data
        errorFlags |= CMR_CAN_HVC_ERROR_CAN_TIMEOUT;
    }
    if(checkBMBTimeout()) {
        // TODO E2 devise a UART monitor system
        errorFlags |= CMR_CAN_HVC_ERROR_BMB_TIMEOUT; /**< @brief BMB has timed out. */
    }
    if(getPackMaxCellTemp() > 590) { // Temp limit of 59C
        // TODO: #Define with 590
        // TODO E3 create structures for cell temp data and stats (min/max)
        errorFlags |= CMR_CAN_HVC_ERROR_CELL_OVERTEMP;
    }
    if(getPackMaxCellVoltage() > 4175) { // Cell voltage limit of 4175
        // TODO E4 create structures for cell voltage data and stats (min/max)
        errorFlags |= CMR_CAN_HVC_ERROR_CELL_OVERVOLT;
    }
    if(getPackMinCellVoltage() < 2400) {
        // TODO E5 create structures for cell voltage data and stats (min/max)
        errorFlags |= CMR_CAN_HVC_ERROR_CELL_UNDERVOLT;
    }
    if((getBattMillivolts()) > maxPackVoltageMV) {
        // E6
        errorFlags |= CMR_CAN_HVC_ERROR_PACK_OVERVOLT;
    }
    if((getBattMillivolts()) < minPackVoltageMV) {
        // E7
        errorFlags |= CMR_CAN_HVC_ERROR_PACK_UNDERVOLT;
    }
    if(false){//getCurrentInstant() > maxPackCurrentInstantMA) {
        // E8
        errorFlags |= CMR_CAN_HVC_ERROR_PACK_OVERCURRENT;
    }
//    if(getCurrentAverage() > maxPackCurrentAverageMA) {
//        // E9
//        errorFlags |= CMR_CAN_HVC_ERROR_PACK_OVERCURRENT;
//    }
    if(checkRelayPowerFault() && (getState() != CMR_CAN_HVC_STATE_ERROR && getState() != CMR_CAN_HVC_STATE_CLEAR_ERROR)) {//(getRelayStatus() & 0xAA) != 0xAA) {
        // TODO look into the AIR_Fault_L signal, it might be necessary to confirm this is not active
        // before looking at relay status, otherwise we could be in dead lock trying to clear errors.
        //errorFlags |= BMS_ERROR_CODE_RELAY;
    }

    if(
    	(currentState == CMR_CAN_HVC_STATE_DRIVE_PRECHARGE ||
        currentState == CMR_CAN_HVC_STATE_DRIVE_PRECHARGE_COMPLETE ||
        currentState == CMR_CAN_HVC_STATE_DRIVE ||
        currentState == CMR_CAN_HVC_STATE_CHARGE_PRECHARGE ||
        currentState == CMR_CAN_HVC_STATE_CHARGE_PRECHARGE_COMPLETE ||
        currentState == CMR_CAN_HVC_STATE_CHARGE_TRICKLE ||
        currentState == CMR_CAN_HVC_STATE_CHARGE_CONSTANT_CURRENT ||
        currentState == CMR_CAN_HVC_STATE_CHARGE_CONSTANT_VOLTAGE) &&
        (cmr_gpioRead(SAFETY_BINARY) == 0)) {
        // E11
        // If SC voltage is below 8v while we're trying to drive relays, throw an error.
        errorFlags |= CMR_CAN_HVC_ERROR_LV_UNDERVOLT;
    }

    if(cmr_gpioRead(GPIO_BMB_FAULT_L) == false) {
        // E2
        // BMB fault pin is asserted (low), could indicate BMB disconnected
        // or hardware UV/OV condition.
        errorFlags |= CMR_CAN_HVC_ERROR_BMB_FAULT;
    }

    // Cut relay power if we have an error
    if (errorFlags != CMR_CAN_HVC_ERROR_NONE) {
        // TODO - Now Set in HW is this needed??
    }
    errorRegister = errorFlags;
    
    return errorFlags;
}

void clearHardwareFault(bool assertClear) {
    // Set GPIO pin low (asserted) if
    // assertClear, high (deasserted) otherwise
    if (assertClear) {
        cmr_gpioWrite(GPIO_CLEAR_FAULT_L, 0);
    } else {
        cmr_gpioWrite(GPIO_CLEAR_FAULT_L, 1);
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

static bool checkCommandTimeout() {
    // CAN error if HVC Command has timed out after 50ms
    // TODO: latch can error?
    TickType_t lastWakeTime = xTaskGetTickCount();
    bool hvc_commmand_error = (cmr_canRXMetaTimeoutError(&(canRXMeta[CANRX_HVC_COMMAND]), lastWakeTime) < 0);

	return hvc_commmand_error;
}

static bool checkBMBTimeout() {
    for (int i = 0; i < NUM_BMBS; i++) {
        if (BMBTimeoutCount[i] >= BMB_TIMEOUT) {
            return true;
        }
    }
    return false;
}
