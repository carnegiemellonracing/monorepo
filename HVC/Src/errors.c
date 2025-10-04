#include "errors.h"


static bool checkHVCCommandTimeout();

static cmr_canHVCError_t errorRegister = CMR_CAN_HVC_STATE_ERROR;

cmr_canHVCError_t checkHVCErrors(cmr_canHVCState_t currentState){
    cmr_canHVCError_t errorFlags = CMR_CAN_HVC_ERROR_NONE;
    if(checkHVCCommandTimeout()) { //HVC 
        // TODO E1 check the timeout field of the command mes sage meta data
        errorFlags |= CMR_CAN_HVC_ERROR_CAN_TIMEOUT;
    } 
//     if(getCurrentInstant() > maxPackCurrentInstantMA) {
//         // E8
//         errorFlags |= CMR_CAN_HVC_ERROR_PACK_OVERCURRENT;
//     }
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
        (getSafetymillivolts() < 14000)) {
        // E11
        // If SC voltage is below 8v while we're trying to drive relays, throw an error.
        errorFlags |= CMR_CAN_HVC_ERROR_LV_UNDERVOLT;
    }

    // Cut relay power if we have an error //HVC? 
    if (errorFlags != CMR_CAN_HVC_ERROR_NONE) {
        cmr_gpioWrite(GPIO_BMB_FAULT_L, 0); 
    } else {
    	cmr_gpioWrite(GPIO_BMB_FAULT_L, 1);
    }
    errorRegister = errorFlags;
    
    return errorFlags;
}

void clearHVCErrorReg() {
    errorRegister = CMR_CAN_HVC_ERROR_NONE;
}

void setHVCErrorReg(cmr_canHVCError_t errorCode){
    errorRegister = errorCode;
}

cmr_canHVCError_t getHVCErrorReg(){
    return errorRegister;
}


static bool checkHVCCommandTimeout() {
    // CAN error if HVC Command has timed out after 50ms
    // TODO: latch can error?
    TickType_t lastWakeTime = xTaskGetTickCount();
    bool hvc_commmand_error = (cmr_canRXMetaTimeoutError(&(canRXMeta[CANRX_HVC_COMMAND]), lastWakeTime) < 0);

	return hvc_commmand_error;
}