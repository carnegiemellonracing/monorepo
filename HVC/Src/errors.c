#include "errors.h"
#include <string.h> //memcpy

static bool checkHVCCommandTimeout();
bool getAMSError(); //forward declaration

static cmr_canHVCError_t errorRegister = CMR_CAN_HVC_ERROR_NONE;

cmr_canHVCError_t checkHVCErrors(cmr_canHVCState_t currentState){
    clearHVCErrorReg();
    cmr_canHVCError_t errorFlags = errorRegister;
    if(checkHVCCommandTimeout()) { 
        // TODO E1 check the timeout field of the command mes sage meta data
        errorFlags |= CMR_CAN_HVC_ERROR_CAN_TIMEOUT;
    } 
    // if(getHVmilliamps() > maxPackCurrentInstantMA) {
    //     // E8
    //     errorFlags |= CMR_CAN_HVC_ERROR_PACK_OVERCURRENT;
    // }
    if(checkRelayPowerFault() && (getState() != CMR_CAN_HVC_STATE_ERROR && getState() != CMR_CAN_HVC_STATE_CLEAR_ERROR)) {//(getRelayStatus() & 0xAA) != 0xAA) {
        // TODO look into the AIR_Fault_L signal, it might be necessary to confirm this is not active
        // before looking at relay status, otherwise we could be in dead lock trying to clear errors.
        errorFlags |= CMR_CAN_HVC_ERROR_RELAY; 
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
        (getSafetymillivolts() < 10000)) {
        // E11
        // If SC voltage is below 8v while we're trying to drive relays, throw an error.
        errorFlags |= CMR_CAN_HVC_ERROR_LV_UNDERVOLT;
    }
     
    if (!cmr_gpioRead(GPIO_IN_IMD_ERR_N)) {
        errorFlags |= CMR_CAN_HVC_LATCH_IMD;
    }
    if (!cmr_gpioRead(GPIO_IN_BSPD_ERR_N)) {
        errorFlags |= CMR_CAN_VSM_LATCH_BSPD;
    }

     if (getAMSError()) {
        errorFlags |= CMR_CAN_HVC_LATCH_AMS;
        cmr_gpioWrite(GPIO_OUT_AMS_ERR_N, 0);
    }
    else {
        cmr_gpioWrite(GPIO_OUT_AMS_ERR_N, 1);
    }

    errorRegister = errorFlags;
    
    return errorFlags;
}

void clearHVCErrorReg() {
    errorRegister = CMR_CAN_HVC_ERROR_NONE;
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


bool getAMSError(){
    return false;
    TickType_t now = xTaskGetTickCount();
    cmr_canHVCHeartbeat_t *hvcHeartbeat = getPayload(CANRX_HEARTBEAT_HVC);
    return (cmr_canRXMetaTimeoutError(&(canRXMeta[CANRX_HEARTBEAT_HVC]), now) != 0)
      || (cmr_canRXMetaTimeoutError(&(canRXMeta[CANRX_HEARTBEAT_HVBMS]), now) != 0)
      || (hvcHeartbeat->errorStatus & CMR_CAN_HVBMS_ERROR_PACK_OVERVOLT)
      || (hvcHeartbeat->errorStatus & CMR_CAN_HVBMS_ERROR_CELL_OVERVOLT);
} //TODO check if this code needs to be uncommented