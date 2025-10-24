/*
 * bms_error.c
 *
 *  Created on: Jun 14, 2020
 *      Author: vamsi
 */

#include "bms_error.h"

static bool checkBMBTimeout();

volatile int BMBTimeoutCount[BOARD_NUM] = { 0 };
volatile int BMBErrs[BOARD_NUM] = { 0 };

// Persistent value for storing the error type. Will be useful if
// error checking becomes its own task
static cmr_canHVCError_t errorRegister = CMR_CAN_HVC_STATE_ERROR;

cmr_canHVCError_t checkHVBMSErrors(cmr_canHVCState_t currentState){
    cmr_canHVCError_t errorFlags = CMR_CAN_HVC_ERROR_NONE; 
    if(checkBMBTimeout()) { //BMSM 
        // TODO E2 devise a UART monitor system
        errorFlags |= CMR_CAN_HVC_ERROR_BMB_TIMEOUT; /**< @brief BMB has timed out. */
    }
    if(getPackMaxCellTemp() > 590) { // Temp limit of 59C //BMSM 
//        // TODO: #Define with 590
//        // TODO E3 create structures for cell temp data and stats (min/max)
       errorFlags |= CMR_CAN_HVC_ERROR_CELL_OVERTEMP; 
    }
    if(getPackMaxCellVoltage() > 4280) { // Cell voltage limit of 4280 //BMSM 
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
//     if(getCurrentInstant() > maxPackCurrentInstantMA) {
//         // E8
//         errorFlags |= CMR_CAN_HVC_ERROR_PACK_OVERCURRENT;
//     }
//    if(getCurrentAverage() > maxPackCurrentAverageMA) {
//        // E9
//        errorFlags |= CMR_CAN_HVC_ERROR_PACK_OVERCURRENT;
//    }
    errorRegister = errorFlags;
    
    return errorFlags;
}


void clearHVBMSErrorReg() {
    errorRegister = CMR_CAN_HVC_ERROR_NONE;
}

void setHVBMSErrorReg(cmr_canHVCError_t errorCode){
    errorRegister = errorCode;
}

cmr_canHVCError_t getHVBMSErrorReg(){
    return errorRegister;
}


static bool checkBMBTimeout() {
    for (int i = 0; i < BOARD_NUM; i++) {
        if (BMBTimeoutCount[i] >= BMB_TIMEOUT) {
            return true;
        }
    }
    return false;
}
