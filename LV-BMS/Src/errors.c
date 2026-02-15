#include "errors.h"


cmr_canLVBMSHeartbeat_t checkLVBMSError() {
    uint32_t LVBMillivolts = getLVBMillivolts(); 

    if(LVBMillivolts < LVBUndervoltageThreshold) {
        errorStatus |= CMR_CAN_ERROR_LV_UNDERVOLT;
    } else if (LVBMillivolts > LVBOvervoltageThreshold) {
        errorStatus |= CMR_CAN_ERROR_LV_OVERVOLT;
    }

}