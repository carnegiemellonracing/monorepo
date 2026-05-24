/**
 * @file remote_boot.c  
 * @brief Remote boot identifier information
 *
 * @author Carnegie Mellon Racing
 */

#include "CMR/remote_boot.h"
#include "shared_params.h"


void cmr_remoteBootInit(void){
    SharedParamsInit();
}

void cmr_requestRemoteFlash(void){
    SharedParamsWriteByIndex(BLT_REQUESTED_RESTART_IDX, 1);
}

bool cmr_requestedRemoteFlash(void){
    uint8_t value;
    SharedParamsReadByIndex(BLT_REQUESTED_RESTART_IDX, &value);
    return value == 1;
}

void cmr_resetRemoteFlash(void){
    SharedParamsWriteByIndex(BLT_REQUESTED_RESTART_IDX, 0);
}