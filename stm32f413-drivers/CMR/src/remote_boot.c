/**
 * @file remote_boot.c  
 * @brief Remote boot identifier information
 *
 * @author Carnegie Mellon Racing
 */

#include "shared_params.h"
#include "CMR/remote_boot.h"
#include "CMR/can_ids.h"
#include "CMR/board_info.h"


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


void cmr_checkMsgForRemoteFlash(uint32_t canID, uint8_t data[8]) {
    if(canID == CMR_CANID_BOOTLOADER_FLASH_READY && data[0] == cmr_getBoardId()) {
        cmr_requestRemoteFlash();
        NVIC_SystemReset();
    }
}