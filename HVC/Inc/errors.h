#ifndef BMS_ERROR_H_
#define BMS_ERROR_H_

#include <stdbool.h>
#include <CMR/can_types.h>
#include "adc.h"
#include "sensors.h"
#include "bms_relay.h"
#include "state_task.h"
#include "can.h"

// Receive mailbox metadata struct
typedef struct ReceiveMeta_t {
    uint16_t missCount;
    uint8_t timeoutFlag;
    uint8_t staleFlag;
    uint8_t differentStateCount;
    uint8_t wrongStateFlag;
} ReceiveMeta_t;


cmr_canHVCError_t checkErrors(cmr_canHVCState_t currentState);


// Heartbeat timeout	
#define HEARTBEAT_TIMEOUT	50		// Periods of 10ms
