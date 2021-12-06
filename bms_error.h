/*
 * bms_error.h
 *
 *  Created on: Jun 14, 2020
 *      Author: vamsi
 */

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

// AC Thresholds
static const int32_t maxPackVoltageMV = 600000;
static const int32_t minPackVoltageMV = 360000;
static const int32_t maxPackCurrentInstantMA = 400000;
static const int32_t maxPackCurrentAverageMA = 110000;
static const uint32_t minShutdownCiruitVoltageMV = 14000;

cmr_canHVCError_t checkErrors(cmr_canHVCState_t currentState);
void clearErrorReg(void);

// Error output signals
void clearHardwareFault(bool assertClear);

void setErrorReg(cmr_canHVCError_t errorCode);
cmr_canHVCError_t getErrorReg(void);

#endif /* BMS_ERROR_H_ */
