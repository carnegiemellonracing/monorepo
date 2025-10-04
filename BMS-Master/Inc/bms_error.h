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

cmr_canHVCError_t checkBMSMErrors(cmr_canHVCState_t currentState);
void clearBMSMErrorReg(void);


void setBMSMErrorReg(cmr_canHVCError_t errorCode);
cmr_canHVCError_t getBMSMErrorReg(void);

// Heartbeat timeout	
#define HEARTBEAT_TIMEOUT	50		// Periods of 10ms

// BMB timeout
#define BMB_TIMEOUT         5

typedef enum {
	BMB_NO_ERR = 0,

	BMB_VOLTAGE_READ_ERROR,
	BMB_TEMP_READ_ERROR
} BMB_UART_ERRORS;

#endif /* BMS_ERROR_H_ */
