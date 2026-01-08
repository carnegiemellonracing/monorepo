/**
 * @file ivt.c
 * @brief Board-specific CAN implementation.
 *
 * Adding a new periodic message struct:
 *
 * 1. Add the corresponding index to the `canRX_t` enum in `can.h`.
 * 2. Add a configuration entry in `canRXMeta` at that index.
 * 3. Access the message using `canRXMeta[index]`.
 *
 * @author Carnegie Mellon Racing
 */

#include <string.h>     // memcpy()
#include <stdint.h> 

#include <CMR/tasks.h>  // Task interface

#include "ivt.h"        // Interface to implement
#include "sensors.h"    // sensorChannel_t
#include "can.h" 

/**
 * @brief CAN periodic message receive metadata
 *
 * @note Indexed by `canRX_t`.
 */
cmr_canRXMeta_t canRXMeta[] = {
    [CANRX_HEARTBEAT_HVC] = {
        .canID = CMR_CANID_HEARTBEAT_HVC,
        .timeoutError_ms = 20000,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .timeoutWarn_ms = 750,
        .warnFlag = CMR_CAN_WARN_VSM_HVC_TIMEOUT
    },
    [CANRX_IVT_RESPONSE]
}

uint_64_t ivt_buildMessage(cmr_IVTCommand_t cmd, cmr_IVTMessageType_t msgt, uint_8_t payload) {
    return ((((cmd << 4) | msgt)) << 56) | payload; 
}

uint_32_t get_result(cmr_IVTMessageType_t msgt){

    //request canID
    uint_64_t request = ivt_buildMessage(CMR_CAN_IVT_GET_CAN_ID, msgt, 0);
    canTX(CMR_CAN_BUS_TRAC, 0x411, &request, sizeof(request), canTX10Hz_period_ms);

    //find response ID
    resp = canTractiveGetPayload(CANRX_IVT_RESPONSE)
    //dereference pointer to get actual msg
    data = canTractiveGetPayload(*resp)
    // remove first byte, remaining is desired can id
    uint_8_t data = data;
    uint_32_t result = (data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4];
    return result;
}

float get_voltage(cmr_IVTMessageType_t msgt){
    uint_32_t raw = get_result(msgt);
    return raw * 0.001;
}

float get_current(void){
    uint_32_t raw = get_result(CURRENT);
    return raw * 0.001;
}

float get_pwr(void){
    uint_32_t raw = get_result(POWER);
    return raw;
}