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

#define SERIAL_NUMBER 45 //change serial number of ?? 
#define CMR_CAN_COMMAND_IVT 0x411
#define CMR_CAN_STORE_ID = 0x30

/**
 * @brief CAN periodic message receive metadata
 *
 * @note Indexed by `canRX_t`.
 */
cmr_canRXMeta_t canRXMeta[] = {
    [CANRX_IVT_RESPONSE] = {
        .canID = CMR_CANID_IVT_RESPONSE
    }
}

void ivtinit(pass in pointers to v and i ){
    init (struct) file and config
}

uint64_t ivt_buildMessage(cmr_IVTCommand_t cmd, cmr_IVTMessageType_t msgt, uint8_t payload) {
    return ((((cmd << 4) | msgt)) << 56) | payload; 
}

uint32_t get_result(cmr_IVTMessageType_t msgt){

    //request canID
    uint64_t request = ivt_buildMessage(CMR_CAN_IVT_GET_CAN_ID, msgt, 0);
    canTX(CMR_CAN_BUS_TRAC, CMR_CAN_COMMAND_IVT, &request, sizeof(request), canTX10Hz_period_ms);

    //find response ID
    resp = canTractiveGetPayload(CANRX_IVT_RESPONSE)
    //dereference pointer to get actual msg
    data = canTractiveGetPayload(*resp)
    // remove first byte, remaining is desired can id
    uint8_t data = data;
    uint32_t result = (data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4];
    return result;
}

float get_voltage(cmr_IVTMessageType_t msgt){
    uint32_t raw = get_result(msgt);
    return raw * 0.001;
}

float get_current(void){
    uint32_t raw = get_result(CURRENT);
    return raw * 0.001;
}

float get_pwr(void){
    uint32_t raw = get_result(POWER);
    return raw;
}

void change_canid(cmr_IVTMessageType_t msgt){
    //set the CAN ID
    uint64_t request = ivt_buildMessage(CMR_CAN_IVT_SET_CAN_ID, msgt, 0);
    canTX(CMR_CAN_BUS_TRAC, CMR_CAN_COMMAND_IVT, &request, sizeof(request), canTX10Hz_period_ms);

}

//store CAN IDs
void set_storing(){
    //command set to stop mode
    uint8_t stopdata[8] = {0};
    stopdata[0] = SET_OPERATION_MODE;
    stopdata[1] = SET_STOP;
    canTX(CMR_CAN_BUS_TRAC, CMR_CAN_COMMAND_IVT, stopdata, sizeof(stopdata), canTX10Hz_period_ms);

    //store the data in CAN
    uint8_t storedata[8] = {0};
    storedata[0] = STORE_COMMAND; 
    canTX(CMR_CAN_BUS_TRAC, CMR_CAN_STORE_ID, storedata, sizeof(storedata), canTX10Hz_period_ms);
    
    //command set to run mode
    uint8_t rundata[8] = {0};
    rundata[0] = SET_OPERATION_MODE; 
    rundata[1] = SET_RUN;
    canTX(CMR_CAN_BUS_TRAC, CMR_CAN_COMMAND_IVT, rundata, sizeof(rundata), canTX10Hz_period_ms);
}

//function to change bitrate 
void change_bit_rate(cmr_IVTMessageType_t msgt){
    f
}

//function to change cycle times and to little endian 
void change_cycle_and_little_endian(cmr_IVTMessageType_t msgt){
    //change bit 6

    replace cycle time with 0xnnnn whatever number
    send to CMR_CAN_COMMAND_IVT
} 