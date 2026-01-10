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

 //TODO: WE NEED TO FIND WHAT PARAMETERS TO PASS IN INITCONFIG 
 //CHECK THE CHANGE CANID MESSSAGE FUNCTION
 //FUNCTION HEADERS WHATEVER THOSE ARE?

#include <string.h>     // memcpy()
#include <stdint.h> 

#include "ivt.h"        // Interface to implement
#include "sensors.h"    // sensorChannel_t
#include "can.h"

#define CMR_CAN_COMMAND_IVT 0x411
#define CMR_CAN_STORE_ID 0x30
#define CMR_CAN_RESTART_TO_BITRATE 0x3A 
#define CMR_CAN_IVT_SET_MODE 0x34
#define CMR_CAN_IVT_SET_CONFIG 2

//ivtData storage struct
typedef struct IVTData {
    cmr_canRXMeta_t* voltage;
    cmr_canRXMeta_t* current;
    cmr_canRXMeta_t* power; 
    uint16_t cyclerate; 
} IVTData_t; 

//initialize ivtData variable
static IVTData_t ivtData; 

void ivtinit(cmr_canRXMeta_t* voltage, cmr_canRXMeta_t* current, cmr_canRXMeta_t* power, uint16_t cyclerate){
    
    //define initial values
    ivtData.voltage = voltage;
    ivtData.current = current; 
    ivtData.power = power;
    ivtData.cyclerate = cyclerate;

    //need to pass in where specifically i'm writing (msgt) and cycle time
    initIVTConfig(); 

}
//initilization helper function

void initIVTConfig (cmr_IVTMessageType_t msgt, cmr_cycleTimes_t cycletime){
    //set to stop mode
    uint64_t request = ivt_buildMessage(CMR_CAN_IVT_SET_MODE, msgt, CMR_CAN_SET_STOP);
    canTX(CMR_CAN_BUS_TRAC, CMR_CAN_COMMAND_IVT, &request, sizeof(request), canTX10Hz_period_ms);

    //set little endian format
    change_little_endian (msgt);

    //set cycle time 
    change_cycle(msgt, cycletime);

    // put to non-volatile memory
    set_storing(msgt);
    
    //set to run mode
    request = ivt_buildMessage(CMR_CAN_IVT_SET_MODE, msgt, CMR_CAN_SET_RUN);
    canTX(CMR_CAN_BUS_TRAC, CMR_CAN_COMMAND_IVT, &request, sizeof(request), canTX10Hz_period_ms);

}

uint64_t ivt_buildMessage(cmr_IVTCommand_t cmd, cmr_IVTMessageType_t msgt, void* payload, uint8_t payloadlength) {
    uint64_t message;
    memcpy(&message, payload, payloadlength);
    message = (message >> (64-payloadlength));
    return ((((cmd << 4) | msgt)) << (payloadlength)) | message; 
}

// Function header needed (seems pedantic but is a requirement of our code base)

float get_voltage(cmr_IVTMessageType_t msgt){
    ivtRes_t* v = canTractiveGetPayload(CANRX_TRAC_IVT_VOLTAGE);
    uint32_t raw = v->res;
    return raw * 0.001;
}

float get_current(void){
    ivtRes_t* c = canTractiveGetPayload(CANRX_TRAC_IVT_CURRENT);
    uint32_t raw = c->res;
    return raw * 0.001;
}

float get_pwr(void){
    ivtRes_t* p = canTractiveGetPayload(CANRX_TRAC_IVT_POWER);
    uint32_t raw = p->res;

    return raw;
}

typedef struct ivtRes{
    uint8_t MUXID; //byte 0
    uint8_t IVT_MsgCount: 4; //byte 1 lower nibble
    uint8_t Result_state: 4; //byte 1 upper nibble
    int32_t res; //bytes 2-5 IVT <result name (power, curr, volt)>

} ivtRes_t;

void change_canid(cmr_IVTMessageType_t msgt, uint16_t new_CAN_ID){
    //set the CAN ID
    //require only 11 bits used 
    configASSERT(new_CAN_ID < 0x800);
    uint64_t request = ivt_buildMessage(CMR_CAN_IVT_SET_CAN_ID, msgt, &new_CAN_ID, sizeof(new_CAN_ID));
    canTX(CMR_CAN_BUS_TRAC, CMR_CAN_COMMAND_IVT, &request, sizeof(request), canTX10Hz_period_ms);

}

//store CAN IDs
void set_storing(cmr_IVTMessageType_t msgt){
    //set to stop mode
    uint64_t request = ivt_buildMessage(CMR_CAN_IVT_SET_MODE, msgt, CMR_CAN_SET_STOP);
    canTX(CMR_CAN_BUS_TRAC, CMR_CAN_COMMAND_IVT, &request, sizeof(request), canTX10Hz_period_ms);

    //store the data in CAN
    request = ivt_buildMessage(CMR_CAN_STORE_ID, msgt, CMR_CAN_STORE_COMMAND);
    canTX(CMR_CAN_BUS_TRAC, CMR_CAN_STORE_ID, &request, sizeof(request), canTX10Hz_period_ms);
    
    //set to run mode
    request = ivt_buildMessage(CMR_CAN_IVT_SET_MODE, msgt, CMR_CAN_SET_RUN);
    canTX(CMR_CAN_BUS_TRAC, CMR_CAN_COMMAND_IVT, &request, sizeof(request), canTX10Hz_period_ms);
}

//function to change bitrate 
void change_bit_rate(cmr_IVTMessageType_t msgt, cmr_bitRateValues_t bitrate){
    //set to stop mode
    uint64_t request = ivt_buildMessage(CMR_CAN_IVT_SET_MODE, msgt, CMR_CAN_SET_STOP);
    canTX(CMR_CAN_BUS_TRAC, CMR_CAN_COMMAND_IVT, &request, sizeof(request), canTX10Hz_period_ms);

    //change bit rate
    request = ivt_buildMessage(CMR_CAN_RESTART_TO_BITRATE, msgt, bitrate);
    canTX(CMR_CAN_BUS_TRAC, CMR_CAN_RESTART_TO_BITRATE, &request, sizeof(request), canTX10Hz_period_ms);

    //set to run mode
    request = ivt_buildMessage(CMR_CAN_IVT_SET_MODE, msgt, CMR_CAN_SET_RUN);
    canTX(CMR_CAN_BUS_TRAC, CMR_CAN_COMMAND_IVT, &request, sizeof(request), canTX10Hz_period_ms);
}   

void change_little_endian (cmr_IVTMessageType_t msgt){
    //set to stop mode
    uint64_t request = ivt_buildMessage(CMR_CAN_IVT_SET_MODE, msgt, CMR_CAN_SET_STOP);
    canTX(CMR_CAN_BUS_TRAC, CMR_CAN_COMMAND_IVT, &request, sizeof(request), canTX10Hz_period_ms);

    //modify little endian
    uint8_t data[8] = {0};
    data[1] |= (1<<6);

    //send CAN message 
    canTX(CMR_CAN_BUS_TRAC, CMR_CAN_COMMAND_IVT, data, sizeof(data), canTX10Hz_period_ms);

    //set to run mode
    request = ivt_buildMessage(CMR_CAN_IVT_SET_MODE, msgt, CMR_CAN_SET_RUN);
    canTX(CMR_CAN_BUS_TRAC, CMR_CAN_COMMAND_IVT, &request, sizeof(request), canTX10Hz_period_ms);

}

//function to change cycle times 
void change_cycle(cmr_IVTMessageType_t msgt, cmr_cycleTimes_t cycletime){
    //set to stop mode
    uint64_t request = ivt_buildMessage(CMR_CAN_IVT_SET_MODE, msgt, CMR_CAN_SET_STOP);
    canTX(CMR_CAN_BUS_TRAC, CMR_CAN_COMMAND_IVT, &request, sizeof(request), canTX10Hz_period_ms);

    // modify the cycle time 
    uint8_t data[8] = {0};
    data[0] = (uint8_t) cycletime & 0xFF; 
    data[1] = (uint8_t) (cycletime >> 8) & 0xFF;

    //send CAN message 
    canTX(CMR_CAN_BUS_TRAC, CMR_CAN_COMMAND_IVT, data, sizeof(data), canTX10Hz_period_ms);

    //set to run mode
    request = ivt_buildMessage(CMR_CAN_IVT_SET_MODE, msgt, CMR_CAN_SET_RUN);
    canTX(CMR_CAN_BUS_TRAC, CMR_CAN_COMMAND_IVT, &request, sizeof(request), canTX10Hz_period_ms);
} 

//random stuff

    // What does this do???
    // Explain 0x26
    /*
    uint64_t request = ivt_buildMessage(CMR_CAN_IVT_SET_CONFIG, 2, (0x26 << 8) | cycletime, 24);
    canTX(CMR_CAN_BUS_TRAC, CMR_CAN_COMMAND_IVT, &request, sizeof(request), canTX10Hz_period_ms);
 
    ivtData.cyclerate = cycletime; 
    */