/**
 * @file ivt.h
 * @brief Shared CAN type definitions.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef IVT_H
#define IVT_H

#include <stdint.h>
#include <stdbool.h>

#include "sensors.h"    // sensorChannel_t
#include "can.h"

// structs and enums of IVT CAN

/** @brief enums. */
typedef enum {
    CMR_CAN_IVT_RESULTS = 0,
    CMR_CAN_IVT_SET_CAN_ID, 
    CMR_CAN_IVT_SET_CONFIG_RESULT, 
    CMR_CAN_IVT_SET_COMMANDS, 
    CMR_CAN_IVT_GET_ERROR_DATA, 
    CMR_CAN_IVT_GET_CAN_ID, 
    CMR_CAN_IVT_GET_CONFIG_RESULT, 
    CMR_CAN_IVT_GET_COMMANDS,
    CMR_CAN_IVT_RESPONSE_ERROR_DATA,
    CMR_CAN_IVT_RESPONSE_CAN_ID,
    CMR_CAN_IVT_RESPONSE_CONFIG_RESULT,
    CMR_CAN_IVT_RESPONSE_SET_GET_COMMANDS,
} cmr_IVTCommand_t;

/** @brief n values. */
typedef enum {
    CURRENT = 0, 
    VOLTAGE_1,
    VOLTAGE_2,
    VOLTAGE_3,
    TEMPERATURE, 
    POWER,
    CURRENT_COUNTER,
    ENERGY_COUNTER,
} cmr_IVTMessageType_t; 

typedef enum {
    CMR_CAN_SET_STOP = 0x00,
    CMR_CAN_SET_RUN = 0x01,
} cmr_setModeOperations_t;

typedef enum {
    CMR_CAN_STORE_COMMAND = 0x32,
} cmr_storeModeOperations_t;

//define whatever cycle time 
typedef enum {
    CMR_CAN_FAST_CYCLE = 0x0100,
    CMR_CAN_MED_CYCLE = 0x0200,
    CMR_CAN_SLOW_CYCLE = 0x0300,
} cmr_cycleTimes_t;

/**
 * @brief function declarations
 */

//init
void ivtInit(cmr_canRXMeta_t* voltage, cmr_canRXMeta_t* current, cmr_canRXMeta_t* power, uint32_t cyclerate); 
void initIVTConfig (cmr_IVTMessageType_t msgt, cmr_cycleTimes_t cycletime);

//interface 
uint64_t ivt_buildMessage(cmr_IVTCommand_t cmd, cmr_IVTMessageType_t msgt, void* payload, uint8_t payloadlength);
uint32_t get_result(cmr_IVTMessageType_t msgt);
float get_voltage(cmr_IVTMessageType_t msgt);
float get_current(void);
float get_pwr(void);

//specialized funcs
void change_canid(cmr_IVTMessageType_t msgt, uint16_t new_CAN_ID); 
void set_storing(cmr_IVTMessageType_t msgt);
void change_bit_rate(cmr_IVTMessageType_t msgt, uint32_t bitrate);
void change_cycle_and_little_endian(cmr_IVTMessageType_t msgt, uint32_t cycletime);

#endif /* IVT_H */