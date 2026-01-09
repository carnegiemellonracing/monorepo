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

// structs and enums of IVT CAN

typedef struct IVTData {
    uint16_t voltage;
    uint16_t current;
} IVTData_t; 

typedef struct IVTConfig {
    uint8_t mode;
    uint16_t cyclerate;
    float voltage;
    float current;
    float power;
} IVTConfig_t;


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
    CMRP_CAN_SET_RUN = 0x01,
} cmr_setModeOperations_t;

typedef enum {
    CMR_CAN_STORE_COMMAND = 0x32,
} cmr_storeModeOperations_t;

typedef enum {
    CMR_CAN_BITRATE_250K = 0x08,
    CMR_CAN_BITRATE_500K = 0x04,
    CMR_CAN_BITRATE_1000K = 0x02,
} cmr_bitRateValues_t; 

//define whatever cycle time 
typedef enum {
    CMR_CAN_FAST_CYCLE = 0x0100,
    CMR_CAN_MED_CYCLE = 0x0200,
    CMR_CAN_SLOW_CYCLE = 0x0300,
} cmr_cycleTimes_t;

//interface 
uint_64_t ivt_buildMessage(cmr_IVTCommand_t cmd, cmr_IVTMessageType_t msgt, uint_8_t payload);
uint_32_t get_result(cmr_IVTMessageType_t msgt)
float get_voltage(cmr_IVTMessageType_t msgt)

void change_canid(cmr_IVTMessageType_t msgt)
#endif /* IVT_H */