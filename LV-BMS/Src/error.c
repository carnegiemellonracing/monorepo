/**
 * @file error.c
 * @brief Implementation for errors
 *
 * @author Ayush Garg
 */

#include <string.h>         // memcpy

#include <CMR/can.h>        // CMR CAN interface

#include "analysis.h"       // Analysis interface
#include "can.h"            // LV-BMS CAN interface
#include "gpio.h"           // GPIO CAN interface


#define MAX_BATT_VOLTAGE_MV 30000
#define MAX_CELL_VOLTAGE_MV 4300
#define MIN_PACK_VOLTAGE_MV 15000
#define MIN_CELL_VOLTAGE_MV 2500

#define MAX_CELL_TEMP_CENTI_C 7500

// When this define statement is 1 we then toggle the UVLO relay upon error
#define HYBRID_MODE 1

cmr_canHeartbeat_t heartbeat;

 void update_errors_and_warnings(TickType_t lastWakeTime){
    uint16_t error = CMR_CAN_ERROR_NONE;
    uint16_t warning = CMR_CAN_ERROR_NONE;
    cmr_canRXMeta_t *heartbeatVSMMeta = (cmr_canRXMeta_t *) getRxMeta(CANRX_HEARTBEAT_VSM);

    if (cmr_canRXMetaTimeoutError(heartbeatVSMMeta, lastWakeTime) < 0) {
        error |= CMR_CAN_ERROR_VSM_TIMEOUT;
    }

    if (cmr_canRXMetaTimeoutWarn(heartbeatVSMMeta, lastWakeTime) < 0) {
        warning |= CMR_CAN_WARN_VSM_TIMEOUT;
    }

    if(BMS_stats.batt_voltage_mV >= MAX_BATT_VOLTAGE_MV){
        error |= CMR_CAN_ERROR_LV_BMS_PACK_OVER_VOLT;
    }

    if(BMS_stats.batt_voltage_mV <= MIN_PACK_VOLTAGE_MV){
        error |= CMR_CAN_ERROR_LV_BMS_PACK_UNDER_VOLT;
        if(HYBRID_MODE){
            cmr_gpioWrite(GPIO_BMS_ERROR, 1);
        }
    }

    if(BMS_stats.max_cell_voltage_mV >= MAX_CELL_VOLTAGE_MV){
        error |= CMR_CAN_ERROR_LV_BMS_CELL_OVER_VOLT;
    }

    if(BMS_stats.min_cell_voltage_mV <= MIN_CELL_VOLTAGE_MV){
        error |= CMR_CAN_ERROR_LV_BMS_CELL_UNDER_VOLT;
        if(HYBRID_MODE){
            cmr_gpioWrite(GPIO_BMS_ERROR, 1);
        }
    }

    if(BMS_stats.max_cell_temp_centi_deg >= MAX_CELL_TEMP_CENTI_C){
        error |= CMR_CAN_ERROR_LV_BMS_CELL_OVER_TEMP;
    }
    
    memcpy(&(heartbeat.error), &(error), sizeof(error));
    memcpy(&(heartbeat.warning), &(warning), sizeof(warning));
 }
