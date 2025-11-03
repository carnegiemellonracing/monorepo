/**
 * @file can.c
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

#include <CMR/tasks.h>  // Task interface

#include "can.h"      // Interface to implement
#include "math.h"

/** @brief Struct to identify stale commands. */
extern ReceiveMeta_t BMSCommandReceiveMeta;

extern volatile int BMBErrs[BOARD_NUM];

/**
 * @brief CAN periodic message receive metadata
 *
 * @note Indexed by `canRX_t`.
 */
cmr_canRXMeta_t canRXMeta[] = {
    [CANRX_HEARTBEAT_VSM] = { //not needed? 
        .canID = CMR_CANID_HEARTBEAT_VSM,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_HVC_COMMAND] = {
        .canID = CMR_CANID_HVC_COMMAND,
        .timeoutError_ms = 200,
        .timeoutWarn_ms = 25
    },
    [CANRX_EMD_MEASURE] = {
        .canID = CMR_CANID_EMD_MEASUREMENT,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 25
    },
    [CANRX_EMD_MEASURE] = {   
        .canID = CMR_CANID_EMD_MEASUREMENT_RETX,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_HEARTBEAT_HVC] = {
        .canID = CMR_CANID_HEARTBEAT_HVC,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25 
    },
    [CANRX_HEARTBEAT_VSM] = { 
        .canID = CMR_CANID_HEARTBEAT_VSM,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25 },
    [CANRX_BALANCE_COMMAND] = {
		.canID = CMR_CANID_CELL_BALANCE_ENABLE,
		.timeoutError_ms = 50,
		.timeoutWarn_ms = 25
	}
};

/** @brief Primary CAN interface. */
static cmr_can_t can;

// Forward declarations
static void sendHeartbeat(TickType_t lastWakeTime);
static void sendBMSBMBStatusErrors(void);
static void sendBMSBMBStatusVoltage(uint8_t bmb_index);
static void sendBMSBMBStatusTemp(uint8_t bmb_index);
static void sendBMSMinMaxCellVoltage(void);
static void sendBMSMinMaxCellTemp(void);
static void sendAllBMBVoltages(void);
static void sendHVBMSPackVoltage(void); 
static void checkClearErr(void); 

/** @brief CAN 1 Hz TX priority. */
static const uint32_t canTX1Hz_priority = 4;

/** @brief CAN 10 Hz TX period (milliseconds). */
static const TickType_t canTX1Hz_period_ms = 1000;

/** @brief CAN 10 Hz TX task. */
static cmr_task_t canTX1Hz_task;

/**
 * @brief Task for sending CAN messages at 10 Hz.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTX1Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {

        // BMB Temperature Status 
        for (uint8_t bmb_index = 0; bmb_index < BOARD_NUM - 1; bmb_index++) {
            sendBMSBMBStatusTemp(bmb_index);
        }
        sendBMSMinMaxCellTemp();
        //send all cells temp and voltage 
        sendAllBMBVoltages();

        vTaskDelayUntil(&lastWakeTime, canTX1Hz_period_ms);
    }
}

/** @brief CAN 10 Hz TX priority. */
static const uint32_t canTX10Hz_priority = 4;

/** @brief CAN 10 Hz TX period (milliseconds). */
static const TickType_t canTX10Hz_period_ms = 100;

/** @brief CAN 10 Hz TX task. */
static cmr_task_t canTX10Hz_task;

/**
 * @brief Task for sending CAN messages at 10 Hz.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTX10Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        // BRUSA Charger decided by state machine 
        // sendBRUSAChargerControl();

        // BMB Voltage Status 
        for (uint8_t bmb_index = 0; bmb_index < BOARD_NUM-1; bmb_index++) {
            sendBMSBMBStatusVoltage(bmb_index);
        }

        vTaskDelayUntil(&lastWakeTime, canTX10Hz_period_ms);
    }
}

/** @brief CAN 100 Hz TX priority. */
static const uint32_t canTX100Hz_priority = 5;

/** @brief CAN 100 Hz TX period (milliseconds). */
static const TickType_t canTX100Hz_period_ms = 10;

/** @brief CAN 100 Hz TX task. */
static cmr_task_t canTX100Hz_task;


static void canTX100Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

//    cmr_canRXMeta_t *heartbeatVSMMeta = canRXMeta + CANRX_HEARTBEAT_VSM;
//    volatile cmr_canHeartbeat_t *heartbeatVSM =
//        (void *) heartbeatVSMMeta->payload;

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        sendHeartbeat(lastWakeTime);
        //sendHVCPackVoltage();
        //sendBMSPackCurrent();
        sendBMSBMBStatusErrors();
        checkClearErr(); 
        vTaskDelayUntil(&lastWakeTime, canTX100Hz_period_ms);
    }
}


/** @brief CAN 100 Hz TX priority. */
static const uint32_t canTX200Hz_priority = 5;

/** @brief CAN 100 Hz TX period (milliseconds). */
static const TickType_t canTX200Hz_period_ms = 10;

/** @brief CAN 100 Hz TX task. */
static cmr_task_t canTX200Hz_task;
/**
 * @brief Task for sending CAN messages at 200 Hz.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTX200Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

//    cmr_canRXMeta_t *heartbeatVSMMeta = canRXMeta + CANRX_HEARTBEAT_VSM;
//    volatile cmr_canHeartbeat_t *heartbeatVSM =
//        (void *) heartbeatVSMMeta->payload;

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        sendBMSMinMaxCellVoltage();
        sendHVBMSPackVoltage(); 

        vTaskDelayUntil(&lastWakeTime, canTX200Hz_period_ms);
    }
}


/**
 * @brief Initializes the CAN interface.
 */
void canInit(void) {
    // CAN2 initialization.
    cmr_canInit(
        &can, CAN1,
		CMR_CAN_BITRATE_500K,
        canRXMeta, sizeof(canRXMeta) / sizeof(canRXMeta[0]),
        NULL,
        GPIOA, GPIO_PIN_11,     // CAN2 RX port/pin.
        GPIOA, GPIO_PIN_12      // CAN2 TX port/pin.
    );

    // CAN2 filters.
    const cmr_canFilter_t canFilters[] = {
        {
            .isMask = false,
            .rxFIFO = CAN_RX_FIFO0,
            .ids = {
                CMR_CANID_HEARTBEAT_VSM,
				CMR_CANID_HV_SENSORS,
                CMR_CANID_HVC_COMMAND,
				CMR_CANID_CELL_BALANCE_ENABLE
            }
        }
    };
    cmr_canFilter(
        &can, canFilters, sizeof(canFilters) / sizeof(canFilters[0])
    );

    // Task initialization.
    cmr_taskInit(
        &canTX1Hz_task,
        "CAN TX 1Hz",
        canTX1Hz_priority,
        canTX1Hz,
        NULL
    );
    cmr_taskInit(
        &canTX10Hz_task,
        "CAN TX 10Hz",
        canTX10Hz_priority,
        canTX10Hz,
        NULL
    );
    cmr_taskInit(
        &canTX200Hz_task,
        "CAN TX 200Hz",
        canTX200Hz_priority,
        canTX200Hz,
        NULL
    );
    cmr_taskInit(
        &canTX100Hz_task,
        "CAN TX 100Hz",
        canTX100Hz_priority,
        canTX100Hz,
        NULL
    );
}

/**
 * @brief Sends a CAN message with the given ID.
 *
 * @param id The ID for the message.
 * @param data The data to send.
 * @param len The data's length, in bytes.
 * @param timeout The timeout, in ticks.
 *
 * @return 0 on success, or a negative error code on timeout.
 */
int canTX(cmr_canID_t id, const void *data, size_t len, TickType_t timeout) {
    return cmr_canTX(&can, id, data, len, timeout);
}

/**
 * @brief Gets a pointer to the payload of a received CAN message.
 *
 * @param rxMsg The message to get the payload of.
 *
 * @return Pointer to payload, or NULL if rxMsg is invalid.
 */
volatile void *getPayload(canRX_t rxMsg) {
    configASSERT(rxMsg < CANRX_LEN);

    cmr_canRXMeta_t *rxMeta = &(canRXMeta[rxMsg]);

    return (void *)(&rxMeta->payload);
}


static void sendBMSBMBStatusVoltage(uint8_t bmb_index) {
    uint8_t maxIndex = getBMBMaxVoltIndex(bmb_index);
    uint8_t minIndex = getBMBMinVoltIndex(bmb_index);
    uint16_t maxVoltage = getBMBVoltage(bmb_index, maxIndex);
    uint16_t minVoltage = getBMBVoltage(bmb_index, minIndex);

    cmr_canBMSBMBStatusVoltage_t BMSBMBStatusVoltage = {
        .maxVoltIndex = maxIndex,
        .minVoltIndex = minIndex,
        .maxCellVoltage_mV = maxVoltage,
        .minCellVoltage_mV = minVoltage,
    };

    canTX(CMR_CANID_HVBMS_BMB_0_STATUS_VOLTAGE + (bmb_index << 1), &BMSBMBStatusVoltage, sizeof(BMSBMBStatusVoltage), canTX10Hz_period_ms);
}

static void sendBMSBMBStatusTemp(uint8_t bmb_index) {
    uint8_t maxIndex = getBMBMaxTempIndex(bmb_index);
    uint8_t minIndex = getBMBMinTempIndex(bmb_index);
    int16_t maxTemp = getBMBTemp(bmb_index, maxIndex);
    int16_t minTemp = getBMBTemp(bmb_index, minIndex);

    cmr_canBMSBMBStatusTemp_t BMSBMBStatusTemp = {
        .maxTempIndex = maxIndex,
        .minTempIndex = minIndex,
        .maxCellTemp_C = maxTemp,
        .minCellTemp_C = minTemp,
    };

    canTX(CMR_CANID_HVBMS_BMB_0_STATUS_TEMP + (bmb_index << 1), &BMSBMBStatusTemp, sizeof(BMSBMBStatusTemp), canTX1Hz_period_ms);
}

static void sendBMSMinMaxCellVoltage(void) {
    uint16_t minCellVoltage = UINT16_MAX;
    uint16_t maxCellVoltage = 0;

    uint8_t minCellVoltageBMBNum = 0;
	uint8_t maxCellVoltageBMBNum = 0;
	
	uint8_t minCellVoltageIndex = 0;
	uint8_t maxCellVoltageIndex = 0;

    for (uint8_t bmb_index = 0; bmb_index < BOARD_NUM-1; bmb_index++) {
        uint8_t maxIndex = getBMBMaxVoltIndex(bmb_index);
        uint8_t minIndex = getBMBMinVoltIndex(bmb_index);
        uint16_t maxVoltage = getBMBVoltage(bmb_index, maxIndex);
        uint16_t minVoltage = getBMBVoltage(bmb_index, minIndex);

        if (maxVoltage > maxCellVoltage) {
            maxCellVoltage = maxVoltage;
            maxCellVoltageBMBNum = bmb_index;
            maxCellVoltageIndex = maxIndex;
        }

        if (minVoltage < minCellVoltage) {
            minCellVoltage = minVoltage;
            minCellVoltageBMBNum = bmb_index;
            minCellVoltageIndex = minIndex;
        }
    }

    cmr_canBMSMinMaxCellVoltage_t BMSBMBMinMaxVoltage = {
        .minCellVoltage_mV = minCellVoltage,
        .maxCellVoltage_mV = maxCellVoltage,
        .minVoltageBMBNum = minCellVoltageBMBNum,
        .maxVoltageBMBNum = maxCellVoltageBMBNum,
        .minVoltageCellNum = minCellVoltageIndex,
        .maxVoltageCellNum = maxCellVoltageIndex
    };

    canTX(CMR_CANID_HVBMS_MIN_MAX_CELL_VOLTAGE, &BMSBMBMinMaxVoltage, sizeof(BMSBMBMinMaxVoltage), canTX200Hz_period_ms);
}

static void sendHVBMSPackVoltage(void){
    uint32_t battvolt = getBattMillivolts();
    
    cmr_canHVBMSPackVoltage_t bmsbattvoltage = {
        .battVoltage_mV = battvolt
    }; 

    canTX(CMR_CANID_HVBMS_PACK_VOLTAGE, &bmsbattvoltage, sizeof(bmsbattvoltage), canTX200Hz_period_ms);  
}

static void sendBMSMinMaxCellTemp(void) {
    uint16_t minCellTemp = UINT16_MAX;
    uint16_t maxCellTemp = 0;

    uint8_t minCellTempBMBNum;
	uint8_t maxCellTempBMBNum;
	
	uint8_t minCellTempIndex;
	uint8_t maxCellTempIndex;

    for (uint8_t bmb_index = 0; bmb_index < BOARD_NUM-1; bmb_index++) {
        uint8_t maxIndex = getBMBMaxTempIndex(bmb_index);
        uint8_t minIndex = getBMBMinTempIndex(bmb_index);
        uint16_t maxTemp = getBMBTemp(bmb_index, maxIndex);
        uint16_t minTemp = getBMBTemp(bmb_index, minIndex);

        //bmb 2 channel 13, bmb 7 channel 0, bmb 8 channel 9/11, bmb 9 channel 6
        if ((maxTemp > maxCellTemp)) {
            maxCellTemp = maxTemp;
            maxCellTempBMBNum = bmb_index;
            maxCellTempIndex = maxIndex;
        }

        if (minTemp < minCellTemp) {
            minCellTemp = minTemp;
            minCellTempBMBNum = bmb_index;
            minCellTempIndex = minIndex;
        }
    }
    float xMin = (4.7f*((float)minCellTemp)/1000.0f)* 1000.0f / ((5.0f - (((float)minCellTemp)/1000.0f)) );
    float temp_minTemp = (1/(0.00335348f + (0.00030662f*log((xMin)/10000.0f)) + powf(0.00000837316f*log(xMin/10000.0f), 2))) - 273.15;
    minCellTemp = (int16_t) (temp_minTemp * 10.0f);

    float xMax = (4.7f*((float)maxCellTemp)/1000.0f) * 1000.0f / ((5.0f - (((float)maxCellTemp)/1000.0f)));
	float temp_maxTemp = (1/(0.00335348f + (0.00030662f*log((xMax)/10000.0f)) + powf(0.00000837316f*log(xMax/10000.0f), 2))) - 273.15;
	maxCellTemp = (int16_t) (temp_maxTemp * 10.0f);
    //currently swapped because the min logic reading only voltage
    cmr_canBMSMinMaxCellTemperature_t BMSBMBMinMaxTemperature = {
        .minCellTemp_C = maxCellTemp,
        .maxCellTemp_C = minCellTemp,
        .minTempBMBNum = maxCellTempBMBNum,
        .maxTempBMBNum = minCellTempBMBNum,
        .minTempCellNum = maxCellTempIndex,
        .maxTempCellNum = minCellTempIndex,
    };

    canTX(CMR_CANID_HVBMS_MIN_MAX_CELL_TEMPERATURE, &BMSBMBMinMaxTemperature, sizeof(BMSBMBMinMaxTemperature), canTX10Hz_period_ms);
}


/**
 * @brief Sets up BMS Master CAN heartbeat with current errors and warnings, then sends it.
 *
 * @param lastWakeTime Pass in from canTX100Hz. Used to update lastStateChangeTime and errors/warnings.
 */
static void sendHeartbeat(TickType_t lastWakeTime) {
    cmr_canHVCHeartbeat_t *hvcheartbeat = getPayload(CANRX_HEARTBEAT_HVC); 
    cmr_canHVCState_t currentState = hvcheartbeat->hvcState; 
    cmr_canHVCError_t currentError = CMR_CAN_HVC_ERROR_NONE;
    currentError = getHVBMSErrorReg();

    cmr_canHeartbeat_t *vsm_heartbeat = getPayload(CANRX_HEARTBEAT_VSM);

    cmr_canHeartbeat_t HVBMSHeartbeat = {
        .state = vsm_heartbeat->state 
    }; 

    cmr_canWarn_t warning = CMR_CAN_WARN_NONE;
    cmr_canError_t error = CMR_CAN_ERROR_NONE;

    memcpy(&HVBMSHeartbeat.error, &currentError, sizeof(HVBMSHeartbeat.error)); 
    canTX(CMR_CANID_HEARTBEAT_HV_BMS, &HVBMSHeartbeat, sizeof(HVBMSHeartbeat), canTX100Hz_period_ms);
}

static void checkClearErr(void){
    cmr_canHVCHeartbeat_t *hvcheartbeat = getPayload(CANRX_HEARTBEAT_HVC); 
    if(hvcheartbeat->hvcState==CMR_CAN_HVC_STATE_CLEAR_ERROR){
        clearHVBMSErrorReg(); 
    }

}


static void sendBMSBMBStatusErrors(void) {
	//TODO Update status error sending
//	cmr_canHVCBMBErrors_t errs = {
//			.BMB1_2_Errs = (BMBErrs[0] << 4) | BMBErrs[1],
//			.BMB3_4_Errs = (BMBErrs[2] << 4) | BMBErrs[3],
//			.BMB5_6_Errs = (BMBErrs[4] << 4) | BMBErrs[5],
//			.BMB7_8_Errs = (BMBErrs[6] << 4) | BMBErrs[7],
//			.BMB9_10_Errs = (BMBErrs[8] << 4) | BMBErrs[9],
//			.BMB11_12_Errs = (BMBErrs[10] << 4) | BMBErrs[11],
//			.BMB13_14_Errs = (BMBErrs[12] << 4) | BMBErrs[13],
//			.BMB15_16_Errs = (BMBErrs[14] << 4) | BMBErrs[15],
//	};

//	canTX(CMR_CANID_HVC_BMB_STATUS_ERRORS, &errs, sizeof(cmr_canHVCBMBErrors_t), canTX100Hz_period_ms);
}

static void sendAllBMBVoltages(void) {
   for (int bmbIndex = 0; bmbIndex < BOARD_NUM; bmbIndex++) {
       BMB_Data_t *data = getBMBData(bmbIndex);
       cmr_canHVBMS_BMB_CellVoltages_t volt0 = {
           .cellVoltage0_mV = data->cellVoltages[0],
           .cellVoltage1_mV = data->cellVoltages[1],
           .cellVoltage2_mV = data->cellVoltages[2],
           .cellVoltage3_mV = data->cellVoltages[3]
       };
       cmr_canHVBMS_BMB_CellVoltages_t volt1 = {
           .cellVoltage0_mV = data->cellVoltages[4],
           .cellVoltage1_mV = data->cellVoltages[5],
           .cellVoltage2_mV = data->cellVoltages[6],
           .cellVoltage3_mV = data->cellVoltages[7] 
       };
       cmr_canHVBMS_BMB_CellVoltages_t volt2 = {
           .cellVoltage0_mV = data->cellVoltages[8],  
           .cellVoltage1_mV = data->cellVoltages[9], 
           .cellVoltage2_mV = data->cellVoltages[10],
           .cellVoltage3_mV = data->cellVoltages[11]
       };
       cmr_canHVBMS_BMB_CellVoltages_t volt3 = {
            .cellVoltage_0_mV = data->cellVoltages[12], 
            .cellVoltage_1_mV = data->cellVoltages[13],
            .cellVoltage_2_mV = data->cellVoltages[14] 
       }; 

       cmr_canHVBMS_BMB_CellTemps_t temp0 = {
           .cellTemp0_dC = data->cellTemperatures[0],
           .cellTemp1_dC = data->cellTemperatures[1],
           .cellTemp2_dC = data->cellTemperatures[2],
           .cellTemp3_dC = data->cellTemperatures[3]
       };
       cmr_canHVBMS_BMB_CellTemps_t temp1 = {
           .cellTemp0_dC = data->cellTemperatures[4],
           .cellTemp1_dC = data->cellTemperatures[5],
           .cellTemp2_dC = data->cellTemperatures[6],
           .cellTemp3_dC = data->cellTemperatures[7] 
       };
       cmr_canHVBMS_BMB_CellTemps_t temp2 = { 
           .cellTemp0_dC = data->cellTemperatures[8], 
           .cellTemp1_dC = data->cellTemperatures[9],
           .cellTemp2_dC = data->cellTemperatures[10],
           .cellTemp3_dC = data->cellTemperatures[11] 
       };
       cmr_canHVBMS_BMB_CellTemps_t temp3 = {
            .cellTemp0_mV = data->cellTemperatures[12],
            .cellTemp1_mV = data->cellTemperatures[13],
            .cellTemp2_mV = data->cellTemperatures[14] 
       }; 

       canTX(CMR_CANID_HVBMS_BMB_0_CELL_VOLTAGES_0_3 + bmbIndex, &volt0, sizeof(volt0), canTX1Hz_period_ms);
       canTX(CMR_CANID_HVBMS_BMB_0_CELL_VOLTAGES_4_7 + bmbIndex, &volt1, sizeof(volt1), canTX1Hz_period_ms);
       canTX(CMR_CANID_HVBMS_BMB_0_CELL_VOLTAGES_8_11 + bmbIndex, &volt2, sizeof(volt2), canTX1Hz_period_ms);
       canTX(CMR_CANID_HVBMS_BMB_0_CELL_VOLTAGES_12_14 + bmbIndex, &volt3, sizeof(volt3), canTX1Hz_period_ms);
       canTX(CMR_CANID_HVBMS_BMB_0_CELL_TEMPS_0_3 + bmbIndex, &temp0, sizeof(temp0), canTX1Hz_period_ms);
       canTX(CMR_CANID_HVBMS_BMB_0_CELL_TEMPS_4_7 + bmbIndex, &temp1, sizeof(temp1), canTX1Hz_period_ms);
       canTX(CMR_CANID_HVBMS_BMB_0_CELL_TEMPS_8_11 + bmbIndex, &temp2, sizeof(temp2), canTX1Hz_period_ms);
       canTX(CMR_CANID_HVBMS_BMB_0_CELL_TEMPS_12_14 + bmbIndex, &temp3, sizeof(temp3), canTX1Hz_period_ms);
   }
}
