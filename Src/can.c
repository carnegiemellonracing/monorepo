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
#include "adc.h"      // adcVSense, adcISense
#include "sensors.h"  // HVC Values
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
    [CANRX_HEARTBEAT_VSM] = {
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
	[CANRX_HVI_COMMAND] = {
		.canID = CMR_CANID_HEARTBEAT_HVI,
		.timeoutError_ms = 50,
		.timeoutWarn_ms = 25
    },
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
static void sendHVCPackVoltage(void);
static void sendBMSPackCurrent(void);
static void sendBMSBMBStatusErrors(void);
static void sendBMSBMBStatusVoltage(uint8_t bmb_index);
static void sendBMSBMBStatusTemp(uint8_t bmb_index);
static void sendBMSMinMaxCellVoltage(void);
static void sendBMSMinMaxCellTemp(void);
static void sendBMSLowVoltage(void);
static void sendAllBMBVoltages(void);

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
static const uint32_t canTX200Hz_priority = 5;

/** @brief CAN 100 Hz TX period (milliseconds). */
static const TickType_t canTX200Hz_period_ms = 10;

/** @brief CAN 100 Hz TX task. */
static cmr_task_t canTX200Hz_task;

/** @brief CAN 100 Hz TX priority. */
static const uint32_t canTX100Hz_priority = 5;

/** @brief CAN 100 Hz TX period (milliseconds). */
static const TickType_t canTX100Hz_period_ms = 10;

/** @brief CAN 100 Hz TX task. */
static cmr_task_t canTX100Hz_task;

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

        vTaskDelayUntil(&lastWakeTime, canTX200Hz_period_ms);
    }
}

static void canTX100Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

//    cmr_canRXMeta_t *heartbeatVSMMeta = canRXMeta + CANRX_HEARTBEAT_VSM;
//    volatile cmr_canHeartbeat_t *heartbeatVSM =
//        (void *) heartbeatVSMMeta->payload;

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        sendHeartbeat(lastWakeTime);
        sendHVCPackVoltage();
        //sendBMSPackCurrent();
        sendBMSLowVoltage();
        sendBMSBMBStatusErrors();

        vTaskDelayUntil(&lastWakeTime, canTX100Hz_period_ms);
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
				CMR_CANID_HEARTBEAT_HVI,
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

/**
 * @brief Sets up HVC CAN heartbeat with current errors and warnings, then sends it.
 *
 * @param lastWakeTime Pass in from canTX100Hz. Used to update lastStateChangeTime and errors/warnings.
 */
static void sendHeartbeat(TickType_t lastWakeTime) {
    cmr_canHVCState_t currentState = getState();
    cmr_canHVCError_t currentError = CMR_CAN_HVC_ERROR_NONE;
    currentError = checkErrors(currentState);

    cmr_canHVCHeartbeat_t HVCHeartbeat = {
        .errorStatus = currentError,
        .hvcMode = CMR_CAN_HVC_MODE_ERROR,
        .hvcState = currentState,
        .relayStatus = getRelayStatus(),
        .uptime_s = 0,
    };

    switch (currentState) {
        case CMR_CAN_HVC_STATE_DISCHARGE: // S1
            HVCHeartbeat.hvcMode = CMR_CAN_HVC_MODE_IDLE;
            break;
        case CMR_CAN_HVC_STATE_STANDBY: // S2
            HVCHeartbeat.hvcMode = CMR_CAN_HVC_MODE_IDLE;
            break;
        case CMR_CAN_HVC_STATE_DRIVE_PRECHARGE: // S3
            HVCHeartbeat.hvcMode = CMR_CAN_HVC_MODE_START;
            break;
        case CMR_CAN_HVC_STATE_DRIVE_PRECHARGE_COMPLETE: // S4
            HVCHeartbeat.hvcMode = CMR_CAN_HVC_MODE_START;
            break;
        case CMR_CAN_HVC_STATE_DRIVE: // S5
            HVCHeartbeat.hvcMode = CMR_CAN_HVC_MODE_RUN;
            break;
        case CMR_CAN_HVC_STATE_CHARGE_PRECHARGE: // S6
            HVCHeartbeat.hvcMode = CMR_CAN_HVC_MODE_CHARGE;
            break;
        case CMR_CAN_HVC_STATE_CHARGE_PRECHARGE_COMPLETE: // S7
            HVCHeartbeat.hvcMode = CMR_CAN_HVC_MODE_CHARGE;
            break;
        case CMR_CAN_HVC_STATE_CHARGE_TRICKLE: // S8
            HVCHeartbeat.hvcMode = CMR_CAN_HVC_MODE_CHARGE;
            break;
        case CMR_CAN_HVC_STATE_CHARGE_CONSTANT_CURRENT: // S9
            HVCHeartbeat.hvcMode = CMR_CAN_HVC_MODE_CHARGE;
            break;
        case CMR_CAN_HVC_STATE_CHARGE_CONSTANT_VOLTAGE: // S10
            HVCHeartbeat.hvcMode = CMR_CAN_HVC_MODE_CHARGE;
            break;
        case CMR_CAN_HVC_STATE_ERROR: // S0
            HVCHeartbeat.hvcMode = CMR_CAN_HVC_MODE_ERROR;
            break;
        case CMR_CAN_HVC_STATE_CLEAR_ERROR: // S11
            HVCHeartbeat.hvcMode = CMR_CAN_HVC_MODE_ERROR;
            break;
        case CMR_CAN_HVC_STATE_UNKNOWN:
        default:
            HVCHeartbeat.hvcMode = CMR_CAN_HVC_MODE_ERROR;
            break;
    }

    canTX(CMR_CANID_HEARTBEAT_HVC, &HVCHeartbeat, sizeof(HVCHeartbeat), canTX100Hz_period_ms);
}

static void sendHVCPackVoltage(void) {
    int32_t bVolt = getBattMillivolts();
    int32_t hvVolt = getHVmillivolts();

    cmr_canHVCPackVoltage_t HVCPackVoltage = {
        .battVoltage_mV = bVolt,
        .hvVoltage_mV = hvVolt,
    };

    canTX(CMR_CANID_HVC_PACK_VOLTAGE, &HVCPackVoltage, sizeof(HVCPackVoltage), canTX100Hz_period_ms);
}

static void sendBMSPackCurrent(void) {
    int32_t instantCurrent = getHVmilliamps();
    // int32_t avgCurrent = getCurrentAverage(); // TODO: Gustav change this back
    int32_t avgCurrent = instantCurrent;

    cmr_canBMSPackCurrent_t BMSPackCurrent = {
        .instantCurrent_mA = instantCurrent,
        .averageCurrent_mA = avgCurrent,
    };

    canTX(CMR_CANID_HVC_PACK_CURRENT, &BMSPackCurrent, sizeof(BMSPackCurrent), canTX100Hz_period_ms);
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

    canTX(CMR_CANID_HVC_BMB_0_STATUS_VOLTAGE + (bmb_index << 1), &BMSBMBStatusVoltage, sizeof(BMSBMBStatusVoltage), canTX10Hz_period_ms);
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

    canTX(CMR_CANID_HVC_BMB_0_STATUS_TEMP + (bmb_index << 1), &BMSBMBStatusTemp, sizeof(BMSBMBStatusTemp), canTX1Hz_period_ms);
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
        .maxVoltageCellNum = maxCellVoltageIndex,
    };

    canTX(CMR_CANID_HVC_MIN_MAX_CELL_VOLTAGE, &BMSBMBMinMaxVoltage, sizeof(BMSBMBMinMaxVoltage), canTX200Hz_period_ms);
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

    canTX(CMR_CANID_HVC_MIN_MAX_CELL_TEMPERATURE, &BMSBMBMinMaxTemperature, sizeof(BMSBMBMinMaxTemperature), canTX10Hz_period_ms);
}

static void sendBMSLowVoltage(void) {
    cmr_canBMSLowVoltage_t BMSLowVoltage = {
        .safety_mV = (getSafetymillivolts()*15)/2000, // Convert mA to 2/15th mA //TODO: Gustav change this back?
        .iDCDC_mA = 0,
        .vAIR_mV = (getAIRmillivolts()*15)/2000, // Convert mV to 2/15th V
        .vbatt_mV= (getLVmillivolts()*15/2000), // Convert mV to 2/15th V
    };
    (void) BMSLowVoltage;

    canTX(CMR_CANID_HVC_LOW_VOLTAGE, &BMSLowVoltage, sizeof(BMSLowVoltage), canTX100Hz_period_ms);
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
//    for (int bmbIndex = 0; bmbIndex < BOARD_NUM; bmbIndex++) {
//        BMB_Data_t *data = getBMBData(bmbIndex);
//        cmr_canHVCBMB_Voltage0_t volt0 = {
//            .cellVoltage0_mV = data->cellVoltages[0],
//            .cellVoltage1_mV = data->cellVoltages[1],
//            .cellVoltage2_mV = data->cellVoltages[2],
//            .cellVoltage3_mV = data->cellVoltages[3]
//        };
//        cmr_canHVCBMB_Voltage1_t volt1 = {
//            .cellVoltage4_mV = data->cellVoltages[4],
//            .cellVoltage5_mV = data->cellVoltages[5],
//            .cellVoltage6_mV = data->cellVoltages[6],
//            .cellVoltage7_mV = data->cellVoltages[7]
//        };
//        cmr_canHVCBMB_Voltage2_t volt2 = {
//            .cellVoltage8_mV = data->cellVoltages[8]
//        };
//
//        cmr_canHVCBMB_Temp0_t temp0 = {
//            .cellTemp0_dC = data->cellTemperatures[0],
//            .cellTemp1_dC = data->cellTemperatures[1],
//            .cellTemp2_dC = data->cellTemperatures[2],
//            .cellTemp3_dC = data->cellTemperatures[3]
//        };
//        cmr_canHVCBMB_Temp1_t temp1 = {
//            .cellTemp4_dC = data->cellTemperatures[4],
//            .cellTemp5_dC = data->cellTemperatures[5],
//            .cellTemp6_dC = data->cellTemperatures[6],
//            .cellTemp7_dC = data->cellTemperatures[7]
//        };
//        cmr_canHVCBMB_Temp2_t temp2 = {
//            .cellTemp8_dC = data->cellTemperatures[8],
//            .cellTemp9_dC = data->cellTemperatures[9],
//            .cellTemp10_dC = data->cellTemperatures[10],
//            .cellTemp11_dC = data->cellTemperatures[11]
//        };
//
//        canTX(CMR_CANID_HVC_BMB_0_STATUS_VOLTAGE_0 + (bmbIndex << 4), &volt0, sizeof(volt0), canTX1Hz_period_ms);
//        canTX(CMR_CANID_HVC_BMB_0_STATUS_VOLTAGE_1 + (bmbIndex << 4), &volt1, sizeof(volt1), canTX1Hz_period_ms);
//        canTX(CMR_CANID_HVC_BMB_0_STATUS_VOLTAGE_2 + (bmbIndex << 4), &volt2, sizeof(volt2), canTX1Hz_period_ms);
//        canTX(CMR_CANID_HVC_BMB_0_STATUS_TEMP_0 + (bmbIndex << 4), &temp0, sizeof(temp0), canTX1Hz_period_ms);
//        canTX(CMR_CANID_HVC_BMB_0_STATUS_TEMP_1 + (bmbIndex << 4), &temp1, sizeof(temp1), canTX1Hz_period_ms);
//        canTX(CMR_CANID_HVC_BMB_0_STATUS_TEMP_2 + (bmbIndex << 4), &temp2, sizeof(temp2), canTX1Hz_period_ms);
//    }
}
