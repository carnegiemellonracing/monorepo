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
int BMBNum = 0;

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
static void sendAllBMBVoltages(uint8_t bmb_index);
static void sendHVBMSPackVoltage(void); 
static void checkClearErr(void); 
static uint16_t thermVoltage_to_tempC(uint16_t thermVolt); 

#define THERM_MV_TO_TEMP_NUM_ITEMS 101

/**
 * @brief Look up table thermistor voltage to to temp (C) (indexed by temp)
 * 
 * goes from temp 0 to 100 C 
 */
static uint16_t thermMV_to_tempC[THERM_MV_TO_TEMP_NUM_ITEMS] = {
    27280, 26140, 25050, 24010, 23020, 22070, 21170, 20310, 19490, 18710,
    17960, 17250, 16570, 15910, 15290, 14700, 14130, 13590, 13070, 12570,
    12090, 11640, 11200, 10780, 10380, 10000, 9633, 9282, 8945, 8622,
    8312, 8015, 7730, 7456, 7194, 6942, 6700, 6468, 6245, 6031,
    5826, 5628, 5438, 5255, 5080, 4911, 4749, 4592, 4442, 4297,
    4158, 4024, 3895, 3771, 3651, 3536, 3425, 3318, 3215, 3115,
    3019, 2927, 2837, 2751, 2668, 2588, 2511, 2436, 2364, 2295,
    2227, 2163, 2100, 2039, 1981, 1924, 1869, 1817, 1765, 1716,
    1668, 1622, 1577, 1534, 1492, 1451, 1412, 1374, 1337, 1302,
    1267, 1234, 1201, 1170, 1139, 1110, 1081, 1053, 1027, 1001,
    975
}; 

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
        // sendAllBMBVoltages();

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
        sendBMSBMBStatusErrors();
        checkClearErr();
        sendAllBMBVoltages(BMBNum);
        BMBNum = (BMBNum+1) % 16;
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
    
    //currently swapped because the min logic reading only voltage
    cmr_canBMSMinMaxCellTemperature_t BMSBMBMinMaxTemperature = {
        .minCellTemp_C = thermVoltage_to_tempC(maxCellTemp),
        .maxCellTemp_C = thermVoltage_to_tempC(minCellTemp),
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
    cmr_canHeartbeat_t *vsm_heartbeat = getPayload(CANRX_HEARTBEAT_VSM);
    cmr_canHVCError_t currentError = checkHVBMSErrors();

    cmr_canHeartbeat_t HVBMSHeartbeat = {
        .state = vsm_heartbeat->state 
    }; 

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

static void sendAllBMBVoltages(uint8_t bmbIndex) {
    uint16_t voltageMask = 0x1FFE;
    BMB_Data_t *data = getBMBData(bmbIndex);
    cmr_can_HVBMS_BMB_CellVoltages_t volt0 = {
        .cellVoltage0_mV = (data->cellVoltages[0] && voltageMask) >> 1,
        .cellVoltage1_mV = (data->cellVoltages[1] && voltageMask) >> 1,
        .cellVoltage2_mV = (data->cellVoltages[2] && voltageMask) >> 1,
        .cellVoltage3_mV = (data->cellVoltages[3] && voltageMask) >> 1,
        .cellVoltage4_mV = (data->cellVoltages[4] && voltageMask) >> 1
    };
    cmr_can_HVBMS_BMB_CellVoltages_t volt1 = {
        .cellVoltage0_mV = (data->cellVoltages[5] && voltageMask) >> 1,
        .cellVoltage1_mV = (data->cellVoltages[6] && voltageMask) >> 1,
        .cellVoltage2_mV = (data->cellVoltages[7] && voltageMask) >> 1,
        .cellVoltage3_mV = (data->cellVoltages[8] && voltageMask) >> 1
    };

    cmr_can_HVBMS_BMB_CellTemps_t temp0 = {
        .cellTemp0_dC = (data->cellTemperaturesVoltageReading[0] && voltageMask) >> 1,
        .cellTemp1_dC = (data->cellTemperaturesVoltageReading[1] && voltageMask) >> 1,
        .cellTemp2_dC = (data->cellTemperaturesVoltageReading[2] && voltageMask) >> 1,
        .cellTemp3_dC = (data->cellTemperaturesVoltageReading[3] && voltageMask) >> 1,
        .cellTemp4_dC = (data->cellTemperaturesVoltageReading[4] && voltageMask) >> 1
    };
    cmr_can_HVBMS_BMB_CellTemps_t temp1 = {
        .cellTemp0_dC = (data->cellTemperaturesVoltageReading[5] && voltageMask) >> 1,
        .cellTemp1_dC = (data->cellTemperaturesVoltageReading[6] && voltageMask) >> 1,
        .cellTemp2_dC = (data->cellTemperaturesVoltageReading[7] && voltageMask) >> 1,
        .cellTemp3_dC = (data->cellTemperaturesVoltageReading[8] && voltageMask) >> 1
    };

    canTX(CMR_CANID_HVBMS_BMB_0_CELL_VOLTAGES_0_4 + bmbIndex, &volt0, sizeof(volt0), canTX100Hz_period_ms);
    canTX(CMR_CANID_HVBMS_BMB_0_CELL_VOLTAGES_5_9 + bmbIndex, &volt1, sizeof(volt1), canTX100Hz_period_ms);
    canTX(CMR_CANID_HVBMS_BMB_0_CELL_TEMPS_0_4 + bmbIndex, &temp0, sizeof(temp0), canTX100Hz_period_ms);
    canTX(CMR_CANID_HVBMS_BMB_0_CELL_TEMPS_5_9 + bmbIndex, &temp1, sizeof(temp1), canTX100Hz_period_ms);
}

static uint16_t thermVoltage_to_tempC(uint16_t thermVolt_mV){
    float pullup_ohms = 4700.0; 
    float v_in_mV = 5000.0; 

    //outside of upper bound 
    if (thermVolt_mV >= v_in_mV){
        return 0; 
    }

    float resistance = (pullup_ohms * thermVolt_mV) /  (v_in_mV - thermVolt_mV); 
    float temp_C; 

    //if thermVolt greater than max (don't allow negative temps)
    if (thermVolt_mV > thermMV_to_tempC[0]) {
        return 0; 
    }

    for (size_t i = 0; i < THERM_MV_TO_TEMP_NUM_ITEMS - 1; i++) {
        float table_resistance = thermMV_to_tempC[i]; 

        if (table_resistance == resistance) {
            return i * 100;
        }

        //temp is between i and i+1 (assume pretty much linear ratio)
        if (resistance < table_resistance && resistance > thermMV_to_tempC[i+1]) {
            temp_C = i + ((float)(table_resistance - resistance)) / ((float)(table_resistance - thermMV_to_tempC[i+1])); 
            return (uint16_t)(temp_C * 100); 
        }

    }
    // if we get to end of loop, voltage is less than lowest voltage in lut
    return 10100;

}