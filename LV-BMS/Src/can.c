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
#include <stm32f4xx_hal_can.h> // HAL interface
#include <CMR/tasks.h>  // Task interface

#include "adc.h"
#include "can.h"
#include "data.h"
#include "bq_interface.h"
// INTERFACES

#define CELLS_PER_CELL_GROUP 5
#define NUM_CELLS 7

/**
 * @brief CAN periodic message receive metadata
 *
 * @note Indexed by `canRX_t`.
 */
cmr_canRXMeta_t canRXMeta[CANRX_LEN] = {
    [CANRX_HEARTBEAT_VSM] = {
        .canID = CMR_CANID_HEARTBEAT_VSM,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 25,
        .errorFlag = CMR_CAN_ERROR_VSM_TIMEOUT,
        .warnFlag = CMR_CAN_WARN_VSM_TIMEOUT
    }, 
    [CANRX_BALANCE_COMMAND] = {
		.canID = CMR_CANID_LV_CELL_BALANCE_ENABLE,
		.timeoutError_ms = 50,
		.timeoutWarn_ms = 25
	}
};

cmr_canHeartbeat_t heartbeat;

/** @brief CAN 2 Hz TX priority. */
const uint32_t canTX2Hz_priority = 4;
/** @brief CAN 2 Hz TX period (milliseconds). */
const TickType_t canTX2Hz_period_ms = 500;

/** @brief CAN 100 Hz TX priority. */
const uint32_t canTX100Hz_priority = 5;
/** @brief CAN 100 Hz TX period (milliseconds). */
const TickType_t canTX100Hz_period_ms = 10;

/** @brief CAN 2 Hz TX task. */
static cmr_task_t canTX2Hz_task;
/** @brief CAN 100 Hz TX task. */
static cmr_task_t canTX2Hz_task;

/** @brief Primary CAN interface. */
static cmr_can_t can;

// Forward declarations 
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

/**
 * @brief Sets up LV-BMS heartbeat, checks for errors, then sends it
 *
 * @param lastWakeTime Pass in from canTX100Hz. Used to determine VSM timeout.
 */
static void sendHeartbeat(TickType_t lastWakeTime) {
    cmr_canRXMeta_t *heartbeatVSMMeta = canRXMeta + CANRX_HEARTBEAT_VSM;
    volatile cmr_canHeartbeat_t *heartbeatVSM = getPayload(CANRX_HEARTBEAT_VSM);

    heartbeat.state = heartbeatVSM->state;

    uint16_t error = CMR_CAN_ERROR_NONE;

    if (cmr_canRXMetaTimeoutError(heartbeatVSMMeta, lastWakeTime) < 0) {
        error |= CMR_CAN_ERROR_VSM_TIMEOUT;
    }

    // Add errors Here ^

    // If error exists, update heartbeat to error state (i.e. update its fields). See can_types.h for the fields.
    if (error != CMR_CAN_ERROR_NONE) {
        heartbeat.state = CMR_CAN_ERROR;
    }
    memcpy(&heartbeat.error, &error, sizeof(error));

    uint16_t warning = CMR_CAN_WARN_NONE;
    if (cmr_canRXMetaTimeoutWarn(heartbeatVSMMeta, lastWakeTime) < 0) {
        warning |= CMR_CAN_WARN_VSM_TIMEOUT;
    }
    memcpy(&heartbeat.warning, &warning, sizeof(warning));

    canTX(CMR_CANID_HEARTBEAT_LV_BMS, &heartbeat, sizeof(heartbeat), canTX100Hz_period_ms);
}

void sendVoltages(uint8_t cell_group) {
    PackedCellVoltages cell_volts;
    uint8_t base  = CLAMP(0, cell_group * CELLS_PER_CELL_GROUP, CELL_NUM); 
    uint8_t cell2 = CLAMP(0, base + 1, CELL_NUM);
    uint8_t cell3 = CLAMP(0, base + 2, CELL_NUM);
    uint8_t cell4 = CLAMP(0, base + 3, CELL_NUM);
    uint8_t cell5 = CLAMP(0, base + 4, CELL_NUM);
    cell_volts.cell1_mV_rs1 = getVoltageData(base);
    cell_volts.cell2_mV_rs1 = getVoltageData(cell2);
    cell_volts.cell3_mV_rs1 = getVoltageData(cell3);
    cell_volts.cell4_mV_rs1 = getVoltageData(cell4);
    cell_volts.cell5_mV_rs1 = getVoltageData(cell5);
    // note this relies on contiguous CAN_Ids
    canTX(CMR_CANID_LVBMS_CELL_VOLTAGE_1_4 + cell_group, &cell_volts, sizeof(cell_volts), canTX10Hz_period_ms);
}

void sendTemps(uint8_t cell_group) {
    PackedCellTemps cell_temps;
    uint8_t base  = CLAMP(0, cell_group * CELLS_PER_CELL_GROUP;, CELL_NUM); 
    uint8_t cell2 = CLAMP(0, base + 1, CELL_NUM);
    uint8_t cell3 = CLAMP(0, base + 2, CELL_NUM);
    uint8_t cell4 = CLAMP(0, base + 3, CELL_NUM);
    uint8_t cell5 = CLAMP(0, base + 4, CELL_NUM);
    cell_temps.cell1_dC = getTempData(base);
    cell_temps.cell1_dC = getTempData(cell2);
    cell_temps.cell1_dC = getTempData(cell3);
    cell_temps.cell1_dC = getTempData(cell4);
    cell_temps.cell1_dC = getTempData(cell5);
    // note this relies on contiguous CAN_Ids
    canTX(CMR_CANID_LVBMS_CELL_TEMP_1_4 + cell_group, &cell_temps, sizeof(cell_temps), canTX10Hz_period_ms);
}


/**
 * @brief Task for sending CAN messages at 2 Hz.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTX2Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.
    static uint8_t send_cell_group = 0; // We alternate between sending cell group 0 and 1

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        sendVoltages(send_cell_group);
        sendTemps(send_cell_group);
        send_cell_group = !send_cell_group;
        vTaskDelayUntil(&lastWakeTime, canTX10Hz_period_ms);
        
    }
}

static void sendLVBMSMinMaxCellVoltage(void) {
    uint16_t minCellVoltage = UINT16_MAX;
    uint16_t maxCellVoltage = 0;
    uint16_t minVoltageCellNum, maxVoltageCellNum;

    for (uint8_t lvbms_index = 0; lvbms_index < NUM_CELLS-1; lvbms_index++) {
        uint8_t currCellVoltage = getVoltageData(lvbms_index);

        if (currCellVoltage > maxCellVoltage) {
            maxCellVoltage = currCellVoltage;
            maxVoltageCellNum = lvbms_index;
        }

        if (currCellVoltage < minCellVoltage) {
            minCellVoltage = currCellVoltage;
            minVoltageCellNum = lvbms_index;
        }
    }

    cmr_canLVBMSMinMaxCellVoltage_t LVBMSMinMaxVoltage = {
        .minCellVoltage_mV = minCellVoltage,
        .maxCellVoltage_mV = maxCellVoltage,
        .minVoltageCellNum = minVoltageCellNum,
        .maxVoltageCellNum = maxVoltageCellNum
    };

    canTX(CMR_CANID_LVBMS_MINMAX_CELL_VOLTAGE, &LVBMSMinMaxVoltage, sizeof(LVBMSMinMaxVoltage), canTX200Hz_period_ms);
}

static void sendLVBMSMinMaxCellTemps(void) {
    uint16_t minCellTemp = UINT16_MAX;
    uint16_t maxCellTemp = 0;
    uint16_t minTempCellNum, maxTempCellNum;

    for (uint8_t lvbms_index = 0; lvbms_index < NUM_CELLS-1; lvbms_index++) {
        uint8_t currCellTemp = getTempData(lvbms_index);

        if (currCellTemp > maxCellTemp) {
            maxCellTemp = currCellTemp;
            maxTempCellNum = lvbms_index;
        }

        if (currCellTemp < minCellTemp) {
            minCellTemp = currCellTemp;
            minTempCellNum = lvbms_index;
        }
    }

    cmr_canLVBMSMinMaxCellTemp_t LVBMSMinMaxTemp = {
        .minCellTemp_C = minCellTemp,
        .maxCellTemp_C = maxCellTemp,
        .minTempCellNum = minTempCellNum,
        .maxTempCellNum = maxTempCellNum
    };

    canTX(CMR_CANID_LVBMS_MINMAX_CELL_TEMPS, &LVBMSMinMaxTemp, sizeof(LVBMSMinMaxTemp), canTX200Hz_period_ms);
}

/**
 * @brief Task for sending CAN messages at 100 Hz.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTX100Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        sendHeartbeat(lastWakeTime);

        vTaskDelayUntil(&lastWakeTime, canTX100Hz_period_ms);
    }
}



/**
 * @brief Initializes the CAN interface.
 */
void canInit(void) {
    // CAN2 initialization.
    cmr_canInit(
        &can, CAN2,
		CMR_CAN_BITRATE_500K,
        canRXMeta, sizeof(canRXMeta) / sizeof(canRXMeta[0]),
        NULL,
        GPIOB, GPIO_PIN_12,     // CAN2 RX port/pin.
        GPIOB, GPIO_PIN_13      // CAN2 TX port/pin.
    );

    // CAN2 filters.
const cmr_canFilter_t canFilters[] = {
        {
            .isMask = false,
            .rxFIFO = CAN_RX_FIFO0,
            .ids = {
                CMR_CANID_HEARTBEAT_VSM,
                CMR_CANID_LV_CELL_BALANCE_ENABLE 
            }
        }
    };
    cmr_canFilter(
        &can, canFilters, sizeof(canFilters) / sizeof(canFilters[0])
    );

    // Task initialization.
    cmr_taskInit(
        &canTX2Hz_task,
        "CAN TX 2Hz",
        canTx2Hz_priority,
        canTX2Hz,
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
int canTX(cmr_canID_t id,const void *data, size_t len, TickType_t timeout) {
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