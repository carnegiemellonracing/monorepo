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
#include <CMR/utils.h>

#include "adc.h"
#include "analysis.h"
#include "bq_interface.h"
#include "can.h"
#include "error.h"

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

extern cmr_canHeartbeat_t heartbeat;

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
static cmr_task_t canTX100Hz_task;

/** @brief Primary CAN interface. */
static cmr_can_t can;

/**
 * @brief Sets up LV-BMS heartbeat, checks for errors, then sends it
 *
 * @param lastWakeTime Pass in from canTX100Hz. Used to determine VSM timeout.
 */
static void sendHeartbeat(TickType_t lastWakeTime) {
    cmr_canRXMeta_t *heartbeatVSMMeta = canRXMeta + CANRX_HEARTBEAT_VSM;
    volatile cmr_canHeartbeat_t *heartbeatVSM = getPayload(CANRX_HEARTBEAT_VSM);
    heartbeat.state = heartbeatVSM->state;
    update_errors_and_warnings(lastWakeTime);

    cmr_canError_t error;

    if (error != CMR_CAN_ERROR_NONE) {
        heartbeat.state = CMR_CAN_ERROR;
    }
    canTX(CMR_CANID_HEARTBEAT_LV_BMS, &heartbeat, sizeof(heartbeat), canTX100Hz_period_ms);
}

void sendVoltages(uint8_t cell_group) {
    PackedCellVoltages cell_volts;
    uint8_t base  = CLAMP(0, cell_group * CELLS_PER_CELL_GROUP, CELL_NUM); 
    uint8_t cell2 = CLAMP(0, base + 1, CELL_NUM);
    uint8_t cell3 = CLAMP(0, base + 2, CELL_NUM);
    uint8_t cell4 = CLAMP(0, base + 3, CELL_NUM);
    uint8_t cell5 = CLAMP(0, base + 4, CELL_NUM);
    cell_volts.cell1_mV_rs1 = getVoltageData_mV(base)  << 1;
    cell_volts.cell2_mV_rs1 = getVoltageData_mV(cell2) << 1;
    cell_volts.cell3_mV_rs1 = getVoltageData_mV(cell3) << 1;
    cell_volts.cell4_mV_rs1 = getVoltageData_mV(cell4) << 1;;
    cell_volts.cell5_mV_rs1 = getVoltageData_mV(cell5) << 1;;
    // note this relies on contiguous CAN_Ids
    canTX(CMR_CANID_LVBMS_CELL_VOLTAGE_1_4 + cell_group, &cell_volts, sizeof(cell_volts), canTX100Hz_period_ms);
}

void sendTemps(uint8_t cell_group) {
    PackedCellTemps cell_temps;
    uint8_t base  = CLAMP(0, cell_group * CELLS_PER_CELL_GROUP, CELL_NUM); 
    uint8_t cell2 = CLAMP(0, base + 1, CELL_NUM);
    uint8_t cell3 = CLAMP(0, base + 2, CELL_NUM);
    uint8_t cell4 = CLAMP(0, base + 3, CELL_NUM);
    uint8_t cell5 = CLAMP(0, base + 4, CELL_NUM);
    cell_temps.cell1_dC = getTempData_centi_C(base);
    cell_temps.cell1_dC = getTempData_centi_C(cell2);
    cell_temps.cell1_dC = getTempData_centi_C(cell3);
    cell_temps.cell1_dC = getTempData_centi_C(cell4);
    cell_temps.cell1_dC = getTempData_centi_C(cell5);
    // note this relies on contiguous CAN_Ids
    canTX(CMR_CANID_LVBMS_CELL_TEMP_1_4 + cell_group, &cell_temps, sizeof(cell_temps), canTX100Hz_period_ms);
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
        vTaskDelayUntil(&lastWakeTime, canTX2Hz_period_ms);
        
    }
}

static void sendLVBMSVoltages(void) {
    cmr_canLVBMSVoltage_t LVBMSVoltages = {
        .battVoltage_mV    = BMS_stats.batt_voltage_mV,
        .minCellVoltage_mV = BMS_stats.min_cell_voltage_mV,
        .maxCellVoltage_mV = BMS_stats.max_cell_voltage_mV,
        .minVoltageCellNum = BMS_stats.min_cell_voltage_idx,
        .maxVoltageCellNum = BMS_stats.max_cell_voltage_idx
    };

    canTX(CMR_CANID_HVBMS_MIN_MAX_CELL_VOLTAGE, &LVBMSVoltages, sizeof(LVBMSVoltages), canTX100Hz_period_ms);
}

static void sendLVBMSTemps(void) {
    cmr_canLVBMSMinMaxCellTemp_t LVBMSTemps = {
        .minCellTemp_centi_C =  BMS_stats.min_cell_temp_centi_deg,
        .maxCellTemp_centi_C =  BMS_stats.max_cell_temp_centi_deg,
        .minTempCellNum      =  BMS_stats.min_cell_temp_idx,
        .maxTempCellNum      =  BMS_stats.max_cell_temp_idx,
    };
    canTX(CMR_CANID_LVBMS_MINMAX_CELL_TEMPS, &LVBMSTemps, sizeof(LVBMSTemps), canTX100Hz_period_ms);
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
        sendLVBMSTemps();
        sendLVBMSVoltages();
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
        canTX2Hz_priority,
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
