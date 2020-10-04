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

/**
 * @brief CAN periodic message receive metadata
 *
 * @note Indexed by `canRX_t`.
 */
cmr_canRXMeta_t canRXMeta[] = {
    // XXX Edit this to include the appropriate periodic messages.
    [CANRX_HEARTBEAT_VSM] = {
        .canID = CMR_CANID_HEARTBEAT_VSM,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_HVC_COMMAND] = {
        .canID = CMR_CANID_HVC_COMMAND,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    }
};

/** @brief Primary CAN interface. */
static cmr_can_t can;

/** @brief CAN 10 Hz TX priority. */
static const uint32_t canTX10Hz_priority = 3;

/** @brief CAN 10 Hz TX period (milliseconds). */
static const TickType_t canTX10Hz_period_ms = 100;

/** @brief CAN 10 Hz TX task. */
static cmr_task_t canTX10Hz_task;

// Forward declarations
static void sendHeartbeat(TickType_t lastWakeTime);
static void sendBRUSAChargerControl(void);
static void sendHVCPackVoltage(void);
static void sendBMSPackCurrent(void);
static void sendBMSBMBStatusVoltage(uint8_t bmb_index);
static void sendBMSBMBStatusTemp(uint8_t bmb_index);
static void sendBMSMinMaxCellVoltage(void);
static void sendBMSMinMaxCellTemp(void);
static void sendBMSLowVoltage(void);

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
        // XXX Replace with an appropriate message.
        sendBRUSAChargerControl();

        vTaskDelayUntil(&lastWakeTime, canTX10Hz_period_ms);
    }
}

/** @brief CAN 100 Hz TX priority. */
static const uint32_t canTX100Hz_priority = 5;

/** @brief CAN 100 Hz TX period (milliseconds). */
static const TickType_t canTX100Hz_period_ms = 10;

/** @brief CAN 100 Hz TX task. */
static cmr_task_t canTX100Hz_task;

/**
 * @brief Task for sending CAN messages at 100 Hz.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTX100Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    // XXX Example message retrieval.
    cmr_canRXMeta_t *heartbeatVSMMeta = canRXMeta + CANRX_HEARTBEAT_VSM;
    volatile cmr_canHeartbeat_t *heartbeatVSM =
        (void *) heartbeatVSMMeta->payload;

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        // XXX Update these fields correctly.
        sendHeartbeat(lastWakeTime);
        sendHVCPackVoltage();
        sendBMSPackCurrent();

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
        canRXMeta, sizeof(canRXMeta) / sizeof(canRXMeta[0]),
        NULL,
        GPIOB, GPIO_PIN_5,     // CAN2 RX port/pin.
        GPIOB, GPIO_PIN_6      // CAN2 TX port/pin.
    );

    // CAN2 filters.
    // XXX Change these to whitelist the appropriate IDs.
    const cmr_canFilter_t canFilters[] = {
        {
            .isMask = false,
            .rxFIFO = CAN_RX_FIFO0,
            .ids = {
                CMR_CANID_HEARTBEAT_VSM,
                CMR_CANID_HEARTBEAT_VSM,
                CMR_CANID_HEARTBEAT_VSM,
                CMR_CANID_HEARTBEAT_VSM
            }
        }
    };
    cmr_canFilter(
        &can, canFilters, sizeof(canFilters) / sizeof(canFilters[0])
    );

    // Task initialization.
    cmr_taskInit(
        &canTX10Hz_task,
        "CAN TX 10Hz",
        canTX10Hz_priority,
        canTX10Hz,
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
void *getPayload(canRX_t rxMsg) {
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
            HVCHeartbeat.mode = CMR_CAN_HVC_MODE_IDLE;
            break;
        case CMR_CAN_HVC_STATE_STANDBY: // S2
            HVCHeartbeat.mode = CMR_CAN_HVC_MODE_IDLE;
            break;
        case CMR_CAN_HVC_STATE_DRIVE_PRECHARGE: // S3
            HVCHeartbeat.mode = CMR_CAN_HVC_MODE_START;
            break;
        case CMR_CAN_HVC_STATE_DRIVE_PRECHARGE_COMPLETE: // S4
            HVCHeartbeat.mode = CMR_CAN_HVC_MODE_START;
            break;
        case CMR_CAN_HVC_STATE_DRIVE: // S5
            HVCHeartbeat.mode = CMR_CAN_HVC_MODE_RUN;
            break;
        case CMR_CAN_HVC_STATE_CHARGE_PRECHARGE: // S6
            HVCHeartbeat.mode = CMR_CAN_HVC_MODE_CHARGE;
            break;
        case CMR_CAN_HVC_STATE_CHARGE_PRECHARGE_COMPLETE: // S7
            HVCHeartbeat.mode = CMR_CAN_HVC_MODE_CHARGE;
            break;
        case CMR_CAN_HVC_STATE_CHARGE_TRICKLE: // S8
            HVCHeartbeat.mode = CMR_CAN_HVC_MODE_CHARGE;
            break;
        case CMR_CAN_HVC_STATE_CHARGE_CONSTANT_CURRENT: // S9
            HVCHeartbeat.mode = CMR_CAN_HVC_MODE_CHARGE;
            break;
        case CMR_CAN_HVC_STATE_CHARGE_CONSTANT_VOLTAGE: // S10
            HVCHeartbeat.mode = CMR_CAN_HVC_MODE_CHARGE;
            break;
        case CMR_CAN_HVC_STATE_ERROR: // S0
            HVCHeartbeat.mode = CMR_CAN_HVC_MODE_ERROR;
            break;
        case CMR_CAN_HVC_STATE_CLEAR_ERROR: // S11
            HVCHeartbeat.mode = CMR_CAN_HVC_MODE_ERROR;
            break;
        case CMR_CAN_HVC_STATE_UNKNOWN:
        default:
            HVCHeartbeat.mode = CMR_CAN_HVC_MODE_ERROR;
            break;
    }

    canTX(CMR_CANID_HEARTBEAT_HVC, &HVCHeartbeat, sizeof(HVCHeartbeat), canTX100Hz_period_ms);
}

/**
 * @brief Sets up Brusa Charger Control commands, then sends it.
 */
static void sendBRUSAChargerControl(void) {
    cmr_canBRUSAChargerControl_t BRUSAChargerControl = {
        .enableVector = 0,
        .maxMainsCurrent = 0,
        .requestedCurrent = 0,
        .requestedVoltage = 0
    };
    cmr_canHVCState_t currentState = getState();


    switch (currentState) {
        case CMR_CAN_HVC_STATE_DISCHARGE: // S1
			BRUSAChargerControl.enableVector = 0;
			BRUSAChargerControl.maxMainsCurrent = 0;
			BRUSAChargerControl.requestedCurrent = 0;
			BRUSAChargerControl.requestedVoltage = 0;
            break;
        case CMR_CAN_HVC_STATE_STANDBY: // S2
			BRUSAChargerControl.enableVector = 0;
			BRUSAChargerControl.maxMainsCurrent = 0;
			BRUSAChargerControl.requestedCurrent = 0;
			BRUSAChargerControl.requestedVoltage = 0;
            break;
        case CMR_CAN_HVC_STATE_DRIVE_PRECHARGE: // S3
            BRUSAChargerControl.enableVector = 0;
            BRUSAChargerControl.maxMainsCurrent = 0;
            BRUSAChargerControl.requestedCurrent = 0;
            BRUSAChargerControl.requestedVoltage = 0; 
            break;
        case CMR_CAN_HVC_STATE_DRIVE_PRECHARGE_COMPLETE: // S4
            BRUSAChargerControl.enableVector = 0;
            BRUSAChargerControl.maxMainsCurrent = 0;
            BRUSAChargerControl.requestedCurrent = 0;
            BRUSAChargerControl.requestedVoltage = 0;
            break;
        case CMR_CAN_HVC_STATE_DRIVE: // S5
            BRUSAChargerControl.enableVector = 0;
            BRUSAChargerControl.maxMainsCurrent = 0;
            BRUSAChargerControl.requestedCurrent = 0;
            BRUSAChargerControl.requestedVoltage = 0;
            break;
        case CMR_CAN_HVC_STATE_CHARGE_PRECHARGE: // S6
            // units for current and voltage are 1/10 Amps and 1/10 Volts
            BRUSAChargerControl.enableVector = 128;
            BRUSAChargerControl.maxMainsCurrent = 150; // 15 A
            BRUSAChargerControl.requestedCurrent = 10; // 1 A
            BRUSAChargerControl.requestedVoltage = 4500; // 450 V
            break;
        case CMR_CAN_HVC_STATE_CHARGE_PRECHARGE_COMPLETE: // S7
            // units for current and voltage are 1/10 Amps and 1/10 Volts
            BRUSAChargerControl.enableVector = 128;
            BRUSAChargerControl.maxMainsCurrent = 150; // 15 A
            BRUSAChargerControl.requestedCurrent = 10; // 1 A
            BRUSAChargerControl.requestedVoltage = 4500; // 450 V
            break;
        case CMR_CAN_HVC_STATE_CHARGE_TRICKLE: // S8
			// units for current and voltage are 1/10 Amps and 1/10 Volts
            BRUSAChargerControl.enableVector = 128;
            BRUSAChargerControl.maxMainsCurrent = 150; // 15 A
            BRUSAChargerControl.requestedCurrent = 10; // 1 A
            BRUSAChargerControl.requestedVoltage = 4500; // 450 V
            break;
        case CMR_CAN_HVC_STATE_CHARGE_CONSTANT_CURRENT: // S9
			// units for current and voltage are 1/10 Amps and 1/10 Volts
			BRUSAChargerControl.enableVector = 128;
			BRUSAChargerControl.maxMainsCurrent = 150; // 15 A
			BRUSAChargerControl.requestedCurrent = 70; // 7 A
			BRUSAChargerControl.requestedVoltage = 4500; // 450 V
            break;
        case CMR_CAN_HVC_STATE_CHARGE_CONSTANT_VOLTAGE: // S10
            BRUSAChargerControl.enableVector = 0;
            BRUSAChargerControl.maxMainsCurrent = 0;
            BRUSAChargerControl.requestedCurrent = 0;
            BRUSAChargerControl.requestedVoltage = 0;
            break;
        case CMR_CAN_HVC_STATE_ERROR: // S0
            BRUSAChargerControl.enableVector = 0;
            BRUSAChargerControl.maxMainsCurrent = 0;
            BRUSAChargerControl.requestedCurrent = 0;
            BRUSAChargerControl.requestedVoltage = 0;
            break;
        case CMR_CAN_HVC_STATE_CLEAR_ERROR: // S11
            BRUSAChargerControl.enableVector = 0;
            BRUSAChargerControl.maxMainsCurrent = 0;
            BRUSAChargerControl.requestedCurrent = 0;
            BRUSAChargerControl.requestedVoltage = 0;
            break;
        case CMR_CAN_HVC_STATE_UNKNOWN:
        default:			
            BRUSAChargerControl.enableVector = 0;
            BRUSAChargerControl.maxMainsCurrent = 0;
            BRUSAChargerControl.requestedCurrent = 0;
            BRUSAChargerControl.requestedVoltage = 0;
            break;
    }

    canTX(CMR_CANID_HVC_BRUSA_MSG, &BRUSAChargerControl, sizeof(BRUSAChargerControl), canTX10Hz_period_ms);
}

static void sendHVCPackVoltage(void) {
    int32_t bVolt = ((int32_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_BATT_PLUS)) - ((int32_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_BATT_MINUS));
    int32_t hvVolt = ((int32_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_HV_PLUS)) - ((int32_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_HV_MINUS));

    cmr_canHVCPackVoltage_t HVCPackVoltage = {
        .battVoltage = bVolt,
        .hvVoltage = hvVolt,
    };

    canTX(CMR_CANID_HVC_PACK_VOLTAGE, &HVCPackVoltage, sizeof(HVCPackVoltage), canTX100Hz_period_ms);
}

static void sendBMSPackCurrent(void) {
    // NEEDS CHANGES
    int32_t instantCurrent = 0;
    int32_t avgCurrent = 0;

    cmr_canBMSPackCurrent_t BMSPackCurrent = {
        .instantCurrent_mA = instantCurrent,
        .avgCurrent_mA = avgCurrent,
    };

    canTX(CMR_CANID_HVC_PACK_CURRENT, &BMSPackCurrent, sizeof(BMSPackCurrent), canTX100Hz_period_ms);
}

static void sendBMSBMBStatusVoltage(uint8_t bmb_index) {
    uint8_t maxIndex = getBMBMaxVoltIndex(bmb_index);
    uint8_t minIndex = getBMBMinVoltIndex(bmb_index);
    uint16_t maxVoltage = 

    cmr_canBMSBMBStatusVoltage_t BMSBMBStatusVoltage = {
        .maxVoltIndex = getBMBMaxVoltIndex(bmb_index);
        .minVoltIndex = 
    }
}