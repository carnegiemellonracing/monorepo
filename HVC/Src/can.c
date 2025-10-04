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

/** @brief Slope for voltage sense transfer function */
#define V_TRANS_M 19.506
/** @brief Intercept for voltage sense transfer function */
#define V_TRANS_B -8313.3


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
        .canID = CMR_CANID_EMD_MEASUREMENT_RETX,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
	[CANRX_HVI_COMMAND] = {
		.canID = CMR_CANID_HV_SENSORS,
		.timeoutError_ms = 50,
		.timeoutWarn_ms = 25
    },
    [CANRX_BMSM_DATA] = {
        .canID = CMR_CANID_HVC_MIN_MAX_CELL_VOLTAGE,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    }
	// [CANRX_BALANCE_COMMAND] = {
	// 	.canID = CMR_CANID_CELL_BALANCE_ENABLE,
	// 	.timeoutError_ms = 50,
	// 	.timeoutWarn_ms = 25
	// }
};

/** @brief Primary CAN interface. */
static cmr_can_t can;

// Forward declarations
static void sendHeartbeat(TickType_t lastWakeTime);
static void sendHVCPower(); 
static void sendBMSLowVoltage(void);

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

        sendHVCPower();

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
        //sendBMSMinMaxCellVoltage();

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
        sendBMSLowVoltage();

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

/**
 * @brief Sets up HVC CAN heartbeat with current errors and warnings, then sends it.
 *
 * @param lastWakeTime Pass in from canTX100Hz. Used to update lastStateChangeTime and errors/warnings.
 */
static void sendHeartbeat(TickType_t lastWakeTime) {
    cmr_canHVCState_t currentState = getState();
    cmr_canHVCError_t currentError = CMR_CAN_HVC_ERROR_NONE;
    currentError = checkHVCErrors(currentState);

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

/** @brief calc that power bitch */
static void sendHVCPower() {
	int32_t power;
    int16_t voltage;
    uint16_t current;

    voltage = cmr_sensorListGetValue(&sensorList, SENSOR_CH_HV);
    current = cmr_sensorListGetValue(&sensorList, SENSOR_CH_CURRENT);
    power = (voltage * 100) * (current * 10);

    cmr_canHVSense_t *hv_sensors; 

    hv_sensors->packVoltage_cV = voltage;
    hv_sensors->packCurrent_dA = current;
    hv_sensors->packPower_W = power;   

    uint16_t voltageRaw, currentRaw;
	voltageRaw = adcRead(ADC_VSENSE);
	currentRaw = adcRead(ADC_ISENSE);

    canTX(CMR_CANID_HV_SENSORS, &hv_sensors, sizeof(hv_sensors), canTX10Hz_period_ms);
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

