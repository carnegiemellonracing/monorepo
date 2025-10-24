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

#include "can.h"        // Interface to implement
#include "sensors.h"    // sensorChannel_t
#include "state.h"      // getCurrentErrors(), getCurrentWarnings()

/**
 * @brief CAN periodic message receive metadata
 *
 * @note Indexed by `canRX_t`.
 */
cmr_canRXMeta_t canRXMeta[] = {
    [CANRX_HEARTBEAT_HVC] = {
        .canID = CMR_CANID_HEARTBEAT_HVC,
        .timeoutError_ms = 20000,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .timeoutWarn_ms = 750,
        .warnFlag = CMR_CAN_WARN_VSM_HVC_TIMEOUT
    },
    [CANRX_HEARTBEAT_CDC] = {
        .canID = CMR_CANID_HEARTBEAT_CDC,
        .timeoutError_ms = 100,
        .errorFlag = CMR_CAN_ERROR_VSM_MODULE_TIMEOUT,
        .timeoutWarn_ms = 25,
        .warnFlag = CMR_CAN_WARN_VSM_CDC_TIMEOUT
    },
    [CANRX_HEARTBEAT_DIM] = {
        .canID = CMR_CANID_HEARTBEAT_DIM,
        .timeoutError_ms = 100,
        .errorFlag = CMR_CAN_ERROR_VSM_MODULE_TIMEOUT,
        .timeoutWarn_ms = 25,
        .warnFlag = CMR_CAN_WARN_VSM_DIM_TIMEOUT
    },
    [CANRX_HEARTBEAT_HVI] = {
        .canID = CMR_CANID_HEARTBEAT_HVI,
        .timeoutError_ms = 100,
        .errorFlag = CMR_CAN_ERROR_VSM_MODULE_STATE,
        .timeoutWarn_ms = 25,
        .warnFlag = CMR_CAN_WARN_VSM_HVI_TIMEOUT
    },
    [CANRX_FSM_DATA] = {
        .canID = CMR_CANID_FSM_DATA,
        .timeoutError_ms = 100,
        .errorFlag = CMR_CAN_ERROR_VSM_MODULE_TIMEOUT,
        .timeoutWarn_ms = 25,
        .warnFlag = CMR_CAN_WARN_VSM_DIM_TIMEOUT
    },
    // [CANRX_FSM_SWANGLE] = {
    //     .canID = CMR_CANID_FSM_SWANGLE,
    //     .timeoutError_ms = 100,
    //     .errorFlag = CMR_CAN_ERROR_VSM_MODULE_TIMEOUT,
    //     .timeoutWarn_ms = 25,
    //     .warnFlag = CMR_CAN_WARN_VSM_DIM_TIMEOUT
    // },
    [CANRX_DIM_REQUEST] = {
        .canID = CMR_CANID_DIM_REQUEST,
        .timeoutError_ms = 500,
        .timeoutWarn_ms = 250,
    },
    [CANRX_INVERTER_1] = {
        .canID = CMR_CANID_AMK_FL_ACT_1,
        .timeoutError_ms = 500,
        .timeoutWarn_ms = 250,
    },
    [CANRX_INVERTER_2] = {
        .canID = CMR_CANID_AMK_FR_ACT_1,
        .timeoutError_ms = 500,
        .timeoutWarn_ms = 250,
    },
    [CANRX_INVERTER_3] = {
        .canID = CMR_CANID_AMK_RL_ACT_1,
        .timeoutError_ms = 500,
        .timeoutWarn_ms = 250,
    },
    [CANRX_INVERTER_4] = {
        .canID = CMR_CANID_AMK_RR_ACT_1,
        .timeoutError_ms = 500,
        .timeoutWarn_ms = 250,
    }
};

/**
 * @brief Array of flags to set in timeoutMatrix and badStateMatrix of cmr_canVSMErrors_t.
 *
 * @details This matrix must be kept up to date with the above canRX meta definitions! Inverters are
 *          labeled as no source because their timeout is handled elsewhere.
 *
 * @note Indexed by `canRX_t`.
 */
const cmr_canVSMErrorSource_t vsmErrorSourceFlags[] = {
    [CANRX_HEARTBEAT_HVC]       = CMR_CAN_VSM_ERROR_SOURCE_NONE,
    [CANRX_HEARTBEAT_CDC]       = CMR_CAN_VSM_ERROR_SOURCE_CDC,
    [CANRX_HEARTBEAT_DIM]       = CMR_CAN_VSM_ERROR_SOURCE_DIM,
    [CANRX_FSM_DATA]            = CMR_CAN_VSM_ERROR_SOURCE_DIM,
    [CANRX_DIM_REQUEST]         = CMR_CAN_VSM_ERROR_SOURCE_NONE, // Don't timeout based on DIM requests
    [CANRX_INVERTER_1]          = CMR_CAN_VSM_ERROR_SOURCE_NONE, // Don't timeout inverter here
    [CANRX_INVERTER_2]          = CMR_CAN_VSM_ERROR_SOURCE_NONE,
    [CANRX_INVERTER_3]          = CMR_CAN_VSM_ERROR_SOURCE_NONE,
    [CANRX_INVERTER_4]          = CMR_CAN_VSM_ERROR_SOURCE_NONE,
};

/** @brief CAN 10 Hz TX priority. */
static const uint32_t canTX10Hz_priority = 3;
/** @brief CAN 10 Hz TX period (milliseconds). */
static const TickType_t canTX10Hz_period_ms = 100;

/** @brief CAN 100 Hz TX priority. */
static const uint32_t canTX100Hz_priority = 5;
/** @brief CAN 100 Hz TX period (milliseconds). */
static const TickType_t canTX100Hz_period_ms = 10;

/** @brief CAN 200 Hz TX priority. */
static const uint32_t canTX200Hz_priority = 5;
/** @brief CAN 200 Hz TX period (milliseconds). */
static const TickType_t canTX200Hz_period_ms = 5;

/** @brief CAN latched status TX priority. */
static const uint32_t canTXLatchedStatus_priority = 1;
/** @brief CAN latched status TX period (milliseconds). */
static const TickType_t canTXLatchedStatus_period_ms = 10000;

/** @brief CAN 10 Hz TX task. */
static cmr_task_t canTX10Hz_task;
/** @brief CAN 100 Hz TX task. */
static cmr_task_t canTX100Hz_task;
/** @brief CAN 200 Hz TX task. */
static cmr_task_t canTX200Hz_task;
/** @brief CAN latched status TX task. */
static cmr_task_t canTXLatchedStatus_task;

/** @brief Primary CAN interface. */
static cmr_can_t can;

// Forward declarations
static void sendHeartbeat(TickType_t lastWakeTime);
static void sendVSMStatus(void);
static void sendVSMSensors(void);
static void sendVSMLatchedStatus(void);
static void sendHVCCommand(void);
static void sendPowerDiagnostics(void);

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
        sendPowerDiagnostics();

        vTaskDelayUntil(&lastWakeTime, canTX10Hz_period_ms);
    }
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
        sendVSMStatus();
        sendHVCCommand();
        sendVSMSensors();

        vTaskDelayUntil(&lastWakeTime, canTX100Hz_period_ms);
    }
}

/**
 * @brief Task for sending CAN messages at 200 Hz.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTX200Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        // sendVSMSensors();

        vTaskDelayUntil(&lastWakeTime, canTX200Hz_period_ms);
    }
}

/**
 * @brief Task for sending latched status.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTXLatchedStatus(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        sendVSMLatchedStatus();

        vTaskDelayUntil(&lastWakeTime, canTXLatchedStatus_period_ms);
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

    // CAN2 filters, balanced across each RX FIFO
    // by expected reception frequency.
    const cmr_canFilter_t canFilters[] = {
        // ----------------------------------------------------------------------------------------
        // RX FIFO 0

        { // 4 messages at 100 Hz
            .isMask = true,
            .rxFIFO = CAN_RX_FIFO0,
            .ids = {
                0x100, // (msg_id & 0x7FC) == (0x100 & 0x7FC) matches 0x100, 0x101, 0x102, 0x103
                0x100,
                0x7FC, // upper 9 bits must match
                0x7FC
            }
        },

        { // 1 message at 100 Hz, 2 messages at 10 Hz
            .isMask = false,
            .rxFIFO = CAN_RX_FIFO0,
            .ids = {
                CMR_CANID_FSM_DATA,
                CMR_CANID_FSM_SWANGLE,
                CMR_CANID_DIM_REQUEST,
                CMR_CANID_AMK_FL_ACT_1,
                CMR_CANID_AMK_FR_ACT_1
            }
        },

        // ----------------------------------------------------------------------------------------
        // RX FIFO 1

        { // 4 messages at 100 Hz
            .isMask = true,
            .rxFIFO = CAN_RX_FIFO1,
            .ids = {
                0x104, // (msg_id & 0x7FC) == (0x104 & 0x7FC) matches 0x104, 0x105, 0x106, 0x107
                0x104,
                0x7FC, // upper 9 bits must match
                0x7FC
            }
        },
        {
            .isMask = false,
            .rxFIFO = CAN_RX_FIFO1,
            .ids = {
                CMR_CANID_AMK_RL_ACT_1,
                CMR_CANID_AMK_RR_ACT_1,
                CMR_CANID_AMK_RL_ACT_1,
                CMR_CANID_AMK_RR_ACT_1
            }
        },
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
    cmr_taskInit(
        &canTX200Hz_task,
        "CAN TX 200Hz",
        canTX200Hz_priority,
        canTX200Hz,
        NULL
    );
    cmr_taskInit(
        &canTXLatchedStatus_task,
        "CAN TX latched status",
        canTXLatchedStatus_priority,
        canTXLatchedStatus,
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
 * @brief Sends an Extended CAN message with the given ID.
 *
 * @param bus The CAN bus to transmit over.
 * @param id The Extended ID for the message.
 * @param data The data to send.
 * @param len The data's length, in bytes.
 * @param timeout The timeout, in ticks.
 *
 * @return 0 on success, or a negative error code on timeout.
 */
int canExtendedTX(cmr_canBusID_t bus, cmr_canExtendedID_t id, const void *data, size_t len, TickType_t timeout) {
    configASSERT(bus < CMR_CAN_BUS_NUM);

    return cmr_canExtendedTX(&(can[bus]), id, data, len, timeout);
}

/**
 * @brief Gets a pointer to the payload of a received CAN message.
 *
 * @param rxMsg The message to get the payload of.
 *
 * @return Pointer to payload, or NULL if rxMsg is invalid.
 */
void *getPayload(canRX_t rxMsg) {
    if (rxMsg >= CANRX_LEN) {
        return NULL; // TODO switch to configassert
    }

    cmr_canRXMeta_t *rxMeta = &(canRXMeta[rxMsg]);

    return (void *)(&rxMeta->payload);
}

/**
 * @brief Gets the state from the heartbeat of a module.
 *
 * @param module The module to get the state of. Must be a value of `CANRX_HEARTBEAT_XXX`
 * from canRX_t in can.h, except for CANRX_HEARTBEAT_HVC.
 *
 * @warning Using a non-heartbeat value of canRX_t will result in an undefined value.
 *
 * @return State of the module when valid, otherwise CMR_CAN_STATE_UNKNOWN.
 */
cmr_canState_t getModuleState(canRX_t module) {
    if ((module >= CANRX_LEN) || (module == CANRX_HEARTBEAT_HVC)) {
        return CMR_CAN_UNKNOWN; // TODO switch to configassert
    }

    cmr_canHeartbeat_t *heartbeat = getPayload(module);
    return (cmr_canState_t)(heartbeat->state);
}

/**
 * @brief Sets up VSM CAN heartbeat with current errors and warnings, then sends it.
 *
 * @param lastWakeTime Pass in from canTX100Hz. Used to update lastStateChangeTime and errors/warnings.
 */
static void sendHeartbeat(TickType_t lastWakeTime) {
    updateErrorsAndWarnings(lastWakeTime);

    cmr_canVSMState_t vsmState = getCurrentState();
    const vsmStatus_t *vsmStatus = getCurrentStatus();
    uint16_t vsmWarnings = getCurrentWarnings();
    cmr_canHeartbeat_t heartbeat;

    heartbeat.state = vsmToCANState[vsmState];

    if (vsmStatus->heartbeatErrors != CMR_CAN_ERROR_NONE) {
        heartbeat.state = CMR_CAN_ERROR;
    }

    // Copy to heartbeat and send
    memcpy(&heartbeat.error, &(vsmStatus->heartbeatErrors), sizeof(heartbeat.error));
    memcpy(&heartbeat.warning, &vsmWarnings, sizeof(heartbeat.warning));

    canTX(CMR_CANID_HEARTBEAT_VSM, &heartbeat, sizeof(heartbeat), canTX100Hz_period_ms);
}

/**
 * @brief Reflect current state onto the LV bus.
 *
 */
static void sendVSMStatus(void) {
    const vsmStatus_t *vsmStatus = getCurrentStatus();

    canTX(CMR_CANID_VSM_STATUS,
          &(vsmStatus->canVSMStatus),
          sizeof(vsmStatus->canVSMStatus),
          canTX100Hz_period_ms
    );
}

/**
 * @brief Reflect sensor values onto the LV bus.
 * TODO: add safety circuit voltage information.
 *
 */
static void sendVSMSensors(void) {
    cmr_canVSMSensors_t msg = {
        .brakePressureRear_PSI = cmr_sensorListGetValue(&sensorList, SENSOR_CH_BPRES_PSI),
        .hallEffect_cA = cmr_sensorListGetValue(&sensorList, SENSOR_CH_HALL_EFFECT_CA),
        .safetyIn_dV = 0,   // TODO
        .safetyOut_dV = 0   // TODO
    };

    canTX(CMR_CANID_VSM_SENSORS, &msg, sizeof(msg), canTX200Hz_period_ms);
}

/**
 * @brief Reflect list of any and all errors seen during uptime
 * onto LV bus.
 *
 */
static void sendVSMLatchedStatus(void) {
    const vsmStatus_t *vsmStatus = getCurrentStatus();

    canTX(CMR_CANID_VSM_LATCHED_STATUS,
          &(vsmStatus->canVSMLatchedStatus),
          sizeof(vsmStatus->canVSMLatchedStatus),
          canTXLatchedStatus_period_ms
    );
}

/**
 * @brief Update HVC with current requested state.
 *
 */
static void sendHVCCommand(void) {
    cmr_canHVCCommand_t hvcCommand = {
        .modeRequest = hvcModeRequest
    };

    canTX(CMR_CANID_HVC_COMMAND, &hvcCommand, sizeof(hvcCommand), canTX100Hz_period_ms);
}

/**
 * @brief Sends latest bus voltage and current draw measurements.
 */
static void sendPowerDiagnostics(void) {
    uint32_t busVoltage_mV =
        cmr_sensorListGetValue(&sensorList, SENSOR_CH_VOLTAGE_MV);
    uint32_t busCurrent_mA =
        cmr_sensorListGetValue(&sensorList, SENSOR_CH_CURRENT_MA);

    cmr_canVSMPowerDiagnostics_t msg = {
        .busVoltage_mV = busVoltage_mV,
        .busCurrent_mA = busCurrent_mA
    };

    canTX(CMR_CANID_VSM_POWER_DIAGNOSTICS, &msg, sizeof(msg), canTX10Hz_period_ms);
}
