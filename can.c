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
#include "sensors.h"    // Sensors interface.

/**
 * @brief CAN periodic message receive metadata
 *
 * @note Indexed by `canRX_t`.
 */
cmr_canRXMeta_t canRXMeta[] = {
    [CANRX_HEARTBEAT_VSM] = {
        .canID = CMR_CANID_HEARTBEAT_VSM,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 25,
        .errorFlag = CMR_CAN_ERROR_VSM_TIMEOUT,
        .warnFlag = CMR_CAN_WARN_VSM_TIMEOUT
    },
    [CANRX_VSM_STATUS] = {
        .canID = CMR_CANID_VSM_STATUS,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25,
        .errorFlag = CMR_CAN_ERROR_VSM_TIMEOUT,
        .warnFlag = CMR_CAN_WARN_VSM_TIMEOUT,
    },
    [CANRX_INV1_STATUS] = {
        .canID = CMR_CANID_AMK_1_ACT_2,
        .timeoutError_ms = 800, // Send error if data not received within 4 cycles, or 800 ms
        .timeoutWarn_ms = 400, // Send warning if data not received within 2 cycles, or 400 ms
        // CAN transmitting frequency = 5 Hz, so ? s = 1 / 5 Hz = 0.2 s = 200ms
    },
    [CANRX_INV2_STATUS] = {
        .canID = CMR_CANID_AMK_2_ACT_2,
        .timeoutError_ms = 800,
        .timeoutWarn_ms = 400,
    },
    [CANRX_INV3_STATUS] = {
        .canID = CMR_CANID_AMK_3_ACT_2,
        .timeoutError_ms = 800,
        .timeoutWarn_ms = 400,
    },
    [CANRX_INV4_STATUS] = {
        .canID = CMR_CANID_AMK_4_ACT_2,
        .timeoutError_ms = 800,
        .timeoutWarn_ms = 400,
    },
    [CANRX_VSM_SENSORS] = {
        .canID = CMR_CANID_VSM_SENSORS,
        .timeoutError_ms = 500,
        .timeoutWarn_ms = 250,
        .errorFlag = CMR_CAN_ERROR_VSM_TIMEOUT,
        .warnFlag = CMR_CAN_WARN_VSM_TIMEOUT
    },
    [CANRX_FSM_DATA] = {
        .canID = CMR_CANID_FSM_DATA,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 50,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
    },
    [CANRX_HVC_MINMAX_TEMPS] = {
        .canID = CMR_CANID_HVC_MINMAX_CELL_TEMPS,
        .timeoutError_ms = 5000,
        .timeoutWarn_ms = 2500,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
    }
};

cmr_canHeartbeat_t heartbeat;

/** @brief Fan/Pump channel states. */
uint16_t fan_1_State;
uint16_t fan_2_State;
uint16_t pump_1_State;
uint16_t pump_2_State;

/** @brief CAN 10 Hz TX priority. */
static const uint32_t canTX10Hz_priority = 3;
/** @brief CAN 10 Hz TX period (milliseconds). */
static const TickType_t canTX10Hz_period_ms = 100;

/** @brief CAN 100 Hz TX priority. */
static const uint32_t canTX100Hz_priority = 5;
/** @brief CAN 100 Hz TX period (milliseconds). */
static const TickType_t canTX100Hz_period_ms = 10;

/** @brief CAN 10 Hz TX task. */
static cmr_task_t canTX10Hz_task;
/** @brief CAN 100 Hz TX task. */
static cmr_task_t canTX100Hz_task;

/** @brief Primary CAN interface. */
static cmr_can_t can;

// Forward declarations
static void sendPowerDiagnostics(void);
static void sendDriverStatus(void);
static void sendHeartbeat(TickType_t lastWakeTime);

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
        sendDriverStatus();

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
                CMR_CANID_HEARTBEAT_VSM,
                CMR_CANID_HEARTBEAT_VSM,
                CMR_CANID_FSM_DATA
            }
        },
        {
            .isMask = false,
            .rxFIFO = CAN_RX_FIFO1,
            .ids = {
                CMR_CANID_VSM_STATUS,
                CMR_CANID_VSM_STATUS,
                CMR_CANID_VSM_SENSORS,
                CMR_CANID_HVC_MINMAX_CELL_TEMPS
            }
        },
        {
            .isMask = false,
            .rxFIFO = CAN_RX_FIFO1,
            .ids = {
                CMR_CANID_AMK_1_ACT_2,
                CMR_CANID_AMK_2_ACT_2,
                CMR_CANID_AMK_3_ACT_2,
                CMR_CANID_AMK_4_ACT_2
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
void *canGetPayload(canRX_t rxMsg) {
    configASSERT(rxMsg < CANRX_LEN);

    cmr_canRXMeta_t *rxMeta = &(canRXMeta[rxMsg]);

    return (void *)(&rxMeta->payload);
}

/**
 * @brief Sets up PTC heartbeat, checks for errors, then sends it
 *
 * @param lastWakeTime Pass in from canTX100Hz. Used to determine VSM timeout.
 */
static void sendHeartbeat(TickType_t lastWakeTime) {
    cmr_canRXMeta_t *heartbeatVSMMeta = canRXMeta + CANRX_HEARTBEAT_VSM;
    volatile cmr_canHeartbeat_t *heartbeatVSM = canGetPayload(CANRX_HEARTBEAT_VSM);

    heartbeat.state = heartbeatVSM->state;

    uint16_t error = CMR_CAN_ERROR_NONE;

    //or the water overheating bit 

    if (cmr_canRXMetaTimeoutError(heartbeatVSMMeta, lastWakeTime) < 0) {
        error |= CMR_CAN_ERROR_VSM_TIMEOUT;
    }

    // If error exists, update heartbeat to error state (i.e. update its fields). See can_types.h for the fields.
    if (error != CMR_CAN_ERROR_NONE) {
        heartbeat.state = CMR_CAN_ERROR; //Field 1 update
    }
    memcpy(&heartbeat.error, &error, sizeof(error)); //Field 2 update

    uint16_t warning = CMR_CAN_WARN_NONE;
    if (cmr_canRXMetaTimeoutWarn(heartbeatVSMMeta, lastWakeTime) < 0) {
        warning |= CMR_CAN_WARN_VSM_TIMEOUT;
    }
    memcpy(&heartbeat.warning, &warning, sizeof(warning));

    canTX(CMR_CANID_HEARTBEAT_PTC, &heartbeat, sizeof(heartbeat), canTX100Hz_period_ms);
}

/**
 * @brief Send Fan or Pump status information.
 */
static void sendDriverStatus(void) {
    uint8_t Fan1_Duty_Cycle_pcnt = fan_1_State;
    uint8_t Fan2_Duty_Cycle_pcnt = fan_2_State;
    uint8_t Pump1_Duty_Cycle_pcnt = pump_1_State;
    uint8_t Pump2_Duty_Cycle_pcnt = pump_2_State;


    cmr_canPTCDriverStatus_t driverStatusMsg = {
        .fan1DutyCycle_pcnt = Fan1_Duty_Cycle_pcnt,
        .fan2DutyCycle_pcnt = Fan2_Duty_Cycle_pcnt,
        .pump1DutyCycle_pcnt = Pump1_Duty_Cycle_pcnt,
        .pump2DutyCycle_pcnt = Pump2_Duty_Cycle_pcnt
    };

    canTX(CMR_CANID_PTC_FANS_PUMPS_STATUS, &driverStatusMsg, sizeof(driverStatusMsg), canTX10Hz_period_ms);
}

/**
 * @brief Send power diagnostic information.
 */
static void sendPowerDiagnostics(void) {
    int32_t logicVoltage_mV =
        cmr_sensorListGetValue(&sensorList, SENSOR_CH_LOGIC_VOLTAGE_MV);
    int32_t loadVoltage_mV =
        cmr_sensorListGetValue(&sensorList, SENSOR_CH_LOAD_VOLTAGE_MV);
    int32_t loadCurrent_mA =
        cmr_sensorListGetValue(&sensorList, SENSOR_CH_LOAD_CURRENT_MA);

    cmr_canPTCPowerDiagnostics_t powerMsg = {
        .logicVoltage_mV = logicVoltage_mV,
        .loadVoltage_mV = loadVoltage_mV,
        .loadCurrent_mA = loadCurrent_mA
    };                                                                                                   

    canTX(CMR_CANID_PTC_POWER_DIAGNOSTICS, &powerMsg, sizeof(powerMsg), canTX10Hz_period_ms);
}

