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
#include "adc.h"        // adcVSense, adcISense
#include "sensors.h"    // Sensors interface

/**
 * @brief CAN periodic message receive metadata
 *
 * @note Indexed by `canRX_t`.
 */
cmr_canRXMeta_t canRXMeta[CANRX_LEN] = {
    [CANRX_HEARTBEAT_VSM] = {
        .canID = CMR_CANID_HEARTBEAT_VSM,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_VSM_SENSORS] = {
        .canID = CMR_CANID_VSM_SENSORS,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    }
};

/** @brief CAN 100 Hz TX priority. */
static const uint32_t canTX100Hz_priority = 5;
/** @brief CAN 100 Hz TX period (milliseconds). */
static const TickType_t canTX100Hz_period_ms = 10;

/** @brief CAN 10 Hz TX priority. */
static const uint32_t canTX10Hz_priority = 3;
/** @brief CAN 10 Hz TX period (milliseconds). */
static const TickType_t canTX10Hz_period_ms = 100;

/** @brief CAN 100 Hz TX task. */
static cmr_task_t canTX100Hz_task;
/** @brief CAN 10 Hz TX task. */
static cmr_task_t canTX10Hz_task;

/** @brief Primary CAN interface. */
static cmr_can_t can;

// Forward declarations
static void sendHeartbeat(TickType_t lastWakeTime);
static void sendFSMData(void);
static void sendFSMPedalsADC(void);
static void sendFSMSensorsADC(void);
static void sendPowerDiagnostics(void);

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
        sendFSMData();

        vTaskDelayUntil(&lastWakeTime, canTX100Hz_period_ms);
    }
}

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

        // TODO These should probably only be sent when in some "calibration"
        // mode that we enter upon receiving some "begin calibration" message
        // from PCAN Explorer
        sendFSMPedalsADC();
        sendFSMSensorsADC();

        vTaskDelayUntil(&lastWakeTime, canTX10Hz_period_ms);
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
                CMR_CANID_HEARTBEAT_VSM,
                CMR_CANID_HEARTBEAT_VSM,
                CMR_CANID_HEARTBEAT_VSM
            }
        },
        {
            .isMask = false,
            .rxFIFO = CAN_RX_FIFO1,
            .ids = {
                CMR_CANID_VSM_SENSORS,
                CMR_CANID_VSM_SENSORS,
                CMR_CANID_VSM_SENSORS,
                CMR_CANID_VSM_SENSORS
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
 * @param timeout_ms The timeout, in ms.
 *
 * @return 0 on success, or a negative error code on timeout.
 */
int canTX(cmr_canID_t id, const void *data, size_t len, TickType_t timeout_ms) {
    return cmr_canTX(&can, id, data, len, timeout_ms);
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
 * @brief Sets up FSM CAN heartbeat by checking errors and sends it.
 *
 * @param lastWakeTime Pass in from canTX100Hz. Used to determine pedal implausibility
 * according to FSAE rule T.6.2.3.
 */
static void sendHeartbeat(TickType_t lastWakeTime) {
    cmr_canRXMeta_t *heartbeatVSMMeta = &(canRXMeta[CANRX_HEARTBEAT_VSM]);
    volatile cmr_canHeartbeat_t *heartbeatVSM = (void *) heartbeatVSMMeta->payload;

    // Create heartbeat
    cmr_canHeartbeat_t heartbeat = {
        .state = heartbeatVSM->state
    };

    cmr_canWarn_t warning = CMR_CAN_WARN_NONE;
    cmr_canError_t error = CMR_CAN_ERROR_NONE;

    cmr_sensorListGetFlags(&sensorList, &warning, &error);

    if (cmr_canRXMetaTimeoutError(heartbeatVSMMeta, lastWakeTime) < 0) {
        error |= CMR_CAN_ERROR_VSM_TIMEOUT;
    }

    if (error != CMR_CAN_ERROR_NONE) {
        heartbeat.state = CMR_CAN_ERROR;
    }

    memcpy(&heartbeat.error, &error, sizeof(heartbeat.error));

    if (cmr_canRXMetaTimeoutWarn(heartbeatVSMMeta, lastWakeTime) < 0) {
        warning |= CMR_CAN_WARN_VSM_TIMEOUT;
    }

    memcpy(&heartbeat.warning, &warning, sizeof(heartbeat.warning));

    canTX(CMR_CANID_HEARTBEAT_FSM, &heartbeat, sizeof(heartbeat), canTX100Hz_period_ms);
}

/**
 * @brief Sends FSM data message.
 */
static void sendFSMData(void) {
    cmr_canRXMeta_t *heartbeatVSMMeta = &(canRXMeta[CANRX_HEARTBEAT_VSM]);
    volatile cmr_canHeartbeat_t *heartbeatVSM = (void *) heartbeatVSMMeta->payload;

    uint8_t throttlePosition = throttleGetPos();
    uint8_t torqueRequested = 0;

    if (heartbeatVSM->state == CMR_CAN_RTD &&
        cmr_sensorListGetValue(&sensorList, SENSOR_CH_BPP_IMPLAUS) == 0
    ) {
        torqueRequested = throttlePosition;
    }

    uint8_t brakePressureFront_PSI = (uint8_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_BPRES_PSI);
    int16_t steeringWheelAngle_deg = (int16_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_SWANGLE_DEG);
    uint8_t brakePedalPosition_pcnt = (uint8_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_BPOS_PCNT);

    cmr_canFSMData_t msg = {
        .torqueRequested         = torqueRequested,
        .throttlePosition        = throttlePosition,
        .brakePressureFront_PSI  = brakePressureFront_PSI,
        .brakePedalPosition_pcnt = brakePedalPosition_pcnt,
        .steeringWheelAngle_deg  = steeringWheelAngle_deg
    };

    canTX(CMR_CANID_FSM_DATA, &msg, sizeof(msg), canTX100Hz_period_ms);
}

/**
 * @brief Sends raw pedal position sensor ADC values.
 *
 * @note This is only useful for calibration and should not be sent constantly.
 */
static void sendFSMPedalsADC(void) {
    cmr_canFSMPedalsADC_t msg = {
        .throttleLeftADC  = adcRead(sensorsADCChannels[SENSOR_CH_TPOS_L]),
        .throttleRightADC = adcRead(sensorsADCChannels[SENSOR_CH_TPOS_R]),
        .brakePedalADC    = adcRead(sensorsADCChannels[SENSOR_CH_BPOS_PCNT])
    };

    canTX(CMR_CANID_FSM_PEDALS_ADC, &msg, sizeof(msg), canTX10Hz_period_ms);
}

/**
 * @brief Sends raw sensor ADC values.
 *
 * @note This is only useful for calibration and should not be sent constantly.
 */
static void sendFSMSensorsADC(void) {
    cmr_canFSMSensorsADC_t msg = {
        .brakePressureFrontADC = adcRead(sensorsADCChannels[SENSOR_CH_BPRES_PSI]),
        .steeringWheelAngleADC = adcRead(sensorsADCChannels[SENSOR_CH_SWANGLE_DEG])
    };

    canTX(CMR_CANID_FSM_SENSORS_ADC, &msg, sizeof(msg), canTX10Hz_period_ms);
}

/**
 * @brief Sends latest bus voltage and current draw measurements.
 */
static void sendPowerDiagnostics(void) {
    // value * 0.8 (mV per bit) * 11 (1:11 voltage divider)
    uint32_t busVoltage_mV = cmr_sensorListGetValue(&sensorList, SENSOR_CH_VOLTAGE_MV);
    uint32_t busCurrent_mA = cmr_sensorListGetValue(&sensorList, SENSOR_CH_AVG_CURRENT_MA);

    cmr_canFSMPowerDiagnostics_t msg = {
        .busVoltage_mV = busVoltage_mV,
        .busCurrent_mA = busCurrent_mA
    };

    canTX(CMR_CANID_FSM_POWER_DIAGNOSTICS, &msg, sizeof(msg), canTX10Hz_period_ms);
}
