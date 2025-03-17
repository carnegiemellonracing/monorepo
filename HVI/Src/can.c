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
#include <CMR/can.h>
#include <CMR/can_types.h>

#include <stm32f4xx_hal.h>  // HAL interface

#include "can.h"      // Interface to implement
#include "adc.h"      // adcVSense, adcISense
#include "sensors.h"
#include "CMR/sensors.h"

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
    [CANRX_EMD_MEASURE] = {
        .canID = CMR_CANID_EMD_MEASUREMENT,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 25
    },
    [CANRX_EMD_MEASURE] = {   
        .canID = CMR_CANID_EMD_MEASUREMENT_RETX,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    }
};

/** @brief Primary CAN interface. */
static cmr_can_t can;

// Forward declarations
static void sendHeartbeat(TickType_t lastWakeTime);

/** @brief CAN 1 Hz TX priority. */
static const uint32_t canTX1Hz_priority = 3;

/** @brief CAN 10 Hz TX period (milliseconds). */
static const TickType_t canTX1Hz_period_ms = 1000;

/** @brief CAN 10 Hz TX task. */
static cmr_task_t canTX1Hz_task;

/** @brief calc that power bitch */
static void calcPower(cmr_canHVIHeartbeat_t *heartbeat) {
	int32_t power;
    int16_t voltage;
    uint16_t current;

    voltage = cmr_sensorListGetValue(&sensorList, SENSOR_CH_HV);
    current = cmr_sensorListGetValue(&sensorList, SENSOR_CH_CURRENT);
    power = (voltage * 100) * (current * 10);

    heartbeat->packVoltage_cV = voltage;
    heartbeat->packCurrent_dA = current;
    heartbeat->packPower_W = power;   
}

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
static const uint32_t canTX10Hz_priority = 3;

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
    cmr_canHVIHeartbeat_t heartbeat;
    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
    	calcPower(&heartbeat);
    	canTX(CMR_CANID_HEARTBEAT_HVI, &heartbeat, sizeof(heartbeat), canTX10Hz_period_ms);
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
        vTaskDelayUntil(&lastWakeTime, canTX100Hz_period_ms);
    }
}

/**
 * @brief Initializes the CAN interface.
 */
void canInit(void) {
    // CAN1 initialization.
    cmr_canInit(
        &can, CAN1,
		CMR_CAN_BITRATE_500K,
        canRXMeta, sizeof(canRXMeta) / sizeof(canRXMeta[0]),
        NULL,
        GPIOA, GPIO_PIN_11,     // CAN2 RX port/pin.
        GPIOA, GPIO_PIN_12      // CAN2 TX port/pin.
    );

    // CAN1 filters.
    const cmr_canFilter_t canFilters[] = {
        {
            .isMask = false,
            .rxFIFO = CAN_RX_FIFO0,
            .ids = {
                CMR_CANID_HEARTBEAT_VSM,
                CMR_CANID_HEARTBEAT_VSM,
				CMR_CANID_EMD_MEASUREMENT
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



