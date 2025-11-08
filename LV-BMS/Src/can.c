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

#include "can.h"
#include "data.h"
// INTERFACES


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
    }
};

cmr_canHeartbeat_t heartbeat;

/** @brief CAN 10 Hz TX priority. */
const uint32_t canTX10Hz_priority = 3;
/** @brief CAN 10 Hz TX period (milliseconds). */
const TickType_t canTX10Hz_period_ms = 100;

/** @brief CAN 100 Hz TX priority. */
const uint32_t canTX100Hz_priority = 5;
/** @brief CAN 100 Hz TX period (milliseconds). */
const TickType_t canTX100Hz_period_ms = 10;

/** @brief CAN 1 Hz TX priority. */
const uint32_t canTX1Hz_priority = 6;
/** @brief CAN 1 Hz TX period (milliseconds). */
const TickType_t canTX1Hz_period_ms = 1000;

/** @brief CAN 10 Hz TX task. */
static cmr_task_t canTX10Hz_task;
/** @brief CAN 100 Hz TX task. */
static cmr_task_t canTX100Hz_task;
/** @brief CAN 1 Hz TX task. */
static cmr_task_t canTX1Hz_task;

/** @brief Primary CAN interface. */
static cmr_can_t can;

// Forward declarations 

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

/**
 * @brief Task for sending CAN messages at 1 Hz.
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
        // Loop through the 4 different MUX channels and select a different one
		// We still monitor all voltages each channel switch
		for(uint8_t j = 0; j < 4; j++) {
			// Small delays put between all transaction

			setMuxOutput(j);
			lastWakeTime = xTaskGetTickCount();
			vTaskDelayUntil(&lastWakeTime, 10);

			uint8_t err = getVoltages();

			lastWakeTime = xTaskGetTickCount();
			vTaskDelayUntil(&lastWakeTime, 10);

			getTemps(j);

			lastWakeTime = xTaskGetTickCount();
			vTaskDelayUntil(&lastWakeTime, 10);

			lastWakeTime = xTaskGetTickCount();
			vTaskDelayUntil(&lastWakeTime, 100);
		}
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
        GPIOA, GPIO_PIN_11,     // CAN1 RX port/pin.
        GPIOA, GPIO_PIN_12      // CAN1 TX port/pin.
    );

    // CAN2 filters.
const cmr_canFilter_t canFilters[] = {
        {
            .isMask = false,
            .rxFIFO = CAN_RX_FIFO0,
            .ids = {
                CMR_CANID_HEARTBEAT_VSM
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
