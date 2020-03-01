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
#include "parser.h"     // parser ingestation

/**
 * @brief CAN periodic message receive metadata
 *
 * @note Indexed by `canRX_t`.
 */
cmr_canRXMeta_t canRXMeta[CANRX_LEN] = {
    [CANRX_HEARTBEAT_VSM] = {
        .canID = CMR_CANID_HEARTBEAT_VSM,
        .timeoutError_ms = -1,
        .timeoutWarn_ms = -1
    },
    [CANRX_VSM_SENSORS] = {
        .canID = CMR_CANID_VSM_STATUS,
        .timeoutError_ms = -1,
        .timeoutWarn_ms = -1
    },
    [CANRX_HVC_VOLTAGE] = {
        .canID = CMR_CANID_HVC_PACK_VOLTAGE,
        .timeoutError_ms = -1,
        .timeoutWarn_ms = -1
    },
    [CANRX_HVC_TEMPS] = {
        .canID = CMR_CANID_HVC_MINMAX_CELL_TEMPS,
        .timeoutError_ms = -1,
        .timeoutWarn_ms = -1
    },
    [CANRX_SBG_VELOCITY] = {
        .canID = 0,
        .timeoutError_ms = -1,
        .timeoutWarn_ms = -1
    },
    [CANRX_SBG_ACCELERATION] = {
        .canID = 0,
        .timeoutError_ms = -1,
        .timeoutWarn_ms = -1
    },
    [CANRX_SBG_GPS_POS] = {
        .canID = 0,
        .timeoutError_ms = -1,
        .timeoutWarn_ms = -1
    },
    [CANRX_PTCf_TEMPS] = {
        .canID = CMR_CANID_PTCf_LOOP_TEMPS_A,
        .timeoutError_ms = -1,
        .timeoutWarn_ms = -1
    },
    [CANRX_PTCp_TEMPS] = {
        .canID = CMR_CANID_PTCp_LOOP_TEMPS_A,
        .timeoutError_ms = -1,
        .timeoutWarn_ms = -1
    },
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
        vTaskDelayUntil(&lastWakeTime, canTX10Hz_period_ms);
    }
}

static void canRX(
    cmr_can_t *can, uint16_t canID, const void *data, size_t dataLen
) {
    int ret = parseData(canID, data, dataLen);
    configASSERT(ret);
    (void) ret;
}

/**
 * @brief Initializes the CAN interface.
 */
void canInit(void) {
    // CAN2 initialization.
    cmr_canInit(
        &can, CAN3,
        CMR_CAN_BITRATE_500K,
        NULL, 0,
        canRX,
        GPIOA, GPIO_PIN_8,     // CAN2 RX port/pin.
        GPIOB, GPIO_PIN_4      // CAN2 TX port/pin.
    );

    // CAN2 filters.
    const cmr_canFilter_t canFilters[] = {
        {
            .isMask = false,
            .rxFIFO = CAN_RX_FIFO0,
            .ids = {
                CMR_CANID_HEARTBEAT_VSM,
                CMR_CANID_VSM_STATUS,
                CMR_CANID_HVC_PACK_VOLTAGE,
                CMR_CANID_HVC_MINMAX_CELL_TEMPS
            }
        },
        {
            .isMask = false,
            .rxFIFO = CAN_RX_FIFO1,
            .ids = {
                CMR_CANID_PTCf_LOOP_TEMPS_A,
                CMR_CANID_PTCf_LOOP_TEMPS_A,
                CMR_CANID_PTCp_LOOP_TEMPS_A,
                CMR_CANID_PTCp_LOOP_TEMPS_A
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
