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

#include "can.h"        // Interface to implement

/**
 * @brief CAN periodic message receive metadata
 *
 * @note Indexed by `canRX_t`.
 */
cmr_canRXMeta_t canRXMeta[] = {
    [CANRX_CUBEMARS_DATA] = {
        .canID = CMR_CANID_EXTENDED_CUBEMARS_DATA,
        .timeoutError_ms = 1000,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .timeoutWarn_ms = 750,
        .warnFlag = CMR_CAN_WARN_VSM_HVC_TIMEOUT
    }
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

        uint8_t x = 5;
        canTX(CMR_CANID_HEARTBEAT_VSM, &x, sizeof(x), canTX10Hz_period_ms);
        
        // cmr_canCubeMarsSpeedLoop_t set_speed = {
        //     .speed_erpm = 5000
        // };
        // canExtendedTX(CMR_CANID_EXTENDED_CUBEMARS_SET_RPM, (void*)&set_speed, sizeof(set_speed), canTX10Hz_period_ms);
        // vTaskDelayUntil(&lastWakeTime, 5000);
        // set_speed.speed_erpm = 0;
        // canExtendedTX(CMR_CANID_EXTENDED_CUBEMARS_SET_RPM, (void*)&set_speed, sizeof(set_speed), canTX10Hz_period_ms);
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
        uint8_t hi = 1;
        canTX(0x2, &hi, sizeof(hi), canTX200Hz_period_ms);
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

        vTaskDelayUntil(&lastWakeTime, canTXLatchedStatus_period_ms);
    }
}

int sendCubeMarsVelocity(int velocity){
    int newVelo = velocity;
    cmr_canCubeMarsSpeedLoop_t speed = {
        .speed_erpm = velocity
    };
    return canExtendedTX(CMR_CANID_EXTENDED_CUBEMARS_SET_RPM, &speed, sizeof(speed), canTX10Hz_period_ms);
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
        GPIOA, GPIO_PIN_12     // CAN2 TX port/pin.
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
int canExtendedTX(cmr_canExtendedID_t id, const void *data, size_t len, TickType_t timeout) {
    return cmr_canExtendedTX(&can, id, data, len, timeout);
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
