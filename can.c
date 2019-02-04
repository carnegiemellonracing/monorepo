/**
 * @file can.c
 * @brief Board-specific CAN implementation.
 *
 * Adding a new CAN RX struct:
 *
 * 1. Declare a `static` CAN struct for the data (suffix it _data).
 *      - The struct should likely have an initializer.
 * 2. Declare a `volatile const` pointer to that struct.
 * 3. Expose the pointer as an `extern volatile const` in `can.h`.
 *
 * @author Carnegie Mellon Racing
 */

#include <string.h>     // memcpy()

#include <FreeRTOS.h>   // FreeRTOS interface
#include <task.h>       // xTaskCreate()

#include <CMR/can.h>    // CAN interface

#include "can.h"    // Interface to implement
#include "adc.h"    // adcVSense, adcISense

/** @brief VSM heartbeat data. */
static cmr_canHeartbeat_t canHeartbeatVSM_data = {
    .state = CMR_CAN_UNKNOWN,
    .error = { 0 },
    .warning = { 0 }
};

/** @brief VSM heartbeat. */
volatile const cmr_canHeartbeat_t *canHeartbeatVSM = &canHeartbeatVSM_data;

/** @brief Primary CAN interface. */
static cmr_can_t can;

/** @brief CAN 10 Hz TX priority. */
static const uint32_t canTX10HzPriority = 3;

/** @brief CAN 10 Hz TX period (milliseconds). */
static const TickType_t canTX10HzPeriod_ms = 100;

/**
 * @brief Task for sending CAN messages at 10 Hz.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTX10HzTask(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        // XXX Replace with an appropriate message.
        cmr_canDIMPowerDiagnostics_t msg = {
            .busVoltage_mV = adcVSense->value,
            .busCurrent_mA = adcISense->value
        };

        // XXX Replace with an appropriate ID.
        canTX(CMR_CANID_DIM_POWER_DIAGNOSTICS, &msg, sizeof(msg));

        vTaskDelayUntil(&lastWakeTime, canTX10HzPeriod_ms);
    }
}

/** @brief CAN 100 Hz TX priority. */
static const uint32_t canTX100HzPriority = 5;

/** @brief CAN 100 Hz TX period (milliseconds). */
static const TickType_t canTX100HzPeriod_ms = 10;

/**
 * @brief Task for sending CAN messages at 100 Hz.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTX100HzTask(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        // XXX Update these fields correctly.
        cmr_canHeartbeat_t heartbeat = {
            .state = canHeartbeatVSM->state,
            .error = { 0 },
            .warning = { 0 }
        };

        // XXX Replace with an appropriate ID.
        canTX(CMR_CANID_HEARTBEAT_DIM, &heartbeat, sizeof(heartbeat));

        vTaskDelayUntil(&lastWakeTime, canTX100HzPeriod_ms);
    }
}

/**
 * @brief Initializes the CAN interface.
 */
void canInit(void) {
    // CAN2 initialization.
    cmr_canInit(
        &can, CAN2,
        "canRX",
        GPIOB, GPIO_PIN_12,     // CAN2 RX port/pin.
        GPIOB, GPIO_PIN_13      // CAN2 TX port/pin.
    );

    // CAN2 filters.
    // XXX Change these to whitelist the appropriate IDs.
    const cmr_canFilter_t canFilters[] = {
        {
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

    // Task creation.
    xTaskCreate(
        canTX10HzTask, "canTX10Hz",
        configMINIMAL_STACK_SIZE, NULL, canTX10HzPriority, NULL
    );
    xTaskCreate(
        canTX100HzTask, "canTX100Hz",
        configMINIMAL_STACK_SIZE, NULL, canTX100HzPriority, NULL
    );
}

/**
 * @brief Sends a CAN message with the given ID.
 *
 * @param id The ID for the message.
 * @param data The data to send.
 * @param len The data's length, in bytes.
 */
void canTX(cmr_canID_t id, const void *data, size_t len) {
    cmr_canTX(&can, id, data, len);
}

