/**
 * @file can.c
 * @brief Board-specific CAN implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <FreeRTOS.h>       // FreeRTOS interface
#include <task.h>           // xTaskCreate()

#include <CMR/can.h>    // CAN interface

#include "can.h"    // Interface to implement
#include "adc.h"    // adcVSense

/** @brief Primary CAN interface. */
static cmr_can_t can;

/** @brief CAN TX priority. */
static const uint32_t canTXPriority  = 5;

/** @brief CAN RX priority. */
static const uint32_t canRXPriority  = 7;

/** @brief CAN TX period (milliseconds). */
static const TickType_t canTXPeriod_ms = 10;

/** @brief CAN RX period (milliseconds). */
static const TickType_t canRXPeriod_ms = 1;

/**
 * @brief Task for sending CAN messages.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTXTask(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    // TODO The HAL internals of a CAN transmission really should be
    // encapsulated in the CMR CAN wrapper library/driver.

    CAN_HandleTypeDef *canHandle = cmr_canHandle(&can);

    // XXX Temporary header and data.
    CAN_TxHeaderTypeDef cdcHeartbeatHeader = {
        .StdId = 0x201,
        .ExtId = 0,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 3,
        .TransmitGlobalTime = DISABLE
    };

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        uint32_t txMailbox;
        uint8_t cdcHeartbeat[3] = {
            1,  // TODO junk
            // TODO Overflow; we will go to reporting mV in a uint16_t soon.
            (uint8_t) (adcVSense->value * 15 / 113),
            3   // TODO junk
        };
        HAL_CAN_AddTxMessage(
            canHandle, &cdcHeartbeatHeader, cdcHeartbeat, &txMailbox
        );

        vTaskDelayUntil(&lastWakeTime, canTXPeriod_ms);
    }
}

/**
 * @brief Task for receiving CAN messages (polling RX FIFOs).
 *
 * TODO The CAN receive task should be part of the CMR CAN library, and
 * receiving should populate CAN structs automatically.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canRXTask(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    CAN_HandleTypeDef *canHandle = cmr_canHandle(&can);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        CAN_RxHeaderTypeDef rxHeader = { 0 };
        uint8_t rxData[8];

        // Receive on FIFO 0.
        while (HAL_CAN_GetRxFifoFillLevel(canHandle, CAN_RX_FIFO0) > 0) {
            HAL_CAN_GetRxMessage(canHandle, CAN_RX_FIFO0, &rxHeader, rxData);
            // TODO Copy from rxData to struct based on ID
        }

        // Receive on FIFO 1.
        while (HAL_CAN_GetRxFifoFillLevel(canHandle, CAN_RX_FIFO1) > 0) {
            HAL_CAN_GetRxMessage(canHandle, CAN_RX_FIFO1, &rxHeader, rxData);
            // TODO Copy from rxData to struct based on ID
        }

        vTaskDelayUntil(&lastWakeTime, canRXPeriod_ms);
    }
}

/**
 * @brief Initializes the CAN interface.
 */
void canInit(void) {
    // CAN2 initialization.
    cmr_canInit(
        &can, CAN2,
        GPIOB, GPIO_PIN_12,     // CAN2 RX port/pin.
        GPIOB, GPIO_PIN_13      // CAN2 TX port/pin.
    );

    // CAN2 filters.
    const cmr_canFilter_t canFilters[] = {
        {
            .rxFIFO = CAN_RX_FIFO0,
            .ids = { 0x281, 0x281, 0x281, 0x281 }   // TODO: use ID constants
        }, {
            .rxFIFO = CAN_RX_FIFO1,
            .ids = { 0x4A7, 0x4A7, 0x4A7, 0x4A7 }   // TODO: use ID constants
        }
    };
    cmr_canFilter(
        &can, canFilters, sizeof(canFilters) / sizeof(canFilters[0])
    );

    // Task creation.
    xTaskCreate(
        canTXTask, "canTX",
        configMINIMAL_STACK_SIZE, NULL, canTXPriority, NULL
    );
    xTaskCreate(
        canRXTask, "canRX",
        configMINIMAL_STACK_SIZE, NULL, canRXPriority, NULL
    );
}

