// ------------------------------------------------------------------------------------------------
// Includes

#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"

#include "can_tasks.h"
#include "adc.h"
#include "can.h"

// ------------------------------------------------------------------------------------------------
// Task priorities and periods

/**
 *      Priority    Freq (Hz)   Period (ms)
 *      -----------------------------------
 *      7           1000        1
 *      6           100-999     calculate
 *      5           100         10
 *      4           11-99       calculate
 *      3           10          100
 *      2           2-9         calculate
 *      1           1           1000
 */

// Globally accessible priorities
const uint32_t          canTX100HzTaskPriority  = 5;
const uint32_t          canTX10HzTaskPriority   = 3;
const uint32_t          canRXTaskPriority       = 7;

// Locally accessible periods
static const TickType_t canTX100HzTaskPeriod_ms = 10;
static const TickType_t canTX10HzTaskPeriod_ms  = 100;
static const TickType_t canRXTaskPeriod_ms      = 1;

// ------------------------------------------------------------------------------------------------
// Tasks

/**
 * @brief Sends CAN messages at 100 Hz or 10 ms
 *
 * @param pvParameters      Unused
 */
void canTX100Hz_task(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();

    CAN_TxHeaderTypeDef cdcHeartbeatHeader = { 0 };
    cdcHeartbeatHeader.StdId = 0x201;
    cdcHeartbeatHeader.IDE = CAN_ID_STD;
    cdcHeartbeatHeader.RTR = CAN_RTR_DATA;
    cdcHeartbeatHeader.DLC = 3;
    cdcHeartbeatHeader.TransmitGlobalTime = DISABLE;

    // Temporary junk data: HV_EN, 1.0 V, AutoX
    uint8_t cdcHeartbeat[3] = {1, 15, 3};
    uint32_t pTxMailbox;

    for (;;) {
        // Currently overflows, we will go to reporting mV in a uint16_t soon
        cdcHeartbeat[1] = (uint8_t)(adcVals[vSenseIndex] * 15 / 113);
        HAL_CAN_AddTxMessage(&can2Handle, &cdcHeartbeatHeader, cdcHeartbeat, &pTxMailbox);

        vTaskDelayUntil(&lastWakeTime, canTX100HzTaskPeriod_ms);
    }
}

/**
 * @brief Sends CAN messages at 10 Hz or 100 ms
 *
 * @param pvParameters      Unused
 */
void canTX10Hz_task(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();

    for (;;) {

        vTaskDelayUntil(&lastWakeTime, canTX10HzTaskPeriod_ms);
    }
}

/**
 * @brief   Receives CAN messages by polling the RX FIFOs at 1000 Hz
 *
 * @param pvParameters      Unused
 */
void canRX_task(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();

    CAN_RxHeaderTypeDef rxHeader = {0};
    uint8_t rxData[8];

    for (;;) {
        // Receive on FIFO 0
        while (HAL_CAN_GetRxFifoFillLevel(&can2Handle, CAN_RX_FIFO0) > 0) {
            HAL_CAN_GetRxMessage(&can2Handle, CAN_RX_FIFO0, &rxHeader, rxData);
            // Copy from rxData to external struct based on ID
        }

        // Receive on FIFO 1
        while (HAL_CAN_GetRxFifoFillLevel(&can2Handle, CAN_RX_FIFO1) > 0) {
            HAL_CAN_GetRxMessage(&can2Handle, CAN_RX_FIFO1, &rxHeader, rxData);
            // Copy from rxData to external struct based on ID
        }

        vTaskDelayUntil(&lastWakeTime, canRXTaskPeriod_ms);
    }
}
