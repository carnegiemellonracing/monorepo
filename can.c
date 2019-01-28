/**
 * @file can.c
 * @brief Board-specific CAN implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <FreeRTOS.h>       // FreeRTOS interface
#include <task.h>           // xTaskCreate()

#include <CMR/can.h>        // CAN interface
#include <CMR/can_types.h>  // CMR CAN types

#include "can.h"    // Interface to implement
#include "adc.h"    // adcVSense

/** @brief Primary CAN interface. */
static cmr_can_t can;

/** @brief CAN TX priority. */
static const uint32_t canTXPriority  = 5;

/** @brief CAN TX period (milliseconds). */
static const TickType_t canTXPeriod_ms = 10;

/**
 * @brief Callback for receiving CAN messages.
 *
 * @param id The received message's CAN ID.
 * @param data The received data.
 * @param len The received data's length.
 */
static void canRXCallback(uint16_t id, const void *data, size_t len) {
    switch (id) {
        // TODO
    }
}

/**
 * @brief Task for sending CAN messages.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTXTask(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        uint8_t cdcHeartbeat[3] = {
            1,  // TODO junk
            // TODO Overflow; we will go to reporting mV in a uint16_t soon.
            (uint8_t) (adcVSense->value * 15 / 113),
            3   // TODO junk
        };
        cmr_canTX(&can, 0x201, cdcHeartbeat, sizeof(cdcHeartbeat));

        vTaskDelayUntil(&lastWakeTime, canTXPeriod_ms);
    }
}

/**
 * @brief Initializes the CAN interface.
 */
void canInit(void) {
    // CAN2 initialization.
    cmr_canInit(
        &can, CAN2,
        canRXCallback, "canRX",
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
}

