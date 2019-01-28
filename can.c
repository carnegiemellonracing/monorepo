/**
 * @file can.c
 * @brief Board-specific CAN implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <FreeRTOS.h>       // FreeRTOS interface
#include <task.h>           // xTaskCreate()

#include <CMR/can.h>        // CAN interface
#include <CMR/can_ids.h>    // CMR CAN IDs
#include <CMR/can_types.h>  // CMR CAN types

#include "can.h"    // Interface to implement
#include "adc.h"    // adcVSense
#include "gpio.h"   // XXX

/** @brief Primary CAN interface. */
static cmr_can_t can;

/** @brief CAN TX priority. */
static const uint32_t canTXPriority = 5;

/** @brief CAN TX period (milliseconds). */
static const TickType_t canTXPeriod_ms = 100;

/**
 * @brief Callback for receiving CAN messages.
 *
 * @param id The received message's CAN ID.
 * @param data The received data.
 * @param len The received data's length.
 */
static void canRXCallback(uint16_t id, const void *data, size_t len) {
    const struct {
        uint8_t pin;
    } *test = data;

    switch (id) {
        // TODO
        // XXX
        case 0x200:
            gpioToggle(test->pin);
            break;
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
        // XXX
        cmr_canDIMPowerDiagnostics_t msg = {
            .busVoltage_mV = adcVSense->value,
            .busCurrent_mA = adcISense->value
        };

        cmr_canTX(
            &can, CMR_CANID_DIM_POWER_DIAGNOSTICS,
            &msg, sizeof(msg)
        );

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
            .ids = { 0x200, 0x200, 0x200, 0x200 }   // TODO: use ID constants
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

