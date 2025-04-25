/**
 * @file brakelight.c
 * @brief Brakelight implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "can.h"        // Board-specific CAN interface
#include "gpio.h"       // Board-specific GPIO interface
#include "brakelight.h"        // Interface to implement
#include <CMR/gpio.h>       // GPIO interface

/** @brief Brake light threshold (pounds-per-square-inch). */
static const uint16_t brakeLightThreshold_PSI = 20;
/** @brief Brake light control task priority. */
static const uint32_t brakelight_priority = 4;
/** @brief Brake light control task period. */
static const TickType_t brakelight_period_ms = 50;
/** @brief Brake light control task. */
static cmr_task_t brakelight_task;

/**
 * @brief Task for controlling the brakelight.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
void brakelight(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    volatile cmr_canVSMSensors_t *vsmSensors = canGetPayload(CANRX_VSM_SENSORS);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        // if (vsmSensors->brakePressureRear_PSI > brakeLightThreshold_PSI) {
        if(vsmSensors->brakePressureRear_PSI >= 0) {
            cmr_gpioWrite(GPIO_BRKLT_ENABLE, 1);
        } else {
            cmr_gpioWrite(GPIO_BRKLT_ENABLE, 0);
        }

        vTaskDelayUntil(&lastWakeTime, brakelight_period_ms);
    }
}

/**
 * @brief Initialization of brakelight task.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
void brakelightInit() {
    cmr_taskInit(
        &brakelight_task,
        "brakelight",
        brakelight_priority,
        brakelight,
        NULL
    );
}
