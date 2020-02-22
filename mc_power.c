/**
 * @file fans.c
 * @brief Fan Control implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "can.h"        // Board-specific CAN interface
#include "gpio.h"       // Board-specific GPIO interface
#include "mc_power.h"        // Interface to implement
#include <CMR/gpio.h>       // GPIO interface

/** @brief Motor Controller Power Control task priority. */
static const uint32_t mcPowerControl_priority = 2;
/** @brief Motor Controller Power Control task period. */
static const TickType_t mcPowerControl_period_ms = 50;
/** @brief Motor Controller Power Control task. */
static cmr_task_t mcPowerControl_task;

//volatile cmr_canHeartbeat_t *heartbeatPTC = &heartbeat;

/**
 * @brief Task for controller power to motor controller.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void mcPowerControl(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    /* Get reference to VSM Heartbeat */
    volatile cmr_canHeartbeat_t *vsmHeartbeat = canGetPayload(CANRX_HEARTBEAT_VSM);

    cmr_gpioWrite(GPIO_MTR_CTRL_ENABLE, 0);
    cmr_gpioWrite(GPIO_MC_EFUSE_AUTO, 0);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        switch (heartbeat.state) {
            case CMR_CAN_RTD:
                cmr_gpioWrite(GPIO_MTR_CTRL_ENABLE, 1);
                break;
            case CMR_CAN_HV_EN:
                cmr_gpioWrite(GPIO_MTR_CTRL_ENABLE, 1);         // This code is subject to change, giving mc 600V before logic voltage is spooky.
                break;                                          // Proposed solution:
            default:                                            // VSM sends ping to PTC to turn on mc logic voltage
                cmr_gpioWrite(GPIO_MTR_CTRL_ENABLE, 0);         // after VSM sees MC can data via CDC it does precharge
                break;
        }
        vTaskDelayUntil(&lastWakeTime, mcPowerControl_period_ms);
    }
}

void mcPowerInit() {
    cmr_taskInit(
        &mcPowerControl_task,
        "mcPowerControl",
        mcPowerControl_priority,
        mcPowerControl,
        NULL
    );
}
