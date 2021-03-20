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
    volatile cmr_canVSMStatus_t *vsmState = canGetPayload(CANRX_VSM_STATUS);

    cmr_gpioWrite(GPIO_MTR_CTRL_ENABLE, 0);
    cmr_gpioWrite(GPIO_MC_EFUSE_AUTO, 0);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        // Inverter should be powered if the car is in HV_EN, RTD, or the
        // VSM is attempting to boot the inverter in it's internal state
        if (vsmHeartbeat->state == CMR_CAN_HV_EN ||
            vsmHeartbeat->state == CMR_CAN_RTD || 
            vsmState->internalState == CMR_CAN_VSM_STATE_INVERTER_EN ||
            vsmState->internalState == CMR_CAN_VSM_STATE_HV_EN) 
        {
            cmr_gpioWrite(GPIO_MTR_CTRL_ENABLE, 1);
        } else {
            cmr_gpioWrite(GPIO_MTR_CTRL_ENABLE, 0);
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
