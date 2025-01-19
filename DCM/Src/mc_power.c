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
//static const uint32_t mcPowerControl_priority = 2;
/** @brief Motor Controller Power Control task period. */
//static const TickType_t mcPowerControl_period_ms = 50;
/** @brief Motor Controller Power Control task. */
//static cmr_task_t mcPowerControl_task;

//volatile cmr_canHeartbeat_t *heartbeatPTC = &heartbeat;

//mcCtrl states
void mcCtrlOff();
void mcCtrlOn();

/**
 * @brief Task for controller power to motor controller.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */

void mcCtrlOff() {
    cmr_gpioWrite(GPIO_MTR_CTRL_ENABLE, 0);
    cmr_gpioWrite(GPIO_MC_EFUSE_AUTO, 0);
}

void mcCtrlOn() {
    cmr_gpioWrite(GPIO_MTR_CTRL_ENABLE, 1);
    cmr_gpioWrite(GPIO_MC_EFUSE_AUTO, 1);
}

