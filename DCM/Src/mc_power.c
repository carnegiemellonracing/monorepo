/**
 * @file mc_power.c
 * @brief Motor controller power control task.
 *
 * @author Ayush Garg
 */

#include "gpio.h"       // Board-specific GPIO interface
#include "mc_power.h"        // Interface to implement
#include <CMR/gpio.h>       // GPIO interface


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
}

void mcCtrlOn() {
    cmr_gpioWrite(GPIO_MTR_CTRL_ENABLE, 1);
}

