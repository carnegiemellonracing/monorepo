/**
 * @file can.c
 * @brief Board-specific CAN implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "can.h"      // Interface to implement
#include <CMR/can.h>  // Can-specific interface

/**
 * @brief Initializes the CAN interface.
 */
void canInit(void) {
    // required clocks
    cmr_canClockEnable(CAN1);

    // Configure CAN RX pin.
    cmr_canGpioInit(CAN1, GPIOB, GPIO_PIN_8, GPIOB, GPIO_PIN_9);
}

void canDeinit(void) {

    cmr_canGpioDeInit(GPIOB, GPIO_PIN_8, GPIOB, GPIO_PIN_9);

    // disable clock jic
    cmr_canClockDisable(CAN1);
    
}