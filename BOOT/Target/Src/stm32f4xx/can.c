/**
 * @file can.c
 * @brief Board-specific CAN implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "can.h"      // Interface to implement
#include <CMR/can.h>  // Can-specific interface
#include <CMR/board_info.h>  // Board info interface

/**
 * @brief Initializes the CAN interface.
 */
void canInit(void) {
    // required clocks
    cmr_canClockEnable(cmr_getBootloaderCanIo().instance);

    cmr_canIo_t io = cmr_getBootloaderCanIo();
    cmr_canGpioInit(&io);
}

/**
 * @brief Deinitializes the CAN interface.
 *
 */
void canDeinit(void) {
    cmr_canIo_t io = cmr_getBootloaderCanIo();
    cmr_canGpioDeInit(&io);

    /* disable clock just in case */
    cmr_canClockDisable(cmr_getBootloaderCanIo().instance);
    
}