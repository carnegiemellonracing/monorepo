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
    cmr_canClockEnable(cmr_getBootloaderCanPointer());

    // Configure CAN RX pin.
    cmr_gpioPin_t tx =cmr_getBootloaderCanTxPin();
    cmr_gpioPin_t rx =cmr_getBootloaderCanRxPin();

    cmr_canGpioInit(cmr_getBootloaderCanPointer(), rx.port, rx.pin, tx.port, tx.pin);
}

/**
 * @brief Deinitializes the CAN interface.
 *
 */
void canDeinit(void) {

    cmr_gpioPin_t tx =cmr_getBootloaderCanTxPin();
    cmr_gpioPin_t rx =cmr_getBootloaderCanRxPin();

    cmr_canGpioDeInit(rx.port, rx.pin, tx.port, tx.pin);

    /* disable clock just in case */
    cmr_canClockDisable(cmr_getBootloaderCanPointer());
    
}