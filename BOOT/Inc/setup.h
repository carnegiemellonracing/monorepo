/**
 * @file setup.h
 * @brief Board-level initialization helpers for the bootloader.
 *
 * Provides `Init` and `DeInit` functions to initialize microcontroller
 * peripherals (clocks, GPIO, CAN, etc.) used by the bootloader.
 *
 */

#ifndef SETUP_H
#define SETUP_H

/****************************************************************************************
* Function prototypes
****************************************************************************************/
void Init(void);
void DeInit(void);

#endif /* SETUP_H */
