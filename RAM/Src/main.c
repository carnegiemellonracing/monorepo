/**
 * @file main.c
 * @brief Firmware entry point.
 *
 * @author Carnegie Mellon Racing
 */

// CMR framework
#include <CMR/can.h>    // CAN interface
#include <CMR/gpio.h>   // GPIO interface
#include <CMR/panic.h>  // cmr_panic()
#include <CMR/rcc.h>    // RCC interface
#include <CMR/rtc.h>    // RTC interface

#include <CMR/tasks.h>  // Task interface
#include <CMR/remote_boot.h>  // Task interface

// Middleware
#include "fatfs.h"      // middleware for file system provided by ST

// Project headers
#include "can.h"        // Board-specific CAN interface
#include "config.h"     // Previous flash configuration
#include "memorator.h"  // Board-specific GPIO interface
#include "parser.h"     // JSON configuration
#include "sample.h"     // CBOR encoding
#include "statusLED.h"  // Board-specific statusLED interface
#include "uart.h"       // Board-specific UART interface


/**
 * @brief Interrupt vector table.
 *
 * Defined by the startup file, and placed at `ORIGIN(FLASH)` by the linker
 * script via the `.isr_vector` section.
 */
extern void (* const g_pfnVectors[])(void);


/**
 * @brief Firmware entry point.
 *
 * Device configuration and task initialization should be performed here.
 *
 * @return Does not return.
 */
int main(void) {
    // Relocate the interrupt vector table.
    //
    // The OpenBLT bootloader owns the first 64K of flash, so this image is
    // linked at 0x08010000 and its vector table is not at FLASH_BASE. VTOR
    // still points at the bootloader's table on entry, so every exception -
    // SVC, PendSV and SysTick included - would dispatch into the bootloader
    // and the FreeRTOS scheduler would never run. Taking the address of
    // g_pfnVectors keeps this in step with ORIGIN(FLASH) in the linker
    // script automatically.
    //
    // Must happen before anything can take an interrupt.
    SCB->VTOR = (uint32_t) g_pfnVectors;

    // System initialization.
    HAL_Init();
    cmr_rccSystemClockEnable();
    cmr_rtc_init();
    cmr_remoteBootInit();


    // Peripheral configuration.
    uartInit();
    canInit();
    statusLEDInit();
    //memoratorInit();

    // // Load in JSON configuration
    parserInit();
    // // Set up CBOR encoder
    sampleInit();
    // // Pull in previous configuration
    configInit();

    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");
}

