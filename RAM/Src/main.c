/**
 * @file main.c
 * @brief Firmware entry point.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h>  // HAL interface

#include <CMR/panic.h>  // cmr_panic()
#include <CMR/rcc.h>    // RCC interface
#include <CMR/can.h>    // CAN interface
#include <CMR/gpio.h>   // GPIO interface
#include <CMR/tasks.h>  // Task interface


#include "can.h"    // Board-specific CAN interface
#include "config.h" // Previous flash configuration
#include "fatfs.h"   // middleware for file system provided by ST
#include "gpio.h"   // Board-specific GPIO interface
#include "memorator.h"   // Board-specific GPIO interface
#include "parser.h" // JSON configuration
#include "sample.h" // CBOR encoding
#include "statusLED.h"   // Board-specific statusLED interface
#include "uart.h"   // Board-specific UART interface
#include "rtc.h"


/**
 * @brief Firmware entry point.
 *
 * Device configuration and task initialization should be performed here.
 *
 * @return Does not return.
 */
int main(void) {
    // System initialization.
    HAL_Init();
    cmr_rccSystemClockEnable();
    // SystemClock_Config();

    // Peripheral configuration.
    uartInit();
    gpioInit();
    canInit();
    statusLEDInit();
    memoratorInit();

    // MX_RTC_Init();
    // Load in JSON configuration
    parserInit();
    // Set up CBOR encoder
    sampleInit();
    // Pull in previous configuration
    configInit();

    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");
}

