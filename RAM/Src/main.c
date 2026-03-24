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
#include <CMR/timer.h>  // Microsecond timer

#include <CMR/tasks.h>  // Task interface

// Middleware
#include "fatfs.h"      // middleware for file system provided by ST

// Project headers
#include "can.h"        // Board-specific CAN interface
#include "config.h"     // Previous flash configuration
#include "gpio.h"       // Board-specific GPIO interface
#include "memorator.h"  // Board-specific GPIO interface
#include "parser.h"     // JSON configuration
#include "sample.h"     // CBOR encoding
#include "uart.h"       // Board-specific UART interface
#include "statusLED.h"  // Board-specific statusLED interface
#include "task_trigger.h" //trigger task


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
    cmr_rtc_init();


    // Peripheral configuration.
    uartInit();
    gpioInit();
    canInit();
    statusLEDInit();
    memoratorInit();

    //start microsecond timer
    microsecond_timer_init();
    //start camera trigger task
    cameraTriggerInit();
    // Load in JSON configuration
    // parserInit();
    // Set up CBOR encoder
    // sampleInit();
    // Pull in previous configuration
    configInit();

    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");
}

