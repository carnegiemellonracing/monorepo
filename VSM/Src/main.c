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
#include <CMR/adc.h>    // ADC interface
#include <CMR/gpio.h>   // GPIO interface
#include <CMR/tasks.h>  // Task interface

#include "adc.h"        // Board-specific ADC interface
#include "assi.h"
#include "can.h"        // Board-specific CAN interface
#include "dac.h"        // Board-specific DAC interface
#include "error.h"
#include "gpio.h"       // Board-specific GPIO interface
#include "sensors.h"    // Board-specific sensors interface
#include "statusLED.h"  // Status LED
#include "state.h"      // stateInit()
#include "tssi.h"       // TSSI control 

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
    
    // Peripheral configuration.
    gpioInit();
    pwmInit();
    canInit();
    adcInit();
    sensorsInit();
    stateInit();
    tssiInit();
    assiInit();
    dacInit();

    statusLEDInit();

    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");
}

