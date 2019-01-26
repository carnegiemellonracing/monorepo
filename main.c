/**
 * @file main.c
 * @brief Firmware entry point.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h>  // HAL interface

#include <FreeRTOS.h>       // FreeRTOS interface
#include <task.h>           // xTaskCreate(), vTaskStartScheduler()

#include <CMR/panic.h>  // cmr_panic()
#include <CMR/rcc.h>    // RCC interface
#include <CMR/can.h>    // CAN interface
#include <CMR/adc.h>    // ADC interface

#include "can.h"    // Board-specific CAN interface
#include "adc.h"    // Board-specific ADC interface

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
    canInit();
    adcInit();

    // Node tasks.
    /* TODO
    xTaskCreate(mcuStatusLED_task, "LED", configMINIMAL_STACK_SIZE, NULL,
                mcuStatusLEDTaskPriority, NULL);
                */

    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");
}

