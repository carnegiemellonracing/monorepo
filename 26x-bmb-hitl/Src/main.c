/**
 * @file main.c
 * @brief Firmware entry point.
 *
 * @author Ayush Garg
 * modified by Yi-An Liao
 */

#include <stm32f4xx_hal.h>  // HAL interface

#include <CMR/panic.h>  // cmr_panic()
#include <CMR/rcc.h>    // RCC interface
#include <CMR/adc.h>    // ADC interface
#include <CMR/gpio.h>   // GPIO interface
#include <CMR/tasks.h>  // Task interface

#include "statusLED.h"
#include "gpio.h"
#include "adc.h"

/**
 * @brief Firmware entry point.
 *
 * Device configuration and task initialization should be performed here.
 *
 * @return Does not return.
 */
int main(void) {
    cmr_rccSystemClockEnable();
    gpioInit();
    adcInit();
    statusLEDInit();

    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");
}
