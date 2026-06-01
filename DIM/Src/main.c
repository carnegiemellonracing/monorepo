/**
 * @file main.c
 * @brief Firmware entry point.
 *
 * @author Carnegie Mellon Racing
 */

#include <CMR/adc.h>    // ADC interface
#include <CMR/can.h>    // CAN interface
#include <CMR/gpio.h>   // GPIO interface
#include <CMR/panic.h>  // cmr_panic()
#include <CMR/rcc.h>    // RCC interface
#include <CMR/i2c.h>    // I2C interface

#include "dv_error.h"
#include "error_leds.h"
#include "gpio.h"       // Board-specific GPIO interface
#include "i2c.h"
#include "state.h"
#include "status_led.h"
#include "tft.h"

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
    canInit();
    adcInit();
    tftInit();
    stateMachineInit();
    sensorsInit();
    i2c_slave_init();
    status_led_init();
    error_led_init();

    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");
}
