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
#include "can.h"        // Board-specific CAN interface
#include "gpio.h"       // Board-specific GPIO interface
#include "i2c.h"        // Board-specific I2C interface
#include "mlx90640.h"   // MLX90640 thermal array interface

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
    adcInit();
    canInit();
    i2cInit();
    mlx90640Init();

    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");
}
