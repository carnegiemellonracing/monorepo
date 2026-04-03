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

#include "adc.h"
#include "analysis.h"
#include "bq_interface.h"
#include "can.h"
#include "dwt.h"
#include "gpio.h"
#include "sampling.h"
#include "sensors.h"
#include "status_led.h"

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
	DWT_Delay_Init();
	gpioInit();
	statusLedInit();
	BMBInit();
	adcInit();
	sensorsInit();
	canInit();
	sampleInit();
	analysisInit();

	vTaskStartScheduler();
  cmr_panic("vTaskStartScheduler returned!"); 
}