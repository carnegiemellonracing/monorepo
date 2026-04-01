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
#include "bq_interface.h"
#include "can.h"
#include "data.h"
#include "dwt.h"
#include "gpio.h"


/** @brief Status LED priority. */
static const uint32_t status_LED_priority = 2;

/** @brief Status LED period (milliseconds). */
static const TickType_t status_LED_period_ms = 250;
static const TickType_t post_ms_monitor_read_period_ms = 10;

/** @brief Status LED task. */
static cmr_task_t status_LED_task;
static cmr_task_t post_ms_monitor_task;

/**
 * @brief Task for toggling the status LED.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */

static void status_LED(void *pvParameters) {
	(void) pvParameters;
	cmr_gpioWrite(GPIO_LED, 0);

	TickType_t lastWakeTime = xTaskGetTickCount();
	while (1) {
		cmr_gpioToggle(GPIO_LED);
		vTaskDelayUntil(&lastWakeTime, status_LED_period_ms);
	}
}


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
  gpio_init();
	BMBInit();
	adc_init();
	canInit();
	sampleInit();

	cmr_taskInit(
		&status_LED_task,
		"Status LED",
		status_LED_priority,
		status_LED,
		NULL
	);

	cmr_gpioWrite(GPIO_BMS_ERROR, 0);


	vTaskStartScheduler();
  	cmr_panic("vTaskStartScheduler returned!"); 
}