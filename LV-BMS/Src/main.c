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

#include "gpio.h"
#include "adc.h"
#include "i2c.h"
#include "can.h"
#include "data.h"

/** @brief Status LED priority. */
static const uint32_t status_LED_priority = 2;
static const uint32_t post_ms_monitor_priority = 1;

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

	TickType_t time_prev = xTaskGetTickCount();
	while (1) {
		cmr_gpioToggle(GPIO_LED);
		vTaskDelayUntil(&time_prev, status_LED_period_ms);
	}
}



static void post_ms_monitor(void *pvParameters) {
	(void) pvParameters;
	TickType_t time_prev = xTaskGetTickCount();
//	while(true) {
////		int post_ms = cmr_gpioRead(GPIO_POST_MS);
////		if(!post_ms)
//			write_sleep();
////		uint8_t prev = read_power_ctl();
////		uint8_t after = read_power_ctl();
//		vTaskDelayUntil(&time_prev, post_ms_read_period_ms);
//	}

	while(true){
		int post_ms = cmr_gpioRead(GPIO_POST_MS);
		if(!post_ms)
			write_sleep(); // This line will kill itself
		vTaskDelayUntil(&time_prev, post_ms_monitor_read_period_ms);
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
    cmr_rccSystemInternalClockEnable();

    // Peripheral configuration.
    gpio_init();
	adc_init();
	i2c_init();
	canInit();
	AFE_SETUP();

//	while(1) {
//
//		int post_ms = cmr_gpioRead(GPIO_POST_MS);
//
//		uint8_t chip_id = read_chip_id();
//
//		(void*)chip_id;
//		uint8_t status = read_status();
//		uint8_t ref_sel = read_ref_sel();
//		uint8_t cell_ctl = read_cell_ctl();
//		write_cell_ctl();
//		cell_ctl = read_cell_ctl();
//
//		uint8_t dummy = 0;
//	}


	// It seems like whichever task first created can turn off the AFE by write_sleep()
	cmr_taskInit(
		&post_ms_monitor_task,
		"post ms monitor",
		post_ms_monitor_priority,
		post_ms_monitor,
		NULL
	);

	cmr_taskInit(
		&status_LED_task,
		"Status LED",
		status_LED_priority,
		status_LED,
		NULL
	);

	vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!"); //what is this?
}