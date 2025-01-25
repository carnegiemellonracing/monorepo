/**
 * @file main.c
 * @brief Firmware entry point.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32l4xx_hal.h>  // HAL interface

#include <CMR/panic.h>  // cmr_panic()
#include <CMR/rcc.h>    // RCC interface
#include <CMR/can.h>    // CAN interface
#include <CMR/adc.h>    // ADC interface
#include <CMR/gpio.h>   // GPIO interface
#include <CMR/tasks.h>  // Task interface

#include <gpio.h>
#include <adc.h>
#include <i2c.h>
#include <can.h>

/** @brief Status LED priority. */
static const uint32_t status_LED_priority = 2;
static const uint32_t vtherm_read_priority = 5;
static const uint32_t vcout_read_priority = 1;
static const uint32_t post_ms_monitor_priority = 1;

/** @brief Status LED period (milliseconds). */
static const TickType_t status_LED_period_ms = 250;
static const TickType_t vtherm_read_period_ms = 10;
static const TickType_t vcout_read_period_ms = 10;
static const TickType_t post_ms_monitor_read_period_ms = 10;

/** @brief Status LED task. */
static cmr_task_t status_LED_task;
static cmr_task_t vtherm_read_task;
static cmr_task_t vcout_read_task;
static cmr_task_t post_ms_monitor_task;

/**
 * @brief Task for toggling the status LED.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */

static void status_LED() {
	cmr_gpioWrite(GPIO_LED, 0);

	TickType_t time_prev = xTaskGetTickCount();
	while (1) {
		cmr_gpioToggle(GPIO_LED);
		vTaskDelayUntil(&time_prev, status_LED_period_ms);
	}
}

/**
 * @param vtherm_index ∈ ℕ, vtherm_index ∈ [0, 15]
 */
static uint32_t vtherm_read_index(uint32_t vtherm_index) {

	int map[] = {4, 6, 7, 5, 2, 1, 0, 3, 3, 0, 1, 2, 5, 4, 2, 1};
	int sel_index = map[vtherm_index];

	cmr_gpioWrite(GPIO_VTHERM_SEL0, sel_index & 0x1);
	cmr_gpioWrite(GPIO_VTHERM_SEL1, sel_index & 0x2);
	cmr_gpioWrite(GPIO_VTHERM_SEL2, sel_index & 0x4);
	if(vtherm_index < 0x8)
		return adc_read(ADC_VTHERM_PIN1);
	return adc_read(ADC_VTHERM_PIN2);
}

static void vtherm_read(const adc_channel_t ch) {
	TickType_t time_prev = xTaskGetTickCount();
	while(true) {
		for(uint32_t i = 0; i < VTHERM_NUM; i++) {
			int temp = vtherm_read_index(i);
			(void *)temp;
		}
		int temp = vtherm_read_index(7);
		int temp2 = vtherm_read_index(8);
		temp + temp2;
		vTaskDelayUntil(&time_prev, vtherm_read_period_ms);
	}
}

static void post_ms_monitor() {
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

static void vcout_read() {
	TickType_t time_prev = xTaskGetTickCount();
	while(true){
//		int vc = adc_read(ADC_AFE_VCOUT);
//
		write_ref_sel(true);
		uint8_t ref_sel = read_ref_sel();
		write_cell_ctl(0x1, 0x5);
		uint8_t cell_ctl = read_cell_ctl();
//
//		uint8_t power_ctl = read_power_ctl();
//		write_sleep(); // This line will kill itself

//		int dummy = vc + 1;
		vTaskDelayUntil(&time_prev, vcout_read_period_ms);
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
    gpio_init();
	adc_init();
	i2c_init();
	canInit();

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

	cmr_taskInit(
		&vtherm_read_task,
		"vtherm read",
		vtherm_read_priority,
		vtherm_read,
		NULL
	);

	// It seems like whichever task first created can turn off the AFE by write_sleep()
	cmr_taskInit(
		&post_ms_monitor_task,
		"post ms monitor",
		post_ms_monitor_priority,
		post_ms_monitor,
		NULL
	);

	cmr_taskInit(
		&vcout_read_task,
		"vcout read",
		vcout_read_priority,
		vcout_read,
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
    cmr_panic("vTaskStartScheduler returned!");
}