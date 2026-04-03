/**
 * @file status_led.c
 * @brief Implementation for status LED
 *
 * @author Ayush Garg
 */

 #include <CMR/tasks.h>  // Task interface

/** @brief Status LED priority. */
static const uint32_t status_LED_priority = 2;

/** @brief Status LED period (milliseconds). */
static const TickType_t status_LED_period_ms = 250;

/** @brief Status LED task. */
static cmr_task_t status_LED_task;

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

void statusLedInit(){
	cmr_taskInit(
		&status_LED_task,
		"Status LED",
		status_LED_priority,
		status_LED,
		NULL
	);
}

