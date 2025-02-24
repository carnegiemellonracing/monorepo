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
#include "gpio.h"

/** @brief Status LED priority. */
static const uint32_t statusLED_priority = 2;

/** @brief Status LED period (milliseconds). */
static const TickType_t statusLED_period_ms = 250;

/** @brief Status LED task. */
static cmr_task_t statusLED_task;

static cmr_task_t checkADC_task;
static const uint32_t checkADC_priority = 1;

/**
 * @brief Task for toggling the status LED.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void statusLED(void *pvParameters) {
    (void) pvParameters;

    cmr_gpioWrite(GPIO_LED_STATUS, 0);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
    	cmr_gpioToggle(GPIO_LED_STATUS);

    	vTaskDelayUntil(&lastWakeTime, statusLED_period_ms);
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

    gpioInit();
    adcInit();
    canInit();

    cmr_taskInit(
        &statusLED_task,
        "statusLED",
        statusLED_priority,
        statusLED,
        NULL
    );

    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");
}

