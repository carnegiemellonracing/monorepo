/**
 * @file main.c
 * @brief Firmware entry point.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h>  // HAL interface

#include <CMR/panic.h>  // cmr_panic()
#include <CMR/rcc.h>    // RCC interface
#include <CMR/tasks.h>  // Task interface

#include "gpio.h"   // Board-specific GPIO interface
#include "can.h"    // Board-specific CAN interface
#include "adc.h"    // Board-specific ADC interface
#include "uart.h"   // Board-specific UART interface
#include "bms_error.h"
#include "watchdog.h"   // Board-specific Watchdog interface

/** @brief Status LED priority. */
static const uint32_t statusLED_priority = 2;

/** @brief BMB Sample Task priority. */
static const uint32_t bmbSample_priority = 7;

/** @brief BMB Sample Task priority. */
static const uint32_t setState_priority = 4;

/** @brief Status LED period (milliseconds). */
static const TickType_t statusLED_period_ms = 250;

/** @brief Status LED task. */
static cmr_task_t statusLED_task;

/** @brief BMB Sample Task */
static cmr_task_t bmbSample_task;

/** @brief Set State Task */
static cmr_task_t setState_task;

/**
 * @brief Task for toggling the status LED.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void statusLED(void *pvParameters) {
    (void) pvParameters;

    cmr_gpioWrite(GPIO_MCU_LED, 0);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        cmr_gpioToggle(GPIO_MCU_LED);

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
    // cmr_rccSystemInternalClockEnable();

    // Peripheral configuration.
    gpioInit();
    canInit();
    uartInit();
//    adcInit();
    sensorsInit();
    // spiInit();
//    wwdgInit();

    cmr_taskInit(
        &statusLED_task,
        "statusLED",
        statusLED_priority,
        statusLED,
        NULL
    );

    // BMB_task
    cmr_taskInit(
        &bmbSample_task,
        "BMB Sample Task",
        bmbSample_priority,
        vBMBSampleTask,
        NULL
    );

    // State Task
    cmr_taskInit(
        &setState_task,
        "Set State Task",
        setState_priority,
        vSetStateTask,
        NULL
    );

    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");
}

