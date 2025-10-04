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
#include "fans.h" // Board-specific Fan interface

/** @brief Status LED priority. */
static const uint32_t statusLED_priority = 2;

/** @brief BMB Sample Task priority. */
static const uint32_t bmbSample_priority = 3;

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

/* Debug Exception and Monitor Control Register base address */
#define DEMCR                 *((volatile uint32_t*) 0xE000EDFCu)

/* ITM register addresses */
#define ITM_STIMULUS_PORT0    *((volatile uint32_t*) 0xE0000000u)
#define ITM_TRACE_EN          *((volatile uint32_t*) 0xE0000E00u)

static void statusLEDInit(){
    cmr_taskInit(
        &statusLED_task,
        "statusLED",
        statusLED_priority,
        statusLED,
        NULL
    );

}

static void stateInit(){
    cmr_taskInit(
        &setState_task,
        "Set State Task",
        setState_priority,
        vSetStateTask,
        NULL
    );
}

/**
 * @brief Firmware entry point.
 *
 * Device configuration and task initialization should be performed here.
 *
 * @return Does not return.
 */
int main(void) {
	HAL_Init();
    cmr_rccSystemClockEnable();
    // cmr_rccSystemInternalClockEnable();

    // Peripheral configuration.
    gpioInit();
    adcInit();
    sensorsInit();
    canInit();

    //init fan task
    fanInit();
    //wwdgInit();

    statusLEDInit(); 

    // State Task
    stateInit(); 


    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");
}

