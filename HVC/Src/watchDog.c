/**
 * @file statusLED.c
 * @brief statusLED task
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h>  // HAL interface
#include <CMR/tasks.h>  // Task interface
#include "gpio.h"   // Board-specific GPIO interface

/** @brief Status LED priority. */
static const uint32_t watchDogPriority = 2;

/** @brief Status LED period (milliseconds). */
static const TickType_t watchDogPeriod_ms = 20;

/** @brief Status LED task. */
static cmr_task_t watchDogTask;

/**
 * @brief Task for toggling the status LED.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void petWatchDog(void *pvParameters) {
    (void) pvParameters;

    cmr_gpioWrite(GPIO_TO_WATCHDOG, 0);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        cmr_gpioToggle(GPIO_TO_WATCHDOG);
        vTaskDelayUntil(&lastWakeTime, watchDogPeriod_ms);
    }
}


void watchDogInit(){
    cmr_taskInit(
        &watchDogTask,
        "Watch Dog",
        watchDogPriority,
        petWatchDog,
        NULL
    );
}