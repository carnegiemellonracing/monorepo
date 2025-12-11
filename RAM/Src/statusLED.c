/**
 * @file  statusLED.c
 * @brief sets up statusLED task
 *
 * @author Carnegie Mellon Racing
 */

 #include <CMR/tasks.h>  // Task interface
#include "statusLED.h"  // interface to implement
#include "gpio.h"  // gpio for turning LED on
#include <CMR/gpio.h>   // GPIO interface


/** @brief Status LED priority. */
static const uint32_t statusLED_priority = 2;

/** @brief Status LED period (milliseconds). */
static const TickType_t statusLED_period_ms = 250;

/** @brief Status LED task. */
static cmr_task_t statusLED_task;


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

void statusLEDInit(){
    cmr_taskInit(
        &statusLED_task,
        "statusLED",
        statusLED_priority,
        statusLED,
        NULL
    );
}




