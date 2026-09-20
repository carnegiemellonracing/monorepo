/**
 * @file statusLED.c
 * @brief status LED.
 * 
 * @author Carnegie Mellon Racing
 */

#include "statusLED.h"
#include "gpio.h"

#include <CMR/tasks.h>

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
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1) {
        cmr_gpioToggle(GPIO_OUT_LED_STATUS);
        vTaskDelayUntil(&lastWakeTime, statusLED_period_ms);
    }
}

void statusLEDInit() {
    cmr_taskInit(
        &statusLED_task,
        "statusLED",
        statusLED_priority,
        statusLED,
        NULL
    );
}
