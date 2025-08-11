/**
 * @file timing.c
 * @brief Just Some Code to run some timing analysis
 *
 * @author Carnegie Mellon Racing
 */

 #include "timing.h"
 #include <CMR/tasks.h>  // Task interface
#include <CMR/rcc.h>    // RCC interface


 /** @brief Timing Task priority. */
static const uint32_t timingPriority = 3;

/** @brief Timing Task period (milliseconds). */
static const TickType_t timingTaskPeriod = 1000;

/** @brief Timing task. */
static cmr_task_t timingTaskObj;


static void timingTaskFn(void *pvParameters) {
    (void)pvParameters;  // Placate compiler.
    static char resultBuffer[700]; 
    
    TickType_t lastWakeTime = xTaskGetTickCount();
    while(1){
        vTaskGetRunTimeStatistics(resultBuffer, sizeof(resultBuffer));
        vTaskDelayUntil(&lastWakeTime, timingTaskPeriod);
    }
}

/**
 * @brief Initializes the timing Task
 */
void timingInit(void) {
    cmr_taskInit(
        &timingTaskObj,
        "Timing Task",
        timingPriority,
        timingTaskFn,
        NULL);
}