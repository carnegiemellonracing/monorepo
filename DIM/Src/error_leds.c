/**
 * @file error_leds.c
 * @brief error led task
 *
 * @author Ayush Garg
 */

#include "can.h"        
#include "error_leds.h"   // Interface to implement
#include "gpio.h"        

/** @brief Error LED priority. */
static const uint32_t errorLEDs_priority = 2;

/** @brief Error LED period (milliseconds). */
static const TickType_t errorLEDs_period_ms = 250;

/** @brief Error LED task. */
static cmr_task_t errorLEDs_task;

/**
* @brief Gets the VSM latch matrix.
*
* @note VSM latch matrix is maintained in the received CAN status.
*
* @return The VSM latch matrix.
*/
uint8_t getVSMlatchMatrix(void) {
    volatile cmr_canVSMStatus_t *statusVSM = getPayload(CANRX_VSM_STATUS);

    return statusVSM->latchMatrix;
}

/**
 * @brief Task for error indication over LEDs.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void errorLEDs(void *pvParameters) {
    (void)pvParameters;

    for (;;) {
        TickType_t lastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&lastWakeTime, errorLEDs_period_ms);
    }
}


/**
 * @brief inits the error LED task
 * 
 */
void error_led_init(void){
    cmr_taskInit(
        &errorLEDs_task,
        "errorLEDs",
        errorLEDs_priority,
        errorLEDs,
        NULL);
}