/**
 * @file main.c
 * @brief Firmware entry point.
 *
 * @author Carnegie Mellon Racing
 */

#include <CMR/adc.h>    // ADC interface
#include <CMR/can.h>    // CAN interface
#include <CMR/gpio.h>   // GPIO interface
#include <CMR/panic.h>  // cmr_panic()
#include <CMR/rcc.h>    // RCC interface

#include "adc.h"        // Board-specific ADC interface
#include "can.h"        // Board-specific CAN interface
#include "gpio.h"       // Board-specific GPIO interface

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
    (void)pvParameters;

    cmr_gpioWrite(GPIO_LED_STATUS, 0);

    static bool toggle = true;
    TickType_t lastWakeTime = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&lastWakeTime, statusLED_period_ms);
        cmr_gpioWrite(GPIO_LED_STATUS, toggle);
        toggle = !toggle;
    }
}
//
///**
// * @brief Gets the VSM latch matrix.
// *
// * @note VSM latch matrix is maintained in the received CAN status.
// *
// * @return The VSM latch matrix.
// */
uint8_t getVSMlatchMatrix(void) {
    volatile cmr_canVSMStatus_t *statusVSM = getPayload(CANRX_VSM_STATUS);

    return statusVSM->latchMatrix;
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
    gpioInit();
    canInit();
    adcInit();

    cmr_taskInit(
        &statusLED_task,
        "statusLED",
        statusLED_priority,
        statusLED,
        NULL);

    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");
}
