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
#include "newState.h"
#include "tft.h"

/** @brief Status LED priority. */
static const uint32_t statusLED_priority = 2;

/** @brief Status LED period (milliseconds). */
static const TickType_t statusLED_period_ms = 250;

/** @brief Status LED task. */
static cmr_task_t statusLED_task;

/** @brief Status LED priority. */
static const uint32_t errorLEDs_priority = 2;

/** @brief Status LED period (milliseconds). */
static const TickType_t errorLEDs_period_ms = 250;

/** @brief Error LED task. */
static cmr_task_t errorLEDs_task;

/**
 * @brief Task for toggling the status LED.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void statusLED(void *pvParameters) {
    (void)pvParameters;

    cmr_gpioWrite(GPIO_LED_AMS, 0);
    cmr_gpioWrite(GPIO_LED_IMD, 0);
    cmr_gpioWrite(GPIO_LED_BSPD, 0);
    cmr_gpioWrite(GPIO_LED_STATUS, 0);

    static bool toggle = true;
    TickType_t lastWakeTime = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&lastWakeTime, statusLED_period_ms);
        cmr_gpioWrite(GPIO_LED_AMS, toggle);
        cmr_gpioWrite(GPIO_LED_IMD, toggle);
        cmr_gpioWrite(GPIO_LED_BSPD, toggle);
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
    cmr_canRXMeta_t *statusVSMMeta = canRXMeta + CANRX_VSM_STATUS;
    volatile cmr_canVSMStatus_t *statusVSM =
        (void *)statusVSMMeta->payload;

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

    cmr_gpioWrite(GPIO_LED_IMD, 0);
    cmr_gpioWrite(GPIO_LED_AMS, 0);
    cmr_gpioWrite(GPIO_LED_BSPD, 0);

    uint8_t latch = getVSMlatchMatrix();
    for (;;) {
        TickType_t lastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&lastWakeTime, errorLEDs_period_ms);
        latch = getVSMlatchMatrix();
        cmr_gpioWrite(GPIO_LED_IMD, latch & CMR_CAN_VSM_LATCH_IMD);
        cmr_gpioWrite(GPIO_LED_AMS, latch & CMR_CAN_VSM_LATCH_AMS);
        cmr_gpioWrite(GPIO_LED_BSPD, latch & CMR_CAN_VSM_LATCH_BSPD);

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

    // Peripheral configuration.
    gpioInit();
    canInit();
    adcInit();
    stateMachineInit();
    sensorsInit();
    //
    //tftInit();

    cmr_taskInit(
        &statusLED_task,
        "statusLED",
        statusLED_priority,
        statusLED,
        NULL);

    cmr_taskInit(
        &errorLEDs_task,
        "errorLEDs",
        errorLEDs_priority,
        errorLEDs,
        NULL);

    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");
}
