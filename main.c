/**
 * @file main.c
 * @brief Firmware entry point.
 *
 * @author Carnegie Mellon Racing
 */

#include <stdio.h>      // snprintf()

#include <stm32f4xx_hal.h>  // HAL interface

#include <CMR/panic.h>  // cmr_panic()
#include <CMR/rcc.h>    // RCC interface
#include <CMR/can.h>    // CAN interface
#include <CMR/adc.h>    // ADC interface
#include <CMR/gpio.h>   // GPIO interface
#include <CMR/tasks.h>  // Task interface

#include "gpio.h"       // Board-specific GPIO interface
#include "can.h"        // Board-specific CAN interface
#include "adc.h"        // Board-specific ADC interface
#include "segments.h"   // Segment display interface
#include "state.h"      // State interface

/** @brief Display length. */
#define DISPLAY_LEN 8

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
    /** @brief Segment text formats for each state. */
    static const char *stateFmt[] = {
        [CMR_CAN_UNKNOWN] = "????????",
        [CMR_CAN_GLV_ON] = "GLV ON  ",
        [CMR_CAN_HV_EN] = "HV %c %.3u",
        [CMR_CAN_RTD] = "RD %c %.3u",
        [CMR_CAN_ERROR] = "ERROR   ",
        [CMR_CAN_CLEAR_ERROR] = "C_ERROR ",
    };

    /** @brief Characters for each gear. */
    static const char gearChars[CMR_CAN_GEAR_LEN] = {
        [CMR_CAN_GEAR_UNKNOWN] = '-',
        [CMR_CAN_GEAR_REVERSE] = 'R',
        [CMR_CAN_GEAR_SLOW] = 'S',
        [CMR_CAN_GEAR_FAST] = 'F',
        [CMR_CAN_GEAR_ENDURANCE] = 'E',
        [CMR_CAN_GEAR_AUTOX] = 'X',
        [CMR_CAN_GEAR_SKIDPAD] = '8',
        [CMR_CAN_GEAR_ACCEL] = 'A',
        [CMR_CAN_GEAR_TEST] = 'T'
    };

    (void) pvParameters;

    cmr_gpioWrite(GPIO_LED_STATUS, 0);

    cmr_canRXMeta_t *metaHVCPackVoltage = canRXMeta + CANRX_HVC_PACK_VOLTAGE;
    volatile cmr_canHVCPackVoltage_t *canHVCPackVoltage =
        (void *) metaHVCPackVoltage->payload;

    for (
        TickType_t lastWakeTime = xTaskGetTickCount();
        1;
        vTaskDelayUntil(&lastWakeTime, statusLED_period_ms)
    ) {
        cmr_canState_t stateVSM = stateGetVSM();
        cmr_canGear_t gear = stateGetGear();

        uint8_t gearChar = (gear < CMR_CAN_GEAR_LEN)
            ? gearChars[gear]
            : gearChars[CMR_CAN_GEAR_UNKNOWN];

        const char *dispFmt = stateFmt[0];
        switch (stateVSM) {
            case CMR_CAN_UNKNOWN:
            case CMR_CAN_GLV_ON:
            case CMR_CAN_HV_EN:
            case CMR_CAN_RTD:
            case CMR_CAN_ERROR:
            case CMR_CAN_CLEAR_ERROR:
                dispFmt = stateFmt[stateVSM];
                break;
            default:
                break;
        }

        // Need space for NUL-terminator from snprintf().
        char disp[DISPLAY_LEN + 1];
        snprintf(
            disp, sizeof(disp), dispFmt,
            gearChar, canHVCPackVoltage->hvVoltage
        );

        segmentsWrite(disp, DISPLAY_LEN);
        cmr_gpioToggle(GPIO_LED_STATUS);
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
    segmentsInit();

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

