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
#include "main.h"       // DIM-specific definitions
#include "segments.h"   // Segment display interface

/** @brief Display length. */
#define DISPLAY_LEN 8

/**======================================**/
/**              STATUS LED              **/
/**======================================**/
/** @brief Status LED priority. */
static const uint32_t statusLED_priority = 2;

/** @brief Status LED period (milliseconds). */
static const TickType_t statusLED_period_ms = 250;

/** @brief Status LED task. */
static cmr_task_t statusLED_task;

volatile cmr_canState_t VSM_state = CMR_CAN_UNKNOWN;
volatile cmr_canState_t DIM_requested_state = CMR_CAN_GLV_ON;
volatile cmr_canGear_t DIM_gear = CMR_CAN_GEAR_FAST;
volatile cmr_canGear_t DIM_newGear = CMR_CAN_GEAR_FAST;
volatile cmr_canGear_t DIM_oldGear = CMR_CAN_GEAR_FAST;

volatile int32_t HVC_pack_voltage = 0;

static const char *segmentText[] = {
    "????????",
    "GLV ON  ",
    "HV %c %.3u",
    "RD %c %.3u",
    "ERROR   ",
    "C_ERROR ",
};

/**
 * @brief Task for toggling the status LED.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void statusLED(void *pvParameters) {
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

    TickType_t lastWakeTime = xTaskGetTickCount();

    //static const uint32_t pack_voltage  = 447;

    while (1) {
        uint8_t gearChar = (DIM_gear < CMR_CAN_GEAR_LEN)
            ? gearChars[DIM_gear]
            : gearChars[CMR_CAN_GEAR_UNKNOWN];

        // Need space for NUL-terminator from snprintf().
        char disp[DISPLAY_LEN + 1];
        switch (VSM_state) {
            case CMR_CAN_UNKNOWN:
            case CMR_CAN_GLV_ON:
            case CMR_CAN_HV_EN:
            case CMR_CAN_RTD:
            case CMR_CAN_ERROR:
            case CMR_CAN_CLEAR_ERROR:
                snprintf(disp, sizeof(disp), segmentText[VSM_state], gearChar, HVC_pack_voltage);
                break;
            default:
                snprintf(disp, sizeof(disp), segmentText[0], gearChar, HVC_pack_voltage);
                break;
        }
        segmentsWrite(disp, DISPLAY_LEN);
        cmr_gpioToggle(GPIO_LED_STATUS);
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

