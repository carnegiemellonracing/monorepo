/**
 * @file main.c
 * @brief Firmware entry point.
 *
 * @author Carnegie Mellon Racing
 */

/**
 * @brief
 * ==========================================================
 *                      IMPORTANT NOTICE
 * ==========================================================
 *
 * This code is relevant to two different systems.
 *
 * There is a PTCf (fan control board) dedicated to driving
 * the fans on the vehicle.
 *
 * There is a PTCp (pump control board) dedicated to driving
 * the pumps on the vehicle.
 *
 * Selection of either PTCp or PTCf firmware occurs in the
 * can.h file with "CMR_PTC_ID"
 *
 * As such, it is important that you are aware of which
 * board you are working with when attempting to flash code.
 *
 * NEGLIGENCE COULD RESULT IN DAMAGE TO HARDWARE!
 *
 *
 * =========================================================
 *                         END NOTICE
 * =========================================================
 */

#include <stm32f4xx_hal.h>  // HAL interface

#include <CMR/panic.h>      // cmr_panic()
#include <CMR/rcc.h>        // RCC interface
#include <CMR/can.h>        // CAN interface
#include <CMR/adc.h>        // ADC interface
#include <CMR/gpio.h>       // GPIO interface
#include <CMR/tasks.h>      // Task interface
#include <CMR/pwm.h>        // PWM interface
#include <CMR/can_types.h>  // CMR CAN types

#include "gpio.h"       // Board-specific GPIO interface
#include "can.h"        // Board-specific CAN interface
#include "adc.h"        // Board-specific ADC interface
#include "sensors.h"    // Board-specific sensors interface

#include "brakelight.h"
#include "fans.h"
#include "pumps.h"
#include "mc_power.h"

/** @brief Status LED task priority. */
static const uint32_t statusLED_priority = 2;
/** @brief Status LED task period. */
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

    cmr_gpioWrite(GPIO_LED_STATUS, 0);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
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
    adcInit();
    canInit();
    sensorsInit();

    cmr_taskInit(
        &statusLED_task,
        "statusLED",
        statusLED_priority,
        statusLED,
        NULL
    );

    // CMR_PTC_ID is defined in can.h
#ifndef CMR_PTC_ID
#error "No PTC ID defined!"
#elif (CMR_PTC_ID == 0) /* Pump Control Board */

    mcPowerInit();
    brakelightInit();
    pumpInit();

#elif (CMR_PTC_ID == 1) /* Fan Control Board */

    fanInit();

#else

    #pragma warning "CMR_PTC_ID is not a valid value!"

#endif

    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");
}

