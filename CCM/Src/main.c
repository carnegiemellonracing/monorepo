/**
 * @file main.c
 * @brief Firmware entry point.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h>  // HAL interface

#include <CMR/panic.h>  // cmr_panic()
#include <CMR/rcc.h>    // RCC interface
#include <CMR/can.h>    // CAN interface
#include <CMR/adc.h>    // ADC interface
#include <CMR/gpio.h>   // GPIO interface
#include <CMR/tasks.h>  // Task interface

#include "gpio.h"    // Board-specific GPIO interface
#include "can.h"     // Board-specific CAN interface
#include "adc.h"     // Board-specific ADC interface
#include "uart.h"    // Board-specific UART interface
#include "sensors.h" // Board-specific sensors interface
#include "state.h"   // Board-specific state machine interface

#include "fans.h"

/** @brief Status LED priority. */
static const uint32_t statusLED_priority = 2;

/** @brief Status LED period (milliseconds). */
static const TickType_t statusLED_period_ms = 500;

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
//        volatile cmr_canHVCPackMinMaxCellVolages_t *canCellVoltage = (void *) canVehicleRXMeta[CANRX_HVC_CELL_VOLTAGE].payload;
//        uint16_t maxCellVoltage_mV = canCellVoltage->maxCellVoltage_mV;
//
//        if (maxCellVoltage_mV > 5000)
//        {
//            cmr_gpioWrite(GPIO_LED_STATUS, 1);
//        }
//        else
//        {
//            cmr_gpioWrite(GPIO_LED_STATUS, 0);
//        }
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
    cmr_rccSystemClockEnable(); // Use HSI - HSE seems broken

    //Comment out lines 67, 69-71, 74 to program the MCU two if ever necessary
    // Peripheral configuration.
    uartInit();
    gpioInit();
    canInit();
    adcInit();
    sensorsInit();

    // Very last one since it depends on the peripherals to be working
    stateInit();

    cmr_taskInit(
        &statusLED_task,
        "statusLED",
        statusLED_priority,
        statusLED,
        NULL
    );

    fanInit();

    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");
}

