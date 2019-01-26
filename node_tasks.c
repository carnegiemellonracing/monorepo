// ------------------------------------------------------------------------------------------------
// Includes

#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"

#include "node_tasks.h"
#include "pin_definitions.h"
#include "adc.h"

// ------------------------------------------------------------------------------------------------
// Task priorities and periods

/**
 *      Priority    Freq (Hz)   Period (ms)
 *      -----------------------------------
 *      7           1000        1
 *      6           100-999     calculate
 *      5           100         10
 *      4           11-99       calculate
 *      3           10          100
 *      2           2-9         calculate
 *      1           1           1000
 */

// Globally accessible priorities
const uint32_t          mcuStatusLEDTaskPriority  = 2;
const uint32_t          adcTaskPriority           = 5;

// Locally accessible periods
static const TickType_t MCUStatusLEDTaskPeriod_ms = 250;
static const TickType_t adcTaskPeriod_ms          = 10;

// ------------------------------------------------------------------------------------------------
// Tasks

/**
 * @brief Blinks the MCU Status LED to indicate that the RTOS is running.
 *
 * @param pvParameters      Unused
 */
void mcuStatusLED_task(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();

    HAL_GPIO_WritePin(mcuStatusLEDPort, mcuStatusLEDPin, RESET);

    for (;;) {
        HAL_GPIO_TogglePin(mcuStatusLEDPort, mcuStatusLEDPin);

        vTaskDelayUntil(&lastWakeTime, MCUStatusLEDTaskPeriod_ms);
    }
}

/**
 * @brief What does this do?
 *
 * @param pvParameters      Unused
 */
void adc_task(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();

    for (;;) {
        // ADC set up in discontinuous scan mode.
        // Each HAL_ADC_Start call converts the next channel in
        // the sequence defined in adc.c
        for (uint32_t i = 0; i < NUM_ADC_CHANNELS; i++) {
            HAL_ADC_Start(&adcHandle);
            HAL_ADC_PollForConversion(&adcHandle, 1);
            adcVals[i] = HAL_ADC_GetValue(&adcHandle);
        }

        vTaskDelayUntil(&lastWakeTime, adcTaskPeriod_ms);
    }
}


