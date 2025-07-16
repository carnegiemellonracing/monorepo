/**
 * @file main.c
 * @brief Firmware entry point.
 *
 * @author Carnegie Mellon Racing
 */

#include <CMR/adc.h>    // ADC interface
#include <CMR/fdcan.h>  // CAN interface
#include <CMR/gpio.h>   // GPIO interface
#include <CMR/panic.h>  // cmr_panic()
#include <CMR/rcc.h>    // RCC interface
#include <CMR/tasks.h>  // Task interface
#include <CMR/uart.h>   // CMR UART interface
#include <stdlib.h>
#include <stm32h7xx_hal.h>  // HAL interface

#include "adc.h"  // Board-specific ADC interface
#include "brakelight.h"
#include "can.h"   // Board-specific CAN interface
#include "gpio.h"  // Board-specific GPIO interface
#include "i2c.h"
#include "lut.h"
#include "lut_3d.h"
#include "motors.h"  // Board-specific motors interface
#include "qform.h"
#include "sensors.h"  // Board-specific sensors interface
#include "servo.h"

/** @brief Status LED priority. */
static const uint32_t statusLED_priority = 2;

/** @brief Status LED period (milliseconds). */
static const TickType_t statusLED_period_ms = 250;

uint32_t brakeLightThreshold_PSI = 20;

/** @brief Status LED task. */
static cmr_task_t statusLED_task;

#define SWEEP_SIZE 24
const float sweep[SWEEP_SIZE][7] = {
#include "sweep.rawh"
};
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

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        cmr_gpioToggle(GPIO_LED_STATUS);

        // cmr_canVSMSensors_t *vsmSensors =
        // canVehicleGetPayload(CANRX_VSM_SENSORS);

        // if (vsmSensors->brakePressureRear_PSI > brakeLightThreshold_PSI) {
        //         cmr_gpioWrite(GPIO_BRKLT_ENABLE, 1);
        //     } else {
        //         cmr_gpioWrite(GPIO_BRKLT_ENABLE, 0);
        //     }

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
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    // System initialization.
    HAL_Init();
    srand(HAL_GetTick());
    cmr_rccSystemClockEnable();

    // time_OSQPControls(0, 0, 0, 0, 0, 0, true, true, false, 0);

    // Peripheral configuration.

    gpioInit();
    i2cInit();
    canInit();
    adcInit();
    servoInit();
    motorsInit();
    sensorsInit();
    brakelightInit();

    // Tests for computing kappa.
    kappaAndFx results_ref[256];
    Fx_kappa_t results[256];
    for (int i = 0; i < 256; i++) {
        results_ref[i] = getKappaFxGlobalMax(MOTOR_FL, i, true);
        float alpha_deg = 0.0;

        float Fz_N = get_fake_downforce(MOTOR_FL);
        float target_Fx_N = getLUTMaxFx() * ((float)i) / ((float)UINT8_MAX);
        results[i].Fx = target_Fx_N;
        results[i].kappa = lut_get_kappa(alpha_deg, Fz_N, target_Fx_N);
    }

    // Test for computing maximum traction at given downforce.
    float max_Fx_ref[FZ_DIM];
    float max_Fx[FZ_DIM];
    for (int i = 0; i < FZ_DIM; i++) {
        float Fz_N = FZ_MIN_N + FZ_SPACING_N * i;
        max_Fx_ref[i] =
            getKappaFxGlobalMaxAtDownforce(0.4 * Fz_N, UINT8_MAX, true).Fx;

        float alpha_deg = 0.0;
        max_Fx[i] = lut_get_max_Fx_kappa(alpha_deg, Fz_N).Fx;
    }

#define MATS 15
    double pd_matrices[MATS][16] = {
        {4, 1, 1, 1, 1, 4, 1, 1, 1, 1, 4, 1, 1, 1, 1, 4},
        {2, 0, 0, 0, 0, 3, 0, 0, 0, 0, 4, 0, 0, 0, 0, 5},
        {6, 2, 1, 0, 2, 6, 2, 1, 1, 2, 6, 2, 0, 1, 2, 6},
        {5, 1, 0, 1, 1, 5, 1, 0, 0, 1, 5, 1, 1, 0, 1, 5},
        {10, 3, 2, 1, 3, 10, 3, 2, 2, 3, 10, 3, 1, 2, 3, 10},
        {3, 0, 1, 0, 0, 3, 0, 1, 1, 0, 3, 0, 0, 1, 0, 3},
        {7, 1, 2, 0, 1, 7, 1, 2, 2, 1, 7, 1, 0, 2, 1, 7},
        {8, 2, 2, 2, 2, 8, 2, 2, 2, 2, 8, 2, 2, 2, 2, 8},
        {5, 0.5, 0, 0, 0.5, 5, 0.5, 0, 0, 0.5, 5, 0.5, 0, 0, 0.5, 5},
        {6, 0.5, 0.5, 0.5, 0.5, 6, 0.5, 0.5, 0.5, 0.5, 6, 0.5, 0.5, 0.5, 0.5,
         6},
        {9, 3, 0, 0, 3, 9, 3, 0, 0, 3, 9, 3, 0, 0, 3, 9},
        {4, 0.1, 0.2, 0.3, 0.1, 4, 0.1, 0.2, 0.2, 0.1, 4, 0.1, 0.3, 0.2, 0.1,
         4},
        {11, 4, 1, 0, 4, 11, 4, 1, 1, 4, 11, 4, 0, 1, 4, 11},
        {6, 1, 1, 0, 1, 6, 1, 1, 1, 1, 6, 1, 0, 1, 1, 6},
        {12, 0.5, 0.3, 0.2, 0.5, 12, 0.4, 0.3, 0.3, 0.4, 12, 0.5, 0.2, 0.3, 0.5,
         12}};

    float time;

    // Test speed on matrix inversion
    for (size_t i = 0; i < MATS; i++) {
        double bogus[16];

        uint32_t start_tick = HAL_GetTick();

        invert4x4_old(pd_matrices[i], bogus);

        uint32_t end_tick = HAL_GetTick();

        time = (float)(end_tick - start_tick);

        start_tick = HAL_GetTick();

        invert4x4(pd_matrices[i], bogus);

        end_tick = HAL_GetTick();

        time = (float)(end_tick - start_tick);
    }

    cmr_taskInit(&statusLED_task, "statusLED", statusLED_priority, statusLED,
                 NULL);

    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");

    return time;
}
