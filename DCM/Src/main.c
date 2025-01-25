 /**
 * @file main.c
 * @brief Firmware entry point.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32h7xx_hal.h> // HAL interface

#include <CMR/panic.h>  // cmr_panic()
#include <CMR/rcc.h>    // RCC interface
#include <CMR/fdcan.h>    // CAN interface
#include <CMR/adc.h>    // ADC interface
#include <CMR/gpio.h>   // GPIO interface
#include <CMR/tasks.h>  // Task interface
#include <CMR/uart.h>       // CMR UART interface

#include "gpio.h"       // Board-specific GPIO interface
#include "can.h"        // Board-specific CAN interface
#include "adc.h"        // Board-specific ADC interface
#include "sensors.h"    // Board-specific sensors interface
#include "motors.h"     // Board-specific motors interface
#include "i2c.h"
#include "servo.h"

#include "cvxgen_interface.h"

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

    cmr_canVSMSensors_t *vsmSensors = canVehicleGetPayload(CANRX_VSM_SENSORS);

    if (vsmSensors->brakePressureRear_PSI > brakeLightThreshold_PSI) {
            cmr_gpioWrite(GPIO_BRKLT_ENABLE, 1);
        } else {
            cmr_gpioWrite(GPIO_BRKLT_ENABLE, 0);
        }

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
    cmr_rccSystemClockEnable();

    // time_OSQPControls(0, 0, 0, 0, 0, 0, true, true, false, 0);

    // Peripheral configuration.

    gpioInit();
    i2cInit();
    MCP3202_init();
    canInit();
    adcInit();
    servoInit();
    motorsInit();
    sensorsInit();
    veInit();
    stateInit();

    cmr_taskInit(&statusLED_task, "statusLED", statusLED_priority, statusLED,
                NULL);

    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");
}
