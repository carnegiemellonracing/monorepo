 /**
 * @file main.c
 * @brief Firmware entry point.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h> // HAL interface

#include <CMR/panic.h>  // cmr_panic()
#include <CMR/rcc.h>    // RCC interface
#include <CMR/can.h>      // CAN interface
#include <CMR/adc.h>    // ADC interface
#include <CMR/gpio.h>   // GPIO interface
#include <CMR/tasks.h>  // Task interface
#include <CMR/uart.h>       // CMR UART interface
#include <stdlib.h>

#include "gpio.h"       // Board-specific GPIO interface
#include "can.h"        // Board-specific CAN interface
#include "adc.h"        // Board-specific ADC interface
#include "sensors.h"    // Board-specific sensors interface
#include "motors.h"     // Board-specific motors interface
#include "i2c.h"
#include "servo.h"
#include "lut.h"
#include "brakelight.h"
#include "pumps.h"
#include "steering.h"
#include "optimizer.h"


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
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    // System initialization.
    HAL_Init();
    srand(HAL_GetTick());
    cmr_rccSystemClockEnable();

    // time_OSQPControls(0, 0, 0, 0, 0, 0, true, true, false, 0);

    // Peripheral configuration.
    // gpioInit();
    // // i2cInit();
    // canInit();
    // adcInit();
    // brakelightInit();
    // motorsInit();
    // sensorsInit();
    // pumpsOn();
    // steeringInit();

    // cmr_taskInit(&statusLED_task, "statusLED", statusLED_priority, statusLED,
    //             NULL);

    // vTaskStartScheduler();
    // cmr_panic("vTaskStartScheduler returned!");

    optimizer_state_t state = {
      /* areq, mreq */
      .areq = 0.0,
      .mreq = 0.0,

      /* theta bounds */
      .theta_left  = -0.5,
      .theta_right =  0.5,

      /* weight arrays [NUM_VARS = 4] */
      .accel_weights    = { 1.0, 1.0, 1.0, 1.0 },
      .moment_weights   = { 1.0, 1.0, 1.0, 1.0 },
      .diagonal_weights = { 0.1, 0.1, 0.1, 0.1 },
      .omegas           = { 0.0, 0.0, 0.0, 0.0 },

      /* scalar limits */
      .power_limit = 100.0,
      .dim         = NUM_VARS,   /* = 4 */

      /* linear constraint: weights[4] and a scalar limit */
      .new_constraint = {
          .weights = { 1.0, 1.0, 1.0, 1.0 },
          .limit   = 50.0,
      },

      /* quadratic form: Q[4x4], q[4], c */
      .qform = {
          .Q = {
              1.0, 0.0, 0.0, 0.0,   /* row 0 */
              0.0, 1.0, 0.0, 0.0,   /* row 1 */
              0.0, 0.0, 1.0, 0.0,   /* row 2 */
              0.0, 0.0, 0.0, 1.0,   /* row 3 */
          },
          .q = { 0.0, 0.0, 0.0, 0.0 },
          .c = 0.0,
      },

      /* optimum: content[4] and void* link[4] */
      .optimum = {
          .content = { 0.0, 0.0, 0.0, 0.0 },
          .link    = { NULL, NULL, NULL, NULL },
      },

      /* optimal variable assignments [4] */
      .optimal_assignment = {
          { .role = UNCONSTRAINED, .val = 0.0 },
          { .role = UNCONSTRAINED, .val = 0.0 },
          { .role = UNCONSTRAINED, .val = 0.0 },
          { .role = UNCONSTRAINED, .val = 0.0 },
      },

      /* scalar cost result */
      .optimal_cost = 0.0,

      /* box variable profile [4]: lower, upper, role */
      .variable_profile = {
          { .lower = -10.0, .upper = 10.0, .role = UNCONSTRAINED },
          { .lower = -10.0, .upper = 10.0, .role = UNCONSTRAINED },
          { .lower = -10.0, .upper = 10.0, .role = UNCONSTRAINED },
          { .lower = -10.0, .upper = 10.0, .role = UNCONSTRAINED },
      },

      /* inverse of Q: Qinv[4x4], flat row-major */
      .Qinv = {
          1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0,
      },
  };
    const long long ITERATIONS = 1LL; // 1 billion
    volatile double a = 1.00000001;
    volatile double result = 1.0;


    unsigned long t1 = DWT->CYCCNT;


    // for (long long i = 0; i < ITERATIONS; i++) {
    //     result *= a;
    // }

  solve_one_case(&state);
  unsigned long t2 = DWT->CYCCNT;

  unsigned long long elapsed = t2 - t1;
  if (elapsed == 0) {
    cmr_panic("Elapsed time is zero, which is unexpected.");
  }


}
