/**
 * @file tssi.c
 * @brief Tractive System Status Indicators.
 * 
 * @author Carnegie Mellon Racing
 */

#include "tssi.h"      // Interface to implement
#include "gpio.h"       // Board-specific GPIO interface
#include "state.h"    // Board Specific State Interface
#include "pwm.h"


#include <CMR/pwm.h>           // PWM interface
#include <CMR/tasks.h>

/** @brief TSSI control task priority. */
static const uint32_t tssiControlPriority = 2;
/** @brief TSSI control task period. */
static const TickType_t tssiControl_period_ms = 250;
/** @brief TSSI control task. */
static cmr_task_t tssiControl_task;

static bool glvReached = false;

static bool LEDError() {  
  return cmr_gpioRead(GPIO_IN_SOFTWARE_ERR) == 1 || cmr_gpioRead(GPIO_IN_IMD_ERR) == 1
    || cmr_gpioRead(GPIO_IN_IMD_ERR_COND) == 1;
}

/**
 * @brief Task for controlling the TSSI.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void tssiControl(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();

    pwmSetDutyCycle(PWM_GREEN, 50);
    pwmSetDutyCycle(PWM_RED, 0);

    while (1) {
      if (getCurrentState() == CMR_CAN_VSM_STATE_GLV_ON) {
        glvReached = true;
      }

      if(glvReached) {
        if(LEDError()) {
          pwmSetDutyCycle(PWM_RED, 150);
          pwmSetDutyCycle(PWM_GREEN, 0);
        }
        else if(getCurrentState() != CMR_CAN_VSM_STATE_ERROR) {
          pwmSetDutyCycle(PWM_GREEN, 50);
          pwmSetDutyCycle(PWM_RED, 0);
        }
      }

      vTaskDelayUntil(&lastWakeTime, tssiControl_period_ms);
    }
}

/**
 * @brief Initialization of ASSI control task.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
void tssiInit() {
    cmr_taskInit(
        &tssiControl_task,
        "TSSIControl",
        tssiControlPriority,
        tssiControl,
        NULL
    );
}