/**
 * @file tssi.c
 * @brief Tractive System Status Indicators.
 * 
 * @author Carnegie Mellon Racing
 */

#include "tssi.h"      // Interface to implement
#include "gpio.h"       // Board-specific GPIO interface
#include "state.h"    // Board Specific State Interface


#include <CMR/pwm.h>           // PWM interface
#include <CMR/tasks.h>

/** @brief PWM driver state. */
static cmr_pwm_t RED_TSSI_PWM;
static cmr_pwm_t GREEN_TSSI_PWM;

/** @brief TSSI control task priority. */
static const uint32_t tssiControlPriority = 2;
/** @brief TSSI control task period. */
static const TickType_t tssiControl_period_ms = 250;
/** @brief TSSI control task. */
static cmr_task_t tssiControl_task;

static bool glvReached = false;

static bool LEDerror() {  
  return cmr_gpioRead(GPIO_IN_SOFTWARE_ERR) == 1 || cmr_gpioRead(GPIO_IN_IMD_ERR) == 1;
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

    static uint16_t LED_Red_State = 0;

    TickType_t lastWakeTime = xTaskGetTickCount();
    TickType_t lastFlashTime = lastWakeTime;
    bool flashed = false;

    cmr_pwmInit(&LED_Red, &pwmPinConfig2);
    cmr_pwmInit(&LED_Green, &pwmPinConfig1);
    cmr_pwmSetDutyCycle(&LED_Green, 15);
    cmr_pwmSetDutyCycle(&LED_Red, 0);

    LED_Red_State = 0;

    while (1) {
      if (getCurrentState() == CMR_CAN_VSM_STATE_GLV_ON) {
        glvReached = true;
      }

      // vTaskDelayUntil(&lastWakeTime, 500);

      if (glvReached) {
        if (LEDerror()) {
        TickType_t currentTick = xTaskGetTickCount();
        if (!flashed || currentTick - lastFlashTime >= 150) {
          if(LED_Red_State == 0) {
            cmr_pwmSetDutyCycle(&LED_Red, 15);
            LED_Red_State = 1;
          }
          else {
            cmr_pwmSetDutyCycle(&LED_Red, 0);
            LED_Red_State = 0;
          }
          // cmr_gpioToggle(GPIO_OUT_LED_FLASH_RED);
          lastFlashTime = currentTick;
          flashed = true;
        }

        cmr_pwmSetDutyCycle(&LED_Green, 0);
        // cmr_gpioWrite(GPIO_OUT_LED_GREEN, 0);
       }
      
        else if (getCurrentState() != CMR_CAN_VSM_STATE_ERROR) {
          cmr_pwmSetDutyCycle(&LED_Green, 15);
          cmr_pwmSetDutyCycle(&LED_Red, 0);
          // cmr_gpioWrite(GPIO_OUT_LED_GREEN, 1);
          flashed = false;
        }
      }

      cmr_gpioToggle(GPIO_OUT_LED_STATUS);

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