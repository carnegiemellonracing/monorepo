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

#include "adc.h"        // Board-specific ADC interface
#include "assi.h"
#include "can.h"        // Board-specific CAN interface
#include "dac.h"        // Board-specific DAC interface
#include "error.h"
#include "gpio.h"       // Board-specific GPIO interface
#include "sensors.h"    // Board-specific sensors interface
#include "state.h"      // stateInit()

/** @brief Status LED priority. */
static const uint32_t statusLED_priority = 2;

/** @brief Status LED period (milliseconds). */
static const TickType_t statusLED_period_ms = 250;

/** @brief Status LED task. */
static cmr_task_t statusLED_task;

static bool glvReached = false;

uint16_t LED_Red_State = 0;

static bool LEDerror() {  
  return cmr_gpioRead(GPIO_IN_SOFTWARE_ERR) == 1 || cmr_gpioRead(GPIO_IN_IMD_ERR) == 1;
}

/**
 * @brief Task for toggling the status LED.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void statusLED(void *pvParameters) {
    // cmr_pwmSetDutyCycle(&LED_Red, 0);
    // cmr_pwmSetDutyCycle(&LED_Red, 1);
    // // cmr_pwmSetDutyCycle(&LED_Red, 30);
    // cmr_pwmSetDutyCycle(&LED_Green, 1);
    // // cmr_pwmSetDutyCycle(&LED_Green, 0);
    // cmr_pwmSetDutyCycle(&LED_Green, 30);
    LED_Red_State = 0;
    // cmr_pwmSetDutyCycle(&LED_Green, 100);
    // cmr_gpioToggle(GPIO_OUT_LED_FLASH_RED);

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

        vTaskDelayUntil(&lastWakeTime, statusLED_period_ms);
      // cmr_pwmSetDutyCycle(&LED_Red, 0);
      // cmr_pwmSetDutyCycle(&LED_Green, 0);
      // cmr_gpioWrite(GPIO_OUT_LED_FLASH_RED, 0);

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
    sensorsInit();
    stateInit();
    assiInit();
    dacInit();
    
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

