/**
 * @file assi.c
 * @brief Autonomous System Status Indicators.
 * 
 * @author Carnegie Mellon Racing
 */

#include "assi.h"      // Interface to implement
#include "can.h"        // Board-specific CAN interface
#include "gpio.h"       // Board-specific GPIO interface
#include "sensors.h"    // Board-specific Sensors interface
#include "state.h"    // Board Specific State Interface
#include "pwm.h"


#include <CMR/pwm.h>           // PWM interface
#include <CMR/sensors.h>       // Sensors interface

/** @brief ASSI control task priority. */
static const uint32_t assiControlPriority = 3; //non safety critical
/** @brief ASSI control task period. */
static const TickType_t assiControl_period_ms = 50;
/** @brief ASSI control task. */
static cmr_task_t assiControl_task;

/**
 * @brief Task for controlling the ASSI.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void assiControl(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    /* Initialize PWM channels for ASSI */
    /* 96 * 10 ** 6 hz / ((1 * 10 ** 4) * (3.2 * 10 ** 3)) = 3.2 hz */

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
    	cmr_canState_t state = getCurrentState();
        // cmr_gpioWrite(GPIO_OUT_LED_BLUE, 1);
        // cmr_gpioWrite(GPIO_OUT_LED_YELLOW, 0);

        switch (state) {
            case CMR_CAN_AS_READY: 
              pwmSetDutyCycle(PWM_BLUE, (uint32_t) 0);
              pwmSetDutyCycle(PWM_YELLOW, (uint32_t) 100);
              break;

            case CMR_CAN_AS_DRIVING: 
              pwmSetDutyCycle(PWM_BLUE, (uint32_t) 0);
              pwmSetDutyCycle(PWM_YELLOW, (uint32_t) 50);
              break;

            case CMR_CAN_AS_EMERGENCY: 
              pwmSetDutyCycle(PWM_BLUE, (uint32_t) 100);
              pwmSetDutyCycle(PWM_YELLOW, (uint32_t) 0);
              break;

            case CMR_CAN_AS_FINISHED: 
                pwmSetDutyCycle(PWM_BLUE, (uint32_t) 50);
                pwmSetDutyCycle(PWM_YELLOW, (uint32_t) 0);
                break;
            
            default:
              pwmSetDutyCycle(PWM_BLUE, (uint32_t) 0);
              pwmSetDutyCycle(PWM_YELLOW, (uint32_t) 0);
              break;
        }

        vTaskDelayUntil(&lastWakeTime, assiControl_period_ms);
    }
}

/**
 * @brief Initialization of ASSI control task.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
void assiInit() {
    cmr_taskInit(
        &assiControl_task,
        "ASSIControl",
        assiControlPriority,
        assiControl,
        NULL
    );
}