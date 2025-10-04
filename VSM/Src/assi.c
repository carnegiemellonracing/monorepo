/**
 * @file assi.c
 * @brief Autonomous System Status Indicators.
 * 
 * @author Carnegie Mellon Racing
 */

#include "can.h"        // Board-specific CAN interface
#include "gpio.h"       // Board-specific GPIO interface
#include "assi.h"       // Interface to implement
#include <CMR/pwm.h>    // PWM interface
#include <CMR/sensors.h>       // Sensors interface
#include "sensors.h"


/** @brief PWM driver state. */
static cmr_pwm_t BLUE_ASSI_PWM;
static cmr_pwm_t YELLOW_ASSI_PWM;

/** @brief Pump control task priority. */
static const uint32_t assiControlPriority = 3; //non safety critical
/** @brief Pump control task period. */
static const TickType_t assiControl_period_ms = 50;
/** @brief Pump control task. */
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
    const cmr_pwmPinConfig_t pwmPinConfig1 = {
        .port = GPIOA,
        .pin = GPIO_PIN_8,
        .channel = TIM_CHANNEL_2,
        .presc = 10000,
        .period_ticks = 3200,
        .timer = TIM3
    };
    const cmr_pwmPinConfig_t pwmPinConfig2 = {
        .port = GPIOB,
        .pin = GPIO_PIN_10,
        .channel = TIM_CHANNEL_3,
        .presc = 10000,
        .period_ticks = 3200,
        .timer = TIM2
    };
    cmr_pwmInit(&YELLOW_ASSI_PWM, &pwmPinConfig1);
    cmr_pwmInit(&BLUE_ASSI_PWM, &pwmPinConfig2);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
    	cmr_canState_t state = canGetCurrentState();

        switch (state) {
            case CMR_CAN_AS_READY: 
              cmr_pwmSetDutyCycle(&BLUE_ASSI_PWM, (uint32_t) 0);
              cmr_pwmSetDutyCycle(&YELLOW_ASSI_PWM, (uint32_t) 100);
              break;

            case CMR_CAN_AS_DRIVING: 
              cmr_pwmSetDutyCycle(&BLUE_ASSI_PWM, (uint32_t) 0);
              cmr_pwmSetDutyCycle(&YELLOW_ASSI_PWM, (uint32_t) 50);
              break;

            case CMR_CAN_AS_EMERGENCY: 
              cmr_pwmSetDutyCycle(&BLUE_ASSI_PWM, (uint32_t) 100);
              cmr_pwmSetDutyCycle(&YELLOW_ASSI_PWM, (uint32_t) 0);
              break;

            case CMR_CAN_AS_FINISHED: 
                cmr_pwmSetDutyCycle(&BLUE_ASSI_PWM, (uint32_t) 50);
                cmr_pwmSetDutyCycle(&YELLOW_ASSI_PWM, (uint32_t) 0);
                break;
            
            default:
              cmr_pwmSetDutyCycle(&BLUE_ASSI_PWM, (uint32_t) 0);
              cmr_pwmSetDutyCycle(&YELLOW_ASSI_PWM, (uint32_t) 0);
              break;
        }

        vTaskDelayUntil(&lastWakeTime, assiControl_period_ms);
    }
}

/**
 * @brief Initialization of pump control task.
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