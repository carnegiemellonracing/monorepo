/**
 * @file fans.c
 * @brief Fan Control implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "can.h"        // Board-specific CAN interface
#include "gpio.h"       // Board-specific GPIO interface
#include "fans.h"        // Interface to implement
#include <CMR/pwm.h>        // PWM interface
#include <CMR/gpio.h>       // GPIO interface

/** @brief PWM driver state. */
static cmr_pwm_t channel_1_PWM;
static cmr_pwm_t channel_2_PWM;
static cmr_pwm_t channel_3_PWM;

/** @brief Pump control task priority. */
static const uint32_t pumpControl_priority = 4;
/** @brief Pump control task period. */
static const TickType_t pumpControl_period_ms = 50;
/** @brief Pump control task. */
static cmr_task_t pumpControl_task;

/**
 * @brief Task for controlling the pumps.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void pumpControl(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    /* Enable the half bridges so that output isn't floating */
        cmr_gpioWrite(GPIO_CHANNEL_1_ENABLE, 1);
        cmr_gpioWrite(GPIO_CHANNEL_2_ENABLE, 1);
        cmr_gpioWrite(GPIO_CHANNEL_3_ENABLE, 1);

        /* Get reference to VSM Heartbeat */
        volatile cmr_canHeartbeat_t *vsmHeartbeat = canGetPayload(CANRX_HEARTBEAT_VSM);

        //cmr_canHeartbeat_t *heartbeat = &heartbeat;

        /* Initialize PWM channels to 25kHz for fan control lines */
        /* 96Mhz / (24 * 40000) = 100Hz */
        /*
        const cmr_pwmPinConfig_t pwmPinConfig1 = {
            .port = GPIOB,
            .pin = GPIO_PIN_5,
            .channel = TIM_CHANNEL_2,
            .presc = 24,
            .period_ticks = 40000,
            .timer = TIM3
        };
        */
        const cmr_pwmPinConfig_t pwmPinConfig1 = {
            .port = GPIOB,
            .pin = GPIO_PIN_7,
            .channel = TIM_CHANNEL_2, //TODO
            .presc = 24,
            .period_ticks = 40000,
            .timer = TIM4
        };
        const cmr_pwmPinConfig_t pwmPinConfig2 = {
            .port = GPIOB,
            .pin = GPIO_PIN_8,
            .channel = TIM_CHANNEL_3, //TODO
            .presc = 24,
            .period_ticks = 40000,
            .timer = TIM4
        };
        cmr_pwmInit(&channel_1_PWM, &pwmPinConfig1);
        cmr_pwmInit(&channel_2_PWM, &pwmPinConfig2);
        // cmr_pwmInit(&channel_3_PWM, &pwmPinConfig3);

        TickType_t lastWakeTime = xTaskGetTickCount();
        while (1) {

            switch (heartbeat.state) {
                case CMR_CAN_RTD:
                /*
                    battTemp = sensor[SENSOR_CH_BOARD_THERM_1].value;
                    if (battTemp) > 60
                        pwmDutyCycle = 100
                    else
                        pwmDutyCycle = 30
                */
                    channel_1_State = 100;
                    channel_2_State = 100;
                    // channel_3_State = 100;
                    cmr_pwmSetDutyCycle(&channel_1_PWM, channel_1_State);
                    cmr_pwmSetDutyCycle(&channel_2_PWM, channel_2_State);
                    // cmr_pwmSetDutyCycle(&channel_3_PWM, channel_3_State);
                    cmr_gpioWrite(GPIO_CHANNEL_1_ENABLE, 1);
                    cmr_gpioWrite(GPIO_CHANNEL_2_ENABLE, 1);
                    // cmr_gpioWrite(GPIO_CHANNEL_3_ENABLE, 1);
                    break;
                case CMR_CAN_HV_EN:
                    channel_1_State = 100;
                    channel_2_State = 100;
                    // channel_3_State = 100;
                    cmr_pwmSetDutyCycle(&channel_1_PWM, channel_1_State);
                    cmr_pwmSetDutyCycle(&channel_2_PWM, channel_2_State);
                    // cmr_pwmSetDutyCycle(&channel_3_PWM, channel_3_State);
                    cmr_gpioWrite(GPIO_CHANNEL_1_ENABLE, 1);
                    cmr_gpioWrite(GPIO_CHANNEL_2_ENABLE, 1);
                    // cmr_gpioWrite(GPIO_CHANNEL_3_ENABLE, 1);
                    break;
                default:
                    channel_1_State = 0;
                    channel_2_State = 0;
                    // channel_3_State = 0;
                    cmr_pwmSetDutyCycle(&channel_1_PWM, channel_1_State);
                    cmr_pwmSetDutyCycle(&channel_2_PWM, channel_2_State);
                    // cmr_pwmSetDutyCycle(&channel_3_PWM, channel_3_State);
                    cmr_gpioWrite(GPIO_CHANNEL_1_ENABLE, 1);
                    cmr_gpioWrite(GPIO_CHANNEL_2_ENABLE, 1);
                    // cmr_gpioWrite(GPIO_CHANNEL_3_ENABLE, 1);
                    break;
            }

            vTaskDelayUntil(&lastWakeTime, pumpControl_period_ms);
        }
}

/**
 * @brief Initialization of pump control task.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
void pumpInit() {
    cmr_taskInit(
        &pumpControl_task,
        "pumpControl",
        pumpControl_priority,
        pumpControl,
        NULL
    );
}
