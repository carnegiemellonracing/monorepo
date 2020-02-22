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

/** @brief Fan control task priority. */
static const uint32_t fanControl_priority = 4;
/** @brief Fan control task period. */
static const TickType_t fanControl_period_ms = 50;
/** @brief Fan control task. */
static cmr_task_t fanControl_task;

/**
 * @brief Task for controlling the fans.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void fanControl(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    /* Enable the half bridges so that output isn't floating */
    cmr_gpioWrite(GPIO_CHANNEL_1_ENABLE, 1);
    cmr_gpioWrite(GPIO_CHANNEL_2_ENABLE, 1);
    cmr_gpioWrite(GPIO_CHANNEL_3_ENABLE, 1);

    /* Get reference to VSM Heartbeat and accumulator min/max cell temperatures */
    volatile cmr_canHeartbeat_t *vsmHeartbeat = canGetPayload(CANRX_HEARTBEAT_VSM);
    //volatile cmr_canHVCPackMinMaxCellTemps_t *minMaxTemps = canGetPayload(CANRX_HVC_MINMAX_TEMPS);

    //cmr_canHeartbeat_t *heartbeat = &heartbeat;

    /* Initialize PWM channels to 25kHz for fan control lines */
    /* 96Mhz / (24 * 160) = 25kHz */
    const cmr_pwmPinConfig_t pwmPinConfig1 = {
        .port = GPIOB,
        .pin = GPIO_PIN_5,
        .channel = TIM_CHANNEL_2,
        .presc = 24,
        .period_ticks = 160,
        .timer = TIM3
    };
    const cmr_pwmPinConfig_t pwmPinConfig2 = {
        .port = GPIOB,
        .pin = GPIO_PIN_7,
        .channel = TIM_CHANNEL_2,
        .presc = 24,
        .period_ticks = 160,
        .timer = TIM4
    };
    const cmr_pwmPinConfig_t pwmPinConfig3 = {
        .port = GPIOB,
        .pin = GPIO_PIN_8,
        .channel = TIM_CHANNEL_3,
        .presc = 24,
        .period_ticks = 160,
        .timer = TIM4
    };
    cmr_pwmInit(&channel_1_PWM, &pwmPinConfig1);
    cmr_pwmInit(&channel_2_PWM, &pwmPinConfig2);
    cmr_pwmInit(&channel_3_PWM, &pwmPinConfig3);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {

        switch (heartbeat.state) {
            case CMR_CAN_RTD:
                channel_1_State = 100;
                channel_2_State = 100;
                channel_3_State = 100;
                cmr_pwmSetDutyCycle(&channel_1_PWM, channel_1_State);
                cmr_pwmSetDutyCycle(&channel_2_PWM, channel_2_State);
                cmr_pwmSetDutyCycle(&channel_3_PWM, channel_3_State);
                cmr_gpioWrite(GPIO_CHANNEL_1_ENABLE, 1);
                cmr_gpioWrite(GPIO_CHANNEL_2_ENABLE, 1);
                cmr_gpioWrite(GPIO_CHANNEL_3_ENABLE, 1);
                break;
            case CMR_CAN_HV_EN:
                channel_1_State = 50;
                channel_2_State = 50;
                channel_3_State = 50;
                cmr_pwmSetDutyCycle(&channel_1_PWM, channel_1_State);
                cmr_pwmSetDutyCycle(&channel_2_PWM, channel_2_State);
                cmr_pwmSetDutyCycle(&channel_3_PWM, channel_3_State);
                cmr_gpioWrite(GPIO_CHANNEL_1_ENABLE, 1);
                cmr_gpioWrite(GPIO_CHANNEL_2_ENABLE, 1);
                cmr_gpioWrite(GPIO_CHANNEL_3_ENABLE, 1);
                break;
            default:
                channel_1_State = 0;
                channel_2_State = 0;
                channel_3_State = 0;
                cmr_pwmSetDutyCycle(&channel_1_PWM, channel_1_State);
                cmr_pwmSetDutyCycle(&channel_2_PWM, channel_2_State);
                cmr_pwmSetDutyCycle(&channel_3_PWM, channel_3_State);
                cmr_gpioWrite(GPIO_CHANNEL_1_ENABLE, 1);
                cmr_gpioWrite(GPIO_CHANNEL_2_ENABLE, 1);
                cmr_gpioWrite(GPIO_CHANNEL_3_ENABLE, 1);
                break;
        }

        vTaskDelayUntil(&lastWakeTime, fanControl_period_ms);
    }
}

/**
 * @brief Initialization of fan control task.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
void fanInit() {
    cmr_taskInit(
        &fanControl_task,
        "fanControl",
        fanControl_priority,
        fanControl,
        NULL
    );
}
