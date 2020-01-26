/**
 * @file main.c
 * @brief Firmware entry point.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h>  // HAL interface

#include <CMR/panic.h>      // cmr_panic()
#include <CMR/rcc.h>        // RCC interface
#include <CMR/can.h>        // CAN interface
#include <CMR/adc.h>        // ADC interface
#include <CMR/gpio.h>       // GPIO interface
#include <CMR/tasks.h>      // Task interface
#include <CMR/pwm.h>        // PWM interface
#include <CMR/can_types.h>  // CMR CAN types

#include "gpio.h"       // Board-specific GPIO interface
#include "can.h"        // Board-specific CAN interface
#include "adc.h"        // Board-specific ADC interface
#include "sensors.h"    // Board-specific sensors interface

/** @brief PWM driver state. */
static cmr_pwm_t channel_1_PWM;
static cmr_pwm_t channel_2_PWM;
static cmr_pwm_t channel_3_PWM;

/** @brief Cell temperature at which PTC begins commanding AFC max cooling. */
//static const uint32_t maxCoolingEnableTemp_C = 55;
/** @brief Cell temperature at which PTC stops commanding AFC max cooling. */
//static const uint32_t minCoolingDisableTemp_C = 52;

/** @brief Status LED task priority. */
static const uint32_t statusLED_priority = 2;
/** @brief Status LED task period. */
static const TickType_t statusLED_period_ms = 250;
/** @brief Status LED task. */
static cmr_task_t statusLED_task;

/** @brief Motor Controller Power Control task priority. */
static const uint32_t mcPowerControl_priority = 2;
/** @brief Motor Controller Power Control task period. */
static const TickType_t mcPowerControl_period_ms = 50;
/** @brief Motor Controller Power Control task. */
static cmr_task_t mcPowerControl_task;

/** @brief Pump control task priority. */
static const uint32_t pumpControl_priority = 4;
/** @brief Pump control task period. */
static const TickType_t pumpControl_period_ms = 50;
/** @brief Pump control task. */
static cmr_task_t pumpControl_task;

/** @brief Fan control task priority. */
static const uint32_t fanControl_priority = 4;
/** @brief Fan control task period. */
static const TickType_t fanControl_period_ms = 50;
/** @brief Fan control task. */
static cmr_task_t fanControl_task;

/** @brief Brake light threshold (pounds-per-square-inch). */
static const uint16_t brakeLightThreshold_PSI = 20;

/** @brief Brake light control task priority. */
static const uint32_t brakelight_priority = 4;
/** @brief Brake light control task period. */
static const TickType_t brakelight_period_ms = 50;
/** @brief Brake light control task. */
static cmr_task_t brakelight_task;

/**
 * @brief Task for toggling the status LED.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void statusLED(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    cmr_gpioWrite(GPIO_LED_STATUS, 0);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        cmr_gpioToggle(GPIO_LED_STATUS);

        vTaskDelayUntil(&lastWakeTime, statusLED_period_ms);
    }
}

/**
 * @brief Task for controller power to motor controller.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void mcPowerControl(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    /* Get reference to VSM Heartbeat */
    volatile cmr_canHeartbeat_t *vsmHeartbeat = canGetPayload(CANRX_HEARTBEAT_VSM);

    cmr_gpioWrite(GPIO_MTR_CTRL_ENABLE, 0);
    cmr_gpioWrite(GPIO_MC_EFUSE_AUTO, 0);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        switch (vsmHeartbeat->state) {
            case CMR_CAN_RTD:
                cmr_gpioWrite(GPIO_MTR_CTRL_ENABLE, 1);
                break;
            case CMR_CAN_HV_EN:
                cmr_gpioWrite(GPIO_MTR_CTRL_ENABLE, 1);         // This code is subject to change, giving mc 600V before logic voltage is spooky.
                break;                                          // Proposed solution:
            default:                                            // VSM sends ping to PTC to turn on mc logic voltage
                cmr_gpioWrite(GPIO_MTR_CTRL_ENABLE, 0);         // after VSM sees MC can data via CDC it does precharge
                break;
        }
        vTaskDelayUntil(&lastWakeTime, mcPowerControl_period_ms);
    }
}

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

        switch (vsmHeartbeat->state) {
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
        cmr_gpioWrite(GPIO_CHANNEL_3_ENABLE, 0);

        /* Get reference to VSM Heartbeat */
        volatile cmr_canHeartbeat_t *vsmHeartbeat = canGetPayload(CANRX_HEARTBEAT_VSM);

        /* Initialize PWM channels to 25kHz for fan control lines */
        /* 96Mhz / (24 * 40000) = 100Hz */
        const cmr_pwmPinConfig_t pwmPinConfig1 = {
            .port = GPIOB,
            .pin = GPIO_PIN_5,
            .channel = TIM_CHANNEL_2,
            .presc = 24,
            .period_ticks = 40000,
            .timer = TIM3
        };
        const cmr_pwmPinConfig_t pwmPinConfig2 = {
            .port = GPIOB,
            .pin = GPIO_PIN_7,
            .channel = TIM_CHANNEL_2,
            .presc = 24,
            .period_ticks = 40000,
            .timer = TIM4
        };
        const cmr_pwmPinConfig_t pwmPinConfig3 = {
            .port = GPIOB,
            .pin = GPIO_PIN_8,
            .channel = TIM_CHANNEL_3,
            .presc = 24,
            .period_ticks = 40000,
            .timer = TIM4
        };
        cmr_pwmInit(&channel_1_PWM, &pwmPinConfig1);
        cmr_pwmInit(&channel_2_PWM, &pwmPinConfig2);
        cmr_pwmInit(&channel_3_PWM, &pwmPinConfig3);

        TickType_t lastWakeTime = xTaskGetTickCount();
        while (1) {

            switch (vsmHeartbeat->state) {
                case CMR_CAN_RTD:
                    channel_1_State = 100;
                    channel_2_State = 100;
                    channel_3_State = 100;
                    cmr_pwmSetDutyCycle(&channel_1_PWM, channel_1_State);
                    cmr_pwmSetDutyCycle(&channel_2_PWM, channel_2_State);
                    cmr_pwmSetDutyCycle(&channel_3_PWM, channel_3_State);
                    cmr_gpioWrite(GPIO_CHANNEL_1_ENABLE, 1);
                    cmr_gpioWrite(GPIO_CHANNEL_2_ENABLE, 1);
                    cmr_gpioWrite(GPIO_CHANNEL_3_ENABLE, 0);
                    break;
                case CMR_CAN_HV_EN:
                    channel_1_State = 100;
                    channel_2_State = 100;
                    channel_3_State = 100;
                    cmr_pwmSetDutyCycle(&channel_1_PWM, channel_1_State);
                    cmr_pwmSetDutyCycle(&channel_2_PWM, channel_2_State);
                    cmr_pwmSetDutyCycle(&channel_3_PWM, channel_3_State);
                    cmr_gpioWrite(GPIO_CHANNEL_1_ENABLE, 1);
                    cmr_gpioWrite(GPIO_CHANNEL_2_ENABLE, 1);
                    cmr_gpioWrite(GPIO_CHANNEL_3_ENABLE, 0);
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
                    cmr_gpioWrite(GPIO_CHANNEL_3_ENABLE, 0);
                    break;
            }

            vTaskDelayUntil(&lastWakeTime, pumpControl_period_ms);
        }
}

/**
 * @brief Task for controlling the brakelight.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void brakelight(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    volatile cmr_canVSMSensors_t *vsmSensors = canGetPayload(CANRX_VSM_SENSORS);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        if (vsmSensors->brakePressureRear_PSI > brakeLightThreshold_PSI) {
            cmr_gpioWrite(GPIO_BRKLT_ENABLE, 1);
        } else {
            cmr_gpioWrite(GPIO_BRKLT_ENABLE, 0);
        }

        vTaskDelayUntil(&lastWakeTime, brakelight_period_ms);
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
    adcInit();
    sensorsInit();

    cmr_taskInit(
        &statusLED_task,
        "statusLED",
        statusLED_priority,
        statusLED,
        NULL
    );

    /* CMR_PTC_ID is defined in can.h */
#ifndef CMR_PTC_ID
#error "No PTC ID defined!"
#elif (CMR_PTC_ID == 0) /* Pump Control Board */

    canInit();

    cmr_taskInit(
        &mcPowerControl_task,
        "mcPowerControl",
        mcPowerControl_priority,
        mcPowerControl,
        NULL
    );
    cmr_taskInit(
        &brakelight_task,
        "brakelight",
        brakelight_priority,
        brakelight,
        NULL
    );
    cmr_taskInit(
        &pumpControl_task,
        "pumpControl",
        pumpControl_priority,
        pumpControl,
        NULL
    );

#elif (CMR_PTC_ID == 1) /* Fan Control Board */

    canInit();

    cmr_taskInit(
        &fanControl_task,
        "fanControl",
        fanControl_priority,
        fanControl,
        NULL
    );

#else

    #pragma warning "CMR_PTC_ID is not a valid value!"

#endif

    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");
}

