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
static cmr_pwm_t fanPWM1;
static cmr_pwm_t fanPWM2;
static cmr_pwm_t fanPWM3;

/** @brief Cell temperature at which PTC begins commanding AFC max cooling. */
static const uint32_t maxCoolingEnableTemp_C = 55;
/** @brief Cell temperature at which PTC stops commanding AFC max cooling. */
static const uint32_t minCoolingDisableTemp_C = 52;

/** @brief Status LED task priority. */
static const uint32_t statusLED_priority = 2;
/** @brief Status LED task period. */
static const TickType_t statusLED_period_ms = 250;
/** @brief Status LED task. */
static cmr_task_t statusLED_task;

/** @brief Cooling control task priority. */
static const uint32_t coolingControl_priority = 4;
/** @brief Cooling control task period. */
static const TickType_t coolingControl_period_ms = 50;
/** @brief Cooling control task. */
static cmr_task_t coolingControl_task;

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
 * @brief Task for controlling the radiator fan and pump.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void coolingControl(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    cmr_gpioWrite(GPIO_CHANNEL_1_ENABLE, 1); // Turn the fan driver on

    // Get reference to VSM Heartbeat and accumulator min/max cell temperatures
    volatile cmr_canHeartbeat_t *vsmHeartbeat = canGetPayload(CANRX_HEARTBEAT_VSM);
    volatile cmr_canHVCPackMinMaxCellTemps_t *minMaxTemps = canGetPayload(CANRX_HVC_MINMAX_TEMPS);

    // Initialize PWM channels
    const cmr_pwmPinConfig_t pwmPinConfig1 = {
        .port = GPIOB,
        .pin = GPIO_PIN_5,
        .channel = TIM_CHANNEL_2,
        .presc = 24,
        .period_ticks = 160,  // 96 MHz / (24 * 40000) = 100 Hz
        .timer = TIM3
    };
    const cmr_pwmPinConfig_t pwmPinConfig2 = {
        .port = GPIOB,
        .pin = GPIO_PIN_7,
        .channel = TIM_CHANNEL_2,
        .presc = 24,
        .period_ticks = 160,  // 96 MHz / (24 * 40000) = 100 Hz
        .timer = TIM4
    };
    const cmr_pwmPinConfig_t pwmPinConfig3 = {
        .port = GPIOB,
        .pin = GPIO_PIN_8,
        .channel = TIM_CHANNEL_3,
        .presc = 24,
        .period_ticks = 160,  // 96 MHz / (24 * 40000) = 100 Hz
        .timer = TIM4
    };
    cmr_pwmInit(&fanPWM1, &pwmPinConfig1);
    cmr_pwmInit(&fanPWM2, &pwmPinConfig2);
    cmr_pwmInit(&fanPWM3, &pwmPinConfig3);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {

        switch (vsmHeartbeat->state) {
            case CMR_CAN_RTD:
                cmr_pwmSetDutyCycle(&fanPWM1, 100);  // Fan full on
                cmr_pwmSetDutyCycle(&fanPWM2, 100);  // Fan full on
                cmr_pwmSetDutyCycle(&fanPWM3, 100);  // Fan full on
                cmr_gpioWrite(GPIO_CHANNEL_1_ENABLE, 1);
                cmr_gpioWrite(GPIO_CHANNEL_2_ENABLE, 1);
                cmr_gpioWrite(GPIO_CHANNEL_3_ENABLE, 1);
                break;
            case CMR_CAN_HV_EN:
                cmr_pwmSetDutyCycle(&fanPWM1, 50);  // Fan full on
                cmr_pwmSetDutyCycle(&fanPWM2, 50);  // Fan full on
                cmr_pwmSetDutyCycle(&fanPWM3, 50);  // Fan full on
                cmr_gpioWrite(GPIO_CHANNEL_1_ENABLE, 1);
                cmr_gpioWrite(GPIO_CHANNEL_2_ENABLE, 1);
                cmr_gpioWrite(GPIO_CHANNEL_3_ENABLE, 1);
                break;
            default:
                cmr_pwmSetDutyCycle(&fanPWM1, 0);    // Fan off
                cmr_pwmSetDutyCycle(&fanPWM2, 0);    // Fan off
                cmr_pwmSetDutyCycle(&fanPWM3, 0);    // Fan off
                cmr_gpioWrite(GPIO_CHANNEL_1_ENABLE, 1);        // Pump off
                cmr_gpioWrite(GPIO_CHANNEL_2_ENABLE, 0);        // Pump off
                cmr_gpioWrite(GPIO_CHANNEL_3_ENABLE, 0);        // Pump off
                break;
        }

        vTaskDelayUntil(&lastWakeTime, coolingControl_period_ms);
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
    canInit();
    adcInit();
    //sensorsInit();

    cmr_taskInit(
        &statusLED_task,
        "statusLED",
        statusLED_priority,
        statusLED,
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
        &coolingControl_task,
        "coolingControl",
        coolingControl_priority,
        coolingControl,
        NULL
    );

    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");
}

